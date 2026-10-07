//! Contact alignment (docs/glossary.md): the car updates before each ball contact moved by a tick or two so
//! that a simulated hit reproduces the ball's next update. Offline: it uses the ball's update after the hit.
//!
//! The car and the ball are placed on the timeline by separately inferred update ticks, so their relative
//! timing is known only to a tick or two, and a tick is 8-25 UU of closing distance at the speeds of a hit:
//! the simulated car may graze or miss the ball, or hit it with the wrong impulse. A first simulation without
//! the expensive fits finds the ball contacts (`annotate`) and the cars' states. For each contact the car's
//! update before the hit is placed -3..=+3 ticks off in a scratch arena, the hit is simulated to the ball's
//! next update, and the shift whose ball velocity is closest to that update is kept when it beats the
//! unshifted one clearly. A car's run of chained updates moves as a whole, by the median of its contacts'
//! shifts when they agree.

use std::collections::{BTreeMap, BTreeSet};

use glam::{Mat3A, Vec3A};
use replicar_format::FrameIndex;
use rocketsim::{Arena, ArenaConfig, BallState, CarControls, CarState, GameMode};

use crate::annotate::{Annotator, ball_intervals};
use crate::decode::{NetworkBody, NetworkReplay};
use crate::hitbox::Hitbox;
use crate::infer::{FittedInference, InferenceOptions};
use crate::simulate::{SimulationOptions, simulate};
use crate::update_ticks::{UpdateTicks, Withheld, quaternion};
use crate::{Error, Meshes};

/// Shifts tried, in ticks the car update is later (+) or earlier (-) than its update tick says.
const SHIFTS: std::ops::RangeInclusive<i64> = -3..=3;
/// A shift is kept only when it lowers the ball velocity residual (UU/s) by at least this.
const MIN_IMPROVEMENT: f32 = 30.0;
/// The best shift must reproduce the ball's velocity this closely (UU/s) to be believed.
const MAX_RESIDUAL: f32 = 100.0;
/// The car update the simulation starts from is at most this many ticks before the hit.
const MAX_LEAD_TICKS: i64 = 4;
/// Shifts within this (UU/s) of the best are equivalent; the smallest one wins.
const TIE: f32 = 10.0;

/// What the alignment did.
#[derive(Debug, Default, Clone, PartialEq, Eq, serde::Serialize)]
pub struct AlignmentSummary {
    /// Ball contacts attributed to a player, and those whose hit could be simulated.
    pub contacts: usize,
    pub fitted: usize,
    /// Fitted contacts whose best shift was not zero, by shift.
    pub shifted: usize,
    pub shifts: BTreeMap<i64, usize>,
    /// Car runs moved as a whole.
    pub moved_runs: usize,
}

/// A body's exact physics at an update.
#[derive(Debug, Clone, Copy)]
struct Update {
    pos: Vec3A,
    vel: Vec3A,
    rot: Mat3A,
    /// rad/s.
    ang: Vec3A,
}

impl Update {
    /// The update of `body` at `frame`, with position, velocity and rotation; a missing angular velocity is
    /// zero.
    fn at(body: &NetworkBody, frame: usize) -> Option<Self> {
        let at = |f: FrameIndex| f.get() == frame;
        let pos = body.position.as_ref().filter(|v| at(v.frame))?.value;
        let vel = body.linear_velocity.as_ref().filter(|v| at(v.frame))?.value;
        let rot = body.rotation.as_ref().filter(|v| at(v.frame))?.value;
        let ang = body
            .angular_velocity_raw
            .as_ref()
            .filter(|v| at(v.frame))
            .map_or([0.0; 3], |v| v.value);
        Some(Self {
            pos: Vec3A::from(pos),
            vel: Vec3A::from(vel),
            rot: Mat3A::from_quat(quaternion(rot)?),
            ang: Vec3A::from(ang) * 0.01,
        })
    }

    fn ball(self) -> BallState {
        let mut ball = BallState::default();
        ball.phys.pos = self.pos;
        ball.phys.vel = self.vel;
        ball.phys.rot_mat = self.rot;
        ball.phys.ang_vel = self.ang;
        ball
    }
}

/// One ball contact of the first simulation that the annotation gave to a player.
struct Contact {
    frame_a: usize,
    frame_b: usize,
    hit_tick: i64,
    tick_a: i64,
    tick_b: i64,
    player: usize,
}

/// `ticks` with the car updates before ball contacts moved by their fitted shifts. A lag-free replay (every
/// car update at its frame's tick) is returned unchanged. RocketSim's meshes must be loaded.
pub fn align_contacts(
    network: &NetworkReplay,
    ticks: &UpdateTicks,
    meshes: &Meshes,
    inference: InferenceOptions,
    options: &SimulationOptions,
) -> Result<(UpdateTicks, AlignmentSummary), Error> {
    let mut aligned = ticks.clone();
    let mut summary = AlignmentSummary::default();
    if ticks.cars.values().all(|&t| t == 0) {
        return Ok((aligned, summary));
    }
    // The first simulation: only its contacts, the players that made them and rough car states are needed,
    // so the expensive fits (the air schedules and the input fits) are left out.
    let withheld = Withheld(options.withheld.as_deref());
    let mut first = FittedInference::new(
        network,
        Some(ticks),
        InferenceOptions {
            air_schedules: false,
            input_fits: false,
            ..inference
        },
        withheld,
    );
    let mut annotator = Annotator::new(ball_intervals(&network.frames, ticks, withheld));
    let mut contacts = Vec::new();
    // Each frame's sim tick and car states.
    let mut states: Vec<(u64, Vec<CarState>)> = Vec::with_capacity(network.frames.len());
    let simulation = simulate(
        network,
        Some(ticks),
        &mut first,
        meshes,
        options.clone(),
        |frame| {
            for contact in annotator.annotate(&frame).ball_contacts {
                if let Some(player) = contact.player {
                    contacts.push(Contact {
                        frame_a: contact.frame_a,
                        frame_b: frame.index.get(),
                        hit_tick: contact.replay_tick as i64,
                        tick_a: contact.tick_from as i64,
                        tick_b: contact.tick_to as i64,
                        player: usize::from(player.0),
                    });
                }
            }
            states.push((
                frame.state.tick_count,
                frame.state.cars.iter().map(|(_, car)| *car).collect(),
            ));
        },
    )?;

    let frames = &network.frames;
    let first_time = f64::from(frames.first().map_or(0.0, |f| f.time));
    let timeline = |f: usize| ((f64::from(frames[f].time) - first_time) * 120.0).round() as i64;
    let mut ball_arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    // One scratch arena, its car rebuilt when the hitbox changes; its tick count carries over.
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    arena.add_car(rocketsim::Team::Blue, Hitbox::Octane.config());
    let mut arena_hitbox = Hitbox::Octane;
    let mut votes: BTreeMap<usize, Vec<i64>> = BTreeMap::new();
    for contact in &contacts {
        let Contact {
            frame_a: fa,
            frame_b: fb,
            hit_tick,
            tick_a,
            tick_b,
            player,
        } = *contact;
        summary.contacts += 1;
        let Some(sim_player) = simulation.players.get(player) else {
            continue;
        };
        // The player's current car (an older car of the same player still in the network is not the one
        // simulated).
        let Some(life) = frames[fb]
            .current_cars()
            .into_iter()
            .find(|c| c.player.as_ref() == Some(&sim_player.key))
            .map(|c| c.life)
        else {
            continue;
        };
        // The car's last update before the hit: its physics are exact at its own tick, so only the tick is
        // uncertain. The ball starts from its update before the contact.
        let car_at = |g: usize| frames[g].cars.iter().find(|c| c.life == life);
        let mut found = None;
        for g in (fa.saturating_sub(3)..=fb).rev() {
            let Some(car) = car_at(g) else {
                continue;
            };
            if Update::at(&car.body, g).is_none() {
                continue;
            }
            let Some(&before) = ticks.cars.get(&(life, FrameIndex(g as u32))) else {
                continue;
            };
            let tick = timeline(g) - i64::from(before);
            if tick <= hit_tick + 1 {
                found = Some((g, tick));
                break;
            }
        }
        let Some((g, car_tick)) = found else {
            continue;
        };
        if hit_tick - car_tick > MAX_LEAD_TICKS {
            continue;
        }
        let car = car_at(g).expect("found above");
        let car_update = Update::at(&car.body, g).expect("checked above");
        let ball_update = |f: usize| frames[f].ball.as_ref().and_then(|b| Update::at(b, f));
        let (Some(ball_a), Some(ball_b), Some((source_tick, cars))) =
            (ball_update(fa), ball_update(fb), states.get(g))
        else {
            continue;
        };
        let Some(exported) = cars.get(player) else {
            continue;
        };
        if tick_b <= tick_a || tick_b - tick_a > 40 {
            continue;
        }
        if sim_player.hitbox != arena_hitbox {
            arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
            arena.add_car(rocketsim::Team::Blue, sim_player.hitbox.config());
            arena_hitbox = sim_player.hitbox;
        }
        // The first simulation's car (wheel contacts, ground state, boost, jump and flip flags, the last hit
        // tick rebased to this arena) with the update's exact physics; the network throttle, steer, handbrake
        // and boost as controls. A demolished car makes no contact.
        if exported.is_demoed {
            continue;
        }
        let mut car_state = *exported;
        car_state.last_extra_hit_tick = car_state
            .last_extra_hit_tick
            .and_then(|t| arena.tick_count().checked_sub(source_tick.checked_sub(t)?));
        car_state.phys.pos = car_update.pos;
        car_state.phys.vel = car_update.vel;
        car_state.phys.rot_mat = car_update.rot;
        car_state.phys.ang_vel = car_update.ang;
        let inputs = &car.inputs;
        let controls = CarControls {
            throttle: inputs.throttle.as_ref().map_or(0.0, |v| v.value),
            steer: inputs.steer.as_ref().map_or(0.0, |v| v.value),
            handbrake: inputs.handbrake.as_ref().is_some_and(|v| v.value),
            boost: inputs
                .boost_active_raw
                .as_ref()
                .is_some_and(|v| v.value % 2 == 1),
            ..CarControls::default()
        };
        // Where the ball waits while the car runs alone: out of reach (a car on the ceiling would touch it
        // at the centre).
        let mut parked = BallState::default();
        parked.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
        if (car_state.phys.pos - parked.phys.pos).length() < 600.0 {
            parked.phys.pos = Vec3A::new(3000.0, 4000.0, 300.0);
        }
        let mut residuals: Vec<(i64, f32)> = Vec::new();
        for shift in SHIFTS {
            let tick_c = car_tick + shift;
            if tick_c >= tick_b {
                continue;
            }
            // The earlier body runs alone until the later one starts. (`set_car_state` replaces the whole
            // state, controls included: the controls are set after it.)
            if tick_a <= tick_c {
                ball_arena.set_ball_state(ball_a.ball());
                for _ in tick_a..tick_c {
                    ball_arena.step_tick();
                }
                arena.set_ball_state(*ball_arena.get_ball_state());
                arena.set_car_state(0, car_state);
                arena.refresh_car_sticky_gate(0);
                arena.set_car_controls(0, controls);
            } else {
                arena.set_ball_state(parked);
                arena.set_car_state(0, car_state);
                arena.refresh_car_sticky_gate(0);
                arena.set_car_controls(0, controls);
                for _ in tick_c..tick_a {
                    arena.step_tick();
                }
                arena.set_ball_state(ball_a.ball());
            }
            for _ in tick_a.max(tick_c)..tick_b {
                arena.step_tick();
            }
            let velocity = arena.get_ball_state().phys.vel;
            residuals.push((shift, (velocity - ball_b.vel).length()));
        }
        let Some(&(_, unshifted)) = residuals.iter().find(|(s, _)| *s == 0) else {
            continue;
        };
        let best = residuals.iter().map(|r| r.1).fold(f32::INFINITY, f32::min);
        summary.fitted += 1;
        // Only a shift that reproduces the ball's velocity is believed; a fit that needs none votes for no
        // shift.
        if best > MAX_RESIDUAL {
            continue;
        }
        let run = ticks.car_run_of.get(&(life, FrameIndex(g as u32))).copied();
        if unshifted - best < MIN_IMPROVEMENT {
            if let Some(run) = run {
                votes.entry(run).or_default().push(0);
            }
            continue;
        }
        let chosen = residuals
            .iter()
            .filter(|(_, r)| *r <= best + TIE)
            // The smallest shift; between -k and +k the better residual (not the first in scan order, which
            // would always lean negative).
            .min_by(|a, b| a.0.abs().cmp(&b.0.abs()).then(a.1.total_cmp(&b.1)))
            .map_or(0, |(s, _)| *s);
        if let Some(run) = run {
            votes.entry(run).or_default().push(chosen);
        }
        if chosen != 0 {
            summary.shifted += 1;
            *summary.shifts.entry(chosen).or_default() += 1;
        }
    }

    // A run's updates share one start: it moves by the median of its contacts' shifts when they agree within
    // a tick, as far as its feasible range allows.
    let mut moved_frames: BTreeSet<FrameIndex> = BTreeSet::new();
    for (index, mut shifts) in votes {
        shifts.sort_unstable();
        let median = shifts[shifts.len() / 2];
        if median == 0 || shifts.iter().any(|s| (s - median).abs() > 1) {
            continue;
        }
        let run = &aligned.car_runs[index];
        let start = (run.start + median).clamp(run.lo, run.hi);
        if start == run.start {
            continue;
        }
        summary.moved_runs += 1;
        let (life, entries) = (run.life, run.entries.clone());
        for &(frame, k) in &entries {
            // An update shared with the neighbouring run belongs to the run `car_run_of` names.
            if aligned.car_run_of.get(&(life, frame)) != Some(&index) {
                continue;
            }
            let before = (timeline(frame.get()) - (start + k)).max(0) as u32;
            aligned.cars.insert((life, frame), before);
        }
        aligned.car_runs[index].start = start;
        moved_frames.extend(entries.iter().map(|e| e.0));
    }
    // The frame's median over its car updates (for a car without a tick of its own) follows the moved runs.
    let mut per_frame: BTreeMap<FrameIndex, Vec<u32>> = BTreeMap::new();
    for (&(_, frame), &before) in &aligned.cars {
        if moved_frames.contains(&frame) {
            per_frame.entry(frame).or_default().push(before);
        }
    }
    for (frame, mut values) in per_frame {
        values.sort_unstable();
        aligned.car_median[frame.get()] = Some(values[values.len() / 2]);
    }
    Ok((aligned, summary))
}
