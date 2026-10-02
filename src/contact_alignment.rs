//! Car/ball tick alignment from the ball packets (offline).
//!
//! The ball packets say when a car touched the ball (`ball_evidence`) and what the ball did
//! afterwards (the velocity in the next packet). The car and the ball are placed on the timeline with
//! separately inferred packet lags, so their relative timing is only known to a tick or two, and a
//! tick is 8-25 UU of closing distance at the speeds of a hit: the simulated car may graze or miss
//! the ball, or hit it with the wrong impulse. For each contact interval the car packet before the
//! hit is placed -3..=+3 ticks off in a scratch arena, the hit is simulated to the second ball
//! packet, and the shift whose ball velocity is closest to that packet is kept when it beats the
//! unshifted one clearly. The kept shifts move the lags of those car packets; the converter then
//! runs again with them. The second ball packet is a future observation: this is a fit, labelled
//! as such, not a prediction.

use std::collections::HashMap;

use rocketsim::{Arena, ArenaConfig, BallState, CarControls, CarState, GameMode, Team};

use crate::conversion::{
    ConvertError, ConvertOptions, PacketLags, convert_observations_with, hitbox_config, infer_packet_lags,
    quaternion, step_tick_with_hit_impulse,
};
use crate::observations::{Body, ObservedReplay};

/// Shifts tried, in ticks the car packet is later (+) or earlier (-) than its lag says.
const SHIFTS: std::ops::RangeInclusive<i64> = -3..=3;
/// A shift is kept only when it lowers the ball velocity residual (UU/s) by at least this.
const MIN_IMPROVEMENT: f32 = 30.0;
/// The best shift must reproduce the ball's velocity this closely (UU/s) to be believed.
const MAX_RESIDUAL: f32 = 100.0;
/// The car packet the simulation starts from is at most this many ticks before the hit.
const MAX_LEAD_TICKS: i64 = 4;

/// Experiment switch (env `ALIGN_MAX_LEAD`): the default reach in ticks.
fn max_lead_ticks() -> i64 {
    static LEAD: std::sync::OnceLock<i64> = std::sync::OnceLock::new();
    *LEAD.get_or_init(|| {
        std::env::var("ALIGN_MAX_LEAD").ok().and_then(|v| v.parse().ok()).unwrap_or(MAX_LEAD_TICKS)
    })
}
/// Shifts within this (UU/s) of the best are equivalent; the smallest one wins.
const TIE: f32 = 10.0;

/// What the alignment did, for diagnostics.
#[derive(Debug, Default, Clone)]
pub struct AlignmentSummary {
    pub contacts: usize,
    pub fitted: usize,
    pub shifted: usize,
    pub shifts: HashMap<i64, usize>,
    /// Car chain runs moved as a whole.
    pub moved_runs: usize,
}

fn phys(body: &Body, frame: usize) -> Option<([f32; 3], [f32; 3], glam::Mat3A, [f32; 3])> {
    let pos = body.position.as_ref().filter(|v| v.frame == frame)?.value;
    let vel = body.linear_velocity.as_ref().filter(|v| v.frame == frame)?.value;
    let rot = body.rotation_xyzw.as_ref().filter(|v| v.frame == frame)?.value;
    let ang = body
        .angular_velocity_replay_units
        .as_ref()
        .filter(|v| v.frame == frame)
        .map_or([0.0; 3], |v| v.value);
    let q = quaternion(rot)?;
    Some((pos, vel, glam::Mat3A::from_quat(q), ang))
}

/// The lags with the car packets before contacts moved by their fitted shifts.
pub fn aligned_lags(
    observations: &ObservedReplay,
    options: &ConvertOptions,
) -> Result<(PacketLags, AlignmentSummary), ConvertError> {
    let mut lags = infer_packet_lags(observations, options);
    let mut summary = AlignmentSummary::default();
    // A lag-free replay (every lag zero) has nothing to align.
    if lags.car_actor.values().all(|l| *l == 0.0) && std::env::var_os("ALIGN_FORCE").is_none() {
        return Ok((lags, summary));
    }
    // First pass: the normal conversion, for its contacts and exported poses.
    // Only the contacts, the cars that made them and rough car poses are needed from it: the lags
    // are reused and the expensive fits (the boundary-value solves, the timing fits, the flip fits)
    // are left out, which is most of a conversion's cost.
    let mut first = options.clone();
    first.align_contacts = false;
    first.external_packet_lags = Some(std::sync::Arc::new(lags.clone()));
    first.air_bvp = false;
    first.infer_dodge_start = false;
    first.infer_flip_cancel = false;
    first.fit_ground_control_timing = false;
    first.fit_jump_timing = false;
    let mut frames = Vec::new();
    let summary_pass = convert_observations_with(observations, &first, |converted, _, _| {
        frames.push(converted.clone());
        Ok(())
    })?;
    let frame_data = &observations.frames;
    let first_time = f64::from(frame_data.first().map_or(0.0, |f| f.time));
    let timeline = |f: usize| ((f64::from(frame_data[f].time) - first_time) * 120.0).round() as i64;
    let mut ball_arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    arena.add_car(Team::Blue, rocketsim::CarBodyConfig::OCTANE);
    let mut current_config = "octane".to_string();
    let mut votes: std::collections::BTreeMap<usize, Vec<i64>> = std::collections::BTreeMap::new();
    for converted in &frames {
        for contact in &converted.ball_contacts {
            let Some(slot) = contact.car_slot else { continue };
            let fb = converted.replay_frame;
            let fa = contact.frame_a;
            summary.contacts += 1;
            let Some(slot_info) = summary_pass.car_slots.iter().find(|s| s.slot == slot) else {
                continue;
            };
            let Some(car_actor) = frame_data[fb]
                .cars
                .iter()
                .find(|c| c.player_key.as_deref() == Some(slot_info.player_key.as_str()))
            else {
                continue;
            };
            // The car packet closest before the hit: its physics are exact (0.01 UU) at its own tick,
            // so only the tick is uncertain. The ball starts from its packet before the contact.
            let hit_tick = contact.tick as i64;
            let mut packet: Option<(usize, i64)> = None;
            for g in (fa.saturating_sub(3)..=fb).rev() {
                let Some(car) = frame_data[g].cars.iter().find(|c| {
                    c.actor_id == car_actor.actor_id && c.actor_created_frame == car_actor.actor_created_frame
                }) else {
                    continue;
                };
                if phys(&car.body, g).is_none() {
                    continue;
                }
                let Some(&lag) = lags.car_actor.get(&(car.actor_id, car.actor_created_frame, g)) else {
                    continue;
                };
                let tick = timeline(g) - lag.round() as i64;
                if tick <= hit_tick + 1 {
                    packet = Some((g, tick));
                    break;
                }
            }
            let Some((g, car_tick)) = packet else { continue };
            if hit_tick - car_tick > max_lead_ticks() {
                continue;
            }
            let car_packet = frame_data[g]
                .cars
                .iter()
                .find(|c| c.actor_id == car_actor.actor_id && c.actor_created_frame == car_actor.actor_created_frame)
                .and_then(|c| phys(&c.body, g))
                .expect("checked above");
            let (Some(ball_a), Some(ball_b), Some(exported)) = (
                frame_data[fa].ball.as_ref().and_then(|b| phys(b, fa)),
                frame_data[fb].ball.as_ref().and_then(|b| phys(b, fb)),
                frames.get(g).and_then(|f| f.state.cars.get(slot)),
            ) else {
                continue;
            };
            let tick_a = contact.tick_from as i64;
            let tick_b = contact.tick_to as i64;
            if tick_b <= tick_a || tick_b - tick_a > 40 {
                continue;
            }
            if slot_info.hitbox != current_config {
                arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
                arena.add_car(Team::Blue, hitbox_config(&slot_info.hitbox));
                current_config = slot_info.hitbox.clone();
            }
            // The state as in the earlier contact study: the packet's physics on a default car, grounded
            // when low; the observed throttle, steer, handbrake and boost as controls.
            let _ = exported;
            let mut car_state = CarState::default();
            car_state.phys.pos = glam::Vec3A::from(car_packet.0);
            car_state.phys.vel = glam::Vec3A::from(car_packet.1);
            car_state.phys.rot_mat = car_packet.2;
            car_state.phys.ang_vel = glam::Vec3A::from(car_packet.3) * 0.01;
            let grounded = car_packet.0[2] < 30.0;
            car_state.is_on_ground = grounded;
            car_state.wheels_with_contact = [grounded.then(rocketsim::RaycastHitInfo::default); 4];
            let observed = frame_data[g]
                .cars
                .iter()
                .find(|c| c.actor_id == car_actor.actor_id && c.actor_created_frame == car_actor.actor_created_frame)
                .expect("checked above");
            let controls = CarControls {
                throttle: observed.inputs.throttle.as_ref().map_or(0.0, |v| v.value),
                steer: observed.inputs.steer.as_ref().map_or(0.0, |v| v.value),
                handbrake: observed.inputs.handbrake.as_ref().is_some_and(|v| v.value),
                boost: observed.inputs.boost_active_raw.as_ref().is_some_and(|v| v.value % 2 == 1),
                ..CarControls::default()
            };
            let ball_state = |p: &([f32; 3], [f32; 3], glam::Mat3A, [f32; 3])| {
                let mut b = BallState::default();
                b.phys.pos = glam::Vec3A::from(p.0);
                b.phys.vel = glam::Vec3A::from(p.1);
                b.phys.rot_mat = p.2;
                b.phys.ang_vel = glam::Vec3A::from(p.3) * 0.01;
                b
            };
            let parked = {
                let mut b = BallState::default();
                b.phys.pos = glam::Vec3A::new(0.0, 0.0, 1800.0);
                if (car_state.phys.pos - b.phys.pos).length() < 600.0 {
                    // A car on the ceiling would touch the parked ball.
                    b.phys.pos = glam::Vec3A::new(3000.0, 4000.0, 300.0);
                }
                b
            };
            let mut residuals: Vec<(i64, f32)> = Vec::new();
            for shift in SHIFTS {
                let tick_c = car_tick + shift;
                if tick_c >= tick_b {
                    continue;
                }
                // The earlier object runs alone until the later one starts. (`set_car_state` replaces the
                // whole state, controls included: the controls are set after it.)
                let t1 = tick_a.max(tick_c);
                if tick_a <= tick_c {
                    ball_arena.set_ball_state(ball_state(&ball_a));
                    for _ in tick_a..tick_c {
                        ball_arena.step_tick();
                    }
                    arena.set_ball_state(*ball_arena.get_ball_state());
                    arena.set_car_state(0, car_state);
                    arena.set_car_controls(0, controls);
                } else {
                    arena.set_ball_state(parked);
                    arena.set_car_state(0, car_state);
                    arena.set_car_controls(0, controls);
                    for _ in tick_c..tick_a {
                        arena.step_tick();
                    }
                    arena.set_ball_state(ball_state(&ball_a));
                }
                if shift == 0 && std::env::var_os("ALIGN_DEBUG").is_some() {
                    let local = arena.get_car_state(0).phys.rot_mat.transpose()
                        * (arena.get_ball_state().phys.pos - arena.get_car_state(0).phys.pos)
                        - hitbox_config(&slot_info.hitbox).hitbox_pos_offset;
                    let q = local.abs() - hitbox_config(&slot_info.hitbox).hitbox_size * 0.5;
                    let gap = q.max(glam::Vec3A::ZERO).length() + q.max_element().min(0.0) - 91.25;
                    eprintln!("START gap at the car packet {gap:.1} UU; car speed {:.0}, ball {:.0}; ticks from start to b {}", arena.get_car_state(0).phys.vel.length(), arena.get_ball_state().phys.vel.length(), tick_b - t1);
                }
                let mut hit = false;
                for _ in t1..tick_b {
                    let events = step_tick_with_hit_impulse(&mut arena, options.apply_hit_extra_impulse);
                    hit |= events.iter().any(|e| matches!(e, rocketsim::ArenaEvent::CarHitBall(_)));
                }
                if shift == 0 && std::env::var_os("ALIGN_DEBUG").is_some() {
                    eprintln!("   sim hit {hit}");
                }
                let v = arena.get_ball_state().phys.vel;
                residuals.push((shift, (v - glam::Vec3A::from(ball_b.1)).length()));
            }
            let lead = hit_tick - car_tick;
            let Some(&(_, r0)) = residuals.iter().find(|(s, _)| *s == 0) else { continue };
            let best = residuals.iter().map(|r| r.1).fold(f32::INFINITY, f32::min);
            summary.fitted += 1;
            if std::env::var_os("ALIGN_DEBUG").is_some() {
                eprintln!(
                    "CONTACT car_key={} lead={} residuals={:?}",
                    slot_info.player_key,
                    lead,
                    residuals.iter().map(|r| (r.0, r.1.round())).collect::<Vec<_>>()
                );
            }
            // Only a shift that reproduces the ball's velocity is believed; a fit that needs none
            // votes for no shift.
            if best > MAX_RESIDUAL {
                continue;
            }
            let run = lags
                .car_run_of
                .get(&(car_actor.actor_id, car_actor.actor_created_frame, g))
                .copied();
            if r0 - best < MIN_IMPROVEMENT {
                if let Some(run) = run {
                    votes.entry(run).or_default().push(0);
                }
                continue;
            }
            let chosen = residuals
                .iter()
                .filter(|(_, r)| *r <= best + TIE)
                // The smallest shift; between -k and +k the better residual (not the first in scan order,
                // which would always lean negative).
                .min_by(|a, b| a.0.abs().cmp(&b.0.abs()).then(a.1.total_cmp(&b.1)))
                .map(|(s, _)| *s)
                .unwrap_or(0);
            if std::env::var_os("ALIGN_DEBUG").is_some() {
                let lag_ball = lags.ball[fa];
                eprintln!(
                    "CHOSEN shift={chosen} car_key={} car_frame={g} car_tick={car_tick} car_pos={:.2},{:.2},{:.2} ball_frame={fa} ball_tick={tick_a} ball_pos={:.2},{:.2},{:.2} ball_lag={:?}",
                    slot_info.player_key, car_packet.0[0], car_packet.0[1], car_packet.0[2],
                    ball_a.0[0], ball_a.0[1], ball_a.0[2], lag_ball
                );
            }
            if let Some(run) = run {
                votes.entry(run).or_default().push(chosen);
            }
            if chosen != 0 {
                summary.shifted += 1;
                *summary.shifts.entry(chosen).or_default() += 1;
            }
        }
    }
    // A run's packets share one level: it moves by the median of its contacts' shifts when they agree
    // (within one tick), as far as its feasible range allows.
    let mut moved_runs = 0usize;
    let mut moved_frames: std::collections::HashSet<usize> = std::collections::HashSet::new();
    for (run_index, mut v) in votes {
        v.sort_unstable();
        let median = v[v.len() / 2];
        if median == 0 || v.iter().any(|x| (x - median).abs() > 1) {
            continue;
        }
        let run = lags.car_runs[run_index].clone();
        let start = (run.start + median).clamp(run.lo, run.hi);
        if start == run.start {
            continue;
        }
        moved_runs += 1;
        for &(frame, k) in &run.entries {
            // A packet shared with the neighbouring run belongs to the run `car_run_of` names.
            if lags.car_run_of.get(&(run.actor, run.created, frame)) != Some(&run_index) {
                continue;
            }
            let lag = (timeline(frame) - (start + k)).max(0) as f32;
            lags.car_actor.insert((run.actor, run.created, frame), lag);
        }
        lags.car_runs[run_index].start = start;
        moved_frames.extend(run.entries.iter().map(|e| e.0));
    }
    // The per-frame median car lag (the fallback for a car without a chain of its own) follows the
    // moved runs.
    if !moved_frames.is_empty() {
        let mut per_frame: HashMap<usize, Vec<f32>> = HashMap::new();
        for (&(_, _, frame), &lag) in &lags.car_actor {
            if moved_frames.contains(&frame) {
                per_frame.entry(frame).or_default().push(lag);
            }
        }
        for (frame, mut values) in per_frame {
            values.sort_by(|a, b| a.total_cmp(b));
            lags.cars[frame] = Some(values[values.len() / 2]);
        }
    }
    summary.moved_runs = moved_runs;
    if std::env::var_os("ALIGN_DEBUG").is_some() {
        eprintln!("contact alignment: {summary:?}");
    }
    Ok((lags, summary))
}
