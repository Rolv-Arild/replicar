//! When does a jump physically start relative to the jump counter turning odd? Offline diagnostic on
//! train/validation, with a held-out packet.
//!
//! Takes consecutive fresh car packets (a, b, c) with exact chain-lag ticks: the car is flat on the
//! ground at a with even jump, double-jump, dodge and flip counters, none of the latter three
//! changes, the jump counter turns odd at a frame after a and no later than c, c is at most 30
//! ticks after a, and there is no ball or other car within 400 UU at the three packets. RocketSim
//! is stepped from a (ball parked out of reach) with the observed throttle, steer, handbrake and
//! boost (midpoint-rule switch ticks, as in `diagnose_ground_driving`), and the jump input equal to
//! the parity of the jump counter with every switch shifted by d ticks (d = -8..=16, relative to the
//! midpoint-rule tick of the frame that first shows the change; d = +4 is about the frame time).
//! The cost at a packet is position error plus 0.1 x velocity error.
//!
//! The shift d is fitted on c only; b is held out. Compared at b: the midpoint rule (d = 0), the
//! frame time (d = +4), the shift fitted on this event (offline, uses c), one constant for the
//! whole corpus (fitted on c over all events), and the median shift of the same player's other
//! events in the replay (the test of a stable per-player latency; it uses no packet of this event).

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Quat, Vec3A};
use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes};
use replay_to_rocketsim::observations::{Body, Car};
use rocketsim::{
    Arena, ArenaConfig, BallState, CarBodyConfig, CarControls, CarState, GameMode, Team,
};

const SHIFTS: std::ops::RangeInclusive<i64> = -8..=16;

fn replay_paths(path: &Path) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    if path.is_file() {
        return Ok(vec![path.to_owned()]);
    }
    let mut result = Vec::new();
    for size in ["1v1", "2v2", "3v3"] {
        for entry in fs::read_dir(path.join(size))? {
            let path = entry?.path();
            if path.extension().is_some_and(|ext| ext == "replay") {
                result.push(path);
            }
        }
    }
    result.sort();
    Ok(result)
}

fn quantile(values: &mut [f32], q: f64) -> f32 {
    if values.is_empty() {
        return f32::NAN;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

type Phys = (Vec3A, Vec3A, Mat3A, Vec3A);

fn physics(body: &Body, frame: usize) -> Option<Phys> {
    let pos = body.position.as_ref().filter(|v| v.frame == frame)?.value;
    let vel = body
        .linear_velocity
        .as_ref()
        .filter(|v| v.frame == frame)?
        .value;
    let rot = body
        .rotation_xyzw
        .as_ref()
        .filter(|v| v.frame == frame)?
        .value;
    let ang = body
        .angular_velocity_replay_units
        .as_ref()
        .filter(|v| v.frame == frame)?
        .value;
    let quat = Quat::from_xyzw(rot[0], rot[1], rot[2], rot[3]);
    if !quat.is_finite() || quat.length_squared() < 0.5 {
        return None;
    }
    Some((
        Vec3A::from_array(pos),
        Vec3A::from_array(vel),
        Mat3A::from_quat(quat.normalize()),
        Vec3A::from_array(ang) * 0.01,
    ))
}

fn hitbox(name: &str) -> CarBodyConfig {
    match name {
        "breakout" => CarBodyConfig::BREAKOUT,
        "dominus" => CarBodyConfig::DOMINUS,
        "hybrid" => CarBodyConfig::HYBRID,
        "merc" => CarBodyConfig::MERC,
        "plank" => CarBodyConfig::PLANK,
        "psyclops" => CarBodyConfig::PSYCLOPS,
        _ => CarBodyConfig::OCTANE,
    }
}

#[derive(Clone, Copy)]
struct Inputs {
    throttle: f32,
    steer: f32,
    handbrake: bool,
    boost: bool,
    jump: bool,
}

fn inputs_of(car: &Car) -> Inputs {
    Inputs {
        throttle: car.inputs.throttle.as_ref().map_or(0.0, |v| v.value),
        steer: car.inputs.steer.as_ref().map_or(0.0, |v| v.value),
        handbrake: car.inputs.handbrake.as_ref().is_some_and(|v| v.value),
        boost: car
            .inputs
            .boost_active_raw
            .as_ref()
            .is_some_and(|v| v.value % 2 == 1),
        jump: car
            .inputs
            .jump_active_raw
            .as_ref()
            .is_some_and(|v| v.value % 2 == 1),
    }
}

struct Packet {
    frame: usize,
    tick: i64,
    phys: Phys,
    inputs: Inputs,
    counters: [Option<u8>; 4],
    hitbox: String,
}

/// (nominal frame tick, midpoint-rule start tick, inputs) for every active frame of a car.
type Timeline = Vec<(i64, i64, Inputs)>;

fn parked_ball() -> BallState {
    let mut ball = BallState::default();
    ball.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
    ball
}

struct Event {
    player: (usize, String),
    /// Cost at b and c, and vertical velocity error at b, for each shift.
    cost_b: Vec<f32>,
    cost_c: Vec<f32>,
    vz_b: Vec<f32>,
    /// True when the counter turns odd after b's tick (the jump starts after the interior packet).
    jump_after_b: bool,
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: diagnose_jump_latency <train or validation dir or replay>")?,
    );
    if path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let options = ConvertOptions::default();
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut arenas: BTreeMap<String, Arena> = BTreeMap::new();
    let mut events: Vec<Event> = Vec::new();
    let zero = (-*SHIFTS.start()) as usize;

    for (replay_index, replay_path) in replay_paths(&path)?.into_iter().enumerate() {
        let output = convert_bytes(&fs::read(&replay_path)?, &options)?;
        let frames = &output.observations.frames;
        let first_time = frames[0].time;
        let tick_of = |f: usize| -> i64 {
            ((f64::from(frames[f].time) - f64::from(first_time)) * 120.0).round() as i64
        };
        let lag_of = |f: usize, actor: i32| {
            output.frames[f]
                .packet_lags
                .iter()
                .find(|l| l.actor_id == Some(actor) && l.source == "chain")
                .map(|l| l.ticks as i64)
        };
        let active = |f: usize| {
            frames[f]
                .game_state
                .as_ref()
                .is_some_and(|s| s.value == "Active")
        };
        let mut timelines: BTreeMap<(i32, usize), Timeline> = BTreeMap::new();
        let mut lifetimes: BTreeMap<(i32, usize), (String, Vec<Packet>)> = BTreeMap::new();
        for f in 0..frames.len() {
            if !active(f) {
                continue;
            }
            for car in &frames[f].cars {
                let key = (car.actor_id, car.actor_created_frame);
                let line = timelines.entry(key).or_default();
                let spacing = line.last().map_or(4, |(t, _, _)| tick_of(f) - t);
                line.push((tick_of(f), tick_of(f) - 2 - spacing / 2, inputs_of(car)));
                let (Some(phys), Some(lag)) = (physics(&car.body, f), lag_of(f, car.actor_id))
                else {
                    continue;
                };
                let Some(player) = car.player_key.as_ref() else {
                    continue;
                };
                let Some(slot) = output.car_slots.iter().find(|s| &s.player_key == player) else {
                    continue;
                };
                let raw = |x: &Option<replay_to_rocketsim::observations::Value<u8>>| {
                    x.as_ref().map(|v| v.value)
                };
                lifetimes
                    .entry(key)
                    .or_insert_with(|| (player.clone(), Vec::new()))
                    .1
                    .push(Packet {
                        frame: f,
                        tick: tick_of(f) - lag,
                        phys,
                        inputs: inputs_of(car),
                        counters: [
                            raw(&car.inputs.jump_active_raw),
                            raw(&car.inputs.double_jump_active_raw),
                            raw(&car.inputs.dodge_active_raw),
                            raw(&car.inputs.flip_car_active_raw),
                        ],
                        hitbox: slot.hitbox.clone(),
                    });
            }
        }
        for (key, (player, packets)) in &lifetimes {
            let timeline = &timelines[key];
            let Some(first) = packets.first() else {
                continue;
            };
            let arena = arenas.entry(first.hitbox.clone()).or_insert_with(|| {
                let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
                arena.add_car(Team::Blue, hitbox(&first.hitbox));
                arena
            });
            for triple in packets.windows(3) {
                let (a, b, c) = (&triple[0], &triple[1], &triple[2]);
                let flat = a.phys.0.z < 25.0 && a.phys.2.z_axis.z > 0.97;
                let counters_ok = a.counters[0].is_some_and(|j| j % 2 == 0)
                    && a.counters[1..] == b.counters[1..]
                    && a.counters[1..] == c.counters[1..]
                    && a.counters[1..].iter().flatten().all(|x| x % 2 == 0);
                if !flat
                    || !counters_ok
                    || b.tick <= a.tick
                    || c.tick <= b.tick
                    || c.tick - a.tick > 30
                    || (a.frame..=c.frame).any(|f| !active(f))
                {
                    continue;
                }
                // The counter turns odd at a frame after a and no later than c.
                let jump_entry = timeline
                    .iter()
                    .find(|(t, _, i)| i.jump && *t > a.tick - 4 && *t <= c.tick + 4);
                let Some(&(jump_frame_tick, jump_start, _)) = jump_entry else {
                    continue;
                };
                if a.inputs.jump || jump_frame_tick <= a.tick - 4 {
                    continue;
                }
                let clear = |p: &Packet| {
                    frames[p.frame].ball.as_ref().is_none_or(|ball| {
                        ball.position.as_ref().is_none_or(|bp| {
                            (Vec3A::from_array(bp.value) - p.phys.0).length() > 400.0
                        })
                    }) && frames[p.frame].cars.iter().all(|o| {
                        o.body.position.as_ref().is_none_or(|op| {
                            let d = (Vec3A::from_array(op.value) - p.phys.0).length();
                            d < 1.0 || d > 400.0
                        })
                    })
                };
                if !clear(a) || !clear(b) || !clear(c) {
                    continue;
                }
                let mut cost_b = Vec::new();
                let mut cost_c = Vec::new();
                let mut vz_b = Vec::new();
                for d in SHIFTS {
                    let mut state = CarState::default();
                    state.phys.pos = a.phys.0;
                    state.phys.vel = a.phys.1;
                    state.phys.rot_mat = a.phys.2;
                    state.phys.ang_vel = a.phys.3;
                    state.is_on_ground = true;
                    state.wheels_with_contact = [Some(rocketsim::RaycastHitInfo::default()); 4];
                    arena.set_ball_state(parked_ball());
                    arena.set_car_state(0, state);
                    let mut at_b = None;
                    for tau in a.tick + 1..=c.tick {
                        let index = timeline.partition_point(|(_, start, _)| *start <= tau);
                        let base = if index == 0 {
                            a.inputs
                        } else {
                            timeline[index - 1].2
                        };
                        let jump_index =
                            timeline.partition_point(|(_, start, _)| *start + d <= tau);
                        let jump = if jump_index == 0 {
                            a.inputs.jump
                        } else {
                            timeline[jump_index - 1].2.jump
                        };
                        arena.set_car_controls(
                            0,
                            CarControls {
                                throttle: base.throttle,
                                steer: base.steer,
                                handbrake: base.handbrake,
                                boost: base.boost,
                                jump,
                                ..CarControls::default()
                            },
                        );
                        arena.step_tick();
                        if tau == b.tick {
                            let s = arena.get_car_state(0);
                            at_b = Some((
                                (s.phys.pos - b.phys.0).length()
                                    + 0.1 * (s.phys.vel - b.phys.1).length(),
                                (s.phys.vel.z - b.phys.1.z).abs(),
                            ));
                        }
                    }
                    let s = arena.get_car_state(0);
                    let at_b = at_b.unwrap_or((f32::NAN, f32::NAN));
                    cost_b.push(at_b.0);
                    vz_b.push(at_b.1);
                    cost_c.push(
                        (s.phys.pos - c.phys.0).length() + 0.1 * (s.phys.vel - c.phys.1).length(),
                    );
                }
                events.push(Event {
                    player: (replay_index, player.clone()),
                    cost_b,
                    cost_c,
                    vz_b,
                    jump_after_b: jump_start > b.tick,
                });
            }
        }
    }

    let n = events.len();
    println!("jump events with three exact packets around the start: {n}");
    if n == 0 {
        return Ok(());
    }
    let best_of = |costs: &Vec<f32>| -> usize {
        let mut best = zero;
        for (i, c) in costs.iter().enumerate() {
            if *c < costs[best] - 1e-4 {
                best = i;
            }
        }
        best
    };
    let fitted: Vec<usize> = events.iter().map(|e| best_of(&e.cost_c)).collect();
    println!("fitted shift d on c (relative to the midpoint rule; +4 is about the frame time):");
    let mut histogram: BTreeMap<i64, usize> = BTreeMap::new();
    for &i in &fitted {
        *histogram.entry(i as i64 - zero as i64).or_default() += 1;
    }
    let line: Vec<String> = histogram
        .iter()
        .map(|(d, count)| format!("{d:+}: {:.1}%", *count as f64 * 100.0 / n as f64))
        .collect();
    println!("  {}", line.join("  "));
    // One constant for the corpus, chosen on c.
    let constant = (0..SHIFTS.clone().count())
        .min_by(|&i, &j| {
            let sum = |k: usize| events.iter().map(|e| e.cost_c[k] as f64).sum::<f64>();
            sum(i).total_cmp(&sum(j))
        })
        .unwrap();
    println!(
        "corpus constant shift chosen on c: d = {:+}",
        constant as i64 - zero as i64
    );
    // Per-player median of the other events' fitted shifts.
    let mut by_player: BTreeMap<&(usize, String), Vec<usize>> = BTreeMap::new();
    for (i, e) in events.iter().enumerate() {
        by_player.entry(&e.player).or_default().push(i);
    }
    let per_player: Vec<usize> = events
        .iter()
        .enumerate()
        .map(|(i, e)| {
            let mut others: Vec<i64> = by_player[&e.player]
                .iter()
                .filter(|&&j| j != i)
                .map(|&j| fitted[j] as i64)
                .collect();
            if others.len() < 5 {
                return constant;
            }
            others.sort();
            others[others.len() / 2] as usize
        })
        .collect();
    let players_used = per_player
        .iter()
        .zip(&events)
        .filter(|(_, e)| by_player[&e.player].len() > 5)
        .count();
    println!(
        "events with at least 5 other events of the same player in the replay: {players_used}"
    );

    println!(
        "\nheld-out packet b (fit used c only). cost = position error + 0.1 x velocity error; vz = vertical velocity error:"
    );
    let report = |name: &str, pick: &dyn Fn(usize) -> usize, subset: &dyn Fn(&Event) -> bool| {
        let mut cost = Vec::new();
        let mut vz = Vec::new();
        for (i, e) in events.iter().enumerate() {
            if !subset(e) {
                continue;
            }
            let k = pick(i);
            if e.cost_b[k].is_nan() {
                continue;
            }
            cost.push(e.cost_b[k]);
            vz.push(e.vz_b[k]);
        }
        println!(
            "  {name:<44} n {:>6}: cost p50/p90/p99 {:>6.1}/{:>6.1}/{:>6.1}  vz p50/p90 {:>6.1}/{:>6.1} UU/s",
            cost.len(),
            quantile(&mut cost, 0.5),
            quantile(&mut cost, 0.9),
            quantile(&mut cost, 0.99),
            quantile(&mut vz, 0.5),
            quantile(&mut vz, 0.9),
        );
    };
    for (label, subset) in [
        ("all events", &(|_: &Event| true) as &dyn Fn(&Event) -> bool),
        ("jump starts after packet b", &|e: &Event| e.jump_after_b),
        ("jump starts before packet b", &|e: &Event| !e.jump_after_b),
    ] {
        println!(" {label}:");
        report("midpoint rule (d = 0)", &|_| zero, subset);
        report("frame time (d = +4)", &|_| zero + 4, subset);
        report("corpus constant", &|_| constant, subset);
        report(
            "player's other events (median d)",
            &|i| per_player[i],
            subset,
        );
        report("fitted on this event's c (offline)", &|i| fitted[i], subset);
    }
    Ok(())
}
