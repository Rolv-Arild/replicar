//! When does an airborne dodge physically start relative to its counter, and is it recoverable from
//! the packets? Offline diagnostic on train/validation, with a held-out packet.
//!
//! Takes consecutive fresh car packets (a, b, c) with exact chain-lag ticks: the car is airborne
//! (z >= 50) and not flipping at a with even dodge, double-jump and flip counters, the dodge counter
//! turns odd (with a fresh dodge torque) at a frame after a and no later than c's frame, the double
//! jump and flip counters do not change, c is at most 40 ticks after a, and no other car is within
//! 400 UU at the three packets. RocketSim is stepped from a with the ball parked, midpoint-rule
//! throttle/steer/handbrake/boost and the jump parity as observed, and the dodge started as the
//! converter does (a jump press with the torque's pitch and yaw at tick T + d, then `cancel` of the
//! flip's pitch torque cancelled), for d = -8..=16 ticks from the midpoint-rule tick of the
//! activation frame and cancel in {0, 0.25, 0.5, 0.75, 1}.
//!
//! The parameters are fitted on c only (position error + 0.1 x velocity error, optionally with the
//! angular-velocity error); b is held out and scored on position, velocity, rotation and angular
//! velocity. Compared: fixed shifts, the shift fitted on this event (offline), and one constant.

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
const CANCELS: [f32; 5] = [0.0, 0.25, 0.5, 0.75, 1.0];

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
    counters: [Option<u8>; 4],
    hitbox: String,
}

type Timeline = Vec<(i64, i64, Inputs)>;

fn parked_ball() -> BallState {
    let mut ball = BallState::default();
    ball.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
    ball
}

/// Errors at a packet: position, velocity, rotation (deg), angular velocity.
type Errors = [f32; 4];

struct Event {
    /// Per (shift, cancel): errors at b and c.
    at_b: Vec<Errors>,
    at_c: Vec<Errors>,
    /// The dodge starts (with the midpoint rule) after packet b.
    after_b: bool,
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: diagnose_dodge_latency <train or validation dir or replay>")?,
    );
    if path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let options = ConvertOptions::default();
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut arenas: BTreeMap<String, Arena> = BTreeMap::new();
    let mut events: Vec<Event> = Vec::new();
    let n_shift = SHIFTS.clone().count();
    let index_of = |shift_i: usize, cancel_i: usize| shift_i * CANCELS.len() + cancel_i;

    for replay_path in replay_paths(&path)? {
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
        let mut lifetimes: BTreeMap<(i32, usize), Vec<Packet>> = BTreeMap::new();
        // (frame, torque) of each dodge activation per car: fresh odd counter with a fresh torque.
        let mut activations: BTreeMap<(i32, usize), Vec<(usize, [f32; 3])>> = BTreeMap::new();
        let mut last_dodge: BTreeMap<(i32, usize), u8> = BTreeMap::new();
        for f in 0..frames.len() {
            if !active(f) {
                continue;
            }
            for car in &frames[f].cars {
                let key = (car.actor_id, car.actor_created_frame);
                let line = timelines.entry(key).or_default();
                let spacing = line.last().map_or(4, |(t, _, _)| tick_of(f) - t);
                line.push((tick_of(f), tick_of(f) - 2 - spacing / 2, inputs_of(car)));
                if let (Some(dodge), Some(torque)) = (
                    car.inputs
                        .dodge_active_raw
                        .as_ref()
                        .filter(|d| d.frame == f),
                    car.inputs
                        .dodge_torque_replay_units
                        .as_ref()
                        .filter(|t| t.frame == f),
                ) {
                    let previous = last_dodge.insert(key, dodge.value);
                    if dodge.value % 2 == 1 && previous.is_some_and(|p| p % 2 == 0) {
                        activations.entry(key).or_default().push((f, torque.value));
                    }
                }
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
                lifetimes.entry(key).or_default().push(Packet {
                    frame: f,
                    tick: tick_of(f) - lag,
                    phys,
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
        for (key, packets) in &lifetimes {
            let timeline = &timelines[key];
            let Some(car_activations) = activations.get(key) else {
                continue;
            };
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
                if a.phys.0.z < 50.0
                    || a.counters[2].is_none_or(|d| d % 2 == 1)
                    || a.counters[1].is_none_or(|d| d % 2 == 1)
                    || a.counters[3].is_some_and(|d| d % 2 == 1)
                    || b.tick <= a.tick
                    || c.tick <= b.tick
                    || c.tick - a.tick > 40
                    || (a.frame..=c.frame).any(|f| !active(f))
                    || a.counters[1] != c.counters[1]
                    || a.counters[3] != c.counters[3]
                {
                    continue;
                }
                // The activation between a's frame and c's frame; no fresh packet between a and it.
                let Some(&(act_frame, torque)) = car_activations
                    .iter()
                    .find(|(g, _)| *g > a.frame && *g <= c.frame)
                else {
                    continue;
                };
                if packets
                    .iter()
                    .any(|p| p.frame > a.frame && p.frame < act_frame)
                {
                    continue;
                }
                let clear = |p: &Packet| {
                    frames[p.frame].cars.iter().all(|o| {
                        o.body.position.as_ref().is_none_or(|op| {
                            let d = (Vec3A::from_array(op.value) - p.phys.0).length();
                            d < 1.0 || d > 400.0
                        })
                    })
                };
                if !clear(a) || !clear(b) || !clear(c) {
                    continue;
                }
                let (tx, ty) = (torque[0], torque[1]);
                let (pitch, yaw) = ((-ty / 2.24).clamp(-1.0, 1.0), (-tx / 2.60).clamp(-1.0, 1.0));
                if (pitch * pitch + yaw * yaw).sqrt() <= 0.01 {
                    continue;
                }
                let act_start = timeline
                    .iter()
                    .find(|(t, _, _)| *t == tick_of(act_frame))
                    .map(|(_, start, _)| *start)
                    .unwrap_or(tick_of(act_frame) - 4);
                let mut at_b = Vec::new();
                let mut at_c = Vec::new();
                for shift_i in 0..n_shift {
                    let d = *SHIFTS.start() + shift_i as i64;
                    for &cancel in &CANCELS {
                        let mut state = CarState::default();
                        state.phys.pos = a.phys.0;
                        state.phys.vel = a.phys.1;
                        state.phys.rot_mat = a.phys.2;
                        state.phys.ang_vel = a.phys.3;
                        state.is_on_ground = false;
                        state.has_jumped = true;
                        state.is_jumping = false;
                        state.air_time_since_jump = 0.05;
                        arena.set_ball_state(parked_ball());
                        arena.set_car_state(0, state);
                        let press = act_start + d;
                        let mut errors_b = [f32::NAN; 4];
                        for tau in a.tick + 1..=c.tick {
                            let index = timeline.partition_point(|(_, start, _)| *start <= tau);
                            let base = if index == 0 {
                                timeline[0].2
                            } else {
                                timeline[index - 1].2
                            };
                            let mut controls = CarControls {
                                throttle: base.throttle,
                                steer: base.steer,
                                handbrake: base.handbrake,
                                boost: base.boost,
                                jump: false,
                                ..CarControls::default()
                            };
                            if tau < press {
                                controls.jump = base.jump && tau < press;
                            } else if tau == press {
                                controls.jump = true;
                                controls.pitch = pitch;
                                controls.yaw = yaw;
                            } else {
                                let sign = arena.get_car_state(0).flip_rel_torque.y.signum();
                                controls.pitch = cancel * sign;
                            }
                            arena.set_car_controls(0, controls);
                            arena.step_tick();
                            let mut end = *arena.get_car_state(0);
                            let speed = end.phys.ang_vel.length();
                            if speed > 5.5 {
                                end.phys.ang_vel *= 5.5 / speed;
                            }
                            let measure = |target: &Packet| -> Errors {
                                [
                                    (end.phys.pos - target.phys.0).length(),
                                    (end.phys.vel - target.phys.1).length(),
                                    rotation_error(end.phys.rot_mat, target.phys.2),
                                    (end.phys.ang_vel - target.phys.3).length(),
                                ]
                            };
                            if tau == b.tick {
                                errors_b = measure(b);
                            }
                            if tau == c.tick {
                                at_c.push(measure(c));
                            }
                        }
                        at_b.push(errors_b);
                    }
                }
                events.push(Event {
                    at_b,
                    at_c,
                    after_b: act_start > b.tick,
                });
            }
        }
    }

    let n = events.len();
    println!("airborne dodge events with three exact packets around the activation: {n}");
    if n == 0 {
        return Ok(());
    }
    let zero = (-*SHIFTS.start()) as usize;
    // Cost for the fit: position + 0.1 velocity (as for jumps), or with angular velocity.
    let fit_cost =
        |e: &Errors, with_ang: bool| e[0] + 0.1 * e[1] + if with_ang { 10.0 * e[3] } else { 0.0 };
    let best_index = |event: &Event, with_ang: bool, fixed_cancel: Option<usize>| -> usize {
        let mut best = index_of(zero, fixed_cancel.unwrap_or(0));
        for shift_i in 0..n_shift {
            for cancel_i in 0..CANCELS.len() {
                if fixed_cancel.is_some_and(|c| c != cancel_i) {
                    continue;
                }
                let k = index_of(shift_i, cancel_i);
                if fit_cost(&event.at_c[k], with_ang) < fit_cost(&event.at_c[best], with_ang) - 1e-4
                {
                    best = k;
                }
            }
        }
        best
    };
    let histogram = |picks: &Vec<usize>| -> String {
        let mut counts: BTreeMap<i64, usize> = BTreeMap::new();
        for &k in picks {
            *counts
                .entry(*SHIFTS.start() + (k / CANCELS.len()) as i64)
                .or_default() += 1;
        }
        counts
            .iter()
            .map(|(d, c)| format!("{d:+}:{:.0}%", *c as f64 * 100.0 / picks.len() as f64))
            .collect::<Vec<_>>()
            .join(" ")
    };
    let fitted: Vec<usize> = events.iter().map(|e| best_index(e, false, None)).collect();
    let cancel_hist: Vec<String> = (0..CANCELS.len())
        .map(|c| {
            format!(
                "{:.2}: {:.0}%",
                CANCELS[c],
                fitted.iter().filter(|&&k| k % CANCELS.len() == c).count() as f64 * 100.0
                    / n as f64
            )
        })
        .collect();
    println!(
        "fitted shift (pos + 0.1 vel on c, joint with cancel): {}",
        histogram(&fitted)
    );
    println!("fitted cancel: {}", cancel_hist.join("  "));

    println!(
        "\nheld-out packet b (fit used c only); errors position UU / velocity UU/s / rotation deg / angular velocity rad/s, p50 and p90:"
    );
    let report = |name: &str,
                  pick: &dyn Fn(usize, &Event) -> usize,
                  subset: &dyn Fn(&Event) -> bool| {
        let mut cols: [Vec<f32>; 4] = Default::default();
        for (i, e) in events.iter().enumerate() {
            if !subset(e) {
                continue;
            }
            let k = pick(i, e);
            if e.at_b[k][0].is_nan() {
                continue;
            }
            for (c, col) in cols.iter_mut().enumerate() {
                col.push(e.at_b[k][c]);
            }
        }
        println!(
            "  {name:<46} n {:>5}: pos {:>5.1}/{:>5.1}  vel {:>6.1}/{:>6.1}  rot {:>5.1}/{:>5.1}  ang {:>4.2}/{:>4.2}",
            cols[0].len(),
            quantile(&mut cols[0], 0.5),
            quantile(&mut cols[0], 0.9),
            quantile(&mut cols[1], 0.5),
            quantile(&mut cols[1], 0.9),
            quantile(&mut cols[2], 0.5),
            quantile(&mut cols[2], 0.9),
            quantile(&mut cols[3], 0.5),
            quantile(&mut cols[3], 0.9),
        );
    };
    for (label, subset) in [
        (
            "dodge starts before packet b (midpoint rule)",
            &(|e: &Event| !e.after_b) as &dyn Fn(&Event) -> bool,
        ),
        ("dodge starts after packet b", &|e: &Event| e.after_b),
    ] {
        println!(" {label}:");
        for (d, name) in [
            (0i64, "d = 0 (midpoint rule), cancel 0.5"),
            (2, "d = +2, cancel 0.5"),
            (4, "d = +4 (about frame time), cancel 0.5"),
        ] {
            report(
                name,
                &move |_, _| index_of((d - *SHIFTS.start()) as usize, 2),
                subset,
            );
        }
        report("d = 0, cancel 0", &|_, _| index_of(zero, 0), subset);
        report("d = 0, cancel 1", &|_, _| index_of(zero, 4), subset);
        report(
            "fitted d only, cancel 0.5 (pos+vel on c)",
            &|_, e| best_index(e, false, Some(2)),
            subset,
        );
        report(
            "fitted d and cancel (pos+vel on c)",
            &|_, e| best_index(e, false, None),
            subset,
        );
        report(
            "fitted d and cancel (pos+vel+ang on c)",
            &|_, e| best_index(e, true, None),
            subset,
        );
    }
    Ok(())
}

fn rotation_error(a: Mat3A, b: Mat3A) -> f32 {
    let trace = a.x_axis.dot(b.x_axis) + a.y_axis.dot(b.y_axis) + a.z_axis.dot(b.z_axis);
    ((trace - 1.0) * 0.5).clamp(-1.0, 1.0).acos().to_degrees()
}
