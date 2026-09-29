//! Where does ground-driving error come from? Train/validation diagnostic, offline.
//!
//! Selects consecutive fresh packets (a, b) of one car with chain-lag physical ticks (so the elapsed
//! whole ticks k are known exactly), both flat on the ground (center z < 30 UU, up axis z > 0.97),
//! no jump or dodge counter change or odd counter, unchanged boost parity, active play throughout,
//! and no ball or other car within 400 UU at either end. RocketSim is started from packet a (all
//! wheels in contact) and stepped k ticks with the observed throttle, steer, handbrake and boost,
//! then compared with packet b in the car's frame. Three control hypotheses are compared: the
//! controls of a's frame held (what a causal converter has), of b's frame held, and linear
//! interpolation from a to b (both of the latter use a future observation, so they measure how much
//! of the error is unobserved control changes rather than the ground model). Short intervals
//! (1-2 ticks) barely depend on the controls, so they show the model's per-tick bias.

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

fn physics(body: &Body, frame: usize) -> Option<(Vec3A, Vec3A, Mat3A, Vec3A)> {
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

/// The scratch arena's ball is parked out of reach; at kickoff it would sit in the way of cars
/// crossing the centre.
fn parked_ball() -> BallState {
    let mut ball = BallState::default();
    ball.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
    ball
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
}

struct Packet {
    frame: usize,
    tick: i64,
    phys: (Vec3A, Vec3A, Mat3A, Vec3A),
    inputs: Inputs,
    counters: [Option<u8>; 4],
    hitbox: String,
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
    }
}

/// RocketSim's handbrake value (a ramp: +5/s while held, -2/s otherwise, in [0, 1]) at `tick`,
/// replayed from the observed handbrake timeline over the preceding 120 ticks. A car state taken
/// from a packet does not carry it, and the converter's arena keeps it across packets.
fn handbrake_value(timeline: &[(i64, i64, Inputs)], tick: i64) -> f32 {
    let mut value = 0.0f32;
    for tau in tick - 120..=tick {
        let index = timeline.partition_point(|(_, start, _)| *start <= tau);
        let held = index > 0 && timeline[index - 1].2.handbrake;
        value = (value + if held { 5.0 } else { -2.0 } / 120.0).clamp(0.0, 1.0);
    }
    value
}

fn ground_state(phys: &(Vec3A, Vec3A, Mat3A, Vec3A), handbrake_val: f32) -> CarState {
    let mut state = CarState::default();
    state.handbrake_val = handbrake_val;
    state.phys.pos = phys.0;
    state.phys.vel = phys.1;
    state.phys.rot_mat = phys.2;
    state.phys.ang_vel = phys.3;
    state.is_on_ground = true;
    state.wheels_with_contact = [Some(rocketsim::RaycastHitInfo::default()); 4];
    state
}

#[derive(Default)]
struct Rows {
    pos: Vec<f32>,
    vel: Vec<f32>,
    rot: Vec<f32>,
    ang: Vec<f32>,
    /// Signed sim - true velocity in the true car frame (forward, right, up).
    vel_local: [Vec<f32>; 3],
    /// Signed sim - true angular velocity in the true car frame (roll, pitch, yaw axes x, y, z).
    ang_local: [Vec<f32>; 3],
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: diagnose_ground_driving <train or validation dir or replay>")?,
    );
    if path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let options = ConvertOptions::default();
    // By default the handbrake ramp starts from the observed history, as the converter's arena
    // does; --reset-handbrake-value starts every pair from zero.
    // --trace N: print N frame-by-frame windows from each unexplained hard-steer class.
    let trace_limit: usize = env::args()
        .skip_while(|arg| arg != "--trace")
        .nth(1)
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let trace_stride: usize = env::args()
        .skip_while(|arg| arg != "--trace-stride")
        .nth(1)
        .and_then(|v| v.parse().ok())
        .unwrap_or(97);
    let mut trace_seen: BTreeMap<&'static str, usize> = BTreeMap::new();
    let mut trace_printed: BTreeMap<&'static str, usize> = BTreeMap::new();
    let with_handbrake_history = !env::args_os().any(|arg| arg == "--reset-handbrake-value");
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut arenas: BTreeMap<String, Arena> = BTreeMap::new();
    let mut rows: BTreeMap<String, Rows> = BTreeMap::new();
    let mut pairs = 0usize;
    // Hard steering with unchanged controls: signed error times steer direction, by speed and boost.
    let mut steer_bias: BTreeMap<String, [Vec<f32>; 3]> = BTreeMap::new();
    // Hard-steer pairs split into a tail (sim turns >0.2 rad/s less than real) and the rest.
    // Per hard-steer pair: which of {observed handbrake, forced handbrake} reproduces the packet.
    let mut brake_classes: BTreeMap<&'static str, Vec<[f32; 8]>> = BTreeMap::new();
    let mut tail_features: BTreeMap<&'static str, Vec<[f32; 9]>> = BTreeMap::new();

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
        let mut lifetimes: BTreeMap<(i32, usize), Vec<Packet>> = BTreeMap::new();
        // Observed controls at every active frame of each car lifetime: (nominal frame tick, inputs).
        let mut timelines: BTreeMap<(i32, usize), Vec<(i64, i64, Inputs)>> = BTreeMap::new();
        for f in 0..frames.len() {
            if active(f) {
                for car in &frames[f].cars {
                    let line = timelines
                        .entry((car.actor_id, car.actor_created_frame))
                        .or_default();
                    // A change first seen at this frame happened in the physical interval that ends
                    // at this frame's state, which is on average 2 ticks (half the 0-4 tick lag
                    // range) before the frame time; take the middle of that interval as the switch.
                    let spacing = line.last().map_or(4, |(t, _, _)| tick_of(f) - t);
                    line.push((tick_of(f), tick_of(f) - 2 - spacing / 2, inputs_of(car)));
                }
            }
        }
        for f in 0..frames.len() {
            if !active(f) {
                continue;
            }
            for car in &frames[f].cars {
                let Some(phys) = physics(&car.body, f) else {
                    continue;
                };
                let Some(lag) = lag_of(f, car.actor_id) else {
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
                    .entry((car.actor_id, car.actor_created_frame))
                    .or_default()
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
        for (key, packets) in &lifetimes {
            let timeline = &timelines[key];
            for pair in packets.windows(2) {
                let (a, b) = (&pair[0], &pair[1]);
                let k = b.tick - a.tick;
                if !(1..=15).contains(&k)
                    || a.counters != b.counters
                    || a.counters.iter().flatten().any(|c| c % 2 == 1)
                    || a.inputs.boost != b.inputs.boost
                    || (a.frame..=b.frame).any(|f| !active(f))
                {
                    continue;
                }
                let flat = |p: &Packet| p.phys.0.z < 30.0 && p.phys.2.z_axis.z > 0.97;
                if !flat(a) || !flat(b) {
                    continue;
                }
                let clear = |p: &Packet| {
                    let ball_far = frames[p.frame].ball.as_ref().is_none_or(|ball| {
                        ball.position.as_ref().is_none_or(|bp| {
                            (Vec3A::from_array(bp.value) - p.phys.0).length() > 400.0
                        })
                    });
                    let cars_far = frames[p.frame].cars.iter().all(|o| {
                        o.body.position.as_ref().is_none_or(|op| {
                            let d = (Vec3A::from_array(op.value) - p.phys.0).length();
                            d < 1.0 || d > 400.0
                        })
                    });
                    ball_far && cars_far
                };
                if !clear(a) || !clear(b) {
                    continue;
                }
                let arena = arenas.entry(a.hitbox.clone()).or_insert_with(|| {
                    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
                    arena.add_car(Team::Blue, hitbox(&a.hitbox));
                    arena
                });
                pairs += 1;
                let speed = a.phys.1.length();
                let bucket_k = match k {
                    1..=2 => "k 1-2",
                    3..=5 => "k 3-5",
                    6..=9 => "k 6-9",
                    10..=12 => "k 10-12",
                    _ => "k 13-15",
                };
                let mut labels = vec!["all".to_string(), bucket_k.to_string()];
                labels.push(format!(
                    "speed {}",
                    if speed < 300.0 {
                        "<300"
                    } else if speed < 1000.0 {
                        "300-1000"
                    } else if speed < 1800.0 {
                        "1000-1800"
                    } else {
                        ">=1800"
                    }
                ));
                labels.push(format!(
                    "throttle {}",
                    if a.inputs.throttle > 0.05 {
                        "forward"
                    } else if a.inputs.throttle < -0.05 {
                        "reverse"
                    } else {
                        "none"
                    }
                ));
                labels.push(format!(
                    "steer {}",
                    if a.inputs.steer.abs() > 0.5 {
                        "hard (>0.5)"
                    } else if a.inputs.steer.abs() > 0.05 {
                        "some"
                    } else {
                        "straight"
                    }
                ));
                labels.push(format!(
                    "handbrake {}",
                    if a.inputs.handbrake || b.inputs.handbrake {
                        "on"
                    } else {
                        "off"
                    }
                ));
                labels.push(format!(
                    "boost {}",
                    if a.inputs.boost { "on" } else { "off" }
                ));
                let brake_nearby = timeline
                    .iter()
                    .any(|(t, _, i)| i.handbrake && *t >= a.tick - 32 && *t <= b.tick + 32);
                labels.push(format!(
                    "handbrake within 32 ticks: {}",
                    if brake_nearby { "yes" } else { "no" }
                ));
                let changed = (a.inputs.throttle - b.inputs.throttle).abs() > 0.1
                    || (a.inputs.steer - b.inputs.steer).abs() > 0.1
                    || a.inputs.handbrake != b.inputs.handbrake;
                labels.push(format!(
                    "controls {}",
                    if changed {
                        "changed a to b"
                    } else {
                        "same at a and b"
                    }
                ));

                let mut hypotheses: Vec<String> = vec![
                    "a held".into(),
                    "b held".into(),
                    "a to b interpolated".into(),
                ];
                hypotheses.push("per frame, midpoint rule".into());
                hypotheses.push("midpoint, handbrake forced on".into());
                for shift in [-24i64, -16, -12, -8, -4, 4, 8, 16] {
                    hypotheses.push(format!("midpoint, handbrake shift {shift:+}"));
                }
                for shift in [-8i64, -6, -4, -3, -2, -1, 0, 1, 2, 4] {
                    hypotheses.push(format!("per frame, shift {shift:+}"));
                }
                if !changed {
                    // Where does the error of intervals with unchanged controls come from?
                    let extra: Vec<String> = labels
                        .iter()
                        .filter(|l| !l.starts_with("controls ") && *l != "all")
                        .map(|l| format!("same controls, {l}"))
                        .collect();
                    labels.extend(extra);
                    let near_wall = a.phys.0.x.abs() > 3800.0
                        || a.phys.0.y.abs() > 4700.0
                        || b.phys.0.x.abs() > 3800.0
                        || b.phys.0.y.abs() > 4700.0;
                    labels.push(format!(
                        "same controls, {}",
                        if near_wall {
                            "near a wall"
                        } else {
                            "open floor"
                        }
                    ));
                    labels.push(format!(
                        "same controls, up axis z {}",
                        if a.phys.2.z_axis.z.min(b.phys.2.z_axis.z) > 0.999 {
                            "> 0.999 (level)"
                        } else {
                            "<= 0.999 (tilted)"
                        }
                    ));
                }
                let mut pair_eval: [Option<f32>; 2] = [None, None];
                for hypothesis in hypotheses.iter().map(String::as_str) {
                    arena.set_ball_state(parked_ball());
                    arena.set_car_state(
                        0,
                        ground_state(
                            &a.phys,
                            if with_handbrake_history {
                                handbrake_value(timeline, a.tick)
                            } else {
                                0.0
                            },
                        ),
                    );
                    for step in 0..k {
                        let t = (step as f32 + 0.5) / k as f32;
                        let inputs = match hypothesis {
                            "a held" => a.inputs,
                            "b held" => b.inputs,
                            h if h.starts_with("midpoint, handbrake shift") => {
                                let shift: i64 = h.rsplit(' ').next().unwrap().parse().unwrap();
                                let tau = a.tick + step + 1;
                                let index = timeline.partition_point(|(_, start, _)| *start <= tau);
                                let mut inputs = if index == 0 {
                                    a.inputs
                                } else {
                                    timeline[index - 1].2
                                };
                                let index =
                                    timeline.partition_point(|(_, start, _)| *start + shift <= tau);
                                inputs.handbrake = if index == 0 {
                                    a.inputs.handbrake
                                } else {
                                    timeline[index - 1].2.handbrake
                                };
                                inputs
                            }
                            "per frame, midpoint rule" | "midpoint, handbrake forced on" => {
                                let tau = a.tick + step + 1;
                                let index = timeline.partition_point(|(_, start, _)| *start <= tau);
                                let mut inputs = if index == 0 {
                                    a.inputs
                                } else {
                                    timeline[index - 1].2
                                };
                                inputs.handbrake |= hypothesis == "midpoint, handbrake forced on";
                                inputs
                            }
                            h if h.starts_with("per frame") => {
                                let shift: i64 = h.rsplit(' ').next().unwrap().parse().unwrap();
                                // Controls of the latest frame whose nominal tick + shift is
                                // at or before this physical tick.
                                let tau = a.tick + step + 1;
                                let index = timeline.partition_point(|(t, _, _)| t + shift <= tau);
                                if index == 0 {
                                    a.inputs
                                } else {
                                    timeline[index - 1].2
                                }
                            }
                            _ => Inputs {
                                throttle: a.inputs.throttle
                                    + (b.inputs.throttle - a.inputs.throttle) * t,
                                steer: a.inputs.steer + (b.inputs.steer - a.inputs.steer) * t,
                                handbrake: if t < 0.5 {
                                    a.inputs.handbrake
                                } else {
                                    b.inputs.handbrake
                                },
                                boost: a.inputs.boost,
                            },
                        };
                        arena.set_car_controls(
                            0,
                            CarControls {
                                throttle: inputs.throttle,
                                steer: inputs.steer,
                                handbrake: inputs.handbrake,
                                boost: inputs.boost,
                                ..CarControls::default()
                            },
                        );
                        arena.step_tick();
                    }
                    let sim = arena.get_car_state(0);
                    let true_rot = b.phys.2;
                    let quat_sim = Quat::from_mat3a(&sim.phys.rot_mat);
                    let quat_true = Quat::from_mat3a(&true_rot);
                    let rot = quat_sim.angle_between(quat_true).to_degrees();
                    let vel_error = sim.phys.vel - b.phys.1;
                    let ang_error = sim.phys.ang_vel - b.phys.3;
                    let local = |v: Vec3A| {
                        [
                            v.dot(true_rot.x_axis),
                            v.dot(true_rot.y_axis),
                            v.dot(true_rot.z_axis),
                        ]
                    };
                    if (hypothesis == "per frame, midpoint rule"
                        || hypothesis == "midpoint, handbrake forced on")
                        && !changed
                        && a.inputs.steer.abs() > 0.5
                        && a.inputs.steer * b.inputs.steer > 0.0
                    {
                        let sign = a.inputs.steer.signum();
                        pair_eval[usize::from(hypothesis == "midpoint, handbrake forced on")] =
                            Some(ang_error.length());
                        if hypothesis == "midpoint, handbrake forced on" {
                            // Same hard-steer pairs with the handbrake forced on, split by whether
                            // the observed handbrake was already on (never, in this selection) and
                            // by the size of the initial slide.
                            let slide = a.phys.1.dot(a.phys.2.y_axis).abs() > 150.0;
                            for key in [
                                "forced handbrake, all hard-steer".to_string(),
                                format!(
                                    "forced handbrake, initial |lateral| {} 150",
                                    if slide { ">" } else { "<=" }
                                ),
                            ] {
                                let entry = steer_bias.entry(key).or_default();
                                entry[0].push(local(ang_error)[2] * sign);
                                entry[1].push(local(vel_error)[1] * sign);
                                entry[2].push(local(vel_error)[0]);
                            }
                        } else {
                            let slide = a.phys.1.dot(a.phys.2.y_axis).abs() > 150.0;
                            for key in [format!(
                                "observed handbrake, initial |lateral| {} 150",
                                if slide { ">" } else { "<=" }
                            )] {
                                let entry = steer_bias.entry(key).or_default();
                                entry[0].push(local(ang_error)[2] * sign);
                                entry[1].push(local(vel_error)[1] * sign);
                                entry[2].push(local(vel_error)[0]);
                            }
                        }
                        if hypothesis == "per frame, midpoint rule" {
                            let la = |v: Vec3A| {
                                [
                                    v.dot(a.phys.2.x_axis),
                                    v.dot(a.phys.2.y_axis),
                                    v.dot(a.phys.2.z_axis),
                                ]
                            };
                            let tail = local(ang_error)[2] * sign < -0.2;
                            tail_features
                                .entry(if tail { "tail" } else { "rest" })
                                .or_default()
                                .push([
                                    la(a.phys.1)[1].abs(),
                                    la(a.phys.1)[0],
                                    la(a.phys.3)[2] * sign,
                                    b.phys.3.dot(b.phys.2.z_axis) * sign,
                                    sim.phys.ang_vel.dot(sim.phys.rot_mat.z_axis) * sign,
                                    a.inputs.throttle,
                                    f32::from(a.inputs.handbrake || b.inputs.handbrake),
                                    f32::from(a.inputs.boost),
                                    k as f32,
                                ]);
                            let speed_bucket = ((speed / 200.0).floor() as i32).clamp(0, 11) * 200;
                            for key in [
                                format!("speed {speed_bucket:>4}+, boost {}", a.inputs.boost),
                                "all hard-steer".to_string(),
                            ] {
                                let entry = steer_bias.entry(key).or_default();
                                entry[0].push(local(ang_error)[2] * sign);
                                entry[1].push(local(vel_error)[1] * sign);
                                entry[2].push(local(vel_error)[0]);
                            }
                        }
                    }
                    for label in &labels {
                        // Breakdowns other than the total and interval length are shown for the
                        // causal hypothesis only.
                        let compact = label == "all"
                            || label.starts_with("k ")
                            || label.starts_with("handbrake within")
                            || label.starts_with("controls ");
                        if hypothesis != "a held"
                            && hypothesis != "per frame, midpoint rule"
                            && !compact
                        {
                            continue;
                        }
                        let row = rows
                            .entry(format!("{hypothesis:<20} | {label}"))
                            .or_default();
                        row.pos.push((sim.phys.pos - b.phys.0).length());
                        row.vel.push(vel_error.length());
                        row.rot.push(rot);
                        row.ang.push(ang_error.length());
                        for (axis, value) in local(vel_error).into_iter().enumerate() {
                            row.vel_local[axis].push(value);
                        }
                        for (axis, value) in local(ang_error).into_iter().enumerate() {
                            row.ang_local[axis].push(value);
                        }
                    }
                }
                if let [Some(observed), Some(forced)] = pair_eval {
                    let class = match (observed < 0.15, forced < 0.15) {
                        (true, true) => "both fit",
                        (true, false) => "observed handbrake fits",
                        (false, true) => "forced handbrake fits",
                        (false, false) => "neither fits",
                    };
                    let nearby = timeline
                        .iter()
                        .any(|(t, _, i)| i.handbrake && *t >= a.tick - 32 && *t <= b.tick + 32);
                    if trace_limit > 0
                        && (class == "forced handbrake fits" || class == "neither fits")
                    {
                        let seen = trace_seen.entry(class).or_default();
                        *seen += 1;
                        let printed = trace_printed.entry(class).or_default();
                        if *seen % trace_stride == 0 && *printed < trace_limit {
                            *printed += 1;
                            let sign = a.inputs.steer.signum();
                            println!(
                                "\nTRACE {class} #{printed}: {} car {} frames {}..{} ticks {}..{} (k {k}) speed {:.0} lateral {:.0} yaw {:.2} true-yaw-at-b {:.2}",
                                replay_path.file_name().unwrap().to_string_lossy(),
                                key.0,
                                a.frame,
                                b.frame,
                                a.tick,
                                b.tick,
                                a.phys.1.length(),
                                a.phys.1.dot(a.phys.2.y_axis) * sign,
                                a.phys.3.dot(a.phys.2.z_axis) * sign,
                                b.phys.3.dot(b.phys.2.z_axis) * sign,
                            );
                            for f in a.frame.saturating_sub(3)..=(b.frame + 3).min(frames.len() - 1)
                            {
                                let Some(car) = frames[f]
                                    .cars
                                    .iter()
                                    .find(|c| (c.actor_id, c.actor_created_frame) == *key)
                                else {
                                    continue;
                                };
                                let stamp = |frame: Option<usize>| {
                                    frame.map_or("-".to_string(), |x| {
                                        if x == f {
                                            "fresh".to_string()
                                        } else {
                                            format!("f{x}")
                                        }
                                    })
                                };
                                let fresh_yaw = physics(&car.body, f)
                                    .map_or("      ".to_string(), |ph| {
                                        format!("{:>6.2}", ph.3.dot(ph.2.z_axis) * sign)
                                    });
                                println!(
                                    "  frame {f} nominal tick {:>5}{}  steer {:>5.2}@{:<6} throttle {:>5.2}@{:<6} handbrake {}@{:<6} boost-raw {}@{:<6} true yaw {fresh_yaw}",
                                    tick_of(f),
                                    if f == a.frame {
                                        " (a)"
                                    } else if f == b.frame {
                                        " (b)"
                                    } else {
                                        "    "
                                    },
                                    car.inputs.steer.as_ref().map_or(f32::NAN, |v| v.value),
                                    stamp(car.inputs.steer.as_ref().map(|v| v.frame)),
                                    car.inputs.throttle.as_ref().map_or(f32::NAN, |v| v.value),
                                    stamp(car.inputs.throttle.as_ref().map(|v| v.frame)),
                                    car.inputs
                                        .handbrake
                                        .as_ref()
                                        .map_or("?".to_string(), |v| v.value.to_string()),
                                    stamp(car.inputs.handbrake.as_ref().map(|v| v.frame)),
                                    car.inputs
                                        .boost_active_raw
                                        .as_ref()
                                        .map_or("?".to_string(), |v| v.value.to_string()),
                                    stamp(car.inputs.boost_active_raw.as_ref().map(|v| v.frame)),
                                );
                            }
                            for forced_run in [false, true] {
                                arena.set_ball_state(parked_ball());
                                arena.set_car_state(
                                    0,
                                    ground_state(
                                        &a.phys,
                                        if with_handbrake_history {
                                            handbrake_value(timeline, a.tick)
                                        } else {
                                            0.0
                                        },
                                    ),
                                );
                                let mut line = format!(
                                    "  sim, handbrake {:<8} yaw x sign / lateral x sign per tick:",
                                    if forced_run { "forced" } else { "observed" }
                                );
                                for step in 0..k {
                                    let tau = a.tick + step + 1;
                                    let index =
                                        timeline.partition_point(|(_, start, _)| *start <= tau);
                                    let mut inputs = if index == 0 {
                                        a.inputs
                                    } else {
                                        timeline[index - 1].2
                                    };
                                    inputs.handbrake |= forced_run;
                                    arena.set_car_controls(
                                        0,
                                        CarControls {
                                            throttle: inputs.throttle,
                                            steer: inputs.steer,
                                            handbrake: inputs.handbrake,
                                            boost: inputs.boost,
                                            ..CarControls::default()
                                        },
                                    );
                                    arena.step_tick();
                                    let st = arena.get_car_state(0);
                                    line += &format!(
                                        " {:.2}/{:.0}",
                                        st.phys.ang_vel.dot(st.phys.rot_mat.z_axis) * sign,
                                        st.phys.vel.dot(st.phys.rot_mat.y_axis) * sign
                                    );
                                }
                                println!("{line}");
                            }
                        }
                    }
                    brake_classes.entry(class).or_default().push([
                        a.phys.1.dot(a.phys.2.y_axis).abs(),
                        a.phys.3.dot(a.phys.2.z_axis).abs(),
                        a.phys.1.length(),
                        a.inputs.throttle,
                        f32::from(nearby),
                        k as f32,
                        observed,
                        forced,
                    ]);
                }
            }
        }
    }

    println!("flat-ground packet pairs: {pairs}");
    println!(
        "{:<44} {:>7} | {:>14} | {:>14} | {:>14} | {:>14}",
        "hypothesis | group",
        "n",
        "pos UU p50/p90",
        "vel UU/s p50/90",
        "rot deg p50/p90",
        "ang rad/s p50/90"
    );
    for (label, row) in rows.iter_mut() {
        println!(
            "{:<44} {:>7} | {:>6.2}/{:>7.2} | {:>6.1}/{:>7.1} | {:>6.2}/{:>7.2} | {:>6.3}/{:>7.3}",
            label,
            row.pos.len(),
            quantile(&mut row.pos, 0.5),
            quantile(&mut row.pos, 0.9),
            quantile(&mut row.vel, 0.5),
            quantile(&mut row.vel, 0.9),
            quantile(&mut row.rot, 0.5),
            quantile(&mut row.rot, 0.9),
            quantile(&mut row.ang, 0.5),
            quantile(&mut row.ang, 0.9),
        );
    }
    println!(
        "\nhard-steer pairs, sim turning >0.2 rad/s less than real ('tail') vs the rest, medians [p10, p90]:"
    );
    println!(
        "hard-steer pairs (unchanged hard steer, observed handbrake off at both ends) by which handbrake setting reproduces packet b (angular velocity error < 0.15 rad/s), medians:"
    );
    let total: usize = brake_classes.values().map(Vec::len).sum();
    for (name, rows) in brake_classes.iter_mut() {
        let mut text = format!(
            "  {name:<24} n {:>6} ({:.1}%):",
            rows.len(),
            rows.len() as f64 * 100.0 / total.max(1) as f64
        );
        for (i, label) in [
            "|lateral vel at a|",
            "|yaw rate at a|",
            "speed",
            "throttle",
            "handbrake observed within 32 ticks",
            "k",
            "observed-setting error",
            "forced-setting error",
        ]
        .iter()
        .enumerate()
        {
            let mut column: Vec<f32> = rows.iter().map(|r| r[i]).collect();
            text += &format!(" {label} {:.2};", quantile(&mut column, 0.5));
        }
        println!("{text}");
    }
    for (name, rows) in tail_features.iter_mut() {
        let mut text = format!("  {name:<5} n {:>6}:", rows.len());
        for (i, label) in [
            "|lateral vel at a|",
            "forward vel at a",
            "yaw rate at a",
            "true yaw rate at b",
            "sim yaw rate at b",
            "throttle",
            "handbrake share",
            "boost share",
            "k",
        ]
        .iter()
        .enumerate()
        {
            let mut column: Vec<f32> = rows.iter().map(|r| r[i]).collect();
            text += &format!(
                " {label} {:.2} [{:.2},{:.2}];",
                quantile(&mut column, 0.5),
                quantile(&mut column, 0.1),
                quantile(&mut column, 0.9)
            );
        }
        println!("{text}");
    }
    println!(
        "\nhard steering (|steer| > 0.5, same sign and unchanged at both packets), sim - true: yaw rate error x steer direction (rad/s), lateral velocity error x steer direction (UU/s), forward velocity error (UU/s), p10/p50/p90:"
    );
    for (key, values) in steer_bias.iter_mut() {
        println!(
            "  {key:<24} n {:>6}: yaw {:.3}/{:.3}/{:.3}  lateral {:.1}/{:.1}/{:.1}  forward {:.1}/{:.1}/{:.1}",
            values[0].len(),
            quantile(&mut values[0], 0.1),
            quantile(&mut values[0], 0.5),
            quantile(&mut values[0], 0.9),
            quantile(&mut values[1], 0.1),
            quantile(&mut values[1], 0.5),
            quantile(&mut values[1], 0.9),
            quantile(&mut values[2], 0.1),
            quantile(&mut values[2], 0.5),
            quantile(&mut values[2], 0.9),
        );
    }
    println!(
        "\nsigned error sim - true in the true car frame, p10/p50/p90 (group: velocity forward, right, up UU/s; angular velocity x, y, z rad/s)"
    );
    for (label, row) in rows.iter_mut() {
        if !(label.contains("| all") || label.contains("| k ")) {
            continue;
        }
        let mut text = String::new();
        for axis in 0..3 {
            text += &format!(
                " v{}: {:.1}/{:.1}/{:.1}",
                ["f", "r", "u"][axis],
                quantile(&mut row.vel_local[axis], 0.1),
                quantile(&mut row.vel_local[axis], 0.5),
                quantile(&mut row.vel_local[axis], 0.9)
            );
        }
        for axis in 0..3 {
            text += &format!(
                " w{}: {:.3}/{:.3}/{:.3}",
                ["x", "y", "z"][axis],
                quantile(&mut row.ang_local[axis], 0.1),
                quantile(&mut row.ang_local[axis], 0.5),
                quantile(&mut row.ang_local[axis], 0.9)
            );
        }
        println!("{label:<44}{text}");
    }
    Ok(())
}
