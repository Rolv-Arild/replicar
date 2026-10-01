//! Score the converter's reconstruction of a replay against the server's true states (RLBot recording).
//!
//! The recording (`states.jsonl`) has every car's true state at every physics tick of the match; the
//! replay is the same match as saved by the host or by a client (with network delay). Each replay frame
//! is mapped to a server tick: fresh car packets that equal a recorded position exactly (0.01 UU grid)
//! give `timeline_tick - frame_num`; subtracting the converter's inferred lag of that packet leaves the
//! offset between the converter's timeline and the server's ticks, which drifts slowly, so its running
//! mode over neighbouring matched packets is used (one nuisance value per time window, not per frame).
//! The exported car state of every frame is compared with the truth at `timeline_tick - offset`.
//!
//! usage: rlbot_reconstruction <replay> <states.jsonl> [--zero-lag]

use std::collections::{BTreeMap, HashMap};
use std::env;
use std::error::Error;
use std::fs::{self, File};
use std::io::{BufRead, BufReader};
use std::path::PathBuf;

use glam::{Mat3A, Vec3A};
use replay_to_rocketsim::conversion::{ConvertOptions, convert_observations};
use replay_to_rocketsim::observations::{Body, extract};
use serde_json::Value;

fn vec3(v: &Value) -> Vec3A {
    Vec3A::new(
        v["x"].as_f64().unwrap_or(0.0) as f32,
        v["y"].as_f64().unwrap_or(0.0) as f32,
        v["z"].as_f64().unwrap_or(0.0) as f32,
    )
}

fn matrix(v: &Value) -> Mat3A {
    let (p, y, r) = (
        v["pitch"].as_f64().unwrap_or(0.0) as f32,
        v["yaw"].as_f64().unwrap_or(0.0) as f32,
        v["roll"].as_f64().unwrap_or(0.0) as f32,
    );
    let (cp, sp, cy, sy, cr, sr) = (p.cos(), p.sin(), y.cos(), y.sin(), r.cos(), r.sin());
    Mat3A::from_cols(
        Vec3A::new(cp * cy, cp * sy, sp),
        Vec3A::new(sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp),
        Vec3A::new(-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp),
    )
}

fn rotation_error(a: Mat3A, b: Mat3A) -> f32 {
    let trace = a.x_axis.dot(b.x_axis) + a.y_axis.dot(b.y_axis) + a.z_axis.dot(b.z_axis);
    ((trace - 1.0) * 0.5).clamp(-1.0, 1.0).acos().to_degrees()
}

fn quantile(values: &mut [f32], q: f64) -> f32 {
    if values.is_empty() {
        return f32::NAN;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

struct Truth {
    pos: Vec3A,
    vel: Vec3A,
    rot: Mat3A,
    ang: Vec3A,
    air_state: u64,
    demolished: bool,
    has_dodged: bool,
    boost: f32,
    has_jumped: bool,
    has_double_jumped: bool,
    dodge_timeout: f32,
    supersonic: bool,
    dodge_dir: (f32, f32),
    /// The input applied in the tick that ended at this packet: throttle, steer, pitch, yaw, roll,
    /// then jump, boost, handbrake as 0 or 1.
    input: [f32; 8],
}

#[derive(Default)]
struct Rows {
    pos: Vec<f32>,
    vel: Vec<f32>,
    rot: Vec<f32>,
    ang: Vec<f32>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 2 {
        return Err("usage: rlbot_reconstruction <replay> <states.jsonl> [--zero-lag]".into());
    }
    let (replay_path, states_path) = (PathBuf::from(&args[0]), PathBuf::from(&args[1]));
    if replay_path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let zero_lag = args.iter().any(|a| a == "--zero-lag");

    // Truth: frame_num -> player name -> state; and exact-position index per name.
    let mut truth: HashMap<u64, HashMap<String, Truth>> = HashMap::new();
    let mut by_position: HashMap<(String, [i32; 3]), Vec<u64>> = HashMap::new();
    let key = |p: Vec3A| {
        [
            (p.x * 100.0).round() as i32,
            (p.y * 100.0).round() as i32,
            (p.z * 100.0).round() as i32,
        ]
    };
    for line in BufReader::new(File::open(&states_path)?).lines() {
        let row: Value = serde_json::from_str(&line?)?;
        let packet = &row["packet"];
        let Some(frame) = packet["match_info"]["frame_num"].as_u64() else {
            continue;
        };
        let entry = truth.entry(frame).or_default();
        if !entry.is_empty() {
            continue;
        }
        for pl in packet["players"].as_array().ok_or("players")? {
            let name = pl["name"].as_str().unwrap_or("").to_string();
            let ph = &pl["physics"];
            let state = Truth {
                pos: vec3(&ph["location"]),
                vel: vec3(&ph["velocity"]),
                rot: matrix(&ph["rotation"]),
                ang: vec3(&ph["angular_velocity"]),
                air_state: pl["air_state"].as_u64().unwrap_or(0),
                demolished: pl["demolished_timeout"].as_f64().unwrap_or(-1.0) >= 0.0,
                has_dodged: pl["has_dodged"].as_bool().unwrap_or(false),
                boost: pl["boost"].as_f64().unwrap_or(0.0) as f32,
                has_jumped: pl["has_jumped"].as_bool().unwrap_or(false),
                has_double_jumped: pl["has_double_jumped"].as_bool().unwrap_or(false),
                dodge_timeout: pl["dodge_timeout"].as_f64().unwrap_or(-1.0) as f32,
                supersonic: pl["is_supersonic"].as_bool().unwrap_or(false),
                dodge_dir: (
                    pl["dodge_dir"]["x"].as_f64().unwrap_or(0.0) as f32,
                    pl["dodge_dir"]["y"].as_f64().unwrap_or(0.0) as f32,
                ),
                input: {
                    let li = &pl["last_input"];
                    let num = |k: &str| li[k].as_f64().unwrap_or(0.0) as f32;
                    let flag = |k: &str| f32::from(u8::from(li[k].as_bool().unwrap_or(false)));
                    [
                        num("throttle"),
                        num("steer"),
                        num("pitch"),
                        num("yaw"),
                        num("roll"),
                        flag("jump"),
                        flag("boost"),
                        flag("handbrake"),
                    ]
                },
            };
            by_position
                .entry((name.clone(), key(state.pos)))
                .or_default()
                .push(frame);
            entry.insert(name, state);
        }
        let bp = &packet["balls"][0]["physics"];
        let ball = vec3(&bp["location"]);
        entry.insert(
            "BALL#".to_string(),
            Truth {
                pos: ball,
                vel: vec3(&bp["velocity"]),
                rot: matrix(&bp["rotation"]),
                ang: vec3(&bp["angular_velocity"]),
                air_state: 0,
                demolished: false,
                has_dodged: false,
                boost: 0.0,
                has_jumped: false,
                has_double_jumped: false,
                dodge_timeout: -1.0,
                supersonic: false,
                dodge_dir: (0.0, 0.0),
                input: [0.0; 8],
            },
        );
        by_position
            .entry(("BALL#".to_string(), key(ball)))
            .or_default()
            .push(frame);
    }

    let replay = boxcars::ParserBuilder::new(&fs::read(&replay_path)?)
        .must_parse_network_data()
        .parse()?;
    let mut observed = extract(&replay).ok_or("no network frames")?;
    // `--thin K`: only every K-th frame keeps its car body packets (the others repeat the last kept body
    // with its old stamps), to imitate a sparser cadence and to get frames between packets.
    if let Some(at) = args.iter().position(|a| a == "--thin") {
        let k: usize = args
            .get(at + 1)
            .and_then(|v| v.parse().ok())
            .ok_or("--thin K")?;
        let mut last: HashMap<(i32, usize), Body> = HashMap::new();
        for (f, frame) in observed.frames.iter_mut().enumerate() {
            for car in &mut frame.cars {
                let key = (car.actor_id, car.actor_created_frame);
                match last.get(&key) {
                    Some(body) if f % k != 0 => car.body = body.clone(),
                    _ => {
                        last.insert(key, car.body.clone());
                    }
                }
            }
        }
    }
    let names: HashMap<String, String> = observed
        .frames
        .iter()
        .flat_map(|f| f.players.iter())
        .filter_map(|p| p.name.clone().map(|n| (p.key.clone(), n)))
        .collect();

    type Tweak = fn(&mut ConvertOptions);
    let variants: [(&str, Tweak); 4] = [
        ("all fits", |_| {}),
        ("no dodge first-packet inference", |o| {
            o.infer_dodge_first_packet_tick = false;
            o.defer_dodge_past_next_packet = false;
        }),
        ("no timing fits (ground, jump, dodge, cancel)", |o| {
            o.fit_ground_control_timing = false;
            o.fit_jump_timing = false;
            o.infer_dodge_start = false;
            o.infer_flip_cancel = false;
        }),
        ("no fits, no lookahead controls", |o| {
            o.fit_ground_control_timing = false;
            o.fit_jump_timing = false;
            o.infer_dodge_start = false;
            o.infer_flip_cancel = false;
            o.lookahead_ground_controls = false;
        }),
    ];
    let score = |label: &str, options: ConvertOptions| -> Result<(), Box<dyn Error>> {
        let mut options = options;
        options.block_sim_pad_pickups |= env::var_os("BLOCK_PADS").is_some();
        options.boost_pickup_lookahead |= env::var_os("BOOST_LOOKAHEAD").is_some();
        if env::var_os("NO_FIT_NEXT").is_some() {
            options.fit_on_next_packet = false;
        }
        if env::var_os("NO_AIR_BVP").is_some() {
            options.air_bvp = false;
        }
        if env::var_os("NO_FLAGS").is_some() {
            options.flags_from_counters = false;
        }
        let output = convert_observations(observed.clone(), &options)?;
        let slot_name: HashMap<usize, String> = output
            .car_slots
            .iter()
            .filter_map(|s| names.get(&s.player_key).map(|n| (s.slot, n.clone())))
            .collect();
        let frames = &output.observations.frames;
        if std::env::var_os("AIR_BVP").is_some() && label.starts_with("all fits") {
            println!(
                "  air BVP: {} airborne packets planned, {} refused",
                output.diagnostics.air_bvp_planned, output.diagnostics.air_bvp_refused
            );
            println!(
                "  air BVP refusal reasons (0 flipping, 1 no rotation, 2 low a, 3 inactive, 4 inactive/withheld span, 5 dodge in span, 6 no next packet, 7 low b, 8 span length, 9 ground, 10 no solution): {:?}",
                replay_to_rocketsim::conversion::AIR_BVP_REFUSALS
                    .iter()
                    .map(|c| c.load(std::sync::atomic::Ordering::Relaxed))
                    .collect::<Vec<_>>()
            );
        }
        // Matched fresh packets: (frame index, offset between converter timeline and server ticks).
        let mut matched: Vec<(usize, i64)> = Vec::new();
        for (f, frame) in frames.iter().enumerate() {
            for car in &frame.cars {
                let (Some(name), Some(p)) = (
                    car.player_key.as_ref().and_then(|k| names.get(k)),
                    car.body.position.as_ref().filter(|p| p.frame == f),
                ) else {
                    continue;
                };
                let Some(hits) = by_position.get(&(name.clone(), key(Vec3A::from_array(p.value))))
                else {
                    continue;
                };
                if hits.len() != 1 {
                    continue;
                }
                let lag = output.frames[f]
                    .packet_lags
                    .iter()
                    .find(|l| {
                        l.actor_id == Some(car.actor_id)
                            && (l.source == "chain" || l.source == "dodge_fit")
                    })
                    .map(|l| l.ticks as i64);
                let Some(lag) = lag else { continue };
                matched.push((
                    f,
                    output.frames[f].timeline_tick as i64 - hits[0] as i64 - lag,
                ));
            }
        }
        if matched.len() < 50 {
            println!("{label}: too few matched packets ({})", matched.len());
            return Ok(());
        }
        // Running mode of the offset over +-100 matched packets.
        let offset_at = |f: usize| -> i64 {
            let i = matched.partition_point(|m| m.0 < f).min(matched.len() - 1);
            let (lo, hi) = (i.saturating_sub(100), (i + 101).min(matched.len()));
            let mut counts: BTreeMap<i64, usize> = BTreeMap::new();
            for m in &matched[lo..hi] {
                *counts.entry(m.1).or_default() += 1;
            }
            counts
                .into_iter()
                .max_by_key(|(_, c)| *c)
                .map(|(o, _)| o)
                .unwrap_or(0)
        };
        let mut rows: BTreeMap<String, Rows> = BTreeMap::new();
        let mut covered: HashMap<String, Vec<(i64, i64)>> = HashMap::new();
        for (f, converted) in output.frames.iter().enumerate() {
            for e in converted.fitted_inputs.iter().filter(|e| e.kind == "air") {
                if let Some(name) = slot_name.get(&e.slot) {
                    let start = e.tick as i64 - offset_at(f);
                    covered
                        .entry(name.clone())
                        .or_default()
                        .push((start, start + e.cancel as i64));
                }
            }
        }
        let mut ball_rows: BTreeMap<String, Rows> = BTreeMap::new();
        // Car-ball consistency: the best single true tick for the exported car and ball of a frame, and the
        // relative-position error there (zero when both sit on one tick, whatever the absolute offset).
        let mut rel_err: Vec<f32> = Vec::new();
        let mut rel_tick: Vec<f32> = Vec::new();
        // Full car state (not only physics): counts of mismatches with the truth, by fresh packet or not.
        let mut flags: BTreeMap<&str, (usize, usize)> = BTreeMap::new(); // name -> (n, mismatches)
        // Exported controls against the input the server applied: per control, the absolute errors
        // (frames where it can matter: the air controls only while airborne).
        let control_names = ["throttle", "steer", "pitch", "yaw", "roll", "jump", "boost", "handbrake"];
        let mut control_err: Vec<Vec<f32>> = vec![Vec::new(); 16]; // [control][ground] then [control][air] at 8 + control
        let mut boost_err: Vec<f32> = Vec::new();
        let mut boost_err_fresh: Vec<f32> = Vec::new();
        let mut timeout_err: Vec<f32> = Vec::new();
        let mut confusion: BTreeMap<String, usize> = BTreeMap::new();
        let mut positives: BTreeMap<&str, usize> = BTreeMap::new();
        let mut last_fresh: HashMap<usize, usize> = HashMap::new();
        let mut scored = 0usize;
        for (f, converted) in output.frames.iter().enumerate() {
            let active = frames[f]
                .game_state
                .as_ref()
                .is_some_and(|g| g.value == "Active");
            let offset = offset_at(f);
            if active {
                let server_tick = converted.timeline_tick as i64 - offset;
                // Contact consistency for the car nearest the ball.
                let ball_sim = &converted.state.ball.phys;
                if let Some((slot, (_, car_sim))) = converted
                    .state
                    .cars
                    .iter()
                    .enumerate()
                    .map(|(i, c)| (i, c))
                    .filter(|(_, c)| (c.1.phys.pos - ball_sim.pos).length() < 350.0)
                    .min_by(|a, b| (a.1.1.phys.pos - ball_sim.pos).length().total_cmp(&(b.1.1.phys.pos - ball_sim.pos).length()))
                {
                    if let Some(name) = slot_name.get(&slot) {
                        let exported = car_sim.phys.pos - ball_sim.pos;
                        let mut best = (f32::INFINITY, 0i64);
                        for tau in server_tick - 20..=server_tick + 20 {
                            let Some(players) = truth.get(&(tau.max(0) as u64)) else { continue };
                            let (Some(tc), Some(tb)) = (players.get(name), players.get("BALL#")) else { continue };
                            let e = (exported - (tc.pos - tb.pos)).length();
                            if e < best.0 {
                                best = (e, tau - server_tick);
                            }
                        }
                        if best.0.is_finite() {
                            rel_err.push(best.0);
                            rel_tick.push(best.1 as f32);
                        }
                    }
                }
                if let Some(players) = truth.get(&(server_tick.max(0) as u64)) {
                    if let Some(tb) = players.get("BALL#") {
                        let near = players
                            .iter()
                            .any(|(n, t)| n != "BALL#" && (t.pos - tb.pos).length() < 300.0);
                        let fresh_ball = frames[f]
                            .ball
                            .as_ref()
                            .and_then(|b| b.position.as_ref())
                            .is_some_and(|p| p.frame == f);
                        let b = &converted.state.ball.phys;
                        for group in [
                            "all".to_string(),
                            if near {
                                "ball near a car (<300 UU)".to_string()
                            } else {
                                "ball away from cars".to_string()
                            },
                            if fresh_ball {
                                "fresh ball packet".to_string()
                            } else {
                                "no fresh ball packet".to_string()
                            },
                        ] {
                            let r = ball_rows.entry(group).or_default();
                            r.pos.push((b.pos - tb.pos).length());
                            r.vel.push((b.vel - tb.vel).length());
                            r.rot.push(rotation_error(b.rot_mat, tb.rot));
                            r.ang.push((b.ang_vel - tb.ang).length());
                        }
                    }
                }
            }
            for car in &frames[f].cars {
                let Some(name) = car.player_key.as_ref().and_then(|k| names.get(k)) else {
                    continue;
                };
                let Some((&slot, _)) = slot_name.iter().find(|(_, n)| *n == name) else {
                    continue;
                };
                let fresh = car.body.position.as_ref().is_some_and(|p| p.frame == f);
                if fresh {
                    last_fresh.insert(slot, f);
                }
                if !active {
                    continue;
                }
                let Some((_, sim)) = converted.state.cars.get(slot) else {
                    continue;
                };
                let server_tick = converted.timeline_tick as i64 - offset;
                let Some(t) = truth
                    .get(&(server_tick.max(0) as u64))
                    .and_then(|m| m.get(name))
                else {
                    continue;
                };
                if t.demolished || sim.is_demoed {
                    continue;
                }
                scored += 1;
                if std::env::var_os("DIR_CHECK").is_some()
                    && label.starts_with("all fits")
                    && t.air_state == 3
                    && sim.is_flipping
                {
                    // RocketSim's flip_rel_torque is (-dir.y, dir.x); the truth's dodge_dir is (x forward, y right).
                    let expect = glam::Vec2::new(-t.dodge_dir.1, t.dodge_dir.0);
                    let got = glam::Vec2::new(sim.flip_rel_torque.x, sim.flip_rel_torque.y);
                    let angle = expect.angle_to(got).to_degrees().abs();
                    println!(
                        "DIR frame {f} {name} angle between sim and true flip torque {angle:.1} deg (true dodge_dir {:?} sim torque {:?})",
                        t.dodge_dir,
                        (got.x, got.y)
                    );
                }
                if std::env::var_os("BOOST_TRACE").is_some()
                    && label.starts_with("all fits")
                    && name.starts_with("Kiyo")
                    && (sim.boost - t.boost).abs() > 8.0
                {
                    let next = truth
                        .get(&((server_tick + 1).max(0) as u64))
                        .and_then(|m| m.get(name))
                        .map_or(t.boost, |n| n.boost);
                    println!(
                        "BOOST frame {f} tick {server_tick} truth {:.1} (boosting {}) sim {:.1} (is_boosting {}, ctl.boost {}) replay amount {:?} boost counter {:?}",
                        t.boost,
                        next < t.boost - 0.05,
                        sim.boost,
                        sim.is_boosting,
                        sim.controls.boost,
                        car.boost_raw.as_ref().map(|b| (b.value, b.frame)),
                        car.inputs
                            .boost_active_raw
                            .as_ref()
                            .map(|b| (b.value, b.frame)),
                    );
                }
                if std::env::var_os("FLAG_TRACE").is_some()
                    && label.starts_with("all fits")
                    && sim.has_flipped != t.has_dodged
                    && name.starts_with("Kiyo")
                {
                    println!(
                        "FLAG frame {f} tick {server_tick} truth air {} has_dodged {} | sim has_flipped {} is_flipping {} flip_time {:.3} has_jumped {} ground {} dodge counter {:?} | fresh {fresh}",
                        t.air_state,
                        t.has_dodged,
                        sim.has_flipped,
                        sim.is_flipping,
                        sim.flip_time,
                        sim.has_jumped,
                        sim.is_on_ground,
                        car.inputs
                            .dodge_active_raw
                            .as_ref()
                            .map(|d| (d.value, d.frame)),
                    );
                }
                if std::env::var_os("DJ_TRACE").is_some() && label.starts_with("all fits") {
                    let prev_air = truth
                        .get(&((server_tick - 1).max(0) as u64))
                        .and_then(|m| m.get(name))
                        .map_or(0, |p| p.air_state);
                    if t.air_state == 2 || prev_air == 2 {
                        println!(
                            "DJ frame {f} tick {server_tick} truth air {} z {:.1} vz {:.1} | sim vz {:.1} has_dbl {} ground {} jump_ctl {} prev_jump_ctl {} | fresh {fresh}",
                            t.air_state,
                            t.pos.z,
                            t.vel.z,
                            sim.phys.vel.z,
                            sim.has_double_jumped,
                            sim.is_on_ground,
                            sim.controls.jump,
                            sim.prev_controls.jump
                        );
                    }
                }
                {
                    let mut tally = |name: &'static str, bad: bool| {
                        let e = flags.entry(name).or_default();
                        e.0 += 1;
                        e.1 += usize::from(bad);
                    };
                    let truth_ground = t.air_state == 0;
                    tally("is_on_ground", sim.is_on_ground != truth_ground);
                    tally("has_jumped", sim.has_jumped != t.has_jumped);
                    tally(
                        "has_double_jumped",
                        sim.has_double_jumped != t.has_double_jumped,
                    );
                    for (name, v) in [
                        ("has_jumped", t.has_jumped),
                        ("has_double_jumped", t.has_double_jumped),
                        ("has_flipped (truth has_dodged)", t.has_dodged),
                        ("is_flipping (truth Dodging)", t.air_state == 3),
                    ] {
                        *positives.entry(name).or_default() += usize::from(v);
                    }
                    tally(
                        "has_flipped (truth has_dodged)",
                        sim.has_flipped != t.has_dodged,
                    );
                    tally(
                        "is_flipping (truth Dodging)",
                        sim.is_flipping != (t.air_state == 3),
                    );
                    tally(
                        "is_jumping (truth Jumping)",
                        sim.is_jumping != (t.air_state == 1),
                    );
                    tally("is_supersonic", sim.is_supersonic != t.supersonic);
                    if std::env::var_os("CONFUSION").is_some() {
                        for (flag, a, b) in [
                            ("has_jumped", sim.has_jumped, t.has_jumped),
                            (
                                "has_double_jumped",
                                sim.has_double_jumped,
                                t.has_double_jumped,
                            ),
                            ("has_flipped/has_dodged", sim.has_flipped, t.has_dodged),
                        ] {
                            if a != b {
                                *confusion
                                    .entry(format!("{flag}: sim {a} truth {b}, truth air_state {}, fresh {fresh}", t.air_state))
                                    .or_default() += 1;
                            }
                        }
                    }
                    // Can the car still flip? RocketSim: jumped, no flip yet, within 1.25 s of the jump.
                    let sim_can_flip = sim.has_jumped
                        && !sim.has_double_jumped
                        && !sim.has_flipped
                        && sim.air_time_since_jump < 1.25
                        && !sim.is_on_ground;
                    let truth_can_flip = t.has_jumped
                        && !t.has_double_jumped
                        && !t.has_dodged
                        && t.dodge_timeout > 0.0
                        && t.air_state != 0;
                    tally(
                        "can flip now (jumped, no flip, within 1.25 s, airborne)",
                        sim_can_flip != truth_can_flip,
                    );
                    let exported = [
                        sim.controls.throttle,
                        sim.controls.steer,
                        sim.controls.pitch,
                        sim.controls.yaw,
                        sim.controls.roll,
                        f32::from(u8::from(sim.controls.jump)),
                        f32::from(u8::from(sim.controls.boost)),
                        f32::from(u8::from(sim.controls.handbrake)),
                    ];
                    if std::env::var_os("STEER_TRACE").is_some()
                        && label.starts_with("all fits")
                        && t.air_state == 0
                        && (sim.controls.steer - t.input[1]).abs() > 0.5
                    {
                        let observed = |g: usize| {
                            frames
                                .get(g)
                                .and_then(|fr| fr.cars.iter().find(|c| c.player_key == car.player_key))
                                .and_then(|c| c.inputs.steer.as_ref())
                                .map(|v| (v.value, v.frame))
                        };
                        let around: Vec<String> = (-6i64..=6)
                            .map(|d| {
                                truth
                                    .get(&((server_tick + d).max(0) as u64))
                                    .and_then(|m| m.get(name))
                                    .map_or("?".to_string(), |x| format!("{:.1}", x.input[1]))
                            })
                            .collect();
                        println!(
                            "STEER frame {f} {name} tick {server_tick} fresh {fresh} sim {:.2} truth {:.2} | observed f-1 {:?} f {:?} f+1 {:?} f+2 {:?} | truth steer ticks -6..+6: {}",
                            sim.controls.steer,
                            t.input[1],
                            observed(f.wrapping_sub(1)),
                            observed(f),
                            observed(f + 1),
                            observed(f + 2),
                            around.join(" ")
                        );
                    }
                    for (i, (&x, &y)) in exported.iter().zip(&t.input).enumerate() {
                        if (2..=4).contains(&i) && t.air_state == 0 {
                            continue;
                        }
                        control_err[i + if t.air_state == 0 { 0 } else { 8 }].push((x - y).abs());
                    }
                    let e = (sim.boost - t.boost).abs();
                    boost_err.push(e);
                    if fresh {
                        boost_err_fresh.push(e);
                    }
                    if t.dodge_timeout >= 0.0
                        && sim.has_jumped
                        && !sim.has_flipped
                        && !sim.is_on_ground
                    {
                        timeout_err
                            .push(((1.25 - sim.air_time_since_jump) - t.dodge_timeout).abs());
                    }
                }
                let since = last_fresh.get(&slot).map_or(99, |&l| f - l);
                if std::env::var_os("AIR_WORST").is_some()
                    && label.starts_with("all fits")
                    && !fresh
                    && (t.air_state == 4
                        || (t.air_state == 3 && std::env::var_os("WORST_FLIP").is_some()))
                    && !covered.get(name).is_some_and(|v| {
                        v.iter().any(|&(a, b)| server_tick > a && server_tick <= b)
                    })
                {
                    let err = rotation_error(sim.phys.rot_mat, t.rot);
                    if err > 6.0 {
                        let ball_distance = truth
                            .get(&(server_tick.max(0) as u64))
                            .and_then(|m| m.get("BALL#"))
                            .map_or(0.0, |b| (b.pos - t.pos).length());
                        println!(
                            "WORST frame {f} {name} rot err {err:.1} deg z {:.0} speed {:.0} ang {:.2} ball {:.0} has_jumped {} dbl {} dodged {} since {} | sim flipping {} has_flipped {}",
                            t.pos.z,
                            t.vel.length(),
                            t.ang.length(),
                            ball_distance,
                            t.has_jumped,
                            t.has_double_jumped,
                            t.has_dodged,
                            last_fresh.get(&slot).map_or(99, |&l| f - l),
                            sim.is_flipping,
                            sim.has_flipped
                        );
                    }
                }
                let behaviour = match t.air_state {
                    0 => "ground",
                    1 => "jump",
                    2 => "double jump",
                    3 => "flip",
                    _ => "air",
                };
                let groups = [
                    "all".to_string(),
                    if fresh {
                        "frames with a fresh packet".to_string()
                    } else {
                        "frames without a fresh packet".to_string()
                    },
                    format!("{behaviour}, no fresh packet"),
                    if t.air_state == 0 && !fresh {
                        let ball_d = truth
                            .get(&(server_tick.max(0) as u64))
                            .and_then(|m| m.get("BALL#"))
                            .map_or(1e9, |b| (b.pos - t.pos).length());
                        let car_d = truth.get(&(server_tick.max(0) as u64)).map_or(1e9, |m| {
                            m.iter()
                                .filter(|(n, _)| {
                                    n.as_str() != "BALL#" && n.as_str() != name.as_str()
                                })
                                .map(|(_, o)| (o.pos - t.pos).length())
                                .fold(1e9, f32::min)
                        });
                        let boosting = truth
                            .get(&((server_tick + 1).max(0) as u64))
                            .and_then(|m| m.get(name))
                            .is_some_and(|n| n.boost < t.boost - 0.05);
                        format!(
                            "ground: {}{}{}",
                            if ball_d < 300.0 { "near ball " } else { "" },
                            if car_d < 300.0 { "near car " } else { "" },
                            if t.pos.z > 25.0 {
                                "wall/ramp "
                            } else if boosting {
                                "boosting "
                            } else {
                                "plain"
                            }
                        )
                    } else {
                        "other".to_string()
                    },
                    format!(
                        "{behaviour}, no fresh packet, {}",
                        if covered.get(name).is_some_and(|v| v
                            .iter()
                            .any(|&(a, b)| server_tick > a && server_tick <= b))
                        {
                            "air BVP interval"
                        } else {
                            "no air BVP interval"
                        }
                    ),
                    if t.air_state == 2 {
                        "truth DoubleJumping (any packet)".to_string()
                    } else {
                        "other".to_string()
                    },
                    format!("{behaviour}: {since} frame(s) since the last fresh packet")
                        .replace("99 frame(s)", "no packet seen"),
                ];
                for (i, group) in groups.into_iter().enumerate() {
                    if i == 3 && t.air_state != 0 {
                        continue;
                    }
                    if (i == 2 || i == 3 || i == 4) && fresh {
                        continue;
                    }
                    if i == 6 && t.air_state != 2 {
                        continue;
                    }
                    let r = rows.entry(group).or_default();
                    r.pos.push((sim.phys.pos - t.pos).length());
                    r.vel.push((sim.phys.vel - t.vel).length());
                    r.rot.push(rotation_error(sim.phys.rot_mat, t.rot));
                    r.ang.push((sim.phys.ang_vel - t.ang).length());
                }
            }
        }
        // Fitted jump and dodge presses against the true ones (server ticks).
        let mut press: BTreeMap<&str, Vec<f32>> = BTreeMap::new();
        let mut events: HashMap<(String, bool), Vec<u64>> = HashMap::new();
        for (&fn_, players) in &truth {
            for (name, now) in players {
                if let Some(before) = truth.get(&fn_.wrapping_sub(1)).and_then(|m| m.get(name)) {
                    if now.has_dodged && !before.has_dodged {
                        events.entry((name.clone(), true)).or_default().push(fn_);
                    }
                    if now.air_state == 1 && before.air_state != 1 {
                        events.entry((name.clone(), false)).or_default().push(fn_);
                    }
                }
            }
        }
        for (f, converted) in output.frames.iter().enumerate() {
            for e in &converted.fitted_inputs {
                let Some(name) = slot_name.get(&e.slot) else {
                    continue;
                };
                let server = e.tick as i64 - offset_at(f);
                let dodge = e.kind == "dodge";
                let Some(list) = events.get(&(name.clone(), dodge)) else {
                    continue;
                };
                if let Some(best) = list.iter().min_by_key(|&&fn_| (fn_ as i64 - server).abs()) {
                    if (*best as i64 - server).abs() <= 30 {
                        press
                            .entry(if dodge { "dodge" } else { "jump" })
                            .or_default()
                            .push((server - *best as i64) as f32);
                    }
                }
            }
        }
        for (kind, values) in press.iter_mut() {
            let mut abs: Vec<f32> = values.iter().map(|v| v.abs()).collect();
            println!(
                "  fitted {kind} press minus the true press tick (server ticks): n {} p10/p50/p90 {:.0}/{:.0}/{:.0}, |.| p50/p90 {:.0}/{:.0}",
                values.len(),
                quantile(values, 0.1),
                quantile(values, 0.5),
                quantile(values, 0.9),
                quantile(&mut abs, 0.5),
                quantile(&mut abs, 0.9),
            );
        }
        if !rel_err.is_empty() {
            let mut ticks = rel_tick.clone();
            println!(
                "  car-ball consistency over {} frames with a car within 350 UU of the ball: relative position error at the best common true tick p50/p90/p99 {:.1}/{:.1}/{:.1} UU; that tick minus the frame's tick p10/p50/p90 {:.0}/{:.0}/{:.0}",
                rel_err.len(),
                quantile(&mut rel_err.clone(), 0.5), quantile(&mut rel_err.clone(), 0.9), quantile(&mut rel_err, 0.99),
                quantile(&mut ticks.clone(), 0.1), quantile(&mut ticks.clone(), 0.5), quantile(&mut ticks, 0.9)
            );
        }
        if label.starts_with("all fits") {
            println!("  full car state against the truth ({scored} scored frames):");
            for (name, (n, bad)) in &flags {
                let pos = positives.get(name).copied().unwrap_or(0);
                println!(
                    "    {name:<60} mismatches {bad:>6} of {n:>6} ({:.2}%); truth true in {pos} frames",
                    100.0 * *bad as f64 / (*n).max(1) as f64
                );
            }
            println!("    exported controls against the server's applied input (|error|; pitch, yaw, roll while airborne):");
            for (i, name) in control_names.iter().enumerate() {
                for (part, label) in [(0, "on the ground"), (8, "airborne")] {
                    let errs = &mut control_err[i + part];
                    if errs.is_empty() {
                        continue;
                    }
                    let n = errs.len();
                    let mean = errs.iter().sum::<f32>() / n as f32;
                    let off = errs.iter().filter(|e| **e > 0.1).count();
                    println!(
                        "      {name:<10} {label:<13} n {n:>6} mean |err| {mean:.3}, |err| > 0.1 in {:.2}% (p99 {:.2})",
                        100.0 * off as f64 / n as f64,
                        quantile(errs, 0.99)
                    );
                }
            }
            println!(
                "    boost error (units of 0..100) p50/p90/p99 {:.2}/{:.2}/{:.2}; on frames with a fresh packet {:.2}/{:.2}/{:.2}",
                quantile(&mut boost_err.clone(), 0.5),
                quantile(&mut boost_err.clone(), 0.9),
                quantile(&mut boost_err, 0.99),
                quantile(&mut boost_err_fresh.clone(), 0.5),
                quantile(&mut boost_err_fresh.clone(), 0.9),
                quantile(&mut boost_err_fresh, 0.99),
            );
            println!(
                "    flip-window time left (1.25 s - air time since jump) vs dodge_timeout, airborne jumped cars: n {} |error| p50/p90/p99 {:.3}/{:.3}/{:.3} s",
                timeout_err.len(),
                quantile(&mut timeout_err.clone(), 0.5),
                quantile(&mut timeout_err.clone(), 0.9),
                quantile(&mut timeout_err, 0.99),
            );
        }
        let mut conf: Vec<_> = confusion.iter().collect();
        conf.sort_by_key(|(_, n)| std::cmp::Reverse(**n));
        for (k, n) in conf.iter().take(14) {
            println!("    mismatch {k}: {n}");
        }
        for (group, r) in ball_rows.iter_mut() {
            println!(
                "  BALL {:<30} {:>6} | pos {:>5.2}/{:>5.1}/{:>5.0} UU | vel {:>6.1}/{:>6.0} UU/s",
                group,
                r.pos.len(),
                quantile(&mut r.pos, 0.5),
                quantile(&mut r.pos, 0.9),
                quantile(&mut r.pos, 0.99),
                quantile(&mut r.vel, 0.5),
                quantile(&mut r.vel, 0.9),
            );
        }
        println!(
            "\n{label}: {} matched fresh packets, {scored} scored car frames",
            matched.len()
        );
        println!(
            "{:<44} {:>6} | {:>17} | {:>15} | {:>13} | {:>13}",
            "group",
            "n",
            "pos UU p50/90/99",
            "vel UU/s p50/90",
            "rot deg p50/90",
            "ang rad/s p50/90"
        );
        for (group, r) in rows.iter_mut() {
            if r.pos.len() < 200 && group != "all" {
                continue;
            }
            println!(
                "{:<44} {:>6} | {:>5.2}/{:>5.1}/{:>5.0} | {:>6.1}/{:>6.0} | {:>5.2}/{:>5.2} | {:>5.3}/{:>5.3}",
                group,
                r.pos.len(),
                quantile(&mut r.pos, 0.5),
                quantile(&mut r.pos, 0.9),
                quantile(&mut r.pos, 0.99),
                quantile(&mut r.vel, 0.5),
                quantile(&mut r.vel, 0.9),
                quantile(&mut r.rot, 0.5),
                quantile(&mut r.rot, 0.9),
                quantile(&mut r.ang, 0.5),
                quantile(&mut r.ang, 0.9),
            );
        }
        Ok(())
    };
    for (label, tweak) in variants {
        let mut options = ConvertOptions::default();
        options.zero_packet_lag = zero_lag;
        options.ball_car_lag_offset = std::env::var("LAG_MU").ok().and_then(|v| v.parse().ok());
        options.ball_hit_chains = std::env::var_os("NO_BALL_HITS").is_none();
        options.estimate_ball_car_lag_offset = std::env::var_os("NO_EST_MU").is_none();
        options.detect_lag_free_replays = std::env::var_os("NO_LAG_FREE").is_none();
        options.packet_interval_control_rule = std::env::var_os("PACKET_CONTROL_RULE").is_some();
        tweak(&mut options);
        score(label, options)?;
    }
    // Oracle lags: the true lag of every packet that matches a server tick exactly, taken as its offset
    // against the running minimum of the offsets (a lag is at least 0 and its minimum over a window is
    // about 0). An upper bound on what better lag inference could give.
    let first_time = f64::from(observed.frames[0].time);
    let tick_of =
        |f: usize| ((f64::from(observed.frames[f].time) - first_time) * 120.0).round() as i64;
    let mut offsets: Vec<(usize, Option<i32>, i64)> = Vec::new(); // (frame, actor or None for the ball, offset)
    for (f, frame) in observed.frames.iter().enumerate() {
        for car in &frame.cars {
            let (Some(name), Some(p)) = (
                car.player_key.as_ref().and_then(|k| names.get(k)),
                car.body.position.as_ref().filter(|p| p.frame == f),
            ) else {
                continue;
            };
            if let Some(hits) = by_position.get(&(name.clone(), key(Vec3A::from_array(p.value)))) {
                if hits.len() == 1 {
                    offsets.push((f, Some(car.actor_id), tick_of(f) - hits[0] as i64));
                }
            }
        }
        if let Some(p) = frame
            .ball
            .as_ref()
            .and_then(|b| b.position.as_ref())
            .filter(|p| p.frame == f)
        {
            if let Some(hits) =
                by_position.get(&("BALL#".to_string(), key(Vec3A::from_array(p.value))))
            {
                if hits.len() == 1 {
                    offsets.push((f, None, tick_of(f) - hits[0] as i64));
                }
            }
        }
    }
    let mut oracle = replay_to_rocketsim::conversion::PacketLags {
        ball: vec![None; observed.frames.len()],
        cars: vec![None; observed.frames.len()],
        car_actor: HashMap::new(),
    };
    let mut per_frame: HashMap<usize, Vec<f32>> = HashMap::new();
    for (i, &(f, actor, off)) in offsets.iter().enumerate() {
        let (lo, hi) = (i.saturating_sub(300), (i + 301).min(offsets.len()));
        let floor = offsets[lo..hi].iter().map(|o| o.2).min().unwrap_or(off);
        let lag = (off - floor).max(0) as f32;
        match actor {
            Some(a) => {
                let created = observed.frames[f]
                    .cars
                    .iter()
                    .find(|c| c.actor_id == a)
                    .map_or(0, |c| c.actor_created_frame);
                oracle.car_actor.insert((a, created, f), lag);
                per_frame.entry(f).or_default().push(lag);
            }
            None => oracle.ball[f] = Some(lag),
        }
    }
    for (f, mut lags) in per_frame {
        lags.sort_by(|a, b| a.total_cmp(b));
        oracle.cars[f] = Some(lags[lags.len() / 2]);
    }
    let mut options = ConvertOptions::default();
    options.external_packet_lags = Some(std::sync::Arc::new(oracle));
    score(
        "ORACLE packet lags (true lags of the exactly matched packets), all fits",
        options.clone(),
    )?;
    options.fit_ground_control_timing = false;
    options.fit_jump_timing = false;
    options.infer_dodge_start = false;
    options.infer_flip_cancel = false;
    options.lookahead_ground_controls = false;
    score(
        "ORACLE packet lags, no fits, no lookahead controls",
        options,
    )?;
    Ok(())
}
