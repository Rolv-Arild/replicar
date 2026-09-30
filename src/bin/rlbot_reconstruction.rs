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
use replay_to_rocketsim::observations::extract;
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
            };
            by_position
                .entry((name.clone(), key(state.pos)))
                .or_default()
                .push(frame);
            entry.insert(name, state);
        }
        let ball = vec3(&packet["balls"][0]["physics"]["location"]);
        by_position
            .entry(("BALL#".to_string(), key(ball)))
            .or_default()
            .push(frame);
    }

    let replay = boxcars::ParserBuilder::new(&fs::read(&replay_path)?)
        .must_parse_network_data()
        .parse()?;
    let observed = extract(&replay).ok_or("no network frames")?;
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
        let output = convert_observations(observed.clone(), &options)?;
        let slot_name: HashMap<usize, String> = output
            .car_slots
            .iter()
            .filter_map(|s| names.get(&s.player_key).map(|n| (s.slot, n.clone())))
            .collect();
        let frames = &output.observations.frames;
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
        let mut last_fresh: HashMap<usize, usize> = HashMap::new();
        let mut scored = 0usize;
        for (f, converted) in output.frames.iter().enumerate() {
            let active = frames[f]
                .game_state
                .as_ref()
                .is_some_and(|g| g.value == "Active");
            let offset = offset_at(f);
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
                let since = last_fresh.get(&slot).map_or(99, |&l| f - l);
                let behaviour = match t.air_state {
                    0 => "ground",
                    1 | 2 => "jump",
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
                    format!("{since} frame(s) since the last fresh packet")
                        .replace("99 frame(s)", "no packet seen"),
                ];
                for (i, group) in groups.into_iter().enumerate() {
                    if i == 2 && fresh {
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
