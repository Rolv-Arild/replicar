//! End-to-end reconstruction test against a BakkesMod state dump (true car state at every frame).
//!
//! The replay of the dump has a fresh car packet in every frame and no replication lag. To imitate
//! online cadence the car packets are thinned: only every K-th frame keeps its car body fields, the
//! others repeat the last kept body with its old stamps (so they are not fresh). The thinned replay is
//! converted (packet lag zero, since the dump's replay has none) and the exported car state of every
//! frame is compared with the dump's truth, split by whether the frame kept its packet, by frames since
//! the last packet, and by the car's behaviour in the dump. K = 1 is the unthinned replay.
//!
//! usage: dump_reconstruction <replay> <dump.json> [--player name] [K...]

use std::collections::{BTreeMap, HashMap};
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

use glam::{Mat3A, Vec3A};
use replay_to_rocketsim::conversion::{ConvertOptions, convert_observations};
use replay_to_rocketsim::observations::{Body, extract};
use serde_json::Value;

fn vec3(v: &Value, keys: [&str; 3]) -> Vec3A {
    Vec3A::new(
        v[keys[0]].as_f64().unwrap_or(0.0) as f32,
        v[keys[1]].as_f64().unwrap_or(0.0) as f32,
        v[keys[2]].as_f64().unwrap_or(0.0) as f32,
    )
}

/// Unreal rotator (1/65536 turn) to the rotation matrix with columns forward, right, up.
fn matrix(v: &Value) -> Mat3A {
    let k = std::f32::consts::PI / 32768.0;
    let (p, y, r) = (
        v["Pitch"].as_f64().unwrap_or(0.0) as f32 * k,
        v["Yaw"].as_f64().unwrap_or(0.0) as f32 * k,
        v["Roll"].as_f64().unwrap_or(0.0) as f32 * k,
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

#[derive(Default)]
struct Rows {
    pos: Vec<f32>,
    vel: Vec<f32>,
    rot: Vec<f32>,
    ang: Vec<f32>,
}

fn print_rows(title: &str, rows: &mut BTreeMap<String, Rows>) {
    println!("\n{title}");
    println!(
        "{:<34} {:>5} | {:>16} | {:>16} | {:>14} | {:>14}",
        "group", "n", "pos UU p50/90/99", "vel UU/s p50/90", "rot deg p50/90", "ang rad/s p50/90"
    );
    for (label, r) in rows.iter_mut() {
        println!(
            "{:<34} {:>5} | {:>5.2}/{:>5.2}/{:>5.1} | {:>6.1}/{:>7.1} | {:>6.2}/{:>6.2} | {:>6.3}/{:>6.3}",
            label,
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
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 2 {
        return Err(
            "usage: dump_reconstruction <replay> <dump.json> [--player name] [K...]".into(),
        );
    }
    let replay_path = PathBuf::from(args.remove(0));
    let dump_path = PathBuf::from(args.remove(0));
    if replay_to_rocketsim::sealed_path_refused(&replay_path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let mut player_name = "Vync62".to_string();
    if let Some(at) = args.iter().position(|a| a == "--player") {
        player_name = args.get(at + 1).cloned().ok_or("--player needs a name")?;
        args.drain(at..at + 2);
    }
    let ks: Vec<usize> = if args.is_empty() {
        vec![1, 2, 3, 4]
    } else {
        args.iter().filter_map(|a| a.parse().ok()).collect()
    };

    let replay = boxcars::ParserBuilder::new(&fs::read(&replay_path)?)
        .must_parse_network_data()
        .parse()?;
    let observed = extract(&replay).ok_or("no network frames")?;
    let dump: Value = serde_json::from_str(&fs::read_to_string(&dump_path)?)?;
    let dump_frames = dump["frames"].as_array().ok_or("frames")?;
    println!(
        "replay frames {}, dump frames {}",
        observed.frames.len(),
        dump_frames.len()
    );
    let player_key = observed
        .frames
        .iter()
        .flat_map(|f| f.players.iter())
        .find(|p| p.name.as_deref() == Some(player_name.as_str()))
        .map(|p| p.key.clone())
        .ok_or("player not found in the replay")?;

    let variants: [(&str, fn(&mut ConvertOptions)); 2] = [
        ("all fits", |_| {}),
        ("no input timing fits", |o| o.input_fits = false),
    ];

    for &k in &ks {
        // Thin the car packets: frames not a multiple of k repeat the last kept body.
        let mut thinned = observed.clone();
        let mut last: HashMap<(i32, usize), Body> = HashMap::new();
        for (f, frame) in thinned.frames.iter_mut().enumerate() {
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
        for (label, tweak) in variants {
            let mut options = ConvertOptions::default();
            options.zero_packet_lag = true;
            tweak(&mut options);
            let output = convert_observations(thinned.clone(), &options)?;
            let slot = output
                .car_slots
                .iter()
                .find(|s| s.player_key == player_key)
                .ok_or("player has no slot")?
                .slot;
            let mut rows: BTreeMap<String, Rows> = BTreeMap::new();
            let mut worst: Vec<(f32, usize, String)> = Vec::new();
            let mut inactive = 0usize;
            for (f, frame) in output.frames.iter().enumerate() {
                let Some(dump_frame) = dump_frames.get(f) else {
                    break;
                };
                let Some((_, car)) = frame.state.cars.get(slot) else {
                    continue;
                };
                // The converter does not step the simulation outside active play (goal replays,
                // countdown): those frames only repeat the packet state, so they are left out.
                let active = |i: usize| {
                    output.observations.frames[i]
                        .game_state
                        .as_ref()
                        .is_some_and(|g| g.value == "Active")
                };
                if !active(f) || f == 0 || !active(f - 1) {
                    inactive += 1;
                    continue;
                }
                let truth = &dump_frame["players"][0]["state"];
                let (tp, tv, tr, ta) = (
                    vec3(&truth["location"], ["X", "Y", "Z"]),
                    vec3(&truth["Velocity"], ["X", "Y", "Z"]),
                    matrix(&truth["Rotation"]),
                    vec3(&truth["AngularVelocity"], ["X", "Y", "Z"]),
                );
                let behaviour = if truth["b_isdodging"].as_bool().unwrap_or(false) {
                    "flip"
                } else if truth["b_jumped"].as_bool().unwrap_or(false)
                    && truth["time_offGround"].as_f64().unwrap_or(9.0) < 0.3
                {
                    "jump window"
                } else if truth["time_onGround"].as_f64().unwrap_or(0.0) > 0.0 {
                    "ground"
                } else {
                    "air"
                };
                let since = f % k;
                let ball_distance =
                    (vec3(&dump_frame["ball"]["location"], ["X", "Y", "Z"]) - tp).length();
                worst.push((
                    (car.phys.pos - tp).length(),
                    f,
                    format!(
                        "{behaviour}, {} ticks after packet, ball {:.0} UU, speed {:.0}, vel err {:.0}",
                        since * 4,
                        ball_distance,
                        tv.length(),
                        (car.phys.vel - tv).length()
                    ),
                ));
                let groups = [
                    "all".to_string(),
                    if since == 0 {
                        "kept packet".to_string()
                    } else {
                        "dropped packet".to_string()
                    },
                    format!("{} tick(s) after packet", since * 4),
                    format!("{behaviour} (dropped only)"),
                ];
                for (i, group) in groups.into_iter().enumerate() {
                    if i == 3 && since == 0 {
                        continue;
                    }
                    let r = rows.entry(group).or_default();
                    r.pos.push((car.phys.pos - tp).length());
                    r.vel.push((car.phys.vel - tv).length());
                    r.rot.push(rotation_error(car.phys.rot_mat, tr));
                    r.ang.push((car.phys.ang_vel - ta).length());
                }
            }
            print_rows(
                &format!("K = {k} ({label}); {inactive} frames outside active play left out"),
                &mut rows,
            );
            if env::var_os("DUMP_WORST").is_some() {
                worst.sort_by(|a, b| b.0.total_cmp(&a.0));
                for (error, f, what) in worst.iter().take(14) {
                    println!("  worst: frame {f}: pos err {error:.1} UU; {what}");
                }
            }
        }
    }
    Ok(())
}
