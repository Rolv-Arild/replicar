//! Are replay ball packets exact 120 Hz physics states? A direct, heuristic-free test.
//!
//! For consecutive frames with clean free-flight ball packets (away from walls, ceiling and any
//! car), start RocketSim from the first packet and step whole ticks k = 0..=12, comparing the
//! result with the second packet. If packets are exact server ticks, some integer k reproduces the
//! second packet to RocketSim's numerical precision for essentially every pair. If they are
//! sampled between ticks, the best-k residual is spread with a scale set by the fractional
//! offset. Signed residuals are converted to ticks (along the velocity, and along gravity from
//! the velocity change) so a fractional offset shows directly as a histogram spread.
//! Offline train diagnostic; refuses paths containing "test".

use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Quat, Vec3A};
use replicar_v1::conversion::rotation_error_degrees;
use replicar_v1::{observations, parse_replay};
use rocketsim::{Arena, ArenaConfig, BallState, GameMode};

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

fn ball_state(body: &observations::Body, frame: usize) -> Option<BallState> {
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
    let mut state = BallState::default();
    state.phys.pos = Vec3A::from_array(pos);
    state.phys.vel = Vec3A::from_array(vel);
    state.phys.ang_vel = Vec3A::from_array(ang) * 0.01;
    state.phys.rot_mat = Mat3A::from_quat(quat.normalize());
    Some(state)
}

fn histogram(values: &[f64], edges: &[f64]) -> Vec<f64> {
    let mut counts = vec![0usize; edges.len() + 1];
    for &v in values {
        counts[edges.iter().position(|&e| v < e).unwrap_or(edges.len())] += 1;
    }
    counts
        .iter()
        .map(|&c| c as f64 * 100.0 / values.len() as f64)
        .collect()
}

fn quantile(values: &mut [f64], q: f64) -> f64 {
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: audit_tick_integrality <train dir or replay>")?,
    );
    if replicar_v1::sealed_path_refused(&path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));

    let mut min_pos = Vec::new(); // position error at the best k (UU)
    let mut min_vel = Vec::new(); // velocity error at the best k (UU/s)
    let mut min_rot = Vec::new(); // rotation error at the best k (deg)
    let mut best_minus_nominal = Vec::new();
    let mut along_ticks = Vec::new(); // signed residual along velocity, in ticks
    let mut gravity_ticks = Vec::new(); // signed vertical velocity residual in ticks
    let mut speeds = Vec::new();

    for replay_path in replay_paths(&path)? {
        let replay = parse_replay(&fs::read(&replay_path)?)?;
        let observed = observations::extract(&replay).ok_or("network frames absent")?;
        for pair in observed.frames.windows(2) {
            let (a, b) = (&pair[0], &pair[1]);
            let active = |f: &observations::Frame| {
                f.game_state.as_ref().is_some_and(|s| s.value == "Active")
            };
            if !active(a) || !active(b) {
                continue;
            }
            let dt = f64::from(b.time - a.time);
            if !(0.02..0.05).contains(&dt) {
                continue;
            }
            let (Some(ba), Some(bb)) = (&a.ball, &b.ball) else {
                continue;
            };
            let (Some(sa), Some(sb)) = (ball_state(ba, a.index), ball_state(bb, b.index)) else {
                continue;
            };
            let clean = |s: &BallState, f: &observations::Frame| {
                let p = s.phys.pos;
                p.z > 200.0
                    && p.z < 1750.0
                    && p.x.abs() < 3700.0
                    && p.y.abs() < 4700.0
                    && f.cars.iter().all(|c| {
                        c.body
                            .position
                            .as_ref()
                            .is_none_or(|cp| (Vec3A::from_array(cp.value) - p).length() > 700.0)
                    })
            };
            if !clean(&sa, a) || !clean(&sb, b) || sa.phys.vel.length() < 300.0 {
                continue;
            }
            arena.set_ball_state(sa);
            let mut best: Option<(usize, f32)> = None;
            let mut per_k = Vec::new();
            for k in 0..=12usize {
                if k > 0 {
                    arena.step_tick();
                }
                let s = arena.get_ball_state();
                let err = (s.phys.pos - sb.phys.pos).length();
                per_k.push((
                    k,
                    err,
                    (s.phys.vel - sb.phys.vel).length(),
                    rotation_error_degrees(s.phys.rot_mat, sb.phys.rot_mat),
                    s.phys.pos,
                    s.phys.vel,
                ));
                if best.is_none_or(|(_, e)| err < e) {
                    best = Some((k, err));
                }
            }
            let (k, _) = best.unwrap();
            let (_, pos_err, vel_err, rot_err, pos, vel) = per_k[k];
            min_pos.push(f64::from(pos_err));
            min_vel.push(f64::from(vel_err));
            min_rot.push(f64::from(rot_err));
            best_minus_nominal.push(k as f64 - (dt * 120.0).round());
            let speed = sb.phys.vel.length();
            speeds.push(f64::from(speed));
            // Signed residual along the velocity: (predicted - actual) / speed * 120 ticks.
            let along = (pos - sb.phys.pos).dot(sb.phys.vel) / (speed * speed);
            along_ticks.push(f64::from(along) * 120.0);
            // Vertical velocity change per tick under gravity is 650/120 UU/s.
            gravity_ticks.push(f64::from((vel - sb.phys.vel).z) / (650.0 / 120.0));
        }
    }

    let n = min_pos.len();
    println!("clean free-flight ball frame pairs: {n}");
    let pos_edges = [0.01, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0];
    let show = |name: &str, values: &[f64], edges: &[f64]| {
        let h = histogram(values, edges);
        print!("{name:<34}");
        for (i, v) in h.iter().enumerate() {
            match edges.get(i) {
                Some(edge) => print!(" <{edge}:{v:.1}%"),
                None => print!(" >={}:{v:.1}%", edges[edges.len() - 1]),
            }
        }
        println!();
    };
    show("best-k position error (UU)", &min_pos, &pos_edges);
    show("best-k velocity error (UU/s)", &min_vel, &pos_edges);
    let rot_edges = [0.001, 0.005, 0.01, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0];
    show("best-k rotation error (deg)", &min_rot, &rot_edges);
    println!(
        "position error at best k: p10 {:.4} p50 {:.4} p90 {:.4} p99 {:.3} UU",
        quantile(&mut min_pos.clone(), 0.1),
        quantile(&mut min_pos.clone(), 0.5),
        quantile(&mut min_pos.clone(), 0.9),
        quantile(&mut min_pos.clone(), 0.99)
    );
    println!(
        "velocity error at best k: p10 {:.4} p50 {:.4} p90 {:.4} p99 {:.3} UU/s",
        quantile(&mut min_vel.clone(), 0.1),
        quantile(&mut min_vel.clone(), 0.5),
        quantile(&mut min_vel.clone(), 0.9),
        quantile(&mut min_vel.clone(), 0.99)
    );
    println!(
        "median ball speed {:.0} UU/s",
        quantile(&mut speeds.clone(), 0.5)
    );

    // Fractional offsets in ticks. Exact ticks would put both histograms in the central bin.
    let tick_edges: Vec<f64> = (-10..=10).map(|i| f64::from(i) * 0.1).collect();
    for (name, values) in [
        ("along-velocity residual (ticks)", &along_ticks),
        ("gravity residual (ticks)", &gravity_ticks),
    ] {
        let h = histogram(values, &tick_edges);
        println!("\n{name}: share of pairs per 0.1-tick bin, from -1 to +1 (tails at ends)");
        let line: Vec<String> = h.iter().map(|v| format!("{v:.1}")).collect();
        println!("  {}", line.join(" "));
        println!(
            "  |residual| < 0.02 tick: {:.1}%   < 0.1 tick: {:.1}%   < 0.5 tick: {:.1}%",
            values.iter().filter(|v| v.abs() < 0.02).count() as f64 * 100.0 / n as f64,
            values.iter().filter(|v| v.abs() < 0.1).count() as f64 * 100.0 / n as f64,
            values.iter().filter(|v| v.abs() < 0.5).count() as f64 * 100.0 / n as f64,
        );
    }
    let mut delta = best_minus_nominal.clone();
    println!(
        "\nbest whole-tick k minus round(frame dt * 120): p10 {:.0} p50 {:.0} p90 {:.0}",
        quantile(&mut delta, 0.1),
        quantile(&mut delta, 0.5),
        quantile(&mut delta, 0.9)
    );
    Ok(())
}
