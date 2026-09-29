//! How often is the inferred packet lag off by a tick? Train replays only.
//!
//! Between consecutive fresh packets of clean free-flight ball motion (and airborne, input-free
//! car motion), RocketSim identifies the exact whole tick count k (as in `audit_tick_integrality`).
//! The converter's inferred lags imply k' = (timeline tick b - lag b) - (timeline tick a - lag a).
//! This reports the distribution of k' - k, overall and by lag source, so the position error that
//! remains from lag inference can be attributed. Refuses paths containing "test".

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Quat, Vec3A};
use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes};
use replay_to_rocketsim::observations::Body;
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

fn ball_state(body: &Body, frame: usize) -> Option<BallState> {
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

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: audit_lag_accuracy <train dir or replay>")?,
    );
    if path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let mut options = ConvertOptions::default();
    options.exact_tick_lag_chains = !env::args_os().any(|arg| arg == "--no-exact-tick-lag-chains");
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    let mut errors: BTreeMap<(&'static str, i64), usize> = BTreeMap::new();
    let mut totals: BTreeMap<&'static str, usize> = BTreeMap::new();
    let mut default_hits = 0usize;
    let mut interval_errors: Vec<f64> = Vec::new();

    for replay_path in replay_paths(&path)? {
        let output = convert_bytes(&fs::read(&replay_path)?, &options)?;
        let frames = &output.observations.frames;
        let first_time = frames[0].time;
        let timeline = |f: usize| -> i64 {
            ((f64::from(frames[f].time) - f64::from(first_time)) * 120.0).round() as i64
        };
        for a in 0..frames.len().saturating_sub(1) {
            let b = a + 1;
            let active = |f: usize| {
                frames[f]
                    .game_state
                    .as_ref()
                    .is_some_and(|s| s.value == "Active")
            };
            if !active(a) || !active(b) {
                continue;
            }
            let (Some(ball_a), Some(ball_b)) = (&frames[a].ball, &frames[b].ball) else {
                continue;
            };
            let (Some(sa), Some(sb)) = (ball_state(ball_a, a), ball_state(ball_b, b)) else {
                continue;
            };
            let clean = |s: &BallState, f: usize| {
                let p = s.phys.pos;
                p.z > 200.0
                    && p.z < 1750.0
                    && p.x.abs() < 3700.0
                    && p.y.abs() < 4700.0
                    && frames[f].cars.iter().all(|c| {
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
            let mut best = (0i64, f32::INFINITY);
            for k in 0..=14i64 {
                if k > 0 {
                    arena.step_tick();
                }
                let err = (arena.get_ball_state().phys.pos - sb.phys.pos).length();
                if err < best.1 {
                    best = (k, err);
                }
            }
            if best.1 >= 0.05 {
                continue;
            }
            {
                let mean = (sa.phys.vel + sb.phys.vel) * 0.5;
                let implied_k =
                    f64::from((sb.phys.pos - sa.phys.pos).dot(mean) / mean.length_squared())
                        * 120.0;
                interval_errors.push((implied_k - best.0 as f64).abs());
            }
            let lag_of = |f: usize| {
                output.frames[f]
                    .packet_lags
                    .iter()
                    .find(|l| l.actor_id.is_none())
            };
            let (Some(la), Some(lb)) = (lag_of(a), lag_of(b)) else {
                continue;
            };
            let implied = (timeline(b) - lb.ticks as i64) - (timeline(a) - la.ticks as i64);
            let source = if la.source == "chain" && lb.source == "chain" {
                "both chain"
            } else {
                "a default"
            };
            *errors.entry((source, implied - best.0)).or_default() += 1;
            *totals.entry(source).or_default() += 1;
            if source == "a default" {
                default_hits += 1;
            }
        }
    }
    println!("clean free-flight ball frame pairs by lag source (k' - k in ticks):");
    for (source, total) in &totals {
        let row: Vec<String> = errors
            .iter()
            .filter(|((s, _), _)| s == source)
            .map(|((_, e), n)| format!("{e:+}: {:.1}%", *n as f64 * 100.0 / *total as f64))
            .collect();
        println!("  {source} (n={total}): {}", row.join("  "));
    }
    let _ = default_hits;
    interval_errors.sort_by(|a, b| a.total_cmp(b));
    let q = |p: f64| interval_errors[((interval_errors.len() - 1) as f64 * p) as usize];
    println!(
        "ball |estimated interval - exact k| (ticks): p50 {:.3} p90 {:.3} p99 {:.3} p99.9 {:.3} max {:.3}",
        q(0.5),
        q(0.9),
        q(0.99),
        q(0.999),
        q(1.0)
    );
    Ok(())
}
