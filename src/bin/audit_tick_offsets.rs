//! How far behind its frame time is each ball packet's server tick, and is that stationary?
//!
//! Uses only exact whole-tick identifications (RocketSim reproduces the next packet within 0.05 UU)
//! between consecutive frames of clean free-flight ball motion. Along an unbroken run, the
//! cumulative physical ticks S_i and the frame times T_i (in ticks since the run's first packet)
//! give r_i = T_i - S_i: each packet's offset behind its frame time, up to one unknown constant per
//! run. A stationary bounded jitter means the constant is pinned by requiring the smallest offset
//! to be about zero; drift or wide spread would break a per-frame-window model. Offline train
//! diagnostic; refuses paths containing "test".

use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Quat, Vec3A};
use replay_to_rocketsim::{observations, parse_replay};
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

fn quantile(values: &mut [f64], q: f64) -> f64 {
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: audit_tick_offsets <train dir or replay>")?,
    );
    if path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));

    let min_run = 30usize;
    let mut ranges = Vec::new(); // max - min of r over each run
    let mut above_min = Vec::new(); // r - min(r), all packets in long runs
    let mut slopes = Vec::new(); // least-squares slope of r per frame, ticks
    let mut lag1 = (0.0f64, 0.0f64);
    let mut run_lengths = Vec::new();
    let mut total_pairs = 0usize;
    let mut increments = std::collections::BTreeMap::<i64, usize>::new();

    let mut finish = |offsets: &mut Vec<f64>| {
        if offsets.len() >= min_run {
            let min = offsets.iter().cloned().fold(f64::INFINITY, f64::min);
            let max = offsets.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            ranges.push(max - min);
            run_lengths.push(offsets.len() as f64);
            above_min.extend(offsets.iter().map(|r| r - min));
            let n = offsets.len() as f64;
            let mean_x = (n - 1.0) / 2.0;
            let mean_y = offsets.iter().sum::<f64>() / n;
            let (mut sxy, mut sxx) = (0.0, 0.0);
            for (i, r) in offsets.iter().enumerate() {
                sxy += (i as f64 - mean_x) * (r - mean_y);
                sxx += (i as f64 - mean_x).powi(2);
            }
            slopes.push(sxy / sxx);
            for w in offsets.windows(2) {
                lag1.0 += (w[0] - mean_y) * (w[1] - mean_y);
                lag1.1 += (w[0] - mean_y).powi(2);
            }
        }
        offsets.clear();
    };

    for replay_path in replay_paths(&path)? {
        let replay = parse_replay(&fs::read(&replay_path)?)?;
        let observed = observations::extract(&replay).ok_or("network frames absent")?;
        let mut offsets: Vec<f64> = Vec::new();
        let mut last_frame = usize::MAX;
        let (mut cumulative, mut origin_time) = (0i64, 0.0f64);
        for pair in observed.frames.windows(2) {
            let (a, b) = (&pair[0], &pair[1]);
            let active = |f: &observations::Frame| {
                f.game_state.as_ref().is_some_and(|s| s.value == "Active")
            };
            let dt = f64::from(b.time - a.time);
            let usable = active(a) && active(b) && (0.02..0.05).contains(&dt);
            let states = if usable {
                a.ball
                    .as_ref()
                    .zip(b.ball.as_ref())
                    .and_then(|(x, y)| Some((ball_state(x, a.index)?, ball_state(y, b.index)?)))
            } else {
                None
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
            let Some((sa, sb)) = states
                .filter(|(sa, sb)| clean(sa, a) && clean(sb, b) && sa.phys.vel.length() > 300.0)
            else {
                finish(&mut offsets);
                last_frame = usize::MAX;
                continue;
            };
            arena.set_ball_state(sa);
            let mut best = (0usize, f32::INFINITY);
            for k in 0..=12usize {
                if k > 0 {
                    arena.step_tick();
                }
                let err = (arena.get_ball_state().phys.pos - sb.phys.pos).length();
                if err < best.1 {
                    best = (k, err);
                }
            }
            total_pairs += 1;
            if best.1 >= 0.05 {
                finish(&mut offsets);
                last_frame = usize::MAX;
                continue;
            }
            if a.index != last_frame {
                finish(&mut offsets);
                cumulative = 0;
                origin_time = f64::from(a.time);
                offsets.push(0.0);
            }
            cumulative += best.0 as i64;
            *increments.entry(best.0 as i64).or_default() += 1;
            offsets.push((f64::from(b.time) - origin_time) * 120.0 - cumulative as f64);
            last_frame = b.index;
        }
        finish(&mut offsets);
    }

    println!(
        "clean pairs examined {total_pairs}; long runs (>= {min_run} packets) {}",
        ranges.len()
    );
    println!(
        "run length p10/p50/p90: {:.0} / {:.0} / {:.0} packets",
        quantile(&mut run_lengths.clone(), 0.1),
        quantile(&mut run_lengths.clone(), 0.5),
        quantile(&mut run_lengths.clone(), 0.9)
    );
    println!(
        "range of offset r within a run (ticks) p10/p50/p90/max: {:.2} / {:.2} / {:.2} / {:.2}",
        quantile(&mut ranges.clone(), 0.1),
        quantile(&mut ranges.clone(), 0.5),
        quantile(&mut ranges.clone(), 0.9),
        quantile(&mut ranges.clone(), 1.0)
    );
    println!(
        "slope of r per frame (ticks) p10/p50/p90: {:.4} / {:.4} / {:.4}   (drift would be non-zero)",
        quantile(&mut slopes.clone(), 0.1),
        quantile(&mut slopes.clone(), 0.5),
        quantile(&mut slopes.clone(), 0.9)
    );
    println!(
        "lag-1 autocorrelation of r within runs: {:.3}",
        lag1.0 / lag1.1
    );
    let n = above_min.len() as f64;
    let edges: Vec<f64> = (1..=14).map(|i| f64::from(i) * 0.5).collect();
    print!("offset above the run minimum, share per 0.5-tick bin (0 to 7+):");
    for (i, e) in std::iter::once(0.0)
        .chain(edges.iter().copied())
        .enumerate()
    {
        let hi = edges.get(i).copied().unwrap_or(f64::INFINITY);
        let share = above_min.iter().filter(|v| **v >= e && **v < hi).count() as f64 * 100.0 / n;
        print!(" {share:.1}");
    }
    println!();
    println!("whole-tick increments between consecutive frames (k: count): {increments:?}");
    Ok(())
}
