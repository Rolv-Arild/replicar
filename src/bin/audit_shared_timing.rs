//! Test whether the physical time between two replay frames is shared by the ball and the cars.
//!
//! For consecutive active frames where the ball and a car both have fresh position and velocity
//! at both frames, compare the 120 Hz ticks implied by each object's own motion (displacement
//! along mean velocity). If snapshots are captured at one physical time per frame, the two should
//! agree far better than either agrees with the nominal `round(dt * 120)`. Offline diagnostic on
//! train replays; the implied ticks use both endpoint packets and are not predictions.

use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::observations::{self, Body};
use replay_to_rocketsim::parse_replay;

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

fn implied_ticks(a: &Body, ai: usize, b: &Body, bi: usize, min_speed: f64) -> Option<f64> {
    let pa = a.position.as_ref().filter(|v| v.frame == ai)?.value;
    let pb = b.position.as_ref().filter(|v| v.frame == bi)?.value;
    let va = a.linear_velocity.as_ref().filter(|v| v.frame == ai)?.value;
    let vb = b.linear_velocity.as_ref().filter(|v| v.frame == bi)?.value;
    let mean: Vec<f64> = (0..3).map(|i| 0.5 * (va[i] + vb[i]) as f64).collect();
    let speed2: f64 = mean.iter().map(|v| v * v).sum();
    if speed2.sqrt() < min_speed {
        return None;
    }
    let dot: f64 = (0..3).map(|i| (pb[i] - pa[i]) as f64 * mean[i]).sum();
    Some(dot / speed2 * 120.0)
}

fn quantile(values: &mut [f64], q: f64) -> f64 {
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn corr(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len() as f64;
    let (mx, my) = (x.iter().sum::<f64>() / n, y.iter().sum::<f64>() / n);
    let cov: f64 = x.iter().zip(y).map(|(a, b)| (a - mx) * (b - my)).sum();
    let vx: f64 = x.iter().map(|a| (a - mx).powi(2)).sum();
    let vy: f64 = y.iter().map(|b| (b - my).powi(2)).sum();
    cov / (vx * vy).sqrt()
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: audit_shared_timing <train dir or replay>")?,
    );
    if replay_to_rocketsim::sealed_path_refused(&path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let mut ball_ticks = Vec::new();
    let mut car_ticks = Vec::new();
    let mut nominal = Vec::new();
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
            let dt = (b.time - a.time) as f64;
            if !(0.02..0.05).contains(&dt) {
                continue;
            }
            let (Some(ba), Some(bb)) = (&a.ball, &b.ball) else {
                continue;
            };
            // Free-flight ball only, so gravity/contact do not bias the reference.
            let ball_z = ba.position.as_ref().map_or(0.0, |p| p.value[2]);
            let Some(ball) = implied_ticks(ba, a.index, bb, b.index, 500.0) else {
                continue;
            };
            if ball_z < 250.0 {
                continue;
            }
            for car_a in &a.cars {
                let Some(car_b) = b.cars.iter().find(|c| {
                    c.actor_id == car_a.actor_id
                        && c.actor_created_frame == car_a.actor_created_frame
                }) else {
                    continue;
                };
                // Straight-ish driving/flying only: avoid cars near the ball (contact) and dodges.
                let near_ball = car_a
                    .body
                    .position
                    .as_ref()
                    .zip(ba.position.as_ref())
                    .is_some_and(|(c, b)| {
                        (0..3)
                            .map(|i| (c.value[i] - b.value[i]).powi(2))
                            .sum::<f32>()
                            .sqrt()
                            < 400.0
                    });
                if near_ball {
                    continue;
                }
                let Some(car) = implied_ticks(&car_a.body, a.index, &car_b.body, b.index, 700.0)
                else {
                    continue;
                };
                ball_ticks.push(ball);
                car_ticks.push(car);
                nominal.push(dt * 120.0);
            }
        }
    }
    let diff: Vec<f64> = car_ticks
        .iter()
        .zip(&ball_ticks)
        .map(|(c, b)| c - b)
        .collect();
    let car_vs_nominal: Vec<f64> = car_ticks.iter().zip(&nominal).map(|(c, n)| c - n).collect();
    let ball_vs_nominal: Vec<f64> = ball_ticks
        .iter()
        .zip(&nominal)
        .map(|(b, n)| b - n)
        .collect();
    let abs = |v: &[f64]| -> Vec<f64> { v.iter().map(|x| x.abs()).collect() };
    println!(
        "pairs (consecutive frames, free-flight ball and a car, both fresh): {}",
        diff.len()
    );
    println!(
        "correlation(car implied ticks, ball implied ticks) = {:.3}",
        corr(&car_ticks, &ball_ticks)
    );
    println!(
        "correlation(car - nominal, ball - nominal)         = {:.3}",
        corr(&car_vs_nominal, &ball_vs_nominal)
    );
    for (label, values) in [
        ("car - ball", &diff),
        ("car - nominal", &car_vs_nominal),
        ("ball - nominal", &ball_vs_nominal),
    ] {
        let a = abs(values);
        println!(
            "|{label}| ticks: p50 {:.2} p75 {:.2} p90 {:.2} p99 {:.2}; signed median {:.2}",
            quantile(&mut a.clone(), 0.5),
            quantile(&mut a.clone(), 0.75),
            quantile(&mut a.clone(), 0.9),
            quantile(&mut a.clone(), 0.99),
            quantile(&mut values.to_vec(), 0.5)
        );
    }
    let within =
        |v: &[f64], t: f64| v.iter().filter(|x| x.abs() <= t).count() as f64 / v.len() as f64;
    println!(
        "fraction within 0.5 tick: car-ball {:.3}, car-nominal {:.3}",
        within(&diff, 0.5),
        within(&car_vs_nominal, 0.5)
    );
    println!(
        "fraction within 1.0 tick: car-ball {:.3}, car-nominal {:.3}",
        within(&diff, 1.0),
        within(&car_vs_nominal, 1.0)
    );
    Ok(())
}
