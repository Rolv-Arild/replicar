//! Compare the 120 Hz tick count implied by free-flight ball motion with replay frame times.
//!
//! For consecutive active frames with fresh ball position and velocity, the ball's displacement
//! along its mean velocity gives an implied elapsed time (gravity is second order for the mean of
//! the two endpoint velocities). This measures the number of physics ticks between the two
//! packets without trusting the frame timestamps. It is an offline diagnostic of packet timing,
//! not a prediction; use train replays.

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::{observations, parse_replay};

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

fn quantile(values: &mut [f64], q: f64) -> f64 {
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: audit_ball_ticks <train dir or replay>")?,
    );
    if replay_to_rocketsim::sealed_path_refused(&path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    // (rounded nominal ticks, rounded implied ticks) -> count
    let mut crosstab: BTreeMap<(i64, i64), usize> = BTreeMap::new();
    let mut residuals = Vec::new(); // implied - nominal ticks, continuous
    let mut deltas_ms = Vec::new();
    let mut lag1_num = 0.0f64;
    let mut lag1_den = 0.0f64;
    let mut per_replay_frame_rate: Vec<(String, f64)> = Vec::new();

    for replay_path in replay_paths(&path)? {
        let replay = parse_replay(&fs::read(&replay_path)?)?;
        let observed = observations::extract(&replay).ok_or("network frames absent")?;
        let mut replay_dts = Vec::new();
        let mut last: Option<(usize, f64)> = None;
        for pair in observed.frames.windows(2) {
            let (a, b) = (&pair[0], &pair[1]);
            let active = |f: &observations::Frame| {
                f.game_state.as_ref().is_some_and(|s| s.value == "Active")
            };
            if !active(a) || !active(b) {
                last = None;
                continue;
            }
            let (Some(ba), Some(bb)) = (&a.ball, &b.ball) else {
                continue;
            };
            let (Some(pa), Some(pb), Some(va), Some(vb)) = (
                ba.position.as_ref().filter(|v| v.frame == a.index),
                bb.position.as_ref().filter(|v| v.frame == b.index),
                ba.linear_velocity.as_ref().filter(|v| v.frame == a.index),
                bb.linear_velocity.as_ref().filter(|v| v.frame == b.index),
            ) else {
                last = None;
                continue;
            };
            // Free flight: away from the floor/walls and no car within reach at either end.
            let z = pa.value[2].min(pb.value[2]);
            let near_car = |f: &observations::Frame, p: [f32; 3]| {
                f.cars.iter().any(|c| {
                    c.body.position.as_ref().is_some_and(|cp| {
                        let d = [cp.value[0] - p[0], cp.value[1] - p[1], cp.value[2] - p[2]];
                        (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() < 500.0
                    })
                })
            };
            let vmean = [
                0.5 * (va.value[0] + vb.value[0]) as f64,
                0.5 * (va.value[1] + vb.value[1]) as f64,
                0.5 * (va.value[2] + vb.value[2]) as f64,
            ];
            let speed2 = vmean.iter().map(|v| v * v).sum::<f64>();
            if z < 250.0 || speed2.sqrt() < 500.0 || near_car(a, pa.value) || near_car(b, pb.value)
            {
                last = None;
                continue;
            }
            let dp = [
                (pb.value[0] - pa.value[0]) as f64,
                (pb.value[1] - pa.value[1]) as f64,
                (pb.value[2] - pa.value[2]) as f64,
            ];
            let implied = dp.iter().zip(&vmean).map(|(d, v)| d * v).sum::<f64>() / speed2;
            let dt = (b.time - a.time) as f64;
            if !(dt > 0.0 && dt < 0.2) {
                last = None;
                continue;
            }
            let (implied_ticks, nominal_ticks) = (implied * 120.0, dt * 120.0);
            *crosstab
                .entry((nominal_ticks.round() as i64, implied_ticks.round() as i64))
                .or_default() += 1;
            let residual = implied_ticks - nominal_ticks;
            residuals.push(residual);
            deltas_ms.push(dt * 1000.0);
            replay_dts.push(dt);
            if let Some((frame, previous)) = last {
                if frame + 1 == a.index {
                    lag1_num += previous * residual;
                    lag1_den += previous * previous;
                }
            }
            last = Some((a.index, residual));
        }
        if !replay_dts.is_empty() {
            let mean = replay_dts.iter().sum::<f64>() / replay_dts.len() as f64;
            per_replay_frame_rate.push((
                replay_path.file_name().unwrap().to_string_lossy().into(),
                1.0 / mean,
            ));
        }
    }

    println!("free-flight ball frame pairs: {}", residuals.len());
    println!(
        "frame delta ms p1/p50/p99: {:.1} / {:.1} / {:.1}",
        quantile(&mut deltas_ms.clone(), 0.01),
        quantile(&mut deltas_ms.clone(), 0.5),
        quantile(&mut deltas_ms.clone(), 0.99)
    );
    println!(
        "implied - nominal ticks: p1 {:.2} p25 {:.2} p50 {:.2} p75 {:.2} p99 {:.2}; mean {:.3}",
        quantile(&mut residuals.clone(), 0.01),
        quantile(&mut residuals.clone(), 0.25),
        quantile(&mut residuals.clone(), 0.5),
        quantile(&mut residuals.clone(), 0.75),
        quantile(&mut residuals.clone(), 0.99),
        residuals.iter().sum::<f64>() / residuals.len() as f64
    );
    println!(
        "lag-1 autocorrelation of consecutive-frame residuals: {:.3} (-0.5 = pure timestamp jitter on a regular tick grid)",
        lag1_num / lag1_den
    );
    println!("\nrows = round(frame dt * 120) nominal ticks; columns = round(implied ticks)");
    let columns: Vec<i64> = {
        let mut c: Vec<i64> = crosstab.keys().map(|k| k.1).collect();
        c.sort();
        c.dedup();
        c.into_iter().filter(|c| (0..=12).contains(c)).collect()
    };
    print!("{:>8}", "nom\\imp");
    for c in &columns {
        print!("{:>9}", c);
    }
    println!();
    let rows: Vec<i64> = {
        let mut r: Vec<i64> = crosstab.keys().map(|k| k.0).collect();
        r.sort();
        r.dedup();
        r.into_iter().filter(|r| (0..=12).contains(r)).collect()
    };
    for r in rows {
        print!("{:>8}", r);
        for c in &columns {
            print!("{:>9}", crosstab.get(&(r, *c)).copied().unwrap_or(0));
        }
        println!();
    }
    per_replay_frame_rate.sort_by(|a, b| a.1.total_cmp(&b.1));
    println!(
        "\nmean frame rate (Hz) per replay: min {:.1}, median {:.1}, max {:.1}",
        per_replay_frame_rate.first().map_or(0.0, |x| x.1),
        per_replay_frame_rate[per_replay_frame_rate.len() / 2].1,
        per_replay_frame_rate.last().map_or(0.0, |x| x.1)
    );
    Ok(())
}
