//! Compare per-packet car position error with and without packet-lag inference to find where the
//! inference makes things worse. Train replays only.

use std::collections::{BTreeMap, HashMap};
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes, infer_packet_lags};

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

#[derive(Default)]
struct Bucket {
    n: usize,
    worse: usize,
    better: usize,
    delta_sum: f64,
    lag_err: f64,
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: lag_outliers <train dir>")?,
    );
    if replay_to_rocketsim::sealed_path_refused(&path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let base = ConvertOptions::default();
    let mut lagged = ConvertOptions::default();
    lagged.infer_packet_lag = true;
    let mut buckets: BTreeMap<String, Bucket> = BTreeMap::new();
    for replay_path in replay_paths(&path)? {
        let bytes = fs::read(&replay_path)?;
        let plain = convert_bytes(&bytes, &base)?;
        let with_lag = convert_bytes(&bytes, &lagged)?;
        let lags = infer_packet_lags(&with_lag.observations, &lagged);
        let baseline: HashMap<(usize, i32), f32> = plain
            .position_residuals
            .iter()
            .filter_map(|r| Some(((r.frame, r.actor_id?), r.simulated_error_uu)))
            .collect();
        for residual in &with_lag.position_residuals {
            let Some(actor) = residual.actor_id else {
                continue;
            };
            let Some(&before) = baseline.get(&(residual.frame, actor)) else {
                continue;
            };
            let after = residual.simulated_error_uu;
            let inferred = lags.cars[residual.frame].is_some();
            let speed = residual
                .previous_linear_velocity_uu_per_second
                .map_or(f32::NAN, |v| {
                    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
                });
            let gap = residual.seconds_since_previous_position;
            let labels = [
                format!("lag {}", if inferred { "inferred" } else { "default" }),
                format!(
                    "lag {} / speed {}",
                    if inferred { "inferred" } else { "default" },
                    if speed < 500.0 {
                        "<500"
                    } else if speed < 1000.0 {
                        "500-1000"
                    } else if speed < 1500.0 {
                        "1000-1500"
                    } else {
                        ">=1500"
                    }
                ),
                format!(
                    "lag {} / gap {}",
                    if inferred { "inferred" } else { "default" },
                    if gap < 0.045 {
                        "1"
                    } else if gap < 0.08 {
                        "2"
                    } else if gap < 0.12 {
                        "3"
                    } else {
                        ">3"
                    }
                ),
                format!(
                    "lag {} / ground {:?}",
                    if inferred { "inferred" } else { "default" },
                    residual.is_on_ground
                ),
            ];
            for label in labels {
                let b = buckets.entry(label).or_default();
                b.n += 1;
                b.worse += usize::from(after > before + 10.0);
                b.better += usize::from(after < before - 10.0);
                b.delta_sum += f64::from(after - before);
                b.lag_err += f64::from(after);
            }
        }
    }
    println!(
        "{:<44} {:>8} {:>8} {:>8} {:>10} {:>9}",
        "group", "n", "worse>10", "better>10", "mean delta", "mean err"
    );
    for (label, b) in &buckets {
        println!(
            "{:<44} {:>8} {:>8} {:>8} {:>10.2} {:>9.2}",
            label,
            b.n,
            b.worse,
            b.better,
            b.delta_sum / b.n as f64,
            b.lag_err / b.n as f64
        );
    }
    Ok(())
}
