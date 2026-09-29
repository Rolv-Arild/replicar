//! Error budget for one-step, pre-correction replay-state prediction on development replays.
//!
//! Splits the converter's own position and linear-velocity residuals by object, packet gap,
//! ground/air/wall regime and proximity to other objects, and reports how much of the total
//! squared error and of the large-error tail each group holds. Position error is also split into
//! along-track (parallel to the last observed velocity) and cross-track parts; a pure timing
//! offset appears as an along-track error proportional to speed.

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes};

#[derive(Default)]
struct Group {
    position: Vec<f32>,
    velocity: Vec<f32>,
    along_sq: f64,
    cross_sq: f64,
    vertical_sq: f64,
    /// Signed along-track error divided by previous speed: an implied time offset in seconds.
    time_offsets: Vec<f32>,
}

fn quantile(values: &mut Vec<f32>, q: f64) -> f32 {
    if values.is_empty() {
        return f32::NAN;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

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

fn norm(v: [f32; 3]) -> f32 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn dist(a: [f32; 3], b: [f32; 3]) -> f32 {
    norm([a[0] - b[0], a[1] - b[1], a[2] - b[2]])
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: error_budget <split dir or replay>")?,
    );
    if path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let mut options = ConvertOptions::default();
    options.infer_packet_lag = env::args_os().any(|arg| arg == "--infer-packet-lag");
    let mut groups: BTreeMap<String, Group> = BTreeMap::new();
    let mut total_position_sq = 0.0f64;
    let mut total_tail = 0usize;
    let mut total_samples = 0usize;

    for replay_path in replay_paths(&path)? {
        let output = convert_bytes(&fs::read(&replay_path)?, &options)?;
        let frames = &output.observations.frames;
        for residual in &output.position_residuals {
            let is_car = residual.actor_id.is_some();
            let frame = &frames[residual.frame];
            let ball = frame
                .ball
                .as_ref()
                .and_then(|body| body.position.as_ref())
                .map(|p| p.value);
            let car_positions: Vec<(i32, [f32; 3])> = frame
                .cars
                .iter()
                .filter_map(|car| Some((car.actor_id, car.body.position.as_ref()?.value)))
                .collect();
            let own_position = residual.actor_id.and_then(|id| {
                car_positions
                    .iter()
                    .find(|(a, _)| *a == id)
                    .map(|(_, p)| *p)
            });
            let gap = residual.seconds_since_previous_position;
            let gap_label = if gap < 0.045 {
                "gap 1 frame"
            } else if gap < 0.08 {
                "gap 2 frames"
            } else if gap < 0.12 {
                "gap 3 frames"
            } else {
                "gap >3 frames"
            };
            let altitude = residual.altitude_z.unwrap_or(f32::NAN);
            let regime = if is_car {
                let near_ball =
                    matches!((own_position, ball), (Some(c), Some(b)) if dist(c, b) < 300.0);
                if near_ball {
                    "car near ball (<300)"
                } else if residual.is_on_ground == Some(true) && altitude < 50.0 {
                    "car ground"
                } else if residual.is_on_ground == Some(true) {
                    "car on wall/ramp/ceiling"
                } else {
                    "car airborne"
                }
            } else {
                let near_car =
                    ball.is_some_and(|b| car_positions.iter().any(|(_, c)| dist(*c, b) < 300.0));
                if near_car {
                    "ball near car (<300)"
                } else if altitude > 200.0 {
                    "ball high air"
                } else {
                    "ball free"
                }
            };
            let kind = if is_car { "CAR" } else { "BALL" };
            let labels = [
                format!("{kind} all"),
                format!("{kind} {gap_label}"),
                format!("{kind} / {regime}"),
                format!("{kind} / {regime} / {gap_label}"),
            ];
            let error = residual.simulated_error_vector_uu;
            let magnitude = residual.simulated_error_uu;
            total_position_sq += f64::from(magnitude).powi(2) * f64::from(is_car);
            total_samples += usize::from(is_car);
            total_tail += usize::from(is_car && magnitude > 50.0);
            for label in labels {
                let group = groups.entry(label).or_default();
                group.position.push(magnitude);
                if let Some(v) = residual.simulated_velocity_error_uu_per_sec {
                    group.velocity.push(v);
                }
                if let Some(v) = residual.previous_linear_velocity_uu_per_second {
                    let speed = norm(v);
                    if speed > 300.0 {
                        let unit = [v[0] / speed, v[1] / speed, v[2] / speed];
                        let along = error[0] * unit[0] + error[1] * unit[1] + error[2] * unit[2];
                        let cross_sq = (magnitude.powi(2) - along.powi(2)).max(0.0);
                        group.along_sq += f64::from(along).powi(2);
                        group.cross_sq += f64::from(cross_sq);
                        group.vertical_sq += f64::from(error[2]).powi(2);
                        group.time_offsets.push(along / speed);
                    }
                }
            }
        }
    }

    println!("car samples {total_samples}; car position error > 50 UU in {total_tail}");
    println!("share columns are fractions of the car position sum of squared error");
    println!(
        "{:<52} {:>8} {:>6} | {:>6} {:>6} {:>7} | {:>6} {:>6} | {:>5} {:>6} | {:>6} {:>7}",
        "group",
        "n",
        "SSshr",
        "p50",
        "p90",
        "p99",
        "vp50",
        "vp90",
        "along",
        "vert",
        "toff50",
        "toffIQR"
    );
    for (label, group) in groups.iter_mut() {
        let sum_sq: f64 = group.position.iter().map(|&v| f64::from(v).powi(2)).sum();
        let share = if label.starts_with("CAR") {
            sum_sq / total_position_sq
        } else {
            f64::NAN
        };
        let n = group.position.len();
        let (p50, p90, p99) = (
            quantile(&mut group.position, 0.5),
            quantile(&mut group.position, 0.9),
            quantile(&mut group.position, 0.99),
        );
        let (v50, v90) = (
            quantile(&mut group.velocity, 0.5),
            quantile(&mut group.velocity, 0.9),
        );
        let axis_total = group.along_sq + group.cross_sq;
        let along = if axis_total > 0.0 {
            group.along_sq / axis_total
        } else {
            f64::NAN
        };
        let vertical = if axis_total > 0.0 {
            group.vertical_sq / axis_total
        } else {
            f64::NAN
        };
        let (t50, tq1, tq3) = (
            quantile(&mut group.time_offsets, 0.5),
            quantile(&mut group.time_offsets, 0.25),
            quantile(&mut group.time_offsets, 0.75),
        );
        println!(
            "{:<52} {:>8} {:>6.3} | {:>6.1} {:>6.1} {:>7.1} | {:>6.0} {:>6.0} | {:>5.2} {:>6.2} | {:>6.3} {:>3.3}..{:<3.3}",
            label, n, share, p50, p90, p99, v50, v90, along, vertical, t50, tq1, tq3
        );
    }
    Ok(())
}
