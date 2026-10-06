//! Error budget for one-step, pre-correction replay-state prediction on development replays.
//!
//! Residuals are the converter's own errors just before a fresh packet corrects the simulation.
//! With packet-lag inference on, only packets whose lag came from their own motion chain
//! (`source == "chain"`) are used, so the residual isolates model error from timing error.
//! Each residual is assigned to exactly one behavior partition (first match) so that shares of
//! squared error add up: flip/jump activity, contact with the ball or another car, boosting,
//! driving on the ground or a wall, and coasting or aerial flight. Reports position, velocity,
//! rotation and angular-velocity quantiles and each partition's share of the total squared error.
//! Train replays only; refuses paths containing "test".

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replicar_v1::conversion::{ConvertOptions, convert_bytes};
use replicar_v1::observations::{Car, Value};

#[derive(Default)]
struct Group {
    position: Vec<f32>,
    velocity: Vec<f32>,
    rotation: Vec<f32>,
    angular: Vec<f32>,
}

fn quantile(values: &mut [f32], q: f64) -> f32 {
    if values.is_empty() {
        return f32::NAN;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn replay_paths(path: &Path, final_assessment: bool) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    // Every directory and file opened is resolved and refused when it is in the sealed test split (a link or
    // junction under another name included).
    replicar_v1::ensure_unsealed(path, final_assessment)?;
    if path.is_file() {
        return Ok(vec![path.to_owned()]);
    }
    let mut result = Vec::new();
    for size in ["1v1", "2v2", "3v3"] {
        replicar_v1::ensure_unsealed(&path.join(size), final_assessment)?;
        for entry in fs::read_dir(path.join(size))? {
            let path = entry?.path();
            if path.extension().is_some_and(|ext| ext == "replay") {
                replicar_v1::ensure_unsealed(&path, final_assessment)?;
                result.push(path);
            }
        }
    }
    result.sort();
    Ok(result)
}

fn dist(a: [f32; 3], b: [f32; 3]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn odd(value: &Option<Value<u8>>) -> bool {
    value.as_ref().is_some_and(|v| v.value % 2 == 1)
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(env::args_os().nth(1).ok_or(
        "usage: error_budget <split dir or replay> [--no-infer-packet-lag] [--final-assessment]",
    )?);
    // The test split is sealed until the frozen assessment (TEST_PROTOCOL.md); only that run passes the flag.
    if replicar_v1::sealed_path_refused(&path, env::args_os().any(|arg| arg == "--final-assessment")) {
        return Err("refusing to inspect a path with a 'test' component (pass --final-assessment for the frozen run)".into());
    }
    let mut options = ConvertOptions::default();
    let no_lag = env::args_os().any(|arg| arg == "--no-infer-packet-lag");
    options.infer_packet_lag = !no_lag;
    // The residuals at packets measure prediction; the boundary-value solve and the next-packet fits use
    // those packets, so they are off here.
    options.air_bvp = false;
    options.fit_on_next_packet = false;
    let mut groups: BTreeMap<String, Group> = BTreeMap::new();
    // Parity of each car's dodge counter at its previous residual, to spot the first packet after
    // an activation.
    // Keyed by car lifetime (actor id and creation frame) and cleared per replay, so an id that is
    // reused or appears in the next replay does not inherit another car's previous packet.
    let mut previous_dodge_parity: std::collections::HashMap<(i32, usize), bool> =
        Default::default();
    let mut previous_jump_parity: BTreeMap<(i32, usize), bool> = BTreeMap::new();
    let mut previous_altitude: std::collections::HashMap<(i32, usize), f32> = Default::default();
    let mut skipped = 0usize;
    let (mut activations, mut fitted) = (0usize, 0usize);
    let mut used = 0usize;

    for replay_path in replay_paths(&path, env::args_os().any(|arg| arg == "--final-assessment"))? {
        let output = convert_bytes(&fs::read(&replay_path)?, &options)?;
        previous_dodge_parity.clear();
        previous_jump_parity.clear();
        previous_altitude.clear();
        activations += output.diagnostics.dodge_activations;
        fitted += output.diagnostics.dodge_starts_fitted;
        let frames = &output.observations.frames;
        for residual in &output.position_residuals {
            let converted = &output.frames[residual.frame];
            let frame = &frames[residual.frame];
            let chain = |actor: Option<i32>| {
                converted
                    .packet_lags
                    .iter()
                    .any(|lag| lag.actor_id == actor && lag.source == "chain")
            };
            if !no_lag && !chain(residual.actor_id) {
                skipped += 1;
                continue;
            }
            used += 1;
            let ball = frame
                .ball
                .as_ref()
                .and_then(|b| b.position.as_ref())
                .map(|p| p.value);
            let label = if let Some(actor) = residual.actor_id {
                let Some(car) = frame.cars.iter().find(|c| c.actor_id == actor) else {
                    continue;
                };
                let position = car.body.position.as_ref().map(|p| p.value);
                let previous = frames
                    .get(residual.frame.wrapping_sub(1))
                    .and_then(|f| f.cars.iter().find(|c| c.actor_id == actor));
                let active_counter = |c: &Car| {
                    odd(&c.inputs.jump_active_raw)
                        || odd(&c.inputs.double_jump_active_raw)
                        || odd(&c.inputs.dodge_active_raw)
                        || odd(&c.inputs.flip_car_active_raw)
                };
                let dodge_odd_now = odd(&car.inputs.dodge_active_raw);
                let lifetime = (actor, car.actor_created_frame);
                let first_after_activation =
                    dodge_odd_now && previous_dodge_parity.get(&lifetime).copied() == Some(false);
                previous_dodge_parity.insert(lifetime, dodge_odd_now);
                let jump_odd_now = odd(&car.inputs.jump_active_raw);
                let first_jump =
                    jump_odd_now && previous_jump_parity.get(&lifetime).copied() == Some(false);
                previous_jump_parity.insert(lifetime, jump_odd_now);
                let previous_z =
                    previous_altitude.insert(lifetime, residual.altitude_z.unwrap_or(f32::NAN));
                let flipping = active_counter(car) || previous.is_some_and(active_counter);
                let subtype = if first_after_activation {
                    match previous_z {
                        Some(z) if z < 50.0 => "CAR first packet after dodge, previous z < 50",
                        Some(z) if z < 120.0 => "CAR first packet after dodge, previous z 50-120",
                        Some(z) if z < 300.0 => "CAR first packet after dodge, previous z 120-300",
                        Some(z) if z >= 300.0 => "CAR first packet after dodge, previous z >= 300",
                        _ => "CAR first packet after dodge, previous z unknown",
                    }
                } else if odd(&car.inputs.dodge_active_raw) {
                    "CAR dodge counter odd"
                } else if odd(&car.inputs.double_jump_active_raw) {
                    "CAR double-jump counter odd"
                } else if odd(&car.inputs.flip_car_active_raw) {
                    "CAR flip-car counter odd"
                } else if first_jump {
                    match previous_z {
                        Some(z) if z < 50.0 => "CAR first packet after jump start, previous z < 50",
                        _ => "CAR first packet after jump start, previous z >= 50",
                    }
                } else if odd(&car.inputs.jump_active_raw) {
                    "CAR jump counter odd"
                } else {
                    "CAR counter odd in previous frame only"
                };
                let near_ball =
                    matches!((position, ball), (Some(c), Some(b)) if dist(c, b) < 300.0);
                let near_car = frame.cars.iter().any(|other| {
                    other.actor_id != actor
                        && matches!((position, other.body.position.as_ref()),
                            (Some(c), Some(o)) if dist(c, o.value) < 300.0)
                });
                let boosting = odd(&car.inputs.boost_active_raw);
                let ground = residual.is_on_ground == Some(true);
                let altitude = residual.altitude_z.unwrap_or(f32::NAN);
                if flipping {
                    subtype
                } else if near_ball {
                    "CAR near ball (<300)"
                } else if near_car {
                    "CAR near other car (<300)"
                } else if ground && altitude < 50.0 {
                    if boosting {
                        "CAR ground boosting"
                    } else {
                        "CAR ground no boost"
                    }
                } else if ground {
                    if boosting {
                        "CAR wall/ramp boosting"
                    } else {
                        "CAR wall/ramp no boost"
                    }
                } else if boosting {
                    "CAR air boosting"
                } else {
                    "CAR air no boost"
                }
            } else {
                let near_car = ball.is_some_and(|b| {
                    frame.cars.iter().any(|c| {
                        c.body
                            .position
                            .as_ref()
                            .is_some_and(|p| dist(p.value, b) < 300.0)
                    })
                });
                let z = residual.altitude_z.unwrap_or(f32::NAN);
                if near_car {
                    "BALL near car (<300)"
                } else if z > 200.0 {
                    "BALL high air"
                } else {
                    "BALL low (ground/bounce/wall)"
                }
            };
            let kind = if residual.actor_id.is_some() {
                "CAR all"
            } else {
                "BALL all"
            };
            for name in [label, kind] {
                let group = groups.entry(name.to_string()).or_default();
                group.position.push(residual.simulated_error_uu);
                if let Some(v) = residual.simulated_velocity_error_uu_per_sec {
                    group.velocity.push(v);
                }
                if let Some(v) = residual.simulated_rotation_error_degrees {
                    group.rotation.push(v);
                }
                if let Some(v) = residual.simulated_angular_velocity_error_rad_per_sec {
                    group.angular.push(v);
                }
            }
        }
    }

    println!("residuals used {used}, skipped without a chain lag {skipped}");
    println!("dodge activations {activations}, start ticks fitted {fitted}");
    let sum_sq = |values: &[f32]| values.iter().map(|&v| f64::from(v).powi(2)).sum::<f64>();
    let totals: BTreeMap<&str, [f64; 4]> = ["CAR all", "BALL all"]
        .iter()
        .map(|&name| {
            let g = groups.get(name);
            (
                name,
                [
                    g.map_or(0.0, |g| sum_sq(&g.position)),
                    g.map_or(0.0, |g| sum_sq(&g.velocity)),
                    g.map_or(0.0, |g| sum_sq(&g.rotation)),
                    g.map_or(0.0, |g| sum_sq(&g.angular)),
                ],
            )
        })
        .collect();
    println!(
        "{:<32} {:>8} | {:>16} {:>6} | {:>14} {:>6} | {:>18} {:>6} | {:>13} {:>6}",
        "partition",
        "n",
        "pos UU p50/90/99",
        "SS",
        "vel p50/90",
        "SS",
        "rot deg p50/90/99",
        "SS",
        "ang p50/90",
        "SS"
    );
    for (label, group) in groups.iter_mut() {
        let kind = if label.starts_with("CAR") {
            "CAR all"
        } else {
            "BALL all"
        };
        let total = totals[kind];
        let share = |sum: f64, index: usize| {
            if total[index] > 0.0 {
                sum / total[index]
            } else {
                f64::NAN
            }
        };
        let n = group.position.len();
        let sums = [
            sum_sq(&group.position),
            sum_sq(&group.velocity),
            sum_sq(&group.rotation),
            sum_sq(&group.angular),
        ];
        println!(
            "{:<32} {:>8} | {:>5.1}/{:>4.1}/{:>5.1} {:>6.3} | {:>6.1}/{:>6.1} {:>6.3} | {:>5.2}/{:>5.2}/{:>5.1} {:>6.3} | {:>5.2}/{:>5.2} {:>6.3}",
            label,
            n,
            quantile(&mut group.position, 0.5),
            quantile(&mut group.position, 0.9),
            quantile(&mut group.position, 0.99),
            share(sums[0], 0),
            quantile(&mut group.velocity, 0.5),
            quantile(&mut group.velocity, 0.9),
            share(sums[1], 1),
            quantile(&mut group.rotation, 0.5),
            quantile(&mut group.rotation, 0.9),
            quantile(&mut group.rotation, 0.99),
            share(sums[2], 2),
            quantile(&mut group.angular, 0.5),
            quantile(&mut group.angular, 0.9),
            share(sums[3], 3),
        );
    }
    Ok(())
}
