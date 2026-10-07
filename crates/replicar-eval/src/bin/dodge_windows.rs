//! The worst prediction errors at the first car update after a dodge activation whose previous update was
//! near the ground (`error_budget`'s "first packet after dodge, previous z < 50"), each with its window: per
//! frame the dodge and jump counters (value and frame of change), height and vertical velocity of the update,
//! the update's applied ticks and source, and the fitted presses of the car's player. For inspection before
//! any correction (AGENTS.md: inspect short train windows frame by frame).
//!
//! usage: `dodge_windows <replay or folder>... [--worst N] [--summary]`

use std::collections::HashMap;
use std::path::PathBuf;
use std::process::ExitCode;

use replicar_eval::collect_replays;
use replicar_v1::observations::{Car, Value};

fn odd(v: &Option<Value<u8>>) -> bool {
    v.as_ref().is_some_and(|v| v.value % 2 == 1)
}

fn main() -> ExitCode {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let worst = args
        .iter()
        .position(|a| a == "--worst")
        .and_then(|i| args.get(i + 1))
        .and_then(|n| n.parse().ok())
        .unwrap_or(10usize);
    let summary = args.iter().any(|a| a == "--summary");
    let mut skip = false;
    args.retain(|a| {
        if skip {
            skip = false;
            return false;
        }
        if a == "--worst" {
            skip = true;
            return false;
        }
        a != "--summary"
    });
    let roots: Vec<PathBuf> = args.iter().map(PathBuf::from).collect();
    let Ok(replays) = collect_replays(&roots, false) else {
        eprintln!("error: cannot list the replays");
        return ExitCode::FAILURE;
    };
    let options = replicar_v1::conversion::ConvertOptions::default();
    // (error, description)
    let mut cases: Vec<(f32, String)> = Vec::new();
    // Error by the gap between the dodge counter's change frame and the previous update, and by the jump
    // counter's state.
    let mut by_kind: HashMap<String, Vec<f32>> = HashMap::new();
    // (vertical error, horizontal error) per kind.
    let mut components: HashMap<String, Vec<(f32, f32)>> = HashMap::new();
    for path in &replays {
        let Ok(bytes) = std::fs::read(path) else {
            continue;
        };
        let Ok(output) = replicar_eval::v2_conversion::convert_bytes(&bytes, &options) else {
            continue;
        };
        let frames = &output.observations.frames;
        let mut dodge_parity: HashMap<(i32, usize), bool> = HashMap::new();
        let mut altitude: HashMap<(i32, usize), (usize, f32)> = HashMap::new();
        for residual in &output.position_residuals {
            let Some(actor) = residual.actor_id else {
                continue;
            };
            let frame = &frames[residual.frame];
            let Some(car) = frame.cars.iter().find(|c| c.actor_id == actor) else {
                continue;
            };
            let life = (actor, car.actor_created_frame);
            let dodge_now = odd(&car.inputs.dodge_active_raw);
            let first = dodge_now && dodge_parity.get(&life).copied() == Some(false);
            dodge_parity.insert(life, dodge_now);
            let previous = altitude.insert(
                life,
                (residual.frame, residual.altitude_z.unwrap_or(f32::NAN)),
            );
            let Some((previous_frame, previous_z)) = previous else {
                continue;
            };
            if !first || previous_z >= 50.0 {
                continue;
            }
            let error = residual.simulated_error_uu;
            // Where the real car is at this update: still resting on the ground, or already off it.
            let resting = matches!(
                (car.body.position.as_ref(), car.body.linear_velocity.as_ref()),
                (Some(p), Some(v)) if p.value[2] < 20.0 && v.value[2].abs() < 10.0
            );
            // Whether the simulation is above the update (it left the ground earlier) or below it.
            let simulated_higher = residual.simulated_error_vector_uu[2] > 1.0;
            let kind = format!(
                "real car {} at this update; simulated z {} the update",
                if resting {
                    "still resting on the ground"
                } else {
                    "off the ground"
                },
                if simulated_higher {
                    "above"
                } else {
                    "at or below"
                },
            );
            let v = residual.simulated_error_vector_uu;
            components
                .entry(kind.clone())
                .or_default()
                .push((v[2], v[0].hypot(v[1])));
            by_kind.entry(kind).or_default().push(error);
            let slot = output.frames[residual.frame]
                .car_actor_slots
                .iter()
                .find(|(a, _)| *a == actor)
                .map(|(_, s)| *s);
            let mut text = format!(
                "{error:.1} UU  {}  frame {} car {actor} (slot {slot:?})  error {:?}  previous update frame {previous_frame} z {previous_z:.1}\n",
                path.display(),
                residual.frame,
                residual.simulated_error_vector_uu
            );
            for g in previous_frame.saturating_sub(1)..=(residual.frame + 1).min(frames.len() - 1) {
                let Some(c) = frames[g].cars.iter().find(|c| c.actor_id == actor) else {
                    continue;
                };
                let counter = |v: &Option<Value<u8>>| {
                    v.as_ref()
                        .map_or("-".to_owned(), |v| format!("{}@{}", v.value, v.frame))
                };
                let body =
                    |c: &Car| match (c.body.position.as_ref(), c.body.linear_velocity.as_ref()) {
                        (Some(p), Some(v)) if p.frame == g => {
                            format!("z {:7.1} vz {:7.1}", p.value[2], v.value[2])
                        }
                        _ => "(no update)        ".to_owned(),
                    };
                let converted = &output.frames[g];
                let lag = converted
                    .packet_lags
                    .iter()
                    .find(|l| l.actor_id == Some(actor))
                    .map_or(String::new(), |l| format!("{} {}", l.ticks, l.source));
                let fitted: Vec<String> = converted
                    .fitted_inputs
                    .iter()
                    .filter(|f| Some(f.slot) == slot && f.kind != "air")
                    .map(|f| {
                        format!(
                            "{} @{} p{:.2} y{:.2} c{:.2}",
                            f.kind, f.tick, f.pitch, f.yaw, f.cancel
                        )
                    })
                    .collect();
                text += &format!(
                    "    frame {g} tick {:6} | jump {:>7} dodge {:>7} | {} | lag {lag:12} | {}\n",
                    converted.timeline_tick,
                    counter(&c.inputs.jump_active_raw),
                    counter(&c.inputs.dodge_active_raw),
                    body(c),
                    fitted.join("; ")
                );
            }
            cases.push((error, text));
        }
    }
    let mut kinds: Vec<_> = by_kind.into_iter().collect();
    kinds.sort_by_key(|k| std::cmp::Reverse(k.1.len()));
    for (kind, mut errors) in kinds {
        errors.sort_by(f32::total_cmp);
        let q = |p: f64| errors[((errors.len() - 1) as f64 * p) as usize];
        let ss: f64 = errors.iter().map(|e| f64::from(*e) * f64::from(*e)).sum();
        println!(
            "{:6} cases  p50 {:6.1}  p90 {:6.1}  sum sq {:10.0}  {kind}",
            errors.len(),
            q(0.5),
            q(0.9),
            ss
        );
        let mut c = components.remove(&kind).unwrap_or_default();
        let n = c.len().max(1) as f32;
        let (mean_z, mean_xy) = c
            .iter()
            .fold((0.0, 0.0), |a, x| (a.0 + x.0 / n, a.1 + x.1 / n));
        c.sort_by(|a, b| a.0.total_cmp(&b.0));
        let median_z = c.get(c.len() / 2).map_or(0.0, |x| x.0);
        println!(
            "         vertical error (simulated minus real) mean {mean_z:6.1} median {median_z:6.1}; horizontal mean {mean_xy:6.1}"
        );
    }
    if !summary {
        cases.sort_by(|a, b| b.0.total_cmp(&a.0));
        for (_, text) in cases.iter().take(worst) {
            println!("\n{text}");
        }
    }
    ExitCode::SUCCESS
}
