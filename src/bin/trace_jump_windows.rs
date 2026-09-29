//! Frame-by-frame windows around jump activations, from the converter's own output. Train/validation.
//!
//! A jump activation is a frame where a car's jump counter turns odd. For every activation of a
//! grounded car, the largest vertical-velocity error between the converter's exported state and a
//! fresh car packet in the following frames is the event's score. Prints windows for events at
//! several score percentiles (median, 75, 90, 95, 99), so both the typical and the failing cases
//! can be read: the counter values with freshness stamps, the exported state (height, vertical
//! velocity, ground/jump flags, jump control) and, where a fresh packet exists, the packet's height
//! and vertical velocity at its chain-inferred physical tick.
//!
//! usage: trace_jump_windows <dir or replay> [per-percentile count, default 2]

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes};

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

struct Event {
    score: f32,
    /// A dodge counter change within 8 frames after the activation.
    dodge_soon: bool,
    text: String,
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: trace_jump_windows <dir or replay> [count]")?,
    );
    if path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let per_percentile: usize = env::args().nth(2).and_then(|v| v.parse().ok()).unwrap_or(2);
    let mut options = ConvertOptions::default();
    options.fit_jump_timing = env::args().any(|arg| arg == "--fit-jump-timing");
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut events: Vec<Event> = Vec::new();

    for replay_path in replay_paths(&path)? {
        let output = convert_bytes(&fs::read(&replay_path)?, &options)?;
        let frames = &output.observations.frames;
        let first_time = frames[0].time;
        let tick_of = |f: usize| -> i64 {
            ((f64::from(frames[f].time) - f64::from(first_time)) * 120.0).round() as i64
        };
        let mut last_counter: BTreeMap<(i32, usize), u8> = BTreeMap::new();
        let name = replay_path
            .file_name()
            .unwrap()
            .to_string_lossy()
            .into_owned();
        for f in 0..frames.len() {
            for car in &frames[f].cars {
                let key = (car.actor_id, car.actor_created_frame);
                let Some(counter) = car.inputs.jump_active_raw.as_ref().filter(|c| c.frame == f)
                else {
                    continue;
                };
                let previous = last_counter.insert(key, counter.value);
                let Some(previous) = previous else { continue };
                if previous % 2 == 1 || counter.value % 2 == 0 {
                    continue;
                }
                let Some(player) = car.player_key.as_ref() else {
                    continue;
                };
                let Some(slot) = output
                    .car_slots
                    .iter()
                    .find(|s| &s.player_key == player)
                    .map(|s| s.slot)
                else {
                    continue;
                };
                let exported = |g: usize| {
                    output.frames[g]
                        .state
                        .cars
                        .iter()
                        .find(|(info, _)| info.idx == slot)
                        .map(|(_, state)| *state)
                };
                // Only jumps from the ground.
                let Some(before) = f.checked_sub(1).and_then(exported) else {
                    continue;
                };
                if before.phys.pos.z > 40.0 {
                    continue;
                }
                let lag_of = |g: usize| {
                    output.frames[g]
                        .packet_lags
                        .iter()
                        .find(|l| l.actor_id == Some(car.actor_id))
                        .map(|l| format!("lag {}({})", l.ticks, &l.source[..1]))
                        .unwrap_or_default()
                };
                let mut score = 0.0f32;
                // Scoring stops at the next jump press or any dodge/double-jump/flip change, so it
                // measures the first jump only.
                let mut scoring = true;
                let dodge_at_start = car.inputs.dodge_active_raw.as_ref().map(|v| v.value);
                let residual_at = |g: usize| {
                    output
                        .position_residuals
                        .iter()
                        .find(|r| r.frame == g && r.actor_id == Some(car.actor_id))
                };
                let mut lines = String::new();
                for g in f.saturating_sub(3)..=(f + 7).min(frames.len() - 1) {
                    let Some(other) = frames[g].cars.iter().find(|c| {
                        c.actor_id == car.actor_id
                            && c.actor_created_frame == car.actor_created_frame
                    }) else {
                        continue;
                    };
                    let Some(state) = exported(g) else { continue };
                    let stamp = |frame: Option<usize>| {
                        frame.map_or("-".to_string(), |x| {
                            if x == g {
                                "fresh".to_string()
                            } else {
                                format!("f{x}")
                            }
                        })
                    };
                    let fresh = other
                        .body
                        .position
                        .as_ref()
                        .filter(|p| p.frame == g)
                        .zip(other.body.linear_velocity.as_ref().filter(|v| v.frame == g));
                    let observed = fresh.map_or(String::new(), |(p, v)| {
                        format!(
                            "| packet z {:>7.1} vz {:>7.1} ({})",
                            p.value[2],
                            v.value[2],
                            lag_of(g)
                        )
                    });
                    let residual = residual_at(g);
                    if g > f
                        && (other
                            .inputs
                            .jump_active_raw
                            .as_ref()
                            .is_some_and(|v| v.value >= counter.value.wrapping_add(2))
                            || other.inputs.dodge_active_raw.as_ref().map(|v| v.value)
                                != dodge_at_start)
                    {
                        scoring = false;
                    }
                    if let Some(r) = residual {
                        if g >= f && scoring {
                            score = score.max(r.simulated_velocity_error_uu_per_sec.unwrap_or(0.0));
                        }
                    }
                    let residual_text = residual.map_or(String::new(), |r| {
                        format!(
                            " | RESIDUAL before correction: pos {:.1} UU (z {:+.1}) vel {:.1} UU/s",
                            r.simulated_error_uu,
                            r.simulated_error_vector_uu[2],
                            r.simulated_velocity_error_uu_per_sec.unwrap_or(f32::NAN)
                        )
                    });
                    lines += &format!(
                        "  frame {g} tick {:>6}{} jump {:>3}@{:<6} dodge {:>3}@{:<6} boost {:>3}@{:<6} | sim z {:>7.1} vz {:>7.1} ground {} jumping {} jump_time {:.3} ctl.jump {} {observed}{residual_text}\n",
                        tick_of(g),
                        if g == f { " (activation)" } else { "" },
                        other
                            .inputs
                            .jump_active_raw
                            .as_ref()
                            .map_or(-1, |v| i32::from(v.value)),
                        stamp(other.inputs.jump_active_raw.as_ref().map(|v| v.frame)),
                        other
                            .inputs
                            .dodge_active_raw
                            .as_ref()
                            .map_or(-1, |v| i32::from(v.value)),
                        stamp(other.inputs.dodge_active_raw.as_ref().map(|v| v.frame)),
                        other
                            .inputs
                            .boost_active_raw
                            .as_ref()
                            .map_or(-1, |v| i32::from(v.value)),
                        stamp(other.inputs.boost_active_raw.as_ref().map(|v| v.frame)),
                        state.phys.pos.z,
                        state.phys.vel.z,
                        u8::from(state.is_on_ground),
                        u8::from(state.is_jumping),
                        state.jump_time(),
                        u8::from(state.controls.jump),
                    );
                }
                let dodge_soon = (f + 1..=(f + 8).min(frames.len() - 1)).any(|g| {
                    frames[g].cars.iter().any(|c| {
                        c.actor_id == car.actor_id
                            && c.inputs.dodge_active_raw.as_ref().map(|v| v.value) != dodge_at_start
                    })
                });
                events.push(Event {
                    score,
                    dodge_soon,
                    text: format!(
                        "{name} car {} activation at frame {f} (score {score:.1} UU/s)\n{lines}",
                        car.actor_id
                    ),
                });
            }
        }
    }

    events.sort_by(|a, b| a.score.total_cmp(&b.score));
    println!("jump activations from the ground: {}", events.len());
    let q = |p: f64| events[((events.len() - 1) as f64 * p).round() as usize].score;
    println!(
        "score (max vertical-velocity error at a fresh packet within 7 frames) p50/p75/p90/p95/p99: {:.1}/{:.1}/{:.1}/{:.1}/{:.1} UU/s",
        q(0.5),
        q(0.75),
        q(0.9),
        q(0.95),
        q(0.99)
    );
    for (label, want) in [
        ("no dodge within 8 frames", false),
        ("dodge within 8 frames", true),
    ] {
        let subset: Vec<f32> = events
            .iter()
            .filter(|e| e.dodge_soon == want)
            .map(|e| e.score)
            .collect();
        if subset.is_empty() {
            continue;
        }
        let sq = |p: f64| subset[((subset.len() - 1) as f64 * p).round() as usize];
        println!(
            "  {label}: n {} score p50/p75/p90/p95 {:.1}/{:.1}/{:.1}/{:.1} UU/s",
            subset.len(),
            sq(0.5),
            sq(0.75),
            sq(0.9),
            sq(0.95)
        );
    }
    for p in [0.5, 0.75, 0.9, 0.95, 0.99] {
        let center = ((events.len() - 1) as f64 * p).round() as usize;
        println!("\n=== windows at the {:.0}th percentile ===", p * 100.0);
        for k in 0..per_percentile {
            // Spread the picks around the percentile position.
            let index = (center + k * 37).min(events.len() - 1);
            println!("{}", events[index].text);
        }
    }
    Ok(())
}
