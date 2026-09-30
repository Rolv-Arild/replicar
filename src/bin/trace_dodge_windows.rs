//! Frame-by-frame windows around dodge activations, from the converter's own output. Train/validation.
//!
//! A dodge activation is a frame where a car's dodge counter turns odd. The event's score is the
//! largest position error (pre-correction residual, UU) at a fresh packet in the frames from the
//! activation to the next dodge counter change or 8 frames. Prints windows for events at several
//! score percentiles, with the counters and their freshness stamps, the dodge torque, the
//! converter's exported state and the converter's own residuals (position, velocity, rotation and
//! angular velocity before correction) at the packets.
//!
//! usage: trace_dodge_windows <dir or replay> [per-percentile count, default 2] [--no-fit-jump-timing]

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
    /// Altitude bucket of the last exported state before the activation.
    low: bool,
    text: String,
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: trace_dodge_windows <dir or replay> [count]")?,
    );
    if path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let per_percentile: usize = env::args().nth(2).and_then(|v| v.parse().ok()).unwrap_or(2);
    let mut options = ConvertOptions::default();
    if env::args().any(|arg| arg == "--no-fit-jump-timing") {
        options.fit_jump_timing = false;
    }
    options.infer_dodge_start = !env::args().any(|arg| arg == "--no-infer-dodge-start");
    options.flip_cancel_holdout = env::args().any(|arg| arg == "--flip-cancel-holdout");
    if let Some(name) = env::args()
        .skip_while(|a| a != "--flip-cancel-source")
        .nth(1)
    {
        if let Some(source) = replay_to_rocketsim::conversion::FlipCancelSource::from_name(&name) {
            options.flip_cancel_source = source;
        }
    }
    if let Some(n) = env::args()
        .skip_while(|a| a != "--flip-cancel-packets")
        .nth(1)
    {
        options.flip_cancel_packets = n.parse().unwrap_or(1);
    }
    options.infer_flip_cancel = !env::args().any(|arg| arg == "--no-infer-flip-cancel");
    let event_lines = env::args().any(|arg| arg == "--event-lines");
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
                let Some(counter) = car
                    .inputs
                    .dodge_active_raw
                    .as_ref()
                    .filter(|c| c.frame == f)
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
                let Some(before) = f.checked_sub(1).and_then(exported) else {
                    continue;
                };
                let residual_at = |g: usize| {
                    output
                        .position_residuals
                        .iter()
                        .find(|r| r.frame == g && r.actor_id == Some(car.actor_id))
                };
                let lag_of = |g: usize| {
                    output.frames[g]
                        .packet_lags
                        .iter()
                        .find(|l| l.actor_id == Some(car.actor_id))
                        .map(|l| format!("lag {}({})", l.ticks, &l.source[..1]))
                        .unwrap_or_default()
                };
                let mut score = 0.0f32;
                let mut scoring = true;
                let mut first: Option<(usize, f32, f32, f32, f32)> = None;
                let mut seq: Vec<String> = Vec::new();
                let mut lines = String::new();
                if let Some(torque) = car
                    .inputs
                    .dodge_torque_replay_units
                    .as_ref()
                    .filter(|t| t.frame == f)
                {
                    lines += &format!(
                        "  dodge torque (replay units) {:?}\n",
                        torque.value.map(|x| (x * 100.0).round() / 100.0)
                    );
                }
                for g in f.saturating_sub(3)..=(f + 8).min(frames.len() - 1) {
                    let Some(other) = frames[g].cars.iter().find(|c| {
                        c.actor_id == car.actor_id
                            && c.actor_created_frame == car.actor_created_frame
                    }) else {
                        continue;
                    };
                    let Some(state) = exported(g) else { continue };
                    if g > f
                        && other
                            .inputs
                            .dodge_active_raw
                            .as_ref()
                            .is_some_and(|v| v.value >= counter.value.wrapping_add(2))
                    {
                        scoring = false;
                    }
                    let residual = residual_at(g);
                    if let Some(r) = residual {
                        if g >= f && scoring {
                            score = score.max(r.simulated_error_uu);
                            if seq.len() < 4 {
                                seq.push(format!(
                                    "{:.2},{:.1},{:.2},{:.3}",
                                    r.simulated_error_uu,
                                    r.simulated_velocity_error_uu_per_sec.unwrap_or(f32::NAN),
                                    r.simulated_rotation_error_degrees.unwrap_or(f32::NAN),
                                    r.simulated_angular_velocity_error_rad_per_sec
                                        .unwrap_or(f32::NAN)
                                ));
                            }
                            if first.is_none() {
                                first = Some((
                                    g,
                                    r.simulated_error_uu,
                                    r.simulated_velocity_error_uu_per_sec.unwrap_or(f32::NAN),
                                    r.simulated_rotation_error_degrees.unwrap_or(f32::NAN),
                                    r.simulated_angular_velocity_error_rad_per_sec
                                        .unwrap_or(f32::NAN),
                                ));
                            }
                        }
                    }
                    let stamp = |frame: Option<usize>| {
                        frame.map_or("-".to_string(), |x| {
                            if x == g {
                                "fresh".to_string()
                            } else {
                                format!("f{x}")
                            }
                        })
                    };
                    let raw = |v: &Option<replay_to_rocketsim::observations::Value<u8>>| {
                        (
                            v.as_ref().map_or(-1, |v| i32::from(v.value)),
                            stamp(v.as_ref().map(|v| v.frame)),
                        )
                    };
                    let (jump, jump_s) = raw(&other.inputs.jump_active_raw);
                    let (dodge, dodge_s) = raw(&other.inputs.dodge_active_raw);
                    let (double, double_s) = raw(&other.inputs.double_jump_active_raw);
                    let fresh = other.body.position.as_ref().filter(|p| p.frame == g).zip(
                        other
                            .body
                            .angular_velocity_replay_units
                            .as_ref()
                            .filter(|w| w.frame == g),
                    );
                    let observed = fresh.map_or(String::new(), |(p, w)| {
                        format!(
                            "| packet z {:>6.1} |w| {:>4.2} ({})",
                            p.value[2],
                            (glam::Vec3A::from_array(w.value) * 0.01).length(),
                            lag_of(g)
                        )
                    });
                    let residual_text = residual.map_or(String::new(), |r| {
                        format!(
                            " | RESIDUAL pos {:.1} vel {:.0} rot {:.1} deg ang {:.2}",
                            r.simulated_error_uu,
                            r.simulated_velocity_error_uu_per_sec.unwrap_or(f32::NAN),
                            r.simulated_rotation_error_degrees.unwrap_or(f32::NAN),
                            r.simulated_angular_velocity_error_rad_per_sec
                                .unwrap_or(f32::NAN)
                        )
                    });
                    lines += &format!(
                        "  frame {g} tick {:>6}{} jump {jump:>3}@{jump_s:<6} dbl {double:>3}@{double_s:<6} dodge {dodge:>3}@{dodge_s:<6} | sim z {:>6.1} vz {:>6.1} |w| {:>4.2} ground {} flipping {} flip_time {:.3} ctl.jump {} pitch {:.2} yaw {:.2} {observed}{residual_text}\n",
                        tick_of(g),
                        if g == f { " (activation)" } else { "" },
                        state.phys.pos.z,
                        state.phys.vel.z,
                        state.phys.ang_vel.length(),
                        u8::from(state.is_on_ground),
                        u8::from(state.is_flipping),
                        state.flip_time,
                        u8::from(state.controls.jump),
                        state.controls.pitch,
                        state.controls.yaw,
                    );
                }
                if event_lines {
                    if let Some((g, pos, vel, rot, ang)) = first {
                        println!(
                            "EVT {name} {} {f} {g} {pos:.2} {vel:.1} {rot:.2} {ang:.3} {:.0} {}",
                            car.actor_id,
                            before.phys.pos.z,
                            seq.join(" ")
                        );
                    }
                }
                events.push(Event {
                    score,
                    low: before.phys.pos.z < 50.0,
                    text: format!(
                        "{name} car {} activation at frame {f}, height before {:.0} (score {score:.1} UU)\n{lines}",
                        car.actor_id, before.phys.pos.z
                    ),
                });
            }
        }
    }

    // --focus <replay name>:<activation frame> prints just that event.
    if let Some(focus) = env::args().skip_while(|a| a != "--focus").nth(1) {
        let (name, frame) = focus.split_once(':').ok_or("--focus name:frame")?;
        for event in &events {
            if event.text.starts_with(name)
                && event
                    .text
                    .contains(&format!("activation at frame {frame},"))
            {
                println!("{}", event.text);
            }
        }
        return Ok(());
    }
    events.sort_by(|a, b| a.score.total_cmp(&b.score));
    println!("dodge activations: {}", events.len());
    let q = |v: &Vec<f32>, p: f64| v[((v.len() - 1) as f64 * p).round() as usize];
    for (label, filter) in [
        ("all", None),
        ("previous z < 50", Some(true)),
        ("previous z >= 50", Some(false)),
    ] {
        let scores: Vec<f32> = events
            .iter()
            .filter(|e| filter.is_none_or(|l| e.low == l))
            .map(|e| e.score)
            .collect();
        if scores.is_empty() {
            continue;
        }
        println!(
            "  {label}: n {} max position error p50/p75/p90/p95/p99 {:.1}/{:.1}/{:.1}/{:.1}/{:.1} UU",
            scores.len(),
            q(&scores, 0.5),
            q(&scores, 0.75),
            q(&scores, 0.9),
            q(&scores, 0.95),
            q(&scores, 0.99)
        );
    }
    for p in [0.5, 0.75, 0.9, 0.95, 0.99] {
        let center = ((events.len() - 1) as f64 * p).round() as usize;
        println!("\n=== windows at the {:.0}th percentile ===", p * 100.0);
        for k in 0..per_percentile {
            let index = (center + k * 37).min(events.len() - 1);
            println!("{}", events[index].text);
        }
    }
    Ok(())
}
