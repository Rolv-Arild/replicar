//! When does the physical flip start relative to the dodge counter, and does RocketSim's flip
//! reproduce the replay's angular-velocity ramp? Train replays only.
//!
//! For each dodge activation (dodge counter even to odd with a fresh `DodgeTorque`) in the air
//! with a fresh car packet shortly before and at least three shortly after, start RocketSim from
//! the earlier packet, trigger the dodge at every candidate tick D, and compare simulated angular
//! velocity with each later packet at its inferred physical tick (frame time minus the inferred
//! packet lag). Reports the best D relative to the activation frame's time, the fit error at the
//! best D and at D = the activation frame time (what the converter assumes), and the error
//! profile after the best start.

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Quat, Vec3A};
use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes};
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, CarState, GameMode, Team};

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

struct Packet {
    tick: f32,
    omega: Vec3A,
}

/// RocketSim limits angular speed at the start of the next tick; replay states obey the limit.
fn clamp_angular_speed(arena: &mut Arena, slot: usize) {
    let mut state = *arena.get_car_state(slot);
    let speed = state.phys.ang_vel.length();
    if speed > 5.5 {
        state.phys.ang_vel *= 5.5 / speed;
        arena.set_car_state(slot, state);
    }
}

fn quantile(values: &mut [f64], q: f64) -> f64 {
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: diagnose_dodge_start <train dir or replay>")?,
    );
    if replay_to_rocketsim::sealed_path_refused(&path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let max_events: usize = env::args_os()
        .nth(2)
        .and_then(|v| v.to_string_lossy().parse().ok())
        .unwrap_or(usize::MAX);
    let options = ConvertOptions::default();
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    let slot = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);

    let mut best_offsets = Vec::new(); // best D minus activation frame tick
    let mut fit_best = Vec::new();
    let mut fit_at_activation = Vec::new();
    let mut by_offset: BTreeMap<i64, usize> = BTreeMap::new();
    let mut profile: BTreeMap<usize, Vec<f64>> = BTreeMap::new(); // packet index after start -> error
    let mut events = 0usize;

    'replays: for replay_path in replay_paths(&path)? {
        let output = convert_bytes(&fs::read(&replay_path)?, &options)?;
        let frames = &output.observations.frames;
        let mut last_dodge: BTreeMap<i32, u8> = BTreeMap::new();
        for (index, frame) in frames.iter().enumerate() {
            for car in &frame.cars {
                let Some(dodge) = car
                    .inputs
                    .dodge_active_raw
                    .as_ref()
                    .filter(|d| d.frame == index)
                else {
                    continue;
                };
                let previous = last_dodge.insert(car.actor_id, dodge.value);
                let rising = previous.is_some_and(|p| p % 2 == 0) && dodge.value % 2 == 1;
                let Some(torque) = car
                    .inputs
                    .dodge_torque_replay_units
                    .as_ref()
                    .filter(|t| t.frame == index)
                else {
                    continue;
                };
                if !rising {
                    continue;
                }
                // Fresh car packets of this actor lifetime around the event.
                let packet_at = |f: usize| -> Option<(Packet, CarState)> {
                    let c = frames[f].cars.iter().find(|c| {
                        c.actor_id == car.actor_id
                            && c.actor_created_frame == car.actor_created_frame
                    })?;
                    let b = &c.body;
                    let p = b.position.as_ref().filter(|x| x.frame == f)?;
                    let v = b.linear_velocity.as_ref().filter(|x| x.frame == f)?;
                    let r = b.rotation_xyzw.as_ref().filter(|x| x.frame == f)?;
                    let w = b
                        .angular_velocity_replay_units
                        .as_ref()
                        .filter(|x| x.frame == f)?;
                    let lag = output.frames[f]
                        .packet_lags
                        .iter()
                        .find(|l| l.actor_id == Some(c.actor_id))?;
                    if lag.source != "chain" {
                        return None;
                    }
                    let quat = Quat::from_xyzw(r.value[0], r.value[1], r.value[2], r.value[3]);
                    let mut state = CarState::default();
                    state.phys.pos = Vec3A::from_array(p.value);
                    state.phys.vel = Vec3A::from_array(v.value);
                    state.phys.ang_vel = Vec3A::from_array(w.value) * 0.01;
                    state.phys.rot_mat = Mat3A::from_quat(quat.normalize());
                    state.is_on_ground = false;
                    Some((
                        Packet {
                            tick: frames[f].time * 120.0 - lag.ticks as f32,
                            omega: state.phys.ang_vel,
                        },
                        state,
                    ))
                };
                let Some((pre, pre_state)) = (index.saturating_sub(6)..index)
                    .rev()
                    .find_map(|f| packet_at(f))
                else {
                    continue;
                };
                if pre_state.phys.pos.z < 150.0 {
                    continue;
                }
                let post: Vec<Packet> = (index..=(index + 14).min(frames.len() - 1))
                    .filter_map(|f| packet_at(f).map(|(p, _)| p))
                    .collect();
                if post.len() < 3 {
                    continue;
                }
                // Observed controls at the activation: stick from the calibrated torque mapping.
                let pitch = -torque.value[1] / 2.24;
                let yaw = -torque.value[0] / 2.60;
                let activation_tick = f64::from(frames[index].time * 120.0 - pre.tick);
                let horizon = post
                    .last()
                    .map(|p| (p.tick - pre.tick).ceil() as usize)
                    .unwrap_or(0);
                if horizon > 60 {
                    continue;
                }
                let mut best: Option<(i64, f64, Vec<f64>)> = None;
                let mut at_activation = f64::NAN;
                for start in 0..=(horizon.min(40) as i64) {
                    let mut state = pre_state;
                    state.has_jumped = true;
                    state.is_jumping = false;
                    state.air_time_since_jump = 0.05;
                    arena.set_car_state(slot, state);
                    arena.set_car_controls(slot, CarControls::default());
                    let mut errors = vec![f64::NAN; post.len()];
                    for tick in 1..=horizon {
                        let controls = if tick as i64 == start {
                            CarControls {
                                jump: true,
                                pitch: pitch.clamp(-1.0, 1.0),
                                yaw: yaw.clamp(-1.0, 1.0),
                                ..CarControls::default()
                            }
                        } else {
                            CarControls::default()
                        };
                        arena.set_car_controls(slot, controls);
                        arena.step_tick();
                        clamp_angular_speed(&mut arena, slot);
                        for (i, packet) in post.iter().enumerate() {
                            if (packet.tick - pre.tick).round() as usize == tick {
                                errors[i] = f64::from(
                                    (arena.get_car_state(slot).phys.ang_vel - packet.omega)
                                        .length(),
                                );
                            }
                        }
                    }
                    let valid: Vec<f64> = errors.iter().copied().filter(|e| !e.is_nan()).collect();
                    if valid.len() < 3 {
                        continue;
                    }
                    let score = valid.iter().map(|e| e * e).sum::<f64>() / valid.len() as f64;
                    if best.as_ref().is_none_or(|(_, s, _)| score < *s) {
                        best = Some((start, score, errors.clone()));
                    }
                    if (start as f64 - activation_tick).abs() < 0.5 {
                        at_activation = score;
                    }
                }
                let Some((start, score, errors)) = best else {
                    continue;
                };
                let offset = start as f64 - activation_tick;
                best_offsets.push(offset);
                *by_offset.entry(offset.round() as i64).or_default() += 1;
                fit_best.push(score.sqrt());
                if !at_activation.is_nan() {
                    fit_at_activation.push(at_activation.sqrt());
                }
                for (i, e) in errors.iter().enumerate() {
                    if !e.is_nan() {
                        profile.entry(i).or_default().push(*e);
                    }
                }
                events += 1;
                if events >= max_events {
                    break 'replays;
                }
            }
        }
    }

    println!("dodge events with an exact packet chain before and after: {events}");
    println!(
        "best flip start minus activation frame time (ticks): p10 {:.1} p25 {:.1} p50 {:.1} p75 {:.1} p90 {:.1}",
        quantile(&mut best_offsets.clone(), 0.1),
        quantile(&mut best_offsets.clone(), 0.25),
        quantile(&mut best_offsets.clone(), 0.5),
        quantile(&mut best_offsets.clone(), 0.75),
        quantile(&mut best_offsets.clone(), 0.9)
    );
    let mut counts: Vec<String> = Vec::new();
    for (offset, count) in &by_offset {
        if (-12..=24).contains(offset) {
            counts.push(format!("{offset}:{count}"));
        }
    }
    println!(
        "histogram of that offset (ticks: count): {}",
        counts.join(" ")
    );
    println!(
        "RMS angular-velocity error (rad/s) at the best start: p50 {:.2} p90 {:.2}; starting at the activation frame time: p50 {:.2} p90 {:.2}",
        quantile(&mut fit_best.clone(), 0.5),
        quantile(&mut fit_best.clone(), 0.9),
        quantile(&mut fit_at_activation.clone(), 0.5),
        quantile(&mut fit_at_activation.clone(), 0.9)
    );
    println!("error at the best start, by packet index after activation (rad/s p50/p90):");
    for (i, values) in profile.iter_mut() {
        if values.len() >= 20 {
            println!(
                "  packet {i}: n={} p50 {:.2} p90 {:.2}",
                values.len(),
                quantile(values, 0.5),
                quantile(values, 0.9)
            );
        }
    }
    Ok(())
}
