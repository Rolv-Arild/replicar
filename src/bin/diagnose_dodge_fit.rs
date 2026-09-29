//! Can the dodge start tick be recovered from position and velocity, with rotation as the check?
//!
//! For dodge activations (counter even to odd, fresh `DodgeTorque`) with a chain-lag car packet
//! shortly before (A) and the first chain-lag packet at or after the activation frame (B), start
//! RocketSim from A with the observed throttle and boost and trigger the dodge at every tick D in
//! (A, B]. The start is fitted to B's position and velocity only. Rotation and angular velocity at B
//! are not used, so they measure whether the fitted start is right. Compared with the start that a
//! frame-time model assumes (the activation frame's own tick). For each start the pitch cancel is
//! chosen from angular velocity, as the converter does. Train replays only.

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

fn quantile(values: &mut [f64], q: f64) -> f64 {
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

struct Packet {
    tick: i64,
    state: CarState,
}

fn rotation_error_degrees(a: Mat3A, b: Mat3A) -> f32 {
    let trace = a.x_axis.dot(b.x_axis) + a.y_axis.dot(b.y_axis) + a.z_axis.dot(b.z_axis);
    ((trace - 1.0) * 0.5).clamp(-1.0, 1.0).acos().to_degrees()
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: diagnose_dodge_fit <train dir or replay> [max events]")?,
    );
    if path.to_string_lossy().contains("test") {
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

    // Per event: (fitted, frame-time) for rotation error deg, angular error rad/s, position error UU, velocity UU/s
    let mut rot = (Vec::new(), Vec::new());
    let mut ang = (Vec::new(), Vec::new());
    let mut pos = (Vec::new(), Vec::new());
    let mut vel = (Vec::new(), Vec::new());
    let mut offsets = Vec::new();
    let mut margins = Vec::new();
    let mut events = 0usize;

    'replays: for replay_path in replay_paths(&path)? {
        let output = convert_bytes(&fs::read(&replay_path)?, &options)?;
        let frames = &output.observations.frames;
        let first_time = frames[0].time;
        let timeline =
            |f: usize| ((f64::from(frames[f].time) - f64::from(first_time)) * 120.0).round() as i64;
        let mut last_dodge = std::collections::BTreeMap::<i32, u8>::new();
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
                let Some(torque) = car
                    .inputs
                    .dodge_torque_replay_units
                    .as_ref()
                    .filter(|t| t.frame == index)
                else {
                    continue;
                };
                if !(previous.is_some_and(|p| p % 2 == 0) && dodge.value % 2 == 1) {
                    continue;
                }
                let packet_at =
                    |f: usize| -> Option<(Packet, &replay_to_rocketsim::observations::Car)> {
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
                                tick: timeline(f) - lag.ticks as i64,
                                state,
                            },
                            c,
                        ))
                    };
                // A: last chain packet before the activation frame (airborne); B: first at/after it.
                let Some((a, a_car)) = (index.saturating_sub(6)..index).rev().find_map(packet_at)
                else {
                    continue;
                };
                let Some((b, _)) = (index..=(index + 6).min(frames.len() - 1)).find_map(packet_at)
                else {
                    continue;
                };
                if a.state.phys.pos.z < 120.0 {
                    continue;
                }
                let k = b.tick - a.tick;
                if !(2..=30).contains(&k) {
                    continue;
                }
                let mut pre = a.state;
                pre.has_jumped = true;
                pre.is_jumping = false;
                pre.air_time_since_jump = 0.05;
                let base = CarControls {
                    throttle: a_car.inputs.throttle.as_ref().map_or(0.0, |v| v.value),
                    boost: a_car
                        .inputs
                        .boost_active_raw
                        .as_ref()
                        .is_some_and(|v| v.value % 2 == 1),
                    ..CarControls::default()
                };
                let pitch = (-torque.value[1] / 2.24).clamp(-1.0, 1.0);
                let yaw = (-torque.value[0] / 2.60).clamp(-1.0, 1.0);
                // Simulate every (start D, cancel c); errors at B.
                #[derive(Clone, Copy)]
                struct Outcome {
                    pos: f32,
                    vel: f32,
                    ang: f32,
                    rot: f32,
                }
                let mut table: Vec<((i64, usize), Outcome)> = Vec::new();
                for start in 1..=k {
                    for cancel_step in 0..=4usize {
                        let cancel = cancel_step as f32 * 0.25;
                        arena.set_car_state(slot, pre);
                        for tick in 1..=k {
                            let mut controls = base;
                            if tick == start {
                                controls.jump = true;
                                controls.pitch = pitch;
                                controls.yaw = yaw;
                            } else if tick > start {
                                // opposite pitch cancels the flip's pitch torque (sign of rel torque y)
                                let sign = arena.get_car_state(slot).flip_rel_torque.y.signum();
                                controls.pitch = cancel * sign;
                            }
                            arena.set_car_controls(slot, controls);
                            arena.step_tick();
                        }
                        let mut end = *arena.get_car_state(slot);
                        let speed = end.phys.ang_vel.length();
                        if speed > 5.5 {
                            end.phys.ang_vel *= 5.5 / speed;
                        }
                        table.push((
                            (start, cancel_step),
                            Outcome {
                                pos: (end.phys.pos - b.state.phys.pos).length(),
                                vel: (end.phys.vel - b.state.phys.vel).length(),
                                ang: (end.phys.ang_vel - b.state.phys.ang_vel).length(),
                                rot: rotation_error_degrees(end.phys.rot_mat, b.state.phys.rot_mat),
                            },
                        ));
                    }
                }
                let pv = |o: &Outcome| f64::from(o.pos) + 0.1 * f64::from(o.vel);
                // Fitted start: minimize position + velocity error over (D, c).
                let best_d = table
                    .iter()
                    .min_by(|x, y| pv(&x.1).total_cmp(&pv(&y.1)))
                    .map(|x| x.0.0)
                    .unwrap();
                let frame_tick_start = (timeline(index) - a.tick).clamp(1, k);
                // For each candidate start, choose the cancel by angular velocity (as the converter does).
                let choose = |d: i64| -> Outcome {
                    table
                        .iter()
                        .filter(|((s, _), _)| *s == d)
                        .min_by(|x, y| x.1.ang.total_cmp(&y.1.ang))
                        .map(|x| x.1)
                        .unwrap()
                };
                let fitted = choose(best_d);
                let framed = choose(frame_tick_start);
                // Determinacy: best pv over starts 2+ ticks away from the fitted one.
                let alternative = table
                    .iter()
                    .filter(|((s, _), _)| (s - best_d).abs() >= 2)
                    .map(|(_, o)| pv(o))
                    .fold(f64::INFINITY, f64::min);
                margins.push(
                    alternative
                        - pv(&fitted).min(
                            table
                                .iter()
                                .map(|(_, o)| pv(o))
                                .fold(f64::INFINITY, f64::min),
                        ),
                );
                offsets.push((best_d - frame_tick_start) as f64);
                rot.0.push(f64::from(fitted.rot));
                rot.1.push(f64::from(framed.rot));
                ang.0.push(f64::from(fitted.ang));
                ang.1.push(f64::from(framed.ang));
                pos.0.push(f64::from(fitted.pos));
                pos.1.push(f64::from(framed.pos));
                vel.0.push(f64::from(fitted.vel));
                vel.1.push(f64::from(framed.vel));
                events += 1;
                if events >= max_events {
                    break 'replays;
                }
            }
        }
    }

    let q = |v: &Vec<f64>, p: f64| quantile(&mut v.clone(), p);
    println!("dodge events with exact packets before and after: {events}");
    println!(
        "fitted start minus the activation frame's tick (ticks): p10 {:.0} p25 {:.0} p50 {:.0} p75 {:.0} p90 {:.0}",
        q(&offsets, 0.1),
        q(&offsets, 0.25),
        q(&offsets, 0.5),
        q(&offsets, 0.75),
        q(&offsets, 0.9)
    );
    println!(
        "fit margin (best start 2+ ticks away minus best), position+0.1 velocity units: p10 {:.2} p50 {:.2} p90 {:.2}",
        q(&margins, 0.1),
        q(&margins, 0.5),
        q(&margins, 0.9)
    );
    println!(
        "errors at packet B, p50 / p90 (start fitted to position+velocity  |  start at the activation frame tick):"
    );
    for (name, pair) in [
        ("position UU", &pos),
        ("velocity UU/s", &vel),
        ("rotation deg (not fitted)", &rot),
        ("angular rad/s (cancel chosen)", &ang),
    ] {
        println!(
            "  {name:<30} {:>7.2} / {:>7.2}  |  {:>7.2} / {:>7.2}",
            q(&pair.0, 0.5),
            q(&pair.0, 0.9),
            q(&pair.1, 0.5),
            q(&pair.1, 0.9)
        );
    }
    let better = rot
        .0
        .iter()
        .zip(&rot.1)
        .filter(|(f, g)| **f < **g - 0.5)
        .count();
    let worse = rot
        .0
        .iter()
        .zip(&rot.1)
        .filter(|(f, g)| **f > **g + 0.5)
        .count();
    println!(
        "rotation error at B (not used by the fit): fitted better/worse than the frame-tick start by more than 0.5 deg: {better} / {worse}"
    );
    Ok(())
}
