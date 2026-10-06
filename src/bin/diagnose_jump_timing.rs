//! Can a jump's start tick and hold length be recovered from the packets, out of sample?
//!
//! For each jump activation (jump counter even to odd) with a fresh car packet on the ground shortly
//! before and at least three packets shortly after, all at chain-inferred physical ticks, start
//! RocketSim from the earlier packet with the observed throttle, steer, handbrake and boost. Search
//! every (start tick D, hold length H) and fit the first two later packets. Score the fit on the
//! remaining held-out packets, against a control that pins D to the activation frame time (what a
//! frame-time model assumes) and fits only H. Reports how far the fitted start lies from the
//! activation frame time, how well-determined it is, and the in-sample and held-out errors.
//! Train replays only; refuses paths containing "test".

use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Quat, Vec3A};
use replicar_v1::conversion::{ConvertOptions, convert_bytes};
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
    pos: Vec3A,
    vel: Vec3A,
}

fn quantile(values: &mut [f64], q: f64) -> f64 {
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: diagnose_jump_timing <train dir or replay> [max events]")?,
    );
    if replicar_v1::sealed_path_refused(&path, false) {
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

    let (mut offsets, mut holds) = (Vec::new(), Vec::new());
    let (mut fit_in, mut fit_out, mut pinned_out) = (Vec::new(), Vec::new(), Vec::new());
    let (mut margins, mut improved, mut worse) = (Vec::new(), 0usize, 0usize);
    let mut events = 0usize;

    'replays: for replay_path in replay_paths(&path)? {
        let output = convert_bytes(&fs::read(&replay_path)?, &options)?;
        let frames = &output.observations.frames;
        let mut last_jump = std::collections::BTreeMap::<i32, u8>::new();
        for (index, frame) in frames.iter().enumerate() {
            for car in &frame.cars {
                let Some(jump) = car
                    .inputs
                    .jump_active_raw
                    .as_ref()
                    .filter(|d| d.frame == index)
                else {
                    continue;
                };
                let previous = last_jump.insert(car.actor_id, jump.value);
                if !(previous.is_some_and(|p| p % 2 == 0) && jump.value % 2 == 1) {
                    continue;
                }
                let packet_at =
                    |f: usize| -> Option<(Packet, CarState, &replicar_v1::observations::Car)> {
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
                        Some((
                            Packet {
                                tick: frames[f].time * 120.0 - lag.ticks as f32,
                                pos: state.phys.pos,
                                vel: state.phys.vel,
                            },
                            state,
                            c,
                        ))
                    };
                let Some((pre, mut pre_state, pre_car)) =
                    (index.saturating_sub(6)..index).rev().find_map(packet_at)
                else {
                    continue;
                };
                if pre_state.phys.pos.z > 25.0 {
                    continue;
                }
                pre_state.is_on_ground = true;
                pre_state.wheels_with_contact = [Some(rocketsim::RaycastHitInfo::default()); 4];
                let post: Vec<Packet> = (index..=(index + 12).min(frames.len() - 1))
                    .filter_map(|f| packet_at(f).map(|(p, _, _)| p))
                    .collect();
                if post.len() < 3 {
                    continue;
                }
                let horizon = (post[2].tick - pre.tick).ceil() as usize;
                if horizon > 45 || post.iter().any(|p| p.tick <= pre.tick) {
                    continue;
                }
                let base = CarControls {
                    throttle: pre_car.inputs.throttle.as_ref().map_or(0.0, |v| v.value),
                    steer: pre_car.inputs.steer.as_ref().map_or(0.0, |v| v.value),
                    handbrake: pre_car.inputs.handbrake.as_ref().is_some_and(|v| v.value),
                    boost: pre_car
                        .inputs
                        .boost_active_raw
                        .as_ref()
                        .is_some_and(|v| v.value % 2 == 1),
                    ..CarControls::default()
                };
                let activation_tick = f64::from(frames[index].time * 120.0 - pre.tick);
                let packet_ticks: Vec<usize> = post
                    .iter()
                    .map(|p| (p.tick - pre.tick).round() as usize)
                    .collect();
                // errors[D][H][packet] = |pos error| + 0.1 |vel error|
                let mut table: Vec<((usize, usize), Vec<f64>)> = Vec::new();
                for start in 1..=horizon.min(30) {
                    for hold in 1..=24usize {
                        arena.set_car_state(slot, pre_state);
                        let mut errors = vec![f64::NAN; post.len()];
                        for tick in 1..=horizon {
                            let mut controls = base;
                            controls.jump = tick >= start && tick < start + hold;
                            arena.set_car_controls(slot, controls);
                            arena.step_tick();
                            for (i, packet) in post.iter().enumerate() {
                                if packet_ticks[i] == tick {
                                    let s = arena.get_car_state(slot);
                                    errors[i] = f64::from((s.phys.pos - packet.pos).length())
                                        + 0.1 * f64::from((s.phys.vel - packet.vel).length());
                                }
                            }
                        }
                        table.push(((start, hold), errors));
                    }
                }
                let fit_score = |errors: &[f64]| (errors[0] + errors[1]) / 2.0;
                let heldout = |errors: &[f64]| {
                    let rest: Vec<f64> = errors[2..]
                        .iter()
                        .copied()
                        .filter(|e| !e.is_nan())
                        .collect();
                    (!rest.is_empty()).then(|| rest.iter().sum::<f64>() / rest.len() as f64)
                };
                if table.iter().any(|(_, e)| e[0].is_nan() || e[1].is_nan()) {
                    continue;
                }
                let best = table
                    .iter()
                    .min_by(|a, b| fit_score(&a.1).total_cmp(&fit_score(&b.1)))
                    .unwrap();
                let pinned_start = activation_tick.round().clamp(1.0, 30.0) as usize;
                let pinned = table
                    .iter()
                    .filter(|((d, _), _)| *d == pinned_start)
                    .min_by(|a, b| fit_score(&a.1).total_cmp(&fit_score(&b.1)));
                let (Some(pinned), Some(out_best)) = (pinned, heldout(&best.1)) else {
                    continue;
                };
                let Some(out_pinned) = heldout(&pinned.1) else {
                    continue;
                };
                // How well-determined is the start? Best score with a start 2+ ticks away.
                let alternative = table
                    .iter()
                    .filter(|((d, _), _)| (*d as i64 - best.0.0 as i64).abs() >= 2)
                    .map(|(_, e)| fit_score(e))
                    .fold(f64::INFINITY, f64::min);
                margins.push(alternative - fit_score(&best.1));
                offsets.push(best.0.0 as f64 - activation_tick);
                holds.push(best.0.1 as f64);
                fit_in.push(fit_score(&best.1));
                fit_out.push(out_best);
                pinned_out.push(out_pinned);
                if out_best < out_pinned - 1.0 {
                    improved += 1;
                } else if out_best > out_pinned + 1.0 {
                    worse += 1;
                }
                events += 1;
                if events >= max_events {
                    break 'replays;
                }
            }
        }
    }

    println!("jump events with exact packets before and at least three after: {events}");
    let q = |v: &Vec<f64>, p: f64| quantile(&mut v.clone(), p);
    println!(
        "fitted start minus activation frame time (ticks): p10 {:.1} p25 {:.1} p50 {:.1} p75 {:.1} p90 {:.1}",
        q(&offsets, 0.1),
        q(&offsets, 0.25),
        q(&offsets, 0.5),
        q(&offsets, 0.75),
        q(&offsets, 0.9)
    );
    println!(
        "fitted hold length (ticks): p10 {:.0} p25 {:.0} p50 {:.0} p75 {:.0} p90 {:.0}",
        q(&holds, 0.1),
        q(&holds, 0.25),
        q(&holds, 0.5),
        q(&holds, 0.75),
        q(&holds, 0.9)
    );
    println!(
        "start identifiability: score margin to the best start 2+ ticks away (UU-equivalent) p10 {:.2} p50 {:.2} p90 {:.2}; events with margin > 1: {:.0}%",
        q(&margins, 0.1),
        q(&margins, 0.5),
        q(&margins, 0.9),
        margins.iter().filter(|m| **m > 1.0).count() as f64 * 100.0 / margins.len() as f64
    );
    println!(
        "in-sample fit error (first two packets): p50 {:.2} p90 {:.2}",
        q(&fit_in, 0.5),
        q(&fit_in, 0.9)
    );
    println!(
        "held-out error (later packets), fitted start: p50 {:.2} p90 {:.2}; start pinned to activation frame time (hold fitted): p50 {:.2} p90 {:.2}",
        q(&fit_out, 0.5),
        q(&fit_out, 0.9),
        q(&pinned_out, 0.5),
        q(&pinned_out, 0.9)
    );
    println!("held-out better/worse than pinned by more than 1 unit: {improved} / {worse}");
    Ok(())
}
