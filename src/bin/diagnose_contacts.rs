//! Is ball-touch error limited by car replication or by the contact model? Train replays only.
//!
//! Uses the converter's ball residuals (chain-lag packets, exact ticks) with a car within 300 UU.
//! For each, the physical-tick distance from the ball packet to the nearest fresh packet of that
//! car (before or after, both chain-lag) measures how far the car state had to be extrapolated.
//! If touch error grows with that distance, car replication limits it; if it stays high right next
//! to a car packet, the contact model does. Also stratifies by whether a touch occurred (a ball
//! velocity change far beyond free flight) and by closing speed.

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Quat, Vec3A};
use replicar::conversion::{ConvertOptions, convert_bytes};
use replicar::observations::Body;
use rocketsim::{Arena, ArenaConfig, ArenaEvent, BallState, GameMode};

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

fn ball_state(body: &Body, frame: usize) -> Option<BallState> {
    let pos = body.position.as_ref().filter(|v| v.frame == frame)?.value;
    let vel = body
        .linear_velocity
        .as_ref()
        .filter(|v| v.frame == frame)?
        .value;
    let rot = body
        .rotation_xyzw
        .as_ref()
        .filter(|v| v.frame == frame)?
        .value;
    let ang = body
        .angular_velocity_replay_units
        .as_ref()
        .filter(|v| v.frame == frame)?
        .value;
    let quat = Quat::from_xyzw(rot[0], rot[1], rot[2], rot[3]);
    if !quat.is_finite() || quat.length_squared() < 0.5 {
        return None;
    }
    let mut state = BallState::default();
    state.phys.pos = Vec3A::from_array(pos);
    state.phys.vel = Vec3A::from_array(vel);
    state.phys.ang_vel = Vec3A::from_array(ang) * 0.01;
    state.phys.rot_mat = Mat3A::from_quat(quat.normalize());
    Some(state)
}

fn quantile(values: &mut [f32], q: f64) -> f32 {
    if values.is_empty() {
        return f32::NAN;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn dist(a: [f32; 3], b: [f32; 3]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

#[derive(Default)]
struct Group {
    velocity: Vec<f32>,
    position: Vec<f32>,
    rotation: Vec<f32>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: diagnose_contacts <train dir or replay>")?,
    );
    if replicar::sealed_path_refused(&path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let options = ConvertOptions::default();
    let mut by_age: BTreeMap<String, Group> = BTreeMap::new();
    let mut by_speed: BTreeMap<String, Group> = BTreeMap::new();
    let mut touched_share: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut scratch = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    // Touch outcome class -> errors, for ball packets with a car packet within 2 ticks. The
    // `rotation` vector holds the size of the real impulse for scale.
    let mut classes: BTreeMap<&'static str, Group> = BTreeMap::new();

    for replay_path in replay_paths(&path)? {
        let output = convert_bytes(&fs::read(&replay_path)?, &options)?;
        let frames = &output.observations.frames;
        let first_time = frames[0].time;
        let tick =
            |f: usize| ((f64::from(frames[f].time) - f64::from(first_time)) * 120.0).round() as i64;
        // Chain-lag physical tick of each car's fresh packets, per actor lifetime.
        let mut car_ticks: BTreeMap<(i32, usize), Vec<(usize, i64)>> = BTreeMap::new();
        for (f, converted) in output.frames.iter().enumerate() {
            for lag in &converted.packet_lags {
                if let (Some(actor), "chain") = (lag.actor_id, lag.source)
                    && let Some(car) = frames[f].cars.iter().find(|c| c.actor_id == actor)
                {
                    car_ticks
                        .entry((actor, car.actor_created_frame))
                        .or_default()
                        .push((f, tick(f) - lag.ticks as i64));
                }
            }
        }
        for residual in output
            .position_residuals
            .iter()
            .filter(|r| r.actor_id.is_none())
        {
            let f = residual.frame;
            let converted = &output.frames[f];
            let Some(ball_lag) = converted
                .packet_lags
                .iter()
                .find(|l| l.actor_id.is_none() && l.source == "chain")
            else {
                continue;
            };
            let Some(ball) = frames[f].ball.as_ref().and_then(|b| b.position.as_ref()) else {
                continue;
            };
            let ball_tick = tick(f) - ball_lag.ticks as i64;
            // Nearest car within 300 UU with chain-lag packets around this time.
            let mut best: Option<(f32, &(i32, usize))> = None;
            for car in &frames[f].cars {
                let Some(p) = car.body.position.as_ref() else {
                    continue;
                };
                let d = dist(p.value, ball.value);
                if d < 300.0 && best.is_none_or(|(bd, _)| d < bd) {
                    best = Some((d, &(0, 0)));
                    let _ = car;
                }
            }
            let Some((_, _)) = best else { continue };
            // Recompute nearest car key properly.
            let near = frames[f]
                .cars
                .iter()
                .filter_map(|c| {
                    let p = c.body.position.as_ref()?;
                    let d = dist(p.value, ball.value);
                    (d < 300.0).then_some((d, c))
                })
                .min_by(|a, b| a.0.total_cmp(&b.0));
            let Some((_, car)) = near else { continue };
            let Some(ticks) = car_ticks.get(&(car.actor_id, car.actor_created_frame)) else {
                continue;
            };
            let age = ticks.iter().map(|&(_, t)| (ball_tick - t).abs()).min();
            let Some(age) = age else { continue };
            let label = match age {
                0..=2 => "0-2",
                3..=5 => "3-5",
                6..=8 => "6-8",
                9..=11 => "9-11",
                _ => "12+",
            }
            .to_string();
            let velocity = residual
                .simulated_velocity_error_uu_per_sec
                .unwrap_or(f32::NAN);
            let touched = velocity > 20.0;
            let entry = touched_share.entry(label.clone()).or_default();
            entry.0 += 1;
            entry.1 += usize::from(touched);
            let group = by_age.entry(label).or_default();
            group.velocity.push(velocity);
            group.position.push(residual.simulated_error_uu);
            if let Some(r) = residual.simulated_rotation_error_degrees {
                group.rotation.push(r);
            }
            // Closing speed: relative velocity magnitude of car and ball from packets.
            let relative = match (
                car.body.linear_velocity.as_ref(),
                frames[f]
                    .ball
                    .as_ref()
                    .and_then(|b| b.linear_velocity.as_ref()),
            ) {
                (Some(cv), Some(bv)) => dist(cv.value, bv.value),
                _ => f32::NAN,
            };
            let speed_label = if relative.is_nan() {
                "unknown"
            } else if relative < 500.0 {
                "<500"
            } else if relative < 1000.0 {
                "500-1000"
            } else if relative < 2000.0 {
                "1000-2000"
            } else {
                ">=2000"
            };
            let group = by_speed.entry(speed_label.to_string()).or_default();
            group.velocity.push(velocity);
            group.position.push(residual.simulated_error_uu);
            // Touch classification for packets with a fresh car packet within 2 ticks.
            if age <= 2 && f > 0 {
                let previous_lag = output.frames[f - 1]
                    .packet_lags
                    .iter()
                    .find(|l| l.actor_id.is_none() && l.source == "chain");
                let (Some(previous_lag), Some(prev_body), Some(cur_body)) = (
                    previous_lag,
                    frames[f - 1].ball.as_ref(),
                    frames[f].ball.as_ref(),
                ) else {
                    continue;
                };
                let (Some(state_a), Some(state_b)) =
                    (ball_state(prev_body, f - 1), ball_state(cur_body, f))
                else {
                    continue;
                };
                let previous_tick = tick(f - 1) - previous_lag.ticks as i64;
                let k = ball_tick - previous_tick;
                if !(1..=14).contains(&k) {
                    continue;
                }
                scratch.set_ball_state(state_a);
                for _ in 0..k {
                    scratch.step_tick();
                }
                let free = scratch.get_ball_state().phys.vel;
                let real_touch = (state_b.phys.vel - free).length() > 50.0;
                let sim_hit = converted
                    .simulated_events
                    .iter()
                    .any(|e| matches!(e.event, ArenaEvent::CarHitBall(_)));
                let class = match (real_touch, sim_hit) {
                    (true, true) => "real touch, sim hit",
                    (true, false) => "real touch, sim NO hit (missed)",
                    (false, true) => "no real touch, sim hit (phantom)",
                    (false, false) => "no touch either",
                };
                let group = classes.entry(class).or_default();
                group.velocity.push(velocity);
                group.position.push(residual.simulated_error_uu);
                group.rotation.push((state_b.phys.vel - free).length());
            }
        }
    }

    println!(
        "ball residuals with a car within 300 UU, by physical ticks to the car's nearest chain-lag packet:"
    );
    println!(
        "{:>8} {:>8} {:>10} | {:>22} | {:>22}",
        "ticks", "n", "vel>20", "vel err UU/s p50/90/99", "pos err UU p50/90/99"
    );
    for (label, group) in by_age.iter_mut() {
        let (n, touched) = touched_share[label];
        println!(
            "{:>8} {:>8} {:>9.1}% | {:>6.1}/{:>6.1}/{:>7.1} | {:>6.2}/{:>6.2}/{:>6.1}",
            label,
            n,
            touched as f64 * 100.0 / n as f64,
            quantile(&mut group.velocity, 0.5),
            quantile(&mut group.velocity, 0.9),
            quantile(&mut group.velocity, 0.99),
            quantile(&mut group.position, 0.5),
            quantile(&mut group.position, 0.9),
            quantile(&mut group.position, 0.99),
        );
    }
    println!(
        "\nball intervals with a car packet within 2 ticks, by touch outcome (real touch = ball velocity differs from a ball-only prediction by > 50 UU/s):"
    );
    for (label, group) in classes.iter_mut() {
        println!(
            "  {:<36} n={:>6}  vel err p50/90/99 {:>6.1}/{:>7.1}/{:>7.1}  pos p90 {:>6.2}  real impulse p50/p90 {:>6.1}/{:>6.1}",
            label,
            group.velocity.len(),
            quantile(&mut group.velocity, 0.5),
            quantile(&mut group.velocity, 0.9),
            quantile(&mut group.velocity, 0.99),
            quantile(&mut group.position, 0.9),
            quantile(&mut group.rotation, 0.5),
            quantile(&mut group.rotation, 0.9),
        );
    }
    println!("\nby car-ball relative speed at the packets:");
    for (label, group) in by_speed.iter_mut() {
        println!(
            "{:>10} n={:>7} vel err p50/90/99 {:>6.1}/{:>6.1}/{:>7.1}  pos p90 {:>6.2}",
            label,
            group.velocity.len(),
            quantile(&mut group.velocity, 0.5),
            quantile(&mut group.velocity, 0.9),
            quantile(&mut group.velocity, 0.99),
            quantile(&mut group.position, 0.9),
        );
    }
    Ok(())
}
