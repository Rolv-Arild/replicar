//! Measure step-to-step unmasked angular velocity and rotation fidelity with and without lookahead.

use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Quat, Vec3A};
use replay_to_rocketsim::conversion::ConvertOptions;
use replay_to_rocketsim::observations;

fn paths(path: &Path) -> Result<Vec<PathBuf>, Box<dyn Error>> {
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

fn rotation_error_degrees(a: Mat3A, b: Mat3A) -> f32 {
    let trace = a.x_axis.dot(b.x_axis) + a.y_axis.dot(b.y_axis) + a.z_axis.dot(b.z_axis);
    ((trace - 1.0) * 0.5).clamp(-1.0, 1.0).acos().to_degrees()
}

fn evaluate(replay_paths: &[PathBuf], lookahead: bool) -> Result<(f32, f32, f32, f32), Box<dyn Error>> {
    let mut options = ConvertOptions::default();
    options.infer_air_controls_from_lookahead = lookahead;

    let mut rot_errors = Vec::new();
    let mut ang_errors = Vec::new();

    for p in replay_paths.iter().take(20) {
        let bytes = fs::read(p)?;
        let parsed = replay_to_rocketsim::parse_replay(&bytes)?;
        let observed = observations::extract(&parsed).unwrap();
        
        let out = replay_to_rocketsim::conversion::convert_observations(observed, &options)?;

        let config = rocketsim::ArenaConfig::new(rocketsim::GameMode::Soccar);
        let mut arena = rocketsim::Arena::new_with_config(config);
        let car_slot = arena.add_car(rocketsim::Team::Blue, rocketsim::CarBodyConfig::OCTANE);

        for i in 1..out.frames.len() {
            let f0 = &out.observations.frames[i - 1];
            let f1 = &out.observations.frames[i];
            let c0 = f0.cars.iter().find(|c| c.actor_id == 1 || c.player_key.is_some());
            let c1 = f1.cars.iter().find(|c| c.actor_id == 1 || c.player_key.is_some());
            let (Some(c0), Some(c1)) = (c0, c1) else { continue };

            let (Some(pos0), Some(rot0), Some(ang0)) = (
                &c0.body.position,
                &c0.body.rotation_xyzw,
                &c0.body.angular_velocity_replay_units,
            ) else { continue };

            let (Some(rot1), Some(ang1)) = (
                &c1.body.rotation_xyzw,
                &c1.body.angular_velocity_replay_units,
            ) else { continue };

            if pos0.value[2] <= 100.0 || rot0.frame != i - 1 || ang0.frame != i - 1 || rot1.frame != i || ang1.frame != i {
                continue;
            }

            let dt = f1.time - f0.time;
            let gap_ticks = ((dt * 120.0).round() as u64).max(1);
            if gap_ticks > 10 {
                continue;
            }

            let q0 = Quat::from_xyzw(rot0.value[0], rot0.value[1], rot0.value[2], rot0.value[3]);
            if !q0.is_finite() || q0.length_squared() < 0.5 {
                continue;
            }
            let mut car_state = rocketsim::CarState::default();
            car_state.is_on_ground = false;
            car_state.phys.pos = glam::Vec3A::from_array(pos0.value);
            car_state.phys.rot_mat = Mat3A::from_quat(q0.normalize());
            car_state.phys.ang_vel = glam::Vec3A::from_array(ang0.value) * 0.01;
            if let Some(vel0) = &c0.body.linear_velocity {
                car_state.phys.vel = glam::Vec3A::from_array(vel0.value);
            }
            arena.set_car_state(car_slot, car_state);

            if let Some((_, sim_car)) = out.frames[i - 1].state.cars.first() {
                arena.set_car_controls(car_slot, sim_car.controls);
            }

            for _ in 0..gap_ticks {
                arena.step_tick();
            }

            let pred_state = arena.get_car_state(car_slot);

            let q1 = Quat::from_xyzw(rot1.value[0], rot1.value[1], rot1.value[2], rot1.value[3]);
            if !q1.is_finite() || q1.length_squared() < 0.5 {
                continue;
            }
            let actual_rot = Mat3A::from_quat(q1.normalize());
            let actual_ang = Vec3A::from_array(ang1.value) * 0.01;

            rot_errors.push(rotation_error_degrees(pred_state.phys.rot_mat, actual_rot));
            ang_errors.push((pred_state.phys.ang_vel - actual_ang).length());
        }
    }

    if rot_errors.is_empty() {
        return Ok((0.0, 0.0, 0.0, 0.0));
    }

    rot_errors.sort_by(f32::total_cmp);
    ang_errors.sort_by(f32::total_cmp);

    let p50_rot = rot_errors[rot_errors.len() / 2];
    let p90_rot = rot_errors[(rot_errors.len() as f64 * 0.9) as usize];
    let p50_ang = ang_errors[ang_errors.len() / 2];
    let p90_ang = ang_errors[(ang_errors.len() as f64 * 0.9) as usize];

    Ok((p50_rot, p90_rot, p50_ang, p90_ang))
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: measure_air_fidelity <train path>")?,
    );
    let replay_paths = paths(&path)?;

    println!("Measuring WITHOUT lookahead...");
    let (no_rot_p50, no_rot_p90, no_ang_p50, no_ang_p90) = evaluate(&replay_paths, false)?;
    println!("Without lookahead: rot p50 = {:.3} deg, p90 = {:.3} deg | ang_vel p50 = {:.3} rad/s, p90 = {:.3} rad/s",
        no_rot_p50, no_rot_p90, no_ang_p50, no_ang_p90);

    println!("Measuring WITH lookahead...");
    let (with_rot_p50, with_rot_p90, with_ang_p50, with_ang_p90) = evaluate(&replay_paths, true)?;
    println!("With lookahead:    rot p50 = {:.3} deg, p90 = {:.3} deg | ang_vel p50 = {:.3} rad/s, p90 = {:.3} rad/s",
        with_rot_p50, with_rot_p90, with_ang_p50, with_ang_p90);

    Ok(())
}
