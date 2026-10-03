//! Measure step-to-step unmasked angular velocity and rotation fidelity with and without lookahead.

use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Quat, Vec3A};
use replay_to_rocketsim::conversion::ConvertOptions;
use replay_to_rocketsim::observations;

#[derive(Clone, Copy)]
enum AirMode {
    ReplayOnly,
    CurrentInverse,
    RlcisInverse,
}

// Port of ReverseAirOrientInputs in external/RLCarInputSolver/AirSolver.cpp.
// This compares the aerial orientation formula, not the full C++ solver.
fn rlcis_air_controls(rot: Mat3A, before: Vec3A, mut after: Vec3A, dt: f32) -> [f32; 3] {
    let max_ang_sq = 5.5_f32.powi(2);
    if ((after.length_squared() - max_ang_sq) / max_ang_sq).abs() < 0.01 {
        after *= 1.25;
    }
    let local_tau = rot.transpose() * ((after - before) / dt);
    let local_omega = rot.transpose() * before;
    let rhs = local_tau + local_omega * Vec3A::new(4.47166, 2.7982, 1.8865);
    let roll = rhs.x / -36.0796;
    let pitch = rhs.y / (-12.1460 - rhs.y.signum() * local_omega.y * 2.7982);
    let yaw = rhs.z / (8.9196 + rhs.z.signum() * local_omega.z * 1.8865);
    let deadzone = |v: f32| {
        let v = v.clamp(-1.0, 1.0);
        if v.abs() < 0.1 {
            0.0
        } else if v.abs() > 0.95 {
            v.signum()
        } else {
            v
        }
    };
    [deadzone(pitch), deadzone(yaw), deadzone(roll)]
}

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

fn evaluate(
    replay_paths: &[PathBuf],
    mode: AirMode,
    preserve_car_state: bool,
) -> Result<(usize, f32, f32, f32, f32), Box<dyn Error>> {
    let mut options = ConvertOptions::default();
    options.infer_air_controls_from_lookahead = !matches!(mode, AirMode::ReplayOnly);

    let mut rot_errors = Vec::new();
    let mut ang_errors = Vec::new();

    for p in replay_paths {
        let bytes = replay_to_rocketsim::read_replay_file(std::path::Path::new(&p), false)?;
        let parsed = replay_to_rocketsim::parse_replay(&bytes)?;
        let observed = observations::extract(&parsed).unwrap();

        let out = replay_to_rocketsim::conversion::convert_observations(observed, &options)?;

        let config = rocketsim::ArenaConfig::new(rocketsim::GameMode::Soccar);
        let mut arena = rocketsim::Arena::new_with_config(config);
        let car_slot = arena.add_car(rocketsim::Team::Blue, rocketsim::CarBodyConfig::OCTANE);

        for i in 1..out.frames.len() {
            let f0 = &out.observations.frames[i - 1];
            let f1 = &out.observations.frames[i];
            if !f0.game_state.as_ref().is_some_and(|s| s.value == "Active")
                || !f1.game_state.as_ref().is_some_and(|s| s.value == "Active")
            {
                continue;
            }
            for c0 in observations::primary_linked_cars(f0) {
                let Some(c1) = observations::primary_linked_cars(f1).into_iter().find(|c| {
                    c.actor_id == c0.actor_id
                        && c.actor_created_frame == c0.actor_created_frame
                        && c.player_key == c0.player_key
                        && c.player_link_active == c0.player_link_active
                }) else {
                    continue;
                };
                let Some(slot) = out
                    .car_slots
                    .iter()
                    .find(|s| Some(&s.player_key) == c0.player_key.as_ref())
                    .map(|s| s.slot)
                else {
                    continue;
                };
                let Some((_, sim_car)) = out.frames[i - 1]
                    .state
                    .cars
                    .iter()
                    .find(|(s, _)| s.idx == slot)
                else {
                    continue;
                };

                let (Some(pos0), Some(rot0), Some(ang0)) = (
                    &c0.body.position,
                    &c0.body.rotation_xyzw,
                    &c0.body.angular_velocity_replay_units,
                ) else {
                    continue;
                };

                let (Some(pos1), Some(rot1), Some(ang1)) = (
                    &c1.body.position,
                    &c1.body.rotation_xyzw,
                    &c1.body.angular_velocity_replay_units,
                ) else {
                    continue;
                };

                if pos0.value[2] <= 100.0
                    || pos1.value[2] <= 100.0
                    || pos0.frame != i - 1
                    || pos1.frame != i
                    || rot0.frame != i - 1
                    || ang0.frame != i - 1
                    || rot1.frame != i
                    || ang1.frame != i
                {
                    continue;
                }
                if c0
                    .inputs
                    .dodge_active_raw
                    .as_ref()
                    .is_some_and(|d| d.frame == i - 1 && d.value % 2 == 1)
                    || c1
                        .inputs
                        .dodge_active_raw
                        .as_ref()
                        .is_some_and(|d| d.frame == i && d.value % 2 == 1)
                {
                    continue;
                }

                let dt = f1.time - f0.time;
                if !dt.is_finite() || dt <= 0.0 || dt > 0.05 {
                    continue;
                }
                let gap_ticks = ((dt * 120.0).round() as u64).max(1);

                let q0 =
                    Quat::from_xyzw(rot0.value[0], rot0.value[1], rot0.value[2], rot0.value[3]);
                if !q0.is_finite() || q0.length_squared() < 0.5 {
                    continue;
                }
                let mut car_state = if preserve_car_state {
                    *sim_car
                } else {
                    rocketsim::CarState::default()
                };
                car_state.is_on_ground = false;
                car_state.phys.pos = glam::Vec3A::from_array(pos0.value);
                car_state.phys.rot_mat = Mat3A::from_quat(q0.normalize());
                car_state.phys.ang_vel = glam::Vec3A::from_array(ang0.value) * 0.01;
                if let Some(vel0) = &c0.body.linear_velocity {
                    car_state.phys.vel = glam::Vec3A::from_array(vel0.value);
                }
                arena.set_car_state(car_slot, car_state);

                let mut controls = sim_car.controls;
                if matches!(mode, AirMode::RlcisInverse) {
                    let [pitch, yaw, roll] = rlcis_air_controls(
                        car_state.phys.rot_mat,
                        car_state.phys.ang_vel,
                        Vec3A::from_array(ang1.value) * 0.01,
                        dt,
                    );
                    controls.pitch = pitch;
                    controls.yaw = yaw;
                    controls.roll = roll;
                }
                arena.set_car_controls(car_slot, controls);

                for _ in 0..gap_ticks {
                    arena.step_tick();
                }

                let pred_state = arena.get_car_state(car_slot);

                let q1 =
                    Quat::from_xyzw(rot1.value[0], rot1.value[1], rot1.value[2], rot1.value[3]);
                if !q1.is_finite() || q1.length_squared() < 0.5 {
                    continue;
                }
                let actual_rot = Mat3A::from_quat(q1.normalize());
                let actual_ang = Vec3A::from_array(ang1.value) * 0.01;

                rot_errors.push(rotation_error_degrees(pred_state.phys.rot_mat, actual_rot));
                ang_errors.push((pred_state.phys.ang_vel - actual_ang).length());
            }
        }
    }

    if rot_errors.is_empty() {
        return Ok((0, 0.0, 0.0, 0.0, 0.0));
    }

    rot_errors.sort_by(f32::total_cmp);
    ang_errors.sort_by(f32::total_cmp);

    let p50_rot = rot_errors[rot_errors.len() / 2];
    let p90_rot = rot_errors[(rot_errors.len() as f64 * 0.9) as usize];
    let p50_ang = ang_errors[ang_errors.len() / 2];
    let p90_ang = ang_errors[(ang_errors.len() as f64 * 0.9) as usize];

    Ok((rot_errors.len(), p50_rot, p90_rot, p50_ang, p90_ang))
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args_os().skip(1);
    let path = PathBuf::from(
        args.next()
            .ok_or("usage: measure_air_fidelity <split path> [--preserve-car-state]")?,
    );
    let preserve_car_state = match args.next().as_deref() {
        None => false,
        Some(value) if value == "--preserve-car-state" => true,
        _ => return Err("usage: measure_air_fidelity <split path> [--preserve-car-state]".into()),
    };
    let replay_paths = paths(&path)?;

    println!("Measuring WITHOUT lookahead...");
    let (no_samples, no_rot_p50, no_rot_p90, no_ang_p50, no_ang_p90) =
        evaluate(&replay_paths, AirMode::ReplayOnly, preserve_car_state)?;
    println!(
        "Without lookahead ({no_samples} pairs, {} replays): rot p50 = {:.3} deg, p90 = {:.3} deg | ang_vel p50 = {:.3} rad/s, p90 = {:.3} rad/s",
        replay_paths.len(),
        no_rot_p50,
        no_rot_p90,
        no_ang_p50,
        no_ang_p90
    );

    println!("Measuring WITH lookahead...");
    let (with_samples, with_rot_p50, with_rot_p90, with_ang_p50, with_ang_p90) =
        evaluate(&replay_paths, AirMode::CurrentInverse, preserve_car_state)?;
    println!(
        "With lookahead ({with_samples} pairs, {} replays): rot p50 = {:.3} deg, p90 = {:.3} deg | ang_vel p50 = {:.3} rad/s, p90 = {:.3} rad/s",
        replay_paths.len(),
        with_rot_p50,
        with_rot_p90,
        with_ang_p50,
        with_ang_p90
    );

    println!("Measuring RLCarInputSolver aerial inverse...");
    let (rlcis_samples, rlcis_rot_p50, rlcis_rot_p90, rlcis_ang_p50, rlcis_ang_p90) =
        evaluate(&replay_paths, AirMode::RlcisInverse, preserve_car_state)?;
    println!(
        "RLCarInputSolver ({rlcis_samples} pairs, {} replays): rot p50 = {:.3} deg, p90 = {:.3} deg | ang_vel p50 = {:.3} rad/s, p90 = {:.3} rad/s",
        replay_paths.len(),
        rlcis_rot_p50,
        rlcis_rot_p90,
        rlcis_ang_p50,
        rlcis_ang_p90
    );

    Ok(())
}
