//! Calibrate and inspect inverse aerial control recovery on train replays.

use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Quat, Vec3A};
use replay_to_rocketsim::{observations, parse_replay};

const PI: f32 = std::f32::consts::PI;
const TORQUE_APPLY_SCALE: f32 = 2.0 * PI / 65536.0 * 1000.0;

// RocketSim air control torque and damping constants
const TORQUE_PITCH: f32 = 130.0 * TORQUE_APPLY_SCALE;
const TORQUE_YAW: f32 = 95.0 * TORQUE_APPLY_SCALE;
const TORQUE_ROLL: f32 = 400.0 * TORQUE_APPLY_SCALE;

const DAMPING_PITCH: f32 = 30.0 * TORQUE_APPLY_SCALE;
const DAMPING_YAW: f32 = 20.0 * TORQUE_APPLY_SCALE;
const DAMPING_ROLL: f32 = 50.0 * TORQUE_APPLY_SCALE;

#[derive(Debug, Clone, Copy)]
pub struct AirControls {
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
}

pub fn solve_inverse_air_controls(
    rot_mat_start: Mat3A,
    ang_vel_start: Vec3A,
    ang_vel_end: Vec3A,
    dt: f32,
) -> AirControls {
    if dt <= 0.0 || !dt.is_finite() {
        return AirControls { pitch: 0.0, yaw: 0.0, roll: 0.0 };
    }

    let forward = rot_mat_start.x_axis;
    let right = rot_mat_start.y_axis;
    let up = rot_mat_start.z_axis;

    let dir_pitch = -right;
    let dir_yaw = up;
    let dir_roll = -forward;

    let tau_world = (ang_vel_end - ang_vel_start) / dt;

    let tau_p = dir_pitch.dot(tau_world);
    let tau_y = dir_yaw.dot(tau_world);
    let tau_r = dir_roll.dot(tau_world);

    let omega_p = dir_pitch.dot(ang_vel_start);
    let omega_y = dir_yaw.dot(ang_vel_start);
    let omega_r = dir_roll.dot(ang_vel_start);

    // Solve pitch: tau_p = u_p * T_p - omega_p * D_p * (1 - |u_p|)
    let rhs_p = tau_p + omega_p * DAMPING_PITCH;
    let denom_p = TORQUE_PITCH + rhs_p.signum() * omega_p * DAMPING_PITCH;
    let pitch = if denom_p.abs() > 1e-4 {
        (rhs_p / denom_p).clamp(-1.0, 1.0)
    } else {
        0.0
    };

    // Solve yaw: tau_y = u_y * T_y - omega_y * D_y * (1 - |u_y|)
    let rhs_y = tau_y + omega_y * DAMPING_YAW;
    let denom_y = TORQUE_YAW + rhs_y.signum() * omega_y * DAMPING_YAW;
    let yaw = if denom_y.abs() > 1e-4 {
        (rhs_y / denom_y).clamp(-1.0, 1.0)
    } else {
        0.0
    };

    // Solve roll: tau_r = u_r * T_r - omega_r * D_r (no damping reduction in RocketSim)
    let rhs_r = tau_r + omega_r * DAMPING_ROLL;
    let roll = (rhs_r / TORQUE_ROLL).clamp(-1.0, 1.0);

    AirControls { pitch, yaw, roll }
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

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: calibrate_inverse_air <train replay or split directory>")?,
    );
    let replay_paths = paths(&path)?;
    let mut air_samples = 0usize;
    let mut active_pitch_count = 0usize;
    let mut active_yaw_count = 0usize;
    let mut active_roll_count = 0usize;
    let mut yaw_matches_steer_sign = 0usize;
    let mut yaw_steer_comparisons = 0usize;

    for p in &replay_paths {
        let replay = parse_replay(&fs::read(p)?)?;
        let observed = observations::extract(&replay).ok_or("network frames absent")?;
        let num_frames = observed.frames.len();

        for i in 0..num_frames.saturating_sub(1) {
            let f0 = &observed.frames[i];
            let f1 = &observed.frames[i + 1];

            if !f0.game_state.as_ref().is_some_and(|s| s.value == "Active")
                || !f1.game_state.as_ref().is_some_and(|s| s.value == "Active")
            {
                continue;
            }

            let dt = f1.time - f0.time;
            if dt <= 0.0 || dt > 0.05 {
                continue;
            }

            for c0 in observations::primary_linked_cars(f0) {
                let Some(c1) = f1.cars.iter().find(|c| c.actor_id == c0.actor_id) else {
                    continue;
                };

                let (Some(pos0), Some(rot0), Some(ang0)) = (
                    &c0.body.position,
                    &c0.body.rotation_xyzw,
                    &c0.body.angular_velocity_replay_units,
                ) else {
                    continue;
                };

                let Some(ang1) = &c1.body.angular_velocity_replay_units else {
                    continue;
                };

                if pos0.value[2] <= 100.0 {
                    continue; // Ground or near-ground
                }

                if pos0.frame != i || rot0.frame != i || ang0.frame != i || ang1.frame != (i + 1) {
                    continue; // Require fresh packets
                }

                let q0 = Quat::from_xyzw(rot0.value[0], rot0.value[1], rot0.value[2], rot0.value[3]);
                if !q0.is_finite() || q0.length_squared() < 0.5 {
                    continue;
                }
                let rot_mat0 = Mat3A::from_quat(q0.normalize());
                let omega0 = Vec3A::from_array(ang0.value) * 0.01;
                let omega1 = Vec3A::from_array(ang1.value) * 0.01;

                let controls = solve_inverse_air_controls(rot_mat0, omega0, omega1, dt);
                air_samples += 1;

                if controls.pitch.abs() > 0.1 {
                    active_pitch_count += 1;
                }
                if controls.yaw.abs() > 0.1 {
                    active_yaw_count += 1;
                }
                if controls.roll.abs() > 0.1 {
                    active_roll_count += 1;
                }

                let steer = c0.inputs.steer.as_ref().map_or(0.0, |v| v.value);
                if steer.abs() > 0.1 && controls.yaw.abs() > 0.1 {
                    yaw_steer_comparisons += 1;
                    if (steer * controls.yaw) > 0.0 {
                        yaw_matches_steer_sign += 1;
                    }
                }
            }
        }
    }

    println!("Total airborne consecutive frame pairs: {air_samples}");
    println!("Active recovered pitch (|pitch| > 0.1): {active_pitch_count} ({:.1}%)", active_pitch_count as f64 * 100.0 / air_samples as f64);
    println!("Active recovered yaw   (|yaw| > 0.1):   {active_yaw_count} ({:.1}%)", active_yaw_count as f64 * 100.0 / air_samples as f64);
    println!("Active recovered roll  (|roll| > 0.1):  {active_roll_count} ({:.1}%)", active_roll_count as f64 * 100.0 / air_samples as f64);
    println!("Yaw vs ReplicatedSteer sign match: {yaw_matches_steer_sign} / {yaw_steer_comparisons} ({:.1}%)",
        yaw_matches_steer_sign as f64 * 100.0 / yaw_steer_comparisons.max(1) as f64);

    Ok(())
}
