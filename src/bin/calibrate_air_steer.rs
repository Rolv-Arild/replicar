//! Calibrate aerial steer mapping to yaw/roll on train replays.

use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3, Quat, Vec3};
use replay_to_rocketsim::{observations, parse_replay};

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
            .ok_or("usage: calibrate_air_steer <train replay or split directory>")?,
    );
    let replay_paths = paths(&path)?;
    let mut air_frames = 0usize;
    let mut non_neutral_steer = 0usize;
    let mut handbrake_active = 0usize;
    let mut steer_yaw_same_sign = 0usize;
    let mut steer_yaw_opposite_sign = 0usize;
    let mut steer_roll_same_sign = 0usize;
    let mut steer_roll_opposite_sign = 0usize;

    for p in &replay_paths {
        let replay = parse_replay(&replay_to_rocketsim::read_replay_file(std::path::Path::new(&p), false)?)?;
        let observed = observations::extract(&replay).ok_or("network frames absent")?;
        for frame in &observed.frames {
            if !frame
                .game_state
                .as_ref()
                .is_some_and(|state| state.value == "Active")
            {
                continue;
            }
            for car in observations::primary_linked_cars(frame) {
                let body = &car.body;
                let (Some(pos), Some(rot), Some(ang)) = (
                    &body.position,
                    &body.rotation_xyzw,
                    &body.angular_velocity_replay_units,
                ) else {
                    continue;
                };
                if pos.value[2] <= 100.0 {
                    continue;
                }
                air_frames += 1;
                let steer = car.inputs.steer.as_ref().map_or(0.0, |v| v.value);
                let handbrake = car.inputs.handbrake.as_ref().is_some_and(|v| v.value);
                if steer.abs() > 0.1 {
                    non_neutral_steer += 1;
                    let q = Quat::from_xyzw(rot.value[0], rot.value[1], rot.value[2], rot.value[3]);
                    if !q.is_finite() || q.length_squared() < 0.5 {
                        continue;
                    }
                    let rot_mat = Mat3::from_quat(q.normalize());
                    let ang_vel_world = Vec3::from_array(ang.value) * 0.01;
                    // In car local frame:
                    // local_ang_vel = rot_mat.transpose() * world_ang_vel
                    let local_ang_vel = rot_mat.transpose() * ang_vel_world;
                    // local axes: x=forward, y=right, z=up
                    // yaw is rotation around z (up)
                    // roll is rotation around x (forward)
                    // pitch is rotation around y (right)
                    let local_roll = local_ang_vel.x;
                    let local_yaw = local_ang_vel.z;

                    if handbrake {
                        handbrake_active += 1;
                        if (steer * local_roll) > 0.05 {
                            steer_roll_same_sign += 1;
                        } else if (steer * local_roll) < -0.05 {
                            steer_roll_opposite_sign += 1;
                        }
                    } else {
                        if (steer * local_yaw) > 0.05 {
                            steer_yaw_same_sign += 1;
                        } else if (steer * local_yaw) < -0.05 {
                            steer_yaw_opposite_sign += 1;
                        }
                    }
                }
            }
        }
    }
    println!("Total air frames: {air_frames}");
    println!("Non-neutral steer in air: {non_neutral_steer} ({:.1}%)", non_neutral_steer as f64 * 100.0 / air_frames as f64);
    println!("Without handbrake (air roll):");
    println!("  steer & local yaw same sign: {steer_yaw_same_sign}");
    println!("  steer & local yaw opposite sign: {steer_yaw_opposite_sign}");
    println!("With handbrake (air roll): {handbrake_active}");
    println!("  steer & local roll same sign: {steer_roll_same_sign}");
    println!("  steer & local roll opposite sign: {steer_roll_opposite_sign}");

    Ok(())
}
