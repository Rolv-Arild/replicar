//! Local-frame angular velocity of RocketSim flips, for comparison with replay dodge statistics.
//!
//! Starts an upright, motionless car in the air, dodges with a given torque direction, and prints
//! the angular velocity in the car's frame (x forward/roll, y right/pitch, z up/yaw) at ticks
//! measured from the first tick with |w| >= 2 rad/s, both as reported after each step and with
//! the 5.5 rad/s limit applied. Train-independent physics probe; no replay data.

use std::path::Path;

use glam::Vec3A;
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, CarState, GameMode, Team};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    let slot = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    // (label, relative torque x = roll axis, y = pitch axis) as in the replay's DodgeTorque / scale
    for (label, tx, ty) in [
        ("roll-dominant (1.0, 0.0)", 1.0f32, 0.0f32),
        ("diagonal (0.7, 0.7)", 0.7, 0.7),
        ("pitch-dominant (0.15, 0.99)", 0.15, 0.99),
    ] {
        let mut state = CarState::default();
        state.phys.pos = Vec3A::new(0.0, 0.0, 900.0);
        state.is_on_ground = false;
        state.has_jumped = true;
        state.air_time_since_jump = 0.05;
        arena.set_car_state(slot, state);
        // flip_rel_torque = (tx, ty): pitch = -ty, yaw = -tx per the converter's mapping
        let mut started: Option<usize> = None;
        println!(
            "\n{label}: ticks since first |w|>=2 | reported local w (x,y,z) |w| | limited |w|"
        );
        for tick in 1..=80usize {
            let controls = if tick == 1 {
                CarControls {
                    jump: true,
                    pitch: -ty,
                    yaw: -tx,
                    ..CarControls::default()
                }
            } else {
                CarControls::default()
            };
            arena.set_car_controls(slot, controls);
            arena.step_tick();
            let s = *arena.get_car_state(slot);
            let r = s.phys.rot_mat;
            let w = s.phys.ang_vel;
            let local = Vec3A::new(r.x_axis.dot(w), r.y_axis.dot(w), r.z_axis.dot(w));
            if started.is_none() && w.length() >= 2.0 {
                started = Some(tick);
            }
            if let Some(start) = started {
                let since = tick - start;
                if since % 6 == 0 && since <= 48 {
                    let limited = w.length().min(5.5);
                    let scale = limited / w.length().max(1e-6);
                    println!(
                        "  {since:3}  ({:6.2},{:6.2},{:6.2}) |w|={:5.2} | limited |w|={:5.2}  local limited ({:5.2},{:5.2},{:5.2}) flipping={}",
                        local.x,
                        local.y,
                        local.z,
                        w.length(),
                        limited,
                        local.x * scale,
                        local.y * scale,
                        local.z * scale,
                        s.is_flipping
                    );
                }
            }
        }
    }
    Ok(())
}
