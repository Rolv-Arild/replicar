//! Compare observed car motion with the replay's velocity fields.

use std::collections::HashMap;
use std::env;
use std::error::Error;
use std::fs;

use replay_to_rocketsim::{observations, parse_replay};

#[derive(Clone, Copy)]
struct Sample {
    time: f32,
    position: [f32; 3],
    rotation: [f32; 4],
    linear: [f32; 3],
    angular: [f32; 3],
}

fn yaw(q: [f32; 4]) -> f32 {
    let [x, y, z, w] = q;
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

fn quantiles(values: &mut [f32]) -> (f32, f32, f32) {
    values.sort_by(f32::total_cmp);
    (
        values[values.len() / 10],
        values[values.len() / 2],
        values[values.len() * 9 / 10],
    )
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = env::args_os()
        .nth(1)
        .ok_or("usage: calibrate_units <replay>")?;
    let replay = parse_replay(&fs::read(path)?)?;
    let observed = observations::extract(&replay).ok_or("network frames absent")?;
    let mut previous: HashMap<i32, Sample> = HashMap::new();
    let mut linear_ratios = Vec::new();
    let mut angular_ratios = Vec::new();
    for frame in &observed.frames {
        for car in &frame.cars {
            let body = &car.body;
            let (Some(pos), Some(rot), Some(vel), Some(ang)) = (
                &body.position,
                &body.rotation_xyzw,
                &body.linear_velocity,
                &body.angular_velocity_replay_units,
            ) else {
                continue;
            };
            if [pos.frame, rot.frame, vel.frame, ang.frame]
                .iter()
                .any(|index| *index != frame.index)
            {
                continue;
            }
            let current = Sample {
                time: frame.time,
                position: pos.value,
                rotation: rot.value,
                linear: vel.value,
                angular: ang.value,
            };
            if let Some(prev) = previous.insert(car.actor_id, current) {
                let dt = current.time - prev.time;
                if !(0.02..0.11).contains(&dt)
                    || current.position[2] > 50.0
                    || prev.position[2] > 50.0
                {
                    continue;
                }
                let v = [
                    (current.linear[0] + prev.linear[0]) * 0.5,
                    (current.linear[1] + prev.linear[1]) * 0.5,
                ];
                let speed_sq = v[0] * v[0] + v[1] * v[1];
                if speed_sq > 400.0 * 400.0 {
                    let displacement = [
                        current.position[0] - prev.position[0],
                        current.position[1] - prev.position[1],
                    ];
                    let ratio = (displacement[0] * v[0] + displacement[1] * v[1]) / (dt * speed_sq);
                    if ratio.is_finite() && (0.0..2.0).contains(&ratio) {
                        linear_ratios.push(ratio);
                    }
                }
                let angular = (current.angular[2] + prev.angular[2]) * 0.5;
                if angular.abs() > 20.0 && angular.abs() < 1000.0 {
                    let dyaw = (yaw(current.rotation) - yaw(prev.rotation))
                        .rem_euclid(std::f32::consts::TAU);
                    let dyaw = if dyaw > std::f32::consts::PI {
                        dyaw - std::f32::consts::TAU
                    } else {
                        dyaw
                    };
                    let ratio = dyaw / (dt * angular);
                    if ratio.is_finite() && (0.0..0.1).contains(&ratio) {
                        angular_ratios.push(ratio);
                    }
                }
            }
        }
    }
    println!(
        "linear samples={} p10/median/p90={:?}",
        linear_ratios.len(),
        quantiles(&mut linear_ratios)
    );
    println!(
        "angular samples={} p10/median/p90={:?}",
        angular_ratios.len(),
        quantiles(&mut angular_ratios)
    );
    println!("candidate angular velocity scale=0.01 radians/second per replay unit");
    Ok(())
}
