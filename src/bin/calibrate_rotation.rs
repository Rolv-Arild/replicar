//! Compare replay angular-velocity coordinates with quaternion motion on train replays.

use std::collections::HashMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Quat, Vec3};
use replay_to_rocketsim::{observations, parse_replay};

#[derive(Clone, Copy)]
struct Sample {
    time: f32,
    rotation: Quat,
    angular: Vec3,
    z: f32,
}

#[derive(Default)]
struct Errors {
    world: Vec<f32>,
    local_to_world: Vec<f32>,
    world_opposite: Vec<f32>,
    alignment: Vec<f32>,
}

fn quantile(values: &mut [f32], fraction: f64) -> Option<f32> {
    if values.is_empty() {
        return None;
    }
    values.sort_by(f32::total_cmp);
    let index = ((values.len() - 1) as f64 * fraction).round() as usize;
    Some(values[index])
}

fn quaternion(value: [f32; 4]) -> Option<Quat> {
    let q = Quat::from_xyzw(value[0], value[1], value[2], value[3]);
    (q.is_finite() && q.length_squared() > 0.5).then(|| q.normalize())
}

fn add_interval(previous: Sample, current: Sample, errors: &mut Errors) {
    let dt = current.time - previous.time;
    if !(0.015..=0.12).contains(&dt) {
        return;
    }
    let mut end = current.rotation;
    if previous.rotation.dot(end) < 0.0 {
        end = -end;
    }
    let (axis, angle) = (end * previous.rotation.conjugate()).to_axis_angle();
    let measured = axis * (angle / dt);
    let reported = (previous.angular + current.angular) * 0.005;
    if !measured.is_finite()
        || !reported.is_finite()
        || !(0.5..15.0).contains(&measured.length())
        || !(0.5..15.0).contains(&reported.length())
    {
        return;
    }
    let midpoint = previous.rotation.slerp(end, 0.5);
    errors.world.push((measured - reported).length());
    errors
        .local_to_world
        .push((measured - midpoint * reported).length());
    errors.world_opposite.push((measured + reported).length());
    errors
        .alignment
        .push(measured.normalize().dot(reported.normalize()));
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
            .ok_or("usage: calibrate_rotation <train replay or split directory>")?,
    );
    let mut ground = Errors::default();
    let mut air = Errors::default();
    let replay_paths = paths(&path)?;
    for path in &replay_paths {
        let replay = parse_replay(&replay_to_rocketsim::read_replay_file(std::path::Path::new(&path), false)?)?;
        let observed = observations::extract(&replay).ok_or("network frames absent")?;
        let mut previous: HashMap<(i32, usize), Sample> = HashMap::new();
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
                let (Some(rotation), Some(angular), Some(position)) = (
                    &body.rotation_xyzw,
                    &body.angular_velocity_replay_units,
                    &body.position,
                ) else {
                    continue;
                };
                if rotation.frame != frame.index
                    || angular.frame != frame.index
                    || position.frame != frame.index
                {
                    continue;
                }
                let Some(rotation) = quaternion(rotation.value) else {
                    continue;
                };
                let current = Sample {
                    time: frame.time,
                    rotation,
                    angular: Vec3::from_array(angular.value),
                    z: position.value[2],
                };
                if let Some(prior) =
                    previous.insert((car.actor_id, car.actor_created_frame), current)
                {
                    let errors = if prior.z > 100.0 && current.z > 100.0 {
                        &mut air
                    } else if prior.z < 50.0 && current.z < 50.0 {
                        &mut ground
                    } else {
                        continue;
                    };
                    add_interval(prior, current, errors);
                }
            }
        }
    }
    println!("{} replay(s)", replay_paths.len());
    for (name, errors) in [("ground", ground), ("air", air)] {
        let count = errors.world.len();
        let mut world = errors.world;
        let mut local = errors.local_to_world;
        let mut opposite = errors.world_opposite;
        let mut alignment = errors.alignment;
        println!(
            "{name}: n={count} world p50/p90={:?}/{:?}, local-to-world={:?}/{:?}, opposite-world={:?}/{:?}, alignment p50={:?}",
            quantile(&mut world, 0.5),
            quantile(&mut world, 0.9),
            quantile(&mut local, 0.5),
            quantile(&mut local, 0.9),
            quantile(&mut opposite, 0.5),
            quantile(&mut opposite, 0.9),
            quantile(&mut alignment, 0.5),
        );
    }
    Ok(())
}
