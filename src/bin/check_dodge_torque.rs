//! Does the replay's dodge torque equal RocketSim's `flip_rel_torque` times (2.60, 2.24)?
//!
//! RocketSim builds a unit dodge direction `normalize(-pitch, yaw + roll)` (x forward, y right) and
//! sets `flip_rel_torque = (-dir.y, dir.x, 0)`, applied times `flip::TORQUE = (260, 224)`. If the
//! replicated dodge torque is that vector in units of 1/100, every dodge activation lies on the
//! ellipse (tx / 2.60)^2 + (ty / 2.24)^2 = 1 (or at the origin for a double jump with no direction).
//! Prints the distribution of that radius, the signs, and the implied dodge direction over the
//! fresh (dodge counter turning odd with a fresh torque) activations. Train/validation only.

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::{observations, parse_replay};

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

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: check_dodge_torque <train or validation dir or replay>")?,
    );
    if path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let mut radii: Vec<f32> = Vec::new();
    let mut last: BTreeMap<(usize, i32, usize), u8> = BTreeMap::new();
    let mut quadrants = [0usize; 4];
    let mut axis_counts = [0usize; 4]; // forward, backward, right, left (pure axis within 0.05)
    for (replay_index, replay_path) in replay_paths(&path)?.into_iter().enumerate() {
        let replay = parse_replay(&fs::read(&replay_path)?)?;
        let observed = observations::extract(&replay).ok_or("network frames absent")?;
        for (index, frame) in observed.frames.iter().enumerate() {
            for car in &frame.cars {
                let (Some(dodge), Some(torque)) = (
                    car.inputs
                        .dodge_active_raw
                        .as_ref()
                        .filter(|d| d.frame == index),
                    car.inputs
                        .dodge_torque_replay_units
                        .as_ref()
                        .filter(|t| t.frame == index),
                ) else {
                    continue;
                };
                let key = (replay_index, car.actor_id, car.actor_created_frame);
                let previous = last.insert(key, dodge.value);
                if dodge.value % 2 == 0 || previous.is_none_or(|p| p % 2 == 1) {
                    continue;
                }
                let [tx, ty, _] = torque.value;
                let (dx, dy) = (ty / 2.24, -tx / 2.60); // dodge_dir: x forward, y right
                let r = (dx * dx + dy * dy).sqrt();
                radii.push(r);
                if r > 0.5 {
                    quadrants[usize::from(dx < 0.0) * 2 + usize::from(dy < 0.0)] += 1;
                    if dy.abs() < 0.05 {
                        axis_counts[usize::from(dx < 0.0)] += 1;
                    } else if dx.abs() < 0.05 {
                        axis_counts[2 + usize::from(dy < 0.0)] += 1;
                    }
                }
            }
        }
    }
    radii.sort_by(|a, b| a.total_cmp(b));
    let n = radii.len();
    println!("dodge activations with a fresh torque: {n}");
    let q = |p: f64| radii[((n - 1) as f64 * p).round() as usize];
    println!(
        "radius sqrt((ty/2.24)^2 + (tx/2.60)^2) p1/p10/p50/p90/p99: {:.3}/{:.3}/{:.3}/{:.3}/{:.3}",
        q(0.01),
        q(0.1),
        q(0.5),
        q(0.9),
        q(0.99)
    );
    let count = |lo: f32, hi: f32| radii.iter().filter(|r| **r >= lo && **r < hi).count();
    println!(
        "radius in [0, 0.05): {} ({:.1}%), [0.05, 0.9): {} ({:.1}%), [0.98, 1.02): {} ({:.1}%), >= 1.02: {} ({:.1}%)",
        count(0.0, 0.05),
        count(0.0, 0.05) as f64 * 100.0 / n as f64,
        count(0.05, 0.9),
        count(0.05, 0.9) as f64 * 100.0 / n as f64,
        count(0.98, 1.02),
        count(0.98, 1.02) as f64 * 100.0 / n as f64,
        count(1.02, 10.0),
        count(1.02, 10.0) as f64 * 100.0 / n as f64
    );
    println!(
        "signs of the implied dodge direction (forward/backward x left/right, y positive = right): forward-left {} forward-right {} backward-left {} backward-right {}",
        quadrants[1], quadrants[0], quadrants[3], quadrants[2]
    );
    println!(
        "pure-axis dodges: forward {} backward {} right {} left {}",
        axis_counts[0], axis_counts[1], axis_counts[2], axis_counts[3]
    );
    Ok(())
}
