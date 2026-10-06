//! Check whether the boost component's ReplicatedActive byte alternates on/off.

use std::collections::HashMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replicar_v1::{observations, parse_replay};

fn collect(root: &Path, paths: &mut Vec<PathBuf>) -> Result<(), Box<dyn Error>> {
    for entry in fs::read_dir(root)? {
        let entry = entry?;
        if entry.file_type()?.is_dir() {
            collect(&entry.path(), paths)?;
        } else if entry.path().extension().is_some_and(|ext| ext == "replay") {
            paths.push(entry.path());
        }
    }
    Ok(())
}

#[derive(Default)]
struct Counts {
    intervals: usize,
    decreasing: usize,
    steady: usize,
    increasing: usize,
    decrease_rate_sum: f64,
}

#[derive(Clone, Copy)]
struct Sample {
    frame: usize,
    time: f32,
    amount: f32,
    raw: u8,
}

fn main() -> Result<(), Box<dyn Error>> {
    let root = env::args_os()
        .nth(1)
        .ok_or("usage: calibrate_boost <replay-directory>")?;
    let mut paths = Vec::new();
    collect(Path::new(&root), &mut paths)?;
    paths.sort();
    let mut by_transition = std::array::from_fn::<_, 4, _>(|_| Counts::default());
    for path in &paths {
        let replay = parse_replay(&replicar_v1::read_replay_file(
            std::path::Path::new(&path),
            false,
        )?)?;
        let observed = observations::extract(&replay).ok_or("network frames absent")?;
        let mut previous = HashMap::<(i32, usize), Sample>::new();
        for frame in &observed.frames {
            for car in &frame.cars {
                let (Some(amount), Some(raw)) = (&car.boost, &car.inputs.boost_active_raw) else {
                    continue;
                };
                if amount.frame != frame.index {
                    continue;
                }
                let key = (car.actor_id, car.actor_created_frame);
                if let Some(old) = previous.get(&key) {
                    let dt = frame.time - old.time;
                    // Require an unchanged active byte between the two amount packets.
                    if dt > 0.0
                        && dt <= 0.25
                        && (raw.frame <= old.frame || raw.frame == frame.index)
                    {
                        let transition = usize::from(old.raw % 2) * 2 + usize::from(raw.value % 2);
                        let counts = &mut by_transition[transition];
                        counts.intervals += 1;
                        let change = amount.value - old.amount;
                        if change < -0.2 {
                            counts.decreasing += 1;
                            counts.decrease_rate_sum += f64::from(-change / dt);
                        } else if change > 0.2 {
                            counts.increasing += 1;
                        } else {
                            counts.steady += 1;
                        }
                    }
                }
                previous.insert(
                    key,
                    Sample {
                        frame: frame.index,
                        time: frame.time,
                        amount: amount.value,
                        raw: raw.value,
                    },
                );
            }
        }
    }
    for (transition, counts) in by_transition.into_iter().enumerate() {
        println!(
            "transition={}→{} intervals={} decreasing={} steady={} increasing={} mean_decrease_rate={:.2} boost/s",
            transition / 2,
            transition % 2,
            counts.intervals,
            counts.decreasing,
            counts.steady,
            counts.increasing,
            counts.decrease_rate_sum / counts.decreasing.max(1) as f64,
        );
    }
    Ok(())
}
