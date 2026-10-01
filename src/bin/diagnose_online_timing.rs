//! Timing properties of real online replays (train/validation): how many ball hits bridge a chain,
//! the ball-minus-car offset estimated from them, and each car's median fitted ground-control shift
//! (the recording client's own car is expected to stand out with a large positive shift).
//!
//! usage: diagnose_online_timing <dir or replay> [max replays]

use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::conversion::{
    ConvertOptions, GROUND_SHIFT_LOG, convert_bytes, infer_packet_lags,
};

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
    let mut args = env::args().skip(1);
    let path = PathBuf::from(args.next().ok_or("usage: diagnose_online_timing <dir or replay> [max]")?);
    if path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let max: usize = args.next().and_then(|v| v.parse().ok()).unwrap_or(usize::MAX);
    replay_to_rocketsim::conversion::GROUND_SHIFT_LOG_ENABLED.store(true, std::sync::atomic::Ordering::Relaxed);
    let options = ConvertOptions::default();
    for replay in replay_paths(&path)?.into_iter().take(max) {
        GROUND_SHIFT_LOG.lock().unwrap().clear();
        let output = convert_bytes(&fs::read(&replay)?, &options)?;
        let lags = infer_packet_lags(&output.observations, &options);
        let mut per_actor: std::collections::BTreeMap<i32, Vec<i64>> = Default::default();
        for &(actor, shift, _) in GROUND_SHIFT_LOG.lock().unwrap().iter() {
            per_actor.entry(actor).or_default().push(shift);
        }
        let mut medians: Vec<(i32, usize, i64)> = per_actor
            .into_iter()
            .filter(|(_, v)| v.len() >= 20)
            .map(|(a, mut v)| {
                v.sort_unstable();
                (a, v.len(), v[v.len() / 2])
            })
            .collect();
        medians.sort_by_key(|m| std::cmp::Reverse(m.2));
        let frames = output.observations.frames.len();
        println!(
            "{} frames {} bridged hits {} offset {} | car median shifts (n): {}",
            replay.file_name().unwrap().to_string_lossy().chars().take(8).collect::<String>(),
            frames,
            lags.bridged_hits,
            lags.ball_car_offset.map_or("none".to_string(), |o| format!("{o:.2}")),
            medians
                .iter()
                .map(|(_, n, m)| format!("{m:+}({n})"))
                .collect::<Vec<_>>()
                .join(" ")
        );
    }
    Ok(())
}
