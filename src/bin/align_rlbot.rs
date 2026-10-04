//! Align an RLBot recording (`states.jsonl`, one packet per physics frame) to its saved replay.
//!
//! For each sampled replay frame with fresh car packets, the recording's frame with the same cars'
//! positions is searched: the offset `O` in `frame_num = round(replay_time * 120) + O` that minimises
//! the summed nearest-car distance over the sample is reported with the residual distribution, and the
//! per-frame best offsets (they show whether the replay's timeline drifts against the recording's).
//!
//! usage: `align_rlbot <replay> <states.jsonl>`

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs::{self, File};
use std::io::{BufRead, BufReader};
use std::path::PathBuf;

use glam::Vec3A;
use replicar::observations::extract;
use serde_json::Value;

fn quantile(values: &mut [f32], q: f64) -> f32 {
    if values.is_empty() {
        return f32::NAN;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let replay_path = PathBuf::from(
        args.next()
            .ok_or("usage: align_rlbot <replay> <states.jsonl>")?,
    );
    let states_path = PathBuf::from(
        args.next()
            .ok_or("usage: align_rlbot <replay> <states.jsonl>")?,
    );
    if replicar::sealed_path_refused(&replay_path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let replay = boxcars::ParserBuilder::new(&fs::read(&replay_path)?)
        .must_parse_network_data()
        .parse()?;
    let observed = extract(&replay).ok_or("no network frames")?;
    // frame_num -> (phase, car positions, ball position)
    let mut packets: BTreeMap<u64, (u64, Vec<Vec3A>)> = BTreeMap::new();
    for line in BufReader::new(File::open(&states_path)?).lines() {
        let row: Value = serde_json::from_str(&line?)?;
        let p = &row["packet"];
        let info = &p["match_info"];
        let (Some(frame), Some(phase)) = (info["frame_num"].as_u64(), info["match_phase"].as_u64())
        else {
            continue;
        };
        let cars = p["players"]
            .as_array()
            .ok_or("players")?
            .iter()
            .map(|pl| {
                let l = &pl["physics"]["location"];
                Vec3A::new(
                    l["x"].as_f64().unwrap_or(0.0) as f32,
                    l["y"].as_f64().unwrap_or(0.0) as f32,
                    l["z"].as_f64().unwrap_or(0.0) as f32,
                )
            })
            .collect();
        packets.insert(frame, (phase, cars));
    }
    let (first, last) = (
        *packets.keys().next().ok_or("no packets")?,
        *packets.keys().last().ok_or("no packets")?,
    );
    println!(
        "recording frames {first}..{last}; replay frames {} ({:.1} s)",
        observed.frames.len(),
        observed.frames.last().map_or(0.0, |f| f.time)
    );
    let t0 = f64::from(observed.frames[0].time);
    // Sampled replay frames: fresh car positions (those of the linked players).
    let mut samples: Vec<(usize, i64, Vec<Vec3A>)> = Vec::new();
    for (f, frame) in observed.frames.iter().enumerate().step_by(7) {
        let cars: Vec<Vec3A> = frame
            .cars
            .iter()
            .filter(|c| c.player_key.is_some())
            .filter_map(|c| c.body.position.as_ref().filter(|p| p.frame == f))
            .map(|p| Vec3A::from_array(p.value))
            .collect();
        if cars.len() >= 3 {
            samples.push((
                f,
                ((f64::from(frame.time) - t0) * 120.0).round() as i64,
                cars,
            ));
        }
    }
    println!(
        "{} sampled replay frames with >= 3 fresh cars",
        samples.len()
    );
    let distance = |cars: &[Vec3A], frame: u64| -> Option<f32> {
        let (_, packet) = packets.get(&frame)?;
        Some(
            cars.iter()
                .map(|c| {
                    packet
                        .iter()
                        .map(|q| (*q - *c).length())
                        .fold(f32::INFINITY, f32::min)
                })
                .sum::<f32>()
                / cars.len() as f32,
        )
    };
    // Per-frame best offset within +-2400 ticks of a coarse global scan.
    let mut best_global = (f32::INFINITY, 0i64);
    for offset in (first as i64 - 300..first as i64 + 1200).step_by(1) {
        let mut total = 0.0;
        let mut n = 0;
        for (_, tick, cars) in samples.iter().step_by(11) {
            if let Some(d) = distance(cars, (tick + offset).max(0) as u64) {
                total += d;
                n += 1;
            }
        }
        if n > 5 && total / n as f32 <= best_global.0 {
            best_global = (total / n as f32, offset);
        }
    }
    println!(
        "coarse global offset {} (mean nearest-car distance {:.1} UU)",
        best_global.1, best_global.0
    );
    let mut per_frame: Vec<(usize, i64, f32)> = Vec::new();
    for (f, tick, cars) in &samples {
        let mut best = (f32::INFINITY, 0i64);
        for d in -40..=40i64 {
            let frame = (tick + best_global.1 + d).max(0) as u64;
            if let Some(dist) = distance(cars, frame)
                && dist < best.0
            {
                best = (dist, d);
            }
        }
        per_frame.push((*f, best.1, best.0));
    }
    let mut hist: BTreeMap<i64, usize> = BTreeMap::new();
    for (_, d, dist) in &per_frame {
        if *dist < 8.0 {
            *hist.entry(*d).or_default() += 1;
        }
    }
    println!("per-frame best extra offset (ticks) among frames with distance < 8 UU: {hist:?}");
    let mut dists: Vec<f32> = per_frame.iter().map(|x| x.2).collect();
    println!(
        "best distance p10/p50/p90/p99: {:.2} / {:.2} / {:.2} / {:.2}",
        quantile(&mut dists, 0.1),
        quantile(&mut dists, 0.5),
        quantile(&mut dists, 0.9),
        quantile(&mut dists, 0.99)
    );
    for (f, d, dist) in per_frame.iter().step_by(per_frame.len() / 12 + 1) {
        println!("  replay frame {f}: best extra offset {d}, distance {dist:.2} UU");
    }
    Ok(())
}
