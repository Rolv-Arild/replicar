//! Physical ticks between consecutive fresh car packets, by replay-frame gap.
//!
//! For consecutive fresh position+velocity packets of one car actor lifetime, the displacement
//! along the mean velocity gives the implied elapsed 120 Hz ticks (gravity and acceleration are
//! second order). Histograms by frame gap, the sequential structure across a car's packets, and
//! agreement between two cars over the same frame pair show whether packet times follow a regular
//! cadence, shared per-frame time, or independent jitter. Offline train diagnostic.

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

struct Packet {
    frame: usize,
    time: f64,
    pos: [f32; 3],
    vel: [f32; 3],
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: audit_car_ticks <train dir or replay>")?,
    );
    if replay_to_rocketsim::sealed_path_refused(&path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    // gap frames -> implied tick histogram (bins of 1 tick, clipped to 0..=20)
    let mut histogram: BTreeMap<usize, [usize; 21]> = BTreeMap::new();
    let mut sequence: Vec<Vec<(usize, f64)>> = Vec::new();
    let mut same_frame_pairs: BTreeMap<(usize, usize), Vec<(usize, f64)>> = BTreeMap::new();
    let mut dumped = 0;
    let mut cumulative: Vec<f64> = Vec::new(); // total implied ticks minus total nominal per car

    for replay_path in replay_paths(&path)? {
        let replay = parse_replay(&fs::read(&replay_path)?)?;
        let observed = observations::extract(&replay).ok_or("network frames absent")?;
        let mut lifetimes: BTreeMap<(i32, usize), Vec<Packet>> = BTreeMap::new();
        for frame in &observed.frames {
            if !frame
                .game_state
                .as_ref()
                .is_some_and(|s| s.value == "Active")
            {
                continue;
            }
            for car in &frame.cars {
                let (Some(p), Some(v)) = (
                    car.body
                        .position
                        .as_ref()
                        .filter(|p| p.frame == frame.index),
                    car.body
                        .linear_velocity
                        .as_ref()
                        .filter(|v| v.frame == frame.index),
                ) else {
                    continue;
                };
                lifetimes
                    .entry((car.actor_id, car.actor_created_frame))
                    .or_default()
                    .push(Packet {
                        frame: frame.index,
                        time: f64::from(frame.time),
                        pos: p.value,
                        vel: v.value,
                    });
            }
        }
        for packets in lifetimes.values() {
            let mut seq = Vec::new();
            let mut drift = 0.0;
            for pair in packets.windows(2) {
                let (a, b) = (&pair[0], &pair[1]);
                let gap = b.frame - a.frame;
                let mean: Vec<f64> = (0..3)
                    .map(|i| 0.5 * f64::from(a.vel[i] + b.vel[i]))
                    .collect();
                let speed2: f64 = mean.iter().map(|v| v * v).sum();
                // Straight, fast motion only, so implied time is well conditioned.
                let speed = speed2.sqrt();
                let turn = {
                    let (na, nb) = (
                        a.vel.iter().map(|v| v * v).sum::<f32>().sqrt(),
                        b.vel.iter().map(|v| v * v).sum::<f32>().sqrt(),
                    );
                    if na < 1.0 || nb < 1.0 {
                        1.0
                    } else {
                        (0..3).map(|i| a.vel[i] * b.vel[i]).sum::<f32>() / (na * nb)
                    }
                };
                if speed < 800.0 || turn < 0.99 || !(1..=6).contains(&gap) || b.time - a.time > 0.25
                {
                    seq.clear();
                    drift = 0.0;
                    continue;
                }
                let dot: f64 = (0..3)
                    .map(|i| f64::from(b.pos[i] - a.pos[i]) * mean[i])
                    .sum();
                let implied = dot / speed2 * 120.0;
                let bin = implied.round().clamp(0.0, 20.0) as usize;
                histogram.entry(gap).or_insert([0; 21])[bin] += 1;
                seq.push((gap, implied));
                drift += implied - (b.time - a.time) * 120.0;
                same_frame_pairs
                    .entry((a.frame, b.frame))
                    .or_default()
                    .push((0, implied));
            }
            if seq.len() >= 40 && dumped < 4 {
                dumped += 1;
                let line: Vec<String> = seq
                    .iter()
                    .take(40)
                    .map(|(g, t)| format!("{g}:{:.0}", t))
                    .collect();
                println!("run (frame gap:implied ticks): {}", line.join(" "));
            }
            cumulative.push(drift);
            if seq.len() > 2 {
                sequence.push(seq);
            }
        }
    }

    println!("implied physical ticks between consecutive fresh car packets, by frame gap");
    print!("{:>6}{:>9}", "gap", "n");
    for t in 0..=20 {
        print!("{:>6}", t);
    }
    println!();
    for (gap, hist) in &histogram {
        let n: usize = hist.iter().sum();
        print!("{:>6}{:>9}", gap, n);
        for count in hist {
            print!("{:>6.1}", *count as f64 * 100.0 / n as f64);
        }
        println!("  (% of packets)");
    }
    // Mean implied ticks per gap and per nominal tick
    println!("\nmean implied ticks / (4 * gap frames):");
    for (gap, hist) in &histogram {
        let n: usize = hist.iter().sum();
        let mean: f64 = hist
            .iter()
            .enumerate()
            .map(|(t, c)| t as f64 * *c as f64)
            .sum::<f64>()
            / n as f64;
        println!("  gap {gap}: mean {:.2} ticks (nominal {})", mean, 4 * gap);
    }
    // Sequential structure: correlation of consecutive (implied - 4*gap) values within a car.
    let (mut num, mut den, mut count) = (0.0, 0.0, 0usize);
    for seq in &sequence {
        for w in seq.windows(2) {
            let (x, y) = (w[0].1 - 4.0 * w[0].0 as f64, w[1].1 - 4.0 * w[1].0 as f64);
            num += x * y;
            den += x * x;
            count += 1;
        }
    }
    println!(
        "\nlag-1 autocorrelation of consecutive (implied - nominal) within a car: {:.3} over {count} pairs",
        num / den
    );
    // Agreement of two cars over the same frame pair
    let (mut n2, mut sx, mut sy, mut sxx, mut syy, mut sxy) = (0usize, 0.0, 0.0, 0.0, 0.0, 0.0);
    for pairs in same_frame_pairs.values() {
        if pairs.len() >= 2 {
            let (x, y) = (pairs[0].1, pairs[1].1);
            n2 += 1;
            sx += x;
            sy += y;
            sxx += x * x;
            syy += y * y;
            sxy += x * y;
        }
    }
    let nf = n2 as f64;
    let cov = sxy / nf - sx / nf * sy / nf;
    println!(
        "two cars over the same frame pair: n={n2}, correlation of implied ticks {:.3}",
        cov / ((sxx / nf - (sx / nf).powi(2)) * (syy / nf - (sy / nf).powi(2))).sqrt()
    );
    let mut drift = cumulative;
    drift.sort_by(|a, b| a.total_cmp(b));
    println!(
        "per-car-lifetime cumulative (implied - nominal) ticks over straight segments: p10 {:.0} p50 {:.0} p90 {:.0}",
        drift[drift.len() / 10],
        drift[drift.len() / 2],
        drift[drift.len() * 9 / 10]
    );
    Ok(())
}
