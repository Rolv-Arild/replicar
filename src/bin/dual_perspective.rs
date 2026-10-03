//! Compare two replays of the same match saved by different clients.
//!
//! Replay packets are exact server states, so a car (or ball) state that appears bit-identically in both
//! replays is the same physics tick. For each replay the cadence of fresh packets is reported, then the
//! exact matches between the two: how many of A's packets appear in B, and the spread of the replay-time
//! difference of matched packets (the difference of the two clients' packet lags plus their clock
//! offset, which is constant per match).
//!
//! usage: dual_perspective <replay_a> <replay_b>

use std::collections::HashMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

use replay_to_rocketsim::conversion::{ConvertOptions, convert_observations};
use replay_to_rocketsim::observations::{ObservedReplay, extract};

fn quantile(values: &mut [f64], q: f64) -> f64 {
    if values.is_empty() {
        return f64::NAN;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

/// (object key, frame, time, position bits) of every fresh position packet.
struct Event {
    key: String,
    frame: usize,
    time: f64,
    bits: [u32; 3],
}

fn events(observed: &ObservedReplay) -> (Vec<Event>, Vec<Event>) {
    let mut cars = Vec::new();
    let mut balls = Vec::new();
    for (f, frame) in observed.frames.iter().enumerate() {
        for car in &frame.cars {
            let (Some(key), Some(p)) = (
                car.player_key.as_ref(),
                car.body.position.as_ref().filter(|p| p.frame == f),
            ) else {
                continue;
            };
            cars.push(Event {
                key: key.clone(),
                frame: f,
                time: f64::from(frame.time),
                bits: p.value.map(f32::to_bits),
            });
        }
        if let Some(p) = frame
            .ball
            .as_ref()
            .and_then(|b| b.position.as_ref())
            .filter(|p| p.frame == f)
        {
            balls.push(Event {
                key: "ball".into(),
                frame: f,
                time: f64::from(frame.time),
                bits: p.value.map(f32::to_bits),
            });
        }
    }
    (cars, balls)
}

fn cadence(label: &str, observed: &ObservedReplay, events: &[Event]) {
    let mut by_key: HashMap<&str, Vec<&Event>> = HashMap::new();
    for e in events {
        by_key.entry(&e.key).or_default().push(e);
    }
    let mut gaps_frames = Vec::new();
    let mut gaps_ms = Vec::new();
    for list in by_key.values() {
        for w in list.windows(2) {
            gaps_frames.push((w[1].frame - w[0].frame) as f64);
            gaps_ms.push((w[1].time - w[0].time) * 1000.0);
        }
    }
    println!(
        "{label}: {} frames ({:.1} s), {} fresh packets over {} objects; gap between an object's packets: frames p10/p50/p90 {:.0}/{:.0}/{:.0}, ms {:.0}/{:.0}/{:.0}",
        observed.frames.len(),
        observed.frames.last().map_or(0.0, |f| f.time),
        events.len(),
        by_key.len(),
        quantile(&mut gaps_frames.clone(), 0.1),
        quantile(&mut gaps_frames.clone(), 0.5),
        quantile(&mut gaps_frames, 0.9),
        quantile(&mut gaps_ms.clone(), 0.1),
        quantile(&mut gaps_ms.clone(), 0.5),
        quantile(&mut gaps_ms, 0.9),
    );
}

fn compare(label: &str, a: &[Event], b: &[Event]) {
    let mut index: HashMap<(&str, [u32; 3]), Vec<&Event>> = HashMap::new();
    for e in b {
        index.entry((&e.key, e.bits)).or_default().push(e);
    }
    let mut own: HashMap<(&str, [u32; 3]), usize> = HashMap::new();
    for e in a {
        *own.entry((&e.key, e.bits)).or_default() += 1;
    }
    let mut matched = 0usize;
    let mut diffs = Vec::new();
    let mut moving = 0usize;
    for e in a {
        if let Some(list) = index.get(&(e.key.as_str(), e.bits)) {
            matched += 1;
            // A resting object repeats one position over many ticks: only positions that occur once in
            // each replay identify a tick.
            if list.len() == 1 && own[&(e.key.as_str(), e.bits)] == 1 {
                diffs.push(e.time - list[0].time);
                moving += 1;
            }
        }
    }
    if std::env::var_os("DUAL_SERIES").is_some() {
        // Time difference (ticks, relative to the median) by 10 s bucket of A's time.
        let median = quantile(&mut diffs.clone(), 0.5);
        let mut buckets: std::collections::BTreeMap<i64, Vec<f64>> = std::collections::BTreeMap::new();
        for e in a {
            if let Some(list) = index.get(&(e.key.as_str(), e.bits)) {
                if list.len() == 1 && own[&(e.key.as_str(), e.bits)] == 1 {
                    buckets
                        .entry((e.time / 10.0) as i64)
                        .or_default()
                        .push((e.time - list[0].time - median) * 120.0);
                }
            }
        }
        for (bucket, mut v) in buckets {
            let inside = v.iter().filter(|d| d.abs() <= 6.0).count();
            println!(
                "  t={:>3}s n {:>4} in +-6 ticks {:>3.0}%  p10/p50/p90 {:>6.1}/{:>6.1}/{:>6.1}",
                bucket * 10,
                v.len(),
                100.0 * inside as f64 / v.len() as f64,
                quantile(&mut v.clone(), 0.1),
                quantile(&mut v.clone(), 0.5),
                quantile(&mut v, 0.9)
            );
        }
    }
    if std::env::var_os("DUAL_HIST").is_some() {
        let median = quantile(&mut diffs.clone(), 0.5);
        let mut hist: std::collections::BTreeMap<i64, usize> = std::collections::BTreeMap::new();
        for d in &diffs {
            *hist.entry(((d - median) * 120.0).round() as i64).or_default() += 1;
        }
        println!("  histogram of (time difference - median) in ticks: {hist:?}");
    }
    let median = quantile(&mut diffs.clone(), 0.5);
    let mut spread: Vec<f64> = diffs.iter().map(|d| (d - median) * 1000.0).collect();
    println!(
        "{label}: {matched} of {} packets of A appear bit-identically in B ({:.1}%); replay-time difference median {:.3} s; deviation from it (ms) p1/p10/p50/p90/p99: {:.1}/{:.1}/{:.1}/{:.1}/{:.1} ({moving} unique-position pairs)",
        a.len(),
        100.0 * matched as f64 / a.len().max(1) as f64,
        median,
        quantile(&mut spread.clone(), 0.01),
        quantile(&mut spread.clone(), 0.1),
        quantile(&mut spread.clone(), 0.5),
        quantile(&mut spread.clone(), 0.9),
        quantile(&mut spread, 0.99),
    );
}

/// Inferred chain lag (ticks) of each frame's fresh ball packet, from the converter.
fn ball_lags(observed: &ObservedReplay) -> Result<Vec<Option<i64>>, Box<dyn Error>> {
    let output = convert_observations(observed.clone(), &ConvertOptions::default())?;
    Ok(output
        .frames
        .iter()
        .map(|f| {
            f.packet_lags
                .iter()
                .find(|l| l.actor_id.is_none() && l.source == "chain")
                .map(|l| l.ticks as i64)
        })
        .collect())
}

fn detrended(values: &[(f64, f64)], window: usize) -> Vec<f64> {
    // (time, value) sorted by time; value minus the running median of the neighbours.
    (0..values.len())
        .map(|i| {
            let lo = i.saturating_sub(window);
            let hi = (i + window + 1).min(values.len());
            let mut near: Vec<f64> = values[lo..hi].iter().map(|v| v.1).collect();
            values[i].1 - quantile(&mut near, 0.5)
        })
        .collect()
}

fn lag_check(a: &ObservedReplay, b: &ObservedReplay, a_balls: &[Event], b_balls: &[Event]) -> Result<(), Box<dyn Error>> {
    let (lag_a, lag_b) = (ball_lags(a)?, ball_lags(b)?);
    let mut index: HashMap<[u32; 3], Vec<&Event>> = HashMap::new();
    for e in b_balls {
        index.entry(e.bits).or_default().push(e);
    }
    let mut own: HashMap<[u32; 3], usize> = HashMap::new();
    for e in a_balls {
        *own.entry(e.bits).or_default() += 1;
    }
    // (time of A, raw difference in ticks, difference corrected by the inferred lags)
    let mut raw = Vec::new();
    let mut corrected = Vec::new();
    for e in a_balls {
        let Some(list) = index.get(&e.bits) else { continue };
        if list.len() != 1 || own[&e.bits] != 1 {
            continue;
        }
        let f = list[0];
        let (Some(la), Some(lb)) = (lag_a[e.frame], lag_b[f.frame]) else { continue };
        let d = (e.time - f.time) * 120.0;
        raw.push((e.time, d));
        corrected.push((e.time, d - (la - lb) as f64));
    }
    for window in [25usize, 100] {
        let (mut r, mut c) = (detrended(&raw, window), detrended(&corrected, window));
        let (mut ra, mut ca): (Vec<f64>, Vec<f64>) = (r.iter().map(|x| x.abs()).collect(), c.iter().map(|x| x.abs()).collect());
        println!(
            "lag check (ball packets with an inferred chain lag in both replays: {}; detrended by a running median of +-{window} pairs), ticks: raw p10/p50/p90 {:.1}/{:.1}/{:.1}, |raw| p50/p90 {:.1}/{:.1}; after subtracting the inferred lag difference p10/p50/p90 {:.1}/{:.1}/{:.1}, |.| p50/p90 {:.1}/{:.1}",
            raw.len(),
            quantile(&mut r.clone(), 0.1), quantile(&mut r.clone(), 0.5), quantile(&mut r, 0.9),
            quantile(&mut ra.clone(), 0.5), quantile(&mut ra, 0.9),
            quantile(&mut c.clone(), 0.1), quantile(&mut c.clone(), 0.5), quantile(&mut c, 0.9),
            quantile(&mut ca.clone(), 0.5), quantile(&mut ca, 0.9),
        );
    }
    Ok(())
}

/// Whether one replay frame holds one tick: the objects of a frame of A that also occur in B, and the
/// B frames they occur in (if a frame were a single tick, every object of it would be in one B frame,
/// given that the other client received that whole tick).
fn frame_coherence(label: &str, a: &[Event], b: &[Event]) {
    let mut index: HashMap<(&str, [u32; 3]), Vec<&Event>> = HashMap::new();
    for e in b {
        index.entry((&e.key, e.bits)).or_default().push(e);
    }
    let mut own: HashMap<(&str, [u32; 3]), usize> = HashMap::new();
    for e in a {
        *own.entry((&e.key, e.bits)).or_default() += 1;
    }
    let mut by_frame: HashMap<usize, Vec<(&str, usize)>> = HashMap::new();
    for e in a {
        if let Some(list) = index.get(&(e.key.as_str(), e.bits)) {
            if list.len() == 1 && own[&(e.key.as_str(), e.bits)] == 1 {
                by_frame.entry(e.frame).or_default().push((&e.key, list[0].frame));
            }
        }
    }
    let mut multi = 0usize;
    let mut same = 0usize;
    let mut spans: std::collections::BTreeMap<usize, usize> = Default::default();
    for objects in by_frame.values().filter(|o| o.len() >= 2) {
        multi += 1;
        let lo = objects.iter().map(|o| o.1).min().unwrap();
        let hi = objects.iter().map(|o| o.1).max().unwrap();
        if lo == hi {
            same += 1;
        }
        *spans.entry(hi - lo).or_default() += 1;
    }
    println!(
        "{label}: of {multi} frames of A with >= 2 objects (ball or cars) found in B, {same} ({:.0}%) have all of them in one frame of B; spread of B frames (frames: count) {spans:?}",
        100.0 * same as f64 / multi.max(1) as f64
    );
}

/// Within one frame, the inferred lag of each fresh packet (ball and cars, chain lags only): if a frame
/// were one tick, they would all be equal.
fn lag_coherence(label: &str, observed: &ObservedReplay) -> Result<(), Box<dyn Error>> {
    let output = convert_observations(observed.clone(), &ConvertOptions::default())?;
    let mut frames = 0usize;
    let mut equal = 0usize;
    let mut spread: std::collections::BTreeMap<i64, usize> = Default::default();
    for f in &output.frames {
        let lags: Vec<i64> = f
            .packet_lags
            .iter()
            .filter(|l| l.source == "chain")
            .map(|l| l.ticks as i64)
            .collect();
        if lags.len() < 2 {
            continue;
        }
        frames += 1;
        let range = lags.iter().max().unwrap() - lags.iter().min().unwrap();
        if range == 0 {
            equal += 1;
        }
        *spread.entry(range).or_default() += 1;
    }
    println!(
        "{label}: of {frames} frames with >= 2 chain-lag packets (ball and cars), {equal} ({:.0}%) have equal lags; max-min lag within a frame (ticks: frames) {spread:?}",
        100.0 * equal as f64 / frames.max(1) as f64
    );
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let a_path = PathBuf::from(args.next().ok_or("usage: dual_perspective <a> <b>")?);
    let b_path = PathBuf::from(args.next().ok_or("usage: dual_perspective <a> <b>")?);
    for path in [&a_path, &b_path] {
        if replay_to_rocketsim::sealed_path_refused(&path, false) {
            return Err("refusing to inspect a path containing 'test'".into());
        }
    }
    let load = |path: &PathBuf| -> Result<ObservedReplay, Box<dyn Error>> {
        let replay = boxcars::ParserBuilder::new(&fs::read(path)?)
            .must_parse_network_data()
            .parse()?;
        Ok(extract(&replay).ok_or("no network frames")?)
    };
    let (a, b) = (load(&a_path)?, load(&b_path)?);
    let (a_cars, a_balls) = events(&a);
    let (b_cars, b_balls) = events(&b);
    cadence("A cars", &a, &a_cars);
    cadence("B cars", &b, &b_cars);
    cadence("A ball", &a, &a_balls);
    cadence("B ball", &b, &b_balls);
    compare("cars A in B", &a_cars, &b_cars);
    compare("cars B in A", &b_cars, &a_cars);
    compare("ball A in B", &a_balls, &b_balls);
    compare("ball B in A", &b_balls, &a_balls);
    let all_a: Vec<Event> = a_cars.iter().chain(&a_balls).map(|e| Event { key: e.key.clone(), frame: e.frame, time: e.time, bits: e.bits }).collect();
    let all_b: Vec<Event> = b_cars.iter().chain(&b_balls).map(|e| Event { key: e.key.clone(), frame: e.frame, time: e.time, bits: e.bits }).collect();
    frame_coherence("frame coherence A to B", &all_a, &all_b);
    frame_coherence("frame coherence B to A", &all_b, &all_a);
    lag_coherence("lag coherence A", &a)?;
    lag_coherence("lag coherence B", &b)?;
    lag_check(&a, &b, &a_balls, &b_balls)?;
    Ok(())
}
