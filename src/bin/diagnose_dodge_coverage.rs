//! Which dodge activations get a fitted start (`infer_dodge_start`) and which do not.
//!
//! Every dodge activation (the dodge counter turning odd with a fresh torque) of the replays under the
//! given directories is classified by what the converter had around it: the last fresh car packet before
//! it (gap in frames, whether the car was on the ground there), whether a fresh packet falls on the
//! activation frame, and how many fresh packets with an exact chain lag follow within 12 frames. The
//! fitted rate of each class shows where the fits stop covering.
//!
//! usage: diagnose_dodge_coverage <dir_or_replay>... [--limit n]

use std::collections::{BTreeMap, HashMap, HashSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::conversion::{ConvertOptions, convert_observations};
use replay_to_rocketsim::observations::extract;

fn collect(path: &Path, out: &mut Vec<PathBuf>) -> std::io::Result<()> {
    if path.is_file() {
        out.push(path.to_path_buf());
    } else {
        for entry in fs::read_dir(path)? {
            collect(&entry?.path(), out)?;
        }
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args: Vec<String> = env::args().skip(1).collect();
    let mut limit = usize::MAX;
    if let Some(at) = args.iter().position(|a| a == "--limit") {
        limit = args
            .get(at + 1)
            .and_then(|n| n.parse().ok())
            .ok_or("--limit n")?;
        args.drain(at..at + 2);
    }
    let mut files = Vec::new();
    for arg in &args {
        if arg.contains("test") {
            return Err("refusing to inspect a path containing 'test'".into());
        }
        collect(Path::new(arg), &mut files)?;
    }
    files.sort();
    files.truncate(limit);
    // class -> (activations, fitted)
    let mut table: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    for path in &files {
        let replay = boxcars::ParserBuilder::new(&fs::read(path)?)
            .must_parse_network_data()
            .parse()?;
        let Some(observed) = extract(&replay) else {
            continue;
        };
        let Ok(output) = convert_observations(observed, &ConvertOptions::default()) else {
            continue;
        };
        let frames = &output.observations.frames;
        let slot_of: HashMap<&str, usize> = output
            .car_slots
            .iter()
            .map(|s| (s.player_key.as_str(), s.slot))
            .collect();
        let fitted: HashSet<(usize, usize)> = output
            .frames
            .iter()
            .flat_map(|f| f.fitted_inputs.iter())
            .filter(|e| e.kind == "dodge")
            .map(|e| (e.slot, e.activation_frame))
            .collect();
        let mut last_raw: HashMap<(i32, usize), u8> = HashMap::new();
        for (f, frame) in frames.iter().enumerate() {
            for car in &frame.cars {
                let Some(&slot) = car.player_key.as_deref().and_then(|k| slot_of.get(k)) else {
                    continue;
                };
                let Some(raw) = car
                    .inputs
                    .dodge_active_raw
                    .as_ref()
                    .filter(|r| r.frame == f)
                else {
                    continue;
                };
                let key = (car.actor_id, car.actor_created_frame);
                let prev = last_raw.insert(key, raw.value);
                let activated = match prev {
                    Some(p) => p % 2 == 0 && raw.value % 2 == 1,
                    None => raw.value % 2 == 1,
                };
                if !activated
                    || replay_to_rocketsim::conversion::activation_torque(frames, f, car).is_none()
                {
                    continue;
                }
                let is_fitted = fitted.contains(&(slot, f));
                let fresh_at = |g: usize| {
                    frames[g]
                        .cars
                        .iter()
                        .find(|c| {
                            c.actor_id == car.actor_id
                                && c.actor_created_frame == car.actor_created_frame
                        })
                        .is_some_and(|c| c.body.position.as_ref().is_some_and(|p| p.frame == g))
                };
                let a = (f.saturating_sub(20)..f).rev().find(|&g| fresh_at(g));
                let (gap, ground, first) = match a {
                    Some(a) => {
                        let st = output.frames[a].state.cars.get(slot).map(|c| c.1);
                        (
                            (f - a) as i64,
                            st.is_some_and(|s| s.is_on_ground),
                            st.is_some_and(|s| !s.has_jumped),
                        )
                    }
                    None => (-1, false, false),
                };
                let exact_after = (f..=(f + 12).min(frames.len() - 1))
                    .filter(|&g| {
                        fresh_at(g)
                            && output.frames[g]
                                .packet_lags
                                .iter()
                                .any(|l| l.actor_id == Some(car.actor_id) && l.source == "chain")
                    })
                    .count()
                    .min(2);
                let any_after = (f..=(f + 12).min(frames.len() - 1))
                    .filter(|&g| fresh_at(g))
                    .count()
                    .min(3);
                // Counters at the last fresh packet before the activation.
                let (jump_odd_at_a, dodge_none_at_a) = match a {
                    Some(a) => frames[a]
                        .cars
                        .iter()
                        .find(|c| {
                            c.actor_id == car.actor_id
                                && c.actor_created_frame == car.actor_created_frame
                        })
                        .map_or((false, false), |c| {
                            (
                                c.inputs
                                    .jump_active_raw
                                    .as_ref()
                                    .is_some_and(|v| v.value % 2 == 1),
                                c.inputs.dodge_active_raw.is_none(),
                            )
                        }),
                    None => (false, false),
                };
                let classes = [
                    "all activations".to_string(),
                    format!(
                        "start: {}",
                        if a.is_none() {
                            "no fresh packet within 20 frames before"
                        } else if ground {
                            "on the ground at the last packet (jump + dodge)"
                        } else {
                            "airborne at the last packet"
                        }
                    ),
                    format!(
                        "frames from the last fresh packet to the activation: {}",
                        match gap {
                            -1 => "none".to_string(),
                            0..=1 => "0-1".to_string(),
                            2 => "2".to_string(),
                            3..=4 => "3-4".to_string(),
                            5..=8 => "5-8".to_string(),
                            _ => "9+".to_string(),
                        }
                    ),
                    format!(
                        "fresh packets with exact chain lag in the next 12 frames: {exact_after}"
                    ),
                    format!("fresh packets (any lag) in the next 12 frames: {any_after}"),
                    format!("has not jumped at the last packet: {first}"),
                    format!(
                        "ground at the last packet; jump counter odd there: {jump_odd_at_a}; dodge counter absent there: {dodge_none_at_a}; exact chain packets after: {exact_after}"
                    ),
                ];
                for class in classes {
                    let entry = table.entry(class).or_default();
                    entry.0 += 1;
                    entry.1 += usize::from(is_fitted);
                }
            }
        }
    }
    println!("{} replays", files.len());
    let names = [
        "inactive or withheld frame in the search window",
        "car missing in a frame of the window",
        "no activation within 14 frames (normal for most packets)",
        "a nearer fresh packet before the activation (normal)",
        "fewer than two usable fresh packets after the activation",
        "no dodge direction",
        "fitted start after the next packet",
        "planned",
        "calls past the entry checks (airborne fresh packet, even counter)",
    ];
    for (i, c) in replay_to_rocketsim::conversion::GROUND_FLIP_COUNTS
        .iter()
        .enumerate()
    {
        let n = c.load(std::sync::atomic::Ordering::Relaxed);
        if n > 0 {
            println!("fit_ground_flip_timing refusal {i}: {n}");
        }
    }
    for (i, name) in names.iter().enumerate() {
        println!(
            "fit_dodge_start {name}: {}",
            replay_to_rocketsim::conversion::DODGE_FIT_COUNTS[i]
                .load(std::sync::atomic::Ordering::Relaxed)
        );
    }
    for (class, (n, fitted)) in &table {
        println!(
            "{class:<80} {n:>6} activations, {fitted:>6} fitted ({:.0}%)",
            100.0 * *fitted as f64 / (*n).max(1) as f64
        );
    }
    Ok(())
}
