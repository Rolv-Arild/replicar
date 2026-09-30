//! Compare the converter's fitted jump and dodge inputs with the true inputs of a BakkesMod state dump.
//!
//! The dump's replay is thinned to every K-th car packet (as in `dump_reconstruction`), converted with
//! `zero_packet_lag`, and each jump press, dodge press, dodge direction and pitch cancel the converter
//! fitted (`ConvertedFrame::fitted_inputs`) is matched to the dump's true event:
//!
//! - jump: the true press lies in the 4-tick window before the first record with `b_jumped`; the
//!   wheels leave the ground `time_offGround` before the first record with air time, which fixes the
//!   press tick to a constant offset (reported as fitted press minus wheels-off tick);
//! - dodge: first record with `b_isdodging`; press window likewise; the direction is
//!   (`DodgeForward`, `DodgeStrafe`) (pitch = -DodgeForward); the true cancel is the pitch input during
//!   the flip signed by the dodge's forward component (1 = full cancel of the flip's pitch torque).
//!
//! usage: dump_inputs <replay> <dump.json> [--player name] [K...]

use std::collections::HashMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

use replay_to_rocketsim::conversion::{ConvertOptions, FittedInput, convert_observations};
use replay_to_rocketsim::observations::{Body, extract};
use serde_json::Value;

fn quantile(values: &mut [f32], q: f64) -> f32 {
    if values.is_empty() {
        return f32::NAN;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn summary(label: &str, values: &mut Vec<f32>) {
    let abs: Vec<f32> = values.iter().map(|v| v.abs()).collect();
    let mut abs = abs;
    println!(
        "  {label:<44} n {:>3}  p10/p50/p90 {:>6.2}/{:>6.2}/{:>6.2}  |.| p50/p90 {:>5.2}/{:>5.2}",
        values.len(),
        quantile(values, 0.1),
        quantile(values, 0.5),
        quantile(values, 0.9),
        quantile(&mut abs, 0.5),
        quantile(&mut abs, 0.9),
    );
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 2 {
        return Err("usage: dump_inputs <replay> <dump.json> [--player name] [K...]".into());
    }
    let replay_path = PathBuf::from(args.remove(0));
    let dump_path = PathBuf::from(args.remove(0));
    if replay_path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let mut player_name = "Vync62".to_string();
    if let Some(at) = args.iter().position(|a| a == "--player") {
        player_name = args.get(at + 1).cloned().ok_or("--player needs a name")?;
        args.drain(at..at + 2);
    }
    let ks: Vec<usize> = if args.is_empty() {
        vec![2, 4]
    } else {
        args.iter().filter_map(|a| a.parse().ok()).collect()
    };
    let verbose = env::var_os("DUMP_VERBOSE").is_some();

    let replay = boxcars::ParserBuilder::new(&fs::read(&replay_path)?)
        .must_parse_network_data()
        .parse()?;
    let observed = extract(&replay).ok_or("no network frames")?;
    let dump: Value = serde_json::from_str(&fs::read_to_string(&dump_path)?)?;
    let frames = dump["frames"].as_array().ok_or("frames")?;
    let state = |j: usize| &frames[j]["players"][0]["state"];
    let flag = |j: usize, key: &str| state(j)[key].as_bool().unwrap_or(false);
    let input = |j: usize, key: &str| state(j)["inputs"][key].as_f64().unwrap_or(0.0) as f32;
    let player_key = observed
        .frames
        .iter()
        .flat_map(|f| f.players.iter())
        .find(|p| p.name.as_deref() == Some(player_name.as_str()))
        .map(|p| p.key.clone())
        .ok_or("player not found in the replay")?;

    // True events: (record where the flag first shows).
    let mut jumps = Vec::new();
    let mut dodges = Vec::new();
    for j in 1..frames.len() {
        if flag(j, "b_jumped") && !flag(j - 1, "b_jumped") {
            jumps.push(j);
        }
        if flag(j, "b_isdodging") && !flag(j - 1, "b_isdodging") {
            dodges.push(j);
        }
    }
    println!(
        "true events in the dump: {} jumps, {} dodges",
        jumps.len(),
        dodges.len()
    );
    // Wheels-off tick of a jump: first record at or after j with air time, minus that air time.
    let wheels_off = |j: usize| -> Option<i64> {
        (j..(j + 6).min(frames.len())).find_map(|r| {
            let air = state(r)["time_offGround"].as_f64().unwrap_or(0.0);
            (air > 0.0).then(|| 4 * r as i64 - (air * 120.0).round() as i64)
        })
    };

    for &k in &ks {
        let mut thinned = observed.clone();
        let mut last: HashMap<(i32, usize), Body> = HashMap::new();
        for (f, frame) in thinned.frames.iter_mut().enumerate() {
            for car in &mut frame.cars {
                let key = (car.actor_id, car.actor_created_frame);
                match last.get(&key) {
                    Some(body) if f % k != 0 => car.body = body.clone(),
                    _ => {
                        last.insert(key, car.body.clone());
                    }
                }
            }
        }
        let mut options = ConvertOptions::default();
        options.zero_packet_lag = true;
        let output = convert_observations(thinned, &options)?;
        let slot = output
            .car_slots
            .iter()
            .find(|s| s.player_key == player_key)
            .ok_or("player has no slot")?
            .slot;
        // Fitted inputs with the frame that fitted them.
        let fitted: Vec<(usize, &FittedInput)> = output
            .frames
            .iter()
            .enumerate()
            .flat_map(|(f, frame)| frame.fitted_inputs.iter().map(move |e| (f, e)))
            .filter(|(_, e)| e.slot == slot)
            .collect();
        println!(
            "\nK = {k}: {} fitted jump presses, {} fitted dodge presses",
            fitted.iter().filter(|(_, e)| e.kind == "jump").count(),
            fitted.iter().filter(|(_, e)| e.kind == "dodge").count()
        );

        let mut jump_vs_window = Vec::new();
        let mut jump_vs_wheels = Vec::new();
        let mut unmatched_jumps = 0;
        for &j in &jumps {
            let true_tick = 4 * j as i64;
            // The latest fit whose press lies near the true window.
            let Some((f, e)) = fitted
                .iter()
                .filter(|(_, e)| e.kind == "jump" && (e.tick as i64 - true_tick).abs() <= 16)
                .last()
            else {
                unmatched_jumps += 1;
                if verbose {
                    let near: Vec<_> = fitted
                        .iter()
                        .filter(|(_, e)| {
                            e.kind == "jump" && (e.tick as i64 - true_tick).abs() <= 200
                        })
                        .map(|(f, e)| (*f, e.tick))
                        .collect();
                    println!(
                        "  jump record {j}: no fitted press (window ({}, {}], wheels off {:?}); fitted jumps within 200 ticks: {near:?}; on ground before {:.2} s",
                        true_tick - 4,
                        true_tick,
                        wheels_off(j),
                        state(j - 1)["time_onGround"].as_f64().unwrap_or(0.0)
                    );
                }
                continue;
            };
            jump_vs_window.push(e.tick as f32 - true_tick as f32);
            if let Some(off) = wheels_off(j) {
                jump_vs_wheels.push(e.tick as f32 - off as f32);
            }
            if verbose {
                println!(
                    "  jump record {j}: fitted press {} (frame {f}), window ({}, {}], wheels off {:?}",
                    e.tick,
                    true_tick - 4,
                    true_tick,
                    wheels_off(j)
                );
            }
        }
        summary(
            "jump press - first b_jumped record tick",
            &mut jump_vs_window,
        );
        summary("jump press - wheels-off tick", &mut jump_vs_wheels);
        println!("  jumps without a fitted press: {unmatched_jumps}");

        let mut dodge_vs_window = Vec::new();
        let mut pitch_error = Vec::new();
        let mut yaw_error = Vec::new();
        let mut cancel_error = Vec::new();
        let mut cancel_true = Vec::new();
        let mut unmatched_dodges = 0;
        for &j in &dodges {
            let true_tick = 4 * j as i64;
            let Some((f, e)) = fitted
                .iter()
                .filter(|(_, e)| e.kind == "dodge" && (e.tick as i64 - true_tick).abs() <= 16)
                .last()
            else {
                unmatched_dodges += 1;
                continue;
            };
            dodge_vs_window.push(e.tick as f32 - true_tick as f32);
            let (forward, strafe) = (input(j, "DodgeForward"), input(j, "DodgeStrafe"));
            pitch_error.push(e.pitch - (-forward));
            yaw_error.push(e.yaw - strafe);
            // True cancel over the plan's ticks (press to the next packet's frame): the pitch input,
            // interpolated linearly between records, signed by the dodge's forward component and
            // clamped to 0..1 (pitch against the flip's own pitch torque cancels it), averaged.
            let sign = if forward >= 0.0 { 1.0 } else { -1.0 };
            let end_tick = 4 * (*f + k).min(frames.len() - 1) as i64;
            let signed_pitch = |t: i64| -> f32 {
                let (r, frac) = (t.div_euclid(4) as usize, t.rem_euclid(4) as f32 / 4.0);
                let a = input(r.min(frames.len() - 1), "Pitch");
                let b = input((r + 1).min(frames.len() - 1), "Pitch");
                (a + (b - a) * frac) * sign
            };
            let ticks: Vec<f32> = (e.tick as i64 + 1..=end_tick)
                .map(|t| signed_pitch(t).clamp(0.0, 1.0))
                .collect();
            let records: Vec<f32> = ticks.clone();
            if !ticks.is_empty() && forward.abs() > 0.3 {
                let mean = ticks.iter().sum::<f32>() / ticks.len() as f32;
                cancel_true.push(mean);
                cancel_error.push(e.cancel - mean);
            }
            if verbose {
                println!(
                    "  dodge record {j}: fitted press {} (frame {f}), window ({}, {}]; direction fitted (pitch {:.2}, yaw {:.2}) true (forward {:.2}, strafe {:.2}); cancel fitted {:.2} true mean over the plan {:.2}",
                    e.tick,
                    true_tick - 4,
                    true_tick,
                    e.pitch,
                    e.yaw,
                    forward,
                    strafe,
                    e.cancel,
                    (records.iter().sum::<f32>() / records.len().max(1) as f32 * 100.0).round()
                        / 100.0
                );
            }
        }
        summary(
            "dodge press - first b_isdodging record tick",
            &mut dodge_vs_window,
        );
        summary("dodge pitch control - (-DodgeForward)", &mut pitch_error);
        summary("dodge yaw control - DodgeStrafe", &mut yaw_error);
        summary(
            "fitted cancel - true cancel over plan ticks",
            &mut cancel_error,
        );
        summary("true cancel over plan ticks itself", &mut cancel_true);
        println!("  dodges without a fitted press: {unmatched_dodges}");
    }
    Ok(())
}
