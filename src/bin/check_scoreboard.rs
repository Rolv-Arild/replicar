//! Self-consistency of the reconstructed scoreboard on replays without a server recording: how often
//! the integer the replay shows differs from the ceiling of the reconstructed clock in running
//! frames (a frame right at a change may differ by one), and counts of the clock states.
//!
//! usage: check_scoreboard <dir or replay> [--final-assessment]
use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::scoreboard::reconstruct;

fn replay_paths(path: &Path) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    if path.is_file() {
        return Ok(vec![path.to_owned()]);
    }
    let mut result = Vec::new();
    for size in ["1v1", "2v2", "3v3"] {
        for entry in fs::read_dir(path.join(size))? {
            let p = entry?.path();
            if p.extension().is_some_and(|e| e == "replay") {
                result.push(p);
            }
        }
    }
    result.sort();
    Ok(result)
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(env::args().nth(1).ok_or("usage: check_scoreboard <dir or replay> [--final-assessment]")?);
    // The test split is sealed until the frozen assessment (TEST_PROTOCOL.md); only that run passes the flag.
    if replay_to_rocketsim::sealed_path_refused(&path, env::args().any(|arg| arg == "--final-assessment")) {
        return Err("refusing to inspect a path containing 'test' (pass --final-assessment for the frozen run)".into());
    }
    let (mut running, mut off_by_one, mut off_more, mut missing) = (0usize, 0usize, 0usize, 0usize);
    let mut states: BTreeMap<&'static str, usize> = BTreeMap::new();
    let mut worst: Vec<(usize, String)> = Vec::new();
    for replay in replay_paths(&path)? {
        let bytes = fs::read(&replay)?;
        let (observed, sb) = if env::var_os("CONVERTED").is_some() {
            // Through the converter: the first floor contact of the simulated ball also decides.
            let output = replay_to_rocketsim::conversion::convert_bytes(
                &bytes,
                &replay_to_rocketsim::conversion::ConvertOptions::default(),
            )?;
            let sb: Vec<_> = output.frames.iter().map(|f| f.scoreboard.clone().expect("scoreboard")).collect();
            (output.observations, sb)
        } else {
            let parsed = replay_to_rocketsim::parse_replay(&bytes)?;
            let observed = replay_to_rocketsim::observations::extract(&parsed).ok_or("no observations")?;
            let sb = reconstruct(&observed);
            (observed, sb)
        };
        let (mut bad, mut n) = (0usize, 0usize);
        for (f, frame) in observed.frames.iter().enumerate() {
            *states.entry(sb[f].clock_state).or_default() += 1;
            let overtime = sb[f].period == "overtime";
            if sb[f].clock_state != "running" {
                continue;
            }
            let (Some(shown), Some(x)) = (
                frame.seconds_remaining.as_ref().map(|v| v.value),
                if overtime { sb[f].overtime_seconds } else { sb[f].seconds_remaining },
            ) else {
                missing += 1;
                continue;
            };
            running += 1;
            n += 1;
            let expected = x.ceil() as i32;
            match (shown - expected).abs() {
                0 => {}
                1 => off_by_one += 1,
                _ => {
                    off_more += 1;
                    bad += 1;
                }
            }
        }
        let expired = sb.iter().filter(|x| x.clock_state == "expired").count();
        let decided = sb.iter().filter(|x| x.clock_state == "decided").count();
        let ended_state = observed.frames.last().and_then(|f| f.game_state.as_ref()).map(|g| g.value.clone());
        if env::var_os("PER_REPLAY").is_some() && (expired > 0 || decided > 0) {
            let min_z = observed
                .frames
                .iter()
                .enumerate()
                .filter(|(f, _)| sb[*f].clock_state == "expired" || sb[*f].clock_state == "decided")
                .filter_map(|(f, fr)| fr.ball.as_ref().and_then(|b| b.position.as_ref()).filter(|p| p.frame == f).map(|p| p.value[2]))
                .fold(f32::INFINITY, f32::min);
            println!("{} expired {} decided {} min fresh ball z after expiry {:.0} last state {:?}", replay.file_name().unwrap().to_string_lossy().chars().take(8).collect::<String>(), expired, decided, min_z, ended_state);
        }
        if bad > 0 {
            worst.push((bad, format!("{} ({} running frames)", replay.file_name().unwrap().to_string_lossy().chars().take(8).collect::<String>(), n)));
        }
    }
    println!("running frames {running}: shown integer equals ceil of the reconstruction {:.3}%, differs by one {:.3}%, by more {:.3}% ({off_more}); running frames without a value {missing}", 100.0 * (running - off_by_one - off_more) as f64 / running as f64, 100.0 * off_by_one as f64 / running as f64, 100.0 * off_more as f64 / running as f64);
    println!("clock states: {states:?}");
    worst.sort_by(|a, b| b.0.cmp(&a.0));
    println!("replays with frames off by more than one: {:?}", &worst[..worst.len().min(10)]);
    Ok(())
}
