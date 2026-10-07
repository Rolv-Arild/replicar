//! Lists every play segment's end kind with the evidence for it (docs/v2-plan.md, story 6.2): the goal
//! reports, the clock phase and the game state after the segment, and whether the replay ends there.
//!
//! usage: `segments <replay or folder>... [--final-assessment]`

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::process::ExitCode;

use replicar::annotate::scoreboard::reconstruct;
use replicar::annotate::segments::segments;
use replicar_eval::collect_replays;

fn main() -> ExitCode {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let final_assessment = args.iter().any(|a| a == "--final-assessment");
    args.retain(|a| a != "--final-assessment");
    let roots: Vec<PathBuf> = args.iter().map(PathBuf::from).collect();
    let replays = match collect_replays(&roots, final_assessment) {
        Ok(replays) => replays,
        Err(error) => {
            eprintln!("error: {error}");
            return ExitCode::FAILURE;
        }
    };
    let mut kinds: BTreeMap<&str, usize> = BTreeMap::new();
    for path in &replays {
        let decoded = std::fs::read(path)
            .map_err(Box::<dyn std::error::Error>::from)
            .and_then(|bytes| Ok(replicar::decode::decode(&replicar::parse(&bytes)?)?));
        let network = match decoded {
            Ok(network) => network,
            Err(error) => {
                println!("FAILED {}  {error}", path.display());
                continue;
            }
        };
        let frames = &network.frames;
        let scoreboard = reconstruct(frames);
        for (index, segment) in segments(frames, &scoreboard).iter().enumerate() {
            *kinds.entry(segment.end.name()).or_default() += 1;
            let after = frames
                .get(segment.last + 1)
                .and_then(|f| f.game_state.as_ref())
                .map(|s| format!("{:?}", s.value));
            println!(
                "{}  segment {index} frames {}..={} end {}  clock {} {:?}  next state {}  replay frames {}",
                path.display(),
                segment.first,
                segment.last,
                segment.end.name(),
                scoreboard[segment.last].clock_phase.name(),
                scoreboard[segment.last].seconds_remaining,
                after.unwrap_or_else(|| "none".to_owned()),
                frames.len(),
            );
        }
    }
    println!("{kinds:?}");
    ExitCode::SUCCESS
}
