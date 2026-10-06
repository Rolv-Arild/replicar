//! Compare a v2 stage with v1 on every replay, value for value (docs/v2-plan.md, section 5).
//!
//! usage: `parity <stage> <replay or folder>... [--final-assessment]`
//!
//! Stages:
//! - `decode`: v2's decoded network feed against v1's observations (`observations::extract`).
//!
//! Prints one line per replay (`equal`, or the first difference) and a summary; exits non-zero when any
//! replay differs or fails.

use std::error::Error;
use std::path::PathBuf;
use std::process::ExitCode;

use replicar_eval::{collect_replays, first_difference, v1_shape};

/// A stage check: `None` when v2 equals v1 on the replay, else the first difference.
type Check = fn(&[u8]) -> Result<Option<String>, Box<dyn Error>>;

fn decode(bytes: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    let replay = replicar::parse(bytes)?;
    let v1 = replicar_v1::observations::extract(&replay).ok_or("v1 found no network frames")?;
    let v2 = replicar::decode::decode(&replay)?;
    let expected = serde_json::to_value(&v1)?;
    let actual = serde_json::to_value(v1_shape::observed_replay(&v2))?;
    Ok(first_difference(&expected, &actual))
}

fn main() -> ExitCode {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let final_assessment = args.iter().any(|a| a == "--final-assessment");
    args.retain(|a| a != "--final-assessment");
    let usage = "usage: parity <decode> <replay or folder>... [--final-assessment]";
    let (Some(stage), true) = (args.first().cloned(), args.len() >= 2) else {
        eprintln!("{usage}");
        return ExitCode::FAILURE;
    };
    let check: Check = match stage.as_str() {
        "decode" => decode,
        other => {
            eprintln!("unknown stage {other}\n{usage}");
            return ExitCode::FAILURE;
        }
    };
    let roots: Vec<PathBuf> = args[1..].iter().map(PathBuf::from).collect();
    let replays = match collect_replays(&roots, final_assessment) {
        Ok(replays) => replays,
        Err(error) => {
            eprintln!("error: {error}");
            return ExitCode::FAILURE;
        }
    };
    let (mut equal, mut different, mut failed) = (0, 0, 0);
    for path in &replays {
        let outcome = std::fs::read(path)
            .map_err(Into::into)
            .and_then(|bytes| check(&bytes));
        match outcome {
            Ok(None) => {
                equal += 1;
                println!("equal      {}", path.display());
            }
            Ok(Some(difference)) => {
                different += 1;
                println!("DIFFERENT  {}  {difference}", path.display());
            }
            Err(error) => {
                failed += 1;
                println!("FAILED     {}  {error}", path.display());
            }
        }
    }
    println!(
        "{stage}: {equal} equal, {different} different, {failed} failed of {}",
        replays.len()
    );
    if different + failed == 0 {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}
