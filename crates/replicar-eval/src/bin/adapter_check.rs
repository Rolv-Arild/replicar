//! Checks `v2_conversion` (v2 in v1's output types) against v1's conversion with the same options, value for
//! value: every frame (as in the parity stages), the position residuals, the car slots and the counters. The
//! default options, and with `--held-out` the evaluator's one-step options.
//!
//! usage: `adapter_check <replay or folder>... [--held-out] [--final-assessment]`

use std::path::PathBuf;
use std::process::ExitCode;

use replicar_eval::{collect_replays, first_difference, v1_shape};

fn json(output: &replicar_v1::conversion::ConversionOutput) -> serde_json::Value {
    serde_json::json!({
        "frames": output.frames.iter().map(|f| {
            let mut v = v1_shape::simulated_frame_v1(f);
            v["annotations"] = v1_shape::annotations_v1(f);
            v["scoreboard"] = serde_json::to_value(&f.scoreboard).unwrap_or_default();
            v
        }).collect::<Vec<_>>(),
        "residuals": serde_json::to_value(&output.position_residuals).unwrap_or_default(),
        "slots": serde_json::to_value(&output.car_slots).unwrap_or_default(),
        "diagnostics": serde_json::to_value(&output.diagnostics).unwrap_or_default(),
        "observations": serde_json::to_value(&output.observations).unwrap_or_default(),
    })
}

fn main() -> ExitCode {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let final_assessment = args.iter().any(|a| a == "--final-assessment");
    let held_out = args.iter().any(|a| a == "--held-out");
    args.retain(|a| a != "--final-assessment" && a != "--held-out");
    let roots: Vec<PathBuf> = args.iter().map(PathBuf::from).collect();
    let replays = match collect_replays(&roots, final_assessment) {
        Ok(replays) => replays,
        Err(error) => {
            eprintln!("error: {error}");
            return ExitCode::FAILURE;
        }
    };
    let mut options = replicar_v1::conversion::ConvertOptions::default();
    if held_out {
        options.flip_cancel_holdout = true;
        options.infer_air_controls_from_lookahead = false;
        options.infer_dodge_first_packet_tick = false;
        options.align_contacts = false;
    }
    let mut bad = 0;
    for path in &replays {
        let outcome = (|| -> Result<Option<String>, Box<dyn std::error::Error>> {
            let bytes = std::fs::read(path)?;
            let v1 = replicar_v1::conversion::convert_bytes(&bytes, &options)?;
            let v2 = replicar_eval::v2_conversion::convert_bytes(&bytes, &options)?;
            Ok(first_difference(&json(&v1), &json(&v2)))
        })();
        match outcome {
            Ok(None) => println!("equal      {}", path.display()),
            Ok(Some(d)) => {
                bad += 1;
                println!("DIFFERENT  {}  {d}", path.display());
            }
            Err(e) => {
                bad += 1;
                println!("FAILED     {}  {e}", path.display());
            }
        }
    }
    println!("{} of {} equal", replays.len() - bad, replays.len());
    if bad == 0 {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}
