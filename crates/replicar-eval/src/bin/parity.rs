//! Compare a v2 stage with v1 on every replay, value for value (docs/v2-plan.md, section 5).
//!
//! usage: `parity <stage> <replay or folder>... [--final-assessment]`
//!
//! Stages:
//! - `decode`: v2's decoded network feed against v1's observations (`observations::extract`).
//! - `update_ticks`: v2's update-tick inference against v1's packet lags (`infer_packet_lags`, default options).
//! - `simulate`: v2's simulation against v1's conversion with every fit off (`input_fits`, `air_bvp`,
//!   `infer_air_controls_from_lookahead`, `align_contacts`): each frame's state, events, applied ticks and holds.
//! - `simulate_air_lookahead`: the same with the air-control lookahead on in both.
//! - `simulate_air_bvp`: the same with the air boundary-value solve on in both.
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

fn update_ticks(bytes: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    let replay = replicar::parse(bytes)?;
    let observations =
        replicar_v1::observations::extract(&replay).ok_or("v1 found no network frames")?;
    let v1 = replicar_v1::conversion::infer_packet_lags(
        &observations,
        &replicar_v1::conversion::ConvertOptions::default(),
    );
    let network = replicar::decode::decode(&replay)?;
    let v2 =
        replicar::update_ticks::infer(&network, true, replicar::update_ticks::Withheld::default());
    Ok(first_difference(
        &v1_shape::canonical_lags_v1(&v1),
        &v1_shape::canonical_lags_v2(&v2),
    ))
}

/// A rung of the ladder (docs/v2-plan.md, section 5): the fits it switches on, in v1 and v2; the others
/// are off.
#[derive(Clone, Copy, Default)]
struct Rung {
    air_lookahead: bool,
    air_bvp: bool,
}

/// The rung without fits: the simulation alone.
fn simulate(bytes: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    simulate_rung(bytes, Rung::default())
}

/// The rung with the air-control lookahead.
fn simulate_air_lookahead(bytes: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    simulate_rung(
        bytes,
        Rung {
            air_lookahead: true,
            ..Rung::default()
        },
    )
}

/// The rung with the air boundary-value solve.
fn simulate_air_bvp(bytes: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    simulate_rung(
        bytes,
        Rung {
            air_bvp: true,
            ..Rung::default()
        },
    )
}

fn simulate_rung(bytes: &[u8], rung: Rung) -> Result<Option<String>, Box<dyn Error>> {
    let replay = replicar::parse(bytes)?;
    let observations =
        replicar_v1::observations::extract(&replay).ok_or("v1 found no network frames")?;
    let options = replicar_v1::conversion::ConvertOptions {
        input_fits: false,
        air_bvp: rung.air_bvp,
        infer_air_controls_from_lookahead: rung.air_lookahead,
        align_contacts: false,
        ..Default::default()
    };
    let mut expected = Vec::new();
    let summary = replicar_v1::conversion::convert_observations_with(
        &observations,
        &options,
        |frame, _, _| {
            expected.push(v1_shape::simulated_frame_v1(frame));
            Ok(())
        },
    )?;
    let network = replicar::decode::decode(&replay)?;
    let withheld = replicar::update_ticks::Withheld::default();
    let ticks = replicar::update_ticks::infer(&network, true, withheld);
    let meshes = replicar::Meshes::load("collision_meshes")?;
    let mut inference = replicar::infer::FittedInference::new(
        &network,
        Some(&ticks),
        replicar::infer::InferenceOptions {
            air_lookahead: rung.air_lookahead,
            air_schedules: rung.air_bvp,
        },
        withheld,
    );
    let mut actual = Vec::new();
    let simulation = replicar::simulate::simulate(
        &network,
        Some(&ticks),
        &mut inference,
        &meshes,
        replicar::simulate::SimulationOptions::default(),
        |frame| actual.push(v1_shape::simulated_frame_v2(&frame)),
    )?;
    if actual.len() != expected.len() {
        return Ok(Some(format!(
            "frames: v1 {} v2 {}",
            expected.len(),
            actual.len()
        )));
    }
    for (index, (e, a)) in expected.iter().zip(&actual).enumerate() {
        if let Some(difference) = first_difference(e, a) {
            return Ok(Some(format!("frame {index}{difference}")));
        }
    }
    Ok(first_difference(
        &v1_shape::simulation_v1(&summary),
        &v1_shape::simulation_v2(&simulation, &inference.diagnostics),
    ))
}

fn main() -> ExitCode {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let final_assessment = args.iter().any(|a| a == "--final-assessment");
    args.retain(|a| a != "--final-assessment");
    let usage = "usage: parity <decode|update_ticks|simulate|simulate_air_lookahead|simulate_air_bvp> <replay or folder>... [--final-assessment]";
    let (Some(stage), true) = (args.first().cloned(), args.len() >= 2) else {
        eprintln!("{usage}");
        return ExitCode::FAILURE;
    };
    let check: Check = match stage.as_str() {
        "decode" => decode,
        "update_ticks" => update_ticks,
        "simulate" => simulate,
        "simulate_air_lookahead" => simulate_air_lookahead,
        "simulate_air_bvp" => simulate_air_bvp,
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
