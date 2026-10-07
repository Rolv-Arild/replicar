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
//! - `simulate_input_fits`: the same with the input fits on in both.
//! - `simulate_all_fits`: every fit on in both except contact alignment.
//! - `aligned_ticks`: v2's contact-aligned update ticks against v1's aligned packet lags (`aligned_lags`).
//! - `convert`: v1's default conversion (every fit and contact alignment) against v2 with the same.
//! - `masked_decode`: the masked evaluation's input (`evaluate_corpus`, default mask schedule) masked by v1 and
//!   by v2, compared as in `decode`.
//! - `held_out`: the evaluator's one-step conversion (`evaluate_corpus` without `--offline-fits`: the flip
//!   cancel held out; the air-control lookahead, the dodge's first-update tick and contact alignment off).
//! - `masked`: the evaluator's masked conversion of the masked input (no update ticks; the withheld frames
//!   refused by every inference; simulated pad pickups and demolitions; no air schedules; the timing fits
//!   against the update after the next).
//! - `recorded`: v2 against itself: the default conversion answered by the fitted inference and recorded,
//!   then answered from the recording (no fits); prints the recording's size per replay to stderr.
//! - `masked_ticks`: the same with update ticks inferred around the withheld frames (`--aligned-targets
//!   --aligned-predictor`).
//!
//! Prints one line per replay (`equal`, or the first difference) and a summary; exits non-zero when any
//! replay differs or fails.

use std::error::Error;
use std::path::PathBuf;
use std::process::ExitCode;

use replicar_eval::mask::{self, MaskSchedule};
use replicar_eval::{collect_replays, first_difference, v1_shape};

/// The evaluator's default mask schedule (no `--mask-seed`).
const MASK: MaskSchedule = MaskSchedule {
    seed: None,
    replay_hash: 0,
};

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

fn masked_decode(bytes: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    let replay = replicar::parse(bytes)?;
    let v1 = replicar_v1::observations::extract(&replay).ok_or("v1 found no network frames")?;
    let v2 = replicar::decode::decode(&replay)?;
    let expected = serde_json::to_value(mask::mask_v1(&v1, MASK))?;
    let actual = serde_json::to_value(v1_shape::observed_replay(&mask::mask(&v2, MASK)))?;
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

fn aligned_ticks(bytes: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    let replay = replicar::parse(bytes)?;
    let observations =
        replicar_v1::observations::extract(&replay).ok_or("v1 found no network frames")?;
    let (v1, _) = replicar_v1::contact_alignment::aligned_lags(
        &observations,
        &replicar_v1::conversion::ConvertOptions::default(),
    )?;
    let network = replicar::decode::decode(&replay)?;
    let ticks =
        replicar::update_ticks::infer(&network, true, replicar::update_ticks::Withheld::default());
    let meshes = replicar::Meshes::load("collision_meshes")?;
    let (v2, _) = replicar::align::align_contacts(
        &network,
        &ticks,
        &meshes,
        replicar::infer::InferenceOptions::default(),
        &replicar::simulate::SimulationOptions::default(),
    )?;
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
    input_fits: bool,
    align_contacts: bool,
    /// The evaluator's held-out one-step options.
    held_out: bool,
    /// The evaluator's masked conversion, with update ticks or without.
    masked: Option<bool>,
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

/// The rung with the input fits.
fn simulate_input_fits(bytes: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    simulate_rung(
        bytes,
        Rung {
            input_fits: true,
            ..Rung::default()
        },
    )
}

/// Every fit except contact alignment.
fn simulate_all_fits(bytes: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    simulate_rung(
        bytes,
        Rung {
            air_lookahead: true,
            air_bvp: true,
            input_fits: true,
            align_contacts: false,
            ..Rung::default()
        },
    )
}

/// The default conversion: every fit and contact alignment.
fn convert(bytes: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    simulate_rung(
        bytes,
        Rung {
            air_lookahead: true,
            air_bvp: true,
            input_fits: true,
            align_contacts: true,
            ..Rung::default()
        },
    )
}

/// The default conversion: every fit and contact alignment.
fn defaults() -> Rung {
    Rung {
        air_lookahead: true,
        air_bvp: true,
        input_fits: true,
        align_contacts: true,
        ..Rung::default()
    }
}

/// The evaluator's held-out one-step conversion.
fn held_out(bytes: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    simulate_rung(
        bytes,
        Rung {
            held_out: true,
            ..defaults()
        },
    )
}

/// The evaluator's masked conversion without update ticks.
fn masked(bytes: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    simulate_rung(
        bytes,
        Rung {
            masked: Some(false),
            ..defaults()
        },
    )
}

/// The evaluator's masked conversion with update ticks.
fn masked_ticks(bytes: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    simulate_rung(
        bytes,
        Rung {
            masked: Some(true),
            ..defaults()
        },
    )
}

fn simulate_rung(bytes: &[u8], rung: Rung) -> Result<Option<String>, Box<dyn Error>> {
    let replay = replicar::parse(bytes)?;
    let mut observations =
        replicar_v1::observations::extract(&replay).ok_or("v1 found no network frames")?;
    let withheld_frames = rung
        .masked
        .map(|_| MASK.withheld(observations.frames.len()));
    let mut options = replicar_v1::conversion::ConvertOptions {
        input_fits: rung.input_fits,
        air_bvp: rung.air_bvp,
        infer_air_controls_from_lookahead: rung.air_lookahead,
        align_contacts: rung.align_contacts,
        ..Default::default()
    };
    if rung.held_out {
        // `evaluate_corpus`'s strict options.
        options.flip_cancel_holdout = true;
        options.infer_air_controls_from_lookahead = false;
        options.infer_dodge_first_packet_tick = false;
        options.align_contacts = false;
    }
    if let Some(update_ticks) = rung.masked {
        // `evaluate_corpus`'s `masked_conversion_options` and `masked_observations`.
        options.infer_packet_lag = update_ticks;
        options.block_sim_pad_pickups = false;
        options.air_bvp = false;
        options.fit_on_next_packet = false;
        options.disable_simulated_demolitions = false;
        options.align_contacts = false;
        options.withheld_frames = withheld_frames.clone().map(std::sync::Arc::new);
        observations = mask::mask_v1(&observations, MASK);
    }
    let mut expected = Vec::new();
    let labels = replicar_v1::labels::ReplayLabels::new(&observations);
    // More slots than any replay has: v1's lists are padded with unknowns, which the comparison drops.
    let mut freshness = replicar_v1::freshness::FreshnessTracker::new(64);
    let summary = replicar_v1::conversion::convert_observations_with(
        &observations,
        &options,
        |frame, _, _| {
            let mut value = v1_shape::simulated_frame_v1(frame);
            value["annotations"] = v1_shape::annotations_v1(frame);
            value["game"] = v1_shape::game_v1(
                frame,
                &freshness.frame(&observations, frame.replay_frame, frame),
                &labels.frame_at(&observations.frames, frame.replay_frame),
            );
            expected.push(value);
            Ok(())
        },
    )?;
    let mut network = replicar::decode::decode(&replay)?;
    if rung.masked.is_some() {
        network = mask::mask(&network, MASK);
    }
    let withheld = replicar::update_ticks::Withheld(withheld_frames.as_deref());
    let held_out = rung.held_out;
    let masked = rung.masked.is_some();
    let inference_options = replicar::infer::InferenceOptions {
        air_lookahead: rung.air_lookahead && !held_out,
        air_schedules: rung.air_bvp && !masked,
        input_fits: rung.input_fits,
        fit_on_next_update: !masked,
        flip_cancel_holdout: held_out,
        dodge_first_update_tick: !held_out,
        ..Default::default()
    };
    let simulation_options = replicar::simulate::SimulationOptions {
        simulated_pad_pickups: masked,
        simulated_demolitions: masked,
        withheld: withheld_frames.clone(),
        ..Default::default()
    };
    let meshes = replicar::Meshes::load("collision_meshes")?;
    let mut ticks = (rung.masked != Some(false))
        .then(|| replicar::update_ticks::infer(&network, true, withheld));
    if let Some(found) = &ticks
        && rung.align_contacts
        && !held_out
        && !masked
    {
        ticks = Some(
            replicar::align::align_contacts(
                &network,
                found,
                &meshes,
                inference_options,
                &simulation_options,
            )?
            .0,
        );
    }
    let mut inference = replicar::infer::FittedInference::new(
        &network,
        ticks.as_ref(),
        inference_options,
        withheld,
    );
    let mut annotator = replicar::annotate::Annotator::new(
        ticks
            .as_ref()
            .map(|ticks| replicar::annotate::ball_intervals(&network.frames, ticks, withheld))
            .unwrap_or_default(),
    );
    let scoreboard = replicar::annotate::scoreboard::reconstruct(&network.frames);
    let segments = replicar::annotate::segments::segment_frames(
        &network.frames,
        &replicar::annotate::segments::segments(&network.frames, &scoreboard),
    );
    let mut decider = replicar::annotate::scoreboard::Decider::default();
    let mut updates = replicar::annotate::updates::UpdateTracker::default();
    let mut actual = Vec::new();
    let simulation = replicar::simulate::simulate(
        &network,
        ticks.as_ref(),
        &mut inference,
        &meshes,
        simulation_options,
        |frame| {
            let mut value = v1_shape::simulated_frame_v2(&frame);
            value["annotations"] = v1_shape::annotations_v2(&annotator.annotate(&frame));
            let f = frame.index.get();
            value["game"] = v1_shape::game_v2(
                &decider.apply(scoreboard[f], &frame.events),
                &updates.frame(&network.frames, &network.frames[f], &frame),
                segments[f].as_ref(),
            );
            actual.push(value);
        },
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

/// The default conversion answered by the fitted inference, then from its recording.
fn recorded(bytes: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    use replicar::infer::recorded::{Choice, RecordedInference, Recorder};
    let replay = replicar::parse(bytes)?;
    let network = replicar::decode::decode(&replay)?;
    let withheld = replicar::update_ticks::Withheld::default();
    let meshes = replicar::Meshes::load("collision_meshes")?;
    let options = replicar::infer::InferenceOptions::default();
    let simulation_options = replicar::simulate::SimulationOptions::default();
    let ticks = replicar::update_ticks::infer(&network, true, withheld);
    let ticks =
        replicar::align::align_contacts(&network, &ticks, &meshes, options, &simulation_options)?.0;
    let run = |inference: &mut dyn replicar::infer::Inference| {
        let mut frames = Vec::new();
        replicar::simulate::simulate(
            &network,
            Some(&ticks),
            inference,
            &meshes,
            simulation_options.clone(),
            |frame| frames.push(v1_shape::simulated_frame_v2(&frame)),
        )
        .map(|_| frames)
    };
    let mut fitted =
        replicar::infer::FittedInference::new(&network, Some(&ticks), options, withheld);
    let mut recorder = Recorder::new(&mut fitted);
    let expected = run(&mut recorder)?;
    let recording = recorder.into_recording();
    let actual = run(&mut RecordedInference::new(&recording))?;
    // The recording's volume: answers by kind, and the entries of the schedules.
    let mut kinds: std::collections::BTreeMap<String, usize> = std::collections::BTreeMap::new();
    let mut entries = 0usize;
    for (_, choice) in recording.choices.values() {
        let kind = format!("{choice:?}");
        let kind = kind.split('(').next().unwrap_or_default().to_owned();
        *kinds.entry(kind).or_default() += 1;
        entries += match choice {
            Choice::AirSchedule(schedule, _) => schedule.entries.len(),
            Choice::GroundSchedule(choice) => choice.schedule.entries.len(),
            _ => 0,
        };
    }
    eprintln!(
        "VOLUME frames {} asked {} choices {} schedule_entries {entries} {kinds:?}",
        network.frames.len(),
        recording.asked.values().sum::<usize>(),
        recording.choices.len(),
    );
    if actual.len() != expected.len() {
        return Ok(Some(format!(
            "frames: recorded {} replayed {}",
            expected.len(),
            actual.len()
        )));
    }
    for (index, (e, a)) in expected.iter().zip(&actual).enumerate() {
        if let Some(difference) = first_difference(e, a) {
            return Ok(Some(format!("frame {index}{difference}")));
        }
    }
    Ok(None)
}

fn main() -> ExitCode {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let final_assessment = args.iter().any(|a| a == "--final-assessment");
    args.retain(|a| a != "--final-assessment");
    let usage = "usage: parity <decode|update_ticks|simulate|simulate_air_lookahead|simulate_air_bvp|simulate_input_fits|simulate_all_fits|aligned_ticks|convert|masked_decode|held_out|masked|masked_ticks|recorded> <replay or folder>... [--final-assessment]";
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
        "simulate_input_fits" => simulate_input_fits,
        "simulate_all_fits" => simulate_all_fits,
        "aligned_ticks" => aligned_ticks,
        "convert" => convert,
        "masked_decode" => masked_decode,
        "held_out" => held_out,
        "masked" => masked,
        "masked_ticks" => masked_ticks,
        "recorded" => recorded,
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
