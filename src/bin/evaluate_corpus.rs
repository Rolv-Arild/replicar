//! Measure one-step, pre-correction prediction against replay positions.

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::conversion::{
    ConversionOutput, ConvertOptions, PositionResidual, convert_bytes, convert_observations,
};
use replay_to_rocketsim::observations::{Body, ObservedReplay};
use serde::Serialize;

#[derive(Default)]
struct Samples {
    simulated: Vec<f32>,
    hold: Vec<f32>,
    linear: Vec<f32>,
}

impl Samples {
    fn add(&mut self, residual: &PositionResidual) {
        if residual.simulated_error_uu.is_finite() && residual.hold_error_uu.is_finite() {
            self.simulated.push(residual.simulated_error_uu);
            self.hold.push(residual.hold_error_uu);
        }
        if let Some(value) = residual
            .linear_extrapolation_error_uu
            .filter(|v| v.is_finite())
        {
            self.linear.push(value);
        }
    }

    fn summary(&self) -> ErrorSummary {
        ErrorSummary {
            simulated: quantiles(&self.simulated),
            hold: quantiles(&self.hold),
            linear: quantiles(&self.linear),
        }
    }

    fn extend(&mut self, other: &Self) {
        self.simulated.extend_from_slice(&other.simulated);
        self.hold.extend_from_slice(&other.hold);
        self.linear.extend_from_slice(&other.linear);
    }
}

#[derive(Serialize)]
struct Quantiles {
    count: usize,
    p50: Option<f32>,
    p90: Option<f32>,
    p99: Option<f32>,
}

fn quantiles(values: &[f32]) -> Quantiles {
    let mut values: Vec<_> = values.iter().copied().filter(|v| v.is_finite()).collect();
    values.sort_by(f32::total_cmp);
    let at = |fraction: f64| {
        (!values.is_empty()).then(|| {
            let index = ((values.len() - 1) as f64 * fraction).round() as usize;
            values[index]
        })
    };
    Quantiles {
        count: values.len(),
        p50: at(0.5),
        p90: at(0.9),
        p99: at(0.99),
    }
}

#[derive(Serialize)]
struct ErrorSummary {
    simulated: Quantiles,
    hold: Quantiles,
    linear: Quantiles,
}

#[derive(Default)]
struct ByBody {
    ball: Samples,
    car: Samples,
}

impl ByBody {
    fn add(&mut self, residual: &PositionResidual) {
        let samples = if residual.actor_id.is_none() {
            &mut self.ball
        } else {
            &mut self.car
        };
        samples.add(residual);
    }

    fn summary(&self) -> BodySummary {
        BodySummary {
            ball: self.ball.summary(),
            car: self.car.summary(),
        }
    }

    fn extend(&mut self, other: &Self) {
        self.ball.extend(&other.ball);
        self.car.extend(&other.car);
    }
}

#[derive(Serialize)]
struct BodySummary {
    ball: ErrorSummary,
    car: ErrorSummary,
}

#[derive(Serialize)]
struct ReplayReport {
    path: String,
    sha256: String,
    frames: usize,
    arena_ticks: u64,
    skipped_timeline_ticks: u64,
    unlinked_car_frames: usize,
    position_uu: BodySummary,
}

#[derive(Serialize)]
struct Failure {
    path: String,
    error: String,
}

#[derive(Serialize)]
struct Report {
    schema_version: u32,
    split_directory: String,
    metric: &'static str,
    masked_metric: &'static str,
    boxcars_version: &'static str,
    rocketsim_revision: &'static str,
    options: ConvertOptions,
    replays: Vec<ReplayReport>,
    failures: Vec<Failure>,
    by_game_size: BTreeMap<String, BodySummary>,
    all: BodySummary,
    masked_position_uu_by_horizon_frames: BTreeMap<usize, BodySummary>,
    masked_by_game_size: BTreeMap<String, BTreeMap<usize, BodySummary>>,
}

fn distance(a: [f32; 3], b: [f32; 3]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn masked_observations(original: &ObservedReplay) -> ObservedReplay {
    let mut masked = original.clone();
    for index in 1..masked.frames.len() {
        if !(1..=4).contains(&(index % 100)) {
            continue;
        }
        let previous = masked.frames[index - 1].clone();
        let frame = &mut masked.frames[index];
        if let Some(previous_ball) = previous.ball {
            frame.ball = Some(previous_ball);
        }
        for car in &mut frame.cars {
            if let Some(prior) = previous
                .cars
                .iter()
                .find(|prior| prior.actor_id == car.actor_id)
            {
                car.body = prior.body.clone();
            }
        }
    }
    masked
}

fn add_masked_error(
    samples: &mut Samples,
    actual: &Body,
    stale: &Body,
    index: usize,
    predicted: [f32; 3],
    frames: &[replay_to_rocketsim::observations::Frame],
) {
    let (Some(position), Some(previous)) = (&actual.position, &stale.position) else {
        return;
    };
    if position.frame != index || previous.frame >= index {
        return;
    }
    let active = |frame: usize| {
        frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active")
    };
    if !active(index) || !active(previous.frame) {
        return;
    }
    let dt = frames[index].time - frames[previous.frame].time;
    if !dt.is_finite() || dt <= 0.0 || dt > 0.5 {
        return;
    }
    let linear = stale.linear_velocity.as_ref().map(|velocity| {
        let extrapolated =
            std::array::from_fn(|axis| previous.value[axis] + velocity.value[axis] * dt);
        distance(extrapolated, position.value)
    });
    samples.add(&PositionResidual {
        frame: index,
        actor_id: None,
        seconds_since_previous_position: dt,
        simulated_error_uu: distance(predicted, position.value),
        hold_error_uu: distance(previous.value, position.value),
        linear_extrapolation_error_uu: linear,
    });
}

fn masked_metrics(
    original: &ObservedReplay,
    conversion: &ConversionOutput,
    result: &mut BTreeMap<usize, ByBody>,
) {
    let slots: BTreeMap<_, _> = conversion
        .car_slots
        .iter()
        .map(|slot| (slot.player_key.as_str(), slot.slot))
        .collect();
    for index in 1..original.frames.len() {
        let horizon = index % 100;
        if !(1..=4).contains(&horizon) {
            continue;
        }
        let original_frame = &original.frames[index];
        let masked_frame = &conversion.observations.frames[index];
        let state = &conversion.frames[index].state;
        let by_body = result.entry(horizon).or_default();
        if let (Some(actual), Some(stale)) = (&original_frame.ball, &masked_frame.ball) {
            add_masked_error(
                &mut by_body.ball,
                actual,
                stale,
                index,
                state.ball.phys.pos.to_array(),
                &original.frames,
            );
        }
        for car in &original_frame.cars {
            let Some(stale) = masked_frame
                .cars
                .iter()
                .find(|c| c.actor_id == car.actor_id)
            else {
                continue;
            };
            let Some(slot) = car.player_key.as_deref().and_then(|key| slots.get(key)) else {
                continue;
            };
            let Some((_, predicted)) = state.cars.iter().find(|(info, _)| info.idx == *slot) else {
                continue;
            };
            add_masked_error(
                &mut by_body.car,
                &car.body,
                &stale.body,
                index,
                predicted.phys.pos.to_array(),
                &original.frames,
            );
        }
    }
}

fn paths(root: &Path) -> Result<Vec<(String, PathBuf)>, Box<dyn Error>> {
    let mut result = Vec::new();
    for size in ["1v1", "2v2", "3v3"] {
        for entry in fs::read_dir(root.join(size))? {
            let path = entry?.path();
            if path.extension().is_some_and(|ext| ext == "replay") {
                result.push((size.to_owned(), path));
            }
        }
    }
    result.sort_by(|a, b| a.1.cmp(&b.1));
    Ok(result)
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args_os().skip(1);
    let root = PathBuf::from(
        args.next()
            .ok_or("usage: evaluate_corpus <split_dir> <report.json> [collision_meshes]")?,
    );
    let output_path = PathBuf::from(
        args.next()
            .ok_or("usage: evaluate_corpus <split_dir> <report.json> [collision_meshes]")?,
    );
    let mut options = ConvertOptions::default();
    let mut meshes = None;
    for arg in args {
        if arg == "--no-inferred-boost" {
            options.infer_boost_from_active = false;
        } else if meshes.is_none() {
            meshes = Some(PathBuf::from(arg));
        } else {
            return Err("usage: evaluate_corpus <split_dir> <report.json> [collision_meshes] [--no-inferred-boost]".into());
        }
    }
    if let Some(meshes) = meshes {
        options.collision_meshes = meshes;
    }
    let replay_paths = paths(&root)?;
    let mut groups: BTreeMap<String, ByBody> = BTreeMap::new();
    let mut all = ByBody::default();
    let mut report = Report {
        schema_version: 1,
        split_directory: root.display().to_string(),
        metric: "pre-correction position error (UU) on fresh replay positions after an active simulation interval; quantiles pool samples within each group",
        masked_metric: "every 100-frame block masks ball/car body fields at offsets 1 through 4; compare uncorrected output with fresh original positions in Active phase and a <=0.5 second gap",
        boxcars_version: "0.11.5",
        rocketsim_revision: replay_to_rocketsim::serialization::ROCKETSIM_REVISION,
        options: options.clone(),
        replays: Vec::new(),
        failures: Vec::new(),
        by_game_size: BTreeMap::new(),
        all: all.summary(),
        masked_position_uu_by_horizon_frames: BTreeMap::new(),
        masked_by_game_size: BTreeMap::new(),
    };
    let mut masked_by_horizon: BTreeMap<usize, ByBody> = BTreeMap::new();
    let mut masked_by_size: BTreeMap<String, BTreeMap<usize, ByBody>> = BTreeMap::new();
    for (index, (size, path)) in replay_paths.iter().enumerate() {
        match fs::read(path)
            .map_err(|error| error.to_string())
            .and_then(|bytes| convert_bytes(&bytes, &options).map_err(|error| error.to_string()))
        {
            Ok(conversion) => {
                let mut own = ByBody::default();
                for residual in &conversion.position_residuals {
                    own.add(residual);
                    groups.entry(size.clone()).or_default().add(residual);
                    all.add(residual);
                }
                let masked = masked_observations(&conversion.observations);
                match convert_observations(masked, &options) {
                    Ok(masked_conversion) => {
                        let mut own_masked = BTreeMap::new();
                        masked_metrics(
                            &conversion.observations,
                            &masked_conversion,
                            &mut own_masked,
                        );
                        for (horizon, samples) in own_masked {
                            masked_by_horizon
                                .entry(horizon)
                                .or_default()
                                .extend(&samples);
                            masked_by_size
                                .entry(size.clone())
                                .or_default()
                                .entry(horizon)
                                .or_default()
                                .extend(&samples);
                        }
                    }
                    Err(error) => report.failures.push(Failure {
                        path: path.display().to_string(),
                        error: format!("masked conversion: {error}"),
                    }),
                }
                report.replays.push(ReplayReport {
                    path: path.display().to_string(),
                    sha256: conversion.source_sha256.unwrap_or_default(),
                    frames: conversion.frames.len(),
                    arena_ticks: conversion.frames.last().map_or(0, |f| f.state.tick_count),
                    skipped_timeline_ticks: conversion.diagnostics.skipped_timeline_ticks,
                    unlinked_car_frames: conversion.diagnostics.unlinked_car_frames,
                    position_uu: own.summary(),
                });
            }
            Err(error) => report.failures.push(Failure {
                path: path.display().to_string(),
                error,
            }),
        }
        eprintln!("{}/{} {}", index + 1, replay_paths.len(), path.display());
    }
    report.by_game_size = groups
        .into_iter()
        .map(|(size, samples)| (size, samples.summary()))
        .collect();
    report.all = all.summary();
    report.masked_position_uu_by_horizon_frames = masked_by_horizon
        .into_iter()
        .map(|(horizon, samples)| (horizon, samples.summary()))
        .collect();
    report.masked_by_game_size = masked_by_size
        .into_iter()
        .map(|(size, by_horizon)| {
            (
                size,
                by_horizon
                    .into_iter()
                    .map(|(horizon, samples)| (horizon, samples.summary()))
                    .collect(),
            )
        })
        .collect();
    fs::write(&output_path, serde_json::to_vec_pretty(&report)?)?;
    println!(
        "{} successes, {} failures -> {}",
        report.replays.len(),
        report.failures.len(),
        output_path.display()
    );
    Ok(())
}
