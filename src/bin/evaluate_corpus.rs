//! Measure one-step, pre-correction prediction against replay positions.

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::conversion::{
    CarSlot, ConversionOutput, ConvertOptions, PositionResidual, convert_bytes,
    convert_observations, quaternion, rotation_error_degrees,
};
use replay_to_rocketsim::observations::{Body, ObservedReplay};
use rocketsim::{ArenaEvent, Mat3A, PhysState};
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

#[derive(Default)]
struct FieldSamples {
    simulated: Vec<f32>,
    hold: Vec<f32>,
}

impl FieldSamples {
    fn add(&mut self, simulated: f32, hold: f32) {
        if simulated.is_finite() && hold.is_finite() {
            self.simulated.push(simulated);
            self.hold.push(hold);
        }
    }

    fn extend(&mut self, other: &Self) {
        self.simulated.extend_from_slice(&other.simulated);
        self.hold.extend_from_slice(&other.hold);
    }

    fn summary(&self) -> FieldSummary {
        FieldSummary {
            simulated: quantiles(&self.simulated),
            hold: quantiles(&self.hold),
        }
    }
}

#[derive(Serialize)]
struct FieldSummary {
    simulated: Quantiles,
    hold: Quantiles,
}

#[derive(Default)]
struct KinematicSamples {
    linear_velocity_uu_per_second: FieldSamples,
    rotation_degrees: FieldSamples,
    angular_velocity_radians_per_second: FieldSamples,
}

impl KinematicSamples {
    fn extend(&mut self, other: &Self) {
        self.linear_velocity_uu_per_second
            .extend(&other.linear_velocity_uu_per_second);
        self.rotation_degrees.extend(&other.rotation_degrees);
        self.angular_velocity_radians_per_second
            .extend(&other.angular_velocity_radians_per_second);
    }

    fn summary(&self) -> KinematicSummary {
        KinematicSummary {
            linear_velocity_uu_per_second: self.linear_velocity_uu_per_second.summary(),
            rotation_degrees: self.rotation_degrees.summary(),
            angular_velocity_radians_per_second: self.angular_velocity_radians_per_second.summary(),
        }
    }
}

#[derive(Serialize)]
struct KinematicSummary {
    linear_velocity_uu_per_second: FieldSummary,
    rotation_degrees: FieldSummary,
    angular_velocity_radians_per_second: FieldSummary,
}

#[derive(Default)]
struct KinematicsByBody {
    ball: KinematicSamples,
    car: KinematicSamples,
}

impl KinematicsByBody {
    fn add_residual(&mut self, residual: &PositionResidual) {
        let samples = if residual.actor_id.is_none() {
            &mut self.ball
        } else {
            &mut self.car
        };
        if let (Some(sim), Some(hold)) = (
            residual.simulated_velocity_error_uu_per_sec,
            residual.hold_velocity_error_uu_per_sec,
        ) {
            samples.linear_velocity_uu_per_second.add(sim, hold);
        }
        if let (Some(sim), Some(hold)) = (
            residual.simulated_rotation_error_degrees,
            residual.hold_rotation_error_degrees,
        ) {
            samples.rotation_degrees.add(sim, hold);
        }
        if let (Some(sim), Some(hold)) = (
            residual.simulated_angular_velocity_error_rad_per_sec,
            residual.hold_angular_velocity_error_rad_per_sec,
        ) {
            samples.angular_velocity_radians_per_second.add(sim, hold);
        }
    }

    fn extend(&mut self, other: &Self) {
        self.ball.extend(&other.ball);
        self.car.extend(&other.car);
    }

    fn summary(&self) -> KinematicsByBodySummary {
        KinematicsByBodySummary {
            ball: self.ball.summary(),
            car: self.car.summary(),
        }
    }
}

#[derive(Serialize)]
struct KinematicsByBodySummary {
    ball: KinematicSummary,
    car: KinematicSummary,
}

#[derive(Serialize)]
struct ReplayReport {
    path: String,
    sha256: String,
    frames: usize,
    arena_ticks: u64,
    skipped_timeline_ticks: u64,
    unlinked_car_frames: usize,
    shadowed_car_frames: usize,
    active_pawn_demo_corrections: usize,
    default_hitbox_players: usize,
    car_slots: Vec<CarSlot>,
    position_uu: BodySummary,
    kinematics: KinematicsByBodySummary,
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
    mask_seed: Option<u64>,
    boxcars_version: &'static str,
    rocketsim_revision: &'static str,
    options: ConvertOptions,
    replays: Vec<ReplayReport>,
    failures: Vec<Failure>,
    by_game_size: BTreeMap<String, BodySummary>,
    all: BodySummary,
    one_step_kinematics_by_game_size: BTreeMap<String, KinematicsByBodySummary>,
    one_step_kinematics_all: KinematicsByBodySummary,
    one_step_car_angular_by_altitude: BTreeMap<String, FieldSummary>,
    masked_position_uu_by_horizon_frames: BTreeMap<usize, BodySummary>,
    masked_by_game_size: BTreeMap<String, BTreeMap<usize, BodySummary>>,
    masked_kinematics_by_horizon_frames: BTreeMap<usize, KinematicsByBodySummary>,
    masked_kinematics_by_game_size: BTreeMap<String, BTreeMap<usize, KinematicsByBodySummary>>,
    masked_boost_by_horizon_frames: BTreeMap<usize, FieldSummary>,
    masked_boost_by_game_size: BTreeMap<String, BTreeMap<usize, FieldSummary>>,
    masked_car_angular_by_altitude: BTreeMap<String, FieldSummary>,
    worst_car_regret_uu: Vec<OutlierRecord>,
}

#[derive(Serialize)]
struct OutlierRecord {
    path: String,
    frame: usize,
    replay_time: f32,
    actor_id: i32,
    player_key: Option<String>,
    team: Option<u8>,
    actor_created_frame: usize,
    player_link_active: bool,
    observed_position: [f32; 3],
    previous_observed_position: Option<[f32; 3]>,
    observed_velocity: Option<[f32; 3]>,
    previous_sim_position: Option<[f32; 3]>,
    previous_sim_velocity: Option<[f32; 3]>,
    previous_sim_on_ground: Option<bool>,
    previous_sim_demoed: Option<bool>,
    boost_active_raw: Option<u8>,
    jump_active_raw: Option<u8>,
    double_jump_active_raw: Option<u8>,
    dodge_active_raw: Option<u8>,
    event_kinds: Vec<&'static str>,
    gap_seconds: f32,
    simulated_error_uu: f32,
    linear_error_uu: f32,
    regret_uu: f32,
}

fn outlier_record(
    path: &Path,
    conversion: &ConversionOutput,
    residual: &PositionResidual,
) -> Option<OutlierRecord> {
    let actor_id = residual.actor_id?;
    let linear_error = residual.linear_extrapolation_error_uu?;
    if !linear_error.is_finite() || !residual.simulated_error_uu.is_finite() {
        return None;
    }
    let observed_frame = conversion.observations.frames.get(residual.frame)?;
    let car = observed_frame
        .cars
        .iter()
        .find(|car| car.actor_id == actor_id)?;
    let observed_position = car.body.position.as_ref()?.value;
    let slot = car.player_key.as_ref().and_then(|key| {
        conversion
            .car_slots
            .iter()
            .find(|slot| &slot.player_key == key)
            .map(|slot| slot.slot)
    });
    let previous_car = conversion
        .frames
        .get(residual.frame.checked_sub(1)?)
        .and_then(|frame| {
            frame
                .state
                .cars
                .iter()
                .find(|(info, _)| Some(info.idx) == slot)
                .map(|(_, state)| state)
        });
    let previous_observed_position = conversion
        .observations
        .frames
        .get(residual.frame.checked_sub(1)?)
        .and_then(|frame| frame.cars.iter().find(|car| car.actor_id == actor_id))
        .and_then(|car| car.body.position.as_ref().map(|value| value.value));
    let event_kinds = conversion.frames[residual.frame]
        .simulated_events
        .iter()
        .map(|event| match event.event {
            ArenaEvent::BallHitWorld(_) => "ball_hit_world",
            ArenaEvent::CarHitBall(_) => "car_hit_ball",
            ArenaEvent::CarHitCar(_) => "car_hit_car",
            ArenaEvent::CarHitWorld(_) => "car_hit_world",
            ArenaEvent::CarPickupBoost(_) => "car_pickup_boost",
        })
        .collect();
    Some(OutlierRecord {
        path: path.display().to_string(),
        frame: residual.frame,
        replay_time: observed_frame.time,
        actor_id,
        player_key: car.player_key.clone(),
        team: car.team,
        actor_created_frame: car.actor_created_frame,
        player_link_active: car.player_link_active,
        observed_position,
        previous_observed_position,
        observed_velocity: car.body.linear_velocity.as_ref().map(|v| v.value),
        previous_sim_position: previous_car.map(|state| state.phys.pos.to_array()),
        previous_sim_velocity: previous_car.map(|state| state.phys.vel.to_array()),
        previous_sim_on_ground: previous_car.map(|state| state.is_on_ground),
        previous_sim_demoed: previous_car.map(|state| state.is_demoed),
        boost_active_raw: car.inputs.boost_active_raw.as_ref().map(|v| v.value),
        jump_active_raw: car.inputs.jump_active_raw.as_ref().map(|v| v.value),
        double_jump_active_raw: car.inputs.double_jump_active_raw.as_ref().map(|v| v.value),
        dodge_active_raw: car.inputs.dodge_active_raw.as_ref().map(|v| v.value),
        event_kinds,
        gap_seconds: residual.seconds_since_previous_position,
        simulated_error_uu: residual.simulated_error_uu,
        linear_error_uu: linear_error,
        regret_uu: residual.simulated_error_uu - linear_error,
    })
}

fn distance(a: [f32; 3], b: [f32; 3]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn valid_masked_interval(
    index: usize,
    previous_frame: usize,
    frames: &[replay_to_rocketsim::observations::Frame],
) -> bool {
    if previous_frame >= index {
        return false;
    }
    let active = |frame: usize| {
        frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active")
    };
    let dt = frames[index].time - frames[previous_frame].time;
    active(index) && active(previous_frame) && dt.is_finite() && dt > 0.0 && dt <= 0.5
}

fn add_masked_kinematics(
    samples: &mut KinematicSamples,
    actual: &Body,
    stale: &Body,
    index: usize,
    predicted: &PhysState,
    frames: &[replay_to_rocketsim::observations::Frame],
) {
    if let (Some(actual), Some(previous)) = (&actual.linear_velocity, &stale.linear_velocity) {
        if actual.frame == index && valid_masked_interval(index, previous.frame, frames) {
            samples.linear_velocity_uu_per_second.add(
                distance(predicted.vel.to_array(), actual.value),
                distance(previous.value, actual.value),
            );
        }
    }
    if let (Some(actual), Some(previous)) = (&actual.rotation_xyzw, &stale.rotation_xyzw) {
        if actual.frame == index && valid_masked_interval(index, previous.frame, frames) {
            if let (Some(actual), Some(previous)) =
                (quaternion(actual.value), quaternion(previous.value))
            {
                let actual = Mat3A::from_quat(actual);
                samples.rotation_degrees.add(
                    rotation_error_degrees(predicted.rot_mat, actual),
                    rotation_error_degrees(Mat3A::from_quat(previous), actual),
                );
            }
        }
    }
    if let (Some(actual), Some(previous)) = (
        &actual.angular_velocity_replay_units,
        &stale.angular_velocity_replay_units,
    ) {
        if actual.frame == index && valid_masked_interval(index, previous.frame, frames) {
            let actual = actual.value.map(|axis| axis * 0.01);
            let previous = previous.value.map(|axis| axis * 0.01);
            samples.angular_velocity_radians_per_second.add(
                distance(predicted.ang_vel.to_array(), actual),
                distance(previous, actual),
            );
        }
    }
}

#[derive(Clone, Copy)]
struct MaskSchedule {
    seed: Option<u64>,
    replay_hash: u64,
}

impl MaskSchedule {
    fn horizon(self, index: usize) -> Option<usize> {
        let offset = index % 100;
        let start = if let Some(seed) = self.seed {
            let mut value = seed ^ self.replay_hash ^ (index / 100) as u64;
            value = value.wrapping_add(0x9e3779b97f4a7c15);
            value = (value ^ (value >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
            value = (value ^ (value >> 27)).wrapping_mul(0x94d049bb133111eb);
            value ^= value >> 31;
            1 + (value % 95) as usize
        } else {
            1
        };
        (start..start + 4)
            .contains(&offset)
            .then(|| offset - start + 1)
    }
}

fn masked_observations(original: &ObservedReplay, schedule: MaskSchedule) -> ObservedReplay {
    let mut masked = original.clone();
    for index in 1..masked.frames.len() {
        if schedule.horizon(index).is_none() {
            continue;
        }
        let previous = masked.frames[index - 1].clone();
        let frame = &mut masked.frames[index];
        frame.pad_pickups.clear();
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
                car.boost = prior.boost.clone();
                car.boost_raw = prior.boost_raw.clone();
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
    if position.frame != index || !valid_masked_interval(index, previous.frame, frames) {
        return;
    }
    let dt = frames[index].time - frames[previous.frame].time;
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
        simulated_velocity_error_uu_per_sec: None,
        hold_velocity_error_uu_per_sec: None,
        simulated_rotation_error_degrees: None,
        hold_rotation_error_degrees: None,
        simulated_angular_velocity_error_rad_per_sec: None,
        hold_angular_velocity_error_rad_per_sec: None,
        altitude_z: Some(position.value[2]),
        is_on_ground: None,
    });
}

fn masked_metrics(
    original: &ObservedReplay,
    conversion: &ConversionOutput,
    schedule: MaskSchedule,
    result: &mut BTreeMap<usize, ByBody>,
    kinematics: &mut BTreeMap<usize, KinematicsByBody>,
    boost: &mut BTreeMap<usize, FieldSamples>,
    car_angular_by_altitude: &mut BTreeMap<String, FieldSamples>,
) {
    let slots: BTreeMap<_, _> = conversion
        .car_slots
        .iter()
        .map(|slot| (slot.player_key.as_str(), slot.slot))
        .collect();
    for index in 1..original.frames.len() {
        let Some(horizon) = schedule.horizon(index) else {
            continue;
        };
        let original_frame = &original.frames[index];
        let masked_frame = &conversion.observations.frames[index];
        let state = &conversion.frames[index].state;
        let by_body = result.entry(horizon).or_default();
        let by_kinematics = kinematics.entry(horizon).or_default();
        let by_boost = boost.entry(horizon).or_default();
        if let (Some(actual), Some(stale)) = (&original_frame.ball, &masked_frame.ball) {
            add_masked_error(
                &mut by_body.ball,
                actual,
                stale,
                index,
                state.ball.phys.pos.to_array(),
                &original.frames,
            );
            add_masked_kinematics(
                &mut by_kinematics.ball,
                actual,
                stale,
                index,
                &state.ball.phys,
                &original.frames,
            );
        }
        for car in replay_to_rocketsim::observations::primary_linked_cars(original_frame) {
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
            add_masked_kinematics(
                &mut by_kinematics.car,
                &car.body,
                &stale.body,
                index,
                &predicted.phys,
                &original.frames,
            );
            if let (Some(actual), Some(previous)) = (&car.boost, &stale.boost) {
                if actual.frame == index
                    && valid_masked_interval(index, previous.frame, &original.frames)
                {
                    by_boost.add(
                        (predicted.boost - actual.value).abs(),
                        (previous.value - actual.value).abs(),
                    );
                }
            }
            if let (Some(position), Some(actual), Some(previous)) = (
                &car.body.position,
                &car.body.angular_velocity_replay_units,
                &stale.body.angular_velocity_replay_units,
            ) {
                if position.frame == index
                    && actual.frame == index
                    && valid_masked_interval(index, previous.frame, &original.frames)
                {
                    let altitude = if position.value[2] < 50.0 {
                        "ground"
                    } else if position.value[2] > 100.0 {
                        "air"
                    } else {
                        "transition"
                    };
                    car_angular_by_altitude
                        .entry(altitude.to_owned())
                        .or_default()
                        .add(
                            distance(
                                predicted.phys.ang_vel.to_array(),
                                actual.value.map(|axis| axis * 0.01),
                            ),
                            distance(
                                previous.value.map(|axis| axis * 0.01),
                                actual.value.map(|axis| axis * 0.01),
                            ),
                        );
                }
            }
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
    let mut mask_seed = None;
    while let Some(arg) = args.next() {
        if arg == "--no-inferred-boost" {
            options.infer_boost_from_active = false;
        } else if arg == "--inferred-jump" {
            options.infer_jump_from_active = true;
            options.gate_jump_on_observed_impulse = false;
        } else if arg == "--gated-jump" {
            options.infer_jump_from_active = true;
            options.gate_jump_on_observed_impulse = true;
        } else if arg == "--no-inferred-jump" {
            options.infer_jump_from_active = false;
            options.gate_jump_on_observed_impulse = false;
        } else if arg == "--inferred-dodge" {
            options.infer_dodge_from_active = true;
            options.gate_dodge_on_observed_impulse = false;
        } else if arg == "--gated-dodge" {
            options.infer_dodge_from_active = true;
            options.gate_dodge_on_observed_impulse = true;
        } else if arg == "--no-inferred-dodge" {
            options.infer_dodge_from_active = false;
            options.gate_dodge_on_observed_impulse = false;
        } else if arg == "--no-sync-pads" {
            options.sync_boost_pad_pickups = false;
        } else if arg == "--sync-pads" {
            options.sync_boost_pad_pickups = true;
        } else if arg == "--no-infer-air-steer" {
            options.infer_air_steer_controls = false;
        } else if arg == "--infer-air-steer" {
            options.infer_air_steer_controls = true;
        } else if arg == "--no-infer-air-lookahead" {
            options.infer_air_controls_from_lookahead = false;
        } else if arg == "--infer-air-lookahead" {
            options.infer_air_controls_from_lookahead = true;
        } else if arg == "--octane-hitbox" {
            options.use_loadout_hitboxes = false;
        } else if arg == "--mask-seed" {
            mask_seed = Some(
                args.next()
                    .ok_or("--mask-seed requires a u64 value")?
                    .to_string_lossy()
                    .parse::<u64>()?,
            );
        } else if meshes.is_none() {
            meshes = Some(PathBuf::from(arg));
        } else {
            return Err("usage: evaluate_corpus <split_dir> <report.json> [collision_meshes] [--no-inferred-boost] [--no-inferred-jump] [--inferred-jump] [--gated-jump] [--no-inferred-dodge] [--inferred-dodge] [--gated-dodge] [--no-sync-pads] [--sync-pads] [--no-infer-air-steer] [--infer-air-steer] [--no-infer-air-lookahead] [--infer-air-lookahead] [--octane-hitbox] [--mask-seed u64]".into());
        }
    }
    if let Some(meshes) = meshes {
        options.collision_meshes = meshes;
    }
    let replay_paths = paths(&root)?;
    let mut groups: BTreeMap<String, ByBody> = BTreeMap::new();
    let mut all = ByBody::default();
    let mut one_step_kinematics_groups: BTreeMap<String, KinematicsByBody> = BTreeMap::new();
    let mut one_step_kinematics_all = KinematicsByBody::default();
    let mut one_step_car_angular_by_altitude: BTreeMap<String, FieldSamples> = BTreeMap::new();
    let mut report = Report {
        schema_version: 1,
        split_directory: root.display().to_string(),
        metric: "pre-correction position error (UU) on fresh replay positions after an active simulation interval; quantiles pool samples within each group",
        masked_metric: "every 100-frame block masks four consecutive ball/car body and car boost frames; default start offset 1 or replay-hash/seed-derived offset when mask_seed is set; compare uncorrected output with fresh original fields in Active phase and a <=0.5 second field-specific gap; hold baseline uses the last unmasked value",
        mask_seed,
        boxcars_version: "0.11.5",
        rocketsim_revision: replay_to_rocketsim::serialization::ROCKETSIM_REVISION,
        options: options.clone(),
        replays: Vec::new(),
        failures: Vec::new(),
        by_game_size: BTreeMap::new(),
        all: all.summary(),
        one_step_kinematics_by_game_size: BTreeMap::new(),
        one_step_kinematics_all: one_step_kinematics_all.summary(),
        one_step_car_angular_by_altitude: BTreeMap::new(),
        masked_position_uu_by_horizon_frames: BTreeMap::new(),
        masked_by_game_size: BTreeMap::new(),
        masked_kinematics_by_horizon_frames: BTreeMap::new(),
        masked_kinematics_by_game_size: BTreeMap::new(),
        masked_boost_by_horizon_frames: BTreeMap::new(),
        masked_boost_by_game_size: BTreeMap::new(),
        masked_car_angular_by_altitude: BTreeMap::new(),
        worst_car_regret_uu: Vec::new(),
    };
    let mut masked_by_horizon: BTreeMap<usize, ByBody> = BTreeMap::new();
    let mut masked_by_size: BTreeMap<String, BTreeMap<usize, ByBody>> = BTreeMap::new();
    let mut kinematics_by_horizon: BTreeMap<usize, KinematicsByBody> = BTreeMap::new();
    let mut kinematics_by_size: BTreeMap<String, BTreeMap<usize, KinematicsByBody>> =
        BTreeMap::new();
    let mut boost_by_horizon: BTreeMap<usize, FieldSamples> = BTreeMap::new();
    let mut boost_by_size: BTreeMap<String, BTreeMap<usize, FieldSamples>> = BTreeMap::new();
    let mut car_angular_by_altitude: BTreeMap<String, FieldSamples> = BTreeMap::new();
    for (index, (size, path)) in replay_paths.iter().enumerate() {
        match fs::read(path)
            .map_err(|error| error.to_string())
            .and_then(|bytes| convert_bytes(&bytes, &options).map_err(|error| error.to_string()))
        {
            Ok(conversion) => {
                let mut own = ByBody::default();
                let mut own_kinematics = KinematicsByBody::default();
                for residual in &conversion.position_residuals {
                    own.add(residual);
                    own_kinematics.add_residual(residual);
                    groups.entry(size.clone()).or_default().add(residual);
                    one_step_kinematics_groups
                        .entry(size.clone())
                        .or_default()
                        .add_residual(residual);
                    all.add(residual);
                    one_step_kinematics_all.add_residual(residual);

                    if residual.actor_id.is_some() {
                        if let (Some(alt), Some(sim_ang), Some(hold_ang)) = (
                            residual.altitude_z,
                            residual.simulated_angular_velocity_error_rad_per_sec,
                            residual.hold_angular_velocity_error_rad_per_sec,
                        ) {
                            let altitude = if alt < 50.0 {
                                "ground"
                            } else if alt > 100.0 {
                                "air"
                            } else {
                                "transition"
                            };
                            one_step_car_angular_by_altitude
                                .entry(altitude.to_owned())
                                .or_default()
                                .add(sim_ang, hold_ang);
                        }
                    }

                    let regret = residual
                        .linear_extrapolation_error_uu
                        .map(|linear| residual.simulated_error_uu - linear);
                    let keep = residual.actor_id.is_some()
                        && regret.is_some_and(f32::is_finite)
                        && (report.worst_car_regret_uu.len() < 100
                            || report
                                .worst_car_regret_uu
                                .last()
                                .is_some_and(|last| regret.unwrap() > last.regret_uu));
                    if keep {
                        if let Some(outlier) = outlier_record(path, &conversion, residual) {
                            report.worst_car_regret_uu.push(outlier);
                            report
                                .worst_car_regret_uu
                                .sort_by(|a, b| b.regret_uu.total_cmp(&a.regret_uu));
                            report.worst_car_regret_uu.truncate(100);
                        }
                    }
                }
                let replay_hash = u64::from_str_radix(
                    conversion
                        .source_sha256
                        .as_deref()
                        .and_then(|hash| hash.get(..16))
                        .ok_or("missing replay SHA-256")?,
                    16,
                )?;
                let schedule = MaskSchedule {
                    seed: mask_seed,
                    replay_hash,
                };
                let masked = masked_observations(&conversion.observations, schedule);
                match convert_observations(masked, &options) {
                    Ok(masked_conversion) => {
                        let mut own_masked = BTreeMap::new();
                        let mut own_masked_kinematics = BTreeMap::new();
                        let mut own_boost = BTreeMap::new();
                        let mut own_angular_by_altitude = BTreeMap::new();
                        masked_metrics(
                            &conversion.observations,
                            &masked_conversion,
                            schedule,
                            &mut own_masked,
                            &mut own_masked_kinematics,
                            &mut own_boost,
                            &mut own_angular_by_altitude,
                        );
                        for (altitude, samples) in own_angular_by_altitude {
                            car_angular_by_altitude
                                .entry(altitude)
                                .or_default()
                                .extend(&samples);
                        }
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
                        for (horizon, samples) in own_masked_kinematics {
                            kinematics_by_horizon
                                .entry(horizon)
                                .or_default()
                                .extend(&samples);
                            kinematics_by_size
                                .entry(size.clone())
                                .or_default()
                                .entry(horizon)
                                .or_default()
                                .extend(&samples);
                        }
                        for (horizon, samples) in own_boost {
                            boost_by_horizon
                                .entry(horizon)
                                .or_default()
                                .extend(&samples);
                            boost_by_size
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
                    shadowed_car_frames: conversion.diagnostics.shadowed_car_frames,
                    active_pawn_demo_corrections: conversion
                        .diagnostics
                        .active_pawn_demo_corrections,
                    default_hitbox_players: conversion.diagnostics.default_hitbox_players,
                    car_slots: conversion.car_slots.clone(),
                    position_uu: own.summary(),
                    kinematics: own_kinematics.summary(),
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
    report.one_step_kinematics_by_game_size = one_step_kinematics_groups
        .into_iter()
        .map(|(size, samples)| (size, samples.summary()))
        .collect();
    report.one_step_kinematics_all = one_step_kinematics_all.summary();
    report.one_step_car_angular_by_altitude = one_step_car_angular_by_altitude
        .into_iter()
        .map(|(alt, samples)| (alt, samples.summary()))
        .collect();
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
    report.masked_kinematics_by_horizon_frames = kinematics_by_horizon
        .into_iter()
        .map(|(horizon, samples)| (horizon, samples.summary()))
        .collect();
    report.masked_kinematics_by_game_size = kinematics_by_size
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
    report.masked_boost_by_horizon_frames = boost_by_horizon
        .into_iter()
        .map(|(horizon, samples)| (horizon, samples.summary()))
        .collect();
    report.masked_boost_by_game_size = boost_by_size
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
    report.masked_car_angular_by_altitude = car_angular_by_altitude
        .into_iter()
        .map(|(altitude, samples)| (altitude, samples.summary()))
        .collect();
    fs::write(&output_path, serde_json::to_vec_pretty(&report)?)?;
    println!(
        "{} successes, {} failures -> {}",
        report.replays.len(),
        report.failures.len(),
        output_path.display()
    );
    if let (Some(sim_ang), Some(hold_ang)) = (
        report
            .one_step_kinematics_all
            .car
            .angular_velocity_radians_per_second
            .simulated
            .p50,
        report
            .one_step_kinematics_all
            .car
            .angular_velocity_radians_per_second
            .hold
            .p50,
    ) {
        println!("1-step car ang_vel rad/s: sim p50={sim_ang:.4}, hold p50={hold_ang:.4}");
    }
    if let (Some(sim_rot), Some(hold_rot)) = (
        report
            .one_step_kinematics_all
            .car
            .rotation_degrees
            .simulated
            .p50,
        report.one_step_kinematics_all.car.rotation_degrees.hold.p50,
    ) {
        println!("1-step car rotation deg: sim p50={sim_rot:.2}, hold p50={hold_rot:.2}");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Quat;

    #[test]
    fn rotation_error_handles_quaternion_sign_and_ninety_degree_turn() {
        let identity = Mat3A::IDENTITY;
        let negated_identity = quaternion([0.0, 0.0, 0.0, -1.0]).unwrap();
        assert!(rotation_error_degrees(identity, Mat3A::from_quat(negated_identity)) < 0.01);
        let quarter_turn = Mat3A::from_quat(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2));
        assert!((rotation_error_degrees(identity, quarter_turn) - 90.0).abs() < 0.01);
        assert!(quaternion([0.0; 4]).is_none());
    }

    #[test]
    fn mask_schedule_selects_four_consecutive_frames_per_block() {
        for seed in [None, Some(42)] {
            let schedule = MaskSchedule {
                seed,
                replay_hash: 0x1234,
            };
            for block in 0..20 {
                let selected: Vec<_> = (block * 100..(block + 1) * 100)
                    .filter_map(|index| schedule.horizon(index).map(|h| (index, h)))
                    .collect();
                assert_eq!(selected.len(), 4);
                assert_eq!(
                    selected.iter().map(|(_, h)| *h).collect::<Vec<_>>(),
                    [1, 2, 3, 4]
                );
                assert!(selected[0].0 % 100 >= 1);
                assert!(selected[3].0 % 100 <= 98);
            }
        }
    }

    #[test]
    fn masked_observations_withhold_car_boost_while_preserving_activation() {
        use replay_to_rocketsim::observations::{
            Car, Frame, Header, Inputs, ObservedReplay, Source, Value,
        };

        let schedule = MaskSchedule {
            seed: None,
            replay_hash: 0,
        };
        let frames = (0..6)
            .map(|index| Frame {
                index,
                time: index as f32 * 0.033,
                delta: 0.033,
                ball: None,
                cars: vec![Car {
                    actor_id: 1,
                    actor_created_frame: 0,
                    player_key: Some("player1".to_string()),
                    player_link_active: true,
                    team: Some(0),
                    body_product_id: None,
                    body: replay_to_rocketsim::observations::Body::default(),
                    boost: Some(Value {
                        value: 50.0 + index as f32,
                        frame: index,
                        source: Source::Replay,
                    }),
                    boost_raw: Some(Value {
                        value: (100 + index) as u8,
                        frame: index,
                        source: Source::Replay,
                    }),
                    inputs: Inputs {
                        boost_active_raw: Some(Value {
                            value: (index % 2) as u8,
                            frame: index,
                            source: Source::Replay,
                        }),
                        ..Inputs::default()
                    },
                }],
                players: Vec::new(),
                team_scores: [None, None],
                seconds_remaining: None,
                overtime: None,
                game_state: Some(Value {
                    value: "Active".to_string(),
                    frame: index,
                    source: Source::Replay,
                }),
                events: Vec::new(),
                pad_pickups: Vec::new(),
            })
            .collect();
        let original = ObservedReplay {
            header: Header {
                game_type: "Soccar".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames,
            diagnostics: Default::default(),
        };
        let masked = masked_observations(&original, schedule);

        assert_eq!(masked.frames[0].cars[0].boost.as_ref().unwrap().frame, 0);
        assert_eq!(masked.frames[0].cars[0].boost.as_ref().unwrap().value, 50.0);
        assert_eq!(
            masked.frames[0].cars[0]
                .inputs
                .boost_active_raw
                .as_ref()
                .unwrap()
                .frame,
            0
        );

        for h in 1..=4 {
            let car = &masked.frames[h].cars[0];
            assert_eq!(
                car.boost.as_ref().unwrap().frame,
                0,
                "horizon {h} boost frame should be 0"
            );
            assert_eq!(
                car.boost.as_ref().unwrap().value,
                50.0,
                "horizon {h} boost value should be 50.0"
            );
            assert_eq!(car.boost_raw.as_ref().unwrap().frame, 0);
            assert_eq!(car.boost_raw.as_ref().unwrap().value, 100);
            assert_eq!(
                car.inputs.boost_active_raw.as_ref().unwrap().frame,
                h,
                "horizon {h} activation frame"
            );
            assert_eq!(
                car.inputs.boost_active_raw.as_ref().unwrap().value,
                (h % 2) as u8
            );
        }

        assert_eq!(masked.frames[5].cars[0].boost.as_ref().unwrap().frame, 5);
        assert_eq!(masked.frames[5].cars[0].boost.as_ref().unwrap().value, 55.0);
        assert_eq!(
            masked.frames[5].cars[0]
                .inputs
                .boost_active_raw
                .as_ref()
                .unwrap()
                .frame,
            5
        );
    }
}
