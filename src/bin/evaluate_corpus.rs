//! Measure one-step, pre-correction prediction against replay positions.

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use replay_to_rocketsim::conversion::{
    CarSlot, ConversionOutput, ConvertOptions, PositionResidual, convert_bytes,
    convert_observations, quaternion, rotation_error_degrees,
};
use replay_to_rocketsim::observations::{Body, Inputs, ObservedReplay, PadPickup};
use rocketsim::{ArenaEvent, CarControls, CarState, Mat3A, PhysState};
use serde::Serialize;

#[derive(Default)]
struct Samples {
    simulated: Vec<f32>,
    hold: Vec<f32>,
    linear: Vec<f32>,
    offline_projection_fit: Vec<f32>,
    /// Seconds from the packet the prediction starts from to the target frame, parallel to
    /// `simulated` and `hold`, and parallel to `linear` (`linear_age`).
    age: Vec<f32>,
    linear_age: Vec<f32>,
}

/// Age buckets (seconds from the start packet to the target frame) for the masked rows.
const AGE_BUCKETS: [(&str, f32); 4] = [
    ("<=0.04", 0.04),
    ("<=0.08", 0.08),
    ("<=0.12", 0.12),
    ("<=0.20", 0.20),
];

fn age_bucket(age: f32) -> &'static str {
    AGE_BUCKETS
        .iter()
        .find(|(_, limit)| age <= *limit)
        .map_or(">0.20", |(name, _)| name)
}

impl Samples {
    fn add(&mut self, residual: &PositionResidual) {
        if residual.simulated_error_uu.is_finite() && residual.hold_error_uu.is_finite() {
            self.simulated.push(residual.simulated_error_uu);
            self.hold.push(residual.hold_error_uu);
            self.age.push(residual.seconds_since_previous_position);
        }
        if let Some(value) = residual
            .linear_extrapolation_error_uu
            .filter(|v| v.is_finite())
        {
            self.linear.push(value);
            self.linear_age
                .push(residual.seconds_since_previous_position);
        }
        if let Some(value) = residual
            .offline_projection_fit_error_uu
            .filter(|v| v.is_finite())
        {
            self.offline_projection_fit.push(value);
        }
    }

    fn summary(&self) -> ErrorSummary {
        ErrorSummary {
            simulated: quantiles(&self.simulated),
            hold: quantiles(&self.hold),
            linear: quantiles(&self.linear),
            offline_projection_fit: (!self.offline_projection_fit.is_empty())
                .then(|| quantiles(&self.offline_projection_fit)),
            start_packet_age_seconds: None,
        }
    }

    /// `summary` plus the age quantiles of the start packet (masked rows).
    fn summary_with_age(&self) -> ErrorSummary {
        ErrorSummary {
            start_packet_age_seconds: Some(age_quantiles(&self.age)),
            ..self.summary()
        }
    }

    /// The samples split by the age of their start packet (`age_bucket`).
    fn by_age_bucket(&self) -> BTreeMap<&'static str, Samples> {
        let mut result: BTreeMap<&'static str, Samples> = BTreeMap::new();
        for ((&simulated, &hold), &age) in self.simulated.iter().zip(&self.hold).zip(&self.age) {
            let bucket = result.entry(age_bucket(age)).or_default();
            bucket.simulated.push(simulated);
            bucket.hold.push(hold);
            bucket.age.push(age);
        }
        for (&linear, &age) in self.linear.iter().zip(&self.linear_age) {
            let bucket = result.entry(age_bucket(age)).or_default();
            bucket.linear.push(linear);
            bucket.linear_age.push(age);
        }
        result
    }

    fn extend(&mut self, other: &Self) {
        self.simulated.extend_from_slice(&other.simulated);
        self.hold.extend_from_slice(&other.hold);
        self.linear.extend_from_slice(&other.linear);
        self.offline_projection_fit
            .extend_from_slice(&other.offline_projection_fit);
        self.age.extend_from_slice(&other.age);
        self.linear_age.extend_from_slice(&other.linear_age);
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

/// Quantiles of the seconds from the packet a masked prediction starts from to its target frame
/// (the horizon label counts frames from the mask window's start; a car's last fresh packet is
/// often older than the frame before the window).
#[derive(Serialize)]
struct AgeQuantiles {
    count: usize,
    p10: Option<f32>,
    p50: Option<f32>,
    p90: Option<f32>,
}

fn age_quantiles(values: &[f32]) -> AgeQuantiles {
    let mut values: Vec<_> = values.iter().copied().filter(|v| v.is_finite()).collect();
    values.sort_by(f32::total_cmp);
    let at = |fraction: f64| {
        (!values.is_empty())
            .then(|| values[((values.len() - 1) as f64 * fraction).round() as usize])
    };
    AgeQuantiles {
        count: values.len(),
        p10: at(0.1),
        p50: at(0.5),
        p90: at(0.9),
    }
}

#[derive(Serialize)]
struct ErrorSummary {
    simulated: Quantiles,
    hold: Quantiles,
    linear: Quantiles,
    #[serde(skip_serializing_if = "Option::is_none")]
    offline_projection_fit: Option<Quantiles>,
    /// Masked rows only: the age of the start packet (`seconds_since_previous_position`).
    #[serde(skip_serializing_if = "Option::is_none")]
    start_packet_age_seconds: Option<AgeQuantiles>,
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

    fn summary_with_age(&self) -> BodySummary {
        BodySummary {
            ball: self.ball.summary_with_age(),
            car: self.car.summary_with_age(),
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

#[derive(Serialize)]
struct TraceControls {
    throttle: f32,
    steer: f32,
    pitch: f32,
    yaw: f32,
    roll: f32,
    jump: bool,
    boost: bool,
    handbrake: bool,
}

impl From<CarControls> for TraceControls {
    fn from(value: CarControls) -> Self {
        Self {
            throttle: value.throttle,
            steer: value.steer,
            pitch: value.pitch,
            yaw: value.yaw,
            roll: value.roll,
            jump: value.jump,
            boost: value.boost,
            handbrake: value.handbrake,
        }
    }
}

#[derive(Serialize)]
struct TraceSimCar {
    position: [f32; 3],
    rotation_basis: [[f32; 3]; 3],
    linear_velocity: [f32; 3],
    angular_velocity: [f32; 3],
    is_on_ground: bool,
    wheel_contact_count: usize,
    world_contact: bool,
    is_jumping: bool,
    is_flipping: bool,
    controls_for_next_interval: TraceControls,
}

#[derive(Serialize)]
struct PriorAngularPacket {
    frame: usize,
    replay_time: f32,
    angular_velocity_radians_per_second: [f32; 3],
}

fn prior_angular_packets(
    original: &ObservedReplay,
    car: &replay_to_rocketsim::observations::Car,
    before_frame: usize,
) -> Vec<PriorAngularPacket> {
    let mut packets = Vec::new();
    for source in (car.actor_created_frame..before_frame).rev() {
        let frame = &original.frames[source];
        if let Some(value) = frame
            .cars
            .iter()
            .find(|candidate| {
                candidate.actor_id == car.actor_id
                    && candidate.actor_created_frame == car.actor_created_frame
            })
            .and_then(|candidate| candidate.body.angular_velocity_replay_units.as_ref())
            .filter(|value| value.frame == source)
        {
            packets.push(PriorAngularPacket {
                frame: source,
                replay_time: frame.time,
                angular_velocity_radians_per_second: value.value.map(|axis| axis * 0.01),
            });
            if packets.len() == 3 {
                break;
            }
        }
    }
    packets
}

impl From<&CarState> for TraceSimCar {
    fn from(state: &CarState) -> Self {
        Self {
            position: state.phys.pos.to_array(),
            rotation_basis: [
                state.phys.rot_mat.x_axis.to_array(),
                state.phys.rot_mat.y_axis.to_array(),
                state.phys.rot_mat.z_axis.to_array(),
            ],
            linear_velocity: state.phys.vel.to_array(),
            angular_velocity: state.phys.ang_vel.to_array(),
            is_on_ground: state.is_on_ground,
            wheel_contact_count: state
                .wheels_with_contact
                .iter()
                .filter(|v| v.is_some())
                .count(),
            world_contact: state.world_contact_normal.is_some(),
            is_jumping: state.is_jumping,
            is_flipping: state.is_flipping,
            controls_for_next_interval: state.controls.into(),
        }
    }
}

#[derive(Serialize)]
struct MaskedRotationTrace {
    replay_path: String,
    replay_sha256: String,
    frame: usize,
    replay_time: f32,
    replay_delta: f32,
    horizon: usize,
    timeline_tick: u64,
    arena_tick: u64,
    actor_id: i32,
    actor_created_frame: usize,
    masked_actor_lifetime_match: bool,
    player_key: String,
    team: Option<u8>,
    slot: usize,
    rotation_source_frame: Option<usize>,
    rotation_source_gap_seconds: Option<f32>,
    observed_body: Body,
    masked_body: Body,
    observed_inputs_at_target: Inputs,
    observed_inputs_before_interval: Option<Inputs>,
    observed_pad_pickups_at_target: Vec<PadPickup>,
    prior_angular_packets_before_mask: Vec<PriorAngularPacket>,
    previous_simulated: Option<TraceSimCar>,
    predicted: TraceSimCar,
    simulated_event_kinds_in_interval: Vec<&'static str>,
    rotation_error_degrees: Option<f32>,
    hold_rotation_error_degrees: Option<f32>,
    angular_error_radians_per_second: Option<f32>,
    position_error_uu: Option<f32>,
}

fn car_event_kinds(
    events: &[replay_to_rocketsim::conversion::SimEvent],
    slot: usize,
) -> Vec<&'static str> {
    events
        .iter()
        .filter_map(|event| match event.event {
            ArenaEvent::CarHitWorld(value) if value.car_idx == slot => Some("sim_car_hit_world"),
            ArenaEvent::CarHitBall(value) if value.car_idx == slot => Some("sim_car_hit_ball"),
            ArenaEvent::CarHitCar(value)
                if value.bumper_car_idx == slot || value.victim_car_idx == slot =>
            {
                Some("sim_car_hit_car")
            }
            ArenaEvent::CarPickupBoost(value) if value.car_idx == slot => {
                Some("sim_car_pickup_boost")
            }
            _ => None,
        })
        .collect()
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
    ball_lag_frames: usize,
    car_lag_frames: usize,
    active_pawn_demo_corrections: usize,
    default_hitbox_players: usize,
    car_slots: Vec<CarSlot>,
    position_uu: BodySummary,
    kinematics: KinematicsByBodySummary,
    masked_kinematics_by_horizon_frames: BTreeMap<usize, KinematicsByBodySummary>,
    /// The masked position errors of this replay alone, by horizon (for per-replay comparisons).
    masked_position_uu_by_horizon_frames: BTreeMap<usize, BodySummary>,
}

/// The options a masked conversion overrides (see `masked_conversion_options`), as applied.
#[derive(Serialize)]
struct MaskedRunOptions {
    /// Packet lags are inferred only by the aligned lag-inferring predictor.
    infer_packet_lag: bool,
    block_sim_pad_pickups: bool,
    air_bvp: bool,
    fit_on_next_packet: bool,
    disable_simulated_demolitions: bool,
    align_contacts: bool,
}

/// The options of the masked conversions: `options` with everything that would use a packet after a
/// withheld frame, or a withheld frame's own data, switched off. `lag_inference` is whether this variant
/// infers packet lags (the aligned lag-inferring predictor and `--infer-packet-lag`).
fn masked_conversion_options(
    options: &ConvertOptions,
    lag_inference: bool,
    withheld: Vec<bool>,
) -> ConvertOptions {
    let mut masked_options = options.clone();
    // A withheld target's own packet lag is unknowable, so masked prediction keeps
    // every state at its frame time; packet-lag inference is an offline improvement
    // measured by the one-step residuals instead.
    masked_options.infer_packet_lag = lag_inference && options.infer_packet_lag;
    // A withheld span has no later boost update to correct a simulated pickup with, so the
    // causal prediction keeps the simulated pad pickups.
    masked_options.block_sim_pad_pickups = false;
    masked_options.air_bvp = false;
    masked_options.fit_on_next_packet = false;
    // The causal prediction has no demolition report for a withheld frame, so RocketSim's
    // own demolition rule stays on; the contact alignment uses later ball packets.
    masked_options.disable_simulated_demolitions = false;
    masked_options.align_contacts = false;
    masked_options.withheld_frames = Some(std::sync::Arc::new(withheld));
    masked_options
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
    /// How the one-step residuals were produced: `held-out` (the flip cancel, air control lookahead, dodge
    /// first-packet tick and contact alignment, which would each use the packet being scored, are off) or
    /// `offline-fits` (in-sample for rotation and angular velocity).
    one_step_fits: &'static str,
    /// What the masked conversions change from `options` (they are causal predictions: no later packet,
    /// control or replay-wide estimate may reach a withheld frame). The values are the ones applied.
    masked_run_options: MaskedRunOptions,
    /// For `--aligned-targets`: `lag-inferring predictor` or `raw predictor` (the default masked predictor),
    /// and that the targets come from the full offline conversion; empty otherwise.
    aligned_variant: &'static str,
    replays: Vec<ReplayReport>,
    failures: Vec<Failure>,
    by_game_size: BTreeMap<String, BodySummary>,
    all: BodySummary,
    one_step_kinematics_by_game_size: BTreeMap<String, KinematicsByBodySummary>,
    one_step_kinematics_all: KinematicsByBodySummary,
    one_step_car_angular_by_altitude: BTreeMap<String, FieldSummary>,
    one_step_transition_angular_by_context: BTreeMap<String, FieldSummary>,
    masked_position_uu_by_horizon_frames: BTreeMap<usize, BodySummary>,
    masked_by_game_size: BTreeMap<String, BTreeMap<usize, BodySummary>>,
    /// All masked position samples pooled over horizons, split by the age of the start packet
    /// (`<=0.04`, `<=0.08`, `<=0.12`, `<=0.20`, `>0.20` seconds), so car and ball compare at equal age.
    masked_position_uu_by_start_packet_age_bucket: BTreeMap<String, BodySummary>,
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
            ArenaEvent::CarLanded(_) => "car_landed",
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

/// Independent diagnostic labels for fresh car packets in the 50–100 UU band.
/// The action labels describe replay component packets, not verified inputs.
fn transition_contexts(
    conversion: &ConversionOutput,
    residual: &PositionResidual,
) -> Option<[&'static str; 3]> {
    let actor_id = residual.actor_id?;
    let frame = conversion.observations.frames.get(residual.frame)?;
    let car = frame.cars.iter().find(|car| car.actor_id == actor_id)?;
    let previous = conversion
        .observations
        .frames
        .get(residual.frame.checked_sub(1)?)?
        .cars
        .iter()
        .find(|previous| {
            previous.actor_id == actor_id && previous.actor_created_frame == car.actor_created_frame
        })?;
    let previous_z = previous.body.position.as_ref()?.value[2];
    let origin = if previous_z < 50.0 {
        "origin_ground"
    } else if previous_z > 100.0 {
        "origin_air"
    } else {
        "origin_transition"
    };
    let ground = if residual.is_on_ground == Some(true) {
        "sim_ground"
    } else {
        "sim_air"
    };
    let mut event = "no_recent_jump_or_dodge";
    for earlier in (0..=residual.frame).rev() {
        let candidate = &conversion.observations.frames[earlier];
        if frame.time - candidate.time > 0.15 {
            break;
        }
        let Some(candidate_car) = candidate.cars.iter().find(|candidate_car| {
            candidate_car.actor_id == actor_id
                && candidate_car.actor_created_frame == car.actor_created_frame
        }) else {
            continue;
        };
        let fresh_odd = |raw: &Option<replay_to_rocketsim::observations::Value<u8>>| {
            raw.as_ref()
                .is_some_and(|value| value.frame == earlier && value.value % 2 == 1)
        };
        if fresh_odd(&candidate_car.inputs.dodge_active_raw) {
            event = "recent_dodge_packet";
            break;
        }
        if fresh_odd(&candidate_car.inputs.double_jump_active_raw) {
            event = "recent_double_jump_packet";
            break;
        }
        if fresh_odd(&candidate_car.inputs.jump_active_raw) {
            event = "recent_jump_packet";
            break;
        }
    }
    Some([origin, ground, event])
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
    if let (Some(actual), Some(previous)) = (&actual.linear_velocity, &stale.linear_velocity)
        && actual.frame == index
        && valid_masked_interval(index, previous.frame, frames)
    {
        samples.linear_velocity_uu_per_second.add(
            distance(predicted.vel.to_array(), actual.value),
            distance(previous.value, actual.value),
        );
    }
    if let (Some(actual), Some(previous)) = (&actual.rotation_xyzw, &stale.rotation_xyzw)
        && actual.frame == index
        && valid_masked_interval(index, previous.frame, frames)
        && let (Some(actual), Some(previous)) =
            (quaternion(actual.value), quaternion(previous.value))
    {
        let actual = Mat3A::from_quat(actual);
        samples.rotation_degrees.add(
            rotation_error_degrees(predicted.rot_mat, actual),
            rotation_error_degrees(Mat3A::from_quat(previous), actual),
        );
    }
    if let (Some(actual), Some(previous)) = (
        &actual.angular_velocity_replay_units,
        &stale.angular_velocity_replay_units,
    ) && actual.frame == index
        && valid_masked_interval(index, previous.frame, frames)
    {
        let actual = actual.value.map(|axis| axis * 0.01);
        let previous = previous.value.map(|axis| axis * 0.01);
        samples.angular_velocity_radians_per_second.add(
            distance(predicted.ang_vel.to_array(), actual),
            distance(previous, actual),
        );
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
            // The same car lifetime: an actor id reused in consecutive frames is another car.
            if let Some(prior) = previous.cars.iter().find(|prior| {
                prior.actor_id == car.actor_id
                    && prior.actor_created_frame == car.actor_created_frame
            }) {
                car.body = prior.body.clone();
                car.boost = prior.boost.clone();
                car.boost_raw = prior.boost_raw.clone();
            }
        }
    }
    masked
}

/// Time in seconds from the stale packet's tick (its frame's timeline tick minus the lag the masked
/// conversion inferred for it) to the target frame's tick. The linear baseline extrapolates over this
/// instead of the raw frame-time difference, so that it gets exactly the timing knowledge of the
/// simulated prediction (the masked conversion's own lags, inferred without the withheld frames), not
/// the offline lags the aligned target is built from. Without inferred lags it is the raw difference.
fn lag_corrected_dt(
    output: &ConversionOutput,
    actor: Option<i32>,
    stale_frame: usize,
    index: usize,
) -> Option<f32> {
    let lag = output
        .frames
        .get(stale_frame)?
        .packet_lags
        .iter()
        .find(|lag| lag.actor_id == actor)?
        .ticks;
    let stale_tick = output.frames[stale_frame].timeline_tick as f64 - lag as f64;
    let target_tick = output.frames.get(index)?.timeline_tick as f64;
    Some(((target_tick - stale_tick) / 120.0) as f32)
}

#[allow(clippy::too_many_arguments)]
fn add_masked_error(
    samples: &mut Samples,
    actual: &Body,
    stale: &Body,
    index: usize,
    predicted: [f32; 3],
    frames: &[replay_to_rocketsim::observations::Frame],
    // The masked conversion and the actor (None: the ball), for the lag-corrected baseline.
    timing: Option<(&ConversionOutput, Option<i32>)>,
) {
    let (Some(position), Some(previous)) = (&actual.position, &stale.position) else {
        return;
    };
    if position.frame != index || !valid_masked_interval(index, previous.frame, frames) {
        return;
    }
    let dt = frames[index].time - frames[previous.frame].time;
    let linear_dt = timing
        .and_then(|(output, actor)| lag_corrected_dt(output, actor, previous.frame, index))
        .unwrap_or(dt);
    let linear = stale.linear_velocity.as_ref().map(|velocity| {
        let extrapolated =
            std::array::from_fn(|axis| previous.value[axis] + velocity.value[axis] * linear_dt);
        distance(extrapolated, position.value)
    });
    samples.add(&PositionResidual {
        frame: index,
        actor_id: None,
        seconds_since_previous_position: dt,
        simulated_error_uu: distance(predicted, position.value),
        simulated_error_vector_uu: [
            predicted[0] - position.value[0],
            predicted[1] - position.value[1],
            predicted[2] - position.value[2],
        ],
        previous_linear_velocity_uu_per_second: None,
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
        offline_interval: None,
        offline_projection_fit_error_uu: None,
    });
}

/// A copy of a packet body whose fresh fields hold the offline reconstruction at the frame time:
/// the packet advanced by its inferred lag. It removes each packet's unknowable timing offset
/// (uniform over about one frame period) from a masked comparison, leaving the model error.
fn aligned_body(packet: &Body, aligned: &PhysState, index: usize) -> Body {
    let mut body = packet.clone();
    if let Some(value) = body.position.as_mut().filter(|v| v.frame == index) {
        value.value = aligned.pos.to_array();
    }
    if let Some(value) = body.linear_velocity.as_mut().filter(|v| v.frame == index) {
        value.value = aligned.vel.to_array();
    }
    if let Some(value) = body.rotation_xyzw.as_mut().filter(|v| v.frame == index) {
        value.value = glam::Quat::from_mat3a(&aligned.rot_mat).to_array();
    }
    if let Some(value) = body
        .angular_velocity_replay_units
        .as_mut()
        .filter(|v| v.frame == index)
    {
        value.value = (aligned.ang_vel * 100.0).to_array();
    }
    body
}

fn masked_metrics(
    original: &ObservedReplay,
    conversion: &ConversionOutput,
    schedule: MaskSchedule,
    replay_path: &Path,
    replay_sha256: &str,
    mut rotation_traces: Option<&mut Vec<MaskedRotationTrace>>,
    result: &mut BTreeMap<usize, ByBody>,
    kinematics: &mut BTreeMap<usize, KinematicsByBody>,
    boost: &mut BTreeMap<usize, FieldSamples>,
    car_angular_by_altitude: &mut BTreeMap<String, FieldSamples>,
    aligned: Option<&ConversionOutput>,
) {
    let aligned_slots: BTreeMap<_, _> = aligned
        .map(|output| {
            output
                .car_slots
                .iter()
                .map(|slot| (slot.player_key.as_str(), slot.slot))
                .collect()
        })
        .unwrap_or_default();
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
            let aligned_ball;
            let actual = if let Some(output) = aligned {
                aligned_ball = aligned_body(actual, &output.frames[index].state.ball.phys, index);
                &aligned_ball
            } else {
                actual
            };
            add_masked_error(
                &mut by_body.ball,
                actual,
                stale,
                index,
                state.ball.phys.pos.to_array(),
                &original.frames,
                aligned.map(|_| (conversion, None)),
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
            let aligned_car_body;
            let car_target: &Body = match (aligned, car.player_key.as_deref()) {
                (Some(output), Some(key)) => {
                    match aligned_slots.get(key).and_then(|slot| {
                        output.frames[index]
                            .state
                            .cars
                            .iter()
                            .find(|(info, _)| info.idx == *slot)
                    }) {
                        Some((_, truth)) => {
                            aligned_car_body = aligned_body(&car.body, &truth.phys, index);
                            &aligned_car_body
                        }
                        None => &car.body,
                    }
                }
                _ => &car.body,
            };
            add_masked_error(
                &mut by_body.car,
                car_target,
                &stale.body,
                index,
                predicted.phys.pos.to_array(),
                &original.frames,
                aligned.map(|_| (conversion, Some(car.actor_id))),
            );
            add_masked_kinematics(
                &mut by_kinematics.car,
                car_target,
                &stale.body,
                index,
                &predicted.phys,
                &original.frames,
            );
            if let Some(traces) = rotation_traces.as_deref_mut() {
                let rotation_errors = match (&car.body.rotation_xyzw, &stale.body.rotation_xyzw) {
                    (Some(actual), Some(previous))
                        if actual.frame == index
                            && valid_masked_interval(index, previous.frame, &original.frames) =>
                    {
                        quaternion(actual.value)
                            .zip(quaternion(previous.value))
                            .map(|(actual_q, previous_q)| {
                                let actual_mat = Mat3A::from_quat(actual_q);
                                (
                                    rotation_error_degrees(predicted.phys.rot_mat, actual_mat),
                                    rotation_error_degrees(
                                        Mat3A::from_quat(previous_q),
                                        actual_mat,
                                    ),
                                )
                            })
                    }
                    _ => None,
                };
                let previous_simulated = conversion.frames[index - 1]
                    .state
                    .cars
                    .iter()
                    .find(|(info, _)| info.idx == *slot)
                    .map(|(_, state)| TraceSimCar::from(state));
                let previous_car = original.frames[index - 1].cars.iter().find(|prior| {
                    prior.actor_id == car.actor_id
                        && prior.actor_created_frame == car.actor_created_frame
                });
                let angular_error_radians_per_second = car
                    .body
                    .angular_velocity_replay_units
                    .as_ref()
                    .zip(stale.body.angular_velocity_replay_units.as_ref())
                    .filter(|(actual, previous)| {
                        actual.frame == index
                            && valid_masked_interval(index, previous.frame, &original.frames)
                    })
                    .map(|(actual, _)| {
                        distance(
                            predicted.phys.ang_vel.to_array(),
                            actual.value.map(|axis| axis * 0.01),
                        )
                    });
                let position_error_uu = car
                    .body
                    .position
                    .as_ref()
                    .zip(stale.body.position.as_ref())
                    .filter(|(actual, previous)| {
                        actual.frame == index
                            && valid_masked_interval(index, previous.frame, &original.frames)
                    })
                    .map(|(actual, _)| distance(predicted.phys.pos.to_array(), actual.value));
                let rotation_source_frame =
                    stale.body.rotation_xyzw.as_ref().map(|value| value.frame);
                traces.push(MaskedRotationTrace {
                    replay_path: replay_path.display().to_string(),
                    replay_sha256: replay_sha256.to_owned(),
                    frame: index,
                    replay_time: original_frame.time,
                    replay_delta: original_frame.delta,
                    horizon,
                    timeline_tick: conversion.frames[index].timeline_tick,
                    arena_tick: state.tick_count,
                    actor_id: car.actor_id,
                    actor_created_frame: car.actor_created_frame,
                    masked_actor_lifetime_match: stale.actor_created_frame
                        == car.actor_created_frame,
                    player_key: car.player_key.clone().unwrap_or_default(),
                    team: car.team,
                    slot: *slot,
                    rotation_source_frame,
                    rotation_source_gap_seconds: rotation_source_frame
                        .map(|source| original_frame.time - original.frames[source].time),
                    observed_body: car.body.clone(),
                    masked_body: stale.body.clone(),
                    observed_inputs_at_target: car.inputs.clone(),
                    observed_inputs_before_interval: previous_car.map(|car| car.inputs.clone()),
                    observed_pad_pickups_at_target: original_frame
                        .pad_pickups
                        .iter()
                        .filter(|pickup| pickup.instigator_car_id == Some(car.actor_id))
                        .cloned()
                        .collect(),
                    prior_angular_packets_before_mask: prior_angular_packets(
                        original,
                        car,
                        index - horizon + 1,
                    ),
                    previous_simulated,
                    predicted: TraceSimCar::from(predicted),
                    simulated_event_kinds_in_interval: car_event_kinds(
                        &conversion.frames[index].simulated_events,
                        *slot,
                    ),
                    rotation_error_degrees: rotation_errors.map(|(sim, _)| sim),
                    hold_rotation_error_degrees: rotation_errors.map(|(_, hold)| hold),
                    angular_error_radians_per_second,
                    position_error_uu,
                });
            }
            if let (Some(actual), Some(previous)) = (&car.boost, &stale.boost)
                && actual.frame == index
                && valid_masked_interval(index, previous.frame, &original.frames)
            {
                by_boost.add(
                    (predicted.boost - actual.value).abs(),
                    (previous.value - actual.value).abs(),
                );
            }
            if let (Some(position), Some(actual), Some(previous)) = (
                &car.body.position,
                &car.body.angular_velocity_replay_units,
                &stale.body.angular_velocity_replay_units,
            ) && position.frame == index
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

fn paths(root: &Path, final_assessment: bool) -> Result<Vec<(String, PathBuf)>, Box<dyn Error>> {
    // Every directory and file opened is resolved and refused when it is in the sealed test split (a link or
    // junction under another name included).
    replay_to_rocketsim::ensure_unsealed(root, final_assessment)?;
    if root.is_file() {
        if !root.extension().is_some_and(|ext| ext == "replay") {
            return Err("single-file input must have a .replay extension".into());
        }
        let size = root
            .parent()
            .and_then(Path::file_name)
            .ok_or("missing game-size directory")?;
        return Ok(vec![(size.to_string_lossy().into_owned(), root.to_owned())]);
    }
    let mut result = Vec::new();
    for size in ["1v1", "2v2", "3v3"] {
        replay_to_rocketsim::ensure_unsealed(&root.join(size), final_assessment)?;
        for entry in fs::read_dir(root.join(size))? {
            let path = entry?.path();
            if path.extension().is_some_and(|ext| ext == "replay") {
                replay_to_rocketsim::ensure_unsealed(&path, final_assessment)?;
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
    // The residuals at packets measure prediction; the boundary-value solve (`--air-bvp`) fits them.
    options.air_bvp = false;
    options.fit_on_next_packet = false;
    let mut meshes = None;
    let mut mask_seed = None;
    let mut aligned_targets = false;
    // Whether the masked predictor infers packet lags (and so also uses the lag-dependent ground and jump
    // fits) in the aligned variant. `--aligned-targets-raw-predictor` scores the unchanged default
    // predictor against the aligned targets, separating the cleaner target from the better predictor.
    let mut aligned_predictor = true;
    // The one-step residuals are held out by default: the fits that choose a value using the packet that is
    // then scored are switched off for that conversion (`--offline-fits` keeps them: reconstruction quality,
    // in-sample for the rotation and angular velocity).
    let mut offline_fits = false;
    let mut rotation_trace_path = None;
    // The test split is sealed until the frozen assessment (TEST_PROTOCOL.md); only that run passes the flag.
    let mut final_assessment = false;
    while let Some(arg) = args.next() {
        if arg == "--final-assessment" {
            final_assessment = true;
        } else if arg == "--offline-fits" {
            offline_fits = true;
        } else if arg == "--aligned-targets-raw-predictor" {
            aligned_targets = true;
            aligned_predictor = false;
        } else if arg == "--aligned-targets" {
            aligned_targets = true;
        } else if arg == "--no-infer-packet-lag" {
            options.infer_packet_lag = false;
        } else if arg == "--octane-hitbox" {
            options.use_loadout_hitboxes = false;
        } else if arg == "--mask-seed" {
            mask_seed = Some(
                args.next()
                    .ok_or("--mask-seed requires a u64 value")?
                    .to_string_lossy()
                    .parse::<u64>()?,
            );
        } else if arg == "--rotation-trace" {
            rotation_trace_path = Some(PathBuf::from(
                args.next()
                    .ok_or("--rotation-trace requires a JSONL output path")?,
            ));
        } else if meshes.is_none() {
            meshes = Some(PathBuf::from(arg));
        } else {
            return Err("usage: evaluate_corpus <split_dir_or_replay> <report.json> [collision_meshes] [--aligned-targets] [--aligned-targets-raw-predictor] [--offline-fits] [--no-infer-packet-lag] [--octane-hitbox] [--mask-seed u64] [--rotation-trace trace.jsonl] [--final-assessment]".into());
        }
    }
    if replay_to_rocketsim::sealed_path_refused(&root, final_assessment) {
        return Err("refusing to inspect a path with a 'test' component (pass --final-assessment for the frozen run)".into());
    }
    if let Some(meshes) = meshes {
        options.collision_meshes = meshes;
    }
    let replay_paths = paths(&root, final_assessment)?;
    let mut rotation_trace = if let Some(path) = rotation_trace_path {
        Some(BufWriter::new(fs::File::create(path)?))
    } else {
        None
    };
    let mut groups: BTreeMap<String, ByBody> = BTreeMap::new();
    let mut all = ByBody::default();
    let mut one_step_kinematics_groups: BTreeMap<String, KinematicsByBody> = BTreeMap::new();
    let mut one_step_kinematics_all = KinematicsByBody::default();
    let mut one_step_car_angular_by_altitude: BTreeMap<String, FieldSamples> = BTreeMap::new();
    let mut one_step_transition_angular_by_context: BTreeMap<String, FieldSamples> =
        BTreeMap::new();
    let mut report = Report {
        schema_version: 1,
        split_directory: root.display().to_string(),
        metric: "pre-correction position error (UU) on fresh replay positions after an active simulation interval; quantiles pool samples within each group",
        masked_metric: "every 100-frame block masks four consecutive ball/car body and car boost frames; default start offset 1 or replay-hash/seed-derived offset when mask_seed is set; compare uncorrected output with fresh original fields in Active phase and a <=0.5 second field-specific gap; hold baseline uses the last unmasked value",
        mask_seed,
        boxcars_version: "0.12.0",
        rocketsim_revision: replay_to_rocketsim::serialization::ROCKETSIM_REVISION,
        options: options.clone(),
        // The boundary-value solve and the next-packet flip fit use the packet being scored: with either on the
        // one-step rows are in sample, whatever else is held out.
        one_step_fits: if offline_fits || options.air_bvp || options.fit_on_next_packet {
            "offline-fits"
        } else {
            "held-out"
        },
        masked_run_options: {
            let applied = masked_conversion_options(
                &options,
                aligned_targets && aligned_predictor,
                Vec::new(),
            );
            MaskedRunOptions {
                infer_packet_lag: applied.infer_packet_lag,
                block_sim_pad_pickups: applied.block_sim_pad_pickups,
                air_bvp: applied.air_bvp,
                fit_on_next_packet: applied.fit_on_next_packet,
                disable_simulated_demolitions: applied.disable_simulated_demolitions,
                align_contacts: applied.align_contacts,
            }
        },
        aligned_variant: match (aligned_targets, aligned_predictor) {
            (false, _) => "",
            (true, true) => "lag-inferring predictor, offline targets",
            (true, false) => "raw predictor, offline targets",
        },
        replays: Vec::new(),
        failures: Vec::new(),
        by_game_size: BTreeMap::new(),
        all: all.summary(),
        one_step_kinematics_by_game_size: BTreeMap::new(),
        one_step_kinematics_all: one_step_kinematics_all.summary(),
        one_step_car_angular_by_altitude: BTreeMap::new(),
        one_step_transition_angular_by_context: BTreeMap::new(),
        masked_position_uu_by_horizon_frames: BTreeMap::new(),
        masked_by_game_size: BTreeMap::new(),
        masked_position_uu_by_start_packet_age_bucket: BTreeMap::new(),
        masked_kinematics_by_horizon_frames: BTreeMap::new(),
        masked_kinematics_by_game_size: BTreeMap::new(),
        masked_boost_by_horizon_frames: BTreeMap::new(),
        masked_boost_by_game_size: BTreeMap::new(),
        masked_car_angular_by_altitude: BTreeMap::new(),
        worst_car_regret_uu: Vec::new(),
    };
    let mut strict_options = options.clone();
    if !offline_fits {
        strict_options.flip_cancel_holdout = true;
        strict_options.infer_air_controls_from_lookahead = false;
        strict_options.infer_dodge_first_packet_tick = false;
        strict_options.align_contacts = false;
    }
    let mut masked_by_horizon: BTreeMap<usize, ByBody> = BTreeMap::new();
    let mut masked_by_size: BTreeMap<String, BTreeMap<usize, ByBody>> = BTreeMap::new();
    let mut masked_by_age: BTreeMap<&'static str, ByBody> = BTreeMap::new();
    let mut kinematics_by_horizon: BTreeMap<usize, KinematicsByBody> = BTreeMap::new();
    let mut kinematics_by_size: BTreeMap<String, BTreeMap<usize, KinematicsByBody>> =
        BTreeMap::new();
    let mut boost_by_horizon: BTreeMap<usize, FieldSamples> = BTreeMap::new();
    let mut boost_by_size: BTreeMap<String, BTreeMap<usize, FieldSamples>> = BTreeMap::new();
    let mut car_angular_by_altitude: BTreeMap<String, FieldSamples> = BTreeMap::new();
    for (index, (size, path)) in replay_paths.iter().enumerate() {
        match fs::read(path)
            .map_err(|error| error.to_string())
            .and_then(|bytes| {
                convert_bytes(&bytes, &strict_options).map_err(|error| error.to_string())
            }) {
            Ok(conversion) => {
                let mut own = ByBody::default();
                let mut own_kinematics = KinematicsByBody::default();
                let mut own_masked_kinematics_report = BTreeMap::new();
                let mut own_masked_position_report = BTreeMap::new();
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

                    if residual.actor_id.is_some()
                        && let (Some(alt), Some(sim_ang), Some(hold_ang)) = (
                            residual.altitude_z,
                            residual.simulated_angular_velocity_error_rad_per_sec,
                            residual.hold_angular_velocity_error_rad_per_sec,
                        )
                    {
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
                        if altitude == "transition"
                            && let Some(contexts) = transition_contexts(&conversion, residual)
                        {
                            for context in contexts {
                                one_step_transition_angular_by_context
                                    .entry(context.to_owned())
                                    .or_default()
                                    .add(sim_ang, hold_ang);
                            }
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
                    if keep && let Some(outlier) = outlier_record(path, &conversion, residual) {
                        report.worst_car_regret_uu.push(outlier);
                        report
                            .worst_car_regret_uu
                            .sort_by(|a, b| b.regret_uu.total_cmp(&a.regret_uu));
                        report.worst_car_regret_uu.truncate(100);
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
                let masked_options = masked_conversion_options(
                    &options,
                    aligned_targets && aligned_predictor,
                    (0..masked.frames.len())
                        .map(|index| schedule.horizon(index).is_some())
                        .collect(),
                );
                // The aligned targets come from the full offline conversion (the best estimate of the state at
                // the target tick), not from the held-out one scored above.
                let offline_target = if aligned_targets && !offline_fits {
                    match fs::read(path)
                        .map_err(|error| error.to_string())
                        .and_then(|bytes| {
                            convert_bytes(&bytes, &options).map_err(|error| error.to_string())
                        }) {
                        Ok(target) => Some(target),
                        Err(error) => {
                            report.failures.push(Failure {
                                path: path.display().to_string(),
                                error: format!("aligned target conversion: {error}"),
                            });
                            None
                        }
                    }
                } else {
                    None
                };
                match convert_observations(masked, &masked_options) {
                    Ok(masked_conversion) => {
                        let mut own_masked = BTreeMap::new();
                        let mut own_masked_kinematics = BTreeMap::new();
                        let mut own_boost = BTreeMap::new();
                        let mut own_angular_by_altitude = BTreeMap::new();
                        let mut own_rotation_traces = Vec::new();
                        masked_metrics(
                            &conversion.observations,
                            &masked_conversion,
                            schedule,
                            path,
                            conversion.source_sha256.as_deref().unwrap_or_default(),
                            rotation_trace.as_ref().map(|_| &mut own_rotation_traces),
                            &mut own_masked,
                            &mut own_masked_kinematics,
                            &mut own_boost,
                            &mut own_angular_by_altitude,
                            aligned_targets.then(|| offline_target.as_ref().unwrap_or(&conversion)),
                        );
                        if let Some(writer) = &mut rotation_trace {
                            for trace in own_rotation_traces {
                                serde_json::to_writer(&mut *writer, &trace)?;
                                writer.write_all(b"\n")?;
                            }
                        }
                        own_masked_kinematics_report = own_masked_kinematics
                            .iter()
                            .map(|(&horizon, samples)| (horizon, samples.summary()))
                            .collect();
                        for (altitude, samples) in own_angular_by_altitude {
                            car_angular_by_altitude
                                .entry(altitude)
                                .or_default()
                                .extend(&samples);
                        }
                        own_masked_position_report = own_masked
                            .iter()
                            .map(|(&horizon, samples)| (horizon, samples.summary_with_age()))
                            .collect();
                        for (horizon, samples) in own_masked {
                            for (bucket, part) in samples.ball.by_age_bucket() {
                                masked_by_age.entry(bucket).or_default().ball.extend(&part);
                            }
                            for (bucket, part) in samples.car.by_age_bucket() {
                                masked_by_age.entry(bucket).or_default().car.extend(&part);
                            }
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
                    ball_lag_frames: conversion.diagnostics.ball_lag_frames,
                    car_lag_frames: conversion.diagnostics.car_lag_frames,
                    active_pawn_demo_corrections: conversion
                        .diagnostics
                        .active_pawn_demo_corrections,
                    default_hitbox_players: conversion.diagnostics.default_hitbox_players,
                    car_slots: conversion.car_slots.clone(),
                    position_uu: own.summary(),
                    kinematics: own_kinematics.summary(),
                    masked_kinematics_by_horizon_frames: own_masked_kinematics_report,
                    masked_position_uu_by_horizon_frames: own_masked_position_report,
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
    report.one_step_transition_angular_by_context = one_step_transition_angular_by_context
        .into_iter()
        .map(|(context, samples)| (context, samples.summary()))
        .collect();
    report.masked_position_uu_by_horizon_frames = masked_by_horizon
        .into_iter()
        .map(|(horizon, samples)| (horizon, samples.summary_with_age()))
        .collect();
    report.masked_position_uu_by_start_packet_age_bucket = masked_by_age
        .into_iter()
        .map(|(bucket, samples)| (bucket.to_owned(), samples.summary_with_age()))
        .collect();
    report.masked_by_game_size = masked_by_size
        .into_iter()
        .map(|(size, by_horizon)| {
            (
                size,
                by_horizon
                    .into_iter()
                    .map(|(horizon, samples)| (horizon, samples.summary_with_age()))
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
    if let Some(writer) = &mut rotation_trace {
        writer.flush()?;
    }
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
    // The report is written either way; a failed replay must not look like a clean run to a script.
    if !report.failures.is_empty() {
        return Err(format!(
            "{} replay(s) failed (listed in the report)",
            report.failures.len()
        )
        .into());
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

    /// The masked observations must not depend on the withheld frames' own body, boost or pad data: perturb
    /// those fields in the original (bodies by hundreds of UU, boost, ball) at the withheld frames and the
    /// masked replay is byte-identical; perturbing a frame that is not withheld changes it (the test can
    /// tell the difference). Uses a local train replay; skipped when it is absent.
    #[test]
    fn masked_observations_do_not_depend_on_the_withheld_frames() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR"));
        let Some(path) = fs::read_dir(root.join("replays/train/1v1"))
            .ok()
            .and_then(|dir| {
                let mut paths: Vec<_> = dir
                    .filter_map(|e| e.ok().map(|e| e.path()))
                    .filter(|p| p.extension().is_some_and(|x| x == "replay"))
                    .collect();
                paths.sort();
                paths.into_iter().next()
            })
        else {
            eprintln!("skipping masked leak test: no local train replay");
            return;
        };
        let bytes = fs::read(path).unwrap();
        let replay = boxcars::ParserBuilder::new(&bytes)
            .must_parse_network_data()
            .parse()
            .unwrap();
        let original = replay_to_rocketsim::observations::extract(&replay).unwrap();
        let schedule = MaskSchedule {
            seed: None,
            replay_hash: 7,
        };
        let shift = |body: &mut Body, by: f32| {
            for value in body.position.iter_mut() {
                value.value[0] += by;
            }
            for value in body.linear_velocity.iter_mut() {
                value.value[1] += by;
            }
            for value in body.rotation_xyzw.iter_mut() {
                value.value[2] += 0.1 * by.signum();
            }
            for value in body.angular_velocity_replay_units.iter_mut() {
                value.value[0] += by;
            }
        };
        let perturb = |observed: &mut ObservedReplay, withheld: bool| {
            for index in 1..observed.frames.len() {
                if schedule.horizon(index).is_some() != withheld {
                    continue;
                }
                let frame = &mut observed.frames[index];
                if let Some(ball) = frame.ball.as_mut() {
                    shift(ball, 300.0);
                }
                for car in &mut frame.cars {
                    shift(&mut car.body, 300.0);
                    for boost in car.boost.iter_mut() {
                        boost.value += 11.0;
                    }
                    for raw in car.boost_raw.iter_mut() {
                        raw.value = raw.value.wrapping_add(9);
                    }
                }
            }
        };
        let json = |observed: &ObservedReplay| -> Vec<String> {
            observed
                .frames
                .iter()
                .map(|f| serde_json::to_string(f).unwrap())
                .collect()
        };
        let baseline = json(&masked_observations(&original, schedule));
        let mut withheld_changed = original.clone();
        perturb(&mut withheld_changed, true);
        assert_eq!(
            baseline,
            json(&masked_observations(&withheld_changed, schedule)),
            "the masked replay depends on a withheld frame's body or boost"
        );
        let mut other_changed = original.clone();
        perturb(&mut other_changed, false);
        assert_ne!(
            baseline,
            json(&masked_observations(&other_changed, schedule)),
            "the perturbation of a frame that is not withheld must show"
        );
    }

    /// The exported physics and boost of the default masked conversion must not depend on anything after a
    /// window: the replay truncated right after a window gives the same states in the window and before
    /// it. A positive control on the truncation itself: the aligned predictor depends on later packets
    /// (the replay-wide ball-car offset), so the same truncation must change its states; a secondary
    /// check feeds a packet from after the window into a window frame on purpose. Pads differ (the replay-wide pad-name votes, RESULTS
    /// 'Audit, part 4', finding 2) and are only reported. The aligned predictor's dependence is by design
    /// (accepted: the evaluation measures the reconstruction, not a causal predictor); its largest position
    /// difference is printed.
    /// 1v1 `00a0da63` has a defined offset; skipped when that train replay or the collision meshes are
    /// absent.
    #[test]
    fn masked_conversions_do_not_depend_on_frames_after_a_window() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR"));
        let path = root.join("replays/train/1v1/00a0da63-492e-4ab7-8a07-16cd5d14dcb4.replay");
        if !path.exists() || !root.join("collision_meshes").exists() {
            eprintln!("skipping the masked causality test: no local train replay or meshes");
            return;
        }
        let bytes = fs::read(path).unwrap();
        let replay = boxcars::ParserBuilder::new(&bytes)
            .must_parse_network_data()
            .parse()
            .unwrap();
        let original = replay_to_rocketsim::observations::extract(&replay).unwrap();
        drop(replay);
        let schedule = MaskSchedule {
            seed: None,
            replay_hash: 7,
        };
        // The evaluator's own base options.
        let mut options = ConvertOptions::default();
        options.air_bvp = false;
        options.fit_on_next_packet = false;
        let signature = |output: &ConversionOutput, frame: usize| -> String {
            let state = &output.frames[frame].state;
            let mut cars: Vec<_> = state
                .cars
                .iter()
                .map(|(info, car)| {
                    format!(
                        "{} {:?} {:?} {}",
                        info.idx,
                        car.phys,
                        car.boost.to_bits(),
                        car.is_demoed
                    )
                })
                .collect();
            cars.sort();
            format!("{:?} {:?}", state.ball.phys, cars)
        };
        let pads = |output: &ConversionOutput, frame: usize| {
            format!("{:?}", output.frames[frame].state.boost_pads)
        };
        let convert =
            |observed: &ObservedReplay, lag_inference: bool, leak: Option<(usize, usize)>| {
                let withheld = (0..observed.frames.len())
                    .map(|i| schedule.horizon(i).is_some())
                    .collect();
                let masked_options = masked_conversion_options(&options, lag_inference, withheld);
                let mut masked = masked_observations(observed, schedule);
                if let Some((from, to)) = leak {
                    // Deliberately feed the ball packet of frame `from` (after the window) into window frame
                    // `to`, stamped as that frame's own fresh packet.
                    let mut ball = observed.frames[from].ball.clone().expect("a ball body");
                    for value in ball.position.iter_mut() {
                        value.frame = to;
                    }
                    for value in ball.linear_velocity.iter_mut() {
                        value.frame = to;
                    }
                    for value in ball.rotation_xyzw.iter_mut() {
                        value.frame = to;
                    }
                    for value in ball.angular_velocity_replay_units.iter_mut() {
                        value.frame = to;
                    }
                    masked.frames[to].ball = Some(ball);
                }
                convert_observations(masked, &masked_options).unwrap()
            };
        let truncated_at = |end: usize| {
            let mut truncated = original.clone();
            // Right after the window: frame `end` is its last frame.
            truncated.frames.truncate(end + 1);
            truncated
        };
        // Default predictor: a reference replay of the first 1,200 frames against the same replay truncated
        // right after the windows at frames 401 and 901.
        let reference_frames = 1200;
        let mut reference_replay = original.clone();
        reference_replay.frames.truncate(reference_frames);
        let reference = convert(&reference_replay, false, None);
        for window in [4usize, 9] {
            let end = window * 100 + 4;
            let cut = convert(&truncated_at(end), false, None);
            let (mut physics, mut pad_frames) = (0usize, 0usize);
            for frame in 0..=end {
                physics += usize::from(signature(&reference, frame) != signature(&cut, frame));
                pad_frames += usize::from(pads(&reference, frame) != pads(&cut, frame));
            }
            eprintln!(
                "default predictor, window at {}: {physics} of {} frames differ in physics or boost, {pad_frames} in pads",
                window * 100 + 1,
                end + 1
            );
            assert_eq!(
                physics,
                0,
                "truncating the replay after the window at {} changes the exported states",
                window * 100 + 1
            );
        }
        // Secondary signature check: a ball packet from 40 frames after the window, fed into its first frame
        // in the longer replay only, changes the states of the window.
        let end = 4 * 100 + 4;
        let leaked = convert(&reference_replay, false, Some((end + 40, 401)));
        let cut = convert(&truncated_at(end), false, None);
        let differing = (0..=end)
            .filter(|&frame| signature(&leaked, frame) != signature(&cut, frame))
            .count();
        eprintln!(
            "signature check (a post-window ball packet fed into frame 401): {differing} frames differ"
        );
        assert!(
            differing > 0,
            "the injected packet found no dependence: the test cannot tell"
        );
        // Aligned predictor: its known future dependence (the replay-wide ball-car offset) is the positive
        // control of the truncation path itself: the same truncation must change the states here.
        let full = convert(&original, true, None);
        let cut = convert(&truncated_at(904), true, None);
        let mut worst = 0.0f32;
        for frame in 0..=904 {
            worst = worst.max(
                (full.frames[frame].state.ball.phys.pos - cut.frames[frame].state.ball.phys.pos)
                    .length(),
            );
            for ((_, a), (_, b)) in full.frames[frame]
                .state
                .cars
                .iter()
                .zip(&cut.frames[frame].state.cars)
            {
                worst = worst.max((a.phys.pos - b.phys.pos).length());
            }
        }
        let differing = (0..=904)
            .filter(|&frame| signature(&full, frame) != signature(&cut, frame))
            .count();
        eprintln!(
            "aligned predictor, window at 901: {differing} of 905 frames differ, largest ball or car position difference {worst:.2} UU"
        );
        assert!(
            differing > 0,
            "the aligned predictor must depend on the later packets (positive control of the truncation)"
        );
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
    fn prior_angular_trace_excludes_masked_targets_and_other_actor_lifetimes() {
        use replay_to_rocketsim::observations::{
            Car, Frame, Header, Inputs, ObservedReplay, Source, Value,
        };

        let frames = (0..6)
            .map(|index| {
                let actor_created_frame = if index == 0 { 0 } else { 1 };
                let angular_velocity_replay_units = Some(Value {
                    value: [index as f32 * 100.0, 0.0, 0.0],
                    frame: index,
                    source: Source::Replay,
                });
                Frame {
                    index,
                    time: index as f32 * 0.03,
                    delta: 0.03,
                    ball: None,
                    cars: vec![Car {
                        actor_id: 7,
                        actor_created_frame,
                        player_key: None,
                        player_link_active: false,
                        team: None,
                        body_product_id: None,
                        body: Body {
                            angular_velocity_replay_units,
                            ..Body::default()
                        },
                        boost: None,
                        boost_raw: None,
                        spawn_pose: None,
                        inputs: Inputs::default(),
                    }],
                    players: Vec::new(),
                    team_scores: [None, None],
                    seconds_remaining: None,
                    overtime: None,
                    game_state: None,
                    events: Vec::new(),
                    pad_pickups: Vec::new(),
                }
            })
            .collect();
        let observed = ObservedReplay {
            header: Header {
                game_type: "Soccar".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames,
            diagnostics: Default::default(),
        };
        let car = &observed.frames[5].cars[0];
        let packets = prior_angular_packets(&observed, car, 4);
        assert_eq!(
            packets.iter().map(|p| p.frame).collect::<Vec<_>>(),
            [3, 2, 1]
        );
        assert_eq!(
            packets[0].angular_velocity_radians_per_second,
            [3.0, 0.0, 0.0]
        );
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
                    spawn_pose: None,
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
