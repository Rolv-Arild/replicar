//! Replay frame snapshots produced by short RocketSim steps and fresh corrections.

use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fmt;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};

use glam::Quat;
use rocketsim::{
    Arena, ArenaConfig, ArenaEvent, ArenaState, BoostPadState, CarBodyConfig, CarControls,
    CarState, GameMode, Mat3A, PhysState, Team, Vec3A,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::observations::{self, Body, ObservedReplay, Value};
use crate::parse_replay;

#[derive(Debug)]
pub enum ConvertError {
    Parse(boxcars::ParseError),
    MissingNetworkFrames,
    UnsupportedMode(String),
    Init(io::Error),
    Output(io::Error),
    InvalidTime { frame: usize, time: f32 },
}

impl fmt::Display for ConvertError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Parse(error) => write!(f, "replay parse failed: {error}"),
            Self::MissingNetworkFrames => write!(f, "replay has no network frames"),
            Self::UnsupportedMode(mode) => write!(f, "unsupported replay mode: {mode}"),
            Self::Init(error) => write!(f, "RocketSim initialization failed: {error}"),
            Self::Output(error) => write!(f, "conversion output failed: {error}"),
            Self::InvalidTime { frame, time } => {
                write!(f, "invalid replay time {time} at frame {frame}")
            }
        }
    }
}

impl Error for ConvertError {}

#[derive(Debug, Clone, Serialize)]
pub struct ConvertOptions {
    pub collision_meshes: PathBuf,
    pub seed: u64,
    /// Interpret odd boost-component ReplicatedActive bytes as active boost input.
    pub infer_boost_from_active: bool,
    /// Infer jump input from odd jump-component activation bytes.
    pub infer_jump_from_active: bool,
    /// Start an inferred jump only while near the ground and before its impulse is observed.
    pub gate_jump_on_observed_impulse: bool,
    /// Infer dodge flip from odd dodge-component activation and DodgeTorque.
    pub infer_dodge_from_active: bool,
    /// Gate dodge impulse so that it only triggers when the impulse has not yet been observed.
    pub gate_dodge_on_observed_impulse: bool,
    /// Reconcile boost pad pickups and cooldowns from replay pickup data.
    pub sync_boost_pad_pickups: bool,
    /// Route steer input to aerial yaw while airborne.
    pub infer_air_steer_controls: bool,
    /// Offline: while a car is flipping, infer how much of the flip's pitch torque a player cancelled
    /// (opposite pitch input, which replays do not carry) by simulating candidates against the
    /// next fresh car packet, and hold the last inferred cancel where no later packet exists.
    pub infer_flip_cancel: bool,
    /// Apply the extra ball-car hit impulse that RocketSim computes (and reports as
    /// `CarHitBall.extra_hit_vel`) but that the pinned build discards, because it is added to a
    /// per-tick accumulator that is cleared before it reaches the ball.
    pub apply_hit_extra_impulse: bool,
    /// Clamp reported car and ball velocities to RocketSim's limits after each step. RocketSim
    /// applies its limits at the start of the next tick, so the state it reports after a step can
    /// exceed them (a flipping car by up to 2.2 rad/s), whereas replay states never do.
    pub limit_reported_velocities: bool,
    /// Chain packet lags on whole tick counts: snap each chained interval to an integer (rejecting
    /// pairs more than 0.25 tick from one), so lag differences are exact instead of independently
    /// rounded estimates, and fix the absolute tick with the packets' real-time windows.
    pub exact_tick_lag_chains: bool,
    /// Offline: infer when inside its frame each ball and car packet was generated (its lag behind
    /// the frame time, in ticks) from chained packet motion, and apply corrections at that time.
    pub infer_packet_lag: bool,
    /// While airborne with the replicated handbrake held, route steer to roll instead of yaw.
    pub infer_air_roll_from_handbrake: bool,
    /// Infer aerial pitch, yaw, and roll controls from subsequent observed angular velocity.
    pub infer_air_controls_from_lookahead: bool,
    /// Longest span, in replay frames, between fresh car angular packets that the offline aerial
    /// inverse may bridge with one constant control (1 = adjacent frames only). The default is
    /// effectively unbounded: a constant control over a bracketing pair beats no control.
    pub air_lookahead_max_frames: usize,
    /// Longest replay-time span, in seconds, that the aerial inverse may bridge.
    pub air_lookahead_max_seconds: f32,
    /// Extra forward-model correction passes for a multi-tick aerial inverse (0 = analytic only).
    pub air_lookahead_refine_iterations: usize,
    /// Causal: keep the aerial control implied by the two latest fresh car angular packets before
    /// the interval, when no later packet brackets it. Uses no data from after the interval.
    pub persist_past_air_controls: bool,
    /// Scale a past control by its measured conditional-median persistence
    /// (`AIR_CONTROL_MEDIAN_RATIO`), and use observed steer/handbrake for the axis they drive.
    /// When false, the legacy gates below (expiry, minimum magnitude, gain, speed drop) apply.
    pub air_persist_calibrated: bool,
    /// Legacy gate: longest time after the latest fresh packet for which a past control is kept.
    pub air_persist_max_seconds: f32,
    /// Scale applied to persisted pitch, yaw, and roll (1 keeps the fitted control).
    pub air_persist_gain: f32,
    /// A past control is persisted only when its larger pitch/roll magnitude reaches this value.
    pub air_persist_min_control: f32,
    /// Skip persistence when angular speed fell by more than this between the two fitted packets
    /// (a large value disables the gate).
    pub air_persist_max_speed_drop: f32,
    /// Frames whose car/ball packets were withheld by an evaluator. A lookahead span that contains
    /// one would use a packet from after a withheld target, so it is refused.
    #[serde(skip)]
    pub withheld_frames: Option<Arc<Vec<bool>>>,
    /// Include 50–100 UU airborne packets in the offline aerial inverse diagnostic.
    pub infer_transition_air_lookahead: bool,
    /// Compensate RocketSim air damping in the low-air transition band without future packets.
    pub compensate_transition_air_damping: bool,
    /// Experimental: carry a recent replay angular velocity across contact-free low-air intervals.
    pub hold_low_air_angular: bool,
    /// Experimental: require a prior observed angular speed near the 5.5 rad/s packet cap.
    pub gate_low_air_angular_by_speed: bool,
    /// Experimental: use RocketSim air controls to steer toward a recent low-air angular packet.
    pub feedback_low_air_angular: bool,
    /// Select a RocketSim hitbox from the replay player's car-body product ID when known.
    pub use_loadout_hitboxes: bool,
    /// Gaps larger than this are left unsimulated and recorded in diagnostics.
    pub max_gap_ticks: u64,
}

impl Default for ConvertOptions {
    fn default() -> Self {
        Self {
            collision_meshes: PathBuf::from("collision_meshes"),
            seed: 0,
            infer_boost_from_active: true,
            infer_jump_from_active: true,
            gate_jump_on_observed_impulse: true,
            infer_dodge_from_active: true,
            gate_dodge_on_observed_impulse: true,
            sync_boost_pad_pickups: true,
            infer_air_steer_controls: true,
            infer_flip_cancel: true,
            apply_hit_extra_impulse: true,
            limit_reported_velocities: true,
            exact_tick_lag_chains: true,
            infer_packet_lag: true,
            infer_air_roll_from_handbrake: true,
            infer_air_controls_from_lookahead: true,
            air_lookahead_max_frames: 10_000,
            air_lookahead_max_seconds: 1_000.0,
            air_lookahead_refine_iterations: 1,
            persist_past_air_controls: true,
            air_persist_calibrated: true,
            air_persist_max_seconds: 0.15,
            air_persist_gain: 1.0,
            air_persist_min_control: 0.5,
            air_persist_max_speed_drop: 1.0e6,
            withheld_frames: None,
            infer_transition_air_lookahead: true,
            compensate_transition_air_damping: false,
            hold_low_air_angular: false,
            gate_low_air_angular_by_speed: false,
            feedback_low_air_angular: false,
            use_loadout_hitboxes: true,
            max_gap_ticks: 1200,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SimEvent {
    pub arena_tick: u64,
    pub event: ArenaEvent,
}

/// The packet lag applied to one object in a frame (`infer_packet_lag`): its packet was treated as
/// generated `ticks` 120 Hz ticks before the frame time. `source` is `chain` (the object's own
/// motion chain), `frame_median` (median of the frame's chained cars) or `default` (half the frame
/// window, the median of an unknown lag). Inferred from neighboring packets; not observed.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct AppliedPacketLag {
    pub actor_id: Option<i32>,
    pub ticks: u64,
    pub source: &'static str,
}

#[derive(Debug, Clone)]
pub struct ConvertedFrame {
    pub replay_frame: usize,
    pub replay_time: f32,
    /// 120 Hz tick on the elapsed replay timeline (including frozen phases).
    pub timeline_tick: u64,
    pub state: ArenaState,
    pub simulated_events: Vec<SimEvent>,
    /// Applied packet lags for objects with a fresh packet; empty unless `infer_packet_lag`.
    pub packet_lags: Vec<AppliedPacketLag>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize)]
pub struct Diagnostics {
    pub skipped_timeline_ticks: u64,
    pub unlinked_car_frames: usize,
    pub default_hitbox_players: usize,
    pub active_pawn_demo_corrections: usize,
    pub shadowed_car_frames: usize,
    /// Simulated frames with an inferred ball packet lag (`infer_packet_lag`).
    pub ball_lag_frames: usize,
    /// Simulated frames with an inferred car packet lag (`infer_packet_lag`).
    pub car_lag_frames: usize,
}

#[derive(Debug, Clone)]
pub struct ConversionOutput {
    pub source_sha256: Option<String>,
    pub options: ConvertOptions,
    pub observations: ObservedReplay,
    pub frames: Vec<ConvertedFrame>,
    pub position_residuals: Vec<PositionResidual>,
    pub car_slots: Vec<CarSlot>,
    pub diagnostics: Diagnostics,
}

/// Metadata from a conversion that emits each state through a callback.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConversionSummary {
    pub car_slots: Vec<CarSlot>,
    pub diagnostics: Diagnostics,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CarSlot {
    pub slot: usize,
    pub player_key: String,
    pub team: u8,
    /// Body product ID available when this RocketSim slot was created.
    pub body_product_id: Option<u32>,
    pub hitbox: String,
}

/// Offline motion-derived interval estimate between two car observations.
/// This is a diagnostic projection, not a measured packet timestamp or engine tick count.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct OfflineIntervalEstimate {
    /// Motion-derived duration after rounding to the nearest 120 Hz tick.
    pub effective_seconds: f32,
    /// Rounded tick count implied by the motion projection; not observed engine ticks.
    pub effective_ticks: u32,
    /// Ratio of effective duration to nominal frame delta (`effective_seconds / nominal_dt`).
    pub scale: f32,
}

/// Error before a fresh replay observation is used to correct the simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PositionResidual {
    pub frame: usize,
    pub actor_id: Option<i32>,
    pub seconds_since_previous_position: f32,
    pub simulated_error_uu: f32,
    /// Predicted minus observed position before correction.
    pub simulated_error_vector_uu: [f32; 3],
    /// Last observed linear velocity before this packet, when a recent one exists.
    pub previous_linear_velocity_uu_per_second: Option<[f32; 3]>,
    pub hold_error_uu: f32,
    pub linear_extrapolation_error_uu: Option<f32>,
    pub simulated_velocity_error_uu_per_sec: Option<f32>,
    pub hold_velocity_error_uu_per_sec: Option<f32>,
    pub simulated_rotation_error_degrees: Option<f32>,
    pub hold_rotation_error_degrees: Option<f32>,
    pub simulated_angular_velocity_error_rad_per_sec: Option<f32>,
    pub hold_angular_velocity_error_rad_per_sec: Option<f32>,
    pub altitude_z: Option<f32>,
    pub is_on_ground: Option<bool>,
    pub offline_interval: Option<OfflineIntervalEstimate>,
    /// In-sample fit: the target position is used both to infer the interval and score it.
    pub offline_projection_fit_error_uu: Option<f32>,
}

pub type KinematicResidual = PositionResidual;

pub fn quaternion(xyzw: [f32; 4]) -> Option<Quat> {
    let q = Quat::from_xyzw(xyzw[0], xyzw[1], xyzw[2], xyzw[3]);
    (q.is_finite() && q.length_squared() > 1e-8).then(|| q.normalize())
}

pub fn rotation_error_degrees(a: Mat3A, b: Mat3A) -> f32 {
    let trace = a.x_axis.dot(b.x_axis) + a.y_axis.dot(b.y_axis) + a.z_axis.dot(b.z_axis);
    ((trace - 1.0) * 0.5).clamp(-1.0, 1.0).acos().to_degrees()
}

const PI: f32 = std::f32::consts::PI;
const TORQUE_APPLY_SCALE: f32 = 2.0 * PI / 65536.0 * 1000.0;
const TORQUE_PITCH: f32 = 130.0 * TORQUE_APPLY_SCALE;
const TORQUE_YAW: f32 = 95.0 * TORQUE_APPLY_SCALE;
const TORQUE_ROLL: f32 = 400.0 * TORQUE_APPLY_SCALE;
const DAMPING_PITCH: f32 = 30.0 * TORQUE_APPLY_SCALE;
const DAMPING_YAW: f32 = 20.0 * TORQUE_APPLY_SCALE;
const DAMPING_ROLL: f32 = 50.0 * TORQUE_APPLY_SCALE;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AirControls {
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
}

/// Analytically inverts RocketSim's air torque equations from consecutive angular velocities.
pub fn solve_inverse_air_controls(
    rot_mat_start: Mat3A,
    ang_vel_start: Vec3A,
    ang_vel_end: Vec3A,
    dt: f32,
) -> AirControls {
    if dt <= 0.0 || !dt.is_finite() {
        return AirControls::default();
    }

    let forward = rot_mat_start.x_axis;
    let right = rot_mat_start.y_axis;
    let up = rot_mat_start.z_axis;

    let dir_pitch = -right;
    let dir_yaw = up;
    let dir_roll = -forward;

    let tau_world = (ang_vel_end - ang_vel_start) / dt;

    let tau_p = dir_pitch.dot(tau_world);
    let tau_y = dir_yaw.dot(tau_world);
    let tau_r = dir_roll.dot(tau_world);

    let omega_p = dir_pitch.dot(ang_vel_start);
    let omega_y = dir_yaw.dot(ang_vel_start);
    let omega_r = dir_roll.dot(ang_vel_start);

    // Solve pitch: tau_p = u_p * T_p - omega_p * D_p * (1 - |u_p|)
    let rhs_p = tau_p + omega_p * DAMPING_PITCH;
    let denom_p = TORQUE_PITCH + rhs_p.signum() * omega_p * DAMPING_PITCH;
    let pitch = if denom_p.abs() > 1e-4 {
        (rhs_p / denom_p).clamp(-1.0, 1.0)
    } else {
        0.0
    };

    // Solve yaw: tau_y = u_y * T_y - omega_y * D_y * (1 - |u_y|)
    let rhs_y = tau_y + omega_y * DAMPING_YAW;
    let denom_y = TORQUE_YAW + rhs_y.signum() * omega_y * DAMPING_YAW;
    let yaw = if denom_y.abs() > 1e-4 {
        (rhs_y / denom_y).clamp(-1.0, 1.0)
    } else {
        0.0
    };

    // Solve roll: tau_r = u_r * T_r - omega_r * D_r (no damping reduction in RocketSim)
    let rhs_r = tau_r + omega_r * DAMPING_ROLL;
    let roll = (rhs_r / TORQUE_ROLL).clamp(-1.0, 1.0);

    AirControls { pitch, yaw, roll }
}

const AIR_MAX_ANGULAR_SPEED: f32 = 5.5;

/// Integrates RocketSim's air torque and damping for `ticks` 120 Hz ticks under constant controls.
/// Returns the final world angular velocity. Flips, contact, and boost/throttle effects are absent.
pub fn air_angular_velocity_forward(
    rot_mat_start: Mat3A,
    ang_vel_start: Vec3A,
    controls: AirControls,
    ticks: u32,
) -> Vec3A {
    const TICK: f32 = 1.0 / 120.0;
    let mut rot = rot_mat_start;
    let mut omega = ang_vel_start;
    for _ in 0..ticks {
        let dir_pitch = -rot.y_axis;
        let dir_yaw = rot.z_axis;
        let dir_roll = -rot.x_axis;
        let any = controls.pitch != 0.0 || controls.yaw != 0.0 || controls.roll != 0.0;
        let torque = if any {
            dir_pitch * (controls.pitch * TORQUE_PITCH)
                + dir_yaw * (controls.yaw * TORQUE_YAW)
                + dir_roll * (controls.roll * TORQUE_ROLL)
        } else {
            Vec3A::ZERO
        };
        let damping = dir_pitch
            * (dir_pitch.dot(omega) * DAMPING_PITCH * (1.0 - controls.pitch.abs()))
            + dir_yaw * (dir_yaw.dot(omega) * DAMPING_YAW * (1.0 - controls.yaw.abs()))
            + dir_roll * (dir_roll.dot(omega) * DAMPING_ROLL);
        omega += (torque - damping) * TICK;
        let speed = omega.length();
        if speed > AIR_MAX_ANGULAR_SPEED {
            omega *= AIR_MAX_ANGULAR_SPEED / speed;
        }
        let step = omega * TICK;
        if step.length_squared() > 0.0 {
            rot = Mat3A::from_quat(Quat::from_scaled_axis(step.into())) * rot;
        }
    }
    omega
}

/// Constant controls over `ticks` that carry `ang_vel_start` to `ang_vel_end`. Starts from the
/// analytic inverse and applies forward-model corrections (`iterations` = 0 gives the analytic
/// result over `ticks / 120` seconds).
pub fn solve_span_air_controls(
    rot_mat_start: Mat3A,
    ang_vel_start: Vec3A,
    ang_vel_end: Vec3A,
    ticks: u32,
    iterations: usize,
) -> AirControls {
    let dt = ticks as f32 / 120.0;
    let mut virtual_target = ang_vel_end;
    let mut controls = solve_inverse_air_controls(rot_mat_start, ang_vel_start, virtual_target, dt);
    for _ in 0..iterations {
        let reached = air_angular_velocity_forward(rot_mat_start, ang_vel_start, controls, ticks);
        virtual_target += ang_vel_end - reached;
        controls = solve_inverse_air_controls(rot_mat_start, ang_vel_start, virtual_target, dt);
    }
    controls
}

fn vec3(value: [f32; 3]) -> Vec3A {
    Vec3A::new(value[0], value[1], value[2])
}

/// Projects displacement onto mean velocity and rounds the implied interval to 120 Hz.
/// Uses both endpoint positions and velocities, so it cannot predict the second position.
pub fn estimate_car_packet_interval(
    pos_start: [f32; 3],
    pos_end: [f32; 3],
    vel_start: [f32; 3],
    vel_end: [f32; 3],
    nominal_dt: f32,
) -> Option<OfflineIntervalEstimate> {
    if !nominal_dt.is_finite() || nominal_dt <= 0.0 || nominal_dt > 0.5 {
        return None;
    }
    let p0 = vec3(pos_start);
    let p1 = vec3(pos_end);
    let v0 = vec3(vel_start);
    let v1 = vec3(vel_end);

    let v_mean = (v0 + v1) * 0.5;
    let v_sq = v_mean.length_squared();
    if !v_sq.is_finite() || v_sq < 100.0 * 100.0 {
        return None;
    }

    let delta_p = p1 - p0;
    let dt_cont = delta_p.dot(v_mean) / v_sq;
    if !dt_cont.is_finite() || dt_cont <= 0.0 || dt_cont > 0.5 {
        return None;
    }

    let k = (dt_cont * 120.0).round().max(1.0) as u32;
    let effective_seconds = k as f32 / 120.0;
    let scale = effective_seconds / nominal_dt;

    Some(OfflineIntervalEstimate {
        effective_seconds,
        effective_ticks: k,
        scale,
    })
}

/// Offline aerial controls for the interval starting at `index`. The interval lies inside a span
/// between two fresh angular packets of the same car actor lifetime; one constant control solved
/// over the whole span is applied to every interval inside it. Adjacent-frame spans reproduce the
/// original one-frame lookahead. Uses a packet from after `index`, so it is offline reconstruction.
fn span_lookahead_air_controls(
    observations: &ObservedReplay,
    index: usize,
    car: &observations::Car,
    min_z: f32,
    options: &ConvertOptions,
) -> Option<AirControls> {
    let ang0 = car.body.angular_velocity_replay_units.as_ref()?;
    let rot0 = car.body.rotation_xyzw.as_ref()?;
    let start = ang0.frame;
    if start > index
        || rot0.frame != start
        || index - start >= options.air_lookahead_max_frames
        || !car
            .body
            .position
            .as_ref()
            .is_some_and(|p| p.frame == start && p.value[2] > min_z)
    {
        return None;
    }
    let same_car = |candidate: &&observations::Car| {
        candidate.actor_id == car.actor_id
            && candidate.actor_created_frame == car.actor_created_frame
            && candidate.player_key == car.player_key
            && candidate.player_link_active == car.player_link_active
    };
    let active = |frame: &observations::Frame| {
        frame
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active")
    };
    let start_frame = observations.frames.get(start)?;
    if !active(start_frame)
        || !observations
            .frames
            .get(start)?
            .cars
            .iter()
            .any(|c| same_car(&c))
    {
        return None;
    }
    for earlier in start + 1..=index {
        let candidate = observations
            .frames
            .get(earlier)?
            .cars
            .iter()
            .find(same_car)?;
        if candidate
            .inputs
            .dodge_active_raw
            .as_ref()
            .is_some_and(|d| d.frame == earlier && d.value % 2 == 1)
        {
            return None;
        }
    }
    let last = (start + options.air_lookahead_max_frames).min(observations.frames.len() - 1);
    let mut end = None;
    for candidate_index in index + 1..=last {
        let candidate_frame = &observations.frames[candidate_index];
        if !active(candidate_frame) {
            return None;
        }
        let Some(candidate) = candidate_frame.cars.iter().find(same_car) else {
            return None;
        };
        if candidate
            .inputs
            .dodge_active_raw
            .as_ref()
            .is_some_and(|d| d.frame == candidate_index && d.value % 2 == 1)
        {
            return None;
        }
        if candidate
            .body
            .angular_velocity_replay_units
            .as_ref()
            .is_some_and(|a| a.frame == candidate_index)
        {
            end = Some((candidate_index, candidate));
            break;
        }
    }
    let (end_index, end_car) = end?;
    if let Some(withheld) = options.withheld_frames.as_ref() {
        if (start + 1..end_index).any(|i| withheld.get(i).copied().unwrap_or(false)) {
            return None;
        }
    }
    let ang1 = end_car.body.angular_velocity_replay_units.as_ref()?;
    if !end_car
        .body
        .position
        .as_ref()
        .is_some_and(|p| p.frame == end_index && p.value[2] > min_z)
    {
        return None;
    }
    let dt = observations.frames[end_index].time - start_frame.time;
    if !(dt > 0.0 && dt <= options.air_lookahead_max_seconds) {
        return None;
    }
    let q0 = quaternion(rot0.value)?;
    if options.air_lookahead_refine_iterations == 0 {
        return Some(solve_inverse_air_controls(
            Mat3A::from_quat(q0),
            vec3(ang0.value) * 0.01,
            vec3(ang1.value) * 0.01,
            dt,
        ));
    }
    Some(solve_span_air_controls(
        Mat3A::from_quat(q0),
        vec3(ang0.value) * 0.01,
        vec3(ang1.value) * 0.01,
        (dt * 120.0).round().max(1.0) as u32,
        options.air_lookahead_refine_iterations,
    ))
}

/// Conditional-median persistence of a fitted aerial control, measured on the 60 train replays by
/// `calibrate_air_control_persistence` (416,600 fitted spans): the median later fitted control,
/// aligned with the earlier control's sign and expressed per unit of the earlier magnitude.
/// Indexed `[axis][lag band][|u| bin]` with axes pitch, yaw, roll; lag bands 0.033-0.083 s (also
/// anything shorter), 0.083-0.133 s and 0.133-0.200 s between span midpoints; |u| bins
/// [0.1,0.3), [0.3,0.5), [0.5,0.7), [0.7,0.9), [0.9,1]. Errors are judged by quantiles of absolute
/// error, for which the optimal point prediction of an uncertain input is its conditional median.
/// Roll ratios near 1 for |u| >= 0.5 reflect fits at the 5.5 rad/s angular speed cap: the fitted
/// roll there is the minimum that balances RocketSim's roll damping (about 0.69) and it persists.
const AIR_CONTROL_MEDIAN_RATIO: [[[f32; 5]; 3]; 3] = [
    [
        [0.210, 0.547, 0.758, 0.535, 0.464],
        [0.119, 0.234, 0.524, 0.397, 0.268],
        [0.014, 0.020, 0.130, 0.043, 0.019],
    ],
    [
        [0.407, 0.676, 0.627, 0.562, 0.508],
        [0.282, 0.412, 0.407, 0.438, 0.325],
        [0.121, 0.109, 0.092, 0.093, 0.051],
    ],
    [
        [0.087, 0.547, 1.031, 0.896, 0.705],
        [0.162, 0.425, 1.022, 0.892, 0.699],
        [0.125, 0.272, 1.004, 0.852, 0.642],
    ],
];

/// Median persistence ratio for a fitted control on `axis` (0 pitch, 1 yaw, 2 roll) with the
/// given `magnitude`, `lag` seconds after its span midpoint. Below the calibrated magnitude range
/// (fit noise) or beyond 0.2 s there is no evidence, so nothing persists.
fn air_control_median_ratio(axis: usize, lag: f32, magnitude: f32) -> f32 {
    if !lag.is_finite() || lag < 0.0 || lag >= 0.2 || !(0.1..=1.0 + 1e-3).contains(&magnitude) {
        return 0.0;
    }
    let band = if lag < 2.5 / 30.0 {
        0
    } else if lag < 4.0 / 30.0 {
        1
    } else {
        2
    };
    let bin = [0.3, 0.5, 0.7, 0.9]
        .iter()
        .position(|&edge| magnitude < edge)
        .unwrap_or(4);
    AIR_CONTROL_MEDIAN_RATIO[axis][band][bin]
}

/// Causal aerial controls for the interval starting at `index`: the constant control that carried
/// the car from its second-latest to its latest fresh angular packet, both at or before `index`.
/// Nothing after `index` is read, so it is valid for prediction across withheld packets.
fn past_persisted_air_controls(
    observations: &ObservedReplay,
    index: usize,
    car: &observations::Car,
    min_z: f32,
    options: &ConvertOptions,
) -> Option<(AirControls, f32)> {
    let ang1 = car.body.angular_velocity_replay_units.as_ref()?;
    let end = ang1.frame;
    if end == 0
        || end > index
        || !car
            .body
            .position
            .as_ref()
            .is_some_and(|p| p.frame == end && p.value[2] > min_z)
    {
        return None;
    }
    let end_frame = observations.frames.get(end)?;
    let elapsed = observations.frames.get(index)?.time - end_frame.time;
    if !options.air_persist_calibrated
        && !(0.0..=options.air_persist_max_seconds).contains(&elapsed)
    {
        return None;
    }
    let same_car = |candidate: &&observations::Car| {
        candidate.actor_id == car.actor_id
            && candidate.actor_created_frame == car.actor_created_frame
            && candidate.player_key == car.player_key
            && candidate.player_link_active == car.player_link_active
    };
    let active = |frame: &observations::Frame| {
        frame
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active")
    };
    let before = observations
        .frames
        .get(end - 1)?
        .cars
        .iter()
        .find(same_car)?;
    let ang0 = before.body.angular_velocity_replay_units.as_ref()?;
    let rot0 = before.body.rotation_xyzw.as_ref()?;
    let start = ang0.frame;
    if start >= end
        || rot0.frame != start
        || !before
            .body
            .position
            .as_ref()
            .is_some_and(|p| p.frame == start && p.value[2] > min_z)
    {
        return None;
    }
    let dt = end_frame.time - observations.frames.get(start)?.time;
    if !(dt > 0.0 && dt <= options.air_lookahead_max_seconds) {
        return None;
    }
    for frame_index in start..=end {
        let frame = &observations.frames[frame_index];
        let candidate = frame.cars.iter().find(same_car)?;
        if !active(frame)
            || (frame_index > start
                && candidate
                    .inputs
                    .dodge_active_raw
                    .as_ref()
                    .is_some_and(|d| d.frame == frame_index && d.value % 2 == 1))
        {
            return None;
        }
    }
    let q0 = quaternion(rot0.value)?;
    let solved = solve_span_air_controls(
        Mat3A::from_quat(q0),
        vec3(ang0.value) * 0.01,
        vec3(ang1.value) * 0.01,
        (dt * 120.0).round().max(1.0) as u32,
        options.air_lookahead_refine_iterations,
    );
    let span_mid = 0.5 * (observations.frames.get(start)?.time + end_frame.time);
    let interval_end = observations
        .frames
        .get(index + 1)
        .map_or(observations.frames.get(index)?.time, |frame| frame.time);
    let interval_mid = 0.5 * (observations.frames.get(index)?.time + interval_end);
    let lag = interval_mid - span_mid;
    if options.air_persist_calibrated {
        return Some((solved, lag));
    }
    if solved.pitch.abs().max(solved.roll.abs()) < options.air_persist_min_control {
        return None;
    }
    let speed_drop = (vec3(ang0.value) * 0.01).length() - (vec3(ang1.value) * 0.01).length();
    if speed_drop > options.air_persist_max_speed_drop {
        return None;
    }
    let gain = options.air_persist_gain;
    Some((
        AirControls {
            pitch: solved.pitch * gain,
            yaw: solved.yaw * gain,
            roll: solved.roll * gain,
        },
        lag,
    ))
}

/// An intentionally narrow causal ablation. The current replay angular packet is not read.
fn low_air_angular_hold(
    observations: &ObservedReplay,
    index: usize,
    car: &observations::Car,
    predicted: &CarState,
    slot: usize,
    events: &[SimEvent],
    min_angular_speed: f32,
) -> Option<Vec3A> {
    let frame = observations.frames.get(index)?;
    let previous = observations.frames.get(index.checked_sub(1)?)?;
    let prior = previous
        .cars
        .iter()
        .find(|c| c.actor_id == car.actor_id && c.actor_created_frame == car.actor_created_frame)?;
    let pos = prior.body.position.as_ref()?;
    let angular = prior.body.angular_velocity_replay_units.as_ref()?;
    let held = vec3(angular.value) * 0.01;
    if !(50.0..=100.0).contains(&pos.value[2])
        || !(50.0..=100.0).contains(&predicted.phys.pos.z)
        || !held.is_finite()
        || held.length() < min_angular_speed
        || predicted.is_on_ground
        || predicted.wheels_with_contact.iter().any(|&contact| contact)
        || predicted.world_contact_normal.is_some()
        || frame.time - observations.frames.get(pos.frame)?.time > 0.15
        || frame.time - observations.frames.get(angular.frame)?.time > 0.15
    {
        return None;
    }
    if events.iter().any(|event| match event.event {
        ArenaEvent::CarHitWorld(v) => v.car_idx == slot,
        ArenaEvent::CarHitBall(v) => v.car_idx == slot,
        ArenaEvent::CarHitCar(v) => v.bumper_car_idx == slot || v.victim_car_idx == slot,
        _ => false,
    }) {
        return None;
    }
    for earlier in (car.actor_created_frame..=index).rev() {
        let candidate = &observations.frames[earlier];
        if frame.time - candidate.time > 0.15 {
            break;
        }
        if let Some(c) = candidate.cars.iter().find(|c| {
            c.actor_id == car.actor_id && c.actor_created_frame == car.actor_created_frame
        }) {
            let odd = |v: &Option<Value<u8>>| {
                v.as_ref()
                    .is_some_and(|v| v.frame == earlier && v.value % 2 == 1)
            };
            if odd(&c.inputs.jump_active_raw)
                || odd(&c.inputs.double_jump_active_raw)
                || odd(&c.inputs.dodge_active_raw)
            {
                return None;
            }
        }
    }
    Some(held)
}

/// Feedback is computed at the beginning of the next interval, so RocketSim
/// integrates orientation and angular velocity under the same controls.
fn low_air_feedback_controls(
    observations: &ObservedReplay,
    index: usize,
    car: &observations::Car,
    state: &CarState,
    slot: usize,
    events: &[SimEvent],
) -> Option<AirControls> {
    let frame = observations.frames.get(index)?;
    let previous = observations.frames.get(index.checked_sub(1)?)?;
    previous
        .cars
        .iter()
        .find(|c| c.actor_id == car.actor_id && c.actor_created_frame == car.actor_created_frame)?;
    let position = car.body.position.as_ref()?;
    let angular = car.body.angular_velocity_replay_units.as_ref()?;
    let age = |source: usize| Some(frame.time - observations.frames.get(source)?.time);
    if !(50.0..=100.0).contains(&position.value[2])
        || !(50.0..=100.0).contains(&state.phys.pos.z)
        || !(0.0..=0.15).contains(&age(position.frame)?)
        || !(0.0..=0.15).contains(&age(angular.frame)?)
        || state.is_on_ground
        || state.wheels_with_contact.iter().any(|&contact| contact)
        || state.world_contact_normal.is_some()
        || state.is_flipping
        || state.is_auto_flipping
        || events.iter().any(|event| match event.event {
            ArenaEvent::CarHitWorld(v) => v.car_idx == slot,
            ArenaEvent::CarHitBall(v) => v.car_idx == slot,
            ArenaEvent::CarHitCar(v) => v.bumper_car_idx == slot || v.victim_car_idx == slot,
            _ => false,
        })
    {
        return None;
    }
    for earlier in (car.actor_created_frame..=index).rev() {
        let candidate = &observations.frames[earlier];
        if frame.time - candidate.time > 0.15 {
            break;
        }
        if let Some(c) = candidate.cars.iter().find(|c| {
            c.actor_id == car.actor_id && c.actor_created_frame == car.actor_created_frame
        }) {
            let odd = |v: &Option<Value<u8>>| {
                v.as_ref()
                    .is_some_and(|v| v.frame == earlier && v.value % 2 == 1)
            };
            if odd(&c.inputs.jump_active_raw)
                || odd(&c.inputs.double_jump_active_raw)
                || odd(&c.inputs.dodge_active_raw)
            {
                return None;
            }
        }
    }
    let target = vec3(angular.value) * 0.01;
    if !target.is_finite() || !state.phys.ang_vel.is_finite() {
        return None;
    }
    let delta = target - state.phys.ang_vel;
    let pitch_error = delta.dot(-state.phys.rot_mat.y_axis);
    let roll_error = delta.dot(-state.phys.rot_mat.x_axis);
    if pitch_error.hypot(roll_error) < 0.75 {
        return None;
    }
    Some(solve_inverse_air_controls(
        state.phys.rot_mat,
        state.phys.ang_vel,
        target,
        4.0 / 120.0,
    ))
}

fn position_residual(
    index: usize,
    actor_id: Option<i32>,
    body: &Body,
    previous: Option<&Body>,
    predicted: &PhysState,
    is_on_ground: Option<bool>,
    frames: &[observations::Frame],
) -> Option<PositionResidual> {
    let actual = body
        .position
        .as_ref()
        .filter(|value| value.frame == index)?;
    let previous_pos = previous?.position.as_ref()?;
    if previous_pos.frame >= index {
        return None;
    }
    let dt = frames[index].time - frames[previous_pos.frame].time;
    if !dt.is_finite() || dt <= 0.0 || dt > 0.5 {
        return None;
    }
    let recent = |previous_frame: usize| {
        previous_frame < index && {
            let gap = frames[index].time - frames[previous_frame].time;
            gap.is_finite() && gap > 0.0 && gap <= 0.5
        }
    };
    let actual_pos = vec3(actual.value);
    let previous_pos_val = vec3(previous_pos.value);
    let linear_extrapolation_error_uu = previous
        .and_then(|body| body.linear_velocity.as_ref())
        .filter(|velocity| recent(velocity.frame))
        .map(|velocity| (previous_pos_val + vec3(velocity.value) * dt - actual_pos).length());

    let (simulated_velocity_error_uu_per_sec, hold_velocity_error_uu_per_sec) =
        if let (Some(actual_vel), Some(prev_vel)) = (
            body.linear_velocity.as_ref().filter(|v| v.frame == index),
            previous.and_then(|b| b.linear_velocity.as_ref()),
        ) {
            if recent(prev_vel.frame) {
                let actual_v = vec3(actual_vel.value);
                let prev_v = vec3(prev_vel.value);
                (
                    Some((predicted.vel - actual_v).length()),
                    Some((prev_v - actual_v).length()),
                )
            } else {
                (None, None)
            }
        } else {
            (None, None)
        };

    let (simulated_rotation_error_degrees, hold_rotation_error_degrees) =
        if let (Some(actual_rot), Some(prev_rot)) = (
            body.rotation_xyzw.as_ref().filter(|v| v.frame == index),
            previous.and_then(|b| b.rotation_xyzw.as_ref()),
        ) {
            if recent(prev_rot.frame) {
                if let (Some(q_act), Some(q_prev)) =
                    (quaternion(actual_rot.value), quaternion(prev_rot.value))
                {
                    let mat_act = Mat3A::from_quat(q_act);
                    let mat_prev = Mat3A::from_quat(q_prev);
                    (
                        Some(rotation_error_degrees(predicted.rot_mat, mat_act)),
                        Some(rotation_error_degrees(mat_prev, mat_act)),
                    )
                } else {
                    (None, None)
                }
            } else {
                (None, None)
            }
        } else {
            (None, None)
        };

    let (simulated_angular_velocity_error_rad_per_sec, hold_angular_velocity_error_rad_per_sec) =
        if let (Some(actual_ang), Some(prev_ang)) = (
            body.angular_velocity_replay_units
                .as_ref()
                .filter(|v| v.frame == index),
            previous.and_then(|b| b.angular_velocity_replay_units.as_ref()),
        ) {
            if recent(prev_ang.frame) {
                let actual_w = vec3(actual_ang.value) * 0.01;
                let prev_w = vec3(prev_ang.value) * 0.01;
                (
                    Some((predicted.ang_vel - actual_w).length()),
                    Some((prev_w - actual_w).length()),
                )
            } else {
                (None, None)
            }
        } else {
            (None, None)
        };

    let (offline_interval, offline_projection_fit_error_uu) = if actor_id.is_some() {
        if let (Some(actual_vel), Some(prev_vel)) = (
            body.linear_velocity.as_ref().filter(|v| v.frame == index),
            previous.and_then(|b| b.linear_velocity.as_ref()),
        ) {
            if prev_vel.frame == previous_pos.frame {
                let est = estimate_car_packet_interval(
                    previous_pos.value,
                    actual.value,
                    prev_vel.value,
                    actual_vel.value,
                    dt,
                );
                let err = est.map(|e| {
                    (previous_pos_val + vec3(prev_vel.value) * e.effective_seconds - actual_pos)
                        .length()
                });
                (est, err)
            } else {
                (None, None)
            }
        } else {
            (None, None)
        }
    } else {
        (None, None)
    };

    Some(PositionResidual {
        frame: index,
        actor_id,
        seconds_since_previous_position: dt,
        simulated_error_uu: (predicted.pos - actual_pos).length(),
        simulated_error_vector_uu: (predicted.pos - actual_pos).to_array(),
        previous_linear_velocity_uu_per_second: previous
            .and_then(|body| body.linear_velocity.as_ref())
            .filter(|velocity| recent(velocity.frame))
            .map(|velocity| velocity.value),
        hold_error_uu: (previous_pos_val - actual_pos).length(),
        linear_extrapolation_error_uu,
        simulated_velocity_error_uu_per_sec,
        hold_velocity_error_uu_per_sec,
        simulated_rotation_error_degrees,
        hold_rotation_error_degrees,
        simulated_angular_velocity_error_rad_per_sec,
        hold_angular_velocity_error_rad_per_sec,
        altitude_z: Some(actual.value[2]),
        is_on_ground,
        offline_interval,
        offline_projection_fit_error_uu,
    })
}

fn should_apply<T>(value: &Value<T>, index: usize, new_entity: bool) -> bool {
    new_entity || value.frame == index
}

fn apply_body(state: &mut PhysState, body: &Body, index: usize, new_entity: bool) -> bool {
    let mut applied = false;
    if let Some(value) = &body.position {
        if should_apply(value, index, new_entity) {
            state.pos = vec3(value.value);
            applied = true;
        }
    }
    if let Some(value) = &body.rotation_xyzw {
        if should_apply(value, index, new_entity) {
            let [x, y, z, w] = value.value;
            let quat = Quat::from_xyzw(x, y, z, w);
            if quat.is_finite() && quat.length_squared() > 0.5 {
                state.rot_mat = Mat3A::from_quat(quat.normalize());
                applied = true;
            }
        }
    }
    if let Some(value) = &body.linear_velocity {
        if should_apply(value, index, new_entity) {
            state.vel = vec3(value.value);
            applied = true;
        }
    }
    if let Some(value) = &body.angular_velocity_replay_units {
        if should_apply(value, index, new_entity) {
            state.ang_vel = vec3(value.value) * 0.01;
            applied = true;
        }
    }
    applied
}

fn controls_from_observation(car: &observations::Car, options: &ConvertOptions) -> CarControls {
    CarControls {
        throttle: car.inputs.throttle.as_ref().map_or(0.0, |v| v.value),
        steer: car.inputs.steer.as_ref().map_or(0.0, |v| v.value),
        handbrake: car.inputs.handbrake.as_ref().is_some_and(|v| v.value),
        boost: options.infer_boost_from_active
            && car
                .inputs
                .boost_active_raw
                .as_ref()
                .is_some_and(|v| v.value % 2 == 1),
        jump: options.infer_jump_from_active
            && car
                .inputs
                .jump_active_raw
                .as_ref()
                .is_some_and(|v| v.value % 2 == 1),
        ..CarControls::default()
    }
}

fn jump_impulse_unobserved(car: &observations::Car, frame: usize) -> bool {
    car.body
        .position
        .as_ref()
        .is_some_and(|position| position.value[2] < 50.0)
        && !car
            .body
            .linear_velocity
            .as_ref()
            .is_some_and(|velocity| velocity.frame == frame && velocity.value[2] > 150.0)
}

fn dodge_impulse_unobserved(car: &observations::Car, frame: usize, state: &CarState) -> bool {
    let airborne = !state.is_on_ground || state.phys.pos.z > 50.0;
    let velocity_fresh = car
        .body
        .linear_velocity
        .as_ref()
        .is_some_and(|velocity| velocity.frame == frame);
    airborne && !velocity_fresh
}

fn team(index: u8) -> Team {
    if index == 0 { Team::Blue } else { Team::Orange }
}

/// Body product IDs are from boxcars' TeamLoadout, not RocketSim's preset indices.
/// The embedded map is generated from the user's item catalog, the official
/// Rocket League hitbox roster, and reviewed name aliases. Unknown IDs retain
/// the Octane fallback.
fn hitbox_for_body_product(id: u32) -> Option<(&'static str, CarBodyConfig)> {
    static CATALOG: OnceLock<Vec<(u32, &'static str)>> = OnceLock::new();
    let catalog = CATALOG.get_or_init(|| {
        let mut rows = Vec::new();
        for line in include_str!("../data/body_hitboxes.tsv").lines().skip(1) {
            let mut columns = line.split('\t');
            let id = columns
                .next()
                .unwrap()
                .parse::<u32>()
                .expect("body product ID");
            let _name = columns.next().expect("body product name");
            let hitbox = columns.next().expect("body hitbox");
            if hitbox != "unmapped" {
                rows.push((id, hitbox));
            }
        }
        assert!(rows.windows(2).all(|pair| pair[0].0 < pair[1].0));
        rows
    });
    let index = catalog.binary_search_by_key(&id, |entry| entry.0).ok()?;
    Some(match catalog[index].1 {
        "octane" => ("octane", CarBodyConfig::OCTANE),
        "breakout" => ("breakout", CarBodyConfig::BREAKOUT),
        "dominus" => ("dominus", CarBodyConfig::DOMINUS),
        "hybrid" => ("hybrid", CarBodyConfig::HYBRID),
        "merc" => ("merc", CarBodyConfig::MERC),
        "plank" => ("plank", CarBodyConfig::PLANK),
        "psyclops" => ("psyclops", CarBodyConfig::PSYCLOPS),
        unknown => panic!("unsupported body hitbox in catalog: {unknown}"),
    })
}

#[derive(Debug, Clone, Default)]
pub struct PacketLags {
    /// Ball packet lag in ticks behind each frame time, when inferred.
    pub ball: Vec<Option<f32>>,
    /// Median lag of the cars with a fresh packet in each frame, when inferred.
    pub cars: Vec<Option<f32>>,
    /// Lag of one car's own packet, keyed by (actor id, creation frame, frame).
    pub car_actor: HashMap<(i32, usize, usize), f32>,
}

/// One fresh packet of a chained object: frame index, position, and velocity.
struct ChainPacket {
    frame: usize,
    pos: [f32; 3],
    vel: [f32; 3],
}

/// Ticks of physical time between two packets, from the displacement along their mean velocity.
fn implied_interval_ticks(a: &ChainPacket, b: &ChainPacket) -> Option<f32> {
    let mean = [
        0.5 * (a.vel[0] + b.vel[0]),
        0.5 * (a.vel[1] + b.vel[1]),
        0.5 * (a.vel[2] + b.vel[2]),
    ];
    let speed_sq = mean[0] * mean[0] + mean[1] * mean[1] + mean[2] * mean[2];
    if speed_sq < 1.0 {
        return None;
    }
    let dot = (0..3).map(|i| (b.pos[i] - a.pos[i]) * mean[i]).sum::<f32>();
    let ticks = dot / speed_sq * 120.0;
    ticks.is_finite().then_some(ticks)
}

/// Assigns lags to one run of chained packets. Each packet was generated inside its frame window
/// `(previous frame time, frame time]`, so its lag lies in `[0, window)`. The chain fixes lag
/// differences; the unknown constant is centered inside the feasible interval.
fn finish_lag_run(run: &[(usize, f32, f32)], lo: f32, hi: f32, mut assign: impl FnMut(usize, f32)) {
    if run.len() < 2 || lo > hi {
        return;
    }
    let offset = 0.5 * (lo + hi);
    for &(frame, u, window) in run {
        assign(frame, (offset + u).clamp(0.0, (window - 1e-3).max(0.0)));
    }
}

/// Exact whole-tick chains. Elapsed ticks between chained packets are snapped to integers (pairs
/// more than 0.25 tick from one are rejected), so each packet's physical tick is `S = S0 + K` with
/// integer `K`. A packet was generated no later than its frame time and no earlier than the
/// previous frame's time, so on the integer timeline `tl(previous) <= S <= tl(frame)`; a run ends at
/// an unreliable pair or when no integer `S0` satisfies these bounds for every packet (a mistaken
/// interval shows up this way). Within the feasible starts, `S0` is the one that violates the
/// real-time window `0 <= T - S <= window` least (ties go to the middle). The assigned lag is
/// `tl(frame) - S`, an integer.
fn chain_packet_lags_exact(
    observations: &ObservedReplay,
    packets: &[ChainPacket],
    valid: impl Fn(&ChainPacket, &ChainPacket) -> bool,
    mut assign: impl FnMut(usize, f32),
) {
    let frames = &observations.frames;
    let first_time = f64::from(frames.first().map_or(0.0, |frame| frame.time));
    let real_tick = |frame: usize| (f64::from(frames[frame].time) - first_time) * 120.0;
    // The first frame has no previous frame, so its window is its own nominal period.
    let window = |frame: usize| {
        if frame == 0 {
            f64::from(frames[0].delta * 120.0).max(1.0)
        } else {
            real_tick(frame) - real_tick(frame - 1)
        }
    };
    let timeline = |frame: usize| real_tick(frame).round() as i64;
    // Integer bounds on S0 from one packet at cumulative interval K.
    let bounds = |frame: usize, k: i64| {
        let earliest = if frame == 0 {
            timeline(0) - window(0).round() as i64
        } else {
            timeline(frame - 1)
        };
        (earliest - k, timeline(frame) - k)
    };
    // (frame, K): physical tick relative to the first packet of the run.
    let mut run: Vec<(usize, i64)> = Vec::new();
    let (mut lo, mut hi) = (i64::MIN, i64::MAX);
    let finish = |run: &[(usize, i64)], lo: i64, hi: i64, assign: &mut dyn FnMut(usize, f32)| {
        if run.len() < 2 || lo > hi {
            return;
        }
        let violation = |start: i64| -> f64 {
            run.iter()
                .map(|&(f, k)| {
                    let lag = real_tick(f) - (start + k) as f64;
                    (-lag).max(0.0) + (lag - window(f)).max(0.0)
                })
                .sum()
        };
        let scores: Vec<(i64, f64)> = (lo..=hi).map(|s| (s, violation(s))).collect();
        let best = scores.iter().map(|&(_, v)| v).fold(f64::INFINITY, f64::min);
        let tied: Vec<i64> = scores
            .iter()
            .filter(|&&(_, v)| v <= best + 1e-9)
            .map(|&(s, _)| s)
            .collect();
        let start = tied[tied.len() / 2];
        for &(frame, k) in run {
            assign(frame, (timeline(frame) - (start + k)).max(0) as f32);
        }
    };
    for pair in packets.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        let interval = if valid(a, b) {
            implied_interval_ticks(a, b).and_then(|interval| {
                let snapped = interval.round();
                ((interval - snapped).abs() <= 0.25).then_some(snapped as i64)
            })
        } else {
            None
        };
        let Some(interval) = interval else {
            finish(&run, lo, hi, &mut assign);
            run.clear();
            (lo, hi) = (i64::MIN, i64::MAX);
            continue;
        };
        if run.is_empty() {
            let (l, h) = bounds(a.frame, 0);
            run.push((a.frame, 0));
            (lo, hi) = (l, h);
        }
        let k_next = run.last().map_or(0, |entry| entry.1) + interval;
        let (l, h) = bounds(b.frame, k_next);
        let (new_lo, new_hi) = (lo.max(l), hi.min(h));
        if new_lo <= new_hi {
            run.push((b.frame, k_next));
            (lo, hi) = (new_lo, new_hi);
        } else {
            finish(&run, lo, hi, &mut assign);
            let (la, ha) = bounds(a.frame, 0);
            let (lb, hb) = bounds(b.frame, interval);
            let (start_lo, start_hi) = (la.max(lb), ha.min(hb));
            if start_lo <= start_hi {
                run = vec![(a.frame, 0), (b.frame, interval)];
                (lo, hi) = (start_lo, start_hi);
            } else {
                run.clear();
                (lo, hi) = (i64::MIN, i64::MAX);
            }
        }
    }
    finish(&run, lo, hi, &mut assign);
}

/// Walks one chain of fresh packets. `valid(prev, next)` decides whether a pair's implied interval
/// is trustworthy; an invalid pair or an infeasible window ends the run.
fn chain_packet_lags(
    observations: &ObservedReplay,
    packets: &[ChainPacket],
    exact: bool,
    valid: impl Fn(&ChainPacket, &ChainPacket) -> bool,
    mut assign: impl FnMut(usize, f32),
) {
    if exact {
        return chain_packet_lags_exact(observations, packets, valid, assign);
    }
    let frame_window = |frame: usize| -> f32 {
        let previous = frame.saturating_sub(1);
        ((observations.frames[frame].time - observations.frames[previous].time) * 120.0).max(0.0)
    };
    let mut run: Vec<(usize, f32, f32)> = Vec::new();
    let (mut lo, mut hi) = (f32::NEG_INFINITY, f32::INFINITY);
    for pair in packets.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        let interval = if valid(a, b) {
            implied_interval_ticks(a, b)
        } else {
            None
        };
        let Some(interval) = interval else {
            finish_lag_run(&run, lo, hi, &mut assign);
            run.clear();
            (lo, hi) = (f32::NEG_INFINITY, f32::INFINITY);
            continue;
        };
        if run.is_empty() {
            run.push((a.frame, 0.0, frame_window(a.frame)));
            (lo, hi) = (-0.0, frame_window(a.frame));
        }
        let nominal =
            (observations.frames[b.frame].time - observations.frames[a.frame].time) * 120.0;
        let u_previous = run.last().map_or(0.0, |entry| entry.1);
        let u = u_previous + nominal - interval;
        let window = frame_window(b.frame);
        let (new_lo, new_hi) = (lo.max(-u), hi.min(window - u));
        if new_lo <= new_hi {
            run.push((b.frame, u, window));
            (lo, hi) = (new_lo, new_hi);
        } else {
            finish_lag_run(&run, lo, hi, &mut assign);
            let window_a = frame_window(a.frame);
            run = vec![(a.frame, 0.0, window_a)];
            let u = nominal - interval;
            let (start_lo, start_hi) = ((-0.0f32).max(-u), window_a.min(window - u));
            if start_lo <= start_hi {
                run.push((b.frame, u, window));
                (lo, hi) = (start_lo, start_hi);
            } else {
                run.clear();
                (lo, hi) = (f32::NEG_INFINITY, f32::INFINITY);
            }
        }
    }
    finish_lag_run(&run, lo, hi, &mut assign);
}

/// Offline inference of packet lags from chained ball and car motion. Uses packets after a frame,
/// so it is reconstruction, not prediction. Chains never bridge a withheld frame.
pub fn infer_packet_lags(observations: &ObservedReplay, options: &ConvertOptions) -> PacketLags {
    let frames = &observations.frames;
    let mut lags = PacketLags {
        ball: vec![None; frames.len()],
        cars: vec![None; frames.len()],
        car_actor: HashMap::new(),
    };
    let active = |frame: usize| {
        frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active")
    };
    let withheld = |frame: usize| {
        options
            .withheld_frames
            .as_ref()
            .is_some_and(|w| w.get(frame).copied().unwrap_or(false))
    };
    let bridge_ok = |a: usize, b: usize| {
        b > a && (a..=b).all(&active) && (a + 1..b).all(|f| !withheld(f)) && !withheld(b)
    };
    let fresh = |body: &Body, frame: usize| -> Option<ChainPacket> {
        let position = body.position.as_ref().filter(|v| v.frame == frame)?;
        let velocity = body.linear_velocity.as_ref().filter(|v| v.frame == frame)?;
        Some(ChainPacket {
            frame,
            pos: position.value,
            vel: velocity.value,
        })
    };
    let norm = |v: [f32; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();

    // Ball: smooth motion between consecutive frames.
    let ball_packets: Vec<ChainPacket> = frames
        .iter()
        .filter_map(|frame| {
            frame
                .ball
                .as_ref()
                .and_then(|body| fresh(body, frame.index))
        })
        .collect();
    chain_packet_lags(
        observations,
        &ball_packets,
        options.exact_tick_lag_chains,
        |a, b| {
            // The implied interval is a displacement along the mean velocity: exact for constant
            // acceleration and biased only at second order in the turn angle, so smooth motion
            // (including rolling and low bounces) is usable; sharp direction or speed changes
            // (hits, bounces) are not.
            let (na, nb) = (norm(a.vel), norm(b.vel));
            let cosine = if na > 1.0 && nb > 1.0 {
                (0..3).map(|i| a.vel[i] * b.vel[i]).sum::<f32>() / (na * nb)
            } else {
                0.0
            };
            bridge_ok(a.frame, b.frame)
                && b.frame - a.frame <= 2
                && na.min(nb) > 300.0
                && cosine >= 0.97
                && (na - nb).abs() <= 0.25 * na.max(nb)
        },
        |frame, lag| lags.ball[frame] = Some(lag),
    );

    // Cars: fast, smooth motion between packets of one actor lifetime (dodges excluded).
    let mut chains: HashMap<(i32, usize), Vec<ChainPacket>> = HashMap::new();
    let mut dodge_frames: HashSet<(i32, usize, usize)> = HashSet::new();
    for frame in frames {
        for car in &frame.cars {
            if car
                .inputs
                .dodge_active_raw
                .as_ref()
                .is_some_and(|d| d.frame == frame.index && d.value % 2 == 1)
            {
                dodge_frames.insert((car.actor_id, car.actor_created_frame, frame.index));
            }
            if let Some(packet) = fresh(&car.body, frame.index) {
                chains
                    .entry((car.actor_id, car.actor_created_frame))
                    .or_default()
                    .push(packet);
            }
        }
    }
    let mut per_frame: Vec<Vec<f32>> = vec![Vec::new(); frames.len()];
    for ((actor, created), packets) in &chains {
        chain_packet_lags(
            observations,
            packets,
            options.exact_tick_lag_chains,
            |a, b| {
                let (na, nb) = (norm(a.vel), norm(b.vel));
                let cosine = if na > 1.0 && nb > 1.0 {
                    (0..3).map(|i| a.vel[i] * b.vel[i]).sum::<f32>() / (na * nb)
                } else {
                    0.0
                };
                bridge_ok(a.frame, b.frame)
                    && b.frame - a.frame <= 8
                    && na.min(nb) > 350.0
                    && cosine >= 0.95
                    && (na - nb).abs() <= 0.4 * na.max(nb)
                    && (a.frame..=b.frame).all(|f| !dodge_frames.contains(&(*actor, *created, f)))
            },
            |frame, lag| {
                per_frame[frame].push(lag);
                lags.car_actor.insert((*actor, *created, frame), lag);
            },
        );
    }
    for (frame, values) in per_frame.iter_mut().enumerate() {
        if !values.is_empty() {
            values.sort_by(|a, b| a.total_cmp(b));
            lags.cars[frame] = Some(values[values.len() / 2]);
        }
    }
    lags
}

/// RocketSim limits speeds at the start of each tick, so a state read after stepping can exceed
/// the limits that recorded server states obey. Apply them to the reported state.
fn limit_reported_velocities(arena: &mut Arena, car_count: usize) {
    const CAR_MAX_SPEED: f32 = 2300.0;
    const CAR_MAX_ANGULAR_SPEED: f32 = 5.5;
    const BALL_MAX_SPEED: f32 = 6000.0;
    const BALL_MAX_ANGULAR_SPEED: f32 = 6.0;
    let limit = |velocity: Vec3A, maximum: f32| {
        let speed = velocity.length();
        (speed > maximum).then(|| velocity * (maximum / speed))
    };
    for index in 0..car_count {
        let mut state = *arena.get_car_state(index);
        let linear = limit(state.phys.vel, CAR_MAX_SPEED);
        let angular = limit(state.phys.ang_vel, CAR_MAX_ANGULAR_SPEED);
        if linear.is_some() || angular.is_some() {
            state.phys.vel = linear.unwrap_or(state.phys.vel);
            state.phys.ang_vel = angular.unwrap_or(state.phys.ang_vel);
            arena.set_car_state(index, state);
        }
    }
    let mut ball = *arena.get_ball_state();
    let linear = limit(ball.phys.vel, BALL_MAX_SPEED);
    let angular = limit(ball.phys.ang_vel, BALL_MAX_ANGULAR_SPEED);
    if linear.is_some() || angular.is_some() {
        ball.phys.vel = linear.unwrap_or(ball.phys.vel);
        ball.phys.ang_vel = angular.unwrap_or(ball.phys.ang_vel);
        arena.set_ball_state(ball);
    }
}

/// Steps one tick. The pinned RocketSim computes the extra ball-car hit impulse but loses it (it
/// is added to an accumulator that is cleared before use), so when `apply_hit_impulse` is set the
/// reported `extra_hit_vel` of every `CarHitBall` event is added to the ball's velocity at the end
/// of the same tick, which is where the intended impulse takes effect.
pub fn step_tick_with_hit_impulse(arena: &mut Arena, apply_hit_impulse: bool) -> Vec<ArenaEvent> {
    let events: Vec<ArenaEvent> = arena.step_tick().to_vec();
    if apply_hit_impulse {
        let extra = events
            .iter()
            .filter_map(|event| match event {
                ArenaEvent::CarHitBall(hit) => Some(hit.extra_hit_vel),
                _ => None,
            })
            .fold(Vec3A::ZERO, |sum, vel| sum + vel);
        if extra != Vec3A::ZERO {
            let mut ball = *arena.get_ball_state();
            ball.phys.vel += extra;
            arena.set_ball_state(ball);
        }
    }
    events
}

fn step_ticks(arena: &mut Arena, ticks: u64, apply_hit_impulse: bool, events: &mut Vec<SimEvent>) {
    for _ in 0..ticks {
        let arena_tick = arena.tick_count() + 1;
        let tick_events = step_tick_with_hit_impulse(arena, apply_hit_impulse);
        events.extend(
            tick_events
                .into_iter()
                .map(|event| SimEvent { arena_tick, event }),
        );
    }
}

/// Fits the flip's pitch-cancel amount over the span from this fresh car packet to the next one.
/// Candidate cancels (opposite pitch input of 0, 0.25, ..., 1) are simulated in a scratch arena
/// from the current corrected state to the next packet's physical tick, and the one whose angular
/// velocity is closest to that packet wins. Uses a later packet, so it is offline reconstruction;
/// spans containing a withheld frame, an inactive frame, or a change of dodge counter are refused.
#[allow(clippy::too_many_arguments)]
fn fit_flip_cancel(
    observations: &ObservedReplay,
    options: &ConvertOptions,
    packet_lags: &Option<PacketLags>,
    first_time: f32,
    index: usize,
    car: &observations::Car,
    state: &CarState,
    base_controls: &CarControls,
    lag_a: u64,
    scratch: &mut Arena,
) -> Option<f32> {
    let frames = &observations.frames;
    let ang0 = car.body.angular_velocity_replay_units.as_ref()?;
    if ang0.frame != index {
        return None;
    }
    let counter = car.inputs.dodge_active_raw.as_ref()?.value;
    let timeline = |frame: usize| -> i64 {
        ((f64::from(frames[frame].time) - f64::from(first_time)) * 120.0).round() as i64
    };
    let active = |frame: usize| {
        frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active")
    };
    let withheld = |frame: usize| {
        options
            .withheld_frames
            .as_ref()
            .is_some_and(|w| w.get(frame).copied().unwrap_or(false))
    };
    if !active(index) {
        return None;
    }
    for candidate in index + 1..=(index + 8).min(frames.len() - 1) {
        if !active(candidate) || withheld(candidate) {
            return None;
        }
        let Some(other) = frames[candidate].cars.iter().find(|c| {
            c.actor_id == car.actor_id
                && c.actor_created_frame == car.actor_created_frame
                && c.player_key == car.player_key
        }) else {
            return None;
        };
        let Some(ang1) = other
            .body
            .angular_velocity_replay_units
            .as_ref()
            .filter(|a| a.frame == candidate)
        else {
            continue;
        };
        if other.inputs.dodge_active_raw.as_ref().map(|d| d.value) != Some(counter) {
            return None;
        }
        let lag_b = match packet_lags {
            Some(lags) => lags
                .car_actor
                .get(&(car.actor_id, car.actor_created_frame, candidate))
                .copied()
                .or(lags.cars[candidate])
                .map_or(
                    (timeline(candidate) - timeline(candidate - 1)).max(0) / 2,
                    |lag| lag.round().max(0.0) as i64,
                ),
            None => 0,
        };
        let ticks = (timeline(candidate) - lag_b) - (timeline(index) - lag_a as i64);
        if !(1..=40).contains(&ticks) {
            return None;
        }
        let target = vec3(ang1.value) * 0.01;
        let sign = state.flip_rel_torque.y.signum();
        let mut best: Option<(f32, f32)> = None;
        for step in 0..=4 {
            let cancel = step as f32 * 0.25;
            scratch.set_car_state(0, *state);
            let mut controls = *base_controls;
            controls.jump = false;
            controls.pitch = cancel * sign;
            scratch.set_car_controls(0, controls);
            for _ in 0..ticks {
                scratch.step_tick();
            }
            let mut end = *scratch.get_car_state(0);
            let speed = end.phys.ang_vel.length();
            if speed > 5.5 {
                end.phys.ang_vel *= 5.5 / speed;
            }
            let error = (end.phys.ang_vel - target).length();
            if best.is_none_or(|(_, e)| error < e - 1e-4) {
                best = Some((cancel, error));
            }
        }
        return best.map(|(cancel, _)| cancel);
    }
    None
}

/// Parse and convert a soccar replay. The returned frame and observation vectors align by index.
pub fn convert_bytes(
    bytes: &[u8],
    options: &ConvertOptions,
) -> Result<ConversionOutput, ConvertError> {
    let replay = parse_replay(bytes).map_err(ConvertError::Parse)?;
    let observed = observations::extract(&replay).ok_or(ConvertError::MissingNetworkFrames)?;
    let mut output = convert_observations(observed, options)?;
    output.source_sha256 = Some(format!("{:x}", Sha256::digest(bytes)));
    Ok(output)
}

/// Convert observations with RocketSim. The mesh directory is initialized once per process.
pub fn convert_observations(
    observations: ObservedReplay,
    options: &ConvertOptions,
) -> Result<ConversionOutput, ConvertError> {
    let mut frames = Vec::with_capacity(observations.frames.len());
    let mut position_residuals = Vec::new();
    let summary = convert_observations_with(&observations, options, |converted, _, residuals| {
        frames.push(converted.clone());
        position_residuals.extend_from_slice(residuals);
        Ok(())
    })?;
    Ok(ConversionOutput {
        source_sha256: None,
        options: options.clone(),
        observations,
        frames,
        position_residuals,
        car_slots: summary.car_slots,
        diagnostics: summary.diagnostics,
    })
}

/// Emit one state at a time. The observed replay remains in memory for lookahead and
/// provenance, but simulated snapshots and residuals are bounded to one frame.
pub fn convert_observations_with(
    observations: &ObservedReplay,
    options: &ConvertOptions,
    mut on_frame: impl FnMut(
        &ConvertedFrame,
        &observations::Frame,
        &[PositionResidual],
    ) -> io::Result<()>,
) -> Result<ConversionSummary, ConvertError> {
    if observations.header.game_type != "TAGame.Replay_Soccar_TA" {
        return Err(ConvertError::UnsupportedMode(
            observations.header.game_type.clone(),
        ));
    }
    rocketsim::init(Path::new(&options.collision_meshes), true).map_err(ConvertError::Init)?;
    let mut config = ArenaConfig::new(GameMode::Soccar);
    config.rng_seed = Some(options.seed);
    let mut arena = Arena::new_with_config(config);
    let mut slots: HashMap<String, usize> = HashMap::new();
    let mut car_slots = Vec::new();
    let mut actor_slots: HashMap<i32, (usize, usize)> = HashMap::new();
    let mut gated_jump_active: HashMap<(i32, usize), bool> = HashMap::new();
    let mut last_dodge_raw: HashMap<(i32, usize), u8> = HashMap::new();
    let mut pad_actor_to_index: HashMap<i32, usize> = HashMap::new();
    let mut last_pad_counter: HashMap<i32, u8> = HashMap::new();
    let mut diagnostics = Diagnostics::default();
    let first_time = observations.frames.first().map_or(0.0, |frame| frame.time);
    let packet_lags = options
        .infer_packet_lag
        .then(|| infer_packet_lags(observations, options));
    let mut previous_tick = 0;
    let mut previous_active = false;
    let mut ball_initialized = false;
    let mut flip_scratch = options.infer_flip_cancel.then(|| {
        let mut scratch_config = ArenaConfig::new(GameMode::Soccar);
        scratch_config.rng_seed = Some(options.seed);
        let mut scratch = Arena::new_with_config(scratch_config);
        scratch.add_car(Team::Blue, CarBodyConfig::OCTANE);
        scratch
    });
    let mut flip_cache: HashMap<(i32, usize, usize), Option<f32>> = HashMap::new();
    let mut flip_last: HashMap<(i32, usize), f32> = HashMap::new();

    for (frame_idx, frame) in observations.frames.iter().enumerate() {
        let mut frame_residuals = Vec::new();
        if !frame.time.is_finite() || frame.time < first_time {
            return Err(ConvertError::InvalidTime {
                frame: frame.index,
                time: frame.time,
            });
        }
        let timeline_tick = (((f64::from(frame.time) - f64::from(first_time)) * 120.0).round()
            as u64)
            .max(previous_tick);
        let gap = timeline_tick - previous_tick;
        let active = frame
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active");
        let mut events = Vec::new();
        let simulated = active && previous_active && gap > 0 && gap <= options.max_gap_ticks;
        previous_tick = timeline_tick;
        previous_active = active;

        if !simulated {
            diagnostics.skipped_timeline_ticks += gap;
        }
        // Without inference a packet is equally likely to have been generated anywhere in its
        // frame window (inferred lags are close to uniform), so half the window is the median.
        let default_lag = gap / 2;
        let lag_ticks = |lag: Option<f32>| -> u64 {
            lag.map_or(default_lag, |lag| lag.round().max(0.0) as u64)
                .min(gap)
        };
        if let (Some(lags), true) = (&packet_lags, simulated) {
            diagnostics.ball_lag_frames += usize::from(lags.ball[frame_idx].is_some());
            diagnostics.car_lag_frames += usize::from(lags.cars[frame_idx].is_some());
        }
        let primary = observations::primary_linked_cars(frame);
        diagnostics.shadowed_car_frames += frame
            .cars
            .iter()
            .filter(|car| car.player_key.is_some())
            .count()
            - primary.len();
        let frame_cars: Vec<&observations::Car> = primary
            .into_iter()
            .chain(frame.cars.iter().filter(|car| car.player_key.is_none()))
            .collect();
        let mut selected_slots = HashSet::new();
        let ball_lag = match (&packet_lags, simulated) {
            (Some(lags), true) => lag_ticks(lags.ball[frame_idx]),
            _ => 0,
        };
        // Each car's packet time is its own inferred lag, else the frame's median car lag.
        let car_lag = |car: &observations::Car| -> u64 {
            match (&packet_lags, simulated) {
                (Some(lags), true) => lag_ticks(
                    lags.car_actor
                        .get(&(car.actor_id, car.actor_created_frame, frame_idx))
                        .copied()
                        .or(lags.cars[frame_idx]),
                ),
                _ => 0,
            }
        };
        let mut applied_lags = Vec::new();
        if let (Some(lags), true) = (&packet_lags, simulated) {
            if frame
                .ball
                .as_ref()
                .and_then(|body| body.position.as_ref())
                .is_some_and(|p| p.frame == frame.index)
            {
                applied_lags.push(AppliedPacketLag {
                    actor_id: None,
                    ticks: ball_lag,
                    source: if lags.ball[frame_idx].is_some() {
                        "chain"
                    } else {
                        "default"
                    },
                });
            }
            for car in &frame_cars {
                if !car
                    .body
                    .position
                    .as_ref()
                    .is_some_and(|p| p.frame == frame.index)
                {
                    continue;
                }
                let own = lags.car_actor.contains_key(&(
                    car.actor_id,
                    car.actor_created_frame,
                    frame_idx,
                ));
                applied_lags.push(AppliedPacketLag {
                    actor_id: Some(car.actor_id),
                    ticks: car_lag(car),
                    source: if own {
                        "chain"
                    } else if lags.cars[frame_idx].is_some() {
                        "frame_median"
                    } else {
                        "default"
                    },
                });
            }
        }
        let mut phase_lags: Vec<u64> = std::iter::once(ball_lag)
            .chain(frame_cars.iter().map(|car| car_lag(car)))
            .collect();
        phase_lags.sort_unstable_by(|a, b| b.cmp(a));
        phase_lags.dedup();
        let mut remaining = if simulated { gap } else { 0 };
        for lag in phase_lags {
            // Advance to this group's packet time (`lag` ticks before the frame time).
            let stepped = remaining - lag.min(remaining);
            step_ticks(
                &mut arena,
                stepped,
                options.apply_hit_extra_impulse,
                &mut events,
            );
            if stepped > 0 && options.limit_reported_velocities {
                limit_reported_velocities(&mut arena, slots.len());
            }
            remaining = lag.min(remaining);
            if ball_lag == lag {
                if let Some(body) = &frame.ball {
                    if simulated && ball_initialized {
                        if let Some(residual) = position_residual(
                            frame.index,
                            None,
                            body,
                            observations
                                .frames
                                .get(frame.index.wrapping_sub(1))
                                .and_then(|f| f.ball.as_ref()),
                            &arena.get_ball_state().phys,
                            None,
                            &observations.frames,
                        ) {
                            frame_residuals.push(residual);
                        }
                    }
                    let mut ball = *arena.get_ball_state();
                    if apply_body(&mut ball.phys, body, frame.index, !ball_initialized) {
                        arena.set_ball_state(ball);
                    }
                    ball_initialized = true;
                }
            }
            for car in frame_cars.iter().copied() {
                if car_lag(car) != lag {
                    continue;
                }
                let slot = if let Some(key) = &car.player_key {
                    if let Some(slot) = slots.get(key).copied() {
                        actor_slots.insert(car.actor_id, (slot, car.actor_created_frame));
                        Some((slot, car.actor_created_frame == frame.index))
                    } else if let Some(team_idx) = car.team {
                        let body_product_id = car.body_product_id.as_ref().map(|v| v.value);
                        let known_hitbox = body_product_id
                            .filter(|_| options.use_loadout_hitboxes)
                            .and_then(hitbox_for_body_product);
                        let (hitbox, config) =
                            known_hitbox.unwrap_or(("octane", CarBodyConfig::OCTANE));
                        let slot = arena.add_car(team(team_idx), config);
                        slots.insert(key.clone(), slot);
                        car_slots.push(CarSlot {
                            slot,
                            player_key: key.clone(),
                            team: team_idx,
                            body_product_id,
                            hitbox: hitbox.to_owned(),
                        });
                        actor_slots.insert(car.actor_id, (slot, car.actor_created_frame));
                        diagnostics.default_hitbox_players += usize::from(known_hitbox.is_none());
                        Some((slot, true))
                    } else {
                        None
                    }
                } else {
                    actor_slots
                        .get(&car.actor_id)
                        .copied()
                        .filter(|(_, created_frame)| *created_frame == car.actor_created_frame)
                        .map(|(slot, _)| (slot, false))
                };
                let Some((slot, new_lifetime)) = slot else {
                    diagnostics.unlinked_car_frames += 1;
                    continue;
                };
                if car.player_key.is_none() && selected_slots.contains(&slot) {
                    diagnostics.shadowed_car_frames += 1;
                    continue;
                }
                selected_slots.insert(slot);
                if simulated && !new_lifetime {
                    let previous = observations
                        .frames
                        .get(frame.index.wrapping_sub(1))
                        .and_then(|f| {
                            f.cars
                                .iter()
                                .find(|previous| previous.actor_id == car.actor_id)
                        })
                        .map(|car| &car.body);
                    let mut car_state = *arena.get_car_state(slot);
                    if options.hold_low_air_angular {
                        if let Some(held) = low_air_angular_hold(
                            observations,
                            frame.index,
                            car,
                            &car_state,
                            slot,
                            &events,
                            if options.gate_low_air_angular_by_speed {
                                5.48
                            } else {
                                0.0
                            },
                        ) {
                            car_state.phys.ang_vel = held;
                            arena.set_car_state(slot, car_state);
                        }
                    }
                    if let Some(residual) = position_residual(
                        frame.index,
                        Some(car.actor_id),
                        &car.body,
                        previous,
                        &car_state.phys,
                        Some(car_state.is_on_ground),
                        &observations.frames,
                    ) {
                        frame_residuals.push(residual);
                    }
                }
                let mut state = if new_lifetime {
                    CarState::default()
                } else {
                    *arena.get_car_state(slot)
                };
                let mut dirty = apply_body(&mut state.phys, &car.body, frame.index, new_lifetime)
                    || new_lifetime;
                if car.player_link_active && state.is_demoed {
                    state.is_demoed = false;
                    state.demo_respawn_timer = 0.0;
                    diagnostics.active_pawn_demo_corrections += 1;
                    dirty = true;
                }
                if let Some(boost) = &car.boost {
                    if should_apply(boost, frame.index, new_lifetime) {
                        state.boost = boost.value;
                        dirty = true;
                    }
                }

                let mut dodge_jump_control = false;
                let mut dodge_pitch_control = 0.0;
                let mut dodge_yaw_control = 0.0;

                if options.infer_dodge_from_active {
                    let key = (car.actor_id, car.actor_created_frame);
                    let dodge_raw = car
                        .inputs
                        .dodge_active_raw
                        .as_ref()
                        .filter(|raw| raw.frame == frame.index)
                        .map(|raw| raw.value);
                    if let Some(raw) = dodge_raw {
                        let prev = last_dodge_raw.insert(key, raw);
                        let activated = match prev {
                            Some(prev_val) => prev_val % 2 == 0 && raw % 2 == 1,
                            None => raw % 2 == 1,
                        };
                        if activated {
                            if let Some(torque) = car
                                .inputs
                                .dodge_torque_replay_units
                                .as_ref()
                                .filter(|torque| torque.frame == frame.index)
                            {
                                let [tx, ty, _] = torque.value;
                                let pitch = -ty / 2.24;
                                let yaw = -tx / 2.60;
                                if (pitch * pitch + yaw * yaw).sqrt() > 0.01 {
                                    if !options.gate_dodge_on_observed_impulse
                                        || dodge_impulse_unobserved(car, frame.index, &state)
                                    {
                                        dodge_jump_control = true;
                                        dodge_pitch_control = pitch;
                                        dodge_yaw_control = yaw;
                                    } else if !state.is_on_ground || state.phys.pos.z > 50.0 {
                                        state.has_flipped = true;
                                        state.is_flipping = true;
                                        state.flip_rel_torque =
                                            Vec3A::new(tx / 2.60, ty / 2.24, 0.0);
                                        state.flip_time = 0.0;
                                        dirty = true;
                                    }
                                }
                            }
                        }
                    }
                }

                if dirty {
                    arena.set_car_state(slot, state);
                }
                let mut controls = controls_from_observation(car, options);
                if options.gate_jump_on_observed_impulse {
                    let key = (car.actor_id, car.actor_created_frame);
                    if let Some(raw) = car
                        .inputs
                        .jump_active_raw
                        .as_ref()
                        .filter(|raw| raw.frame == frame.index)
                    {
                        gated_jump_active.insert(
                            key,
                            raw.value % 2 == 1 && jump_impulse_unobserved(car, frame.index),
                        );
                    }
                    controls.jump &= gated_jump_active.get(&key).copied().unwrap_or(false);
                }
                let airborne = !state.is_on_ground || (new_lifetime && state.phys.pos.z > 50.0);
                let min_lookahead_z = if options.infer_transition_air_lookahead {
                    50.0
                } else {
                    100.0
                };
                let mut air_controls_applied = false;
                if options.infer_air_controls_from_lookahead
                    && airborne
                    && !dodge_jump_control
                    && active
                {
                    if let Some(solved) = span_lookahead_air_controls(
                        observations,
                        frame_idx,
                        car,
                        min_lookahead_z,
                        options,
                    ) {
                        controls.pitch = solved.pitch;
                        controls.yaw = solved.yaw;
                        controls.roll = solved.roll;
                        air_controls_applied = true;
                    }
                }

                if !air_controls_applied
                    && options.persist_past_air_controls
                    && airborne
                    && !dodge_jump_control
                    && active
                {
                    if let Some((solved, lag)) = past_persisted_air_controls(
                        observations,
                        frame_idx,
                        car,
                        min_lookahead_z,
                        options,
                    ) {
                        if options.air_persist_calibrated {
                            let keep = |axis: usize, value: f32| {
                                value * air_control_median_ratio(axis, lag, value.abs())
                            };
                            controls.pitch = keep(0, solved.pitch);
                            let steer_observed =
                                options.infer_air_steer_controls && car.inputs.steer.is_some();
                            if !steer_observed {
                                controls.yaw = keep(1, solved.yaw);
                                controls.roll = keep(2, solved.roll);
                            } else if options.infer_air_roll_from_handbrake && controls.handbrake {
                                controls.roll = controls.steer;
                                controls.yaw = keep(1, solved.yaw);
                            } else {
                                controls.yaw = controls.steer;
                                controls.roll = keep(2, solved.roll);
                            }
                        } else {
                            controls.pitch = solved.pitch;
                            controls.yaw = solved.yaw;
                            controls.roll = solved.roll;
                        }
                        air_controls_applied = true;
                    }
                }

                if !air_controls_applied
                    && options.compensate_transition_air_damping
                    && airborne
                    && !dodge_jump_control
                    && (50.0..=100.0).contains(&state.phys.pos.z)
                {
                    let angular = state.phys.ang_vel;
                    let solved = solve_inverse_air_controls(
                        state.phys.rot_mat,
                        angular,
                        angular,
                        1.0 / 120.0,
                    );
                    controls.pitch = solved.pitch;
                    controls.yaw = solved.yaw;
                    controls.roll = solved.roll;
                    air_controls_applied = true;
                }
                if !air_controls_applied
                    && options.feedback_low_air_angular
                    && active
                    && !new_lifetime
                {
                    if let Some(solved) = low_air_feedback_controls(
                        observations,
                        frame.index,
                        car,
                        &state,
                        slot,
                        &events,
                    ) {
                        controls.pitch = solved.pitch;
                        controls.roll = solved.roll;
                        controls.yaw = if options.infer_air_steer_controls {
                            controls.steer
                        } else {
                            0.0
                        };
                        air_controls_applied = true;
                    }
                }
                if options.infer_flip_cancel {
                    let key = (car.actor_id, car.actor_created_frame);
                    if airborne
                        && !dodge_jump_control
                        && state.is_flipping
                        && state.flip_rel_torque.y != 0.0
                    {
                        let sign = state.flip_rel_torque.y.signum();
                        let mut cancel = flip_last.get(&key).copied().unwrap_or(0.0);
                        let packet_frame = car
                            .body
                            .angular_velocity_replay_units
                            .as_ref()
                            .map(|value| value.frame);
                        if let Some(packet_frame) = packet_frame {
                            let cache_key = (car.actor_id, car.actor_created_frame, packet_frame);
                            if !flip_cache.contains_key(&cache_key) && packet_frame == frame_idx {
                                let mut base = controls;
                                if options.infer_air_steer_controls {
                                    if options.infer_air_roll_from_handbrake && controls.handbrake {
                                        base.roll = controls.steer;
                                    } else {
                                        base.yaw = controls.steer;
                                    }
                                }
                                let fitted = match flip_scratch.as_mut() {
                                    Some(scratch) if active && !new_lifetime => fit_flip_cancel(
                                        observations,
                                        options,
                                        &packet_lags,
                                        first_time,
                                        frame_idx,
                                        car,
                                        &state,
                                        &base,
                                        car_lag(car),
                                        scratch,
                                    ),
                                    _ => None,
                                };
                                flip_cache.insert(cache_key, fitted);
                            }
                            if let Some(Some(fitted)) = flip_cache.get(&cache_key) {
                                cancel = *fitted;
                                flip_last.insert(key, cancel);
                            }
                        }
                        controls.pitch = cancel * sign;
                    } else if !state.is_flipping {
                        flip_last.remove(&key);
                    }
                }
                if !air_controls_applied && options.infer_air_steer_controls && airborne {
                    if options.infer_air_roll_from_handbrake && controls.handbrake {
                        controls.roll = controls.steer;
                    } else {
                        controls.yaw = controls.steer;
                    }
                }
                if dodge_jump_control {
                    controls.jump = true;
                    controls.pitch = dodge_pitch_control;
                    controls.yaw = dodge_yaw_control;
                }
                arena.set_car_controls(slot, controls);
            }
        }
        step_ticks(
            &mut arena,
            remaining,
            options.apply_hit_extra_impulse,
            &mut events,
        );
        if remaining > 0 && options.limit_reported_velocities {
            limit_reported_velocities(&mut arena, slots.len());
        }
        if options.sync_boost_pad_pickups {
            for pickup in &frame.pad_pickups {
                let pad_idx = if let Some(&idx) = pad_actor_to_index.get(&pickup.pad_actor_id) {
                    Some(idx)
                } else if let Some(instigator_id) = pickup.instigator_car_id {
                    let car_pos = frame
                        .cars
                        .iter()
                        .find(|c| c.actor_id == instigator_id)
                        .and_then(|c| c.body.position.as_ref())
                        .filter(|p| {
                            p.frame <= frame.index
                                && frame.time - observations.frames[p.frame].time <= 0.1
                        })
                        .map(|p| vec3(p.value))
                        .or_else(|| {
                            actor_slots
                                .get(&instigator_id)
                                .map(|&(slot, _)| arena.get_car_state(slot).phys.pos)
                        });
                    if let Some(pos) = car_pos {
                        let mut best_pad = None;
                        let mut best_dist_sq = f32::INFINITY;
                        let mut second_dist_sq = f32::INFINITY;
                        for idx in 0..arena.num_boost_pads() {
                            let pad_pos = arena.get_boost_pad_config(idx).pos;
                            let d2 = (pad_pos.x - pos.x).powi(2) + (pad_pos.y - pos.y).powi(2);
                            if d2 < best_dist_sq {
                                second_dist_sq = best_dist_sq;
                                best_dist_sq = d2;
                                best_pad = Some(idx);
                            } else if d2 < second_dist_sq {
                                second_dist_sq = d2;
                            }
                        }
                        if best_dist_sq < 350.0 * 350.0
                            && second_dist_sq.sqrt() - best_dist_sq.sqrt() >= 100.0
                        {
                            if let Some(idx) = best_pad {
                                pad_actor_to_index.insert(pickup.pad_actor_id, idx);
                            }
                            best_pad
                        } else {
                            None
                        }
                    } else {
                        None
                    }
                } else {
                    None
                };

                let prev_counter = last_pad_counter.insert(pickup.pad_actor_id, pickup.picked_up);
                let changed = prev_counter != Some(pickup.picked_up);

                if changed {
                    if let Some(idx) = pad_idx {
                        if pickup.picked_up == 255 {
                            arena.set_boost_pad_state(idx, BoostPadState { cooldown: 0.0 });
                        } else if pickup.picked_up % 2 == 1 {
                            let max_cooldown = if arena.get_boost_pad_config(idx).is_big {
                                10.0
                            } else {
                                4.0
                            };
                            arena.set_boost_pad_state(
                                idx,
                                BoostPadState {
                                    cooldown: max_cooldown,
                                },
                            );
                        }
                    }
                }
            }
        }
        let converted = ConvertedFrame {
            replay_frame: frame.index,
            replay_time: frame.time,
            timeline_tick,
            state: arena.get_arena_state(),
            simulated_events: events,
            packet_lags: applied_lags,
        };
        on_frame(&converted, frame, &frame_residuals).map_err(ConvertError::Output)?;
    }
    Ok(ConversionSummary {
        car_slots,
        diagnostics,
    })
}

#[cfg(test)]
mod tests {
    use rocketsim::BallState;

    use super::*;
    use crate::observations::Source;

    #[test]
    fn stale_replay_fields_do_not_overwrite_simulated_physics() {
        let mut state = BallState::default().phys;
        state.pos = Vec3A::new(10.0, 20.0, 30.0);
        let body = Body {
            position: Some(Value {
                value: [1.0, 2.0, 3.0],
                frame: 3,
                source: Source::Replay,
            }),
            angular_velocity_replay_units: Some(Value {
                value: [0.0, 0.0, 100.0],
                frame: 3,
                source: Source::Replay,
            }),
            ..Body::default()
        };

        assert!(!apply_body(&mut state, &body, 4, false));
        assert_eq!(state.pos, Vec3A::new(10.0, 20.0, 30.0));
        assert!(apply_body(&mut state, &body, 3, false));
        assert_eq!(state.pos, Vec3A::new(1.0, 2.0, 3.0));
        assert_eq!(state.ang_vel.z, 1.0);
    }

    #[test]
    fn jump_counter_inference_can_be_ablated_and_gated() {
        let mut car = observations::Car {
            actor_id: 1,
            actor_created_frame: 0,
            player_key: None,
            player_link_active: false,
            team: None,
            body_product_id: None,
            body: Body::default(),
            boost: None,
            boost_raw: None,
            inputs: observations::Inputs {
                jump_active_raw: Some(Value {
                    value: 1,
                    frame: 1,
                    source: Source::Replay,
                }),
                ..observations::Inputs::default()
            },
        };
        let mut options = ConvertOptions::default();
        assert!(options.infer_jump_from_active);
        assert!(options.gate_jump_on_observed_impulse);
        options.infer_jump_from_active = false;
        options.gate_jump_on_observed_impulse = false;
        assert!(!controls_from_observation(&car, &options).jump);
        options.infer_jump_from_active = true;
        assert!(controls_from_observation(&car, &options).jump);
        car.body.position = Some(Value {
            value: [0.0, 0.0, 17.0],
            frame: 1,
            source: Source::Replay,
        });
        car.body.linear_velocity = Some(Value {
            value: [0.0, 0.0, 300.0],
            frame: 1,
            source: Source::Replay,
        });
        assert!(!jump_impulse_unobserved(&car, 1));
        car.body.linear_velocity.as_mut().unwrap().frame = 0;
        assert!(jump_impulse_unobserved(&car, 1));
        car.body.position.as_mut().unwrap().value[2] = 100.0;
        assert!(!jump_impulse_unobserved(&car, 1));
        car.inputs.jump_active_raw.as_mut().unwrap().value = 2;
        assert!(!controls_from_observation(&car, &options).jump);
    }

    #[test]
    fn dodge_impulse_unobserved_checks_airborne_and_velocity_freshness() {
        let mut car = observations::Car {
            actor_id: 1,
            actor_created_frame: 0,
            player_key: None,
            player_link_active: false,
            team: None,
            body_product_id: None,
            body: Body::default(),
            boost: None,
            boost_raw: None,
            inputs: observations::Inputs::default(),
        };
        let mut sim_state = CarState::default();
        sim_state.phys.pos.z = 17.0;
        sim_state.is_on_ground = true;

        assert!(!dodge_impulse_unobserved(&car, 1, &sim_state));

        sim_state.phys.pos.z = 200.0;
        sim_state.is_on_ground = false;
        assert!(dodge_impulse_unobserved(&car, 1, &sim_state));

        car.body.linear_velocity = Some(Value {
            value: [500.0, 0.0, 0.0],
            frame: 1,
            source: Source::Replay,
        });
        assert!(!dodge_impulse_unobserved(&car, 1, &sim_state));

        car.body.linear_velocity.as_mut().unwrap().frame = 0;
        assert!(dodge_impulse_unobserved(&car, 1, &sim_state));
    }

    #[test]
    fn known_body_products_select_their_rocketsim_hitbox() {
        for (product, name, expected) in [
            (21, "octane", CarBodyConfig::OCTANE),
            (22, "breakout", CarBodyConfig::BREAKOUT),
            (23, "octane", CarBodyConfig::OCTANE),
            (26, "octane", CarBodyConfig::OCTANE),
            (403, "dominus", CarBodyConfig::DOMINUS),
            (4284, "octane", CarBodyConfig::OCTANE),
            (7012, "hybrid", CarBodyConfig::HYBRID),
            (7477, "merc", CarBodyConfig::MERC),
            (7979, "merc", CarBodyConfig::MERC),
            (25, "octane", CarBodyConfig::OCTANE),
            (1691, "plank", CarBodyConfig::PLANK),
            (1919, "plank", CarBodyConfig::PLANK),
            (10900, "octane", CarBodyConfig::OCTANE),
            (4782, "psyclops", CarBodyConfig::PSYCLOPS),
            (11141, "hybrid", CarBodyConfig::HYBRID),
            (12325, "dominus", CarBodyConfig::DOMINUS),
            (12657, "breakout", CarBodyConfig::BREAKOUT),
            (12814, "octane", CarBodyConfig::OCTANE),
        ] {
            let (actual_name, actual) = hitbox_for_body_product(product).unwrap();
            assert_eq!(actual_name, name);
            assert_eq!(actual.hitbox_size, expected.hitbox_size);
        }
        assert!(hitbox_for_body_product(13008).is_none());
    }

    #[test]
    fn boost_pad_pickup_reconciliation_tracks_cooldown() {
        use crate::observations::{Frame, Header, PadPickup};

        let mut options = ConvertOptions::default();
        options.sync_boost_pad_pickups = true;

        let mut frames = vec![Frame {
            index: 0,
            time: 0.0,
            delta: 0.033,
            ball: None,
            cars: vec![observations::Car {
                actor_id: 1,
                actor_created_frame: 0,
                player_key: Some("player1".to_string()),
                player_link_active: true,
                team: Some(0),
                body_product_id: None,
                body: Body {
                    position: Some(Value {
                        value: [0.0, -4240.0, 17.0],
                        frame: 0,
                        source: Source::Replay,
                    }),
                    ..Body::default()
                },
                boost: Some(Value {
                    value: 33.0,
                    frame: 0,
                    source: Source::Replay,
                }),
                boost_raw: None,
                inputs: observations::Inputs::default(),
            }],
            players: Vec::new(),
            team_scores: [None, None],
            seconds_remaining: None,
            overtime: None,
            game_state: Some(Value {
                value: "Active".to_string(),
                frame: 0,
                source: Source::Replay,
            }),
            events: Vec::new(),
            pad_pickups: vec![PadPickup {
                pad_actor_id: 50,
                pad_actor_name: Some(
                    "cs_p.TheWorld:PersistentLevel.VehiclePickup_Boost_TA_0".to_string(),
                ),
                instigator_car_id: Some(1),
                picked_up: 1,
            }],
        }];
        let mut reset = frames[0].clone();
        reset.index = 1;
        reset.time = 0.033;
        reset.pad_pickups[0].instigator_car_id = None;
        reset.pad_pickups[0].picked_up = 255;
        frames.push(reset);

        let replay = ObservedReplay {
            header: Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames,
            diagnostics: Default::default(),
        };

        let output = convert_observations(replay, &options).unwrap();
        let pad_state = output.frames[0].state.boost_pads[0].1;
        assert_eq!(pad_state.cooldown, 4.0);
        assert_eq!(output.frames[1].state.boost_pads[0].1.cooldown, 0.0);
    }

    #[test]
    fn air_steer_controls_route_to_aerial_yaw_and_roll() {
        let car = observations::Car {
            actor_id: 1,
            actor_created_frame: 0,
            player_key: Some("p1".to_string()),
            player_link_active: true,
            team: Some(0),
            body_product_id: None,
            body: Body {
                position: Some(Value {
                    value: [0.0, 0.0, 200.0],
                    frame: 0,
                    source: Source::Replay,
                }),
                ..Body::default()
            },
            boost: None,
            boost_raw: None,
            inputs: observations::Inputs {
                steer: Some(Value {
                    value: 0.75,
                    frame: 0,
                    source: Source::Replay,
                }),
                handbrake: Some(Value {
                    value: false,
                    frame: 0,
                    source: Source::Replay,
                }),
                ..observations::Inputs::default()
            },
        };

        let make_replay = |car: observations::Car| ObservedReplay {
            header: observations::Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames: vec![observations::Frame {
                index: 0,
                time: 0.0,
                delta: 0.033,
                ball: None,
                cars: vec![car],
                players: Vec::new(),
                team_scores: [None, None],
                seconds_remaining: None,
                overtime: None,
                game_state: Some(Value {
                    value: "Active".to_string(),
                    frame: 0,
                    source: Source::Replay,
                }),
                events: Vec::new(),
                pad_pickups: Vec::new(),
            }],
            diagnostics: Default::default(),
        };

        let mut options = ConvertOptions::default();
        options.infer_air_steer_controls = true;
        let out = convert_observations(make_replay(car.clone()), &options).unwrap();
        let controls = out.frames[0].state.cars[0].1.controls;
        let (yaw, roll) = (controls.yaw, controls.roll);
        assert_eq!(yaw, 0.75);
        assert_eq!(roll, 0.0);

        // Handbrake held in the air turns steer into roll.
        let mut rolling = car.clone();
        rolling.inputs.handbrake = Some(Value {
            value: true,
            frame: 0,
            source: Source::Replay,
        });
        let out_roll = convert_observations(make_replay(rolling.clone()), &options).unwrap();
        let roll_controls = out_roll.frames[0].state.cars[0].1.controls;
        assert_eq!((roll_controls.yaw, roll_controls.roll), (0.0, 0.75));
        let mut no_roll = options.clone();
        no_roll.infer_air_roll_from_handbrake = false;
        let out_no_roll = convert_observations(make_replay(rolling), &no_roll).unwrap();
        let no_roll_controls = out_no_roll.frames[0].state.cars[0].1.controls;
        assert_eq!((no_roll_controls.yaw, no_roll_controls.roll), (0.75, 0.0));

        // When option disabled, yaw remains zero
        options.infer_air_steer_controls = false;
        let out_disabled = convert_observations(make_replay(car), &options).unwrap();
        let controls_disabled = out_disabled.frames[0].state.cars[0].1.controls;
        let (yaw_dis, roll_dis) = (controls_disabled.yaw, controls_disabled.roll);
        assert_eq!(yaw_dis, 0.0);
        assert_eq!(roll_dis, 0.0);
    }

    #[test]
    fn solve_inverse_air_controls_recovers_pure_inputs() {
        let rot = Mat3A::IDENTITY;
        let dt = 1.0 / 120.0;
        let omega0 = Vec3A::ZERO;
        let omega1 = Vec3A::new(0.0, -TORQUE_PITCH * dt, 0.0);
        let solved = solve_inverse_air_controls(rot, omega0, omega1, dt);
        assert!((solved.pitch - 1.0).abs() < 1e-3);
        assert!(solved.yaw.abs() < 1e-3);
        assert!(solved.roll.abs() < 1e-3);

        let omega_yaw = Vec3A::new(0.0, 0.0, TORQUE_YAW * dt);
        let solved_yaw = solve_inverse_air_controls(rot, omega0, omega_yaw, dt);
        assert!(solved_yaw.pitch.abs() < 1e-3);
        assert!((solved_yaw.yaw - 1.0).abs() < 1e-3);
        assert!(solved_yaw.roll.abs() < 1e-3);

        let omega_roll = Vec3A::new(-TORQUE_ROLL * dt, 0.0, 0.0);
        let solved_roll = solve_inverse_air_controls(rot, omega0, omega_roll, dt);
        assert!(solved_roll.pitch.abs() < 1e-3);
        assert!(solved_roll.yaw.abs() < 1e-3);
        assert!((solved_roll.roll - 1.0).abs() < 1e-3);
    }

    #[test]
    fn lookahead_infer_air_controls_populates_pitch_and_roll() {
        let dt = 0.033;
        let car0 = observations::Car {
            actor_id: 1,
            actor_created_frame: 0,
            player_key: Some("p1".to_string()),
            player_link_active: true,
            team: Some(0),
            body_product_id: None,
            body: Body {
                position: Some(Value {
                    value: [0.0, 0.0, 200.0],
                    frame: 0,
                    source: Source::Replay,
                }),
                rotation_xyzw: Some(Value {
                    value: [0.0, 0.0, 0.0, 1.0],
                    frame: 0,
                    source: Source::Replay,
                }),
                angular_velocity_replay_units: Some(Value {
                    value: [0.0, 0.0, 0.0],
                    frame: 0,
                    source: Source::Replay,
                }),
                ..Body::default()
            },
            boost: None,
            boost_raw: None,
            inputs: observations::Inputs::default(),
        };

        let mut car1 = car0.clone();
        car1.body.position.as_mut().unwrap().frame = 1;
        car1.body.rotation_xyzw.as_mut().unwrap().frame = 1;
        car1.body.angular_velocity_replay_units = Some(Value {
            value: [0.0, -100.0, 0.0],
            frame: 1,
            source: Source::Replay,
        });

        let replay = ObservedReplay {
            header: observations::Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames: vec![
                observations::Frame {
                    index: 0,
                    time: 0.0,
                    delta: dt,
                    ball: None,
                    cars: vec![car0],
                    players: Vec::new(),
                    team_scores: [None, None],
                    seconds_remaining: None,
                    overtime: None,
                    game_state: Some(Value {
                        value: "Active".to_string(),
                        frame: 0,
                        source: Source::Replay,
                    }),
                    events: Vec::new(),
                    pad_pickups: Vec::new(),
                },
                observations::Frame {
                    index: 1,
                    time: dt,
                    delta: dt,
                    ball: None,
                    cars: vec![car1],
                    players: Vec::new(),
                    team_scores: [None, None],
                    seconds_remaining: None,
                    overtime: None,
                    game_state: Some(Value {
                        value: "Active".to_string(),
                        frame: 1,
                        source: Source::Replay,
                    }),
                    events: Vec::new(),
                    pad_pickups: Vec::new(),
                },
            ],
            diagnostics: Default::default(),
        };

        let mut options = ConvertOptions::default();
        options.infer_air_controls_from_lookahead = true;
        let out = convert_observations(replay.clone(), &options).unwrap();
        let controls0 = out.frames[0].state.cars[0].1.controls;
        let pitch0 = controls0.pitch;
        assert!(pitch0 > 0.5, "expected pitch > 0.5, got {pitch0}");

        let mut stopped = replay.clone();
        stopped.frames[1].game_state.as_mut().unwrap().value = "Inactive".to_string();
        let stopped_out = convert_observations(stopped, &options).unwrap();
        let stopped_pitch = stopped_out.frames[0].state.cars[0].1.controls.pitch;
        assert_eq!(stopped_pitch, 0.0);

        let mut replacement = replay.clone();
        replacement.frames[1].cars[0].actor_created_frame = 1;
        let replacement_out = convert_observations(replacement, &options).unwrap();
        let replacement_pitch = replacement_out.frames[0].state.cars[0].1.controls.pitch;
        assert_eq!(replacement_pitch, 0.0);

        options.infer_air_controls_from_lookahead = false;
        let out_dis = convert_observations(replay, &options).unwrap();
        let controls0_dis = out_dis.frames[0].state.cars[0].1.controls;
        let pitch0_dis = controls0_dis.pitch;
        assert_eq!(pitch0_dis, 0.0);
    }

    #[test]
    fn span_solver_matches_forward_model_and_refinement_helps() {
        let rot = Mat3A::from_quat(Quat::from_rotation_x(0.2));
        let omega0 = Vec3A::new(0.3, -0.4, 0.2);
        let truth = AirControls {
            pitch: 0.6,
            yaw: -0.3,
            roll: 0.4,
        };
        let ticks = 12;
        let omega1 = air_angular_velocity_forward(rot, omega0, truth, ticks);
        let error = |iterations| {
            let solved = solve_span_air_controls(rot, omega0, omega1, ticks, iterations);
            let reached = air_angular_velocity_forward(rot, omega0, solved, ticks);
            (reached - omega1).length()
        };
        assert!(error(0) > 1e-3, "analytic span solve should be approximate");
        assert!(error(3) < error(0) * 0.1);
        assert!(error(3) < 5e-3);
    }

    fn span_test_replay(ang_frames: &[usize], frame_count: usize, dt: f32) -> ObservedReplay {
        let car_at = |index: usize| {
            let fresh = ang_frames
                .iter()
                .rev()
                .find(|&&f| f <= index)
                .copied()
                .unwrap();
            let mut car = observations::Car {
                actor_id: 1,
                actor_created_frame: 0,
                player_key: Some("p1".to_string()),
                player_link_active: true,
                team: Some(0),
                body_product_id: None,
                body: Body::default(),
                boost: None,
                boost_raw: None,
                inputs: observations::Inputs::default(),
            };
            let value = |value: [f32; 3]| Value {
                value,
                frame: fresh,
                source: Source::Replay,
            };
            car.body.position = Some(value([0.0, 0.0, 400.0]));
            car.body.rotation_xyzw = Some(Value {
                value: [0.0, 0.0, 0.0, 1.0],
                frame: fresh,
                source: Source::Replay,
            });
            let spin = fresh as f32 * -60.0;
            car.body.angular_velocity_replay_units = Some(value([0.0, spin, 0.0]));
            car
        };
        ObservedReplay {
            header: observations::Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames: (0..frame_count)
                .map(|index| observations::Frame {
                    index,
                    time: index as f32 * dt,
                    delta: dt,
                    ball: None,
                    cars: vec![car_at(index)],
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
                .collect(),
            diagnostics: Default::default(),
        }
    }

    #[test]
    fn lookahead_span_bridges_gap_and_refuses_withheld_or_long_spans() {
        let dt = 1.0 / 30.0;
        // Fresh car angular packets at frames 0 and 2; frame 1 carries the stale packet.
        let replay = span_test_replay(&[0, 2], 3, dt);
        let pitch = |replay: &ObservedReplay, options: &ConvertOptions, frame: usize| {
            convert_observations(replay.clone(), options)
                .unwrap()
                .frames[frame]
                .state
                .cars[0]
                .1
                .controls
                .pitch
        };
        let options = ConvertOptions::default();
        let first = pitch(&replay, &options, 0);
        assert!(first > 0.3, "span start pitch {first}");
        assert_eq!(
            first,
            pitch(&replay, &options, 1),
            "one constant control across the gap"
        );

        let mut adjacent_only = ConvertOptions::default();
        adjacent_only.air_lookahead_max_frames = 1;
        adjacent_only.air_lookahead_max_seconds = 0.05;
        assert_eq!(pitch(&replay, &adjacent_only, 0), 0.0);
        assert_eq!(pitch(&replay, &adjacent_only, 1), 0.0);

        let mut withheld = ConvertOptions::default();
        withheld.withheld_frames = Some(Arc::new(vec![false, true, false]));
        assert_eq!(pitch(&replay, &withheld, 0), 0.0);
        assert_eq!(pitch(&replay, &withheld, 1), 0.0);

        let mut short = ConvertOptions::default();
        short.air_lookahead_max_seconds = 0.05;
        assert_eq!(pitch(&replay, &short, 0), 0.0);

        let mut legacy = ConvertOptions::default();
        legacy.air_lookahead_refine_iterations = 0;
        assert!(pitch(&replay, &legacy, 0) > 0.3);
    }

    #[test]
    fn persisted_air_controls_use_only_packets_at_or_before_the_interval() {
        let dt = 1.0 / 30.0;
        // Fresh packets at frames 0 and 2 imply a strong pitch that persists over stale frames.
        let past_only = span_test_replay(&[0, 2], 5, dt);
        // A later packet at frame 4 must not change controls chosen for frames 2 and 3.
        let with_future = span_test_replay(&[0, 2, 4], 5, dt);
        let mut options = ConvertOptions::default();
        options.infer_air_controls_from_lookahead = false;
        options.air_persist_calibrated = false;
        let pitches = |replay: &ObservedReplay, options: &ConvertOptions| {
            convert_observations(replay.clone(), options)
                .unwrap()
                .frames
                .iter()
                .map(|frame| frame.state.cars[0].1.controls.pitch)
                .collect::<Vec<_>>()
        };
        let base = pitches(&past_only, &options);
        let future = pitches(&with_future, &options);
        assert!(base[2] > 0.5 && base[3] > 0.5, "persisted pitch {base:?}");
        assert_eq!(
            base[..4],
            future[..4],
            "later packets must not alter earlier controls"
        );
        assert_eq!(base[0], 0.0, "the first span has no earlier packet pair");

        let mut off = options.clone();
        off.persist_past_air_controls = false;
        assert_eq!(pitches(&past_only, &off)[3], 0.0);

        let mut expiring = options.clone();
        expiring.air_persist_max_seconds = 0.05;
        let expired = pitches(&past_only, &expiring);
        assert!(expired[3] > 0.5 && expired[4] == 0.0, "expiry {expired:?}");
        assert!(
            base[4] > 0.5,
            "default keeps the control through 0.067 s: {base:?}"
        );

        let mut strict = options.clone();
        strict.air_persist_min_control = 1.1;
        assert_eq!(pitches(&past_only, &strict)[2], 0.0);
    }

    #[test]
    fn median_ratios_follow_the_calibrated_table() {
        // Large roll (sustained air roll at the angular speed cap) persists at every lag.
        assert!(air_control_median_ratio(2, 0.05, 0.6) > 0.9);
        assert!(air_control_median_ratio(2, 0.15, 0.6) > 0.9);
        // Small controls are fit noise and lose most of their magnitude.
        assert!(air_control_median_ratio(2, 0.05, 0.2) < 0.2);
        assert_eq!(
            air_control_median_ratio(2, 0.05, 0.05),
            0.0,
            "below the calibrated range"
        );
        // Pitch is short-lived: partial persistence soon after the span, none after 0.133 s.
        assert!((0.3..0.9).contains(&air_control_median_ratio(0, 0.05, 0.6)));
        assert!(air_control_median_ratio(0, 0.17, 0.6) < 0.2);
        // Nothing persists beyond 0.2 s or for invalid lags.
        assert_eq!(air_control_median_ratio(2, 0.25, 0.9), 0.0);
        assert_eq!(air_control_median_ratio(2, -0.01, 0.9), 0.0);
    }

    #[test]
    fn calibrated_persistence_is_causal_and_prefers_observed_steer() {
        let dt = 1.0 / 30.0;
        let mut replay = span_test_replay(&[0, 2], 5, dt);
        for frame in &mut replay.frames {
            let car = &mut frame.cars[0];
            car.inputs.steer = Some(Value {
                value: 0.4,
                frame: frame.index,
                source: Source::Replay,
            });
        }
        let mut options = ConvertOptions::default();
        options.infer_air_controls_from_lookahead = false;
        let controls = |replay: &ObservedReplay, options: &ConvertOptions| {
            convert_observations(replay.clone(), options)
                .unwrap()
                .frames
                .iter()
                .map(|frame| frame.state.cars[0].1.controls)
                .collect::<Vec<_>>()
        };
        let calibrated = controls(&replay, &options);
        let mut legacy_options = options.clone();
        legacy_options.air_persist_calibrated = false;
        let legacy = controls(&replay, &legacy_options);
        // The calibrated model only ever shrinks the fitted control.
        for (calibrated, legacy) in calibrated.iter().zip(&legacy) {
            let (calibrated_pitch, legacy_pitch) = (calibrated.pitch, legacy.pitch);
            assert!(calibrated_pitch.abs() <= legacy_pitch.abs() + 1e-6);
        }
        // Observed steer drives yaw exactly; without handbrake roll is the shrunk fitted roll.
        let yaw = calibrated[2].yaw;
        assert_eq!(yaw, 0.4);
        // With the handbrake held steer drives roll instead.
        for frame in &mut replay.frames {
            frame.cars[0].inputs.handbrake = Some(Value {
                value: true,
                frame: frame.index,
                source: Source::Replay,
            });
        }
        let rolling = controls(&replay, &options);
        let roll = rolling[2].roll;
        assert_eq!(roll, 0.4);
        // A later packet changes nothing before it, and lags beyond the table are dropped.
        let mut late = span_test_replay(&[0, 2], 14, dt);
        for frame in &mut late.frames {
            frame.cars[0].inputs.steer = Some(Value {
                value: 0.0,
                frame: frame.index,
                source: Source::Replay,
            });
        }
        let long = controls(&late, &options);
        let pitch = long[13].pitch;
        assert_eq!(pitch, 0.0);
    }

    #[test]
    fn packet_lag_inference_recovers_whole_tick_lags_and_never_bridges_withheld_frames() {
        // Packets are exact whole-tick server states generated up to a frame period before the
        // frame time; frame times carry a small jitter so the absolute tick is constrained.
        let true_lags = [2i64, 0, 3, 1, 3, 0, 2, 1, 3, 0, 1, 2, 3, 0, 2, 1];
        let velocity = [1200.0f32, 300.0, 0.0];
        let frame_tick = |index: usize| 4.0 * index as f32 + 0.3 * (index % 3) as f32;
        let build = |lags: &[i64]| -> ObservedReplay {
            let frames: Vec<observations::Frame> = lags
                .iter()
                .enumerate()
                .map(|(index, lag)| {
                    let physical_tick = 4.0 * index as f32 - *lag as f32;
                    let body = Body {
                        position: Some(Value {
                            value: [
                                velocity[0] * physical_tick / 120.0,
                                velocity[1] * physical_tick / 120.0,
                                400.0,
                            ],
                            frame: index,
                            source: Source::Replay,
                        }),
                        linear_velocity: Some(Value {
                            value: velocity,
                            frame: index,
                            source: Source::Replay,
                        }),
                        ..Body::default()
                    };
                    observations::Frame {
                        index,
                        time: frame_tick(index) / 120.0,
                        delta: 4.0 / 120.0,
                        ball: Some(body),
                        cars: Vec::new(),
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
                    }
                })
                .collect();
            ObservedReplay {
                header: observations::Header {
                    game_type: "TAGame.Replay_Soccar_TA".to_string(),
                    levels: Vec::new(),
                    final_team_scores: [None, None],
                },
                frames,
                diagnostics: Default::default(),
            }
        };
        let replay = build(&true_lags);
        let options = ConvertOptions::default();
        assert!(options.exact_tick_lag_chains);
        let lags = infer_packet_lags(&replay, &options);
        // Lag = round(frame tick) - physical tick, exactly, for every packet in the chain.
        for (index, truth) in true_lags.iter().enumerate() {
            let expected = frame_tick(index).round() as i64 - (4 * index as i64 - truth);
            let estimate = lags.ball[index].expect("lag inferred");
            assert_eq!(estimate as i64, expected, "frame {index}");
        }
        // Fractional lags are not whole-tick physics: exact chains reject them.
        let fractional = {
            let mut replay = build(&true_lags);
            for (index, frame) in replay.frames.iter_mut().enumerate() {
                let extra = 0.5 * (index % 2) as f32 * velocity[0] / 120.0;
                frame
                    .ball
                    .as_mut()
                    .unwrap()
                    .position
                    .as_mut()
                    .unwrap()
                    .value[0] += extra;
            }
            infer_packet_lags(&replay, &options)
        };
        assert!(fractional.ball.iter().all(|lag| lag.is_none()));
        // A withheld frame breaks the chain: no pair bridges it.
        let mut withheld = options.clone();
        withheld.withheld_frames = Some(Arc::new(
            (0..true_lags.len()).map(|index| index == 5).collect(),
        ));
        let mut masked = replay.clone();
        masked.frames[5].ball = masked.frames[4].ball.clone();
        let blocked = infer_packet_lags(&masked, &withheld);
        assert!(blocked.ball[5].is_none());
        assert!(blocked.ball[4].is_some() && blocked.ball[6].is_some());
    }

    #[test]
    fn reported_hit_impulse_reaches_the_ball_only_when_applied() {
        rocketsim::init(Path::new("collision_meshes"), true).unwrap();
        let run = |apply: bool| -> (f32, f32) {
            let mut config = ArenaConfig::new(GameMode::Soccar);
            config.rng_seed = Some(0);
            let mut arena = Arena::new_with_config(config);
            arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
            let mut car = CarState::default();
            car.phys.pos = Vec3A::new(0.0, -600.0, 17.0);
            car.phys.vel = Vec3A::new(0.0, 1400.0, 0.0);
            car.phys.rot_mat = Mat3A::from_cols(Vec3A::Y, -Vec3A::X, Vec3A::Z);
            car.is_on_ground = true;
            car.wheels_with_contact = [true; 4];
            arena.set_car_state(0, car);
            let mut ball = BallState::default();
            ball.phys.pos = Vec3A::new(0.0, 0.0, 93.15);
            arena.set_ball_state(ball);
            arena.set_car_controls(
                0,
                CarControls {
                    throttle: 1.0,
                    ..CarControls::default()
                },
            );
            let mut reported = 0.0f32;
            let mut hit_tick = None;
            for tick in 1..=60u32 {
                for event in step_tick_with_hit_impulse(&mut arena, apply) {
                    if let ArenaEvent::CarHitBall(hit) = event {
                        if hit_tick.is_none() {
                            hit_tick = Some(tick);
                            reported = hit.extra_hit_vel.length();
                        }
                    }
                }
                if hit_tick.is_some_and(|hit| tick == hit + 4) {
                    return (arena.get_ball_state().phys.vel.length(), reported);
                }
            }
            panic!("no hit");
        };
        let (without, reported) = run(false);
        let (with, _) = run(true);
        assert!(
            reported > 500.0,
            "the hit reports an extra impulse ({reported})"
        );
        // The extra impulse is missing without the workaround and present with it.
        assert!(
            with > without + 0.8 * reported,
            "with {with} without {without} reported {reported}"
        );
    }

    #[test]
    fn position_residual_calculates_kinematics() {
        let frame0 = observations::Frame {
            index: 0,
            time: 0.0,
            delta: 0.033,
            ball: None,
            cars: vec![],
            players: vec![],
            team_scores: [None, None],
            seconds_remaining: None,
            overtime: None,
            game_state: None,
            events: vec![],
            pad_pickups: vec![],
        };
        let frame1 = observations::Frame {
            index: 1,
            time: 0.033,
            delta: 0.033,
            ball: None,
            cars: vec![],
            players: vec![],
            team_scores: [None, None],
            seconds_remaining: None,
            overtime: None,
            game_state: None,
            events: vec![],
            pad_pickups: vec![],
        };
        let frames = vec![frame0, frame1];

        let prev_body = Body {
            position: Some(Value {
                value: [0.0, 0.0, 100.0],
                frame: 0,
                source: Source::Replay,
            }),
            rotation_xyzw: Some(Value {
                value: [0.0, 0.0, 0.0, 1.0],
                frame: 0,
                source: Source::Replay,
            }),
            linear_velocity: Some(Value {
                value: [100.0, 0.0, 0.0],
                frame: 0,
                source: Source::Replay,
            }),
            angular_velocity_replay_units: Some(Value {
                value: [0.0, 0.0, 0.0],
                frame: 0,
                source: Source::Replay,
            }),
            ..Body::default()
        };

        let curr_body = Body {
            position: Some(Value {
                value: [10.0, 0.0, 100.0],
                frame: 1,
                source: Source::Replay,
            }),
            rotation_xyzw: Some(Value {
                value: [0.0, 0.0, 0.7071068, 0.7071068],
                frame: 1,
                source: Source::Replay,
            }),
            linear_velocity: Some(Value {
                value: [150.0, 0.0, 0.0],
                frame: 1,
                source: Source::Replay,
            }),
            angular_velocity_replay_units: Some(Value {
                value: [0.0, 0.0, 100.0],
                frame: 1,
                source: Source::Replay,
            }),
            ..Body::default()
        };

        let mut predicted = BallState::default().phys;
        predicted.pos = Vec3A::new(12.0, 0.0, 100.0);
        predicted.vel = Vec3A::new(140.0, 0.0, 0.0);
        predicted.rot_mat = Mat3A::IDENTITY;
        predicted.ang_vel = Vec3A::new(0.0, 0.0, 0.5);

        let residual = position_residual(
            1,
            Some(1),
            &curr_body,
            Some(&prev_body),
            &predicted,
            Some(false),
            &frames,
        )
        .expect("residual");

        assert!((residual.simulated_error_uu - 2.0).abs() < 1e-3);
        assert!((residual.hold_error_uu - 10.0).abs() < 1e-3);
        assert!((residual.simulated_velocity_error_uu_per_sec.unwrap() - 10.0).abs() < 1e-3);
        assert!((residual.hold_velocity_error_uu_per_sec.unwrap() - 50.0).abs() < 1e-3);
        assert!((residual.simulated_rotation_error_degrees.unwrap() - 90.0).abs() < 0.1);
        assert!((residual.hold_rotation_error_degrees.unwrap() - 90.0).abs() < 0.1);
        assert!(
            (residual
                .simulated_angular_velocity_error_rad_per_sec
                .unwrap()
                - 0.5)
                .abs()
                < 1e-3
        );
        assert!((residual.hold_angular_velocity_error_rad_per_sec.unwrap() - 1.0).abs() < 1e-3);
        assert_eq!(residual.altitude_z, Some(100.0));
        assert_eq!(residual.is_on_ground, Some(false));
        assert!(residual.offline_interval.is_some());
        let interval = residual.offline_interval.unwrap();
        assert_eq!(interval.effective_ticks, 10);
        assert!((interval.effective_seconds - 10.0 / 120.0).abs() < 1e-4);
        assert!(residual.offline_projection_fit_error_uu.is_some());
        assert!(
            (residual.offline_projection_fit_error_uu.unwrap()
                - (10.0f32 - 100.0 * 10.0 / 120.0).abs())
            .abs()
                < 1e-3
        );

        let mut frames = frames;
        frames[1].time = 0.8;
        let mut third = frames[1].clone();
        third.index = 2;
        third.time = 1.0;
        frames.push(third);
        let mut stale_previous = prev_body.clone();
        stale_previous.position.as_mut().unwrap().frame = 1;
        let mut fresh_actual = curr_body.clone();
        fresh_actual.position.as_mut().unwrap().frame = 2;
        fresh_actual.linear_velocity.as_mut().unwrap().frame = 2;
        fresh_actual.rotation_xyzw.as_mut().unwrap().frame = 2;
        fresh_actual
            .angular_velocity_replay_units
            .as_mut()
            .unwrap()
            .frame = 2;
        let stale = position_residual(
            2,
            Some(1),
            &fresh_actual,
            Some(&stale_previous),
            &predicted,
            Some(false),
            &frames,
        )
        .unwrap();
        assert!(stale.simulated_velocity_error_uu_per_sec.is_none());
        assert!(stale.simulated_rotation_error_degrees.is_none());
        assert!(stale.simulated_angular_velocity_error_rad_per_sec.is_none());
        assert!(stale.linear_extrapolation_error_uu.is_none());
        assert!(stale.offline_interval.is_none());
        assert!(stale.offline_projection_fit_error_uu.is_none());
    }

    #[test]
    fn estimate_car_packet_interval_quantizes_to_120hz() {
        let est = estimate_car_packet_interval(
            [0.0, 0.0, 0.0],
            [100.0, 0.0, 0.0],
            [1200.0, 0.0, 0.0],
            [1200.0, 0.0, 0.0],
            0.033333,
        )
        .expect("valid interval");
        assert_eq!(est.effective_ticks, 10);
        assert!((est.effective_seconds - 10.0 / 120.0).abs() < 1e-4);
        assert!((est.scale - (10.0 / 120.0) / 0.033333).abs() < 1e-3);

        let slow_est = estimate_car_packet_interval(
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [20.0, 0.0, 0.0],
            [20.0, 0.0, 0.0],
            0.033333,
        );
        assert!(slow_est.is_none());

        assert!(
            estimate_car_packet_interval(
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                [300.0, 0.0, 0.0],
                [300.0, 0.0, 0.0],
                -0.03,
            )
            .is_none()
        );
        assert!(
            estimate_car_packet_interval(
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                [300.0, 0.0, 0.0],
                [300.0, 0.0, 0.0],
                0.6,
            )
            .is_none()
        );
    }
}
