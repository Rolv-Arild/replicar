//! Replay frame snapshots produced by short RocketSim steps and fresh corrections.

use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fmt;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

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
    /// Infer aerial pitch, yaw, and roll controls from subsequent observed angular velocity.
    pub infer_air_controls_from_lookahead: bool,
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
            infer_air_controls_from_lookahead: true,
            infer_transition_air_lookahead: false,
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

#[derive(Debug, Clone)]
pub struct ConvertedFrame {
    pub replay_frame: usize,
    pub replay_time: f32,
    /// 120 Hz tick on the elapsed replay timeline (including frozen phases).
    pub timeline_tick: u64,
    pub state: ArenaState,
    pub simulated_events: Vec<SimEvent>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize)]
pub struct Diagnostics {
    pub skipped_timeline_ticks: u64,
    pub unlinked_car_frames: usize,
    pub default_hitbox_players: usize,
    pub active_pawn_demo_corrections: usize,
    pub shadowed_car_frames: usize,
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
    let mut previous_tick = 0;
    let mut previous_active = false;
    let mut ball_initialized = false;

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
        if simulated {
            for _ in 0..gap {
                let arena_tick = arena.tick_count() + 1;
                let tick_events = arena.step_tick();
                events.extend(
                    tick_events
                        .iter()
                        .copied()
                        .map(|event| SimEvent { arena_tick, event }),
                );
            }
        } else {
            diagnostics.skipped_timeline_ticks += gap;
        }
        previous_tick = timeline_tick;
        previous_active = active;

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

        let primary = observations::primary_linked_cars(frame);
        diagnostics.shadowed_car_frames += frame
            .cars
            .iter()
            .filter(|car| car.player_key.is_some())
            .count()
            - primary.len();
        let mut selected_slots = HashSet::new();
        for car in primary
            .into_iter()
            .chain(frame.cars.iter().filter(|car| car.player_key.is_none()))
        {
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
            let mut dirty =
                apply_body(&mut state.phys, &car.body, frame.index, new_lifetime) || new_lifetime;
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
                                    state.flip_rel_torque = Vec3A::new(tx / 2.60, ty / 2.24, 0.0);
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
                if let Some(next_frame) = observations.frames.get(frame_idx + 1) {
                    let dt = next_frame.time - frame.time;
                    if dt > 0.0
                        && dt <= 0.05
                        && next_frame
                            .game_state
                            .as_ref()
                            .is_some_and(|s| s.value == "Active")
                    {
                        if let Some(next_car) = next_frame.cars.iter().find(|c| {
                            c.actor_id == car.actor_id
                                && c.actor_created_frame == car.actor_created_frame
                                && c.player_key == car.player_key
                                && c.player_link_active == car.player_link_active
                        }) {
                            if let (Some(ang1), Some(ang0), Some(rot0)) = (
                                &next_car.body.angular_velocity_replay_units,
                                &car.body.angular_velocity_replay_units,
                                &car.body.rotation_xyzw,
                            ) {
                                if ang1.frame == next_frame.index
                                    && ang0.frame == frame.index
                                    && rot0.frame == frame.index
                                    && car.body.position.as_ref().is_some_and(|p| {
                                        p.frame == frame.index && p.value[2] > min_lookahead_z
                                    })
                                    && next_car.body.position.as_ref().is_some_and(|p| {
                                        p.frame == next_frame.index && p.value[2] > min_lookahead_z
                                    })
                                    && !next_car.inputs.dodge_active_raw.as_ref().is_some_and(|d| {
                                        d.frame == next_frame.index && d.value % 2 == 1
                                    })
                                {
                                    let q0 = Quat::from_xyzw(
                                        rot0.value[0],
                                        rot0.value[1],
                                        rot0.value[2],
                                        rot0.value[3],
                                    );
                                    if q0.is_finite() && q0.length_squared() > 0.5 {
                                        let solved = solve_inverse_air_controls(
                                            Mat3A::from_quat(q0.normalize()),
                                            Vec3A::from_array(ang0.value) * 0.01,
                                            Vec3A::from_array(ang1.value) * 0.01,
                                            dt,
                                        );
                                        controls.pitch = solved.pitch;
                                        controls.yaw = solved.yaw;
                                        controls.roll = solved.roll;
                                        air_controls_applied = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if !air_controls_applied
                && options.compensate_transition_air_damping
                && airborne
                && !dodge_jump_control
                && (50.0..=100.0).contains(&state.phys.pos.z)
            {
                let angular = state.phys.ang_vel;
                let solved =
                    solve_inverse_air_controls(state.phys.rot_mat, angular, angular, 1.0 / 120.0);
                controls.pitch = solved.pitch;
                controls.yaw = solved.yaw;
                controls.roll = solved.roll;
                air_controls_applied = true;
            }
            if !air_controls_applied && options.feedback_low_air_angular && active && !new_lifetime
            {
                if let Some(solved) =
                    low_air_feedback_controls(observations, frame.index, car, &state, slot, &events)
                {
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
            if !air_controls_applied && options.infer_air_steer_controls && airborne {
                controls.yaw = controls.steer;
            }
            if dodge_jump_control {
                controls.jump = true;
                controls.pitch = dodge_pitch_control;
                controls.yaw = dodge_yaw_control;
            }
            arena.set_car_controls(slot, controls);
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
