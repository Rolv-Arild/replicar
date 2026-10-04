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
    CarState, DemoMode, GameMode, Mat3A, PhysState, Team, Vec3A,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::observations::{self, Body, ObservedReplay, Value};
use crate::parse_replay;

mod air;
mod fits;
mod packet_lags;

pub use air::*;
use fits::*;
pub use packet_lags::*;

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
    /// Offline, two passes: place each car packet before a ball contact -3..=+3 ticks off in a scratch
    /// arena, simulate the hit to the next ball packet, and keep the shift whose ball velocity is
    /// closest (`contact_alignment`); the second pass uses the moved lags. Uses a future ball packet.
    pub align_contacts: bool,
    /// With `apply_observed_demolitions`, switch RocketSim's own demolition rule off so that the
    /// observed demolitions are the only ones (no duplicate `car_hit_car` with `is_demo`, no
    /// invented demolitions).
    pub disable_simulated_demolitions: bool,
    /// Keep every boost pad on cooldown inside the simulation, so a simulated car never picks boost up
    /// by driving over a pad (a car whose simulated position is off by a few UU picks up a pad the real
    /// car missed, or the reverse): the boost amount then comes only from the replay's own updates
    /// (offline reconstruction; masked prediction has no later update and keeps the simulated pickups).
    pub block_sim_pad_pickups: bool,
    /// Offline: fit the unobserved timing of inputs against later packets in scratch arenas: when observed
    /// ground control changes took effect, jump presses, dodge starts and the flip's pitch cancel
    /// (RESULTS.md). Off: the counters' frame times and no cancel.
    pub input_fits: bool,
    /// Treat every fresh packet as lag-free (physical tick = frame time) instead of inferring lags:
    /// for replays recorded without replication lag (offline play), where a chain of lag-free
    /// packets fixes the lags only up to a constant and inference advances the states wrongly.
    pub zero_packet_lag: bool,
    /// Offline: solve an airborne car's air controls between two fresh packets as a boundary-value
    /// problem on both the rotation and the angular velocity at the end, with controls that may change
    /// every few ticks, and drive the interval with them (`plan_air_bvp`). Uses the next packet, so
    /// its residual is no longer a prediction.
    pub air_bvp: bool,
    /// Fit the ground control and jump timings against the *next* fresh packet (the end of the interval
    /// being driven) instead of the packet after it: the interior frames are then constrained by both
    /// ends, but the residual at that packet is no longer a held-out check.
    pub fit_on_next_packet: bool,
    /// Infer the tick of the first fresh car packet after a dodge activation (it has no chain lag) from
    /// the simulated path with the fitted start, within the lag range 0-4 ticks (offline; disabled
    /// without inferred packet lags).
    pub infer_dodge_first_packet_tick: bool,
    /// Leave the first interval (the one the cancel is used for) out of the flip-cancel fit when later
    /// packets exist, so the residual at the next packet is a check. The default fits it too: the
    /// exported states are then constrained by that packet, but its residual is in sample and
    /// flatters rotation and angular velocity in flip windows.
    pub flip_cancel_holdout: bool,
    /// Offline: infer when inside its frame each ball and car packet was generated (its lag behind
    /// the frame time, in ticks) from chained packet motion, and apply corrections at that time.
    pub infer_packet_lag: bool,
    /// Infer aerial pitch, yaw, and roll controls from subsequent observed angular velocity.
    pub infer_air_controls_from_lookahead: bool,
    /// Frames whose car/ball packets were withheld by an evaluator. A lookahead span that contains
    /// one would use a packet from after a withheld target, so it is refused.
    #[serde(skip)]
    pub withheld_frames: Option<Arc<Vec<bool>>>,
    /// Packet lags supplied from outside (for example true lags recovered from a server recording),
    /// used instead of inferring them. Experiments only.
    #[serde(skip)]
    pub external_packet_lags: Option<Arc<PacketLags>>,
    /// Select a RocketSim hitbox from the replay player's car-body product ID when known.
    pub use_loadout_hitboxes: bool,
}

impl ConvertOptions {}

/// Longest span, in frames, the offline air-control lookahead bridges between two fresh packets.
const AIR_LOOKAHEAD_MAX_FRAMES: usize = 10_000;
/// Gauss-Newton refinements of the span air-control solve (one refines the closed-form start).
const AIR_LOOKAHEAD_REFINE_ITERATIONS: usize = 1;
/// Later fresh packets the flip-cancel fit scores against.
const FLIP_CANCEL_PACKETS: usize = 1;
/// Gaps between active frames longer than this (10 s) are left unsimulated and recorded in diagnostics.
const MAX_GAP_TICKS: u64 = 1200;

impl Default for ConvertOptions {
    fn default() -> Self {
        Self {
            collision_meshes: PathBuf::from("collision_meshes"),
            seed: 0,
            align_contacts: true,
            disable_simulated_demolitions: true,
            block_sim_pad_pickups: true,
            input_fits: true,
            zero_packet_lag: false,
            air_bvp: true,
            fit_on_next_packet: true,
            infer_dodge_first_packet_tick: true,
            flip_cancel_holdout: false,
            infer_packet_lag: true,
            infer_air_controls_from_lookahead: true,
            withheld_frames: None,
            external_packet_lags: None,
            use_loadout_hitboxes: true,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SimEvent {
    pub arena_tick: u64,
    pub event: ArenaEvent,
}

/// A car-ball contact found from the ball packets (`ball_evidence`): the ball's velocity at a fresh
/// packet differs from what the ball alone would have by more than
/// `ball_evidence::CONTACT_VELOCITY_THRESHOLD`. The replay is the evidence; the car and the tick
/// are placed with the cars' exported states (the first tick the nearest car's hitbox reaches the
/// ball's no-touch path), so they are estimates.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct BallContact {
    /// The replay frame of the earlier packet; the contact happened after it and up to this frame's
    /// packet.
    pub frame_a: usize,
    /// Estimated tick of the contact on the replay timeline (120 Hz).
    pub tick: u64,
    /// The physical ticks of the two packets bracketing it.
    pub tick_from: u64,
    pub tick_to: u64,
    /// The car closest to the ball when it reached it, and that gap in UU (hitbox to ball surface,
    /// negative: overlapping); `None` when no car was within 150 UU (a goal post or another object).
    pub car_slot: Option<usize>,
    pub gap_uu: Option<f32>,
    /// Velocity difference to the no-touch rollout, UU/s.
    pub velocity_residual: f32,
    /// A simulated touch (`touches`) falls in the same interval.
    pub simulated_touch: bool,
}

/// A boost pad pickup reported by the replay (`PadPickup`, not a repeat), placed on the simulation's
/// pad list and checked against the cars' paths: the replay's instigator is believed only if its
/// path over the last frames crosses the pad's trigger cylinder.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct BoostPickup {
    /// Index in RocketSim's pad list (matched from the instigator's position when the pad was first
    /// seen; `None` while the pad could not be matched).
    pub pad_index: Option<usize>,
    pub pad_actor_id: i32,
    pub is_big: Option<bool>,
    /// The slot of the replay's instigator car.
    pub car_slot: Option<usize>,
    /// The instigator's path (straight lines between the exported poses of the last frames) enters
    /// the pad's trigger cylinder (radius 144 small, 208 big, plus 60 for the car's size).
    pub verified: bool,
    /// Closest horizontal distance of the instigator's path to the pad centre (UU).
    pub distance_uu: Option<f32>,
    /// When the instigator is not verified: another car whose path does.
    pub suggested_car_slot: Option<usize>,
    /// Closest approach of the (verified or suggested) car on the replay timeline; the frame's tick
    /// when no path reaches the pad.
    pub tick: u64,
}

/// A ball touch of the simulation: the first tick of a car-ball contact.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct TouchEvent {
    pub car_slot: usize,
    /// On the replay timeline (120 Hz, like `ConvertedFrame::timeline_tick`).
    pub tick: u64,
    pub contact_point: [f32; 3],
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

/// An input the converter inferred by fitting a later packet (not observed): a jump press, a dodge
/// press with its direction and pitch cancel, or an airborne interval whose pitch, yaw and roll were
/// solved by the boundary-value fit (`air_bvp`; the controls themselves are on the exported car
/// states). `tick` is on the replay timeline (120 Hz, like `ConvertedFrame::timeline_tick`) and is
/// the first tick the input takes effect.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FittedInput {
    pub slot: usize,
    /// Dodge only: the replay frame where the dodge counter turned odd (0 otherwise).
    pub activation_frame: usize,
    /// `"jump"`, `"dodge"` or `"air"`.
    pub kind: &'static str,
    pub tick: u64,
    /// Dodge only: RocketSim pitch and yaw controls of the press and the fitted cancel (0..1 of the
    /// flip's pitch torque removed); 0 for the other kinds.
    pub pitch: f32,
    pub yaw: f32,
    pub cancel: f32,
    /// Air only: the length in ticks of the interval the air controls were solved for (from `tick`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub span_ticks: Option<u64>,
}

#[derive(Debug, Clone)]
pub struct ConvertedFrame {
    pub replay_frame: usize,
    pub replay_time: f32,
    /// 120 Hz tick on the elapsed replay timeline (including frozen phases).
    pub timeline_tick: u64,
    pub state: ArenaState,
    pub simulated_events: Vec<SimEvent>,
    /// One entry per ball touch the simulation produced: the first tick of a contact between a car
    /// and the ball (RocketSim reports a hit every tick of a contact, and the extra impulse again
    /// within one). A simulated event, not an observation: 86-100% of the server's touches are
    /// reproduced (`RESULTS.md`).
    pub touches: Vec<TouchEvent>,
    /// Contacts found from the ball packets that end at this frame (`ball_evidence`). Preferred over
    /// `touches` (which are simulated; the two overlap: do not add them).
    pub ball_contacts: Vec<BallContact>,
    /// Boost pad pickups the replay reports in this frame (new ones only), checked against the cars'
    /// paths.
    pub boost_pickups: Vec<BoostPickup>,
    /// The match clock and its phase (`scoreboard`).
    pub scoreboard: Option<crate::scoreboard::ScoreboardFrame>,
    /// Applied packet lags for objects with a fresh packet; empty unless `infer_packet_lag`.
    pub packet_lags: Vec<AppliedPacketLag>,
    /// Jump and dodge inputs fitted at this frame's packets (arena ticks converted to the timeline).
    pub fitted_inputs: Vec<FittedInput>,
    /// The car slot of each replay car actor linked to a player in this frame (actor id, slot), the
    /// same slot as the car's column in the main export. Shadowed older cars of a player map to the
    /// player's slot too; a car actor without a linked player or without a slot is absent.
    pub car_actor_slots: Vec<(i32, usize)>,
    /// Bodies (replay car actor id; `None`: the ball) that had a fresh sleeping packet in this frame: their
    /// simulated linear and angular velocity were set to zero (inferred; the packet omits the velocities).
    pub sleeping_velocity_inferred: Vec<Option<i32>>,
    /// Car actors marked demolished in this frame because a fresh sleeping packet showed a dead pawn shell
    /// (no active player link): inferred, held until the slot's next lifetime or live packet.
    pub demolition_inferred: Vec<i32>,
    /// The slots held demolished as dead pawn shells in this frame (every frame of the hold, not just its
    /// start), with the reason: `observed` (a goal-explosion demolition report) or `inferred` (a sleeping
    /// packet of a car with no active pawn link).
    pub dead_shells_held: Vec<DeadShellHold>,
    /// Slots whose car is known only from its spawn pose in this frame (no rigid-body packet yet): the exported
    /// pose is the inferred spawn pose, and the car is kept out of the simulation's collisions.
    pub spawn_pose_held: Vec<usize>,
    /// The frame carries a fresh ball rigid-body packet (the ball's position was updated in this frame), which
    /// the correction step applied. Observed. A superset of the ball's `packet_lags` records: those exist
    /// only with `infer_packet_lag` and in frames the converter simulates, a fresh packet in a goal pause,
    /// countdown or first kickoff frame has no record.
    pub ball_fresh: bool,
    /// Slots whose primary car has a fresh rigid-body packet in this frame, applied by the correction step,
    /// ascending. Observed. A superset of the cars' `packet_lags` records, for the same reason.
    pub fresh_car_slots: Vec<usize>,
}

/// A slot held demolished as a dead pawn shell (`ConvertedFrame::dead_shells_held`).
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct DeadShellHold {
    pub slot: usize,
    pub source: &'static str,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize)]
pub struct Diagnostics {
    pub skipped_timeline_ticks: u64,
    /// Why the ball-contact intervals could not be prepared (`contacts_from_ball_packets`); the export then has
    /// no `ball_contacts`.
    pub ball_interval_error: Option<String>,
    /// A slot's car later showed another team or car body than the one its slot was created with (counted per
    /// distinct (slot, team, body)); the slot keeps its first hitbox and team.
    pub slot_loadout_changes: usize,
    pub unlinked_car_frames: usize,
    pub default_hitbox_players: usize,
    pub active_pawn_demo_corrections: usize,
    /// Observed dodge-refresh counter increases, and how many of them found the car's flags still set (the
    /// simulation had not reproduced the reset, so it was applied).
    pub dodge_refreshes_observed: usize,
    pub dodge_refreshes_applied: usize,
    /// Fresh sleeping rigid-body packets whose velocity was set to zero in the simulation (cars, ball).
    pub sleeping_car_packets: usize,
    /// Car lifetimes whose first simulated state came from the spawn trajectory because no rigid-body
    /// packet had arrived yet.
    pub cars_started_from_spawn_trajectory: usize,
    /// Dead pawn shells held demolished: marked by an observed goal-explosion demolition, inferred from a
    /// sleeping packet of a car with no active link, and how many holds ended (next lifetime or live packet).
    pub goal_explosion_demolitions: usize,
    pub dead_shells_inferred: usize,
    /// Dead pawn shells whose hold began with an observed (non-goal) demolition of a car with no active link
    /// or a sleeping body, after the frame's interval (so the bump of the demolition is still simulated).
    pub dead_shells_after_demolition: usize,
    pub dead_shells_released: usize,
    pub sleeping_ball_packets: usize,
    pub shadowed_car_frames: usize,
    /// Simulated frames with an inferred ball packet lag (`infer_packet_lag`).
    pub ball_lag_frames: usize,
    /// Simulated frames with an inferred car packet lag (`infer_packet_lag`).
    pub car_lag_frames: usize,
    /// Dodge activations (dodge counter turning odd with a fresh torque) seen while simulating.
    pub dodge_activations: usize,
    /// Dodge start ticks fitted against a later packet (`infer_dodge_start`).
    pub dodge_starts_fitted: usize,
    /// Airborne packets whose interval to the next packet got a boundary-value solution (`air_bvp`),
    /// and those that were refused (not free flight, a flip in the span, or no solution).
    pub air_bvp_planned: usize,
    pub air_bvp_refused: usize,
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

pub fn quaternion(xyzw: [f32; 4]) -> Option<Quat> {
    let q = Quat::from_xyzw(xyzw[0], xyzw[1], xyzw[2], xyzw[3]);
    (q.is_finite() && q.length_squared() > 1e-8).then(|| q.normalize())
}

pub fn rotation_error_degrees(a: Mat3A, b: Mat3A) -> f32 {
    let trace = a.x_axis.dot(b.x_axis) + a.y_axis.dot(b.y_axis) + a.z_axis.dot(b.z_axis);
    ((trace - 1.0) * 0.5).clamp(-1.0, 1.0).acos().to_degrees()
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
    if let Some(value) = &body.position
        && should_apply(value, index, new_entity)
    {
        state.pos = vec3(value.value);
        applied = true;
    }
    if let Some(value) = &body.rotation_xyzw
        && should_apply(value, index, new_entity)
    {
        let [x, y, z, w] = value.value;
        let quat = Quat::from_xyzw(x, y, z, w);
        if quat.is_finite() && quat.length_squared() > 0.5 {
            state.rot_mat = Mat3A::from_quat(quat.normalize());
            applied = true;
        }
    }
    if let Some(value) = &body.linear_velocity
        && should_apply(value, index, new_entity)
    {
        state.vel = vec3(value.value);
        applied = true;
    }
    if let Some(value) = &body.angular_velocity_replay_units
        && should_apply(value, index, new_entity)
    {
        state.ang_vel = vec3(value.value) * 0.01;
        applied = true;
    }
    applied
}

/// A fresh rigid-body packet with `sleeping` set omits the velocities: the body is at rest. The stale
/// velocity of an earlier packet would carry the body away in the simulation, so the simulated linear
/// and angular velocity are zeroed (inferred: the replay says only that the body sleeps, the omitted
/// velocity stays unknown in the observations). A velocity that is fresh in the same packet wins.
/// Returns `None` when the body has no fresh sleeping packet at `index`, else whether the velocity changed.
fn zero_sleeping_velocity(state: &mut PhysState, body: &Body, index: usize) -> Option<bool> {
    let sleeping_now = body
        .sleeping
        .as_ref()
        .is_some_and(|v| v.frame == index && v.value);
    if !sleeping_now {
        return None;
    }
    let mut changed = false;
    if !body
        .linear_velocity
        .as_ref()
        .is_some_and(|v| v.frame == index)
    {
        changed |= state.vel != Vec3A::ZERO;
        state.vel = Vec3A::ZERO;
    }
    if !body
        .angular_velocity_replay_units
        .as_ref()
        .is_some_and(|v| v.frame == index)
    {
        changed |= state.ang_vel != Vec3A::ZERO;
        state.ang_vel = Vec3A::ZERO;
    }
    Some(changed)
}

pub fn controls_from_observation(car: &observations::Car) -> CarControls {
    CarControls {
        throttle: car.inputs.throttle.as_ref().map_or(0.0, |v| v.value),
        steer: car.inputs.steer.as_ref().map_or(0.0, |v| v.value),
        handbrake: car.inputs.handbrake.as_ref().is_some_and(|v| v.value),
        boost: car
            .inputs
            .boost_active_raw
            .as_ref()
            .is_some_and(|v| v.value % 2 == 1),
        jump: car
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

/// The dodge torque of a car whose dodge counter turned odd at frame `g`. The replay sends the torque only
/// when it changes, so a dodge in the same direction as the last one has an old stamp (22% of the
/// activations of a host replay), and it can also arrive a frame after the counter: the value visible a
/// frame later is the one in effect, unless there is no later frame of the car.
pub fn activation_torque(
    frames: &[observations::Frame],
    g: usize,
    car: &observations::Car,
) -> Option<[f32; 3]> {
    let later = frames.get(g + 1).and_then(|frame| {
        frame.cars.iter().find(|c| {
            c.actor_id == car.actor_id && c.actor_created_frame == car.actor_created_frame
        })
    });
    later
        .unwrap_or(car)
        .inputs
        .dodge_torque_replay_units
        .as_ref()
        .or(car.inputs.dodge_torque_replay_units.as_ref())
        .map(|t| t.value)
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

/// RocketSim's hitbox preset of a roster name (`octane` for an unknown one).
pub fn hitbox_config(name: &str) -> CarBodyConfig {
    match name {
        "breakout" => CarBodyConfig::BREAKOUT,
        "dominus" => CarBodyConfig::DOMINUS,
        "hybrid" => CarBodyConfig::HYBRID,
        "merc" => CarBodyConfig::MERC,
        "plank" => CarBodyConfig::PLANK,
        "psyclops" => CarBodyConfig::PSYCLOPS,
        _ => CarBodyConfig::OCTANE,
    }
}

/// A scratch arena for the offline fits: one car with the given hitbox and no reachable boost pad. The
/// fits reuse their scratch arenas for the whole replay, so a pad one fit picked up would stay on
/// cooldown into later, unrelated fits; the main arena holds its pads on cooldown anyway
/// (`block_sim_pad_pickups`). RocketSim requires at least one pad (`BoostPadGrid::new` asserts a
/// non-empty list), so the only pad lies far below the floor.
pub(crate) fn scratch_arena(seed: u64, config: CarBodyConfig) -> Arena {
    let mut scratch_config = ArenaConfig::new(GameMode::Soccar);
    scratch_config.rng_seed = Some(seed);
    scratch_config.custom_boost_pads = Some(vec![rocketsim::BoostPadConfig {
        pos: Vec3A::new(0.0, 0.0, -10_000.0),
        is_big: false,
    }]);
    let mut scratch = Arena::new_with_config(scratch_config);
    scratch.add_car(Team::Blue, config);
    scratch
}

/// A car state moved to another arena whose tick counter differs: `last_extra_hit_tick` is an absolute
/// arena tick and RocketSim grants the extra ball-hit impulse only when `last_hit_tick + 1 < tick_count`, so
/// the copy keeps the hit's age (`source_tick` is the tick counter the state belongs to, `target_tick` the
/// destination's). A hit older than the destination's own clock reaches back (age above `target_tick`) is
/// dropped: it cannot matter, as a hit that old no longer blocks anything.
pub(crate) fn rebase_car_ticks(
    mut state: CarState,
    source_tick: u64,
    target_tick: u64,
) -> CarState {
    state.last_extra_hit_tick = rebase_tick(state.last_extra_hit_tick, source_tick, target_tick);
    state
}

/// An absolute arena tick of a state in an arena whose tick counter was `source_tick`, as the same age
/// before `target_tick`; `None` for no tick, a tick after `source_tick`, or one that would be negative.
pub fn rebase_tick(tick: Option<u64>, source_tick: u64, target_tick: u64) -> Option<u64> {
    target_tick.checked_sub(source_tick.checked_sub(tick?)?)
}

/// Seeds the scratch arena's car (`set_car_state(0, ..)`) with `state` from a timeline whose arena tick was
/// `source_tick` (the main arena's tick, or the tick the scratch arena had when it produced the state),
/// rebasing `last_extra_hit_tick` into the scratch arena's own tick counter (`rebase_car_ticks`).
pub(crate) fn seed_scratch_car(scratch: &mut Arena, state: CarState, source_tick: u64) {
    let target_tick = scratch.tick_count();
    scratch.set_car_state(0, rebase_car_ticks(state, source_tick, target_tick));
    scratch.refresh_car_sticky_gate(0);
}

/// Body product IDs are from boxcars' TeamLoadout, not RocketSim's preset indices.
/// The embedded map is generated from the user's item catalog, the official
/// Rocket League hitbox roster, and reviewed name aliases. Unknown IDs retain
/// the Octane fallback.
fn hitbox_for_body_product(id: u32) -> Option<(&'static str, CarBodyConfig)> {
    static CATALOG: OnceLock<Vec<(u32, &'static str)>> = OnceLock::new();
    let catalog = CATALOG.get_or_init(|| {
        let mut rows = Vec::new();
        for line in include_str!("../../data/body_hitboxes.tsv").lines().skip(1) {
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

/// Steps one tick and returns its events.
pub fn step_arena_tick(arena: &mut Arena) -> Vec<ArenaEvent> {
    arena.step_tick().to_vec()
}

/// A dodge scheduled at tick granularity: `jump` is pressed with the dodge direction on the tick
/// numbered `start_tick`, and from the next tick to `end_tick` the pitch cancel is held (opposite
/// pitch input, with the sign of the flip's relative pitch torque). `base` carries the other
/// controls.
#[derive(Debug, Clone, Copy)]
struct PendingDodge {
    slot: usize,
    start_tick: u64,
    end_tick: u64,
    pitch: f32,
    yaw: f32,
    cancel: f32,
    base: CarControls,
}

/// Per-tick ground controls for one car over the interval to its next fresh packet, from a fitted
/// timing of the observed control changes (`fit_ground_control_timing`).
struct GroundSchedule {
    slot: usize,
    /// Last arena tick the schedule covers (the tick that reaches the next fresh packet).
    end_tick: u64,
    /// (first arena tick, throttle, steer, handbrake, boost, jump), in order; a jump of `None`
    /// leaves the jump control as it is.
    entries: Vec<(u64, f32, f32, bool, bool, Option<bool>)>,
    /// The timing shift the ground control fit chose, when it was strictly better than the midpoint
    /// rule (the fit is informative); `None` for other fits.
    shift: Option<i64>,
}

/// The arena ticks at which a ground schedule presses jump. A press is a rising edge: the button is
/// already down when `previous_jump`, the jump control of the previous interval's last tick (not of
/// the current frame, which may already show the press), is set.
fn jump_press_ticks(previous_jump: bool, schedule: &GroundSchedule) -> Vec<u64> {
    let mut jumping = previous_jump;
    let mut ticks = Vec::new();
    for entry in &schedule.entries {
        if let Some(jump) = entry.5 {
            if jump && !jumping {
                ticks.push(entry.0);
            }
            jumping = jump;
        }
    }
    ticks
}

fn step_ticks(
    arena: &mut Arena,
    ticks: u64,
    pending: &mut Vec<PendingDodge>,
    ground: &mut Vec<GroundSchedule>,
    air: &mut Vec<AirSchedule>,
    events: &mut Vec<SimEvent>,
) {
    for _ in 0..ticks {
        let arena_tick = arena.tick_count() + 1;
        pending.retain(|dodge| dodge.end_tick >= arena_tick);
        ground.retain(|schedule| schedule.end_tick >= arena_tick);
        air.retain(|schedule| schedule.end_tick >= arena_tick);
        for schedule in air.iter() {
            if let Some(entry) = schedule.entries.iter().rev().find(|e| e.0 <= arena_tick) {
                let mut controls = *arena.get_car_controls(schedule.slot);
                // A jump press in the air is a double jump or a flip, whose kind RocketSim takes from
                // the direction of the same controls: leave those ticks to the press itself. A held
                // jump (no new press) is not one, and keeps the solved controls.
                let state = arena.get_car_state(schedule.slot);
                if controls.jump && !state.prev_controls.jump && !state.is_on_ground {
                    continue;
                }
                controls.pitch = entry.1.pitch;
                controls.yaw = entry.1.yaw;
                controls.roll = entry.1.roll;
                arena.set_car_controls(schedule.slot, controls);
            }
        }
        for schedule in ground.iter() {
            // From its press tick a pending dodge drives the car.
            if pending
                .iter()
                .any(|dodge| dodge.slot == schedule.slot && arena_tick >= dodge.start_tick)
            {
                continue;
            }
            if let Some(entry) = schedule.entries.iter().rev().find(|e| e.0 <= arena_tick) {
                let mut controls = *arena.get_car_controls(schedule.slot);
                controls.throttle = entry.1;
                controls.steer = entry.2;
                controls.handbrake = entry.3;
                controls.boost = entry.4;
                if let Some(jump) = entry.5 {
                    controls.jump = jump;
                }
                arena.set_car_controls(schedule.slot, controls);
            }
        }
        for dodge in pending.iter() {
            // An air schedule solved around this dodge owns the controls except on the press tick.
            if arena_tick != dodge.start_tick && air.iter().any(|s| s.slot == dodge.slot) {
                if arena_tick < dodge.start_tick {
                    let mut current = *arena.get_car_controls(dodge.slot);
                    current.jump = false;
                    arena.set_car_controls(dodge.slot, current);
                }
                continue;
            }
            let mut controls = dodge.base;
            controls.jump = false;
            if arena_tick < dodge.start_tick {
                // Release jump before the press, so the press at the start tick is a new edge (a
                // jump schedule for the same car sets the jump input itself).
                if !ground.iter().any(|schedule| schedule.slot == dodge.slot) {
                    arena.set_car_controls(dodge.slot, controls);
                }
            } else if arena_tick == dodge.start_tick {
                controls.jump = true;
                controls.pitch = dodge.pitch;
                controls.yaw = dodge.yaw;
                // The dodge direction is (-pitch, yaw + roll): a roll left in `base` would turn it.
                controls.roll = 0.0;
                arena.set_car_controls(dodge.slot, controls);
            } else if arena_tick > dodge.start_tick {
                let sign = arena.get_car_state(dodge.slot).flip_rel_torque.y.signum();
                controls.pitch = dodge.cancel * sign;
                arena.set_car_controls(dodge.slot, controls);
            }
        }
        let tick_events = step_arena_tick(arena);
        events.extend(
            tick_events
                .into_iter()
                .map(|event| SimEvent { arena_tick, event }),
        );
    }
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

/// The number of car slots a conversion of `observations` creates, without simulating: one per player
/// key that appears on a primary linked car with a known team (the rule `convert_observations_with`
/// adds a slot by). The order of the slots depends on the packet lags, their number does not, so a
/// writer can size per-slot columns before the single conversion pass.
pub fn car_slot_count(observations: &ObservedReplay) -> usize {
    let mut keys: HashSet<&str> = HashSet::new();
    for frame in &observations.frames {
        for car in observations::primary_linked_cars(frame) {
            if let (Some(key), Some(_)) = (car.player_key.as_deref(), car.team) {
                keys.insert(key);
            }
        }
    }
    keys.len()
}

/// RocketSim panics (instead of returning an error) when the mesh directory exists but holds no soccar
/// collision meshes, so look for them first: `<dir>/soccar/*.cmf`.
pub fn check_soccar_meshes(meshes: &Path) -> Result<(), ConvertError> {
    let soccar = meshes.join("soccar");
    let found = std::fs::read_dir(&soccar)
        .map(|entries| {
            entries.filter_map(Result::ok).any(|entry| {
                entry
                    .path()
                    .extension()
                    .is_some_and(|e| e.eq_ignore_ascii_case("cmf"))
            })
        })
        .unwrap_or(false);
    if found {
        Ok(())
    } else {
        Err(ConvertError::Init(io::Error::new(
            io::ErrorKind::NotFound,
            format!(
                "no soccar collision meshes (*.cmf) in {}: put the supplied RocketSim meshes under <collision_meshes>/soccar/",
                soccar.display()
            ),
        )))
    }
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
    if options.align_contacts
        && options.external_packet_lags.is_none()
        && options.infer_packet_lag
        && !options.zero_packet_lag
    {
        let (lags, _) = crate::contact_alignment::aligned_lags(observations, options)?;
        let mut second = options.clone();
        second.align_contacts = false;
        second.external_packet_lags = Some(Arc::new(lags));
        return convert_observations_with(observations, &second, on_frame);
    }
    if observations.header.game_type != "TAGame.Replay_Soccar_TA" {
        return Err(ConvertError::UnsupportedMode(
            observations.header.game_type.clone(),
        ));
    }
    check_soccar_meshes(Path::new(&options.collision_meshes))?;
    rocketsim::init(Path::new(&options.collision_meshes), true).map_err(ConvertError::Init)?;
    let mut config = ArenaConfig::new(GameMode::Soccar);
    config.rng_seed = Some(options.seed);
    if options.disable_simulated_demolitions {
        // The replay reports every demolition; RocketSim's own bump detection reproduced 83% of
        // them and invented as many (114 of 254 on the train split, 27 of 57 replays with no
        // demolition in the replay at all).
        config.mutators.demo_mode = DemoMode::Disabled;
    }
    let mut arena = Arena::new_with_config(config);
    let mut slots: HashMap<String, usize> = HashMap::new();
    let mut car_slots: Vec<CarSlot> = Vec::new();
    let mut actor_slots: HashMap<i32, (usize, usize)> = HashMap::new();
    // Car lifetimes already started from their spawn trajectory (`observations::SpawnPose`).
    let mut spawn_started: HashSet<(i32, usize)> = HashSet::new();
    let mut slot_changes_seen: HashSet<(usize, Option<u8>, Option<u32>)> = HashSet::new();
    // Slots whose car is known only from its spawn pose (no rigid-body packet yet in this lifetime): kept out of
    // the simulation's collisions until the first packet. RocketSim has no per-car collision switch other than
    // the demolished state, so the car is held demolished internally; the exported state shows it as not
    // demolished at the spawn pose (`ConvertedFrame::spawn_pose_held`).
    // (slot, the car lifetime that holds it)
    let mut spawn_held: HashMap<usize, (i32, usize)> = HashMap::new();
    // Car lifetimes whose spawn-pose hold ended in an observed demolition: a normal demolition from then on.
    let mut spawn_demolished: HashSet<(i32, usize)> = HashSet::new();
    let mut gated_jump_active: HashMap<(i32, usize), bool> = HashMap::new();
    let mut last_dodge_raw: HashMap<(i32, usize), u8> = HashMap::new();
    let mut last_double_raw: HashMap<(i32, usize), u8> = HashMap::new();
    // Action counters (jump, double jump, dodge) of each car at its last frame, and at its last frame on
    // the ground.
    let mut last_counters: HashMap<(i32, usize), [u8; 3]> = HashMap::new();
    let mut ground_counters: HashMap<(i32, usize), [u8; 3]> = HashMap::new();
    let mut pad_actor_to_index: HashMap<i32, usize> = HashMap::new();
    // The pads' true cooldowns in seconds (0 = available), tracked from the replay's pickups while
    // the arena's own pads are held on cooldown to stop simulated pickups; written to the arena
    // before each export.
    let mut pad_cooldowns: Vec<f32> = vec![0.0; arena.num_boost_pads()];
    // A pad keeps its name (`VehiclePickup_Boost_TA_14`) when its actor is created again (after a
    // goal: 184 actors for 34 pads in one game), so the pad list index is voted for by the nearest
    // pad to the instigator at every pickup of that name, over the whole replay.
    let pad_name_to_index: HashMap<String, usize> = {
        let mut votes: HashMap<String, HashMap<usize, u32>> = HashMap::new();
        for frame in &observations.frames {
            for pickup in &frame.pad_pickups {
                let (Some(name), Some(instigator)) =
                    (&pickup.pad_actor_name, pickup.instigator_car_id)
                else {
                    continue;
                };
                if pickup.picked_up == 255 || pickup.repeat {
                    continue;
                }
                let Some(pos) = frame
                    .cars
                    .iter()
                    .find(|c| c.actor_id == instigator)
                    .and_then(|c| c.body.position.as_ref())
                    .filter(|p| {
                        p.frame <= frame.index
                            && frame.time - observations.frames[p.frame].time <= 0.1
                    })
                    .map(|p| vec3(p.value))
                else {
                    continue;
                };
                let mut ranked: Vec<(f32, usize)> = (0..arena.num_boost_pads())
                    .map(|idx| {
                        let pad = arena.get_boost_pad_config(idx).pos;
                        ((pad.x - pos.x).hypot(pad.y - pos.y), idx)
                    })
                    .collect();
                ranked.sort_by(|a, b| a.0.total_cmp(&b.0));
                if ranked.len() >= 2 && ranked[0].0 < 350.0 && ranked[1].0 - ranked[0].0 >= 100.0 {
                    *votes
                        .entry(name.clone())
                        .or_default()
                        .entry(ranked[0].1)
                        .or_default() += 1;
                }
            }
        }
        votes
            .into_iter()
            .filter_map(|(name, v)| {
                let mut ranked: Vec<(u32, usize)> = v.into_iter().map(|(i, n)| (n, i)).collect();
                ranked.sort_by(|a, b| b.cmp(a));
                let top = ranked[0];
                let second = ranked.get(1).map_or(0, |r| r.0);
                (top.0 >= 2 && top.0 >= 2 * second).then_some((name, top.1))
            })
            .collect()
    };
    let mut last_pad_counter: HashMap<i32, u8> = HashMap::new();
    let mut diagnostics = Diagnostics::default();
    let first_time = observations.frames.first().map_or(0.0, |frame| frame.time);
    let packet_lags = if let Some(external) = options.external_packet_lags.as_ref() {
        Some((**external).clone())
    } else if options.zero_packet_lag {
        Some(zero_packet_lags(observations))
    } else {
        options
            .infer_packet_lag
            .then(|| infer_packet_lags(observations, options))
    };
    let mut previous_tick = 0;
    let mut previous_active = false;
    let mut ball_initialized = false;
    let mut pending_dodges: Vec<PendingDodge> = Vec::new();
    let mut ground_schedules: Vec<GroundSchedule> = Vec::new();
    // Ball intervals with a contact, by the frame that ends them (`contacts_from_ball_packets`), and
    // the exported car poses of the last frames to place the contact.
    let contact_intervals: HashMap<usize, crate::ball_evidence::BallInterval> = if options
        .infer_packet_lag
        && !options.zero_packet_lag
    {
        match packet_lags
            .as_ref()
            .map(|lags| crate::ball_evidence::ball_intervals(observations, lags, options))
        {
            Some(Ok(v)) => v
                .into_iter()
                .filter(|i| i.velocity_residual > crate::ball_evidence::CONTACT_VELOCITY_THRESHOLD)
                .map(|i| (i.frame_b, i))
                .collect(),
            Some(Err(error)) => {
                // Without them there are no ball contacts: report it instead of silently exporting none.
                diagnostics.ball_interval_error = Some(error.to_string());
                HashMap::new()
            }
            None => HashMap::new(),
        }
    } else {
        HashMap::new()
    };
    // Per recent frame: (timeline tick, [(slot, position, rotation, demolished, lifetime)]), the lifetime
    // being the creation frame of the slot's car actor (poses of two lifetimes are never interpolated).
    let mut recent_poses: std::collections::VecDeque<(
        u64,
        Vec<(usize, Vec3A, Mat3A, bool, usize)>,
    )> = std::collections::VecDeque::new();
    let mut recent_touch_ticks: std::collections::VecDeque<u64> = std::collections::VecDeque::new();
    // The match clock and its lifecycle from the replay's integer clock (offline).
    let scoreboard = crate::scoreboard::reconstruct(observations);
    let mut ball_decided = false;
    // Last arena tick of a car-ball contact event per slot (to find where a contact starts).
    let mut last_contact_tick: HashMap<usize, u64> = HashMap::new();
    // Timeline tick until which an observed demolition keeps a slot demolished.
    let mut demo_hold_until: HashMap<usize, u64> = HashMap::new();
    // Slots held demolished because their car is a dead pawn shell (a goal-explosion victim, or a body whose
    // sleeping packet says its pawn has no active link), by the car actor lifetime they belong to. Held until
    // the slot's next lifetime or its next live packet (active link, not sleeping), not for RocketSim's 3 s.
    // (car actor, creation frame, source): `observed` (a goal-explosion report) or `inferred` (a sleeping packet).
    let mut dead_shells: HashMap<usize, (i32, usize, &'static str)> = HashMap::new();
    // Informative shifts chosen by the ground timing fit, per car actor lifetime.
    let mut car_shifts: HashMap<(i32, usize), Vec<i64>> = HashMap::new();
    let mut air_schedules: Vec<AirSchedule> = Vec::new();
    let mut ground_scratch: HashMap<&'static str, Arena> = HashMap::new();
    let mut slot_bodies: HashMap<usize, (&'static str, CarBodyConfig)> = HashMap::new();
    let mut handled_dodges: HashSet<(i32, usize, usize)> = HashSet::new();
    // Lags (ticks) fitted for the first car packet after a dodge activation, by (actor, lifetime, frame).
    let lag_overrides: std::cell::RefCell<HashMap<(i32, usize, usize), u64>> =
        std::cell::RefCell::new(HashMap::new());
    // Scratch arenas of the flip cancel, dodge start and flip boundary-value fits, one per hitbox: the
    // dodge start fit simulates the ball, so a contact depends on the car's hitbox.
    let flip_fits = options.input_fits;
    let mut flip_scratch: HashMap<&'static str, Arena> = HashMap::new();
    let mut flip_cache: HashMap<(i32, usize, usize), Option<f32>> = HashMap::new();
    let mut flip_last: HashMap<(i32, usize), f32> = HashMap::new();

    for (frame_idx, frame) in observations.frames.iter().enumerate() {
        let mut frame_residuals = Vec::new();
        // Bodies (None: the ball) whose simulated velocity was set to zero by a sleeping packet.
        let mut sleeping_inferred: Vec<Option<i32>> = Vec::new();
        // Car actors marked demolished by the sleeping-shell rule (inferred).
        let mut demolition_inferred: Vec<i32> = Vec::new();
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
        let mut fitted_arena: Vec<(usize, &'static str, u64, f32, f32, f32, usize)> = Vec::new();
        let simulated = active && previous_active && gap > 0 && gap <= MAX_GAP_TICKS;
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
                (Some(lags), true) => {
                    let fitted = lag_overrides
                        .borrow()
                        .get(&(car.actor_id, car.actor_created_frame, frame_idx))
                        .copied();
                    lag_ticks(fitted.map(|lag| lag as f32).or_else(|| {
                        lags.car_actor
                            .get(&(car.actor_id, car.actor_created_frame, frame_idx))
                            .copied()
                            .or(lags.cars[frame_idx])
                    }))
                }
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
                    source: if lag_overrides.borrow().contains_key(&(
                        car.actor_id,
                        car.actor_created_frame,
                        frame_idx,
                    )) {
                        "dodge_fit"
                    } else if own {
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
        let frame_withheld = options
            .withheld_frames
            .as_ref()
            .is_some_and(|w| w.get(frame_idx).copied().unwrap_or(false));
        // Control switches inside this interval: (ticks after its start, the car whose observed
        // controls take effect), in time order. Controls first seen at this frame act from the
        // `gap / 2 - 2`
        // ticks into the interval that ends at its state (from the start if shorter). The next
        // frame's controls also act inside this interval when the middle of its car's packet
        // interval falls before this frame's time (a change takes about as long to be seen as a
        // frame lasts).
        let span = remaining;
        let mut switches: Vec<(u64, &observations::Car)> = Vec::new();
        if packet_lags.is_some() && remaining > 0 && !frame_withheld {
            let interval_start = timeline_tick as i64 - gap as i64;
            for car in frame_cars.iter().copied() {
                let median_shift = car_shifts
                    .get(&(car.actor_id, car.actor_created_frame))
                    .filter(|v| v.len() >= 5)
                    .map(|v| {
                        let mut recent: Vec<i64> = v[v.len().saturating_sub(15)..].to_vec();
                        recent.sort_unstable();
                        recent[recent.len() / 2]
                    })
                    .filter(|&m| m != 0);
                if let Some(shift) = median_shift {
                    // Observed controls of the frames around this one, each moved by the car's shift
                    // from the midpoint rule; the latest one in effect at the interval's start is
                    // applied at its start.
                    let mut in_effect: Option<&observations::Car> = None;
                    let mut later: Vec<(u64, &observations::Car)> = Vec::new();
                    // The frames whose switch (at their rule tick plus the shift) can fall in this
                    // interval: further back the larger the shift (at least three ticks per frame).
                    let back = 4 + shift.unsigned_abs() as usize / 3;
                    for g in frame_idx.saturating_sub(back)
                        ..=(frame_idx + 4).min(observations.frames.len() - 1)
                    {
                        let Some(other) = observations.frames[g].cars.iter().find(|c| {
                            c.actor_id == car.actor_id
                                && c.actor_created_frame == car.actor_created_frame
                        }) else {
                            continue;
                        };
                        let spacing = if g == 0 {
                            4
                        } else {
                            let t = |x: usize| {
                                ((f64::from(observations.frames[x].time) - f64::from(first_time))
                                    * 120.0)
                                    .round() as i64
                            };
                            t(g) - t(g - 1)
                        };
                        let tick = ((f64::from(observations.frames[g].time)
                            - f64::from(first_time))
                            * 120.0)
                            .round() as i64
                            - 2
                            - spacing / 2
                            + shift;
                        let switch = tick - interval_start;
                        if switch < 0 {
                            in_effect = Some(other);
                        } else if switch as u64 <= span {
                            later.push((switch as u64, other));
                        }
                    }
                    if let Some(other) = in_effect {
                        switches.push((0, other));
                    }
                    switches.extend(later);
                    continue;
                }
                let switch = (gap / 2).saturating_sub(2);
                switches.push((switch.min(span), car));
            }
            switches.sort_by_key(|(switch, _)| *switch);
        }
        let mut next_switch = 0usize;
        // Steps the arena to `target` ticks after the interval's start, applying every control
        // switch due on the way.
        macro_rules! advance_to {
            ($target:expr) => {{
                let target: u64 = ($target).min(span);
                while next_switch < switches.len() && switches[next_switch].0 <= target {
                    let (switch, car) = switches[next_switch];
                    next_switch += 1;
                    let elapsed = span - remaining;
                    if switch > elapsed {
                        step_ticks(
                            &mut arena,
                            switch - elapsed,
                            &mut pending_dodges,
                            &mut ground_schedules,
                            &mut air_schedules,
                            &mut events,
                        );
                        limit_reported_velocities(&mut arena, slots.len());

                        remaining -= switch - elapsed;
                    }
                    let Some(&(slot, created)) = actor_slots.get(&car.actor_id) else {
                        continue;
                    };
                    if created != car.actor_created_frame || !arena.get_car_state(slot).is_on_ground
                    {
                        continue;
                    }
                    let next = controls_from_observation(car);
                    let mut controls = *arena.get_car_controls(slot);
                    controls.throttle = next.throttle;
                    controls.steer = next.steer;
                    controls.handbrake = next.handbrake;
                    controls.boost = next.boost;
                    arena.set_car_controls(slot, controls);
                }
                let elapsed = span - remaining;
                if target > elapsed {
                    step_ticks(
                        &mut arena,
                        target - elapsed,
                        &mut pending_dodges,
                        &mut ground_schedules,
                        &mut air_schedules,
                        &mut events,
                    );
                    limit_reported_velocities(&mut arena, slots.len());

                    remaining -= target - elapsed;
                }
            }};
        }
        if options.block_sim_pad_pickups {
            // Held on cooldown from the first tick of the interval: the arena carries the true
            // cooldowns written back at the end of the previous frame (for the export), and the
            // ticks before the last lag phase would otherwise let a car pick up an available pad
            // that the replay has not reported.
            for idx in 0..arena.num_boost_pads() {
                arena.set_boost_pad_state(idx, BoostPadState { cooldown: 20.0 });
            }
        }
        for lag in phase_lags {
            // Advance to this group's packet time (`lag` ticks before the frame time).
            advance_to!(span - lag.min(remaining));
            if ball_lag == lag
                && let Some(body) = &frame.ball
            {
                if simulated
                    && ball_initialized
                    && let Some(residual) = position_residual(
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
                    )
                {
                    frame_residuals.push(residual);
                }
                let mut ball = *arena.get_ball_state();
                let mut applied = apply_body(&mut ball.phys, body, frame.index, !ball_initialized);
                if let Some(changed) = zero_sleeping_velocity(&mut ball.phys, body, frame.index) {
                    applied |= changed;
                    sleeping_inferred.push(None);
                    diagnostics.sleeping_ball_packets += 1;
                }
                if applied {
                    arena.set_ball_state(ball);
                }
                ball_initialized = true;
            }
            for car in frame_cars.iter().copied() {
                if car_lag(car) != lag {
                    continue;
                }
                let slot = if let Some(key) = &car.player_key {
                    if let Some(slot) = slots.get(key).copied() {
                        actor_slots.insert(car.actor_id, (slot, car.actor_created_frame));
                        if let Some(info) = car_slots.iter().find(|s| s.slot == slot) {
                            let body_now = car.body_product_id.as_ref().map(|v| v.value);
                            let team_changed = car.team.is_some_and(|team| team != info.team);
                            let body_changed = body_now.is_some()
                                && info.body_product_id.is_some()
                                && body_now != info.body_product_id;
                            if (team_changed || body_changed)
                                && slot_changes_seen.insert((slot, car.team, body_now))
                            {
                                diagnostics.slot_loadout_changes += 1;
                            }
                        }
                        Some((slot, car.actor_created_frame == frame.index))
                    } else if let Some(team_idx) = car.team {
                        let body_product_id = car.body_product_id.as_ref().map(|v| v.value);
                        let known_hitbox = body_product_id
                            .filter(|_| options.use_loadout_hitboxes)
                            .and_then(hitbox_for_body_product);
                        let (hitbox, config) =
                            known_hitbox.unwrap_or(("octane", CarBodyConfig::OCTANE));
                        let slot = arena.add_car(team(team_idx), config);
                        slot_bodies.insert(slot, (hitbox, config));
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
                    let car_state = *arena.get_car_state(slot);
                    // A demolished car is not simulated: comparing it with a packet means nothing.
                    if let Some(residual) = (!car_state.is_demoed)
                        .then(|| {
                            position_residual(
                                frame.index,
                                Some(car.actor_id),
                                &car.body,
                                previous,
                                &car_state.phys,
                                Some(car_state.is_on_ground),
                                &observations.frames,
                            )
                        })
                        .flatten()
                    {
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
                // Before its first rigid-body packet a car starts from the replay's spawn trajectory (inferred),
                // not from RocketSim's default pose or from the previous car of its player.
                if car.body.position.is_none()
                    && let Some(spawn) = &car.spawn_pose
                    && spawn_started.insert((car.actor_id, car.actor_created_frame))
                {
                    state.phys.pos = vec3(spawn.position);
                    if let Some([x, y, z, w]) = spawn.rotation_xyzw {
                        state.phys.rot_mat = Mat3A::from_quat(Quat::from_xyzw(x, y, z, w));
                    }
                    state.phys.vel = Vec3A::ZERO;
                    state.phys.ang_vel = Vec3A::ZERO;
                    dirty = true;
                    diagnostics.cars_started_from_spawn_trajectory += 1;
                }
                // The spawn pose is an inference, not a body: the car it stands for must not hit the ball or other
                // cars before its first packet (a respawned car whose real self already hit the ball before the
                // pose was taken would hit it a second time).
                // The hold is kept for as long as the car has no body in this frame's observations, also when it
                // already has no spawn pose: in a withheld frame the observations hide a first packet (the body
                // is masked) but not the spawn pose's disappearance, so the release must not follow it.
                let spawn_lifetime = (car.actor_id, car.actor_created_frame);
                // A hold belongs to the lifetime that started it: another car on the slot (a new lifetime) ends it,
                // so it cannot keep holding a later car that has neither a body nor a spawn pose.
                if spawn_held
                    .get(&slot)
                    .is_some_and(|&owner| owner != spawn_lifetime)
                {
                    spawn_held.remove(&slot);
                    if state.is_demoed && !dead_shells.contains_key(&slot) {
                        state.is_demoed = false;
                        state.demo_respawn_timer = 0.0;
                        dirty = true;
                    }
                }
                if car.body.position.is_none()
                    && (car.spawn_pose.is_some() || spawn_held.get(&slot) == Some(&spawn_lifetime))
                    && !spawn_demolished.contains(&spawn_lifetime)
                {
                    spawn_held.insert(slot, spawn_lifetime);
                    if !state.is_demoed || state.demo_respawn_timer < 3.0 {
                        state.is_demoed = true;
                        state.demo_respawn_timer = 3.0;
                        dirty = true;
                    }
                } else if car.body.position.is_some()
                    && spawn_held.remove(&slot).is_some()
                    && state.is_demoed
                {
                    state.is_demoed = false;
                    state.demo_respawn_timer = 0.0;
                    dirty = true;
                }
                let sleeping_now = zero_sleeping_velocity(&mut state.phys, &car.body, frame.index);
                if let Some(changed) = sleeping_now {
                    dirty |= changed;
                    sleeping_inferred.push(Some(car.actor_id));
                    diagnostics.sleeping_car_packets += 1;
                }
                // Dead pawn shells. A live packet (active link, fresh position, not sleeping) or the slot's next
                // lifetime ends the hold; a sleeping packet of a car whose pawn link is inactive starts it
                // (inferred; a live car that merely sleeps has an active link and is never touched).
                let lifetime_key = (car.actor_id, car.actor_created_frame);
                let live_packet = car.player_link_active
                    && car
                        .body
                        .position
                        .as_ref()
                        .is_some_and(|p| p.frame == frame.index)
                    && !car.body.sleeping.as_ref().is_some_and(|s| s.value);
                if let Some(&(held_actor, held_created, _)) = dead_shells.get(&slot)
                    && ((held_actor, held_created) != lifetime_key || live_packet)
                {
                    dead_shells.remove(&slot);
                    // (A new car held on its spawn pose, just above, stays held out of collisions.)
                    if state.is_demoed && !spawn_held.contains_key(&slot) {
                        state.is_demoed = false;
                        state.demo_respawn_timer = 0.0;
                        dirty = true;
                    }
                    diagnostics.dead_shells_released += 1;
                }
                // A shell whose sleeping packet comes with the report of the demolition that killed it is not held
                // from its packet time on: the hold would take the body out of the simulation before the bump
                // (RocketSim's own demolition rule in the masked runs) that also slows the attacker. The
                // demolition is applied after the frame's interval, and the hold starts there. That handler runs
                // only in a simulated, non-withheld frame with `apply_observed_demolitions`; in any other frame
                // (a goal replay, a countdown) nothing would start the hold, so the sleeping packet does.
                let demolition_handler_runs = simulated && !frame_withheld;
                let demolished_this_frame = demolition_handler_runs
                    && frame.events.iter().any(|event| {
                        matches!(event, observations::Event::Demolish { source, victim_car: Some(v), repeat: false, .. }
                            if *source != "goal_explosion" && *v == car.actor_id)
                    });
                if sleeping_now.is_some()
                    && !car.player_link_active
                    && !frame_withheld
                    && !demolished_this_frame
                    && !dead_shells.contains_key(&slot)
                {
                    // A goal-explosion report for this very car in the frame is the observed reason (counted
                    // below); only a shell with no such report is an inference.
                    let observed = frame.events.iter().any(|event| {
                        matches!(event, observations::Event::Demolish { source: "goal_explosion", victim_car: Some(v), .. }
                            if *v == car.actor_id)
                    });
                    // Started as inferred; the goal-explosion report, applied below in this frame, makes it observed
                    // (and counts the hold once).
                    dead_shells.insert(slot, (lifetime_key.0, lifetime_key.1, "inferred"));
                    if !observed {
                        demolition_inferred.push(car.actor_id);
                        diagnostics.dead_shells_inferred += 1;
                    }
                }
                if dead_shells
                    .get(&slot)
                    .is_some_and(|&(a, c, _)| (a, c) == lifetime_key)
                    && (!state.is_demoed || state.demo_respawn_timer < 3.0)
                {
                    state.is_demoed = true;
                    state.demo_respawn_timer = 3.0;
                    dirty = true;
                }
                if car.player_link_active
                    && state.is_demoed
                    && !dead_shells.contains_key(&slot)
                    && !spawn_held.contains_key(&slot)
                    && demo_hold_until
                        .get(&slot)
                        .is_none_or(|&until| timeline_tick >= until)
                {
                    state.is_demoed = false;
                    state.demo_respawn_timer = 0.0;
                    diagnostics.active_pawn_demo_corrections += 1;
                    dirty = true;
                }
                if let Some(boost) = &car.boost
                    && should_apply(boost, frame.index, new_lifetime)
                {
                    state.boost = boost.value;
                    dirty = true;
                }

                let mut dodge_jump_control = false;
                let mut dodge_pitch_control = 0.0;
                let mut dodge_yaw_control = 0.0;

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
                    let torque_now = activation_torque(&observations.frames, frame.index, car);
                    if activated && torque_now.is_some() {
                        diagnostics.dodge_activations += 1;
                    }
                    if activated
                        && !handled_dodges.contains(&(
                            car.actor_id,
                            car.actor_created_frame,
                            frame.index,
                        ))
                        && let Some(torque) = torque_now
                    {
                        let [tx, ty, _] = torque;
                        let pitch = -ty / 2.24;
                        let yaw = -tx / 2.60;
                        if (pitch * pitch + yaw * yaw).sqrt() > 0.01 {
                            if dodge_impulse_unobserved(car, frame.index, &state) {
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

                let key = (car.actor_id, car.actor_created_frame);
                let double_raw = car
                    .inputs
                    .double_jump_active_raw
                    .as_ref()
                    .filter(|raw| raw.frame == frame.index)
                    .map(|raw| raw.value);
                if let Some(raw) = double_raw {
                    let prev = last_double_raw.insert(key, raw);
                    let activated = match prev {
                        Some(prev_val) => prev_val % 2 == 0 && raw % 2 == 1,
                        None => raw % 2 == 1,
                    };
                    if activated && !dodge_jump_control {
                        if dodge_impulse_unobserved(car, frame.index, &state) {
                            // A jump press with no direction is RocketSim's double jump.
                            dodge_jump_control = true;
                            dodge_pitch_control = 0.0;
                            dodge_yaw_control = 0.0;
                        } else if !state.is_on_ground || state.phys.pos.z > 50.0 {
                            // A fresh velocity packet at the activation frame already holds the
                            // impulse; only the state flags are missing.
                            state.has_jumped = true;
                            state.has_double_jumped = true;
                            dirty = true;
                        }
                    }
                }

                let key = (car.actor_id, car.actor_created_frame);
                let counter =
                    |v: &Option<observations::Value<u8>>| v.as_ref().map_or(0, |v| v.value);
                let current = [
                    counter(&car.inputs.jump_active_raw),
                    counter(&car.inputs.double_jump_active_raw),
                    counter(&car.inputs.dodge_active_raw),
                ];
                if new_lifetime {
                    ground_counters.remove(&key);
                } else if state.is_on_ground
                    && let Some(previous) = last_counters.get(&key)
                {
                    ground_counters.insert(key, *previous);
                }
                last_counters.insert(key, current);
                if !state.is_on_ground
                    && let Some(ground) = ground_counters.get(&key)
                {
                    let jumped = current[0] != ground[0];
                    let double_jumped = current[1] != ground[1];
                    let flipped = current[2] != ground[2];
                    // Seconds since the counter last changed (the stamp frame of its value).
                    let since = |v: &Option<observations::Value<u8>>| -> f32 {
                        v.as_ref().map_or(0.0, |v| {
                            (frame.time - observations.frames[v.frame].time).max(0.0)
                        })
                    };
                    if !state.has_jumped && (jumped || double_jumped || flipped) {
                        state.has_jumped = true;
                        state.air_time_since_jump = state
                            .air_time_since_jump
                            .max(since(&car.inputs.jump_active_raw));
                        dirty = true;
                    }
                    // An action whose counter changed this frame, or whose dodge is planned for
                    // later, is applied by the simulation itself; setting its flag first would
                    // block that.
                    let acting =
                        dodge_jump_control || pending_dodges.iter().any(|dodge| dodge.slot == slot);
                    if double_jumped && !state.has_double_jumped && !acting {
                        state.has_double_jumped = true;
                        state.has_jumped = true;
                        dirty = true;
                    }
                    if flipped && !state.has_flipped && !acting {
                        state.has_flipped = true;
                        state.has_jumped = true;
                        // Time since the flip, so the pitch lock after a flip ends on time.
                        state.flip_time = since(&car.inputs.dodge_active_raw).min(1.0);
                        dirty = true;
                    }
                }

                if dirty {
                    arena.set_car_state(slot, state);
                    // RocketSim asks for this after teleporting a car mid-drive (`arena/base.rs`).
                    arena.refresh_car_sticky_gate(slot);
                }
                let mut controls = controls_from_observation(car);
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

                let airborne = !state.is_on_ground || (new_lifetime && state.phys.pos.z > 50.0);
                let min_lookahead_z = 50.0;
                let mut air_controls_applied = false;
                if options.infer_air_controls_from_lookahead
                    && airborne
                    && !dodge_jump_control
                    && active
                    && let Some(solved) = span_lookahead_air_controls(
                        observations,
                        frame_idx,
                        car,
                        min_lookahead_z,
                        options,
                    )
                {
                    controls.pitch = solved.pitch;
                    controls.yaw = solved.yaw;
                    controls.roll = solved.roll;
                    air_controls_applied = true;
                }

                if !air_controls_applied
                    && airborne
                    && !dodge_jump_control
                    && active
                    && let Some((solved, lag)) =
                        past_persisted_air_controls(observations, frame_idx, car, min_lookahead_z)
                {
                    let keep = |axis: usize, value: f32| {
                        value * air_control_median_ratio(axis, lag, value.abs())
                    };
                    controls.pitch = keep(0, solved.pitch);
                    let steer_observed = car.inputs.steer.is_some();
                    if !steer_observed {
                        controls.yaw = keep(1, solved.yaw);
                        controls.roll = keep(2, solved.roll);
                    } else if controls.handbrake {
                        controls.roll = controls.steer;
                        controls.yaw = keep(1, solved.yaw);
                    } else {
                        controls.yaw = controls.steer;
                        controls.roll = keep(2, solved.roll);
                    }

                    air_controls_applied = true;
                }

                if options.input_fits {
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
                                if controls.handbrake {
                                    base.roll = controls.steer;
                                } else {
                                    base.yaw = controls.steer;
                                }

                                let ball_now = *arena.get_ball_state();
                                let fitted = if flip_fits && active && !new_lifetime {
                                    let (name, config) = slot_bodies
                                        .get(&slot)
                                        .copied()
                                        .unwrap_or(("octane", CarBodyConfig::OCTANE));
                                    let scratch = flip_scratch
                                        .entry(name)
                                        .or_insert_with(|| scratch_arena(options.seed, config));
                                    fit_flip_cancel(
                                        observations,
                                        options,
                                        &packet_lags,
                                        first_time,
                                        frame_idx,
                                        car,
                                        &state,
                                        &base,
                                        car_lag(car),
                                        arena.tick_count(),
                                        &ball_now,
                                        scratch,
                                    )
                                } else {
                                    None
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
                if !air_controls_applied && airborne {
                    if controls.handbrake {
                        controls.roll = controls.steer;
                    } else {
                        controls.yaw = controls.steer;
                    }
                }
                if dodge_jump_control {
                    controls.jump = true;
                    controls.pitch = dodge_pitch_control;
                    controls.yaw = dodge_yaw_control;
                    // The dodge direction is (-pitch, yaw + roll): a roll left over from the air
                    // controls would turn a double jump into a flip or rotate a dodge.
                    controls.roll = 0.0;
                }
                if options.input_fits
                    && airborne
                    && !dodge_jump_control
                    && !state.is_flipping
                    && active
                    && !new_lifetime
                    && !pending_dodges.iter().any(|dodge| dodge.slot == slot)
                    && flip_fits
                {
                    let (name, config) = slot_bodies
                        .get(&slot)
                        .copied()
                        .unwrap_or(("octane", CarBodyConfig::OCTANE));
                    let scratch = flip_scratch
                        .entry(name)
                        .or_insert_with(|| scratch_arena(options.seed, config));
                    let ball_now = *arena.get_ball_state();
                    if let Some(plan) = fit_dodge_start(
                        observations,
                        options,
                        &packet_lags,
                        first_time,
                        frame_idx,
                        car,
                        &state,
                        &controls,
                        car_lag(car),
                        arena.tick_count(),
                        &ball_now,
                        scratch,
                    ) {
                        let now = arena.tick_count();
                        fitted_arena.push((
                            slot,
                            "dodge",
                            now + plan.start_offset,
                            plan.pitch,
                            plan.yaw,
                            plan.cancel,
                            plan.activation_frame,
                        ));
                        pending_dodges.push(PendingDodge {
                            slot,
                            start_tick: now + plan.start_offset,
                            end_tick: now + plan.duration,
                            pitch: plan.pitch,
                            yaw: plan.yaw,
                            cancel: plan.cancel,
                            base: controls,
                        });
                        handled_dodges.insert((
                            car.actor_id,
                            car.actor_created_frame,
                            plan.activation_frame,
                        ));
                        if let Some((frame_b, lag)) = plan.first_packet {
                            lag_overrides
                                .borrow_mut()
                                .insert((car.actor_id, car.actor_created_frame, frame_b), lag);
                        }
                        diagnostics.dodge_starts_fitted += 1;
                        flip_last.insert((car.actor_id, car.actor_created_frame), plan.cancel);
                    }
                }
                // The jump control the car had at the end of the previous interval (the frame's own
                // controls replace it next).
                let previous_jump = arena.get_car_controls(slot).jump;
                arena.set_car_controls(slot, controls);
                if options.air_bvp
                    && simulated
                    && active
                    && !new_lifetime
                    && !dodge_jump_control
                    && car
                        .body
                        .position
                        .as_ref()
                        .is_some_and(|p| p.frame == frame.index)
                {
                    air_schedules.retain(|schedule| schedule.slot != slot);
                    let current = *arena.get_car_state(slot);
                    let (name, config) = slot_bodies
                        .get(&slot)
                        .copied()
                        .unwrap_or(("octane", CarBodyConfig::OCTANE));
                    let flip_or_press = current.is_flipping
                        || pending_dodges
                            .iter()
                            .any(|d| d.slot == slot && d.start_tick > arena.tick_count());
                    let planned = plan_air_bvp(
                        observations,
                        options,
                        &packet_lags,
                        first_time,
                        frame_idx,
                        car,
                        &current,
                        car_lag(car),
                        slot,
                        arena.tick_count(),
                        if flip_fits && flip_or_press {
                            Some(
                                flip_scratch
                                    .entry(name)
                                    .or_insert_with(|| scratch_arena(options.seed, config)),
                            )
                        } else {
                            None
                        },
                        &pending_dodges,
                        &lag_overrides.borrow(),
                    );
                    if planned.is_some() {
                        diagnostics.air_bvp_planned += 1;
                    } else if !current.is_on_ground {
                        diagnostics.air_bvp_refused += 1;
                    }
                    if let Some((schedule, shift)) = planned {
                        if shift != 0 {
                            // The solution moved the flip's start: a dodge press by the shift, a flip
                            // already running by its flip time.
                            if let Some(d) = pending_dodges
                                .iter_mut()
                                .find(|d| d.slot == slot && d.start_tick > arena.tick_count())
                            {
                                d.start_tick = (d.start_tick as i64 + i64::from(shift)) as u64;
                            } else if current.is_flipping {
                                let mut adjusted = current;
                                adjusted.flip_time =
                                    (adjusted.flip_time + shift as f32 / 120.0).max(0.0);
                                arena.set_car_state(slot, adjusted);
                            }
                        }
                        // Provenance: the interval this car's air controls were solved for (the span in
                        // ticks rides in the cancel slot of this tuple and becomes `span_ticks`).
                        fitted_arena.push((
                            slot,
                            "air",
                            arena.tick_count(),
                            0.0,
                            0.0,
                            (schedule.end_tick - arena.tick_count()) as f32,
                            0,
                        ));
                        air_schedules.push(schedule);
                    }
                }
                if options.input_fits
                    && simulated
                    && !new_lifetime
                    && car
                        .body
                        .position
                        .as_ref()
                        .is_some_and(|p| p.frame == frame.index)
                {
                    ground_schedules.retain(|schedule| schedule.slot != slot);
                    let (name, config) = slot_bodies
                        .get(&slot)
                        .copied()
                        .unwrap_or(("octane", CarBodyConfig::OCTANE));
                    let scratch = ground_scratch
                        .entry(name)
                        .or_insert_with(|| scratch_arena(options.seed, config));
                    let current = *arena.get_car_state(slot);
                    let mut schedule = options
                        .input_fits
                        .then(|| {
                            fit_ground_control_timing(
                                observations,
                                options,
                                &packet_lags,
                                first_time,
                                frame_idx,
                                car,
                                &current,
                                car_lag(car),
                                slot,
                                arena.tick_count(),
                                scratch,
                            )
                        })
                        .flatten();
                    if let Some(shift) = schedule.as_ref().and_then(|s| s.shift) {
                        car_shifts
                            .entry((car.actor_id, car.actor_created_frame))
                            .or_default()
                            .push(shift);
                    }
                    if schedule.is_none() && options.input_fits {
                        let ball_now = *arena.get_ball_state();
                        schedule = fit_jump_timing(
                            observations,
                            options,
                            &packet_lags,
                            first_time,
                            frame_idx,
                            car,
                            &current,
                            car_lag(car),
                            slot,
                            arena.tick_count(),
                            &ball_now,
                            scratch,
                        );
                    }
                    if schedule.is_none()
                        && options.input_fits
                        && !pending_dodges.iter().any(|dodge| dodge.slot == slot)
                    {
                        let ball_now = *arena.get_ball_state();
                        if let Some(flip) = fit_ground_flip_timing(
                            observations,
                            options,
                            &packet_lags,
                            first_time,
                            frame_idx,
                            car,
                            &current,
                            car_lag(car),
                            slot,
                            arena.tick_count(),
                            &ball_now,
                            scratch,
                        ) {
                            if let Some(plan) = flip.dodge {
                                let now = arena.tick_count();
                                fitted_arena.push((
                                    slot,
                                    "dodge",
                                    now + plan.start_offset,
                                    plan.pitch,
                                    plan.yaw,
                                    plan.cancel,
                                    plan.activation_frame,
                                ));
                                pending_dodges.push(PendingDodge {
                                    slot,
                                    start_tick: now + plan.start_offset,
                                    end_tick: now + plan.duration,
                                    pitch: plan.pitch,
                                    yaw: plan.yaw,
                                    cancel: plan.cancel,
                                    base: controls,
                                });
                                handled_dodges.insert((
                                    car.actor_id,
                                    car.actor_created_frame,
                                    plan.activation_frame,
                                ));
                                diagnostics.dodge_starts_fitted += 1;
                                flip_last
                                    .insert((car.actor_id, car.actor_created_frame), plan.cancel);
                            }
                            if let Some((frame_b, lag)) = flip.first_packet {
                                lag_overrides
                                    .borrow_mut()
                                    .insert((car.actor_id, car.actor_created_frame, frame_b), lag);
                            }
                            schedule = Some(flip.schedule);
                        }
                    }
                    if let Some(schedule) = schedule {
                        for tick in jump_press_ticks(previous_jump, &schedule) {
                            fitted_arena.push((slot, "jump", tick, 0.0, 0.0, 0.0, 0));
                        }
                        ground_schedules.push(schedule);
                    }
                }
            }
        }
        advance_to!(span);
        let _ = remaining;
        if options.block_sim_pad_pickups {
            // The pads the replay has not picked up recharge in real time; the arena's own copy is
            // held on cooldown only to keep the simulation from picking them up.
            let elapsed = gap as f32 / 120.0;
            for cooldown in pad_cooldowns.iter_mut() {
                *cooldown = (*cooldown - elapsed).max(0.0);
            }
        }
        if simulated && !frame_withheld {
            for event in &frame.events {
                let observations::Event::Demolish {
                    source,
                    victim_car: Some(victim),
                    repeat,
                    ..
                } = event
                else {
                    continue;
                };
                // A repeated report would demolish a respawned car a second time; a victim actor
                // that is gone from the frame has been replaced. A goal-explosion victim is a dead pawn
                // shell: it stays demolished until the slot's next lifetime or live packet (`dead_shells`).
                let goal_explosion = *source == "goal_explosion";
                if (!goal_explosion && *repeat) || !frame.cars.iter().any(|c| c.actor_id == *victim)
                {
                    continue;
                }
                let Some(&(slot, created)) = actor_slots.get(victim) else {
                    continue;
                };
                // The slot belongs to this car actor's lifetime, not to an earlier owner of the id,
                // and the car must still be its player's primary car (a shadowed older car of the
                // same player does not own the slot).
                let Some(victim_car) = frame_cars.iter().find(|c| {
                    c.actor_id == *victim
                        && c.actor_created_frame == created
                        && c.player_key.is_some()
                }) else {
                    continue;
                };
                // A goal explosion starts a hold only for a body that is not a live car in this frame (pawn
                // link inactive, or sleeping): a car that is still driven keeps simulating.
                let sleeping = victim_car.body.sleeping.as_ref().is_some_and(|s| s.value);
                if goal_explosion && victim_car.player_link_active && !sleeping {
                    continue;
                }
                // An observed demolition of a car still held on its spawn pose is a normal demolition: exported as
                // demolished, with the normal hold rules.
                if spawn_held.remove(&slot).is_some() {
                    spawn_demolished.insert((*victim, created));
                }
                let mut state = *arena.get_car_state(slot);
                if !state.is_demoed || goal_explosion {
                    state.is_demoed = true;
                    state.demo_respawn_timer = 3.0;
                    arena.set_car_state(slot, state);
                    arena.refresh_car_sticky_gate(slot);
                }
                if goal_explosion {
                    // Counted once per hold (the replay re-sends the event); a hold already begun by a sleeping
                    // packet becomes observed.
                    let new_hold = !dead_shells.get(&slot).is_some_and(|&(a, c, source)| {
                        (a, c) == (*victim, created) && source == "observed"
                    });
                    dead_shells.insert(slot, (*victim, created, "observed"));
                    if new_hold {
                        diagnostics.goal_explosion_demolitions += 1;
                    }
                } else {
                    demo_hold_until.insert(slot, timeline_tick + 360);
                    // A demolished car whose pawn is dead (no active link, or sleeping) stays out of the
                    // simulation until the slot's next lifetime or live packet, not just 3 s.
                    if (!victim_car.player_link_active || sleeping)
                        && !dead_shells
                            .get(&slot)
                            .is_some_and(|&(a, c, _)| (a, c) == (*victim, created))
                    {
                        dead_shells.insert(slot, (*victim, created, "observed"));
                        diagnostics.dead_shells_after_demolition += 1;
                    }
                }
            }
        }
        if simulated && !frame_withheld {
            for event in &frame.events {
                let observations::Event::DodgeRefreshed { car, .. } = event else {
                    continue;
                };
                let Some(&(slot, created)) = actor_slots.get(car) else {
                    continue;
                };
                if !frame_cars.iter().any(|c| {
                    c.actor_id == *car && c.actor_created_frame == created && c.player_key.is_some()
                }) {
                    continue;
                }
                diagnostics.dodge_refreshes_observed += 1;
                let mut state = *arena.get_car_state(slot);
                if state.is_demoed
                    || !(state.has_jumped
                        || state.has_double_jumped
                        || state.has_flipped
                        || state.is_flipping)
                {
                    continue;
                }
                // What RocketSim does on the tick a wheel touches something (`update_double_jump_or_flip`,
                // and the re-arm of the jump on landing).
                state.has_jumped = false;
                state.has_double_jumped = false;
                state.has_flipped = false;
                state.is_flipping = false;
                state.is_jumping = false;
                state.flip_time = 0.0;
                state.air_time = 0.0;
                state.air_time_since_jump = 0.0;
                // `set_car_state` overwrites the controls: keep them.
                let controls = *arena.get_car_controls(slot);
                arena.set_car_state(slot, state);
                arena.set_car_controls(slot, controls);
                diagnostics.dodge_refreshes_applied += 1;
            }
        }
        for pickup in &frame.pad_pickups {
            let by_name = pickup
                .pad_actor_name
                .as_ref()
                .and_then(|name| pad_name_to_index.get(name))
                .copied();
            if let Some(idx) = by_name {
                pad_actor_to_index.insert(pickup.pad_actor_id, idx);
            }
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

            if changed && let Some(idx) = pad_idx {
                if pickup.picked_up == 255 {
                    arena.set_boost_pad_state(idx, BoostPadState { cooldown: 0.0 });
                    pad_cooldowns[idx] = 0.0;
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
                    pad_cooldowns[idx] = max_cooldown;
                }
            }
        }

        if options.block_sim_pad_pickups {
            // Write the true pad cooldowns (decayed at the start of this frame, then updated by this
            // frame's pickups) back into the arena for the export.
            for (idx, cooldown) in pad_cooldowns.iter().enumerate() {
                arena.set_boost_pad_state(
                    idx,
                    BoostPadState {
                        cooldown: *cooldown,
                    },
                );
            }
        }
        let timeline_offset = timeline_tick as i64 - arena.tick_count() as i64;
        if options.block_sim_pad_pickups {
            // The pads are held on cooldown (the replay reports the pickups): a pickup the
            // simulation still reports would count one twice.
            events.retain(|e| !matches!(e.event, ArenaEvent::CarPickupBoost(_)));
        }
        let mut touches = Vec::new();
        for e in &events {
            if let ArenaEvent::CarHitBall(hit) = &e.event {
                let new_contact = last_contact_tick
                    .get(&hit.car_idx)
                    .is_none_or(|&last| e.arena_tick > last + 2);
                last_contact_tick.insert(hit.car_idx, e.arena_tick);
                if new_contact {
                    touches.push(TouchEvent {
                        car_slot: hit.car_idx,
                        tick: (e.arena_tick as i64 + timeline_offset).max(0) as u64,
                        contact_point: hit.contact_point.to_array(),
                    });
                }
            }
        }
        // Contacts found from the ball packets that end at this frame.
        let mut ball_contacts = Vec::new();
        {
            let state_now = arena.get_arena_state();
            let slot_life: HashMap<usize, usize> = frame_cars
                .iter()
                .filter_map(|car| match actor_slots.get(&car.actor_id) {
                    Some(&(slot, created)) if created == car.actor_created_frame => {
                        Some((slot, created))
                    }
                    _ => None,
                })
                .collect();
            recent_poses.push_back((
                timeline_tick,
                state_now
                    .cars
                    .iter()
                    .enumerate()
                    .map(|(i, c)| {
                        let life = slot_life.get(&i).copied().unwrap_or(usize::MAX);
                        (i, c.1.phys.pos, c.1.phys.rot_mat, c.1.is_demoed, life)
                    })
                    .collect(),
            ));
            while recent_poses.len() > 8 {
                recent_poses.pop_front();
            }
            for t in &touches {
                recent_touch_ticks.push_back(t.tick);
            }
            while recent_touch_ticks.len() > 24 {
                recent_touch_ticks.pop_front();
            }
            if let Some(interval) = contact_intervals.get(&frame_idx) {
                let (tick_from, tick_to) =
                    (interval.tick_a.max(0) as u64, interval.tick_b.max(0) as u64);
                let pose_at = |slot: usize, tick: u64| -> Option<(Vec3A, Mat3A, bool)> {
                    let at = |k: usize| {
                        recent_poses.get(k).and_then(|(t, cars)| {
                            cars.iter()
                                .find(|c| c.0 == slot)
                                .map(|c| (*t, c.1, c.2, c.3, c.4))
                        })
                    };
                    let n = recent_poses.len();
                    let after = (0..n).find(|&k| recent_poses[k].0 >= tick)?;
                    let (t1, p1, r1, d1, life1) = at(after)?;
                    if after == 0 || t1 == tick {
                        return Some((p1, r1, d1));
                    }
                    let (t0, p0, r0, _, life0) = at(after - 1)?;
                    // A pose between two lifetimes of the slot's car (a respawn) is unknown.
                    if life0 != life1 {
                        return None;
                    }
                    let f = (tick - t0) as f32 / (t1 - t0).max(1) as f32;
                    // The rotation is interpolated along the shortest arc (a flipping car turns a lot in a frame).
                    let rotation =
                        Mat3A::from_quat(Quat::from_mat3a(&r0).slerp(Quat::from_mat3a(&r1), f));
                    Some((p0 + (p1 - p0) * f, rotation, d1))
                };
                let mut best: Option<(u64, usize, f32)> = None; // tick, slot, gap
                'ticks: for (k, ball_pos) in interval.path.iter().enumerate() {
                    let tick = tick_from + k as u64 + 1;
                    let ball = Vec3A::from(*ball_pos);
                    let mut here: Option<(usize, f32)> = None;
                    for (&slot, &(_, config)) in slot_bodies.iter() {
                        let Some((pos, rot, demoed)) = pose_at(slot, tick) else {
                            continue;
                        };
                        if demoed {
                            continue;
                        }
                        let local = rot.transpose() * (ball - pos) - config.hitbox_pos_offset;
                        let q = local.abs() - config.hitbox_size * 0.5;
                        let gap = q.max(Vec3A::ZERO).length() + q.max_element().min(0.0) - 91.25;
                        if here.is_none_or(|(_, g)| gap < g) {
                            here = Some((slot, gap));
                        }
                    }
                    if let Some((slot, gap)) = here {
                        if best.is_none_or(|(_, _, g)| gap < g) {
                            best = Some((tick, slot, gap));
                        }
                        if gap <= 0.0 {
                            best = Some((tick, slot, gap));
                            break 'ticks;
                        }
                    }
                }
                let simulated_touch = recent_touch_ticks
                    .iter()
                    .any(|&t| t + 2 >= tick_from && t <= tick_to + 2);
                let near = best.filter(|&(_, _, gap)| gap <= 150.0);
                ball_contacts.push(BallContact {
                    frame_a: interval.frame_a,
                    tick: near.map_or((tick_from + tick_to) / 2, |b| b.0),
                    tick_from,
                    tick_to,
                    car_slot: near.map(|b| b.1),
                    gap_uu: near.map(|b| b.2),
                    velocity_residual: interval.velocity_residual,
                    simulated_touch,
                });
            }
        }
        // New pad pickups of this frame, with the instigator checked against the cars' paths.
        let mut boost_pickups = Vec::new();
        for pickup in &frame.pad_pickups {
            if pickup.repeat || pickup.picked_up == 255 || pickup.instigator_car_id.is_none() {
                continue;
            }
            let pad_index = pad_actor_to_index.get(&pickup.pad_actor_id).copied();
            let (pad_pos, is_big) = match pad_index {
                Some(idx) => {
                    let config = arena.get_boost_pad_config(idx);
                    (Some(config.pos), Some(config.is_big))
                }
                None => (None, None),
            };
            let car_slot = pickup
                .instigator_car_id
                .and_then(|id| actor_slots.get(&id).map(|&(slot, _)| slot));
            // Closest approach of a slot's path to the pad over the recent frames: (distance in the
            // horizontal plane, tick).
            let closest = |slot: usize| -> Option<(f32, u64)> {
                let pad = pad_pos?;
                let mut best: Option<(f32, u64)> = None;
                let mut previous: Option<(u64, Vec3A)> = None;
                for (tick, cars) in recent_poses.iter() {
                    let Some(&(_, pos, _, demoed, _)) = cars.iter().find(|c| c.0 == slot) else {
                        previous = None;
                        continue;
                    };
                    if demoed {
                        previous = None;
                        continue;
                    }
                    if let Some((t0, p0)) = previous {
                        // Distance from the pad to the segment p0 -> pos in the horizontal plane.
                        let (a, b) = (glam::Vec2::new(p0.x, p0.y), glam::Vec2::new(pos.x, pos.y));
                        let target = glam::Vec2::new(pad.x, pad.y);
                        let ab = b - a;
                        let f = if ab.length_squared() > 1e-6 {
                            ((target - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0)
                        } else {
                            0.0
                        };
                        let d = (a + ab * f - target).length();
                        if best.is_none_or(|(bd, _)| d < bd) {
                            best = Some((d, t0 + ((*tick - t0) as f32 * f) as u64));
                        }
                    } else {
                        let d = (glam::Vec2::new(pos.x, pos.y) - glam::Vec2::new(pad.x, pad.y))
                            .length();
                        if best.is_none_or(|(bd, _)| d < bd) {
                            best = Some((d, *tick));
                        }
                    }
                    previous = Some((*tick, pos));
                }
                best
            };
            // The trigger radius plus half a car (the game tests the hitbox, RocketSim the centre).
            let radius = if is_big == Some(true) { 208.0 } else { 144.0 } + 60.0;
            let own = car_slot.and_then(&closest);
            let verified = own.is_some_and(|(d, _)| d <= radius);
            let mut suggested = None;
            if !verified && pad_pos.is_some() {
                let mut best: Option<(f32, usize, u64)> = None;
                for &slot in slot_bodies.keys() {
                    if Some(slot) == car_slot {
                        continue;
                    }
                    if let Some((d, t)) = closest(slot)
                        && d <= radius
                        && best.is_none_or(|(bd, _, _)| d < bd)
                    {
                        best = Some((d, slot, t));
                    }
                }
                suggested = best.map(|(_, slot, t)| (slot, t));
            }
            boost_pickups.push(BoostPickup {
                pad_index,
                pad_actor_id: pickup.pad_actor_id,
                is_big,
                car_slot,
                verified,
                distance_uu: own.map(|o| o.0),
                suggested_car_slot: suggested.map(|s| s.0),
                tick: if verified {
                    own.map_or(timeline_tick, |o| o.1)
                } else {
                    suggested.map_or(timeline_tick, |s| s.1)
                },
            });
        }
        let scoreboard_frame = scoreboard.get(frame_idx).cloned().map(|mut sb| {
            // After expiry the first floor contact of the ball (the simulation's) decides the
            // game, also when no fresh ball packet caught it.
            if sb.clock_state == "expired" || sb.clock_state == "decided" {
                if events.iter().any(
                    |e| matches!(&e.event, ArenaEvent::BallHitWorld(h) if h.contact_normal.z > 0.9),
                ) {
                    ball_decided = true;
                }
                if ball_decided {
                    sb.clock_state = "decided";
                }
            } else {
                ball_decided = false;
            }
            sb
        });
        let car_actor_slots: Vec<(i32, usize)> = frame
            .cars
            .iter()
            .filter_map(|car| {
                let slot = *slots.get(car.player_key.as_ref()?)?;
                Some((car.actor_id, slot))
            })
            .collect();
        let ball_fresh = frame
            .ball
            .as_ref()
            .and_then(|body| body.position.as_ref())
            .is_some_and(|p| p.frame == frame.index);
        let mut fresh_car_slots: Vec<usize> = observations::primary_linked_cars(frame)
            .into_iter()
            .filter(|car| {
                car.body
                    .position
                    .as_ref()
                    .is_some_and(|p| p.frame == frame.index)
            })
            .filter_map(|car| slots.get(car.player_key.as_ref()?).copied())
            .collect();
        fresh_car_slots.sort_unstable();
        fresh_car_slots.dedup();
        let converted = ConvertedFrame {
            replay_frame: frame.index,
            replay_time: frame.time,
            timeline_tick,
            state: {
                let mut exported = arena.get_arena_state();
                // A car held out of collisions on its spawn pose is not demolished.
                for (info, car) in exported.cars.iter_mut() {
                    if spawn_held.contains_key(&info.idx) {
                        car.is_demoed = false;
                        car.demo_respawn_timer = 0.0;
                    }
                }
                exported
            },
            simulated_events: events,
            touches,
            ball_contacts,
            boost_pickups,
            scoreboard: scoreboard_frame,
            packet_lags: applied_lags,
            fitted_inputs: fitted_arena
                .into_iter()
                .map(|(slot, kind, tick, pitch, yaw, cancel, activation_frame)| {
                    let air = kind == "air";
                    FittedInput {
                        slot,
                        activation_frame,
                        kind,
                        tick: (tick as i64 + timeline_offset).max(0) as u64,
                        pitch,
                        yaw,
                        cancel: if air { 0.0 } else { cancel },
                        span_ticks: air.then_some(cancel as u64),
                    }
                })
                .collect(),
            car_actor_slots,
            sleeping_velocity_inferred: sleeping_inferred,
            demolition_inferred,
            spawn_pose_held: {
                let mut held: Vec<usize> = spawn_held.keys().copied().collect();
                held.sort_unstable();
                held
            },
            dead_shells_held: {
                let mut held: Vec<DeadShellHold> = dead_shells
                    .iter()
                    .map(|(&slot, &(_, _, source))| DeadShellHold { slot, source })
                    .collect();
                held.sort_by_key(|h| h.slot);
                held
            },
            ball_fresh,
            fresh_car_slots,
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

    /// A jump held since the previous interval continues across a schedule's first entry; only a rising
    /// edge after a release is a press.
    #[test]
    fn a_jump_press_is_a_rising_edge_from_the_previous_interval() {
        let entry = |tick: u64, jump: Option<bool>| (tick, 1.0, 0.0, false, false, jump);
        let schedule = GroundSchedule {
            slot: 0,
            end_tick: 20,
            entries: vec![
                entry(11, Some(true)),
                entry(13, None),
                entry(15, Some(false)),
                entry(18, Some(true)),
            ],
            shift: None,
        };
        assert_eq!(jump_press_ticks(false, &schedule), vec![11, 18]);
        assert_eq!(jump_press_ticks(true, &schedule), vec![18]);
    }

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
    fn jump_counter_inference_is_gated_on_an_observed_impulse() {
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
            spawn_pose: None,
            inputs: observations::Inputs {
                jump_active_raw: Some(Value {
                    value: 1,
                    frame: 1,
                    source: Source::Replay,
                }),
                ..observations::Inputs::default()
            },
        };
        assert!(controls_from_observation(&car).jump);
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
        assert!(!controls_from_observation(&car).jump);
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
            spawn_pose: None,
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
    fn ball_hit_interval_recovers_the_ticks_between_packets() {
        // A ball in free flight, hit at tick 7 (its velocity changes by 1500 UU/s), seen at ticks 3
        // and 14: the two exact paths meet at the hit, so the interval between the packets is 11.
        let mut pos = [100.0f32, -300.0, 600.0];
        let mut vel = [1200.0f32, 200.0, 300.0];
        let mut packets = Vec::new();
        for tick in 0..=14 {
            if tick == 3 || tick == 14 {
                packets.push(ChainPacket {
                    frame: tick,
                    pos,
                    vel,
                });
            }
            if tick == 7 {
                vel = [vel[0] - 900.0, vel[1] + 1100.0, vel[2] + 400.0];
            }
            (pos, vel) = ball_free_step(pos, vel);
        }
        let (a, b) = (&packets[0], &packets[1]);
        let (d, ticks_after_a, _) = ball_hit_interval_ticks(a, b, 6, 16).expect("hit interval");
        assert_eq!(d, 11);
        assert_eq!(ticks_after_a, 4);
        // The step and its inverse agree.
        let (p, v) = ball_free_step(a.pos, a.vel);
        let (p0, v0) = ball_free_step_back(p, v);
        for i in 0..3 {
            assert!((p0[i] - a.pos[i]).abs() < 1e-2 && (v0[i] - a.vel[i]).abs() < 1e-2);
        }
    }

    /// Two runs share frame 5: the first has three packets, the second two. `Earlier` gives the shared
    /// packet to the first run; `Later` keeps every entry (the second run overwrites the first).
    #[test]
    fn a_packet_shared_by_two_runs_has_one_owner() {
        let run = |frames: &[usize]| RawRun {
            entries: frames.iter().map(|&f| (f, 0)).collect(),
            lo: 0,
            hi: 0,
            start: 0,
        };
        let runs = [run(&[3, 4, 5]), run(&[5, 6]), run(&[8, 9])];
        assert_eq!(
            owned_entries(&runs),
            vec![vec![true; 3], vec![false, true], vec![true; 2]]
        );
        // A short run that follows a long one still loses its first packet.
        let runs = [run(&[3, 4]), run(&[4, 5, 6])];
        assert_eq!(
            owned_entries(&runs),
            vec![vec![true; 2], vec![false, true, true]]
        );
    }

    /// A hit tick moved between arenas keeps its age before the destination's tick; a hit from the future of
    /// its source, or older than the destination's clock reaches back, is dropped.
    #[test]
    fn a_fresh_sleeping_packet_zeroes_the_simulated_velocity_unless_a_velocity_is_fresh() {
        let sleeping = |frame: usize, linear_frame: Option<usize>| Body {
            sleeping: Some(Value {
                value: true,
                frame,
                source: Source::Replay,
            }),
            linear_velocity: linear_frame.map(|frame| Value {
                value: [1.0, 2.0, 3.0],
                frame,
                source: Source::Replay,
            }),
            ..Body::default()
        };
        let mut state = CarState::default().phys;
        state.vel = Vec3A::new(500.0, 0.0, 0.0);
        state.ang_vel = Vec3A::new(0.0, 2.0, 0.0);
        // A stale sleeping flag does nothing.
        assert_eq!(
            zero_sleeping_velocity(&mut state, &sleeping(3, None), 4),
            None
        );
        assert_eq!(state.vel, Vec3A::new(500.0, 0.0, 0.0));
        // A fresh one zeroes both (the omitted velocities are not observed).
        assert_eq!(
            zero_sleeping_velocity(&mut state, &sleeping(4, None), 4),
            Some(true)
        );
        assert_eq!((state.vel, state.ang_vel), (Vec3A::ZERO, Vec3A::ZERO));
        // One at zero velocity already is still a sleeping packet, with nothing to change.
        assert_eq!(
            zero_sleeping_velocity(&mut state, &sleeping(4, None), 4),
            Some(false)
        );
        // A velocity that is fresh in the same packet is kept.
        state.vel = Vec3A::new(7.0, 0.0, 0.0);
        zero_sleeping_velocity(&mut state, &sleeping(4, Some(4)), 4);
        assert_eq!(state.vel, Vec3A::new(7.0, 0.0, 0.0));
    }

    /// A sleeping packet of a car whose pawn link is inactive is a dead pawn shell: demolished (inferred) until
    /// its next live packet, with no 3 s timer. A car that merely sleeps (link active) is never touched.
    #[test]
    fn a_sleeping_car_with_no_active_link_is_a_demolished_shell_until_a_live_packet() {
        fn value<T>(value: T, frame: usize) -> Value<T> {
            Value {
                value,
                frame,
                source: Source::Replay,
            }
        }
        // Frame 0: live packet; frame 1: sleeping packet; frames 2-3 no packet; frame 4: live packet again.
        let replay = |link_after_death: bool| {
            let frames = (0..5)
                .map(|index| {
                    let packet = if index == 4 { 4 } else { index.min(1) };
                    let sleeping = packet == 1;
                    let alive = index == 0 || index == 4;
                    observations::Frame {
                        index,
                        time: index as f32 * 0.033,
                        delta: 0.033,
                        ball: None,
                        cars: vec![observations::Car {
                            actor_id: 1,
                            actor_created_frame: 0,
                            player_key: Some("p1".to_string()),
                            player_link_active: alive || link_after_death,
                            team: Some(0),
                            body_product_id: None,
                            body: Body {
                                position: Some(value(
                                    [0.0, 0.0, 300.0 + 10.0 * packet as f32],
                                    packet,
                                )),
                                rotation_xyzw: Some(value([0.0, 0.0, 0.0, 1.0], packet)),
                                linear_velocity: (!sleeping)
                                    .then(|| value([200.0, 0.0, 0.0], packet)),
                                sleeping: Some(value(sleeping, packet)),
                                ..Body::default()
                            },
                            boost: None,
                            boost_raw: None,
                            spawn_pose: None,
                            inputs: observations::Inputs::default(),
                        }],
                        players: Vec::new(),
                        team_scores: [None, None],
                        seconds_remaining: None,
                        overtime: None,
                        game_state: Some(value("Active".to_string(), index)),
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
        let options = ConvertOptions::default();
        let demoed = |output: &ConversionOutput| -> Vec<bool> {
            output
                .frames
                .iter()
                .map(|f| f.state.cars[0].1.is_demoed)
                .collect()
        };
        let shell = convert_observations(replay(false), &options).unwrap();
        assert_eq!(demoed(&shell), [false, true, true, true, false]);
        assert_eq!(shell.frames[1].demolition_inferred, vec![1]);
        assert!(shell.frames[2].demolition_inferred.is_empty());
        // The sleeping car with an active link is never demolished or marked.
        let live = convert_observations(replay(true), &options).unwrap();
        assert_eq!(demoed(&live), [false; 5]);
        assert!(live.frames.iter().all(|f| f.demolition_inferred.is_empty()));
        assert_eq!(live.frames[1].sleeping_velocity_inferred, vec![Some(1)]);
    }

    /// The dead-shell hold from a goal-explosion report: observed, counted once per hold (the replay re-sends
    /// the event), held every frame until a live packet; not started for a live car, for a shadowed older car
    /// of the player, or in a withheld frame; ended by the slot's next lifetime.
    #[test]
    fn goal_explosion_holds_start_only_for_dead_shells_and_end_with_a_lifetime_or_live_packet() {
        fn value<T>(value: T, frame: usize) -> Value<T> {
            Value {
                value,
                frame,
                source: Source::Replay,
            }
        }
        // (actor, created, link active, frame of its latest packet, that packet is a sleeping one)
        type Spec = (i32, usize, bool, usize, bool);
        let goal = |victim: i32| observations::Event::Demolish {
            source: "goal_explosion",
            attacker_car: None,
            victim_car: Some(victim),
            attacker_pri: None,
            self_demolish: false,
            attacker_velocity: [0.0; 3],
            victim_velocity: [0.0; 3],
            repeat: false,
        };
        let build = |frames: Vec<(Vec<Spec>, Vec<observations::Event>)>| {
            let frames = frames
                .into_iter()
                .enumerate()
                .map(|(index, (cars, events))| observations::Frame {
                    index,
                    time: index as f32 * 0.033,
                    delta: 0.033,
                    ball: None,
                    cars: cars
                        .into_iter()
                        .map(
                            |(actor, created, link, packet, sleeping)| observations::Car {
                                actor_id: actor,
                                actor_created_frame: created,
                                player_key: Some("p1".to_string()),
                                player_link_active: link,
                                team: Some(0),
                                body_product_id: None,
                                body: Body {
                                    position: Some(value(
                                        [0.0, 100.0 * actor as f32, 300.0],
                                        packet,
                                    )),
                                    rotation_xyzw: Some(value([0.0, 0.0, 0.0, 1.0], packet)),
                                    linear_velocity: (!sleeping)
                                        .then(|| value([200.0, 0.0, 0.0], packet)),
                                    sleeping: Some(value(sleeping, packet)),
                                    ..Body::default()
                                },
                                boost: None,
                                boost_raw: None,
                                spawn_pose: None,
                                inputs: observations::Inputs::default(),
                            },
                        )
                        .collect(),
                    players: Vec::new(),
                    team_scores: [None, None],
                    seconds_remaining: None,
                    overtime: None,
                    game_state: Some(value("Active".to_string(), index)),
                    events,
                    pad_pickups: Vec::new(),
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
        let options = ConvertOptions::default();
        let counts = |frames: Vec<(Vec<Spec>, Vec<observations::Event>)>,
                      options: &ConvertOptions| {
            let mut converted = Vec::new();
            let summary = convert_observations_with(&build(frames), options, |frame, _, _| {
                converted.push(frame.clone());
                Ok(())
            })
            .unwrap();
            (converted, summary.diagnostics.goal_explosion_demolitions)
        };
        let shell = |sleep_frame: usize| (1, 0, false, sleep_frame, true);
        let live = |packet: usize| (1, 0, true, packet, false);

        // 1. The goal-explosion path: a sleeping unlinked body with the report (re-sent in frame 2) is held
        // as observed in every frame until the live packet of frame 4; one hold, one count, no inference.
        let (frames, goal_holds) = counts(
            vec![
                (vec![live(0)], vec![]),
                (vec![shell(1)], vec![goal(1)]),
                (vec![shell(1)], vec![goal(1)]),
                (vec![shell(1)], vec![]),
                (vec![live(4)], vec![]),
            ],
            &options,
        );
        let demoed_of = |frames: &[ConvertedFrame]| -> Vec<bool> {
            frames.iter().map(|f| f.state.cars[0].1.is_demoed).collect()
        };
        let held_of = |frames: &[ConvertedFrame]| -> Vec<Vec<&'static str>> {
            frames
                .iter()
                .map(|f| f.dead_shells_held.iter().map(|h| h.source).collect())
                .collect()
        };
        assert_eq!(demoed_of(&frames), [false, true, true, true, false]);
        assert_eq!(
            held_of(&frames),
            [
                vec![],
                vec!["observed"],
                vec!["observed"],
                vec!["observed"],
                vec![]
            ]
        );
        assert_eq!(frames[1].dead_shells_held[0].slot, 0);
        assert!(frames.iter().all(|f| f.demolition_inferred.is_empty()));
        assert_eq!(
            goal_holds, 1,
            "one hold is one count, however often the event is re-sent"
        );

        // 2. A goal-explosion report for a live car (link active, not sleeping) starts no hold.
        let (frames, goal_holds) = counts(
            vec![
                (vec![live(0)], vec![]),
                (vec![live(1)], vec![goal(1)]),
                (vec![live(2)], vec![]),
            ],
            &options,
        );
        assert_eq!(demoed_of(&frames), [false; 3]);
        assert!(held_of(&frames).iter().all(Vec::is_empty));
        assert_eq!(goal_holds, 0);

        // 3. A shell held by an inference ends at the slot's next lifetime (actor 2, created in frame 3).
        let (frames, _) = counts(
            vec![
                (vec![live(0)], vec![]),
                (vec![shell(1)], vec![]),
                (vec![shell(1)], vec![]),
                (vec![(2, 3, true, 3, false)], vec![]),
                (vec![(2, 3, true, 4, false)], vec![]),
            ],
            &options,
        );
        assert_eq!(
            held_of(&frames),
            [vec![], vec!["inferred"], vec!["inferred"], vec![], vec![]]
        );
        assert_eq!(demoed_of(&frames), [false, true, true, false, false]);
        assert_eq!(frames[1].demolition_inferred, vec![1]);

        // 4. A goal-explosion report for a shadowed older car (a live replacement of the same player exists)
        // holds nothing: the slot belongs to the replacement.
        let old = (1, 0, false, 0, true);
        let new = |packet: usize| (2, 1, true, packet, false);
        let (frames, goal_holds) = counts(
            vec![
                (vec![old, new(1)], vec![]),
                (vec![old, new(1)], vec![]),
                (vec![old, new(2)], vec![goal(1)]),
                (vec![old, new(3)], vec![]),
            ],
            &options,
        );
        assert_eq!(demoed_of(&frames), [false; 4]);
        assert!(held_of(&frames).iter().all(Vec::is_empty));
        assert_eq!(goal_holds, 0);

        // 5. No hold starts in a withheld frame (the report and the sleeping packet are both ignored there).
        let mut withheld_options = ConvertOptions::default();
        withheld_options.withheld_frames = Some(Arc::new(vec![false, true, false, false]));
        let (frames, goal_holds) = counts(
            vec![
                (vec![live(0)], vec![]),
                (vec![shell(1)], vec![goal(1)]),
                (vec![shell(1)], vec![]),
                (vec![shell(1)], vec![]),
            ],
            &withheld_options,
        );
        assert!(frames[1].dead_shells_held.is_empty() && frames[1].demolition_inferred.is_empty());
        assert_eq!(goal_holds, 0);
    }

    /// A shell whose sleeping packet comes with the report of its demolition is not held from the packet on (the
    /// bump of the demolition, which also slows the attacker, must still be simulated): the demolition is applied
    /// after the frame's interval and the hold starts there, observed, until the live packet.
    #[test]
    fn a_demolished_shell_is_held_after_the_interval_not_from_its_packet() {
        for (shell_frame_state, expected_source) in
            [("Active", "observed"), ("PostGoalScored", "inferred")]
        {
            fn value<T>(value: T, frame: usize) -> Value<T> {
                Value {
                    value,
                    frame,
                    source: Source::Replay,
                }
            }
            let demolish = observations::Event::Demolish {
                source: "extended",
                attacker_car: Some(2),
                victim_car: Some(1),
                attacker_pri: None,
                self_demolish: false,
                attacker_velocity: [0.0; 3],
                victim_velocity: [0.0; 3],
                repeat: false,
            };
            let car = |packet: usize, link: bool, sleeping: bool| observations::Car {
                actor_id: 1,
                actor_created_frame: 0,
                player_key: Some("p1".to_string()),
                player_link_active: link,
                team: Some(0),
                body_product_id: None,
                body: Body {
                    position: Some(value([0.0, 0.0, 300.0], packet)),
                    rotation_xyzw: Some(value([0.0, 0.0, 0.0, 1.0], packet)),
                    linear_velocity: (!sleeping).then(|| value([200.0, 0.0, 0.0], packet)),
                    sleeping: Some(value(sleeping, packet)),
                    ..Body::default()
                },
                boost: None,
                boost_raw: None,
                spawn_pose: None,
                inputs: observations::Inputs::default(),
            };
            let frames = (0..4)
                .map(|index| observations::Frame {
                    index,
                    time: index as f32 * 0.033,
                    delta: 0.033,
                    ball: None,
                    cars: vec![match index {
                        0 => car(0, true, false),
                        3 => car(3, true, false),
                        _ => car(1, false, true),
                    }],
                    players: Vec::new(),
                    team_scores: [None, None],
                    seconds_remaining: None,
                    overtime: None,
                    game_state: Some(value(
                        if index == 1 {
                            shell_frame_state
                        } else {
                            "Active"
                        }
                        .to_string(),
                        index,
                    )),
                    events: if index == 1 {
                        vec![demolish.clone()]
                    } else {
                        Vec::new()
                    },
                    pad_pickups: Vec::new(),
                })
                .collect();
            let replay = ObservedReplay {
                header: observations::Header {
                    game_type: "TAGame.Replay_Soccar_TA".to_string(),
                    levels: Vec::new(),
                    final_team_scores: [None, None],
                },
                frames,
                diagnostics: Default::default(),
            };
            let mut converted = Vec::new();
            let summary =
                convert_observations_with(&replay, &ConvertOptions::default(), |frame, _, _| {
                    converted.push(frame.clone());
                    Ok(())
                })
                .unwrap();
            // Held from the end of frame 1 through frame 2, released by the live packet. In a simulated frame the
            // demolition handler holds it after the interval (observed); in a frame that is not simulated (a goal
            // replay, here `PostGoalScored`) no handler runs, so the sleeping packet starts the hold (inferred).
            let held: Vec<usize> = converted.iter().map(|f| f.dead_shells_held.len()).collect();
            assert_eq!(held, [0, 1, 1, 0], "{shell_frame_state}");
            assert_eq!(converted[1].dead_shells_held[0].source, expected_source);
            if expected_source == "observed" {
                assert!(converted.iter().all(|f| f.demolition_inferred.is_empty()));
                assert_eq!(summary.diagnostics.dead_shells_after_demolition, 1);
                assert_eq!(summary.diagnostics.dead_shells_inferred, 0);
            } else {
                assert_eq!(converted[1].demolition_inferred, vec![1]);
                assert_eq!(summary.diagnostics.dead_shells_inferred, 1);
            }
        }
    }

    /// A car known only from its spawn pose is kept out of collisions until its first packet (RocketSim's
    /// demolished state, internally); the exported state is the spawn pose, not demolished, and the frames are
    /// listed in `spawn_pose_held`.
    #[test]
    fn a_car_on_its_spawn_pose_takes_no_part_in_collisions_until_its_first_packet() {
        fn value<T>(value: T, frame: usize) -> Value<T> {
            Value {
                value,
                frame,
                source: Source::Replay,
            }
        }
        let car = |packet: Option<usize>| observations::Car {
            actor_id: 1,
            actor_created_frame: 0,
            player_key: Some("p1".to_string()),
            player_link_active: true,
            team: Some(0),
            body_product_id: None,
            body: match packet {
                Some(frame) => Body {
                    position: Some(value([0.0, 0.0, 17.0], frame)),
                    rotation_xyzw: Some(value([0.0, 0.0, 0.0, 1.0], frame)),
                    linear_velocity: Some(value([0.0, 0.0, 0.0], frame)),
                    ..Body::default()
                },
                None => Body::default(),
            },
            boost: None,
            boost_raw: None,
            spawn_pose: packet.is_none().then_some(observations::SpawnPose {
                position: [0.0, 0.0, 36.0],
                rotation_xyzw: Some([0.0, 0.0, 0.0, 1.0]),
                frame: 0,
            }),
            inputs: observations::Inputs::default(),
        };
        let ball = Body {
            position: Some(value([0.0, 0.0, 120.0], 0)),
            rotation_xyzw: Some(value([0.0, 0.0, 0.0, 1.0], 0)),
            linear_velocity: Some(value([0.0, 0.0, 0.0], 0)),
            ..Body::default()
        };
        let frames = (0..4)
            .map(|index| observations::Frame {
                index,
                time: index as f32 * 0.033,
                delta: 0.033,
                ball: Some(ball.clone()),
                cars: vec![car(if index < 2 { None } else { Some(2) })],
                players: Vec::new(),
                team_scores: [None, None],
                seconds_remaining: None,
                overtime: None,
                game_state: Some(value("Active".to_string(), index)),
                events: Vec::new(),
                pad_pickups: Vec::new(),
            })
            .collect();
        let replay = ObservedReplay {
            header: observations::Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames,
            diagnostics: Default::default(),
        };
        let output = convert_observations(replay, &ConvertOptions::default()).unwrap();
        let held: Vec<Vec<usize>> = output
            .frames
            .iter()
            .map(|f| f.spawn_pose_held.clone())
            .collect();
        assert_eq!(held, [vec![0], vec![0], vec![], vec![]]);
        // The exported pose is the spawn pose and the car is not exported as demolished.
        for frame in &output.frames[..2] {
            let car = &frame.state.cars[0].1;
            assert!(!car.is_demoed);
            assert!(
                (car.phys.pos - Vec3A::new(0.0, 0.0, 36.0)).length() < 1e-3,
                "{:?}",
                car.phys.pos
            );
        }
        // The ball resting on the spawn pose is not hit by it before the first packet.
        assert!(output.frames[..2].iter().all(|f| f.touches.is_empty()));
        assert!(!output.frames[2].state.cars[0].1.is_demoed);
    }

    /// The spawn-pose hold is not released in a withheld frame (the evaluator masks the body but the spawn pose
    /// disappears with the first packet, which would reveal it), and an observed demolition of a car still held
    /// on its spawn pose makes it a normal demolition (exported as demolished).
    #[test]
    fn the_spawn_pose_hold_survives_a_withheld_frame_and_ends_in_a_demolition() {
        fn value<T>(value: T, frame: usize) -> Value<T> {
            Value {
                value,
                frame,
                source: Source::Replay,
            }
        }
        let car = |packet: Option<usize>, spawn: bool| observations::Car {
            actor_id: 1,
            actor_created_frame: 0,
            player_key: Some("p1".to_string()),
            player_link_active: true,
            team: Some(0),
            body_product_id: None,
            body: match packet {
                Some(frame) => Body {
                    position: Some(value([0.0, 0.0, 17.0], frame)),
                    rotation_xyzw: Some(value([0.0, 0.0, 0.0, 1.0], frame)),
                    linear_velocity: Some(value([0.0, 0.0, 0.0], frame)),
                    ..Body::default()
                },
                None => Body::default(),
            },
            boost: None,
            boost_raw: None,
            spawn_pose: spawn.then_some(observations::SpawnPose {
                position: [0.0, 0.0, 36.0],
                rotation_xyzw: Some([0.0, 0.0, 0.0, 1.0]),
                frame: 0,
            }),
            inputs: observations::Inputs::default(),
        };
        let demolish = observations::Event::Demolish {
            source: "extended",
            attacker_car: Some(2),
            victim_car: Some(1),
            attacker_pri: None,
            self_demolish: false,
            attacker_velocity: [0.0; 3],
            victim_velocity: [0.0; 3],
            repeat: false,
        };
        let replay = |cars: Vec<observations::Car>, events: Vec<Vec<observations::Event>>| {
            let frames = cars
                .into_iter()
                .zip(events)
                .enumerate()
                .map(|(index, (car, events))| observations::Frame {
                    index,
                    time: index as f32 * 0.033,
                    delta: 0.033,
                    ball: None,
                    cars: vec![car],
                    players: Vec::new(),
                    team_scores: [None, None],
                    seconds_remaining: None,
                    overtime: None,
                    game_state: Some(value("Active".to_string(), index)),
                    events,
                    pad_pickups: Vec::new(),
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
        // Frames 0-1 on the spawn pose; frame 2 is withheld and shows the masked car (no body, and no spawn
        // pose because the real first packet came in that frame); frame 3 shows the packet.
        let mut options = ConvertOptions::default();
        options.withheld_frames = Some(Arc::new(vec![false, false, true, false]));
        let held = |output: &ConversionOutput| -> Vec<Vec<usize>> {
            output
                .frames
                .iter()
                .map(|f| f.spawn_pose_held.clone())
                .collect()
        };
        let output = convert_observations(
            replay(
                vec![
                    car(None, true),
                    car(None, true),
                    car(None, false),
                    car(Some(3), false),
                ],
                vec![vec![], vec![], vec![], vec![]],
            ),
            &options,
        )
        .unwrap();
        assert_eq!(held(&output), [vec![0], vec![0], vec![0], vec![]]);
        // A demolition of the car on the spawn pose (frame 1): demolished from then on, no longer held.
        let output = convert_observations(
            replay(
                vec![
                    car(None, true),
                    car(None, true),
                    car(None, true),
                    car(Some(3), false),
                ],
                vec![vec![], vec![demolish], vec![], vec![]],
            ),
            &ConvertOptions::default(),
        )
        .unwrap();
        let demoed: Vec<bool> = output
            .frames
            .iter()
            .map(|f| f.state.cars[0].1.is_demoed)
            .collect();
        assert_eq!(demoed, [false, true, true, true]);
        assert_eq!(held(&output), [vec![0], vec![], vec![], vec![]]);
    }

    /// A respawned car on its spawn pose, replacing a dead shell held on the same slot, is itself held out of
    /// collisions: ending the shell's hold at the new lifetime must not undo the spawn hold (the car would fall
    /// and collide from its first frame).
    #[test]
    fn a_spawn_pose_hold_survives_the_end_of_the_previous_shells_hold() {
        fn value<T>(value: T, frame: usize) -> Value<T> {
            Value {
                value,
                frame,
                source: Source::Replay,
            }
        }
        let live_body = |frame: usize, sleeping: bool| Body {
            position: Some(value([0.0, 0.0, 300.0], frame)),
            rotation_xyzw: Some(value([0.0, 0.0, 0.0, 1.0], frame)),
            linear_velocity: (!sleeping).then(|| value([200.0, 0.0, 0.0], frame)),
            sleeping: Some(value(sleeping, frame)),
            ..Body::default()
        };
        let car =
            |actor: i32, created: usize, link: bool, body: Body, spawn: bool| observations::Car {
                actor_id: actor,
                actor_created_frame: created,
                player_key: Some("p1".to_string()),
                player_link_active: link,
                team: Some(0),
                body_product_id: None,
                body,
                boost: None,
                boost_raw: None,
                spawn_pose: spawn.then_some(observations::SpawnPose {
                    position: [0.0, 0.0, 36.0],
                    rotation_xyzw: Some([0.0, 0.0, 0.0, 1.0]),
                    frame: 2,
                }),
                inputs: observations::Inputs::default(),
            };
        let cars = vec![
            car(1, 0, true, live_body(0, false), false),
            car(1, 0, false, live_body(1, true), false),
            car(2, 2, true, Body::default(), true),
            car(2, 2, true, Body::default(), true),
            car(
                2,
                2,
                true,
                Body {
                    position: Some(value([0.0, 0.0, 17.0], 4)),
                    rotation_xyzw: Some(value([0.0, 0.0, 0.0, 1.0], 4)),
                    linear_velocity: Some(value([0.0, 0.0, 0.0], 4)),
                    ..Body::default()
                },
                false,
            ),
        ];
        let frames = cars
            .into_iter()
            .enumerate()
            .map(|(index, car)| observations::Frame {
                index,
                time: index as f32 * 0.033,
                delta: 0.033,
                ball: None,
                cars: vec![car],
                players: Vec::new(),
                team_scores: [None, None],
                seconds_remaining: None,
                overtime: None,
                game_state: Some(value("Active".to_string(), index)),
                events: Vec::new(),
                pad_pickups: Vec::new(),
            })
            .collect();
        let replay = ObservedReplay {
            header: observations::Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames,
            diagnostics: Default::default(),
        };
        let output = convert_observations(replay, &ConvertOptions::default()).unwrap();
        // Frame 1: the shell is held; frames 2-3: the new car is held on its spawn pose (not falling).
        assert_eq!(output.frames[1].dead_shells_held.len(), 1);
        for frame in &output.frames[2..4] {
            assert_eq!(frame.spawn_pose_held, vec![0]);
            assert!(frame.dead_shells_held.is_empty());
            assert!(
                (frame.state.cars[0].1.phys.pos.z - 36.0).abs() < 1e-3,
                "{}",
                frame.state.cars[0].1.phys.pos.z
            );
        }
        assert!(output.frames[4].spawn_pose_held.is_empty());
    }

    /// A spawn-pose hold belongs to the car lifetime that started it: a later car on the slot that has neither a
    /// body nor a spawn pose is not held by it.
    #[test]
    fn a_spawn_pose_hold_ends_with_its_lifetime() {
        fn value<T>(value: T, frame: usize) -> Value<T> {
            Value {
                value,
                frame,
                source: Source::Replay,
            }
        }
        let car =
            |actor: i32, created: usize, spawn: bool, packet: Option<usize>| observations::Car {
                actor_id: actor,
                actor_created_frame: created,
                player_key: Some("p1".to_string()),
                player_link_active: true,
                team: Some(0),
                body_product_id: None,
                body: match packet {
                    Some(frame) => Body {
                        position: Some(value([0.0, 0.0, 17.0], frame)),
                        rotation_xyzw: Some(value([0.0, 0.0, 0.0, 1.0], frame)),
                        linear_velocity: Some(value([0.0, 0.0, 0.0], frame)),
                        ..Body::default()
                    },
                    None => Body::default(),
                },
                boost: None,
                boost_raw: None,
                spawn_pose: spawn.then_some(observations::SpawnPose {
                    position: [0.0, 0.0, 36.0],
                    rotation_xyzw: Some([0.0, 0.0, 0.0, 1.0]),
                    frame: 0,
                }),
                inputs: observations::Inputs::default(),
            };
        // Actor 1 on its spawn pose in frames 0-1; actor 2 (a new lifetime, created in frame 2) shows neither a
        // body nor a spawn pose in frames 2-3, and a packet in frame 4.
        let cars = vec![
            car(1, 0, true, None),
            car(1, 0, true, None),
            car(2, 2, false, None),
            car(2, 2, false, None),
            car(2, 2, false, Some(4)),
        ];
        let frames = cars
            .into_iter()
            .enumerate()
            .map(|(index, car)| observations::Frame {
                index,
                time: index as f32 * 0.033,
                delta: 0.033,
                ball: None,
                cars: vec![car],
                players: Vec::new(),
                team_scores: [None, None],
                seconds_remaining: None,
                overtime: None,
                game_state: Some(value("Active".to_string(), index)),
                events: Vec::new(),
                pad_pickups: Vec::new(),
            })
            .collect();
        let replay = ObservedReplay {
            header: observations::Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames,
            diagnostics: Default::default(),
        };
        let output = convert_observations(replay, &ConvertOptions::default()).unwrap();
        let held: Vec<Vec<usize>> = output
            .frames
            .iter()
            .map(|f| f.spawn_pose_held.clone())
            .collect();
        assert_eq!(held, [vec![0], vec![0], vec![], vec![], vec![]]);
    }

    #[test]
    fn missing_meshes_are_refused_cleanly() {
        // A mesh directory without soccar meshes is an error, not a RocketSim panic.
        let empty = std::env::temp_dir().join(format!("empty-meshes-{}", std::process::id()));
        std::fs::create_dir_all(empty.join("soccar")).unwrap();
        assert!(matches!(
            check_soccar_meshes(&empty),
            Err(ConvertError::Init(_))
        ));
        std::fs::write(empty.join("soccar").join("field.cmf"), b"").unwrap();
        assert!(check_soccar_meshes(&empty).is_ok());
        std::fs::remove_dir_all(&empty).ok();
    }

    #[test]
    fn a_hit_tick_keeps_its_age_when_the_tick_counter_changes() {
        assert_eq!(rebase_tick(Some(5000), 5000, 0), Some(0));
        assert_eq!(rebase_tick(Some(4998), 5000, 10), Some(8));
        assert_eq!(rebase_tick(Some(4990), 5000, 6), None);
        assert_eq!(rebase_tick(Some(5001), 5000, 100), None);
        assert_eq!(rebase_tick(None, 5000, 100), None);
        let mut state = CarState::default();
        state.last_extra_hit_tick = Some(99);
        assert_eq!(
            rebase_car_ticks(state, 100, 1000).last_extra_hit_tick,
            Some(999)
        );
    }

    #[test]
    fn ball_runs_are_placed_relative_to_the_cars() {
        // A car run starts at tick 100 and a ball run overlaps it in frames 0..4; with an offset
        // of 3 ticks the ball run's start is the car's physical tick plus 3, clamped to the range.
        let car = RawRun {
            entries: (0..5).map(|f| (f, 8 * f as i64)).collect(),
            lo: 90,
            hi: 110,
            start: 100,
        };
        let mut ball = vec![RawRun {
            entries: (0..5).map(|f| (f, 8 * f as i64)).collect(),
            lo: 90,
            hi: 110,
            start: 95,
        }];
        place_ball_runs(&mut ball, &[((1, 0), car.clone())], 3.0);
        assert_eq!(ball[0].start, 103);
        place_ball_runs(&mut ball, &[((1, 0), car)], 30.0);
        assert_eq!(ball[0].start, 110);
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

        let options = ConvertOptions::default();

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
                spawn_pose: None,
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
                repeat: false,
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
            spawn_pose: None,
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

        let options = ConvertOptions::default();
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
    }

    #[test]
    fn lookahead_ground_controls_drive_the_interval_before_a_frame_and_respect_withholding() {
        let stamp = |value: f32, frame: usize| {
            Some(Value {
                value,
                frame,
                source: Source::Replay,
            })
        };
        // A grounded car observed once (frame 0), coasting at 500 UU/s; the throttle is pressed
        // in frame 2, so the throttle observed at frame 2 acts in the interval before it.
        let make_car = |frame: usize| observations::Car {
            actor_id: 1,
            actor_created_frame: 0,
            player_key: Some("p1".to_string()),
            player_link_active: true,
            team: Some(0),
            body_product_id: None,
            body: Body {
                position: Some(Value {
                    value: [0.0, 0.0, 17.0],
                    frame: 0,
                    source: Source::Replay,
                }),
                linear_velocity: Some(Value {
                    value: [500.0, 0.0, 0.0],
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
            spawn_pose: None,
            inputs: observations::Inputs {
                throttle: stamp(if frame == 2 { 1.0 } else { 0.0 }, frame),
                steer: stamp(0.0, frame),
                handbrake: Some(Value {
                    value: false,
                    frame,
                    source: Source::Replay,
                }),
                ..observations::Inputs::default()
            },
        };
        let replay = ObservedReplay {
            header: observations::Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames: (0..3)
                .map(|index| observations::Frame {
                    index,
                    time: index as f32 * 4.0 / 120.0,
                    delta: 4.0 / 120.0,
                    ball: None,
                    cars: vec![make_car(index)],
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
        };
        let speed_at_last_frame = |options: &ConvertOptions| {
            let out = convert_observations(replay.clone(), options).unwrap();
            out.frames[2].state.cars[0].1.phys.vel.x
        };
        let mut options = ConvertOptions::default();
        options.infer_packet_lag = true;
        let with_lookahead = speed_at_last_frame(&options);
        // A frame withheld by an evaluator is never used to drive the interval before it: the car coasts.
        options.withheld_frames = Some(Arc::new(vec![false, false, true]));
        let coasting = speed_at_last_frame(&options);
        assert!(
            with_lookahead > coasting + 5.0,
            "the throttle first seen at frame 2 should act before it: {coasting} vs {with_lookahead}"
        );
    }

    #[test]
    fn ground_control_timing_fit_recovers_a_change_seen_a_frame_late() {
        // Truth: a grounded car coasting at 500 UU/s whose throttle is pressed at tick 5. The
        // replay shows throttle 1 first at frame 2 (tick 8), and fresh packets at frames 0, 2, 4.
        rocketsim::init(Path::new("collision_meshes"), true).unwrap();
        let mut config = ArenaConfig::new(GameMode::Soccar);
        config.rng_seed = Some(1);
        let mut truth = Arena::new_with_config(config.clone());
        truth.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let mut start = CarState::default();
        start.phys.pos = Vec3A::new(0.0, 0.0, 17.0);
        start.phys.vel = Vec3A::new(500.0, 0.0, 0.0);
        start.is_on_ground = true;
        start.wheels_with_contact = [Some(rocketsim::RaycastHitInfo::default()); 4];
        let mut parked = BallState::default();
        parked.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
        truth.set_ball_state(parked);
        truth.set_car_state(0, start);
        let mut packets: Vec<CarState> = vec![*truth.get_car_state(0)];
        for tick in 1..=16u64 {
            truth.set_car_controls(
                0,
                CarControls {
                    throttle: if tick >= 5 { 1.0 } else { 0.0 },
                    ..CarControls::default()
                },
            );
            truth.step_tick();
            packets.push(*truth.get_car_state(0));
        }
        let fresh_frames = [0usize, 2, 4];
        let value = |value: f32, frame: usize| {
            Some(Value {
                value,
                frame,
                source: Source::Replay,
            })
        };
        let make_car = |frame: usize| {
            let source = *fresh_frames.iter().rev().find(|f| **f <= frame).unwrap();
            let state = &packets[source * 4];
            let stamp = |v: [f32; 3]| {
                Some(Value {
                    value: v,
                    frame: source,
                    source: Source::Replay,
                })
            };
            let q = Quat::from_mat3a(&state.phys.rot_mat);
            observations::Car {
                actor_id: 1,
                actor_created_frame: 0,
                player_key: Some("p1".to_string()),
                player_link_active: true,
                team: Some(0),
                body_product_id: None,
                body: Body {
                    position: stamp(state.phys.pos.to_array()),
                    linear_velocity: stamp(state.phys.vel.to_array()),
                    rotation_xyzw: Some(Value {
                        value: [q.x, q.y, q.z, q.w],
                        frame: source,
                        source: Source::Replay,
                    }),
                    angular_velocity_replay_units: stamp((state.phys.ang_vel * 100.0).to_array()),
                    ..Body::default()
                },
                boost: None,
                boost_raw: None,
                spawn_pose: None,
                inputs: observations::Inputs {
                    throttle: value(if frame >= 2 { 1.0 } else { 0.0 }, frame),
                    steer: value(0.0, frame),
                    handbrake: Some(Value {
                        value: false,
                        frame,
                        source: Source::Replay,
                    }),
                    ..observations::Inputs::default()
                },
            }
        };
        let replay = ObservedReplay {
            header: observations::Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames: (0..7)
                .map(|index| observations::Frame {
                    index,
                    time: index as f32 * 4.0 / 120.0,
                    delta: 4.0 / 120.0,
                    ball: None,
                    cars: vec![make_car(index)],
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
        };
        let mut lags = PacketLags {
            ball: vec![None; 7],
            cars: vec![None; 7],
            car_actor: HashMap::new(),
            ..PacketLags::default()
        };
        for frame in fresh_frames {
            lags.car_actor.insert((1, 0, frame), 0.0);
        }
        let options = ConvertOptions::default();
        let mut scratch = Arena::new_with_config(config.clone());
        scratch.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let car = &replay.frames[0].cars[0];
        let schedule = fit_ground_control_timing(
            &replay,
            &options,
            &Some(lags.clone()),
            0.0,
            0,
            car,
            &start,
            0,
            0,
            0,
            &mut scratch,
        )
        .expect("a flat grounded car with a control change and two later exact packets");
        // The midpoint rule puts the switch at tick 4; the truth is tick 5, so the fit shifts by +1.
        assert_eq!(schedule.end_tick, 8);
        assert!(
            schedule.entries.iter().any(|e| e.0 == 5 && e.1 == 1.0),
            "{:?}",
            schedule.entries
        );
        // Driving the main arena with the schedule reproduces the packet at frame 2 (tick 8).
        let mut arena = Arena::new_with_config(config);
        arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        arena.set_ball_state(parked);
        arena.set_car_state(0, start);
        let (mut pending, mut ground, mut events) = (Vec::new(), vec![schedule], Vec::new());
        let mut air: Vec<AirSchedule> = Vec::new();
        step_ticks(
            &mut arena,
            8,
            &mut pending,
            &mut ground,
            &mut air,
            &mut events,
        );
        let end = arena.get_car_state(0);
        assert!(
            (end.phys.pos - packets[8].phys.pos).length() < 0.01
                && (end.phys.vel - packets[8].phys.vel).length() < 0.1,
            "{:?} vs {:?}",
            end.phys.pos,
            packets[8].phys.pos
        );
        // Without the fit (throttle from the frame-2 packet time, tick 8) the car coasts instead.
        let mut coasting = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
        coasting.add_car(Team::Blue, CarBodyConfig::OCTANE);
        coasting.set_ball_state(parked);
        coasting.set_car_state(0, start);
        for _ in 0..8 {
            coasting.step_tick();
        }
        assert!((coasting.get_car_state(0).phys.vel - packets[8].phys.vel).length() > 10.0);
    }

    #[test]
    fn jump_timing_fit_recovers_a_press_before_the_counter_shows_it() {
        // Truth: a grounded car whose jump is held from tick 6 to 18. The counter turns odd at
        // frame 2 (tick 8) and even at frame 5 (tick 20); fresh packets at frames 0, 2, 4.
        rocketsim::init(Path::new("collision_meshes"), true).unwrap();
        let mut config = ArenaConfig::new(GameMode::Soccar);
        config.rng_seed = Some(1);
        let mut parked = rocketsim::BallState::default();
        parked.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
        let mut truth = Arena::new_with_config(config.clone());
        truth.add_car(Team::Blue, CarBodyConfig::OCTANE);
        truth.set_ball_state(parked);
        let mut start = CarState::default();
        start.phys.pos = Vec3A::new(0.0, 0.0, 17.0);
        start.phys.vel = Vec3A::new(500.0, 0.0, 0.0);
        start.is_on_ground = true;
        start.wheels_with_contact = [Some(rocketsim::RaycastHitInfo::default()); 4];
        truth.set_car_state(0, start);
        let mut packets: Vec<CarState> = vec![*truth.get_car_state(0)];
        for tick in 1..=16u64 {
            truth.set_car_controls(
                0,
                CarControls {
                    jump: (6..18).contains(&tick),
                    ..CarControls::default()
                },
            );
            truth.step_tick();
            packets.push(*truth.get_car_state(0));
        }
        let fresh_frames = [0usize, 2, 4];
        let make_car = |frame: usize| {
            let source = *fresh_frames.iter().rev().find(|f| **f <= frame).unwrap();
            let state = &packets[source * 4];
            let stamp = |v: [f32; 3]| {
                Some(Value {
                    value: v,
                    frame: source,
                    source: Source::Replay,
                })
            };
            let counter = if frame < 2 {
                0u8
            } else if frame < 5 {
                1
            } else {
                2
            };
            observations::Car {
                actor_id: 1,
                actor_created_frame: 0,
                player_key: Some("p1".to_string()),
                player_link_active: true,
                team: Some(0),
                body_product_id: None,
                body: Body {
                    position: stamp(state.phys.pos.to_array()),
                    linear_velocity: stamp(state.phys.vel.to_array()),
                    ..Body::default()
                },
                boost: None,
                boost_raw: None,
                spawn_pose: None,
                inputs: observations::Inputs {
                    throttle: Some(Value {
                        value: 0.0,
                        frame,
                        source: Source::Replay,
                    }),
                    steer: Some(Value {
                        value: 0.0,
                        frame,
                        source: Source::Replay,
                    }),
                    jump_active_raw: Some(Value {
                        value: counter,
                        frame,
                        source: Source::Replay,
                    }),
                    ..observations::Inputs::default()
                },
            }
        };
        let replay = ObservedReplay {
            header: observations::Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames: (0..7)
                .map(|index| observations::Frame {
                    index,
                    time: index as f32 * 4.0 / 120.0,
                    delta: 4.0 / 120.0,
                    ball: None,
                    cars: vec![make_car(index)],
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
        };
        let mut lags = PacketLags {
            ball: vec![None; 7],
            cars: vec![None; 7],
            car_actor: HashMap::new(),
            ..PacketLags::default()
        };
        for frame in fresh_frames {
            lags.car_actor.insert((1, 0, frame), 0.0);
        }
        let options = ConvertOptions::default();
        let mut scratch = Arena::new_with_config(config.clone());
        scratch.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let schedule = fit_jump_timing(
            &replay,
            &options,
            &Some(lags),
            0.0,
            0,
            &replay.frames[0].cars[0],
            &start,
            0,
            0,
            0,
            &parked,
            &mut scratch,
        )
        .expect("a grounded car whose jump counter turns odd before the second next exact packet");
        // The midpoint rule puts the press at tick 4; the truth is tick 6, a shift of +2.
        assert_eq!(schedule.end_tick, 8);
        assert!(
            schedule
                .entries
                .iter()
                .any(|e| e.0 == 6 && e.5 == Some(true)),
            "{:?}",
            schedule.entries
        );
        let run = |schedule: Option<GroundSchedule>, from: u64| {
            let mut arena = Arena::new_with_config(config.clone());
            arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
            arena.set_ball_state(parked);
            arena.set_car_state(0, start);
            let mut pending: Vec<PendingDodge> = Vec::new();
            let mut ground: Vec<GroundSchedule> = schedule.into_iter().collect();
            let mut events: Vec<SimEvent> = Vec::new();
            for tick in 1..=8u64 {
                if ground.is_empty() {
                    arena.set_car_controls(
                        0,
                        CarControls {
                            jump: tick >= from,
                            ..CarControls::default()
                        },
                    );
                }
                step_ticks(
                    &mut arena,
                    1,
                    &mut pending,
                    &mut ground,
                    &mut Vec::new(),
                    &mut events,
                );
            }
            *arena.get_car_state(0)
        };
        let fitted = run(Some(schedule), 0);
        assert!(
            (fitted.phys.pos - packets[8].phys.pos).length() < 0.05
                && (fitted.phys.vel - packets[8].phys.vel).length() < 0.5,
            "{:?} vs {:?}",
            fitted.phys.vel,
            packets[8].phys.vel
        );
        // Pressing at the midpoint-rule tick (4) instead leaves the car off by the extra ticks.
        let early = run(None, 4);
        assert!((early.phys.vel - packets[8].phys.vel).length() > 5.0);
    }

    #[test]
    fn dodge_start_fit_uses_the_second_packet_and_plans_only_the_next_interval() {
        // Truth: an airborne car that presses a forward dodge (pitch 1) at tick 6 with no cancel.
        // The counter turns odd at frame 2 (tick 8) with a fresh torque; fresh packets at frames
        // 0, 2, 4 (ticks 0, 8, 16) with exact lags.
        rocketsim::init(Path::new("collision_meshes"), true).unwrap();
        let mut config = ArenaConfig::new(GameMode::Soccar);
        config.rng_seed = Some(1);
        let mut parked = rocketsim::BallState::default();
        parked.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
        let mut truth = Arena::new_with_config(config.clone());
        truth.add_car(Team::Blue, CarBodyConfig::OCTANE);
        truth.set_ball_state(parked);
        let mut start = CarState::default();
        start.phys.pos = Vec3A::new(0.0, 0.0, 800.0);
        start.phys.vel = Vec3A::new(600.0, 0.0, 200.0);
        start.is_on_ground = false;
        start.has_jumped = true;
        start.air_time_since_jump = 0.05;
        truth.set_car_state(0, start);
        let mut packets: Vec<CarState> = vec![*truth.get_car_state(0)];
        for tick in 1..=16u64 {
            let mut controls = CarControls::default();
            if tick == 6 {
                controls.jump = true;
                controls.pitch = 1.0;
            }
            truth.set_car_controls(0, controls);
            truth.step_tick();
            packets.push(*truth.get_car_state(0));
        }
        let fresh_frames = [0usize, 2, 4];
        let make_car = |frame: usize| {
            let source = *fresh_frames.iter().rev().find(|f| **f <= frame).unwrap();
            let state = &packets[source * 4];
            let stamp = |v: [f32; 3]| {
                Some(Value {
                    value: v,
                    frame: source,
                    source: Source::Replay,
                })
            };
            let q = Quat::from_mat3a(&state.phys.rot_mat);
            observations::Car {
                actor_id: 1,
                actor_created_frame: 0,
                player_key: Some("p1".to_string()),
                player_link_active: true,
                team: Some(0),
                body_product_id: None,
                body: Body {
                    position: stamp(state.phys.pos.to_array()),
                    linear_velocity: stamp(state.phys.vel.to_array()),
                    rotation_xyzw: Some(Value {
                        value: [q.x, q.y, q.z, q.w],
                        frame: source,
                        source: Source::Replay,
                    }),
                    angular_velocity_replay_units: stamp((state.phys.ang_vel * 100.0).to_array()),
                    ..Body::default()
                },
                boost: None,
                boost_raw: None,
                spawn_pose: None,
                inputs: observations::Inputs {
                    dodge_active_raw: Some(Value {
                        value: u8::from(frame >= 2),
                        frame: if frame >= 2 { frame.max(2) } else { frame },
                        source: Source::Replay,
                    }),
                    dodge_torque_replay_units: (frame >= 2).then_some(Value {
                        value: [0.0, -2.24, 0.0],
                        frame: 2,
                        source: Source::Replay,
                    }),
                    ..observations::Inputs::default()
                },
            }
        };
        let replay = ObservedReplay {
            header: observations::Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames: (0..7)
                .map(|index| observations::Frame {
                    index,
                    time: index as f32 * 4.0 / 120.0,
                    delta: 4.0 / 120.0,
                    ball: None,
                    cars: vec![make_car(index)],
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
        };
        let mut lags = PacketLags {
            ball: vec![None; 7],
            cars: vec![None; 7],
            car_actor: HashMap::new(),
            ..PacketLags::default()
        };
        for frame in fresh_frames {
            lags.car_actor.insert((1, 0, frame), 0.0);
        }
        let options = ConvertOptions::default();
        let mut scratch = Arena::new_with_config(config);
        scratch.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let plan = fit_dodge_start(
            &replay,
            &options,
            &Some(lags),
            0.0,
            0,
            &replay.frames[0].cars[0],
            &start,
            &CarControls::default(),
            0,
            0,
            &parked,
            &mut scratch,
        )
        .expect(
            "an airborne car whose dodge counter turns odd before the second next exact packet",
        );
        // Fitted on the packet at tick 16 (the second next), the plan covers the interval to the
        // next packet (tick 8) and starts at the true press tick.
        assert_eq!(plan.activation_frame, 2);
        assert_eq!(plan.start_offset, 6);
        assert_eq!(plan.duration, 8);
        assert_eq!(plan.cancel, 0.0);
        assert!((plan.pitch - 1.0).abs() < 1e-6);
    }

    #[test]
    fn scratch_arenas_have_the_cars_hitbox_and_no_reachable_boost_pad() {
        rocketsim::init(Path::new("collision_meshes"), true).unwrap();
        let pad = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar))
            .get_boost_pad_config(0)
            .pos;
        let mut scratch = scratch_arena(0, CarBodyConfig::DOMINUS);
        assert_eq!(scratch.num_boost_pads(), 1);
        assert!(scratch.get_boost_pad_config(0).pos.z < -1000.0);
        assert_eq!(scratch.get_car_info(0).config, CarBodyConfig::DOMINUS);
        // A car with no boost parked on a pad's spot of the real arena stays without boost.
        let mut car = CarState::default();
        car.phys.pos = Vec3A::new(pad.x, pad.y, 17.0);
        car.boost = 0.0;
        scratch.set_car_state(0, car);
        for _ in 0..10 {
            scratch.step_tick();
        }
        assert_eq!(scratch.get_car_state(0).boost, 0.0);
    }

    #[test]
    fn flip_cancel_fit_recovers_the_cancel_whatever_the_scratch_ball_was_left_at() {
        // Truth: an airborne car presses a diagonal dodge at tick 1 and holds a 0.75 pitch cancel.
        // Fresh packets at ticks 3 (frame 0, already flipping) and 11 (frame 1), exact lags.
        rocketsim::init(Path::new("collision_meshes"), true).unwrap();
        let mut config = ArenaConfig::new(GameMode::Soccar);
        config.rng_seed = Some(1);
        let mut parked = rocketsim::BallState::default();
        parked.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
        let mut truth = Arena::new_with_config(config.clone());
        truth.add_car(Team::Blue, CarBodyConfig::OCTANE);
        truth.set_ball_state(parked);
        let mut start = CarState::default();
        start.phys.pos = Vec3A::new(-2000.0, 1000.0, 900.0);
        start.phys.vel = Vec3A::new(600.0, 0.0, 100.0);
        start.is_on_ground = false;
        start.has_jumped = true;
        start.air_time_since_jump = 0.05;
        truth.set_car_state(0, start);
        let mut states = vec![*truth.get_car_state(0)];
        for tick in 1..=11u64 {
            let mut controls = CarControls::default();
            if tick == 1 {
                // A diagonal dodge: the cancel turns the spin's direction, which the fit can see even
                // after the angular speed reaches its limit.
                controls.jump = true;
                controls.pitch = 0.7;
                controls.yaw = 0.7;
            } else {
                controls.pitch = 0.75 * truth.get_car_state(0).flip_rel_torque.y.signum();
            }
            truth.set_car_controls(0, controls);
            truth.step_tick();
            states.push(*truth.get_car_state(0));
        }
        let packet_ticks = [3usize, 11];
        let car = |frame: usize| {
            let state = &states[packet_ticks[frame]];
            let stamp = |v: [f32; 3]| {
                Some(Value {
                    value: v,
                    frame,
                    source: Source::Replay,
                })
            };
            let q = Quat::from_mat3a(&state.phys.rot_mat);
            observations::Car {
                actor_id: 1,
                actor_created_frame: 0,
                player_key: Some("p1".to_string()),
                player_link_active: true,
                team: Some(0),
                body_product_id: None,
                body: Body {
                    position: stamp(state.phys.pos.to_array()),
                    linear_velocity: stamp(state.phys.vel.to_array()),
                    rotation_xyzw: Some(Value {
                        value: [q.x, q.y, q.z, q.w],
                        frame,
                        source: Source::Replay,
                    }),
                    angular_velocity_replay_units: stamp((state.phys.ang_vel * 100.0).to_array()),
                    ..Body::default()
                },
                boost: None,
                boost_raw: None,
                spawn_pose: None,
                inputs: observations::Inputs {
                    dodge_active_raw: Some(Value {
                        value: 1,
                        frame: 0,
                        source: Source::Replay,
                    }),
                    ..observations::Inputs::default()
                },
            }
        };
        let replay = ObservedReplay {
            header: observations::Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames: (0..2)
                .map(|index| observations::Frame {
                    index,
                    time: index as f32 * 8.0 / 120.0,
                    delta: 8.0 / 120.0,
                    ball: None,
                    cars: vec![car(index)],
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
        };
        let mut lags = PacketLags {
            ball: vec![None; 2],
            cars: vec![None; 2],
            car_actor: HashMap::new(),
            ..PacketLags::default()
        };
        lags.car_actor.insert((1, 0, 0), 0.0);
        lags.car_actor.insert((1, 0, 1), 0.0);
        let lags = Some(lags);
        let options = ConvertOptions::default();
        let at_packet = states[packet_ticks[0]];
        assert!(at_packet.is_flipping);
        let mut scratch = Arena::new_with_config(config);
        scratch.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let fit = |scratch: &mut Arena| {
            fit_flip_cancel(
                &replay,
                &options,
                &lags,
                0.0,
                0,
                &replay.frames[0].cars[0],
                &at_packet,
                &CarControls::default(),
                0,
                0,
                &parked,
                scratch,
            )
        };
        assert_eq!(fit(&mut scratch), Some(0.75));
        // Whatever ball an earlier fit left in the shared scratch arena (here one on this car's path),
        // the fit starts from the ball it is given: the result and the scratch arena's simulated ball
        // are the same.
        let mut ends = Vec::new();
        for left_behind in [states[7].phys.pos, Vec3A::new(2500.0, -3000.0, 93.0)] {
            let mut ball = rocketsim::BallState::default();
            ball.phys.pos = left_behind;
            scratch.set_ball_state(ball);
            assert_eq!(fit(&mut scratch), Some(0.75));
            ends.push(scratch.get_ball_state().phys.pos);
        }
        assert_eq!(ends[0], ends[1]);
    }

    #[test]
    fn activation_torque_uses_the_value_in_effect_a_frame_after_the_counter() {
        let car = |torque: Option<(f32, usize)>| observations::Car {
            actor_id: 1,
            actor_created_frame: 0,
            player_key: Some("p1".to_string()),
            player_link_active: true,
            team: Some(0),
            body_product_id: None,
            body: Body::default(),
            boost: None,
            boost_raw: None,
            spawn_pose: None,
            inputs: observations::Inputs {
                dodge_torque_replay_units: torque.map(|(x, frame)| Value {
                    value: [x, 0.0, 0.0],
                    frame,
                    source: Source::Replay,
                }),
                ..observations::Inputs::default()
            },
        };
        let frame = |index: usize, torque: Option<(f32, usize)>| observations::Frame {
            index,
            time: index as f32 / 30.0,
            delta: 1.0 / 30.0,
            ball: None,
            cars: vec![car(torque)],
            players: Vec::new(),
            team_scores: [None, None],
            seconds_remaining: None,
            overtime: None,
            game_state: None,
            events: Vec::new(),
            pad_pickups: Vec::new(),
        };
        // Same direction as the previous dodge: the stamp is old and the value still holds.
        let repeated = [frame(0, Some((1.5, 1))), frame(1, Some((1.5, 1)))];
        assert_eq!(
            activation_torque(&repeated, 0, &repeated[0].cars[0]),
            Some([1.5, 0.0, 0.0])
        );
        // The new torque arrives a frame after the counter: the stale value at the counter's frame is not it.
        let late = [frame(0, Some((1.5, 1))), frame(1, Some((-2.0, 1)))];
        assert_eq!(
            activation_torque(&late, 0, &late[0].cars[0]),
            Some([-2.0, 0.0, 0.0])
        );
        // No torque has ever been sent: there is no direction.
        let none = [frame(0, None), frame(1, None)];
        assert_eq!(activation_torque(&none, 0, &none[0].cars[0]), None);
    }

    #[test]
    fn dodge_start_fit_infers_the_tick_of_the_first_packet_after_the_activation() {
        // Truth: an airborne car that presses a forward dodge (pitch 1) at tick 6 with no cancel.
        // The counter turns odd at frame 2 (tick 8) with a fresh torque; the fresh packets at frames
        // 0 and 4 (ticks 0 and 16) have exact lags, the one at frame 2 was generated `lag` ticks
        // before its frame time and has none. Its tick is inferred from the path with the dodge.
        for (b_tick, deferred) in [(7usize, false), (5, true), (8, false)] {
            first_packet_case(b_tick, deferred);
        }
    }

    fn first_packet_case(b_tick: usize, deferred: bool) {
        rocketsim::init(Path::new("collision_meshes"), true).unwrap();
        let mut config = ArenaConfig::new(GameMode::Soccar);
        config.rng_seed = Some(1);
        let mut parked = rocketsim::BallState::default();
        parked.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
        let mut truth = Arena::new_with_config(config.clone());
        truth.add_car(Team::Blue, CarBodyConfig::OCTANE);
        truth.set_ball_state(parked);
        let mut start = CarState::default();
        start.phys.pos = Vec3A::new(0.0, 0.0, 800.0);
        start.phys.vel = Vec3A::new(600.0, 0.0, 200.0);
        start.is_on_ground = false;
        start.has_jumped = true;
        start.air_time_since_jump = 0.05;
        truth.set_car_state(0, start);
        let mut packets: Vec<CarState> = vec![*truth.get_car_state(0)];
        for tick in 1..=16u64 {
            let mut controls = CarControls::default();
            if tick == 6 {
                controls.jump = true;
                controls.pitch = 1.0;
            }
            truth.set_car_controls(0, controls);
            truth.step_tick();
            packets.push(*truth.get_car_state(0));
        }
        let fresh_frames = [0usize, 2, 4];
        let make_car = |frame: usize| {
            let source = *fresh_frames.iter().rev().find(|f| **f <= frame).unwrap();
            let state = &packets[if source == 2 { b_tick } else { source * 4 }];
            let stamp = |v: [f32; 3]| {
                Some(Value {
                    value: v,
                    frame: source,
                    source: Source::Replay,
                })
            };
            let q = Quat::from_mat3a(&state.phys.rot_mat);
            observations::Car {
                actor_id: 1,
                actor_created_frame: 0,
                player_key: Some("p1".to_string()),
                player_link_active: true,
                team: Some(0),
                body_product_id: None,
                body: Body {
                    position: stamp(state.phys.pos.to_array()),
                    linear_velocity: stamp(state.phys.vel.to_array()),
                    rotation_xyzw: Some(Value {
                        value: [q.x, q.y, q.z, q.w],
                        frame: source,
                        source: Source::Replay,
                    }),
                    angular_velocity_replay_units: stamp((state.phys.ang_vel * 100.0).to_array()),
                    ..Body::default()
                },
                boost: None,
                boost_raw: None,
                spawn_pose: None,
                inputs: observations::Inputs {
                    // A car that has not dodged yet has no dodge counter.
                    dodge_active_raw: (frame >= 2).then_some(Value {
                        value: 1,
                        frame: 2,
                        source: Source::Replay,
                    }),
                    dodge_torque_replay_units: (frame >= 2).then_some(Value {
                        value: [0.0, -2.24, 0.0],
                        frame: 2,
                        source: Source::Replay,
                    }),
                    ..observations::Inputs::default()
                },
            }
        };
        let replay = ObservedReplay {
            header: observations::Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames: (0..7)
                .map(|index| observations::Frame {
                    index,
                    time: index as f32 * 4.0 / 120.0,
                    delta: 4.0 / 120.0,
                    ball: None,
                    cars: vec![make_car(index)],
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
        };
        let mut lags = PacketLags {
            ball: vec![None; 7],
            cars: vec![None; 7],
            car_actor: HashMap::new(),
            ..PacketLags::default()
        };
        for frame in [0usize, 4] {
            lags.car_actor.insert((1, 0, frame), 0.0);
        }
        let options = ConvertOptions::default();
        let mut scratch = Arena::new_with_config(config);
        scratch.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let plan = fit_dodge_start(
            &replay,
            &options,
            &Some(lags),
            0.0,
            0,
            &replay.frames[0].cars[0],
            &start,
            &CarControls::default(),
            0,
            0,
            &parked,
            &mut scratch,
        )
        .expect(
            "an airborne car whose dodge counter turns odd before the second next exact packet",
        );
        let lag = (8 - b_tick) as u64;
        assert_eq!(plan.activation_frame, 2);
        assert_eq!(plan.start_offset, 6);
        assert_eq!(plan.first_packet, Some((2, lag)), "b_tick {b_tick}");
        assert_eq!(plan.duration, if deferred { 16 } else { b_tick as u64 });
        assert!((plan.pitch - 1.0).abs() < 1e-6);
    }

    #[test]
    fn ground_flip_fit_recovers_the_jump_shift_and_the_dodge_press_together() {
        // Truth: a grounded car whose jump is held over ticks 3..7 and whose forward dodge is
        // pressed at tick 10. The jump counter is odd at frame 1 and even from frame 2; the dodge
        // counter turns odd at frame 3 with a fresh torque; fresh packets at frames 0, 4, 6.
        rocketsim::init(Path::new("collision_meshes"), true).unwrap();
        let mut config = ArenaConfig::new(GameMode::Soccar);
        config.rng_seed = Some(1);
        let mut parked = rocketsim::BallState::default();
        parked.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
        let mut truth = Arena::new_with_config(config.clone());
        truth.add_car(Team::Blue, CarBodyConfig::OCTANE);
        truth.set_ball_state(parked);
        let mut start = CarState::default();
        start.phys.pos = Vec3A::new(0.0, 0.0, 17.0);
        start.phys.vel = Vec3A::new(500.0, 0.0, 0.0);
        start.is_on_ground = true;
        start.wheels_with_contact = [Some(rocketsim::RaycastHitInfo::default()); 4];
        truth.set_car_state(0, start);
        let mut packets: Vec<CarState> = vec![*truth.get_car_state(0)];
        for tick in 1..=24u64 {
            let mut controls = CarControls::default();
            controls.jump = (3..7).contains(&tick) || tick == 10;
            if tick == 10 {
                controls.pitch = 1.0;
            }
            truth.set_car_controls(0, controls);
            truth.step_tick();
            packets.push(*truth.get_car_state(0));
        }
        let fresh_frames = [0usize, 4, 6];
        let make_car = |frame: usize| {
            let source = *fresh_frames.iter().rev().find(|f| **f <= frame).unwrap();
            let state = &packets[source * 4];
            let stamp = |v: [f32; 3]| {
                Some(Value {
                    value: v,
                    frame: source,
                    source: Source::Replay,
                })
            };
            let q = Quat::from_mat3a(&state.phys.rot_mat);
            let counter = |value: u8| {
                Some(Value {
                    value,
                    frame,
                    source: Source::Replay,
                })
            };
            observations::Car {
                actor_id: 1,
                actor_created_frame: 0,
                player_key: Some("p1".to_string()),
                player_link_active: true,
                team: Some(0),
                body_product_id: None,
                body: Body {
                    position: stamp(state.phys.pos.to_array()),
                    linear_velocity: stamp(state.phys.vel.to_array()),
                    rotation_xyzw: Some(Value {
                        value: [q.x, q.y, q.z, q.w],
                        frame: source,
                        source: Source::Replay,
                    }),
                    angular_velocity_replay_units: stamp((state.phys.ang_vel * 100.0).to_array()),
                    ..Body::default()
                },
                boost: None,
                boost_raw: None,
                spawn_pose: None,
                inputs: observations::Inputs {
                    jump_active_raw: counter(match frame {
                        0 => 0,
                        1 => 1,
                        _ => 2,
                    }),
                    dodge_active_raw: counter(u8::from(frame >= 3)),
                    dodge_torque_replay_units: (frame >= 3).then_some(Value {
                        value: [0.0, -2.24, 0.0],
                        frame: 3,
                        source: Source::Replay,
                    }),
                    ..observations::Inputs::default()
                },
            }
        };
        let replay = ObservedReplay {
            header: observations::Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames: (0..9)
                .map(|index| observations::Frame {
                    index,
                    time: index as f32 * 4.0 / 120.0,
                    delta: 4.0 / 120.0,
                    ball: None,
                    cars: vec![make_car(index)],
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
        };
        let mut lags = PacketLags {
            ball: vec![None; 9],
            cars: vec![None; 9],
            car_actor: HashMap::new(),
            ..PacketLags::default()
        };
        for frame in fresh_frames {
            lags.car_actor.insert((1, 0, frame), 0.0);
        }
        let options = ConvertOptions::default();
        let mut scratch = Arena::new_with_config(config);
        scratch.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let flip = fit_ground_flip_timing(
            &replay,
            &options,
            &Some(lags),
            0.0,
            0,
            &replay.frames[0].cars[0],
            &start,
            0,
            0,
            0,
            &parked,
            &mut scratch,
        )
        .expect(
            "a grounded car whose jump and dodge counters turn odd before the second next packet",
        );
        // Fitted on the packet at tick 24; the plan covers the interval to the packet at tick 16.
        assert_eq!(flip.schedule.end_tick, 16);
        let dodge = flip
            .dodge
            .expect("the dodge press (tick 10) is inside the interval");
        assert_eq!(
            (dodge.activation_frame, dodge.start_offset, dodge.duration),
            (3, 10, 16)
        );
        assert_eq!(dodge.cancel, 0.0);
        // The jump is pressed at tick 3 and released at tick 7 (a shift of +3 from the midpoint rule).
        let entries = &flip.schedule.entries;
        assert!(
            entries.iter().any(|e| e.0 == 3 && e.5 == Some(true)),
            "{entries:?}"
        );
        assert!(
            entries.iter().any(|e| e.0 == 7 && e.5 == Some(false)),
            "{entries:?}"
        );
    }

    /// Free flight in RocketSim with piecewise-constant controls against the analytic forward model.
    fn rocketsim_air_rotation(
        rot: Mat3A,
        omega: Vec3A,
        segments: &[(AirControls, u32)],
    ) -> (Mat3A, Vec3A) {
        rocketsim::init(Path::new("collision_meshes"), true).unwrap();
        let mut config = ArenaConfig::new(GameMode::Soccar);
        config.rng_seed = Some(1);
        let mut arena = Arena::new_with_config(config);
        arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let mut parked = rocketsim::BallState::default();
        parked.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
        arena.set_ball_state(parked);
        let mut state = CarState::default();
        state.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
        state.phys.vel = Vec3A::new(300.0, 100.0, 50.0);
        state.phys.rot_mat = rot;
        state.phys.ang_vel = omega;
        state.is_on_ground = false;
        state.has_jumped = true;
        state.has_double_jumped = true;
        arena.set_car_state(0, state);
        for &(controls, ticks) in segments {
            arena.set_car_controls(
                0,
                CarControls {
                    pitch: controls.pitch,
                    yaw: controls.yaw,
                    roll: controls.roll,
                    ..CarControls::default()
                },
            );
            for _ in 0..ticks {
                arena.step_tick();
            }
        }
        let end = arena.get_car_state(0);
        (end.phys.rot_mat, end.phys.ang_vel)
    }

    #[test]
    fn the_air_forward_model_matches_rocketsim_and_the_bvp_recovers_piecewise_controls() {
        let rot = Mat3A::from_quat(Quat::from_euler(glam::EulerRot::ZYX, 0.6, 0.2, -0.3));
        let omega = Vec3A::new(0.8, -1.1, 0.4);
        let truth = [
            (
                AirControls {
                    pitch: 0.7,
                    yaw: -0.4,
                    roll: 0.1,
                },
                4,
            ),
            (
                AirControls {
                    pitch: -0.2,
                    yaw: 0.6,
                    roll: -0.9,
                },
                4,
            ),
            (
                AirControls {
                    pitch: 0.3,
                    yaw: 0.0,
                    roll: 0.5,
                },
                4,
            ),
        ];
        let (sim_rot, sim_omega) = rocketsim_air_rotation(rot, omega, &truth);
        let (model_rot, model_omega) = air_state_forward(rot, omega, &truth);
        let rot_error = rotation_error_degrees(sim_rot, model_rot);
        let omega_error = (sim_omega - model_omega).length();
        eprintln!(
            "model vs RocketSim over 12 ticks: rotation {rot_error:.4} deg, angular velocity {omega_error:.4} rad/s"
        );
        assert!(rot_error < 0.05 && omega_error < 0.02);
        // The BVP reaches the end state of a run it has not seen, from a prior of zero controls.
        let prior = [AirControls::default(); 3];
        let (solved, rot_residual, omega_residual) =
            solve_air_bvp(rot, omega, sim_rot, sim_omega, &[4, 4, 4], &prior);
        let (reached_rot, reached_omega) = rocketsim_air_rotation(
            rot,
            omega,
            &solved.iter().map(|c| (*c, 4)).collect::<Vec<_>>(),
        );
        eprintln!(
            "bvp residual (model) {rot_residual:.5} rad, {omega_residual:.4} rad/s; in RocketSim {:.4} deg, {:.4} rad/s",
            rotation_error_degrees(reached_rot, sim_rot),
            (reached_omega - sim_omega).length()
        );
        assert!(rotation_error_degrees(reached_rot, sim_rot) < 0.3);
        assert!((reached_omega - sim_omega).length() < 0.1);
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
            spawn_pose: None,
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
                spawn_pose: None,
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
    fn lookahead_span_bridges_gap_and_refuses_withheld_spans() {
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

        let mut withheld = ConvertOptions::default();
        withheld.withheld_frames = Some(Arc::new(vec![false, true, false]));
        assert_eq!(pitch(&replay, &withheld, 0), 0.0);
        assert_eq!(pitch(&replay, &withheld, 1), 0.0);
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
        assert!(base[2] > 0.0 && base[3] > 0.0, "persisted pitch {base:?}");
        assert_eq!(
            base[..4],
            future[..4],
            "later packets must not alter earlier controls"
        );
        assert_eq!(base[0], 0.0, "the first span has no earlier packet pair");
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

    /// RocketSim before `0b02051` computed the extra ball-car hit impulse but dropped it, so a
    /// 1,400 UU/s car sent a resting ball off slower than itself (ROCKETSIM_NOTES.md, entry 1).
    #[test]
    fn rocketsim_applies_the_extra_hit_impulse() {
        rocketsim::init(Path::new("collision_meshes"), true).unwrap();
        let mut config = ArenaConfig::new(GameMode::Soccar);
        config.rng_seed = Some(0);
        let mut arena = Arena::new_with_config(config);
        arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let mut car = CarState::default();
        car.phys.pos = Vec3A::new(0.0, -600.0, 17.0);
        car.phys.vel = Vec3A::new(0.0, 1400.0, 0.0);
        car.phys.rot_mat = Mat3A::from_cols(Vec3A::Y, -Vec3A::X, Vec3A::Z);
        car.is_on_ground = true;
        car.wheels_with_contact = [Some(rocketsim::RaycastHitInfo::default()); 4];
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
        let mut hit_tick = None;
        for tick in 1..=60u32 {
            for event in step_arena_tick(&mut arena) {
                if matches!(event, ArenaEvent::CarHitBall(_)) && hit_tick.is_none() {
                    hit_tick = Some(tick);
                }
            }
            if hit_tick.is_some_and(|hit| tick == hit + 4) {
                let speed = arena.get_ball_state().phys.vel.length();
                assert!(speed > 1600.0, "ball speed after the hit {speed}");
                return;
            }
        }
        panic!("no hit");
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
                value: [
                    0.0,
                    0.0,
                    std::f32::consts::FRAC_1_SQRT_2,
                    std::f32::consts::FRAC_1_SQRT_2,
                ],
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
