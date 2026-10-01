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

/// How the flip's pitch cancel is chosen for the interval from a fresh packet to the next one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum FlipCancelSource {
    /// Simulate the candidate cancels from this packet and keep the one whose angular velocity
    /// matches the next packet (in sample there; the default).
    NextPacketFit,
    /// The same fit on the *previous* interval (previous fresh packet to this one), used for the
    /// next interval: causal, so the residual at the next packet is a check.
    PreviousIntervalFit,
    /// The rule of `external/RLCarInputSolver` (AirSolver.cpp) on the previous interval, used for
    /// the next one: a full cancel when the local pitch angular speed fell by more than 0.05 rad/s
    /// per tick, else none. Causal.
    ExternalRulePrevious,
    /// The same rule on the interval to the next packet (in sample there).
    ExternalRuleNext,
}

impl FlipCancelSource {
    pub fn from_name(name: &str) -> Option<Self> {
        Some(match name {
            "next-fit" => Self::NextPacketFit,
            "previous-fit" => Self::PreviousIntervalFit,
            "external-previous" => Self::ExternalRulePrevious,
            "external-next" => Self::ExternalRuleNext,
            _ => return None,
        })
    }
}

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
    /// Demolish the victim car in the simulation when the replay reports a demolition
    /// (`ReplicatedDemolish*`), instead of relying on RocketSim's own bump detection (which found 10
    /// of 12 on a host replay and 8 of 12 on a client replay of the remote-client games). The car
    /// stays demolished for RocketSim's respawn delay even while the replay's car actor is linked.
    pub apply_observed_demolitions: bool,
    /// Offline: find car-ball contacts from the ball packets (a ball-only rollout between
    /// consecutive packets; `ball_evidence`) and report them as `ball_contacts`.
    pub contacts_from_ball_packets: bool,
    /// With `apply_observed_demolitions`, switch RocketSim's own demolition rule off so that the
    /// observed demolitions are the only ones (no duplicate `car_hit_car` with `is_demo`, no
    /// invented demolitions).
    pub disable_simulated_demolitions: bool,
    /// Keep every boost pad on cooldown inside the simulation, so a simulated car never picks boost up
    /// by driving over a pad (a car whose simulated position is off by a few UU picks up a pad the real
    /// car missed, or the reverse): the boost amount then comes only from the replay's own updates
    /// (offline reconstruction; masked prediction has no later update and keeps the simulated pickups).
    pub block_sim_pad_pickups: bool,
    /// Offline: a boost amount that jumps up (a pad pickup) is first seen about a frame after the car
    /// picked it up, so apply an increase seen in the next frame one frame early.
    pub boost_pickup_lookahead: bool,
    /// Route steer input to aerial yaw while airborne.
    pub infer_air_steer_controls: bool,
    /// Offline: fit the physical start tick of each dodge (and its pitch cancel) by simulating
    /// candidates against the next fresh car packet, and trigger the dodge at that tick instead of
    /// at the frame time of its counter.
    pub infer_dodge_start: bool,
    /// Offline: drive a grounded car with the controls of the frame that ends the interval instead
    /// of the frame that starts it. A throttle, steer, handbrake or boost change is first seen in
    /// the frame after it happened, and that frame's state is itself on average 2 ticks (half the
    /// 0-4 tick lag) older than its time, so the change took effect about `2 + gap / 2` ticks
    /// before the frame time; the new value is applied from there (`gap / 2 - 2` ticks into the
    /// interval, at least at its start). Needs the inferred packet lags (the rule is about physical
    /// ticks; without them every state sits at its frame time) and is not used for a frame withheld
    /// by an evaluator.
    pub lookahead_ground_controls: bool,
    /// Where a control change first seen in a frame took effect: a replicated attribute is sent with
    /// the car's next update, so the change tick lies in `(S_prev, S_cur]`, the physical ticks of
    /// the car's previous packet and its packet in this frame (97% of 21,000 changes on the two
    /// remote-client games; uniform inside, median at 0.57 of the span), not a fixed `2 + gap / 2`
    /// ticks before the frame time (mean 10.9 ticks late there, about 7 by the rule). With both
    /// packets' exact chain lags the rule is the middle of that interval; without, the old rule.
    pub packet_interval_control_rule: bool,
    /// Offline: a car's observed controls can lead or lag the server by a car-specific amount (the
    /// recording client's own inputs lead by a median 16 ticks, everyone else's are within a few).
    /// The intervals the ground timing fit does not cover use the midpoint rule moved by the median
    /// of that car's last 15 informative fitted shifts (at least 5 so far).
    pub per_car_control_shift: bool,
    /// Offline: fit one common timing shift of a grounded car's observed control changes against the
    /// second-next fresh car packet and drive the interval to the next fresh packet with it
    /// (`fit_ground_control_timing`). Needs the inferred packet lags; overrides
    /// `lookahead_ground_controls` on the intervals it covers.
    pub fit_ground_control_timing: bool,
    /// Offline: fit one shift of the jump counter's switches for a jump from the ground against the
    /// second-next fresh car packet and drive the interval to the next fresh packet with it
    /// (`fit_jump_timing`). Needs the inferred packet lags and `infer_jump_from_active`.
    pub fit_jump_timing: bool,
    /// Fresh packets, from the next one on, that the flip's pitch cancel is fitted on together (one
    /// cancel for all the intervals, the state reset to each packet); 1 fits the next packet alone.
    pub flip_cancel_packets: usize,
    /// Treat every fresh packet as lag-free (physical tick = frame time) instead of inferring lags:
    /// for replays recorded without replication lag (offline play), where a chain of lag-free
    /// packets fixes the lags only up to a constant and inference advances the states wrongly.
    pub zero_packet_lag: bool,
    /// Plan a dodge whose fitted start falls after the next fresh packet (that packet predates the
    /// dodge: its lag is longer than the counter's) for the interval after that packet, instead of
    /// applying it at the activation frame and losing it when the packet resets the state.
    pub defer_dodge_past_next_packet: bool,
    /// Apply a double jump (a second jump press in the air without a dodge direction) when the double
    /// jump counter turns odd; before this the converter never simulated one.
    pub infer_double_jump: bool,
    /// Set the jump, double jump and flip flags of an airborne car from the replay's counters: a counter
    /// that differs from its value when the car was last on the ground means the action was used since.
    /// The simulation otherwise only knows the actions it applied itself (a jump it never applied leaves
    /// `has_jumped` false, which changes what the car can do next).
    pub flags_from_counters: bool,
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
    /// How the flip's pitch cancel is chosen (`FlipCancelSource`); the default fits the next packet.
    pub flip_cancel_source: FlipCancelSource,
    /// Offline: while a car is flipping, infer how much of the flip's pitch torque a player cancelled
    /// (opposite pitch input, which replays do not carry) by simulating candidates against the
    /// next fresh car packet, and hold the last inferred cancel where no later packet exists.
    pub infer_flip_cancel: bool,
    /// Legacy workaround for RocketSim revisions before `0b02051`, which computed the extra
    /// ball-car hit impulse (reported as `CarHitBall.extra_hit_vel`) but discarded it. Newer
    /// RocketSim applies it, so this must stay off there (it would double count).
    pub apply_hit_extra_impulse: bool,
    /// Clamp reported car and ball velocities to RocketSim's limits after each step. RocketSim
    /// applies its limits at the start of the next tick, so the state it reports after a step can
    /// exceed them (a flipping car by up to 2.2 rad/s), whereas replay states never do.
    pub limit_reported_velocities: bool,
    /// Chain packet lags on whole tick counts: snap each chained interval to an integer (rejecting
    /// pairs more than 0.25 tick from one), so lag differences are exact instead of independently
    /// rounded estimates, and fix the absolute tick with the packets' real-time windows.
    pub exact_tick_lag_chains: bool,
    /// Continue a ball chain across a hit: the exact free-flight paths before and after it meet at
    /// the hit, which fixes the ticks between the two packets (`ball_hit_interval_ticks`). Joins
    /// the ball runs on both sides of a hit into one run with one lag level.
    pub ball_hit_chains: bool,
    /// Place the chain runs of ball and cars on common lag levels: in one frame the ball's physical
    /// tick minus a car's has this mean (ticks; measured 3.1 on the two remote-client games, both
    /// stable across cars, minutes and games) and cars have equal means, so each run's level is
    /// pulled toward the levels of the runs it shares frames with (`place_ball_runs`), inside the
    /// run's feasible range. `None` keeps each run at the middle of its own range.
    pub ball_car_lag_offset: Option<f32>,
    /// Without `ball_car_lag_offset`, estimate it from the hits: the offset at which the hitting
    /// car's hitbox just touches the ball at the last state before the median hit
    /// (`estimate_ball_car_offset`). Needs 20 bridged hits.
    pub estimate_ball_car_lag_offset: bool,
    /// Treat a replay whose chain links (9 in 10) match the frame timeline's gaps as lag-free (a
    /// server's own replay): every fresh packet gets lag 0 instead of an inferred 1-2 ticks.
    pub detect_lag_free_replays: bool,
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
    /// Packet lags supplied from outside (for example true lags recovered from a server recording),
    /// used instead of inferring them. Experiments only.
    #[serde(skip)]
    pub external_packet_lags: Option<Arc<PacketLags>>,
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
            apply_observed_demolitions: true,
            contacts_from_ball_packets: true,
            disable_simulated_demolitions: true,
            sync_boost_pad_pickups: true,
            block_sim_pad_pickups: true,
            boost_pickup_lookahead: false,
            infer_air_steer_controls: true,
            infer_dodge_start: true,
            lookahead_ground_controls: true,
            packet_interval_control_rule: false,
            per_car_control_shift: true,
            fit_ground_control_timing: true,
            fit_jump_timing: true,
            flip_cancel_packets: 1,
            zero_packet_lag: false,
            defer_dodge_past_next_packet: true,
            infer_double_jump: true,
            flags_from_counters: true,
            air_bvp: true,
            fit_on_next_packet: true,
            infer_dodge_first_packet_tick: true,
            flip_cancel_holdout: false,
            flip_cancel_source: FlipCancelSource::NextPacketFit,
            infer_flip_cancel: true,
            apply_hit_extra_impulse: false,
            limit_reported_velocities: true,
            exact_tick_lag_chains: true,
            ball_hit_chains: true,
            ball_car_lag_offset: None,
            estimate_ball_car_lag_offset: true,
            detect_lag_free_replays: true,
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
            external_packet_lags: None,
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

/// An input the converter inferred by fitting a later packet (not observed): a jump press, or a dodge
/// press with its direction and pitch cancel. `tick` is on the replay timeline (120 Hz, like
/// `ConvertedFrame::timeline_tick`) and is the first tick the input takes effect.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FittedInput {
    pub slot: usize,
    /// Dodge only: the replay frame where the dodge counter turned odd (0 for a jump).
    pub activation_frame: usize,
    /// `"jump"` or `"dodge"`.
    pub kind: &'static str,
    pub tick: u64,
    /// Dodge only: RocketSim pitch and yaw controls of the press and the fitted cancel (0..1 of the
    /// flip's pitch torque removed).
    pub pitch: f32,
    pub yaw: f32,
    pub cancel: f32,
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
    /// Applied packet lags for objects with a fresh packet; empty unless `infer_packet_lag`.
    pub packet_lags: Vec<AppliedPacketLag>,
    /// Jump and dodge inputs fitted at this frame's packets (arena ticks converted to the timeline).
    pub fitted_inputs: Vec<FittedInput>,
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

/// Integrates the free-flight rotation of a car (RocketSim's air torque and damping, see
/// `air_angular_velocity_forward`) through consecutive segments of constant controls, given as
/// (controls, ticks). Returns the final rotation and world angular velocity.
pub fn air_state_forward(
    rot_mat_start: Mat3A,
    ang_vel_start: Vec3A,
    segments: &[(AirControls, u32)],
) -> (Mat3A, Vec3A) {
    const TICK: f32 = 1.0 / 120.0;
    let mut rot = rot_mat_start;
    let mut omega = ang_vel_start;
    for &(controls, ticks) in segments {
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
    }
    (rot, omega)
}

/// The rotation vector (radians) that takes `from` to `to`, in the world frame.
fn rotation_vector(from: Mat3A, to: Mat3A) -> Vec3A {
    let delta = Quat::from_mat3a(&(to * from.transpose())).normalize();
    let (axis, angle) = delta.to_axis_angle();
    let angle = if angle > PI { angle - 2.0 * PI } else { angle };
    Vec3A::from(axis) * angle
}

/// Solves the boundary-value problem of free flight: per-segment air controls (segments of `ticks`
/// ticks each) that carry the car from its start rotation and angular velocity to its end rotation
/// and angular velocity, as a Levenberg-Marquardt fit of the forward model that stays as close as
/// possible to `prior` (one control per segment). Six end conditions against three unknowns per
/// segment: two or three segments of a span pin the end state down; with more segments the prior
/// chooses among the solutions. Returns the controls and the remaining end error (radians of
/// rotation, radians per second of angular velocity).
pub fn solve_air_bvp(
    rot_start: Mat3A,
    omega_start: Vec3A,
    rot_end: Mat3A,
    omega_end: Vec3A,
    ticks: &[u32],
    prior: &[AirControls],
) -> (Vec<AirControls>, f32, f32) {
    let mut analytic =
        |segments: &[(AirControls, u32)]| air_state_forward(rot_start, omega_start, segments);
    solve_bvp_with(&mut analytic, rot_end, omega_end, ticks, prior)
}

/// `solve_air_bvp` with the forward model as a function of the per-segment controls and their tick
/// counts (RocketSim itself for a flipping car).
pub fn solve_bvp_with(
    forward: &mut dyn FnMut(&[(AirControls, u32)]) -> (Mat3A, Vec3A),
    rot_end: Mat3A,
    omega_end: Vec3A,
    ticks: &[u32],
    prior: &[AirControls],
) -> (Vec<AirControls>, f32, f32) {
    let n = ticks.len();
    let dim = 3 * n;
    let to_vec = |c: &[AirControls]| -> Vec<f32> {
        c.iter().flat_map(|c| [c.pitch, c.yaw, c.roll]).collect()
    };
    let from_vec = |v: &[f32]| -> Vec<AirControls> {
        v.chunks(3)
            .map(|c| AirControls {
                pitch: c[0],
                yaw: c[1],
                roll: c[2],
            })
            .collect()
    };
    // Residual scales: half a degree of rotation and 0.05 rad/s of angular velocity are one unit.
    const ROT_SCALE: f32 = 0.5 * PI / 180.0;
    const OMEGA_SCALE: f32 = 0.05;
    // Weight of staying at the prior, per unit of control.
    const PRIOR_WEIGHT: f32 = 0.05;
    let forward = std::cell::RefCell::new(forward);
    let residual = |u: &[f32]| -> [f32; 6] {
        let segments: Vec<(AirControls, u32)> =
            from_vec(u).into_iter().zip(ticks.iter().copied()).collect();
        let (rot, omega) = (forward.borrow_mut())(&segments);
        let dr = rotation_vector(rot, rot_end) / ROT_SCALE;
        let dw = (omega_end - omega) / OMEGA_SCALE;
        [dr.x, dr.y, dr.z, dw.x, dw.y, dw.z]
    };
    let u0 = to_vec(prior);
    let mut u = u0.clone();
    let mut lambda = 1.0f32;
    let cost = |u: &[f32]| -> f32 {
        let r = residual(u);
        let prior_cost: f32 = u.iter().zip(&u0).map(|(a, b)| (a - b) * (a - b)).sum();
        r.iter().map(|x| x * x).sum::<f32>() + PRIOR_WEIGHT * PRIOR_WEIGHT * prior_cost
    };
    let mut current = cost(&u);
    for _ in 0..12 {
        let r = residual(&u);
        // Finite-difference Jacobian (6 x dim).
        let mut jac = vec![[0.0f32; 6]; dim];
        for k in 0..dim {
            let mut up = u.clone();
            up[k] += 0.02;
            let rp = residual(&up);
            for i in 0..6 {
                jac[k][i] = (rp[i] - r[i]) / 0.02;
            }
        }
        // (J^T J + w^2 I + lambda I) delta = -(J^T r + w^2 (u - u0)).
        let mut a = vec![vec![0.0f32; dim]; dim];
        let mut g = vec![0.0f32; dim];
        for p in 0..dim {
            for q in 0..dim {
                a[p][q] = (0..6).map(|i| jac[p][i] * jac[q][i]).sum();
            }
            a[p][p] += PRIOR_WEIGHT * PRIOR_WEIGHT + lambda;
            g[p] = -((0..6).map(|i| jac[p][i] * r[i]).sum::<f32>()
                + PRIOR_WEIGHT * PRIOR_WEIGHT * (u[p] - u0[p]));
        }
        // Gaussian elimination with partial pivoting.
        let mut delta = g.clone();
        let mut m = a.clone();
        let mut ok = true;
        for col in 0..dim {
            let pivot = (col..dim)
                .max_by(|&x, &y| m[x][col].abs().total_cmp(&m[y][col].abs()))
                .unwrap_or(col);
            if m[pivot][col].abs() < 1e-9 {
                ok = false;
                break;
            }
            m.swap(col, pivot);
            delta.swap(col, pivot);
            for row in col + 1..dim {
                let factor = m[row][col] / m[col][col];
                for k in col..dim {
                    m[row][k] -= factor * m[col][k];
                }
                delta[row] -= factor * delta[col];
            }
        }
        if !ok {
            break;
        }
        for col in (0..dim).rev() {
            let tail: f32 = (col + 1..dim).map(|k| m[col][k] * delta[k]).sum();
            delta[col] = (delta[col] - tail) / m[col][col];
        }
        let candidate: Vec<f32> = u
            .iter()
            .zip(&delta)
            .map(|(a, d)| (a + d).clamp(-1.0, 1.0))
            .collect();
        let candidate_cost = cost(&candidate);
        if candidate_cost < current {
            let improvement = current - candidate_cost;
            u = candidate;
            current = candidate_cost;
            lambda = (lambda * 0.3).max(1e-4);
            if improvement < 1e-4 {
                break;
            }
        } else {
            lambda *= 4.0;
            if lambda > 1e4 {
                break;
            }
        }
    }
    let r = residual(&u);
    let rot_error = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt() * ROT_SCALE;
    let omega_error = (r[3] * r[3] + r[4] * r[4] + r[5] * r[5]).sqrt() * OMEGA_SCALE;
    (from_vec(&u), rot_error, omega_error)
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
        || predicted
            .wheels_with_contact
            .iter()
            .any(|contact| contact.is_some())
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
        || state
            .wheels_with_contact
            .iter()
            .any(|contact| contact.is_some())
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

/// Body product IDs are from boxcars' TeamLoadout, not RocketSim's preset indices.
/// The embedded map is generated from the user's item catalog, the official
/// Rocket League hitbox roster, and reviewed name aliases. Unknown IDs retain
/// the Octane fallback.
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

/// Lags of zero for every fresh packet (`zero_packet_lag`).
pub fn zero_packet_lags(observations: &ObservedReplay) -> PacketLags {
    let frames = &observations.frames;
    let mut lags = PacketLags {
        ball: vec![None; frames.len()],
        cars: vec![None; frames.len()],
        car_actor: HashMap::new(),
        ..PacketLags::default()
    };
    for (f, frame) in frames.iter().enumerate() {
        if frame
            .ball
            .as_ref()
            .and_then(|b| b.position.as_ref())
            .is_some_and(|p| p.frame == f)
        {
            lags.ball[f] = Some(0.0);
        }
        for car in &frame.cars {
            if car.body.position.as_ref().is_some_and(|p| p.frame == f) {
                lags.cars[f] = Some(0.0);
                lags.car_actor
                    .insert((car.actor_id, car.actor_created_frame, f), 0.0);
            }
        }
    }
    lags
}

#[derive(Debug, Clone, Default)]
pub struct PacketLags {
    /// Ball packet lag in ticks behind each frame time, when inferred.
    pub ball: Vec<Option<f32>>,
    /// Median lag of the cars with a fresh packet in each frame, when inferred.
    pub cars: Vec<Option<f32>>,
    /// Lag of one car's own packet, keyed by (actor id, creation frame, frame).
    pub car_actor: HashMap<(i32, usize, usize), f32>,
    /// Diagnostics of the ball-car placement: the offset used (ticks, ball minus car) and the
    /// number of bridged ball hits found.
    pub ball_car_offset: Option<f32>,
    pub bridged_hits: usize,
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

fn packet_pos(car: &observations::Car, frame: usize) -> [f32; 3] {
    car.body
        .position
        .as_ref()
        .filter(|v| v.frame == frame)
        .map_or([0.0; 3], |v| v.value)
}

fn packet_vel(car: &observations::Car, frame: usize) -> [f32; 3] {
    car.body
        .linear_velocity
        .as_ref()
        .filter(|v| v.frame == frame)
        .map_or([0.0; 3], |v| v.value)
}

/// The ball-minus-car offset at which the median hit is a touch. The gap between the hitting car
/// and the ball at the last state before a hit shrinks as the offset grows (the car is placed
/// earlier relative to the ball). It is evaluated on a grid of offsets with the runs placed by
/// `place_ball_runs`; the first grid step where the median gap drops to zero or below is
/// interpolated linearly. (On the two remote-client games, truth 3.1 ticks: 2.75 and 2.97.)
fn estimate_ball_car_offset(
    hits: &[BallHit],
    ball_runs: &[RawRun],
    car_runs: &[((i32, usize), RawRun)],
    samples: &HashMap<(i32, usize, usize), CarSample>,
    hitboxes: &HashMap<(i32, usize), CarBodyConfig>,
) -> Option<f32> {
    const STEP: f32 = 0.5;
    let mut previous: Option<(f32, f32)> = None;
    for step in -8..=20 {
        let offset = step as f32 * STEP;
        let mut b = ball_runs.to_vec();
        place_ball_runs(&mut b, car_runs, offset);
        let result = median_hit_gap(hits, &b, car_runs, samples, hitboxes);
        if std::env::var_os("OFFSET_PROFILE").is_some() {
            eprintln!("offset {offset}: {result:?} of {} hits", hits.len());
        }
        let (gap, _) = result?;
        if gap <= 0.0 {
            let (before, gap_before) = previous?;
            return Some(before + STEP * gap_before / (gap_before - gap));
        }
        previous = Some((offset, gap));
    }
    None
}

/// One run of chained packets on the integer tick timeline: `(frame, K)` entries (physical tick of
/// each packet relative to the run's first), the feasible integer starts `lo..=hi` of the run (the
/// physical tick of the entry with `K = 0`), and the start chosen so far.
#[derive(Debug, Clone)]
struct RawRun {
    entries: Vec<(usize, i64)>,
    lo: i64,
    hi: i64,
    start: i64,
}

/// Places the ball runs relative to the car runs. A car's physical tick in a frame is
/// `tick - lag` with a lag spread uniformly over the frame window, so a car run is already
/// centred by its own window bounds (`chain_packet_lags_exact`). In one frame the ball's physical
/// tick minus a car's is `offset` on average (standard deviation 2.55 ticks per pair, measured on
/// the two remote-client games; the offset 3.1 there, stable across cars, minutes and games), so
/// every ball run starts at the mean over its frames of (the frame's car ticks + offset - its own
/// K), rounded and clamped to its feasible range. Car runs do not move: pulling them toward
/// the ball's noisy levels made the cars worse.
fn place_ball_runs(ball_runs: &mut [RawRun], car_runs: &[((i32, usize), RawRun)], offset: f32) {
    let mut car_ticks: HashMap<usize, (f64, usize)> = HashMap::new();
    for (_, run) in car_runs {
        for &(frame, k) in &run.entries {
            let entry = car_ticks.entry(frame).or_default();
            entry.0 += (run.start + k) as f64;
            entry.1 += 1;
        }
    }
    for run in ball_runs.iter_mut() {
        let (mut sum, mut count) = (0.0f64, 0usize);
        for &(frame, k) in &run.entries {
            if let Some(&(total, n)) = car_ticks.get(&frame) {
                // The mean car tick of the frame, one term per car.
                sum += n as f64 * (total / n as f64 + f64::from(offset) - k as f64);
                count += n;
            }
        }
        if count > 0 {
            run.start = ((sum / count as f64).round() as i64).clamp(run.lo, run.hi);
        }
    }
}

/// One tick of the ball in free flight (gravity, then the exponential damping; the position moves
/// with the new velocity; measured against server states: 0.008 UU/s and 0.005 UU per tick).
fn ball_free_step(pos: [f32; 3], vel: [f32; 3]) -> ([f32; 3], [f32; 3]) {
    const DT: f32 = 1.0 / 120.0;
    let keep = 0.97f32.powf(DT);
    let vel = [vel[0] * keep, vel[1] * keep, vel[2] * keep - 650.0 * DT];
    (
        [pos[0] + vel[0] * DT, pos[1] + vel[1] * DT, pos[2] + vel[2] * DT],
        vel,
    )
}

/// The inverse of `ball_free_step`: the state one tick earlier.
fn ball_free_step_back(pos: [f32; 3], vel: [f32; 3]) -> ([f32; 3], [f32; 3]) {
    const DT: f32 = 1.0 / 120.0;
    let keep = 0.97f32.powf(DT);
    let before = [pos[0] - vel[0] * DT, pos[1] - vel[1] * DT, pos[2] - vel[2] * DT];
    (
        before,
        [vel[0] / keep, vel[1] / keep, (vel[2] + 650.0 * DT) / keep],
    )
}

/// Whether a ball position is in free flight: away from the floor, ceiling, side walls and goals
/// (a bounce or a wall hit changes the velocity just like a hit does).
fn ball_in_free_air(pos: [f32; 3]) -> bool {
    pos[2] > 125.0 && pos[2] < 1900.0 && pos[0].abs() < 3900.0 && pos[1].abs() < 4900.0
}

/// Elapsed ticks between two ball packets with a hit between them. The ball follows its exact
/// free-flight path up to the hit, and after it the exact path leading to the second packet. The
/// two paths meet at the hit up to the part of the hit tick spent moving with the new velocity:
/// the displacement of that tick lies along the velocity change, by a fraction (measured between
/// -0.4 and +0.4 of one tick of the velocity change at the 10th to 90th percentile; the window
/// here is wider). The candidate elapsed time `d` (ticks between the packets, within the real-time
/// bounds `d_lo..=d_hi` of the frame windows) that leaves the smallest component across the
/// velocity change wins; it must beat every other candidate by more than `HIT_MARGIN` UU, or the
/// pair is not used.
fn ball_hit_interval_ticks(
    a: &ChainPacket,
    b: &ChainPacket,
    d_lo: i64,
    d_hi: i64,
) -> Option<(i64, usize, [f32; 3])> {
    const MIN_VELOCITY_CHANGE: f32 = 400.0;
    const HIT_MARGIN: f32 = 2.0;
    const ALONG_MIN: f32 = -0.6;
    const ALONG_MAX: f32 = 1.2;
    const DT: f32 = 1.0 / 120.0;
    let dv = [b.vel[0] - a.vel[0], b.vel[1] - a.vel[1], b.vel[2] - a.vel[2]];
    let dv_norm = (dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]).sqrt();
    if dv_norm < MIN_VELOCITY_CHANGE || d_hi < d_lo.max(1) || d_hi > 64 {
        return None;
    }
    let dir = [dv[0] / dv_norm, dv[1] / dv_norm, dv[2] / dv_norm];
    let steps = d_hi as usize;
    // Positions along the pre-hit path (forward from a) and the post-hit path (back from b), as
    // long as the ball stays in free flight.
    let mut pre = vec![a.pos];
    let (mut p, mut v) = (a.pos, a.vel);
    for _ in 0..steps {
        (p, v) = ball_free_step(p, v);
        if !ball_in_free_air(p) {
            break;
        }
        pre.push(p);
    }
    let mut post = vec![b.pos];
    let (mut p, mut v) = (b.pos, b.vel);
    for _ in 0..steps {
        (p, v) = ball_free_step_back(p, v);
        if !ball_in_free_air(p) {
            break;
        }
        post.push(p);
    }
    if !ball_in_free_air(a.pos) || !ball_in_free_air(b.pos) {
        return None;
    }
    let mut costs: Vec<(i64, usize, f32)> = Vec::new();
    for d in d_lo.max(1)..=d_hi {
        let mut best = f32::INFINITY;
        let mut best_t1 = 0;
        for t1 in 0..=(d as usize) {
            let t2 = d as usize - t1;
            let (Some(pa), Some(pb)) = (pre.get(t1), post.get(t2)) else {
                continue;
            };
            let r = [pa[0] - pb[0], pa[1] - pb[1], pa[2] - pb[2]];
            let along = r[0] * dir[0] + r[1] * dir[1] + r[2] * dir[2];
            let fraction = along / (dv_norm * DT);
            if !(ALONG_MIN..=ALONG_MAX).contains(&fraction) {
                continue;
            }
            let across = [r[0] - along * dir[0], r[1] - along * dir[1], r[2] - along * dir[2]];
            let across = (across[0] * across[0] + across[1] * across[1] + across[2] * across[2]).sqrt();
            if across < best {
                best = across;
                best_t1 = t1;
            }
        }
        costs.push((d, best_t1, best));
    }
    costs.sort_by(|x, y| x.2.total_cmp(&y.2));
    let (first, second) = (costs.first()?, costs.get(1));
    if !first.2.is_finite() {
        return None;
    }
    match second {
        Some(second) if second.2 - first.2 <= HIT_MARGIN => None,
        // The last state before the hit: `t1` ticks after the first packet.
        _ => Some((first.0, first.1, pre[first.1])),
    }
}

/// A hit between two ball packets that the chain bridged: the last free-flight state before it.
#[derive(Debug, Clone)]
struct BallHit {
    frame_a: usize,
    frame_b: usize,
    ticks_after_a: usize,
    ball_pos: [f32; 3],
}

/// One fresh car sample for the contact check: replay position, velocity, rotation and angular
/// velocity at a frame.
#[derive(Debug, Clone, Copy)]
struct CarSample {
    pos: glam::Vec3A,
    vel: glam::Vec3A,
    rot: Quat,
    ang: glam::Vec3A,
}

/// Median over the bridged hits of the gap between the hitting car's hitbox and the ball at the
/// last state before the hit (the closest car, its fresh samples extrapolated to that tick;
/// negative: overlapping), for run starts as they are now. The extrapolation uses the car's
/// velocity, gravity when it is off the ground, and its angular velocity: over a few ticks the
/// error is a few UU. Returns the median and the number of hits used.
fn median_hit_gap(
    hits: &[BallHit],
    ball_runs: &[RawRun],
    car_runs: &[((i32, usize), RawRun)],
    samples: &HashMap<(i32, usize, usize), CarSample>,
    hitboxes: &HashMap<(i32, usize), CarBodyConfig>,
) -> Option<(f32, usize)> {
    const MAX_EXTRAPOLATION_TICKS: i64 = 12;
    let mut gaps: Vec<f32> = Vec::new();
    for hit in hits {
        let Some(run) = ball_runs.iter().find(|r| {
            r.entries.iter().any(|e| e.0 == hit.frame_a) && r.entries.iter().any(|e| e.0 == hit.frame_b)
        }) else {
            continue;
        };
        let Some(&(_, ka)) = run.entries.iter().find(|e| e.0 == hit.frame_a) else {
            continue;
        };
        let tick = run.start + ka + hit.ticks_after_a as i64;
        let ball = glam::Vec3A::from(hit.ball_pos);
        let mut best: Option<f32> = None;
        for (key, car_run) in car_runs {
            let Some(&(frame, k)) = car_run
                .entries
                .iter()
                .min_by_key(|(_, k)| (car_run.start + k - tick).abs())
            else {
                continue;
            };
            let delta = tick - (car_run.start + k);
            if delta.abs() > MAX_EXTRAPOLATION_TICKS {
                continue;
            }
            let (Some(sample), Some(config)) = (samples.get(&(key.0, key.1, frame)), hitboxes.get(key)) else {
                continue;
            };
            let seconds = delta as f32 / 120.0;
            let airborne = sample.pos.z > 30.0;
            let gravity = if airborne { -650.0 } else { 0.0 };
            let pos = sample.pos
                + sample.vel * seconds
                + glam::Vec3A::new(0.0, 0.0, 0.5 * gravity * seconds * seconds);
            let rot = glam::Mat3A::from_quat(Quat::from_scaled_axis((sample.ang * seconds).into()) * sample.rot);
            let local = rot.transpose() * (ball - pos) - config.hitbox_pos_offset;
            let q = local.abs() - config.hitbox_size * 0.5;
            let gap = q.max(glam::Vec3A::ZERO).length() + q.max_element().min(0.0) - 91.25;
            best = Some(best.map_or(gap, |b: f32| b.min(gap)));
        }
        if let Some(gap) = best {
            gaps.push(gap);
        }
    }
    if gaps.len() < 20 {
        return None;
    }
    gaps.sort_by(|a, b| a.total_cmp(b));
    Some((gaps[gaps.len() / 2], gaps.len()))
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
    fallback: impl Fn(&ChainPacket, &ChainPacket, i64, i64) -> Option<i64>,
    runs: &mut Vec<RawRun>,
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
    let finish = |run: &[(usize, i64)], lo: i64, hi: i64, runs: &mut Vec<RawRun>| {
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
        runs.push(RawRun {
            entries: run.to_vec(),
            lo,
            hi,
            start,
        });
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
        // Physical ticks between two packets lie between the frame times that bracket them.
        let interval = interval.or_else(|| {
            let d_lo = timeline(b.frame.saturating_sub(1)) - timeline(a.frame);
            let d_hi = timeline(b.frame) - timeline(a.frame.saturating_sub(1));
            fallback(a, b, d_lo, d_hi)
        });
        let Some(interval) = interval else {
            finish(&run, lo, hi, runs);
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
            finish(&run, lo, hi, runs);
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
    finish(&run, lo, hi, runs);
}

/// Walks one chain of fresh packets. `valid(prev, next)` decides whether a pair's implied interval
/// is trustworthy; an invalid pair or an infeasible window ends the run.
fn chain_packet_lags(
    observations: &ObservedReplay,
    packets: &[ChainPacket],
    exact: bool,
    valid: impl Fn(&ChainPacket, &ChainPacket) -> bool,
    fallback: impl Fn(&ChainPacket, &ChainPacket, i64, i64) -> Option<i64>,
    runs: &mut Vec<RawRun>,
    mut assign: impl FnMut(usize, f32),
) {
    if exact {
        return chain_packet_lags_exact(observations, packets, valid, fallback, runs);
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
        ..PacketLags::default()
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
    let mut ball_runs: Vec<RawRun> = Vec::new();
    let ball_hits: std::cell::RefCell<Vec<BallHit>> = std::cell::RefCell::new(Vec::new());
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
        |a, b, d_lo, d_hi| {
            let (d, ticks_after_a, ball_pos) = (options.ball_hit_chains
                && bridge_ok(a.frame, b.frame)
                && b.frame - a.frame <= 3)
                .then(|| ball_hit_interval_ticks(a, b, d_lo, d_hi))
                .flatten()?;
            ball_hits.borrow_mut().push(BallHit {
                frame_a: a.frame,
                frame_b: b.frame,
                ticks_after_a,
                ball_pos,
            });
            Some(d)
        },
        &mut ball_runs,
        |frame, lag| lags.ball[frame] = Some(lag),
    );

    // Cars: fast, smooth motion between packets of one actor lifetime (dodges excluded).
    let mut chains: HashMap<(i32, usize), Vec<ChainPacket>> = HashMap::new();
    let mut dodge_frames: HashSet<(i32, usize, usize)> = HashSet::new();
    let mut samples: HashMap<(i32, usize, usize), CarSample> = HashMap::new();
    let mut hitboxes: HashMap<(i32, usize), CarBodyConfig> = HashMap::new();
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
                let key = (car.actor_id, car.actor_created_frame);
                hitboxes.entry(key).or_insert_with(|| {
                    car.body_product_id
                        .as_ref()
                        .and_then(|v| hitbox_for_body_product(v.value))
                        .map_or(CarBodyConfig::OCTANE, |(_, config)| config)
                });
                if let Some(rot) = car
                    .body
                    .rotation_xyzw
                    .as_ref()
                    .and_then(|r| quaternion(r.value))
                {
                    let ang = car
                        .body
                        .angular_velocity_replay_units
                        .as_ref()
                        .map_or([0.0; 3], |v| v.value);
                    samples.insert(
                        (car.actor_id, car.actor_created_frame, frame.index),
                        CarSample {
                            pos: glam::Vec3A::from(packet_pos(car, frame.index)),
                            vel: glam::Vec3A::from(packet_vel(car, frame.index)),
                            rot,
                            ang: glam::Vec3A::from(ang) * 0.01,
                        },
                    );
                }
            }
        }
    }
    let mut per_frame: Vec<Vec<f32>> = vec![Vec::new(); frames.len()];
    let mut car_runs: Vec<((i32, usize), RawRun)> = Vec::new();
    for ((actor, created), packets) in &chains {
        let mut runs: Vec<RawRun> = Vec::new();
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
            |_, _, _, _| None,
            &mut runs,
            |frame, lag| {
                per_frame[frame].push(lag);
                lags.car_actor.insert((*actor, *created, frame), lag);
            },
        );
        car_runs.extend(runs.into_iter().map(|run| ((*actor, *created), run)));
    }
    // A replay saved by the server (host) has every packet fresh at its frame's own tick: its chain
    // links equal the gaps of the frame timeline (99.5-99.7% on two host replays of the remote-client
    // games, 13-47% on 36 client replays of the corpus and 24-25% on the two remote-client replays),
    // while a client's packets have lags that jitter inside the window. Such a replay has no lag.
    if options.detect_lag_free_replays {
        let first_time = f64::from(frames.first().map_or(0.0, |frame| frame.time));
        let tl = |frame: usize| ((f64::from(frames[frame].time) - first_time) * 120.0).round() as i64;
        let (mut equal, mut total) = (0usize, 0usize);
        for run in ball_runs.iter().chain(car_runs.iter().map(|(_, r)| r)) {
            for pair in run.entries.windows(2) {
                total += 1;
                equal += usize::from(pair[1].1 - pair[0].1 == tl(pair[1].0) - tl(pair[0].0));
            }
        }
        if total >= 200 && equal as f64 >= 0.9 * total as f64 {
            return zero_packet_lags(observations);
        }
    }
    let offset = options.ball_car_lag_offset.or_else(|| {
        options
            .estimate_ball_car_lag_offset
            .then(|| {
                estimate_ball_car_offset(&ball_hits.borrow(), &ball_runs, &car_runs, &samples, &hitboxes)
            })
            .flatten()
    });
    if std::env::var_os("LAG_MU_PROFILE").is_some() {
        eprintln!("ball-car lag offset used: {offset:?}");
    }
    lags.ball_car_offset = offset;
    lags.bridged_hits = ball_hits.borrow().len();
    if let Some(offset) = offset {
        place_ball_runs(&mut ball_runs, &car_runs, offset);
    }
    // Exact runs: the lag of an entry is the frame's tick minus its physical tick.
    let first_time = f64::from(frames.first().map_or(0.0, |frame| frame.time));
    let timeline = |frame: usize| ((f64::from(frames[frame].time) - first_time) * 120.0).round() as i64;
    for run in &ball_runs {
        for &(frame, k) in &run.entries {
            lags.ball[frame] = Some((timeline(frame) - (run.start + k)).max(0) as f32);
        }
    }
    for ((actor, created), run) in &car_runs {
        for &(frame, k) in &run.entries {
            let lag = (timeline(frame) - (run.start + k)).max(0) as f32;
            per_frame[frame].push(lag);
            lags.car_actor.insert((*actor, *created, frame), lag);
        }
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

/// Per-tick air controls for one car over the interval to its next fresh packet (`plan_air_bvp`):
/// (first arena tick, controls), in order.
struct AirSchedule {
    slot: usize,
    end_tick: u64,
    entries: Vec<(u64, AirControls)>,
}

/// Plans the air controls of an airborne car for the interval from its fresh packet at `index` to its
/// next fresh packet as a boundary-value problem (`solve_air_bvp`): controls that may change every
/// few ticks carry the car to the next packet's rotation and angular velocity (not just the angular
/// velocity with one constant control, as the span solve does), starting from the constant span
/// solution. The packets' ticks are their frame times minus their lags. Refused for a car that is
/// not in free flight at both packets, a dodge in the span, a withheld or inactive frame, or a
/// solution that does not reach the end state. Uses the next packet: offline reconstruction.
#[allow(clippy::too_many_arguments)]
fn plan_air_bvp(
    observations: &ObservedReplay,
    options: &ConvertOptions,
    packet_lags: &Option<PacketLags>,
    first_time: f32,
    index: usize,
    car: &observations::Car,
    state: &CarState,
    lag_a: u64,
    slot: usize,
    now_tick: u64,
    scratch: Option<&mut Arena>,
    pending: &[PendingDodge],
) -> Option<(AirSchedule, i32)> {
    static TUNING: OnceLock<(f32, f32, f32)> = OnceLock::new();
    let (min_z_value, rot_tol_deg, omega_tol) = *TUNING.get_or_init(|| {
        let get = |k: &str, d: f32| {
            std::env::var(k)
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(d)
        };
        (
            get("AIR_BVP_MIN_Z", 30.0),
            get("AIR_BVP_ROT_TOL", 3.0),
            get("AIR_BVP_OMEGA_TOL", 0.5),
        )
    });
    let press = pending
        .iter()
        .find(|d| d.slot == slot && d.start_tick > now_tick)
        .copied();
    if state.is_on_ground || ((state.is_flipping || press.is_some()) && scratch.is_none()) {
        return air_refused(if state.is_flipping { 0 } else { 9 });
    }
    let frames = &observations.frames;
    let lags = packet_lags.as_ref()?;
    let timeline = |frame: usize| -> i64 {
        ((f64::from(frames[frame].time) - f64::from(first_time)) * 120.0).round() as i64
    };
    let fresh_rotation = |c: &observations::Car, frame: usize| -> Option<(Mat3A, Vec3A, f32)> {
        let p = c.body.position.as_ref().filter(|v| v.frame == frame)?;
        let r = c.body.rotation_xyzw.as_ref().filter(|v| v.frame == frame)?;
        let w = c
            .body
            .angular_velocity_replay_units
            .as_ref()
            .filter(|v| v.frame == frame)?;
        Some((
            Mat3A::from_quat(quaternion(r.value)?),
            vec3(w.value) * 0.01,
            p.value[2],
        ))
    };
    let Some((rot_a, omega_a, z_a)) = fresh_rotation(car, index) else {
        return air_refused(1);
    };
    if z_a < min_z_value {
        return air_refused(2);
    }
    let active = |frame: usize| {
        frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|g| g.value == "Active")
    };
    let withheld = |frame: usize| {
        options
            .withheld_frames
            .as_ref()
            .is_some_and(|w| w.get(frame).copied().unwrap_or(false))
    };
    if !active(index) {
        return air_refused(3);
    }
    let same_car = |c: &&observations::Car| {
        c.actor_id == car.actor_id
            && c.actor_created_frame == car.actor_created_frame
            && c.player_key == car.player_key
    };
    let t_a = timeline(index) - lag_a as i64;
    let mut end = None;
    for g in index + 1..=(index + 24).min(frames.len() - 1) {
        if !active(g) || withheld(g) {
            return air_refused(4);
        }
        let other = frames[g].cars.iter().find(same_car)?;
        let dodge_in_span = other
            .inputs
            .dodge_active_raw
            .as_ref()
            .is_some_and(|d| d.frame == g && d.value % 2 == 1);
        if dodge_in_span
            && !pending
                .iter()
                .any(|d| d.slot == slot && d.start_tick > now_tick)
        {
            return air_refused(5);
        }
        if let Some((rot_b, omega_b, z_b)) = fresh_rotation(other, g) {
            let lag = lags
                .car_actor
                .get(&(car.actor_id, car.actor_created_frame, g))
                .copied()
                .or(lags.cars[g])
                .map_or((timeline(g) - timeline(g - 1)).max(0) / 2, |lag| {
                    lag.round().max(0.0) as i64
                });
            end = Some((g, rot_b, omega_b, z_b, timeline(g) - lag));
            break;
        }
    }
    let Some((_, rot_b, omega_b, z_b, t_b)) = end else {
        return air_refused(6);
    };
    let total = t_b - t_a;
    if z_b < min_z_value || !(2..=90).contains(&total) {
        return air_refused(if z_b < min_z_value { 7 } else { 8 });
    }
    let total = total as u32;
    // Segments of about four ticks (a frame at 30 fps).
    let parts = total.div_ceil(4).max(1);
    let ticks: Vec<u32> = (0..parts)
        .map(|i| total * (i + 1) / parts - total * i / parts)
        .collect();
    let constant = solve_span_air_controls(
        rot_a,
        omega_a,
        omega_b,
        total,
        options.air_lookahead_refine_iterations,
    );
    let prior = vec![constant; ticks.len()];
    let mut shift = 0i32;
    let (solved, rot_error, omega_error) = match scratch {
        Some(scratch) if state.is_flipping || press.is_some() => {
            // A flip is not free flight: its torque, pitch lock and cancel are RocketSim's, so the
            // forward model is RocketSim itself (a single car in a scratch arena, from the packet).
            // The tick of the flip's start is only known to a few ticks (a fitted dodge press, or the
            // flip time a simulated flip has reached), so a shift of it is tried too: the first
            // that reaches the end state, else the one that gets closest.
            let mut best: Option<(Vec<AirControls>, f32, f32, i32)> = None;
            for offset in [0i32, -1, 1, -2, 2, -3, 3, -4, 4, -5, 5, -6, 6] {
                let mut start = *state;
                start.phys.rot_mat = rot_a;
                start.phys.ang_vel = omega_a;
                let mut press_tick = press.map(|d| d.start_tick);
                if let Some(tick) = press_tick.as_mut() {
                    let shifted = *tick as i64 + i64::from(offset);
                    if shifted <= now_tick as i64 || shifted > now_tick as i64 + i64::from(total) {
                        continue;
                    }
                    *tick = shifted as u64;
                } else {
                    let shifted = start.flip_time + offset as f32 / 120.0;
                    if shifted < 0.0 {
                        continue;
                    }
                    start.flip_time = shifted;
                }
                let mut forward = |segments: &[(AirControls, u32)]| -> (Mat3A, Vec3A) {
                    // The ball is parked far from the car: a contact in the scratch would not be the
                    // real one.
                    let mut parked = rocketsim::BallState::default();
                    parked.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
                    if (start.phys.pos - parked.phys.pos).length() < 600.0 {
                        parked.phys.pos = Vec3A::new(3000.0, 4000.0, 300.0);
                    }
                    scratch.set_ball_state(parked);
                    scratch.set_car_state(0, start);
                    let mut tick = now_tick;
                    for &(controls, n) in segments {
                        for _ in 0..n {
                            tick += 1;
                            let mut c = CarControls {
                                pitch: controls.pitch,
                                yaw: controls.yaw,
                                roll: controls.roll,
                                ..CarControls::default()
                            };
                            if let (Some(d), Some(at)) = (press, press_tick) {
                                if tick == at {
                                    c.jump = true;
                                    c.pitch = d.pitch;
                                    c.yaw = d.yaw;
                                    c.roll = 0.0;
                                }
                            }
                            scratch.set_car_controls(0, c);
                            scratch.step_tick();
                        }
                    }
                    let end = scratch.get_car_state(0);
                    let mut omega = end.phys.ang_vel;
                    let speed = omega.length();
                    if speed > AIR_MAX_ANGULAR_SPEED {
                        omega *= AIR_MAX_ANGULAR_SPEED / speed;
                    }
                    (end.phys.rot_mat, omega)
                };
                let (solved, rot_error, omega_error) =
                    solve_bvp_with(&mut forward, rot_b, omega_b, &ticks, &prior);
                let within = rot_error <= rot_tol_deg.to_radians() && omega_error <= omega_tol;
                if let Some(range) = std::env::var("AIR_BVP_DEBUG").ok() {
                    let mut parts = range.split('-').filter_map(|v| v.parse::<usize>().ok());
                    let (lo, hi) = (parts.next().unwrap_or(0), parts.next().unwrap_or(0));
                    if (lo..=hi).contains(&index) {
                        eprintln!(
                            "BVPDEBUG frame {index} slot {slot} ticks {total} flipping {} flip_time {:.3} press {:?} offset {offset}: rot error {:.2} deg, omega error {:.3} rad/s; omega a {:.2} b {:.2}",
                            state.is_flipping,
                            state.flip_time,
                            press_tick.map(|t| t as i64 - now_tick as i64),
                            rot_error.to_degrees(),
                            omega_error,
                            omega_a.length(),
                            omega_b.length()
                        );
                    }
                }
                let better = best.as_ref().is_none_or(|b| {
                    rot_error / rot_tol_deg.to_radians() + omega_error / omega_tol
                        < b.1 / rot_tol_deg.to_radians() + b.2 / omega_tol
                });
                if better {
                    best = Some((solved, rot_error, omega_error, offset));
                }
                if within {
                    break;
                }
            }
            let (solved, rot_error, omega_error, offset) = best?;
            shift = offset;
            (solved, rot_error, omega_error)
        }
        _ => solve_air_bvp(rot_a, omega_a, rot_b, omega_b, &ticks, &prior),
    };
    // A solution that cannot reach the end state means the free-flight model does not hold (a
    // contact, a wall, an unseen flip): leave the interval to the other control paths.
    if rot_error > rot_tol_deg.to_radians() || omega_error > omega_tol {
        if std::env::var_os("AIR_NOSOL").is_some() {
            eprintln!(
                "NOSOL frame {index} pos {:.2} {:.2} ticks {total} flip_time {:.3} flipping {} press {} z {:.0} speed {:.0} rot_err {:.1} deg omega_err {:.2} omega_a {:.2} omega_b {:.2}",
                state.phys.pos.x,
                state.phys.pos.y,
                state.flip_time,
                state.is_flipping,
                press.is_some(),
                state.phys.pos.z,
                state.phys.vel.length(),
                rot_error.to_degrees(),
                omega_error,
                omega_a.length(),
                omega_b.length()
            );
        }
        return air_refused(10);
    }
    let mut entries = Vec::with_capacity(ticks.len());
    let mut tick = now_tick + 1;
    for (controls, n) in solved.iter().zip(&ticks) {
        entries.push((tick, *controls));
        tick += u64::from(*n);
    }
    Some((
        AirSchedule {
            slot,
            end_tick: now_tick + u64::from(total),
            entries,
        },
        shift,
    ))
}

/// Refusal counters of `plan_air_bvp` (diagnostics): 0 flipping at the packet, 1 no fresh rotation,
/// 2 low at the first packet, 3 inactive, 4 inactive or withheld frame in the span, 5 dodge in the
/// span, 6 no next packet within 24 frames, 7 low at the next packet, 8 span of unsupported length,
/// 9 on the ground, 10 no solution that reaches the end state.
pub static AIR_BVP_REFUSALS: [std::sync::atomic::AtomicUsize; 11] =
    [const { std::sync::atomic::AtomicUsize::new(0) }; 11];

/// The physical tick (timeline) from which the controls first seen in frame `g` act. The middle of
/// the car's packet interval `(S_prev, S_cur]` with exact chain lags (see
/// `ConvertOptions::packet_interval_control_rule`), otherwise `2 + spacing / 2` ticks before the
/// frame time.
fn control_change_tick(
    observations: &ObservedReplay,
    options: &ConvertOptions,
    lags: Option<&PacketLags>,
    first_time: f32,
    car: &observations::Car,
    g: usize,
) -> i64 {
    let frames = &observations.frames;
    let timeline = |frame: usize| -> i64 {
        ((f64::from(frames[frame].time) - f64::from(first_time)) * 120.0).round() as i64
    };
    let spacing = if g == 0 { 4 } else { timeline(g) - timeline(g - 1) };
    packet_interval_change_tick(observations, options, lags, first_time, car, g)
        .unwrap_or(timeline(g) - 2 - spacing / 2)
}

/// The middle of the car's packet interval `(S_prev, S_cur]` for the control change first seen in
/// frame `g`, when both packets have exact chain lags (`packet_interval_control_rule`).
fn packet_interval_change_tick(
    observations: &ObservedReplay,
    options: &ConvertOptions,
    lags: Option<&PacketLags>,
    first_time: f32,
    car: &observations::Car,
    g: usize,
) -> Option<i64> {
    if !options.packet_interval_control_rule {
        return None;
    }
    let lags = lags?;
    let frames = &observations.frames;
    let timeline = |frame: usize| -> i64 {
        ((f64::from(frames[frame].time) - f64::from(first_time)) * 120.0).round() as i64
    };
    let key = |frame: usize| (car.actor_id, car.actor_created_frame, frame);
    let s_cur = timeline(g) - lags.car_actor.get(&key(g))?.round() as i64;
    let h = (g.saturating_sub(8)..g)
        .rev()
        .find(|&h| lags.car_actor.contains_key(&key(h)))?;
    let s_prev = timeline(h) - lags.car_actor[&key(h)].round() as i64;
    (s_prev < s_cur && s_cur - s_prev <= 40).then(|| (s_prev + s_cur + 1).div_euclid(2))
}

fn air_refused<T>(reason: usize) -> Option<T> {
    AIR_BVP_REFUSALS[reason].fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    None
}

/// Diagnostic: (actor id, shift chosen by `fit_ground_control_timing`, whether it is the best), in
/// the order the fits ran.
pub static GROUND_SHIFT_LOG: std::sync::Mutex<Vec<(i32, i64, bool)>> = std::sync::Mutex::new(Vec::new());

/// Shifts, in ticks later than the midpoint rule, tried for the observed control changes.
const GROUND_TIMING_SHIFTS: std::ops::RangeInclusive<i64> = -8..=40;

/// Fits when the observed throttle, steer, handbrake and boost changes took effect. A control change
/// is first seen in the frame after it happened and each frame's state is 0-4 ticks older than its
/// time, so the change tick is uncertain by several ticks per event (`diagnose_control_latency`
/// shows the best shift is spread over the whole range and does not carry from one interval to
/// the next). For a grounded car with a fresh packet at `index`, one common shift of every control
/// switch (relative to the midpoint rule of `lookahead_ground_controls`) is chosen by simulating the
/// span to the *second* next fresh packet in a scratch arena and comparing angular velocity (per
/// 0.3 rad/s) and velocity (per 50 UU/s) with it. The returned schedule covers only the interval
/// to the *next* fresh packet, so that packet is not used by the fit and the residual there stays a
/// held-out check. Uses later packets (offline reconstruction). Refused without exact chain lags for
/// both later packets, with a withheld or inactive frame, a jump/dodge counter change, a car that
/// is not on a surface at the first packet (any surface: floor, wall, ramp or ceiling), or the ball
/// within 400 UU (the scratch
/// arena's ball is parked; other cars are not modelled either, and refusing spans near them removed
/// coverage without protecting the fit: near-other-car velocity p90 65.5 to 54.5 UU/s without it).
#[allow(clippy::too_many_arguments)]
fn fit_ground_control_timing(
    observations: &ObservedReplay,
    options: &ConvertOptions,
    packet_lags: &Option<PacketLags>,
    first_time: f32,
    index: usize,
    car: &observations::Car,
    state: &CarState,
    lag_a: u64,
    slot: usize,
    now_tick: u64,
    scratch: &mut Arena,
) -> Option<GroundSchedule> {
    let frames = &observations.frames;
    let lags = packet_lags.as_ref()?;
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
    // On any surface (floor, wall, ramp or ceiling); the counters rule out a jump or dodge in the span.
    if !active(index) || !state.is_on_ground {
        return None;
    }
    let same_car = |c: &&observations::Car| {
        c.actor_id == car.actor_id
            && c.actor_created_frame == car.actor_created_frame
            && c.player_key == car.player_key
    };
    let counters = |c: &observations::Car| {
        [
            c.inputs.jump_active_raw.as_ref().map(|v| v.value),
            c.inputs.double_jump_active_raw.as_ref().map(|v| v.value),
            c.inputs.dodge_active_raw.as_ref().map(|v| v.value),
            c.inputs.flip_car_active_raw.as_ref().map(|v| v.value),
        ]
    };
    let a_counters = counters(car);
    if a_counters.iter().flatten().any(|c| c % 2 == 1) {
        return None;
    }
    let clear = |g: usize, pos: Vec3A| {
        frames[g].ball.as_ref().is_none_or(|ball| {
            ball.position
                .as_ref()
                .is_none_or(|p| (vec3(p.value) - pos).length() > 400.0)
        })
    };
    if !clear(index, state.phys.pos) {
        return None;
    }
    let t_a = timeline(index) - lag_a as i64;
    // The next two fresh packets with exact chain lags.
    let mut found: Vec<(usize, i64, Vec3A, Vec3A)> = Vec::new();
    for g in index + 1..=(index + 12).min(frames.len() - 1) {
        if !active(g) || withheld(g) {
            return None;
        }
        let other = frames[g].cars.iter().find(same_car)?;
        if counters(other) != a_counters {
            return None;
        }
        let b = &other.body;
        let (Some(p), Some(v), Some(w)) = (
            b.position.as_ref().filter(|x| x.frame == g),
            b.linear_velocity.as_ref().filter(|x| x.frame == g),
            b.angular_velocity_replay_units
                .as_ref()
                .filter(|x| x.frame == g),
        ) else {
            continue;
        };
        let Some(lag) = lags
            .car_actor
            .get(&(car.actor_id, car.actor_created_frame, g))
            .copied()
        else {
            continue;
        };
        if !clear(g, vec3(p.value)) {
            return None;
        }
        found.push((
            g,
            timeline(g) - lag.round().max(0.0) as i64,
            vec3(v.value),
            vec3(w.value) * 0.01,
        ));
        if found.len() == 2 {
            break;
        }
    }
    let [first, second] = found[..] else {
        return None;
    };
    let (_, t_b, _, _) = first;
    let (last_frame, t_c, target_vel, target_ang) = if options.fit_on_next_packet {
        first
    } else {
        second
    };
    let (ticks_ab, ticks_ac) = (t_b - t_a, t_c - t_a);
    if ticks_ab < 1 || (ticks_ac <= ticks_ab && !options.fit_on_next_packet) || ticks_ac > 24 {
        return None;
    }
    // Observed controls of the frames around the span, with midpoint-rule switch ticks.
    let mut entries: Vec<(i64, f32, f32, bool, bool)> = Vec::new();
    for g in index.saturating_sub(3)..=(last_frame + 1).min(frames.len() - 1) {
        if !active(g) {
            continue;
        }
        let Some(other) = frames[g].cars.iter().find(same_car) else {
            continue;
        };
        let controls = controls_from_observation(other, options);
        entries.push((
            control_change_tick(observations, options, Some(lags), first_time, other, g),
            controls.throttle,
            controls.steer,
            controls.handbrake,
            controls.boost,
        ));
    }
    let own = controls_from_observation(car, options);
    let own_entry = (t_a, own.throttle, own.steer, own.handbrake, own.boost);
    let changes = entries
        .iter()
        .filter(|e| e.0 >= t_a - 16 && e.0 <= t_c + 16)
        .collect::<Vec<_>>()
        .windows(2)
        .any(|w| {
            (w[0].1 - w[1].1).abs() > 0.1 || (w[0].2 - w[1].2).abs() > 0.1 || w[0].3 != w[1].3
        });
    if !changes {
        return None;
    }
    let controls_at = |entries: &[(i64, f32, f32, bool, bool)], shift: i64, tau: i64| {
        let i = entries.partition_point(|e| e.0 + shift <= tau);
        if i == 0 { own_entry } else { entries[i - 1] }
    };
    let mut costs: Vec<(i64, f32)> = Vec::new();
    // The scratch arena holds only this car: the ball is parked out of reach so that it cannot
    // touch the car (the fit uses spans with no ball or car nearby).
    let mut parked = rocketsim::BallState::default();
    parked.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
    for shift in GROUND_TIMING_SHIFTS {
        scratch.set_ball_state(parked);
        scratch.set_car_state(0, *state);
        for tau in t_a + 1..=t_c {
            let e = controls_at(&entries, shift, tau);
            scratch.set_car_controls(
                0,
                CarControls {
                    throttle: e.1,
                    steer: e.2,
                    handbrake: e.3,
                    boost: e.4,
                    ..CarControls::default()
                },
            );
            scratch.step_tick();
        }
        let end = scratch.get_car_state(0);
        let cost = (end.phys.ang_vel - target_ang).length() / 0.3
            + (end.phys.vel - target_vel).length() / 50.0;
        costs.push((shift, cost));
    }
    // The midpoint rule (shift 0) unless another shift is strictly better.
    let mut best = costs.iter().position(|(shift, _)| *shift == 0)?;
    for (i, (_, cost)) in costs.iter().enumerate() {
        if *cost < costs[best].1 - 1e-6 {
            best = i;
        }
    }
    let shift = costs[best].0;
    if let Ok(mut log) = GROUND_SHIFT_LOG.lock() {
        log.push((car.actor_id, shift, costs[best].1 < costs.iter().map(|c| c.1).fold(f32::INFINITY, f32::min) + 1e-6));
    }
    // The schedule for the interval to the next packet, in arena ticks.
    let mut schedule = vec![(
        now_tick,
        own_entry.1,
        own_entry.2,
        own_entry.3,
        own_entry.4,
        None,
    )];
    for e in &entries {
        let tick = now_tick as i64 + (e.0 + shift - t_a);
        if tick > now_tick as i64 {
            schedule.push((tick as u64, e.1, e.2, e.3, e.4, None));
        } else {
            // A switch at or before the packet replaces the starting controls.
            schedule[0] = (now_tick, e.1, e.2, e.3, e.4, None);
        }
    }
    let zero_cost = costs.iter().find(|(s, _)| *s == 0).map(|c| c.1);
    Some(GroundSchedule {
        slot,
        end_tick: now_tick + ticks_ab as u64,
        entries: schedule,
        shift: zero_cost.filter(|&z| costs[best].1 < z - 1e-6).map(|_| shift),
    })
}

/// Shifts, in ticks later than the midpoint rule, tried for the jump counter's switches.
const JUMP_TIMING_SHIFTS: std::ops::RangeInclusive<i64> = -8..=16;

/// Fits when a jump physically started. The jump counter turns odd in the frame after the press was
/// applied and each frame's state is 0-4 ticks older than its time, and the two windows do not
/// line up: the fitted start is at 0-3 ticks after the midpoint-rule tick in two thirds of the
/// events and up to 12 ticks later in the rest, per event (`diagnose_jump_latency`: a per-player
/// median of other events' shifts does not remove the tail, while a per-event fit does). For a car
/// on a surface (floor, wall, ramp or ceiling) with a fresh packet at `index` and an even jump counter that turns odd before
/// the second next fresh packet, one shift of the jump counter's switches (press and release move
/// together) is chosen by simulating the span to that packet in a scratch arena (position error plus
/// 0.1 x velocity error), and the interval to the *next* fresh packet is driven with it, so that
/// packet is not used by the fit and its residual stays a held-out check. Other controls use the
/// midpoint rule. Uses later packets (offline reconstruction). Refused without exact chain lags for
/// both later packets, spans over 30 ticks, a withheld or inactive frame, a change of double-jump,
/// dodge or flip counter. The ball is simulated (its state at this packet's time in the main
/// arena), so jumps at the ball are fitted with their contacts; other cars are not modelled.
#[allow(clippy::too_many_arguments)]
fn fit_jump_timing(
    observations: &ObservedReplay,
    options: &ConvertOptions,
    packet_lags: &Option<PacketLags>,
    first_time: f32,
    index: usize,
    car: &observations::Car,
    state: &CarState,
    lag_a: u64,
    slot: usize,
    now_tick: u64,
    ball: &rocketsim::BallState,
    scratch: &mut Arena,
) -> Option<GroundSchedule> {
    let frames = &observations.frames;
    let lags = packet_lags.as_ref()?;
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
    // On any surface (floor, wall, ramp or ceiling): a jump leaves it along the surface normal.
    if !options.infer_jump_from_active || !active(index) || !state.is_on_ground {
        return None;
    }
    let same_car = |c: &&observations::Car| {
        c.actor_id == car.actor_id
            && c.actor_created_frame == car.actor_created_frame
            && c.player_key == car.player_key
    };
    let others = |c: &observations::Car| {
        [
            c.inputs.double_jump_active_raw.as_ref().map(|v| v.value),
            c.inputs.dodge_active_raw.as_ref().map(|v| v.value),
            c.inputs.flip_car_active_raw.as_ref().map(|v| v.value),
        ]
    };
    let a_others = others(car);
    let jump_odd = |c: &observations::Car| {
        c.inputs
            .jump_active_raw
            .as_ref()
            .is_some_and(|v| v.value % 2 == 1)
    };
    if a_others.iter().flatten().any(|c| c % 2 == 1) || jump_odd(car) {
        return None;
    }
    let t_a = timeline(index) - lag_a as i64;
    let mut found: Vec<(usize, i64, Vec3A, Vec3A)> = Vec::new();
    for g in index + 1..=(index + 12).min(frames.len() - 1) {
        if !active(g) || withheld(g) {
            return None;
        }
        let other = frames[g].cars.iter().find(same_car)?;
        if others(other) != a_others {
            return None;
        }
        let b = &other.body;
        let (Some(p), Some(v)) = (
            b.position.as_ref().filter(|x| x.frame == g),
            b.linear_velocity.as_ref().filter(|x| x.frame == g),
        ) else {
            continue;
        };
        let Some(lag) = lags
            .car_actor
            .get(&(car.actor_id, car.actor_created_frame, g))
            .copied()
        else {
            continue;
        };
        found.push((
            g,
            timeline(g) - lag.round().max(0.0) as i64,
            vec3(p.value),
            vec3(v.value),
        ));
        if found.len() == 2 {
            break;
        }
    }
    let [first, second] = found[..] else {
        return None;
    };
    let (_, t_b, _, _) = first;
    let (last_frame, t_c, target_pos, target_vel) = if options.fit_on_next_packet {
        first
    } else {
        second
    };
    let (ticks_ab, ticks_ac) = (t_b - t_a, t_c - t_a);
    if ticks_ab < 1 || (ticks_ac <= ticks_ab && !options.fit_on_next_packet) || ticks_ac > 30 {
        return None;
    }
    // (nominal tick, midpoint-rule start tick, throttle, steer, handbrake, boost, jump) per frame.
    let mut entries: Vec<(i64, i64, f32, f32, bool, bool, bool)> = Vec::new();
    for g in index.saturating_sub(3)..=(last_frame + 1).min(frames.len() - 1) {
        if !active(g) {
            continue;
        }
        let Some(other) = frames[g].cars.iter().find(same_car) else {
            continue;
        };
        let mut controls = controls_from_observation(other, options);
        controls.jump = jump_odd(other);
        entries.push((
            timeline(g),
            control_change_tick(observations, options, Some(lags), first_time, other, g),
            controls.throttle,
            controls.steer,
            controls.handbrake,
            controls.boost,
            controls.jump,
        ));
    }
    // The counter must turn odd after a and no later than the second next packet.
    if !entries
        .iter()
        .any(|e| e.6 && e.0 > t_a - 4 && e.0 <= t_c + 4)
    {
        return None;
    }
    let own = controls_from_observation(car, options);
    let own_entry = (own.throttle, own.steer, own.handbrake, own.boost);
    // Controls at arena-independent tick `tau` for a jump shift `shift`.
    let controls_at = |shift: i64, tau: i64| -> (f32, f32, bool, bool, bool) {
        let i = entries.partition_point(|e| e.1 <= tau);
        let base = if i == 0 {
            own_entry
        } else {
            let e = entries[i - 1];
            (e.2, e.3, e.4, e.5)
        };
        let j = entries.partition_point(|e| e.1 + shift <= tau);
        let jump = j > 0 && entries[j - 1].6;
        (base.0, base.1, base.2, base.3, jump)
    };
    let mut costs: Vec<(i64, f32)> = Vec::new();
    for shift in JUMP_TIMING_SHIFTS {
        scratch.set_ball_state(*ball);
        scratch.set_car_state(0, *state);
        for tau in t_a + 1..=t_c {
            let c = controls_at(shift, tau);
            scratch.set_car_controls(
                0,
                CarControls {
                    throttle: c.0,
                    steer: c.1,
                    handbrake: c.2,
                    boost: c.3,
                    jump: c.4,
                    ..CarControls::default()
                },
            );
            scratch.step_tick();
        }
        let end = scratch.get_car_state(0);
        let cost =
            (end.phys.pos - target_pos).length() + 0.1 * (end.phys.vel - target_vel).length();
        costs.push((shift, cost));
    }
    // The midpoint rule (shift 0) unless another shift is strictly better.
    let mut best = costs.iter().position(|(shift, _)| *shift == 0)?;
    for (i, (_, cost)) in costs.iter().enumerate() {
        if *cost < costs[best].1 - 1e-4 {
            best = i;
        }
    }
    let shift = costs[best].0;
    // Per-tick schedule for the interval to the next packet, in arena ticks.
    let mut schedule: Vec<(u64, f32, f32, bool, bool, Option<bool>)> = Vec::new();
    let mut previous = None;
    for step in 1..=ticks_ab {
        let c = controls_at(shift, t_a + step);
        if previous != Some(c) {
            schedule.push((now_tick + step as u64, c.0, c.1, c.2, c.3, Some(c.4)));
            previous = Some(c);
        }
    }
    Some(GroundSchedule {
        slot,
        end_tick: now_tick + ticks_ab as u64,
        entries: schedule,
        shift: None,
    })
}

/// A ground jump followed by a dodge, fitted together: the jump schedule for the interval to the next
/// fresh packet and, when the dodge press falls inside it, the dodge plan.
struct FlipFit {
    schedule: GroundSchedule,
    dodge: Option<DodgePlan>,
    /// The first fresh packet after the activation and the lag (ticks) inferred for it.
    first_packet: Option<(usize, u64)>,
}

/// Fits a jump from the ground and the dodge that follows it (both counters turn odd before the
/// second-next fresh packet). One shift of the jump counter's switches and the dodge press tick
/// (relative to the midpoint-rule tick of the activation frame) are searched on the *second* next
/// fresh packet (exact chain lag; position and velocity), the dodge press from a saved no-dodge
/// path per jump shift, then the pitch cancel from its angular velocity. The plan drives only the
/// interval to the *next* fresh packet (the jump input per tick, and the dodge if its press falls
/// inside it), so that packet is not used by the fit. Uses later packets (offline reconstruction).
/// Refused for a car not on a surface, uneven double-jump or flip counters, spans over 45
/// ticks, or a withheld or inactive frame. The ball is simulated (its state at this packet's time in
/// the main arena); other cars are not.
#[allow(clippy::too_many_arguments)]
fn fit_ground_flip_timing(
    observations: &ObservedReplay,
    options: &ConvertOptions,
    packet_lags: &Option<PacketLags>,
    first_time: f32,
    index: usize,
    car: &observations::Car,
    state: &CarState,
    lag_a: u64,
    slot: usize,
    now_tick: u64,
    ball: &rocketsim::BallState,
    scratch: &mut Arena,
) -> Option<FlipFit> {
    let frames = &observations.frames;
    let lags = packet_lags.as_ref()?;
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
    if !options.infer_jump_from_active
        || !options.infer_dodge_from_active
        || !active(index)
        || !state.is_on_ground
    {
        return refused_ground(1);
    }
    let same_car = |c: &&observations::Car| {
        c.actor_id == car.actor_id
            && c.actor_created_frame == car.actor_created_frame
            && c.player_key == car.player_key
    };
    let parity = |v: &Option<observations::Value<u8>>| v.as_ref().map(|v| v.value);
    let others = |c: &observations::Car| {
        [
            parity(&c.inputs.double_jump_active_raw),
            parity(&c.inputs.flip_car_active_raw),
        ]
    };
    let jump_odd =
        |c: &observations::Car| parity(&c.inputs.jump_active_raw).is_some_and(|v| v % 2 == 1);
    let dodge_of = |c: &observations::Car| parity(&c.inputs.dodge_active_raw);
    let a_others = others(car);
    if a_others.iter().flatten().any(|c| c % 2 == 1)
        || jump_odd(car)
        || dodge_of(car).is_some_and(|d| d % 2 == 1)
    {
        return refused_ground(2);
    }
    let t_a = timeline(index) - lag_a as i64;
    // The dodge activation: the first frame with a fresh odd dodge counter and a fresh torque.
    let last = (index + 14).min(frames.len() - 1);
    let mut activation = None;
    for g in index + 1..=last {
        if !active(g) || withheld(g) {
            return refused_ground(3);
        }
        let other = frames[g].cars.iter().find(same_car)?;
        if others(other) != a_others {
            return refused_ground(4);
        }
        if let Some(dodge) = other
            .inputs
            .dodge_active_raw
            .as_ref()
            .filter(|d| d.frame == g)
        {
            if dodge.value % 2 == 1 {
                if let Some(torque) = activation_torque(frames, g, other) {
                    activation = Some((g, torque));
                    break;
                }
            }
        }
    }
    let (activation_frame, torque) = activation?;
    let [tx, ty, _] = torque;
    let (pitch, yaw) = ((-ty / 2.24).clamp(-1.0, 1.0), (-tx / 2.60).clamp(-1.0, 1.0));
    if (pitch * pitch + yaw * yaw).sqrt() <= 0.01 {
        return refused_ground(5);
    }
    // The next two fresh packets after a: the first is the next reset of the state (its own lag,
    // exact or not) and stays held out; the second needs an exact chain lag and is the fit target.
    let mut fresh: Vec<(usize, i64, Vec3A, Vec3A, Vec3A)> = Vec::new();
    for g in index + 1..=(index + 16).min(frames.len() - 1) {
        if !active(g) || withheld(g) {
            return refused_ground(6);
        }
        let other = frames[g].cars.iter().find(same_car)?;
        if others(other) != a_others {
            return refused_ground(7);
        }
        let b = &other.body;
        let (Some(p), Some(v), Some(w)) = (
            b.position.as_ref().filter(|x| x.frame == g),
            b.linear_velocity.as_ref().filter(|x| x.frame == g),
            b.angular_velocity_replay_units
                .as_ref()
                .filter(|x| x.frame == g),
        ) else {
            continue;
        };
        let chain = lags
            .car_actor
            .get(&(car.actor_id, car.actor_created_frame, g))
            .copied();
        let lag = if fresh.is_empty() {
            chain
                .or(lags.cars[g])
                .unwrap_or((timeline(g) - timeline(g - 1)).max(0) as f32 / 2.0)
        } else {
            let Some(lag) = chain else {
                continue;
            };
            lag
        };
        fresh.push((
            g,
            timeline(g) - lag.round().max(0.0) as i64,
            vec3(p.value),
            vec3(v.value),
            vec3(w.value) * 0.01,
        ));
        if fresh.len() == 2 {
            break;
        }
    }
    let [
        first_fresh,
        (last_frame, t_c, target_pos, target_vel, target_ang),
    ] = fresh[..]
    else {
        return refused_ground(8);
    };
    let (_, t_b, ..) = first_fresh;
    let (ticks_ab, ticks_ac) = (t_b - t_a, t_c - t_a);
    if ticks_ab < 1 {
        return refused_ground(30);
    }
    if ticks_ac <= ticks_ab {
        return refused_ground(31);
    }
    if ticks_ac > 45 {
        return refused_ground(32);
    }
    if activation_frame > last_frame {
        return refused_ground(33);
    }
    // (nominal tick, midpoint-rule start tick, throttle, steer, handbrake, boost, jump) per frame.
    let mut entries: Vec<(i64, i64, f32, f32, bool, bool, bool)> = Vec::new();
    for g in index.saturating_sub(3)..=(last_frame + 1).min(frames.len() - 1) {
        if !active(g) {
            continue;
        }
        let Some(other) = frames[g].cars.iter().find(same_car) else {
            continue;
        };
        let mut controls = controls_from_observation(other, options);
        controls.jump = jump_odd(other);
        entries.push((
            timeline(g),
            control_change_tick(observations, options, Some(lags), first_time, other, g),
            controls.throttle,
            controls.steer,
            controls.handbrake,
            controls.boost,
            controls.jump,
        ));
    }
    // The jump counter must turn odd after a and no later than the activation.
    if !entries
        .iter()
        .any(|e| e.6 && e.0 > t_a - 4 && e.0 <= timeline(activation_frame) + 4)
    {
        return refused_ground(10);
    }
    let act_start = entries
        .iter()
        .find(|e| e.0 == timeline(activation_frame))
        .map(|e| e.1)?;
    let own = controls_from_observation(car, options);
    let own_entry = (own.throttle, own.steer, own.handbrake, own.boost);
    let controls_at = |shift: i64, tau: i64| -> (f32, f32, bool, bool, bool) {
        let i = entries.partition_point(|e| e.1 <= tau);
        let base = if i == 0 {
            own_entry
        } else {
            let e = entries[i - 1];
            (e.2, e.3, e.4, e.5)
        };
        let j = entries.partition_point(|e| e.1 + shift <= tau);
        let jump = j > 0 && entries[j - 1].6;
        (base.0, base.1, base.2, base.3, jump)
    };
    let horizon = ticks_ac as usize;
    let mut best: Option<(i64, i64, f32)> = None; // (jump shift, press tick relative to a, cost)
    for shift in JUMP_TIMING_SHIFTS {
        // The path with the jump at this shift and no dodge, saved tick by tick.
        let mut path = vec![*state];
        let mut path_ball = vec![*ball];
        scratch.set_ball_state(*ball);
        scratch.set_car_state(0, *state);
        for step in 1..=horizon {
            let c = controls_at(shift, t_a + step as i64);
            scratch.set_car_controls(
                0,
                CarControls {
                    throttle: c.0,
                    steer: c.1,
                    handbrake: c.2,
                    boost: c.3,
                    jump: c.4,
                    ..CarControls::default()
                },
            );
            scratch.step_tick();
            path.push(*scratch.get_car_state(0));
            path_ball.push(*scratch.get_ball_state());
        }
        for d in JUMP_TIMING_SHIFTS {
            let press = act_start + d - t_a;
            if !(2..=horizon as i64).contains(&press) {
                continue;
            }
            let press = press as usize;
            scratch.set_ball_state(path_ball[press - 1]);
            scratch.set_car_state(0, path[press - 1]);
            for step in press..=horizon {
                let c = controls_at(shift, t_a + step as i64);
                let mut controls = CarControls {
                    throttle: c.0,
                    steer: c.1,
                    handbrake: c.2,
                    boost: c.3,
                    ..CarControls::default()
                };
                if step == press {
                    controls.jump = true;
                    controls.pitch = pitch;
                    controls.yaw = yaw;
                }
                scratch.set_car_controls(0, controls);
                scratch.step_tick();
            }
            let end = scratch.get_car_state(0);
            let cost =
                (end.phys.pos - target_pos).length() + 0.1 * (end.phys.vel - target_vel).length();
            if best.is_none_or(|(_, _, c)| cost < c - 1e-4) {
                best = Some((shift, press as i64, cost));
            }
        }
    }
    let (shift, press, _) = best?;
    // The pitch cancel from the angular velocity at the second-next packet.
    let press_u = press as usize;
    let mut best_cancel: Option<(f32, f32)> = None;
    for step in 0..=4 {
        let cancel = step as f32 * 0.25;
        // Rebuild the state at the press for this jump shift.
        scratch.set_ball_state(*ball);
        scratch.set_car_state(0, *state);
        for step_tick in 1..=horizon {
            let c = controls_at(shift, t_a + step_tick as i64);
            let mut controls = CarControls {
                throttle: c.0,
                steer: c.1,
                handbrake: c.2,
                boost: c.3,
                jump: c.4,
                ..CarControls::default()
            };
            if step_tick == press_u {
                controls.jump = true;
                controls.pitch = pitch;
                controls.yaw = yaw;
            } else if step_tick > press_u {
                controls.jump = false;
                let sign = scratch.get_car_state(0).flip_rel_torque.y.signum();
                controls.pitch = cancel * sign;
            }
            scratch.set_car_controls(0, controls);
            scratch.step_tick();
        }
        let mut end = *scratch.get_car_state(0);
        let speed = end.phys.ang_vel.length();
        if speed > 5.5 {
            end.phys.ang_vel *= 5.5 / speed;
        }
        let error = (end.phys.ang_vel - target_ang).length();
        if best_cancel.is_none_or(|(_, e)| error < e - 1e-4) {
            best_cancel = Some((cancel, error));
        }
    }
    let cancel = best_cancel?.0;
    // The first fresh packet after the activation has no chain lag (see `fit_dodge_start`): its tick is
    // the one within the lag range 0-4 ticks before its frame time at which the path with the fitted
    // jump, start and cancel reproduces it (position and velocity).
    let mut ticks_ab_eff = ticks_ab;
    let mut first_packet = None;
    if options.infer_dodge_first_packet_tick && first_fresh.0 >= activation_frame {
        scratch.set_ball_state(*ball);
        scratch.set_car_state(0, *state);
        let mut states: Vec<CarState> = vec![*state];
        for step_tick in 1..=horizon {
            let c = controls_at(shift, t_a + step_tick as i64);
            let mut controls = CarControls {
                throttle: c.0,
                steer: c.1,
                handbrake: c.2,
                boost: c.3,
                jump: c.4,
                ..CarControls::default()
            };
            if step_tick == press_u {
                controls.jump = true;
                controls.pitch = pitch;
                controls.yaw = yaw;
            } else if step_tick > press_u {
                controls.jump = false;
                let sign = scratch.get_car_state(0).flip_rel_torque.y.signum();
                controls.pitch = cancel * sign;
            }
            scratch.set_car_controls(0, controls);
            scratch.step_tick();
            states.push(*scratch.get_car_state(0));
        }
        let (frame_b, _, pos_b, vel_b, _) = first_fresh;
        let frame_tick = timeline(frame_b) - t_a;
        let gap = (timeline(frame_b) - timeline(frame_b.saturating_sub(1))).max(4);
        let (lo, hi) = ((frame_tick - gap).max(1), frame_tick.min(ticks_ac - 1));
        let mut best_tick: Option<(i64, f32)> = None;
        for tb in lo..=hi {
            let st = &states[tb as usize];
            let error = (st.phys.pos - pos_b).length() + 0.1 * (st.phys.vel - vel_b).length();
            if best_tick.is_none_or(|(_, e)| error < e - 1e-4) {
                best_tick = Some((tb, error));
            }
        }
        if let Some((tb, _)) = best_tick {
            ticks_ab_eff = tb;
            first_packet = Some((frame_b, (frame_tick - tb).max(0) as u64));
        }
    }
    // The jump schedule for the interval to the next packet, up to the dodge press if it falls in it.
    let mut entries_out: Vec<(u64, f32, f32, bool, bool, Option<bool>)> = Vec::new();
    let mut previous = None;
    let until = if press <= ticks_ab_eff {
        press - 1
    } else {
        ticks_ab_eff
    };
    for step in 1..=until {
        let c = controls_at(shift, t_a + step);
        if previous != Some(c) {
            entries_out.push((now_tick + step as u64, c.0, c.1, c.2, c.3, Some(c.4)));
            previous = Some(c);
        }
    }
    Some(FlipFit {
        schedule: GroundSchedule {
            slot,
            end_tick: now_tick + ticks_ab_eff as u64,
            entries: entries_out,
            shift: None,
        },
        first_packet,
        dodge: (press <= ticks_ab_eff || options.defer_dodge_past_next_packet).then_some(
            DodgePlan {
                activation_frame,
                start_offset: press as u64,
                duration: if press <= ticks_ab_eff {
                    ticks_ab_eff as u64
                } else {
                    ticks_ac as u64
                },
                pitch,
                yaw,
                cancel,
                first_packet: None,
            },
        ),
    })
}

fn step_ticks(
    arena: &mut Arena,
    ticks: u64,
    apply_hit_impulse: bool,
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
                // the direction of the same controls: leave those ticks to the press itself.
                if controls.jump && !arena.get_car_state(schedule.slot).is_on_ground {
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
        let tick_events = step_tick_with_hit_impulse(arena, apply_hit_impulse);
        events.extend(
            tick_events
                .into_iter()
                .map(|event| SimEvent { arena_tick, event }),
        );
    }
}

/// Fits the flip's pitch-cancel amount from this fresh car packet. Candidate cancels (opposite pitch
/// input of 0, 0.25, ..., 1) are simulated in a scratch arena from the current corrected state
/// through the next `flip_cancel_packets` fresh packets (the state reset to each, as the converter
/// does), and the one whose summed angular-velocity error is smallest wins; it is used for the
/// interval to the next packet. With `flip_cancel_holdout` that first interval is left out of the
/// sum. Fitting the next packet alone (the default, one packet) is in sample there, and once the
/// flip's speed saturates at 5.5 rad/s the candidates barely differ and the choice can alternate
/// between packets. Uses later packets, so it is offline reconstruction; spans containing a withheld
/// frame, an inactive frame, or a change of dodge counter are refused.
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
        return refused_ground(11);
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
        return refused_ground(12);
    }
    // Causal choices use the previous interval only (previous fresh packet of the same flip to this one).
    if matches!(
        options.flip_cancel_source,
        FlipCancelSource::PreviousIntervalFit | FlipCancelSource::ExternalRulePrevious
    ) {
        let mut previous = None;
        for candidate in (index.saturating_sub(24)..index).rev() {
            if !active(candidate) || withheld(candidate) {
                return refused_ground(13);
            }
            let Some(other) = frames[candidate].cars.iter().find(|c| {
                c.actor_id == car.actor_id
                    && c.actor_created_frame == car.actor_created_frame
                    && c.player_key == car.player_key
            }) else {
                return refused_ground(14);
            };
            let b = &other.body;
            let (Some(pos), Some(vel), Some(rot), Some(ang)) = (
                b.position.as_ref().filter(|x| x.frame == candidate),
                b.linear_velocity.as_ref().filter(|x| x.frame == candidate),
                b.rotation_xyzw.as_ref().filter(|x| x.frame == candidate),
                b.angular_velocity_replay_units
                    .as_ref()
                    .filter(|x| x.frame == candidate),
            ) else {
                continue;
            };
            if other.inputs.dodge_active_raw.as_ref().map(|d| d.value) != Some(counter) {
                return refused_ground(15);
            }
            let lag = match packet_lags {
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
            let ticks = (timeline(index) - lag_a as i64) - (timeline(candidate) - lag);
            if !(1..=40).contains(&ticks) {
                return refused_ground(16);
            }
            let quat = quaternion(rot.value)?;
            previous = Some((
                ticks,
                vec3(pos.value),
                vec3(vel.value),
                Mat3A::from_quat(quat),
                vec3(ang.value) * 0.01,
            ));
            break;
        }
        let (ticks, pos, vel, rot, ang) = previous?;
        let now_ang = vec3(ang0.value) * 0.01;
        if options.flip_cancel_source == FlipCancelSource::ExternalRulePrevious {
            // external/RLCarInputSolver AirSolver.cpp: local pitch angular speed fell by more than
            // 0.05 rad/s per tick (local y is the right axis).
            let from = ang.dot(rot.y_axis).abs();
            let to = now_ang.dot(state.phys.rot_mat.y_axis).abs();
            return Some(if from > to + 0.05 * ticks as f32 {
                1.0
            } else {
                0.0
            });
        }
        let sign = state.flip_rel_torque.y.signum();
        let mut start = *state;
        start.phys.pos = pos;
        start.phys.vel = vel;
        start.phys.rot_mat = rot;
        start.phys.ang_vel = ang;
        start.flip_time = (state.flip_time - ticks as f32 / 120.0).max(0.0);
        let mut best: Option<(f32, f32)> = None;
        for step in 0..=4 {
            let cancel = step as f32 * 0.25;
            scratch.set_car_state(0, start);
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
            let error = (end.phys.ang_vel - now_ang).length();
            if best.is_none_or(|(_, e)| error < e - 1e-4) {
                best = Some((cancel, error));
            }
        }
        return best.map(|(cancel, _)| cancel);
    }
    // The next fresh packets of the flip (up to `flip_cancel_packets`, within 80 ticks, while the dodge
    // counter is unchanged): angular velocity to score and the full physical state to reset to, as
    // the converter does at each packet.
    let max_packets = options.flip_cancel_packets.max(1);
    let mut targets: Vec<(i64, Vec3A, Vec3A, Mat3A, Vec3A)> = Vec::new();
    for candidate in index + 1..=(index + 24).min(frames.len() - 1) {
        let searching_more = !targets.is_empty();
        if !active(candidate) || withheld(candidate) {
            if searching_more {
                break;
            }
            return refused_ground(17);
        }
        let Some(other) = frames[candidate].cars.iter().find(|c| {
            c.actor_id == car.actor_id
                && c.actor_created_frame == car.actor_created_frame
                && c.player_key == car.player_key
        }) else {
            if searching_more {
                break;
            }
            return refused_ground(18);
        };
        let b = &other.body;
        let (Some(pos), Some(vel), Some(rot), Some(ang)) = (
            b.position.as_ref().filter(|x| x.frame == candidate),
            b.linear_velocity.as_ref().filter(|x| x.frame == candidate),
            b.rotation_xyzw.as_ref().filter(|x| x.frame == candidate),
            b.angular_velocity_replay_units
                .as_ref()
                .filter(|x| x.frame == candidate),
        ) else {
            continue;
        };
        if other.inputs.dodge_active_raw.as_ref().map(|d| d.value) != Some(counter) {
            if searching_more {
                break;
            }
            return refused_ground(19);
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
        let previous_ticks = targets.last().map_or(0, |t| t.0);
        if !(previous_ticks + 1..=80).contains(&ticks) {
            if searching_more {
                break;
            }
            return refused_ground(20);
        }
        let Some(quat) = quaternion(rot.value) else {
            continue;
        };
        targets.push((
            ticks,
            vec3(pos.value),
            vec3(vel.value),
            Mat3A::from_quat(quat),
            vec3(ang.value) * 0.01,
        ));
        if targets.len() == max_packets {
            break;
        }
    }
    if targets.is_empty() {
        return refused_ground(21);
    }
    if options.flip_cancel_source == FlipCancelSource::ExternalRuleNext {
        let (ticks, _, _, rot, ang) = targets[0];
        let from = (vec3(ang0.value) * 0.01)
            .dot(state.phys.rot_mat.y_axis)
            .abs();
        let to = ang.dot(rot.y_axis).abs();
        return Some(if from > to + 0.05 * ticks as f32 {
            1.0
        } else {
            0.0
        });
    }
    let sign = state.flip_rel_torque.y.signum();
    // One cancel for all the intervals: each candidate is simulated interval by interval from the
    // packet, the state reset to each later packet as the converter does, and the angular-velocity
    // errors are summed. With `flip_cancel_holdout` the first interval (the one the cancel is used
    // for) is left out of the sum when there are later ones, so its residual stays a check.
    let first_scored = usize::from(options.flip_cancel_holdout && targets.len() > 1);
    let mut best: Option<(f32, f32)> = None;
    for step in 0..=4 {
        let cancel = step as f32 * 0.25;
        let mut start = *state;
        let mut previous_ticks = 0;
        let mut total = 0.0f32;
        for (j, target) in targets.iter().enumerate() {
            scratch.set_car_state(0, start);
            let mut controls = *base_controls;
            controls.jump = false;
            controls.pitch = cancel * sign;
            scratch.set_car_controls(0, controls);
            for _ in 0..(target.0 - previous_ticks) {
                scratch.step_tick();
            }
            let mut end = *scratch.get_car_state(0);
            let speed = end.phys.ang_vel.length();
            let mut clamped = end.phys.ang_vel;
            if speed > 5.5 {
                clamped *= 5.5 / speed;
            }
            if j >= first_scored {
                total += (clamped - target.4).length();
            }
            end.phys.pos = target.1;
            end.phys.vel = target.2;
            end.phys.rot_mat = target.3;
            end.phys.ang_vel = target.4;
            start = end;
            previous_ticks = target.0;
        }
        if best.is_none_or(|(_, e)| total < e - 1e-4) {
            best = Some((cancel, total));
        }
    }
    best.map(|(cancel, _)| cancel)
}

/// A dodge start plan: press `jump` with the dodge direction `start_offset` ticks after the current
/// packet, holding `cancel` of the flip's pitch torque cancelled until `duration` ticks after it.
struct DodgePlan {
    activation_frame: usize,
    start_offset: u64,
    duration: u64,
    pitch: f32,
    yaw: f32,
    cancel: f32,
    /// The first fresh packet after the activation and the lag (ticks) inferred for it.
    first_packet: Option<(usize, u64)>,
}

/// Counters of the refusal reasons of `fit_dodge_start` (diagnostics, `diagnose_dodge_coverage`):
/// 0 inactive or withheld frame in the search window, 1 car missing, 2 no activation within 14 frames,
/// 3 a nearer fresh packet before the activation, 4 fewer than two usable fresh packets after it,
/// 5 no dodge direction, 6 fitted start after the next packet, 7 planned, 8 calls that got past the
/// entry checks (airborne fresh packet with an even dodge counter).
pub static DODGE_FIT_COUNTS: [std::sync::atomic::AtomicUsize; 9] =
    [const { std::sync::atomic::AtomicUsize::new(0) }; 9];

/// Refusal counters of `fit_ground_flip_timing`, one per `return` (diagnostics).
pub static GROUND_FLIP_COUNTS: [std::sync::atomic::AtomicUsize; 40] =
    [const { std::sync::atomic::AtomicUsize::new(0) }; 40];

fn refused_ground<T>(reason: usize) -> Option<T> {
    GROUND_FLIP_COUNTS[reason].fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    None
}

fn refused<T>(reason: usize) -> Option<T> {
    DODGE_FIT_COUNTS[reason].fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    None
}

/// Fits when a dodge physically started. Given a fresh airborne car packet at `index` and a dodge
/// counter that turns odd (with a fresh `DodgeTorque`) before the next fresh car packet, every start
/// tick up to the *second* next fresh packet (both with exact chain lags) is simulated in a scratch
/// arena and the one whose position and velocity best match that packet wins; the pitch cancel is
/// then chosen from its angular velocity. The plan drives only the interval to the *next* packet,
/// so that packet is not used by the fit and its residual stays a held-out check (fitting the next
/// packet itself was in-sample and made later angular velocity worse). Uses later packets (offline
/// reconstruction); spans with a withheld or inactive frame are refused. The ball is simulated too
/// (its state at this packet's time in the main arena), so dodges at the ball are fitted with their
/// contacts; other cars are not modelled.
#[allow(clippy::too_many_arguments)]
fn fit_dodge_start(
    observations: &ObservedReplay,
    options: &ConvertOptions,
    packet_lags: &Option<PacketLags>,
    first_time: f32,
    index: usize,
    car: &observations::Car,
    state: &CarState,
    base_controls: &CarControls,
    lag_a: u64,
    ball: &rocketsim::BallState,
    scratch: &mut Arena,
) -> Option<DodgePlan> {
    let frames = &observations.frames;
    let ang0 = car.body.angular_velocity_replay_units.as_ref()?;
    if ang0.frame != index || state.is_on_ground {
        return None;
    }
    // A car that has not dodged yet has no dodge counter (it is created with one at its first dodge).
    let counter = car.inputs.dodge_active_raw.as_ref().map_or(0, |d| d.value);
    if counter % 2 == 1 {
        return None;
    }
    DODGE_FIT_COUNTS[8].fetch_add(1, std::sync::atomic::Ordering::Relaxed);
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
    let same_car = |candidate: &&observations::Car| {
        candidate.actor_id == car.actor_id
            && candidate.actor_created_frame == car.actor_created_frame
            && candidate.player_key == car.player_key
    };
    let last = (index + 14).min(frames.len() - 1);
    // Activation: the first frame whose fresh dodge counter is odd with a fresh torque.
    let mut activation = None;
    for g in index + 1..=last {
        if !active(g) || withheld(g) {
            return refused(0);
        }
        let Some(other) = frames[g].cars.iter().find(same_car) else {
            return refused(1);
        };
        if let Some(dodge) = other
            .inputs
            .dodge_active_raw
            .as_ref()
            .filter(|d| d.frame == g)
        {
            if dodge.value % 2 == 1 {
                if let Some(torque) = activation_torque(frames, g, other) {
                    activation = Some((g, torque));
                    break;
                }
            }
        }
    }
    let Some((activation_frame, torque)) = activation else {
        return refused(2);
    };
    // Plan only from the last fresh packet before the activation: a nearer packet would reset the
    // state under a plan that ignores it.
    for g in index + 1..activation_frame {
        let other = frames[g].cars.iter().find(same_car)?;
        if other.body.position.as_ref().is_some_and(|p| p.frame == g) {
            return refused(3);
        }
    }
    // The next two fresh packets with exact chain lags at or after the activation frame. The first
    // is the next reset of the state and is held out; the fit uses the second.
    let lags = packet_lags.as_ref()?;
    let origin_tick = timeline(index) - lag_a as i64;
    let mut fresh: Vec<(u64, Vec3A, Vec3A, Vec3A)> = Vec::new();
    let mut fresh_frames: Vec<usize> = Vec::new();
    for g in activation_frame..=(activation_frame + 12).min(frames.len() - 1) {
        if !active(g) || withheld(g) {
            break;
        }
        let Some(other) = frames[g].cars.iter().find(same_car) else {
            break;
        };
        let b = &other.body;
        if let (Some(p), Some(v), Some(w)) = (
            b.position.as_ref().filter(|x| x.frame == g),
            b.linear_velocity.as_ref().filter(|x| x.frame == g),
            b.angular_velocity_replay_units
                .as_ref()
                .filter(|x| x.frame == g),
        ) {
            // The first fresh packet after the activation is the next reset of the state: it ends the
            // plan at the tick the converter injects it (its own lag, exact or not). The fit target
            // is the next one and needs an exact chain lag.
            let chain = lags
                .car_actor
                .get(&(car.actor_id, car.actor_created_frame, g))
                .copied();
            let lag = if fresh.is_empty() {
                chain
                    .or(lags.cars[g])
                    .map_or((timeline(g) - timeline(g - 1)).max(0) as f32 / 2.0, |lag| {
                        lag
                    })
            } else {
                let Some(lag) = chain else {
                    continue;
                };
                lag
            };
            let tick = (timeline(g) - lag.round().max(0.0) as i64) - origin_tick;
            if !(1..=45).contains(&tick) {
                break;
            }
            fresh.push((
                tick as u64,
                vec3(p.value),
                vec3(v.value),
                vec3(w.value) * 0.01,
            ));
            fresh_frames.push(g);
            if fresh.len() == 2 {
                break;
            }
        }
    }
    let [(tick_b, ..), second] = fresh[..] else {
        return refused(4);
    };
    let first_frame = fresh_frames[0];
    let targets = vec![second];
    let first_fresh_state = (fresh[0].1, fresh[0].2);
    let horizon = second.0;
    let [tx, ty, _] = torque;
    let (pitch, yaw) = ((-ty / 2.24).clamp(-1.0, 1.0), (-tx / 2.60).clamp(-1.0, 1.0));
    if (pitch * pitch + yaw * yaw).sqrt() <= 0.01 {
        return refused(5);
    }
    let mut start = *state;
    start.has_jumped = true;
    start.is_jumping = false;
    start.air_time_since_jump = start.air_time_since_jump.max(0.05);
    let mut base = *base_controls;
    base.jump = false;
    // The path with no dodge, saved tick by tick, so each candidate resumes from its start tick.
    let mut path = vec![start];
    let mut path_ball = vec![*ball];
    scratch.set_ball_state(*ball);
    scratch.set_car_state(0, start);
    scratch.set_car_controls(0, base);
    for _ in 0..horizon {
        scratch.step_tick();
        path.push(*scratch.get_car_state(0));
        path_ball.push(*scratch.get_ball_state());
    }
    // States at every tick from `dodge_tick` to the horizon for a dodge at `dodge_tick` with `cancel`.
    let run_all = |scratch: &mut Arena, dodge_tick: u64, cancel: f32| -> Vec<CarState> {
        scratch.set_ball_state(path_ball[dodge_tick as usize - 1]);
        scratch.set_car_state(0, path[dodge_tick as usize - 1]);
        let mut after: Vec<CarState> = Vec::new();
        for tick in dodge_tick..=horizon {
            let mut controls = base;
            if tick == dodge_tick {
                controls.jump = true;
                controls.pitch = pitch;
                controls.yaw = yaw;
                controls.roll = 0.0;
            } else {
                let sign = scratch.get_car_state(0).flip_rel_torque.y.signum();
                controls.pitch = cancel * sign;
            }
            scratch.set_car_controls(0, controls);
            scratch.step_tick();
            let mut end = *scratch.get_car_state(0);
            let speed = end.phys.ang_vel.length();
            if speed > 5.5 {
                end.phys.ang_vel *= 5.5 / speed;
            }
            after.push(end);
        }
        after
    };
    // States at every target tick for a dodge at `dodge_tick` with `cancel`.
    let run = |scratch: &mut Arena, dodge_tick: u64, cancel: f32| -> Vec<CarState> {
        let after = run_all(scratch, dodge_tick, cancel);
        targets
            .iter()
            .map(|&(tick, ..)| {
                if tick < dodge_tick {
                    path[tick as usize]
                } else {
                    after[(tick - dodge_tick) as usize]
                }
            })
            .collect()
    };
    let position_velocity_error = |states: &[CarState]| -> f32 {
        targets
            .iter()
            .zip(states)
            .map(|(target, end)| {
                (end.phys.pos - target.1).length() + 0.1 * (end.phys.vel - target.2).length()
            })
            .sum()
    };
    let mut best: Option<(u64, f32)> = None;
    for dodge_tick in 1..=horizon {
        let error = position_velocity_error(&run(scratch, dodge_tick, 0.0));
        if best.is_none_or(|(_, e)| error < e - 1e-4) {
            best = Some((dodge_tick, error));
        }
    }
    let (dodge_tick, _) = best?;
    // A start after the next packet is not driven here (that packet resets the state); the normal
    // trigger at the activation frame applies instead.
    let mut deferred = dodge_tick > tick_b;
    if deferred && !options.defer_dodge_past_next_packet && !options.infer_dodge_first_packet_tick {
        return refused(6);
    }
    if deferred {
        DODGE_FIT_COUNTS[6].fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }
    let mut best_cancel: Option<(f32, f32)> = None;
    for step in 0..=4 {
        let cancel = step as f32 * 0.25;
        let states = run(scratch, dodge_tick, cancel);
        // Angular velocity only counts at packets at or after the start.
        let error: f32 = targets
            .iter()
            .zip(&states)
            .filter(|(target, _)| target.0 >= dodge_tick)
            .map(|(target, end)| (end.phys.ang_vel - target.3).length())
            .sum();
        if best_cancel.is_none_or(|(_, e)| error < e - 1e-4) {
            best_cancel = Some((cancel, error));
        }
    }
    // The first fresh packet after the activation has no chain lag (a dodge breaks the motion the chain
    // inference relies on), so its tick is only known to lie within the frame gap (0-4 ticks at 30 fps)
    // before its frame time. With the start and cancel fitted on the exact second packet, the tick in that
    // range at which the simulated path reproduces this packet (position and velocity) is its tick.
    let mut first_packet = None;
    let final_cancel = best_cancel?.0;
    if options.infer_dodge_first_packet_tick {
        let frame_tick = timeline(first_frame) - origin_tick;
        // A packet was generated within its frame window: the lag is at most the frame gap.
        let gap = (timeline(first_frame) - timeline(first_frame.saturating_sub(1))).max(4);
        let (lo, hi) = (
            (frame_tick - gap).max(1),
            frame_tick.min(second.0 as i64 - 1),
        );
        if lo <= hi {
            let after = run_all(scratch, dodge_tick, final_cancel);
            let target = &first_fresh_state;
            let mut best_tick: Option<(u64, f32)> = None;
            for tb in lo as u64..=hi as u64 {
                let st = if tb < dodge_tick {
                    &path[tb as usize]
                } else {
                    &after[(tb - dodge_tick) as usize]
                };
                let error =
                    (st.phys.pos - target.0).length() + 0.1 * (st.phys.vel - target.1).length();
                if best_tick.is_none_or(|(_, e)| error < e - 1e-4) {
                    best_tick = Some((tb, error));
                }
            }
            if let Some((tb, _)) = best_tick {
                let lag = frame_tick - tb as i64;
                first_packet = Some((first_frame, lag.max(0) as u64, tb));
                deferred = dodge_tick > tb;
            }
        }
    }
    if deferred && !options.defer_dodge_past_next_packet {
        return refused(6);
    }
    DODGE_FIT_COUNTS[7].fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    Some(DodgePlan {
        activation_frame,
        start_offset: dodge_tick,
        duration: match first_packet {
            Some((_, _, tb)) if !deferred => tb,
            _ if deferred => second.0,
            _ => tick_b,
        },
        pitch,
        yaw,
        cancel: final_cancel,
        first_packet: first_packet.map(|(frame, lag, _)| (frame, lag)),
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
    if options.apply_observed_demolitions && options.disable_simulated_demolitions {
        // The replay reports every demolition; RocketSim's own bump detection reproduced 83% of
        // them and invented as many (114 of 254 on the train split, 27 of 57 replays with no
        // demolition in the replay at all).
        config.mutators.demo_mode = DemoMode::Disabled;
    }
    let mut arena = Arena::new_with_config(config);
    let mut slots: HashMap<String, usize> = HashMap::new();
    let mut car_slots = Vec::new();
    let mut actor_slots: HashMap<i32, (usize, usize)> = HashMap::new();
    let mut gated_jump_active: HashMap<(i32, usize), bool> = HashMap::new();
    let mut last_dodge_raw: HashMap<(i32, usize), u8> = HashMap::new();
    let mut last_double_raw: HashMap<(i32, usize), u8> = HashMap::new();
    // Action counters (jump, double jump, dodge) of each car at its last frame, and at its last frame on
    // the ground.
    let mut last_counters: HashMap<(i32, usize), [u8; 3]> = HashMap::new();
    let mut ground_counters: HashMap<(i32, usize), [u8; 3]> = HashMap::new();
    let mut pad_actor_to_index: HashMap<i32, usize> = HashMap::new();
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
        .contacts_from_ball_packets
        && options.infer_packet_lag
        && !options.zero_packet_lag
    {
        packet_lags
            .as_ref()
            .and_then(|lags| crate::ball_evidence::ball_intervals(observations, lags, options).ok())
            .map(|v| {
                v.into_iter()
                    .filter(|i| i.velocity_residual > crate::ball_evidence::CONTACT_VELOCITY_THRESHOLD)
                    .map(|i| (i.frame_b, i))
                    .collect()
            })
            .unwrap_or_default()
    } else {
        HashMap::new()
    };
    let mut recent_poses: std::collections::VecDeque<(u64, Vec<(usize, Vec3A, Mat3A, bool)>)> =
        std::collections::VecDeque::new();
    let mut recent_touch_ticks: std::collections::VecDeque<u64> = std::collections::VecDeque::new();
    // Last arena tick of a car-ball contact event per slot (to find where a contact starts).
    let mut last_contact_tick: HashMap<usize, u64> = HashMap::new();
    // Timeline tick until which an observed demolition keeps a slot demolished.
    let mut demo_hold_until: HashMap<usize, u64> = HashMap::new();
    // Informative shifts chosen by the ground timing fit, per car actor lifetime.
    let mut car_shifts: HashMap<(i32, usize), Vec<i64>> = HashMap::new();
    let mut air_schedules: Vec<AirSchedule> = Vec::new();
    let mut ground_scratch: HashMap<&'static str, Arena> = HashMap::new();
    let mut slot_bodies: HashMap<usize, (&'static str, CarBodyConfig)> = HashMap::new();
    let mut handled_dodges: HashSet<(i32, usize, usize)> = HashSet::new();
    // Lags (ticks) fitted for the first car packet after a dodge activation, by (actor, lifetime, frame).
    let lag_overrides: std::cell::RefCell<HashMap<(i32, usize, usize), u64>> =
        std::cell::RefCell::new(HashMap::new());
    let mut flip_scratch = (options.infer_flip_cancel || options.infer_dodge_start).then(|| {
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
        let mut fitted_arena: Vec<(usize, &'static str, u64, f32, f32, f32, usize)> = Vec::new();
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
        // middle of the car's packet interval (`packet_interval_control_rule`) or else `gap / 2 - 2`
        // ticks into the interval that ends at its state (from the start if shorter). The next
        // frame's controls also act inside this interval when the middle of its car's packet
        // interval falls before this frame's time (a change takes about as long to be seen as a
        // frame lasts).
        let span = remaining;
        let mut switches: Vec<(u64, &observations::Car)> = Vec::new();
        if options.lookahead_ground_controls
            && packet_lags.is_some()
            && remaining > 0
            && !frame_withheld
        {
            let interval_start = timeline_tick as i64 - gap as i64;
            for car in frame_cars.iter().copied() {
                let median_shift = options
                    .per_car_control_shift
                    .then(|| car_shifts.get(&(car.actor_id, car.actor_created_frame)))
                    .flatten()
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
                    for g in frame_idx.saturating_sub(4)..=(frame_idx + 4).min(observations.frames.len() - 1) {
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
                        let tick = ((f64::from(observations.frames[g].time) - f64::from(first_time))
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
                let switch = packet_interval_change_tick(
                    observations,
                    options,
                    packet_lags.as_ref(),
                    first_time,
                    car,
                    frame_idx,
                )
                .map_or((gap / 2).saturating_sub(2), |tick| {
                    (tick - interval_start).max(0) as u64
                });
                switches.push((switch.min(span), car));
            }
            if options.packet_interval_control_rule && frame_idx + 1 < observations.frames.len() {
                let next_frame = &observations.frames[frame_idx + 1];
                let next_active = next_frame
                    .game_state
                    .as_ref()
                    .is_some_and(|state| state.value == "Active");
                let next_withheld = options
                    .withheld_frames
                    .as_ref()
                    .is_some_and(|w| w.get(frame_idx + 1).copied().unwrap_or(false));
                if next_active && !next_withheld {
                    for car in frame_cars.iter().copied() {
                        let Some(next_car) = next_frame.cars.iter().find(|c| {
                            c.actor_id == car.actor_id
                                && c.actor_created_frame == car.actor_created_frame
                        }) else {
                            continue;
                        };
                        let Some(tick) = packet_interval_change_tick(
                            observations,
                            options,
                            packet_lags.as_ref(),
                            first_time,
                            next_car,
                            frame_idx + 1,
                        ) else {
                            continue;
                        };
                        let switch = tick - interval_start;
                        if switch >= 0 && (switch as u64) < span {
                            switches.push((switch as u64, next_car));
                        }
                    }
                }
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
                            options.apply_hit_extra_impulse,
                            &mut pending_dodges,
                            &mut ground_schedules,
                            &mut air_schedules,
                            &mut events,
                        );
                        if options.limit_reported_velocities {
                            limit_reported_velocities(&mut arena, slots.len());
                        }
                        remaining -= switch - elapsed;
                    }
                    let Some(&(slot, created)) = actor_slots.get(&car.actor_id) else {
                        continue;
                    };
                    if created != car.actor_created_frame
                        || !arena.get_car_state(slot).is_on_ground
                    {
                        continue;
                    }
                    let next = controls_from_observation(car, options);
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
                        options.apply_hit_extra_impulse,
                        &mut pending_dodges,
                        &mut ground_schedules,
                        &mut air_schedules,
                        &mut events,
                    );
                    if options.limit_reported_velocities {
                        limit_reported_velocities(&mut arena, slots.len());
                    }
                    remaining -= target - elapsed;
                }
            }};
        }
        for lag in phase_lags {
            // Advance to this group's packet time (`lag` ticks before the frame time).
            advance_to!(span - lag.min(remaining));
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
                    // A demolished car is not simulated: comparing it with a packet means nothing.
                    if let Some(residual) = (!car_state.is_demoed).then(|| position_residual(
                        frame.index,
                        Some(car.actor_id),
                        &car.body,
                        previous,
                        &car_state.phys,
                        Some(car_state.is_on_ground),
                        &observations.frames,
                    )).flatten() {
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
                if car.player_link_active
                    && state.is_demoed
                    && demo_hold_until.get(&slot).is_none_or(|&until| timeline_tick >= until)
                {
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
                if options.boost_pickup_lookahead && !new_lifetime {
                    let next_boost = observations
                        .frames
                        .get(frame_idx + 1)
                        .and_then(|next| {
                            next.cars.iter().find(|c| {
                                c.actor_id == car.actor_id
                                    && c.actor_created_frame == car.actor_created_frame
                            })
                        })
                        .and_then(|c| c.boost.as_ref())
                        .filter(|b| b.frame == frame_idx + 1);
                    // A pickup adds at least 12; consumption only lowers the amount.
                    if let Some(next) = next_boost {
                        if next.value > state.boost + 6.0 {
                            state.boost = next.value;
                            dirty = true;
                        }
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
                        {
                            if let Some(torque) = torque_now {
                                let [tx, ty, _] = torque;
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

                if options.infer_double_jump {
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
                            if !options.gate_dodge_on_observed_impulse
                                || dodge_impulse_unobserved(car, frame.index, &state)
                            {
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
                }

                if options.flags_from_counters {
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
                    } else if state.is_on_ground {
                        if let Some(previous) = last_counters.get(&key) {
                            ground_counters.insert(key, *previous);
                        }
                    }
                    last_counters.insert(key, current);
                    if !state.is_on_ground {
                        if let Some(ground) = ground_counters.get(&key) {
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
                            let acting = dodge_jump_control
                                || pending_dodges.iter().any(|dodge| dodge.slot == slot);
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
                    // The dodge direction is (-pitch, yaw + roll): a roll left over from the air
                    // controls would turn a double jump into a flip or rotate a dodge.
                    controls.roll = 0.0;
                }
                if options.infer_dodge_start
                    && airborne
                    && !dodge_jump_control
                    && !state.is_flipping
                    && active
                    && !new_lifetime
                    && !pending_dodges.iter().any(|dodge| dodge.slot == slot)
                {
                    if let Some(scratch) = flip_scratch.as_mut() {
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
                }
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
                        if current.is_flipping
                            || pending_dodges
                                .iter()
                                .any(|d| d.slot == slot && d.start_tick > arena.tick_count())
                        {
                            flip_scratch.as_mut()
                        } else {
                            None
                        },
                        &pending_dodges,
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
                        // ticks is in `cancel`).
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
                if (options.fit_ground_control_timing || options.fit_jump_timing)
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
                    let scratch = ground_scratch.entry(name).or_insert_with(|| {
                        let mut scratch_config = ArenaConfig::new(GameMode::Soccar);
                        scratch_config.rng_seed = Some(options.seed);
                        let mut scratch = Arena::new_with_config(scratch_config);
                        scratch.add_car(Team::Blue, config);
                        scratch
                    });
                    let current = *arena.get_car_state(slot);
                    let mut schedule = options
                        .fit_ground_control_timing
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
                    if schedule.is_none() && options.fit_jump_timing {
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
                        && options.fit_jump_timing
                        && options.infer_dodge_start
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
                        // A press is a rising edge: the button is already down when the car's current
                        // controls (the previous interval's last) have the jump set.
                        let mut jumping = arena.get_car_controls(slot).jump;
                        for entry in &schedule.entries {
                            if let Some(jump) = entry.5 {
                                if jump && !jumping {
                                    fitted_arena.push((slot, "jump", entry.0, 0.0, 0.0, 0.0, 0));
                                }
                                jumping = jump;
                            }
                        }
                        ground_schedules.push(schedule);
                    }
                }
            }
        }
        if options.block_sim_pad_pickups {
            for idx in 0..arena.num_boost_pads() {
                arena.set_boost_pad_state(idx, BoostPadState { cooldown: 20.0 });
            }
        }
        advance_to!(span);
        let _ = remaining;
        if options.apply_observed_demolitions && simulated && !frame_withheld {
            for event in &frame.events {
                let observations::Event::Demolish {
                    source,
                    victim_car: Some(victim),
                    ..
                } = event
                else {
                    continue;
                };
                if *source == "goal_explosion" {
                    continue;
                }
                let Some(&(slot, _)) = actor_slots.get(victim) else {
                    continue;
                };
                let mut state = *arena.get_car_state(slot);
                if !state.is_demoed {
                    state.is_demoed = true;
                    state.demo_respawn_timer = 3.0;
                    arena.set_car_state(slot, state);
                }
                demo_hold_until.insert(slot, timeline_tick + 360);
            }
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
            recent_poses.push_back((
                timeline_tick,
                state_now
                    .cars
                    .iter()
                    .enumerate()
                    .map(|(i, c)| (i, c.1.phys.pos, c.1.phys.rot_mat, c.1.is_demoed))
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
                let (tick_from, tick_to) = (interval.tick_a.max(0) as u64, interval.tick_b.max(0) as u64);
                let pose_at = |slot: usize, tick: u64| -> Option<(Vec3A, Mat3A, bool)> {
                    let at = |k: usize| {
                        recent_poses
                            .get(k)
                            .and_then(|(t, cars)| cars.iter().find(|c| c.0 == slot).map(|c| (*t, c.1, c.2, c.3)))
                    };
                    let n = recent_poses.len();
                    let after = (0..n).find(|&k| recent_poses[k].0 >= tick)?;
                    let (t1, p1, r1, d1) = at(after)?;
                    if after == 0 || t1 == tick {
                        return Some((p1, r1, d1));
                    }
                    let (t0, p0, _, _) = at(after - 1)?;
                    let f = (tick - t0) as f32 / (t1 - t0).max(1) as f32;
                    Some((p0 + (p1 - p0) * f, r1, d1))
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
        let converted = ConvertedFrame {
            replay_frame: frame.index,
            replay_time: frame.time,
            timeline_tick,
            state: arena.get_arena_state(),
            simulated_events: events,
            touches,
            ball_contacts,
            packet_lags: applied_lags,
            fitted_inputs: fitted_arena
                .into_iter()
                .map(
                    |(slot, kind, tick, pitch, yaw, cancel, activation_frame)| FittedInput {
                        slot,
                        activation_frame,
                        kind,
                        tick: (tick as i64 + timeline_offset).max(0) as u64,
                        pitch,
                        yaw,
                        cancel,
                    },
                )
                .collect(),
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
    fn ball_hit_interval_recovers_the_ticks_between_packets() {
        // A ball in free flight, hit at tick 7 (its velocity changes by 1500 UU/s), seen at ticks 3
        // and 14: the two exact paths meet at the hit, so the interval between the packets is 11.
        let mut pos = [100.0f32, -300.0, 600.0];
        let mut vel = [1200.0f32, 200.0, 300.0];
        let mut packets = Vec::new();
        for tick in 0..=14 {
            if tick == 3 || tick == 14 {
                packets.push(ChainPacket { frame: tick, pos, vel });
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
        options.lookahead_ground_controls = false;
        let coasting = speed_at_last_frame(&options);
        options.lookahead_ground_controls = true;
        let with_lookahead = speed_at_last_frame(&options);
        assert!(
            with_lookahead > coasting + 5.0,
            "the throttle first seen at frame 2 should act before it: {coasting} vs {with_lookahead}"
        );
        // A frame withheld by an evaluator is never used to drive the interval before it.
        options.withheld_frames = Some(Arc::new(vec![false, false, true]));
        assert_eq!(speed_at_last_frame(&options), coasting);
        // Without inferred packet lags every state sits at its frame time and the rule is off.
        options.withheld_frames = None;
        options.infer_packet_lag = false;
        let unlagged = speed_at_last_frame(&options);
        options.lookahead_ground_controls = false;
        assert_eq!(unlagged, speed_at_last_frame(&options));
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
            false,
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
        let mut options = ConvertOptions::default();
        options.infer_jump_from_active = true;
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
                    false,
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
        let mut options = ConvertOptions::default();
        options.infer_jump_from_active = true;
        options.infer_dodge_from_active = true;
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
    fn reported_hit_impulse_is_part_of_the_ball_speed_without_a_workaround() {
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
