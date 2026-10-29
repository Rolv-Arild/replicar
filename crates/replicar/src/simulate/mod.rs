//! Simulate: the match in RocketSim. Each update is applied at its update tick, RocketSim steps the ticks in
//! between, and the cars drive on the controls the network values and the counters say (docs/v2-plan.md,
//! section 4.2). The simulator contains no fitting.

mod car;
mod pads;
mod players;
mod reports;
mod updates;

use std::collections::{BTreeMap, BTreeSet, HashMap};

use replicar_format::{AirControlSource, FrameIndex, GroundControlSource, PlayerIndex};
use rocketsim::{Arena, ArenaConfig, ArenaEvent, ArenaState, CarControls, DemoMode, GameMode};

pub use players::SimPlayer;
pub(crate) use updates::{dodge_torque, network_controls};

use crate::decode::{ActorId, CarLife, GameState, NetworkCar, NetworkFrame, NetworkReplay};
use crate::infer::{AirSchedule, GroundSchedule, Inference};
use crate::update_ticks::UpdateTicks;
use crate::{Error, Meshes};
use car::CarTrack;
use pads::Pads;
use players::Players;

/// Gaps between frames in play longer than this (10 s) are not simulated.
const MAX_GAP_TICKS: u64 = 1200;
/// A car whose fitted control shift (`Inference::control_shift`) is at least this many ticks in most of its life's
/// frames so far shows its controls ahead of their effect (a remote player whose inputs the server buffers): in
/// the air its boost, throttle and handbrake follow the shifted switches too.
const LEAD_MIN_SHIFT: i64 = 6;

/// How long after the counter's press a leading car's double jump waits for an update to show it: its control
/// shift. The double-jump counter leads by about half as much as the jump counter (whose lead is the shift), so an
/// update usually shows the rise first and places the press (`plan_early_presses`); the wait is the latest it comes.
fn lead_double_jump_delay(shift: i64) -> u64 {
    shift.max(0) as u64
}

/// How the simulation runs.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct SimulationOptions {
    /// RocketSim's random seed.
    pub seed: u64,
    /// Give each player the hitbox of its car body, else Octane.
    pub loadout_hitboxes: bool,
    /// Let the simulated cars pick up boost pads. Off: the boost comes only from the replay, and the pads'
    /// cooldowns from its pad records.
    pub simulated_pad_pickups: bool,
    /// Let RocketSim demolish cars by its own rule. Off: the replay's demolitions are the only ones.
    pub simulated_demolitions: bool,
    /// Frames whose updates a masked evaluation hides; their demolitions and wreck holds are not applied.
    #[serde(skip)]
    pub withheld: Option<Vec<bool>>,
    /// Time an airborne car's boost switches by the velocity of its next update (`plan_air_boost`; offline: it
    /// reads the update the interval ends at).
    #[serde(default = "enabled")]
    pub air_boost_timing: bool,
}

fn enabled() -> bool {
    true
}

impl Default for SimulationOptions {
    fn default() -> Self {
        Self {
            seed: 0,
            loadout_hitboxes: true,
            simulated_pad_pickups: false,
            simulated_demolitions: false,
            withheld: None,
            air_boost_timing: true,
        }
    }
}

/// Why a car is held demolished as a wreck.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldSource {
    /// A demolition report (a goal explosion, or a demolition of a car without an active player link).
    Observed,
    /// A sleeping update of a car whose player link is inactive.
    Inferred,
}

/// Where an applied update tick came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TickSource {
    /// The body's own chain of updates.
    Chain,
    /// The median of the frame's chained cars.
    FrameMedian,
    /// Half the frame's window: the median of an unknown tick.
    Default,
    /// The dodge-start fit placed the first update after a dodge.
    DodgeFit,
}

/// The update tick an update was applied at: `ticks` before its frame's own tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AppliedTick {
    /// `None` for the ball.
    pub car: Option<ActorId>,
    pub ticks: u64,
    pub source: TickSource,
}

/// One of RocketSim's events, with its sim tick.
#[derive(Debug, Clone, Copy)]
pub struct SimEvent {
    pub sim_tick: u64,
    pub event: ArenaEvent,
}

/// An input the inference chose for a car, on the replay timeline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FittedInput {
    pub player: PlayerIndex,
    /// The replay tick the input takes effect.
    pub replay_tick: u64,
    pub kind: FittedKind,
}

/// What kind of input was chosen.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FittedKind {
    /// Air controls solved tick by tick for `span_ticks` ticks.
    Air { span_ticks: u64 },
    /// A jump press.
    Jump,
    /// A dodge press with its direction and pitch cancel; `activation_frame` is the frame whose dodge counter
    /// turned odd.
    Dodge {
        pitch: f32,
        yaw: f32,
        cancel: f32,
        activation_frame: usize,
    },
}

/// A dodge to press: jump with the direction at `start_tick`, then the pitch cancel until `end_tick`; `base`
/// carries the air controls before the press (throttle, steer, boost and handbrake follow the car's switches).
#[derive(Debug, Clone, Copy)]
struct PendingDodge {
    player: PlayerIndex,
    /// The car life the dodge was planned for.
    life: CarLife,
    start_tick: u64,
    end_tick: u64,
    pitch: f32,
    yaw: f32,
    cancel: f32,
    base: rocketsim::CarControls,
}

/// What an early press is: a jump from the ground, a double jump, or a dodge with its direction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum EarlyKind {
    Jump,
    DoubleJump,
    Dodge { pitch: f32, yaw: f32 },
}

/// A press the frame's update already shows (its impulse is in the update), planned at the start of the
/// interval at a tick before that update, so that RocketSim applies it and the controls show it. `tick` is the
/// sim tick of the step the press is applied in; `update_tick` the sim tick the car's update lands at.
#[derive(Debug, Clone, Copy)]
pub(super) struct EarlyPress {
    pub(super) player: PlayerIndex,
    pub(super) life: CarLife,
    pub(super) kind: EarlyKind,
    tick: u64,
    update_tick: u64,
    /// RocketSim got the press (the car was in the state the press needs at its tick).
    pub(super) applied: bool,
}

/// A dodge the replay's counter shows while the simulated car is still on the ground (its takeoff not yet in the
/// updates, which can lag the counters by frames), a dodge or double jump pressed while a leading car's jump is
/// held for its release (`defer_held_press`), or a leading car's double jump its update does not show yet
/// (`lead_double_jump_delay`, from `from`): pressed at the first tick from `from` the car is airborne with the jump
/// released, unless the counter turns even or `until` passes first.
#[derive(Debug, Clone, Copy)]
pub(super) struct DeferredDodge {
    pub(super) player: PlayerIndex,
    pub(super) life: CarLife,
    pub(super) pitch: f32,
    pub(super) yaw: f32,
    /// Not pressed before this tick: nonzero only for a leading car's double jump (0: no wait).
    pub(super) from: u64,
    pub(super) until: u64,
}

/// A pickup the replay reports in a frame (new, with an instigator), as the simulation matched it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PickupMatch {
    pub pad: ActorId,
    /// RocketSim's index of the pad, when the record could be matched to one.
    pub pad_index: Option<usize>,
    /// The player of the instigator car.
    pub player: Option<PlayerIndex>,
}

/// The simulated state of a body just before an update corrected it: what the simulation predicted.
#[derive(Debug, Clone, Copy)]
pub struct Prediction {
    /// The car whose update it was; `None` for the ball.
    pub car: Option<ActorId>,
    pub phys: rocketsim::PhysState,
    /// Whether the simulated car was on the ground.
    pub is_on_ground: Option<bool>,
}

/// One stepped tick: the state at `sim_tick` (after the updates applied at it) with each car's `controls` set to the
/// controls the step after it applied, and what set them.
#[derive(Debug, Clone)]
pub struct TickRecord {
    pub sim_tick: u64,
    pub ball: rocketsim::BallState,
    /// Per player slot. A car held on its spawn pose is shown as not demolished.
    pub cars: Vec<rocketsim::CarState>,
    pub control_sources: Vec<ControlSources>,
}

/// The simulation at one replay frame.
#[derive(Debug, Clone)]
pub struct SimulatedFrame {
    pub index: FrameIndex,
    /// The replay tick (`replay_tick`), as an integer count.
    pub replay_tick: u64,
    /// The state at the frame time. A car held on its spawn pose is shown as not demolished.
    pub state: ArenaState,
    /// RocketSim's events of the interval that ends at this frame.
    pub events: Vec<SimEvent>,
    /// The ticks the frame's updates were applied at (simulated frames only).
    pub applied_ticks: Vec<AppliedTick>,
    /// Bodies (`None`: the ball) whose velocities a sleeping update zeroed.
    pub sleeping_velocity_zeroed: Vec<Option<ActorId>>,
    /// Cars found to be wrecks in this frame by inference.
    pub wrecks_inferred: Vec<ActorId>,
    /// Players held as wrecks in this frame, with the reason.
    pub wrecks_held: Vec<(PlayerIndex, HoldSource)>,
    /// Players whose car is shown on its spawn pose in this frame.
    pub spawning: Vec<PlayerIndex>,
    /// Every car actor of the frame with a player, and that player.
    pub car_players: Vec<(ActorId, PlayerIndex)>,
    /// The ball got an update in this frame.
    pub ball_updated: bool,
    /// Players whose current car got an update in this frame.
    pub updated_players: Vec<PlayerIndex>,
    /// Inputs the inference chose at this frame's updates.
    pub fitted: Vec<FittedInput>,
    /// The car life of each player's car in this frame (the frame's cars that resolved to a player, later
    /// ones winning), as the creation frame of the car actor.
    pub lives: Vec<(PlayerIndex, FrameIndex)>,
    /// The new pickups the replay reports in this frame.
    pub pickups: Vec<PickupMatch>,
    /// The simulated state of each body an update corrected in this frame, just before the correction (the
    /// ball once initialized; a car of a continuing life that is not demolished; simulated frames only).
    pub predictions: Vec<Prediction>,
    /// Per player slot: what set the controls the frame's state has.
    pub control_sources: Vec<ControlSources>,
    /// The ticks stepped in the frame's interval, in order. The first is the previous frame's own tick (the controls
    /// applied after a frame are known only once the next interval starts); this frame's own tick is the first of
    /// the next frame's. Empty when the frame was not simulated.
    pub ticks: Vec<TickRecord>,
}

/// What the simulation counted.
#[derive(Debug, Clone, Default, PartialEq, Eq, serde::Serialize)]
pub struct SimDiagnostics {
    pub skipped_replay_ticks: u64,
    pub player_loadout_changes: usize,
    pub unlinked_car_frames: usize,
    pub default_hitbox_players: usize,
    pub active_car_demolition_corrections: usize,
    pub flip_resets_observed: usize,
    pub flip_resets_applied: usize,
    pub flip_resets_inferred: usize,
    pub kickoffs_from_release: usize,
    pub air_boost_switches: usize,
    pub air_throttle_switches: usize,
    pub jump_releases_timed: usize,
    pub air_presses_timed: usize,
    pub sleeping_car_updates: usize,
    pub cars_started_from_spawn_pose: usize,
    pub goal_explosion_demolitions: usize,
    pub wrecks_inferred: usize,
    pub wrecks_after_demolition: usize,
    pub wrecks_released: usize,
    pub sleeping_ball_updates: usize,
    pub shadowed_car_frames: usize,
    pub ball_tick_frames: usize,
    pub car_tick_frames: usize,
    pub dodge_activations: usize,
}

/// The players and what the simulation counted.
#[derive(Debug, Clone)]
pub struct Simulation {
    pub players: Vec<SimPlayer>,
    pub diagnostics: SimDiagnostics,
}

/// Holds per player.
#[derive(Default)]
struct Holds {
    /// Players whose car is held on its spawn pose, by the car life that holds it.
    spawning: BTreeMap<PlayerIndex, CarLife>,
    /// Players held as wrecks, by the car life the wreck belongs to.
    wrecks: BTreeMap<PlayerIndex, (CarLife, HoldSource)>,
    /// The replay tick until which an observed demolition keeps a player demolished.
    demolished_until: BTreeMap<PlayerIndex, u64>,
}

/// What a frame's car updates collect.
struct FrameContext {
    index: FrameIndex,
    replay_tick: u64,
    /// Ticks since the previous frame.
    gap: u64,
    simulated: bool,
    withheld: bool,
    in_play: bool,
    /// Players whose car was already updated in this frame.
    selected: BTreeSet<PlayerIndex>,
    sleeping_velocity_zeroed: Vec<Option<ActorId>>,
    wrecks_inferred: Vec<ActorId>,
    /// Inputs chosen at this frame's updates, with their sim tick.
    fitted: Vec<(PlayerIndex, u64, FittedKind)>,
    predictions: Vec<Prediction>,
}

/// What last set a car's controls: its pitch, yaw and roll, and its throttle, steer, handbrake and boost.
pub type ControlSources = (AirControlSource, GroundControlSource);

/// The control sources of player slot `slot`, growing the list as players appear.
fn sources(list: &mut Vec<ControlSources>, slot: usize) -> &mut ControlSources {
    if list.len() <= slot {
        list.resize(
            slot + 1,
            (AirControlSource::Unset, GroundControlSource::Network),
        );
    }
    &mut list[slot]
}

/// The car's jump counter was odd (a jump) or changed (a jump pressed and released between two frames) in one of
/// the last 10 frames up to `f`: a jump that may not have taken off yet. A counter re-sent with its even value (on
/// a curved wall, where the velocity has a share along the car's up axis) does not count, nor a jump after which
/// the double-jump or dodge counter changed while the simulated car was airborne since (`airborne`, the last frame
/// whose update found it so): that jump flew in the simulation, and the car has landed.
fn recent_jump(
    frames: &[NetworkFrame],
    f: usize,
    car: &NetworkCar,
    airborne: Option<usize>,
) -> bool {
    let counters = |g: usize| {
        frames[g].cars.iter().find(|c| c.life == car.life).map(|c| {
            (
                c.inputs.jump_active_raw.as_ref().map(|v| v.value),
                c.inputs.double_jump_active_raw.as_ref().map(|v| v.value),
                c.inputs.dodge_active_raw.as_ref().map(|v| v.value),
            )
        })
    };
    let Some(now) = counters(f) else {
        return false;
    };
    (f.saturating_sub(10).max(1)..=f).rev().any(|g| {
        counters(g).is_some_and(|c| {
            // An absent counter is 0, as in the counter rule.
            let jump = c.0.unwrap_or(0);
            let changed = counters(g - 1).is_some_and(|before| before.0.unwrap_or(0) != jump);
            (jump % 2 == 1 || changed)
                && ((c.1, c.2) == (now.1, now.2) || airborne.is_none_or(|a| a < g))
        })
    })
}

/// A car's update at the end of a simulated interval, for `switch_by_simulation`.
struct UpdateTarget {
    /// Ticks from the interval's start to the update.
    update: u64,
    position: glam::Vec3A,
    velocity: glam::Vec3A,
    /// The car's network switches in the interval: (ticks after the start, the controls).
    switches: Vec<(u64, CarControls)>,
    /// The car shows its controls ahead of their effect (`leads`): in the air its switches set more, as in
    /// `advance_to`.
    leads: bool,
}

impl UpdateTarget {
    fn new(
        update: u64,
        position: glam::Vec3A,
        velocity: glam::Vec3A,
        car: &NetworkCar,
        switches: &[(u64, &NetworkCar)],
    ) -> Self {
        let switches = switches
            .iter()
            .filter(|(_, c)| c.life == car.life)
            .map(|&(at, c)| (at, updates::network_controls(c)))
            .collect();
        Self {
            update,
            position,
            velocity,
            switches,
            leads: false,
        }
    }
}

/// The ticks of one frame's interval and the control switches due in it.
struct Interval<'a> {
    span: u64,
    remaining: u64,
    /// (ticks after the interval's start, the car whose network controls take effect), in time order.
    switches: Vec<(u64, &'a NetworkCar)>,
    next_switch: usize,
}

struct Simulator<'a, 'i> {
    network: &'a NetworkReplay,
    inference: &'i mut dyn Inference,
    ticks: Option<&'a UpdateTicks>,
    options: SimulationOptions,
    arena: Arena,
    players: Players,
    cars: HashMap<CarLife, CarTrack>,
    holds: Holds,
    /// The air schedules being flown.
    air_schedules: Vec<AirSchedule>,
    /// The ground schedules being driven, and the dodges to press.
    ground_schedules: Vec<(PlayerIndex, GroundSchedule)>,
    pending_dodges: Vec<PendingDodge>,
    /// The current interval's early presses (`EarlyPress`).
    pub(super) early_presses: Vec<EarlyPress>,
    /// The players whose flip reset was inferred since their last frame (`infer_flip_resets`).
    inferred_resets: BTreeSet<usize>,
    /// The players pressing a double jump or a dodge in the coming step (`infer_flip_resets`).
    action_presses: BTreeSet<usize>,
    /// The current interval's boost switches in the air (`plan_air_boost`): sim tick, player slot, boost.
    air_boost: Vec<(u64, usize, bool)>,
    /// The current interval's throttle switches in the air (`plan_air_throttle`): sim tick, player slot, throttle.
    air_throttle: Vec<(u64, usize, f32)>,
    /// The current interval's jump releases (`plan_jump_release`): sim tick, player slot.
    jump_releases: Vec<(u64, usize)>,
    /// The held jumps of cars showing their controls ahead of their effect (`leads`), released at their tick:
    /// (tick, slot), across intervals.
    pub(super) lead_releases: Vec<(u64, usize)>,
    /// Dodges waiting for their car to leave the ground (`DeferredDodge`).
    pub(super) deferred_dodges: Vec<DeferredDodge>,
    /// The ticks stepped in the current frame's interval.
    stepped: Vec<TickRecord>,
    /// The interval's cars whose throttle and steer ramp across a change (by player slot), the replay tick of sim
    /// tick 0 (replay tick = sim tick + offset), and the frame.
    ramp_cars: Vec<(usize, &'a NetworkCar)>,
    /// Per car life: the frames whose control shift was at least `LEAD_MIN_SHIFT`, and all frames with a shift.
    lead_votes: HashMap<CarLife, (u32, u32)>,
    ramp_offset: i64,
    ramp_frame: usize,
    /// Per player slot: what last set the car's pitch, yaw and roll, and its throttle, steer, handbrake and boost.
    control_sources: Vec<ControlSources>,
    pads: Pads,
    diagnostics: SimDiagnostics,
    first_time: f32,
    previous_tick: u64,
    previous_in_play: bool,
    ball_initialized: bool,
}

/// Simulates the replay, calling `on_frame` with every frame in order. `ticks` places each update at its
/// update tick; without it every update is applied at its frame's own tick. `inference` answers what the
/// replay does not say.
pub fn simulate(
    network: &NetworkReplay,
    ticks: Option<&UpdateTicks>,
    inference: &mut dyn Inference,
    _meshes: &Meshes,
    options: SimulationOptions,
    mut on_frame: impl FnMut(SimulatedFrame),
) -> Result<Simulation, Error> {
    if network.header.game_type != "TAGame.Replay_Soccar_TA" {
        return Err(Error::UnsupportedMode(network.header.game_type.clone()));
    }
    let mut config = ArenaConfig::new(GameMode::Soccar);
    config.rng_seed = Some(options.seed);
    if !options.simulated_demolitions {
        // The replay reports every demolition; RocketSim's own rule reproduced 83% of them and invented as many
        // (RESULTS.md, "Correction: RocketSim does report demolitions").
        config.mutators.demo_mode = DemoMode::Disabled;
    }
    let arena = Arena::new_with_config(config);
    let pads = Pads::new(&arena, &network.frames);
    let mut simulator = Simulator {
        network,
        inference,
        ticks,
        options,
        arena,
        players: Players::default(),
        cars: HashMap::new(),
        holds: Holds::default(),
        air_schedules: Vec::new(),
        ground_schedules: Vec::new(),
        pending_dodges: Vec::new(),
        early_presses: Vec::new(),
        inferred_resets: BTreeSet::new(),
        action_presses: BTreeSet::new(),
        air_boost: Vec::new(),
        air_throttle: Vec::new(),
        jump_releases: Vec::new(),
        lead_releases: Vec::new(),
        deferred_dodges: Vec::new(),
        stepped: Vec::new(),
        ramp_cars: Vec::new(),
        lead_votes: HashMap::new(),
        ramp_offset: 0,
        ramp_frame: 0,
        control_sources: Vec::new(),
        pads,
        diagnostics: SimDiagnostics::default(),
        first_time: network.frames.first().map_or(0.0, |frame| frame.time),
        previous_tick: 0,
        previous_in_play: false,
        ball_initialized: false,
    };
    for frame in &network.frames {
        on_frame(simulator.frame(frame)?);
    }
    Ok(Simulation {
        players: simulator.players.players,
        diagnostics: simulator.diagnostics,
    })
}

impl<'a> Simulator<'a, '_> {
    fn frame(&mut self, frame: &'a NetworkFrame) -> Result<SimulatedFrame, Error> {
        let f = frame.index.get();
        if !frame.time.is_finite() || frame.time < self.first_time {
            return Err(Error::InvalidTime {
                frame: frame.index,
                time: frame.time,
            });
        }
        let replay_tick = (((f64::from(frame.time) - f64::from(self.first_time)) * 120.0).round()
            as u64)
            .max(self.previous_tick);
        let mut gap = replay_tick - self.previous_tick;
        let state = frame.game_state.as_ref().map(|s| &s.value);
        let active = state == Some(&GameState::Active);
        // The frame that reports a goal ends the play before it: the ball crossed the line in its interval, so the
        // interval is simulated like any other (it is the last of its play segment); play stops after it.
        let goal_frame =
            !active && self.previous_in_play && state == Some(&GameState::PostGoalScored);
        let in_play = active || goal_frame;
        // The first frame of play after the countdown: its interval runs from the cars' release.
        let kickoff = active
            && !self.previous_in_play
            && gap > 0
            && gap <= MAX_GAP_TICKS
            && f > 0
            && self.network.frames[f - 1]
                .game_state
                .as_ref()
                .is_some_and(|s| s.value == GameState::Countdown);
        let mut simulated = in_play && self.previous_in_play && gap > 0 && gap <= MAX_GAP_TICKS;
        self.previous_tick = replay_tick;
        self.previous_in_play = active;
        let current = frame.current_cars();
        self.diagnostics.shadowed_car_frames +=
            frame.cars.iter().filter(|car| car.player.is_some()).count() - current.len();
        let frame_cars: Vec<&NetworkCar> = current
            .into_iter()
            .chain(frame.cars.iter().filter(|car| car.player.is_none()))
            .collect();
        let withheld = self
            .options
            .withheld
            .as_ref()
            .is_some_and(|w| w.get(f).copied().unwrap_or(false));
        // A kickoff is simulated from the release: the cars at rest on their spawns with the inputs they hold, so
        // that the segment's first tick is the spawn and the action taken there (`release_kickoff`).
        let from_release = kickoff && !withheld;
        if from_release {
            let release = self.release_kickoff(&frame_cars, f, gap);
            self.diagnostics.skipped_replay_ticks += gap - release;
            self.diagnostics.kickoffs_from_release += 1;
            gap = release;
            simulated = true;
        }
        if !simulated {
            self.diagnostics.skipped_replay_ticks += gap;
        }
        if let (Some(ticks), true) = (self.ticks, simulated) {
            self.diagnostics.ball_tick_frames += usize::from(ticks.ball[f].is_some());
            self.diagnostics.car_tick_frames += usize::from(ticks.car_median[f].is_some());
        }
        let mut ctx = FrameContext {
            index: frame.index,
            replay_tick,
            gap,
            simulated,
            withheld,
            in_play,
            selected: BTreeSet::new(),
            sleeping_velocity_zeroed: Vec::new(),
            wrecks_inferred: Vec::new(),
            fitted: Vec::new(),
            predictions: Vec::new(),
        };
        let ball_ticks = self.ball_ticks(f, simulated, gap);
        let applied_ticks = self.applied_ticks(frame, &frame_cars, simulated, gap);
        let mut phases: Vec<u64> = std::iter::once(ball_ticks)
            .chain(
                frame_cars
                    .iter()
                    .map(|car| self.car_ticks(car, f, simulated, gap)),
            )
            .collect();
        phases.sort_unstable_by(|a, b| b.cmp(a));
        phases.dedup();
        let span = if simulated { gap } else { 0 };
        let mut interval = Interval {
            span,
            remaining: span,
            switches: if from_release {
                // The inputs this frame reports are those held at the release.
                frame_cars.iter().map(|&car| (0, car)).collect()
            } else {
                self.switches(&frame_cars, f, replay_tick, span, gap, withheld)
            },
            next_switch: 0,
        };
        for &car in &frame_cars {
            if let Some(shift) = self.inference.control_shift(car.life) {
                let votes = self.lead_votes.entry(car.life).or_default();
                votes.0 += u32::from(shift >= LEAD_MIN_SHIFT);
                votes.1 += 1;
            }
        }
        self.ramp_cars.clear();
        if simulated && !withheld && !from_release && self.ticks.is_some() {
            self.ramp_frame = f;
            self.ramp_offset = replay_tick as i64 - (self.arena.tick_count() + span) as i64;
            for &car in &frame_cars {
                if let Some(&(player, created)) = self.players.by_actor.get(&car.life.actor)
                    && created == car.life.created
                {
                    self.ramp_cars.push((player.get(), car));
                }
            }
        }
        self.early_presses.clear();
        self.air_boost.clear();
        self.air_throttle.clear();
        self.jump_releases.clear();
        if simulated && !withheld && self.ticks.is_some() {
            self.plan_early_presses(&frame_cars, f, span, gap, &interval.switches);
            self.plan_lead_releases(&frame_cars, f, gap);
            if self.options.air_boost_timing {
                self.plan_air_boost(&frame_cars, f, span, gap, &interval.switches);
                self.plan_air_throttle(&frame_cars, f, span, gap);
                self.plan_jump_release(&frame_cars, f, span, gap, &interval.switches);
            }
        }
        let mut events = Vec::new();
        if !self.options.simulated_pad_pickups {
            // Held on cooldown from the first tick: the arena carries the true cooldowns written back at the end
            // of the last frame, and the ticks before the last phase would let a car pick up a pad the replay
            // has not reported.
            Pads::hold(&mut self.arena);
        }
        for ticks_before in phases {
            let target = span - ticks_before.min(interval.remaining);
            self.advance_to(&mut interval, target, &mut events);
            if ball_ticks == ticks_before
                && let Some(body) = &frame.ball
            {
                self.update_ball(&mut ctx, body);
            }
            for car in frame_cars.iter().copied() {
                if self.car_ticks(car, f, simulated, gap) == ticks_before {
                    self.update_car(&mut ctx, car);
                }
            }
        }
        self.advance_to(&mut interval, span, &mut events);
        if !self.options.simulated_pad_pickups {
            self.pads.recharge(gap as f32 / 120.0);
        }
        if simulated && !withheld {
            self.apply_demolitions(frame, &frame_cars, replay_tick);
            self.apply_flip_resets(frame, &frame_cars);
        }
        self.pads
            .apply(&mut self.arena, frame, &self.network.frames, &self.players);
        if !self.options.simulated_pad_pickups {
            self.pads.write(&mut self.arena);
            // The pads are held on cooldown: a pickup the simulation still reports would count one twice.
            events.retain(|e| !matches!(e.event, ArenaEvent::CarPickupBoost(_)));
        }
        Ok(self.output(frame, ctx, replay_tick, events, applied_ticks))
    }

    /// Puts every car of a kickoff at rest on its spawn (the pose of its countdown updates, at RocketSim's rest
    /// height) with the inputs it holds at the release, those this first frame of play reports (the boost and jump
    /// counters only turn on at the release; a held jump is left to the presses), and returns the release's ticks
    /// before this frame: the one at which RocketSim, from rest with those inputs, best reproduces the speeds of
    /// the cars' first updates where the interval places them (all cars are released together); half the frame
    /// gap when no car moves. Within 1..=`gap`, after the last countdown frame.
    fn release_kickoff(&mut self, cars: &[&'a NetworkCar], f: usize, gap: u64) -> u64 {
        let mut released = Vec::new();
        for &car in cars {
            let Some(&(player, created)) = self.players.by_actor.get(&car.life.actor) else {
                continue;
            };
            let slot = player.get();
            let mut state = *self.arena.get_car_state(slot);
            if created != car.life.created || state.is_demoed {
                continue;
            }
            state.phys.vel = glam::Vec3A::ZERO;
            state.phys.ang_vel = glam::Vec3A::ZERO;
            let rest = rocketsim::consts::car::spawn::REST_Z;
            if (state.phys.pos.z - rest).abs() < 5.0 {
                state.phys.pos.z = rest;
            }
            state.controls = updates::network_controls(car);
            state.controls.jump = false;
            self.arena.set_car_state_full(slot, state);
            self.arena.refresh_car_contacts(slot);
            released.push((slot, car));
        }
        let mut probe = self.arena.clone();
        let mut speeds = vec![Vec::with_capacity(gap as usize); released.len()];
        for _ in 0..gap {
            probe.step_tick();
            for (i, &(slot, _)) in released.iter().enumerate() {
                speeds[i].push(probe.get_car_state(slot).phys.vel.length());
            }
        }
        let frame = FrameIndex(f as u32);
        let moving: Vec<(&NetworkCar, &Vec<f32>, f32)> = released
            .iter()
            .zip(&speeds)
            .filter_map(|(&(_, car), speeds)| {
                let velocity = car
                    .body
                    .linear_velocity
                    .as_ref()
                    .filter(|v| v.frame == frame)?;
                let speed = glam::Vec3A::from(velocity.value).length();
                (speed >= 5.0).then_some((car, speeds, speed))
            })
            .collect();
        if moving.is_empty() {
            return (gap / 2).clamp(1, gap);
        }
        // Each candidate release with the update ticks the interval would then use (a car without its own takes
        // half the interval), so that the chosen one reproduces the speeds where the updates land.
        (1..=gap)
            .min_by(|&a, &b| {
                let cost = |release: u64| -> f32 {
                    moving
                        .iter()
                        .map(|&(car, speeds, speed)| {
                            let since = release - self.car_ticks(car, f, true, release);
                            let simulated = if since == 0 {
                                0.0
                            } else {
                                speeds[since as usize - 1]
                            };
                            (simulated - speed).abs()
                        })
                        .sum()
                };
                cost(a).total_cmp(&cost(b))
            })
            .unwrap_or(1)
    }

    /// The ball's update ticks before its frame: its chain's, else half the window.
    fn ball_ticks(&self, f: usize, simulated: bool, gap: u64) -> u64 {
        match (self.ticks, simulated) {
            (Some(ticks), true) => ticks.ball[f].map_or(gap / 2, u64::from).min(gap),
            _ => 0,
        }
    }

    /// A car update's ticks before its frame: a fit's, else its own chain's, else the frame's median car, else
    /// half the window.
    fn car_ticks(&self, car: &NetworkCar, f: usize, simulated: bool, gap: u64) -> u64 {
        match (self.ticks, simulated) {
            (Some(ticks), true) => self
                .inference
                .ticks_override(car.life, f)
                .or_else(|| {
                    ticks
                        .cars
                        .get(&(car.life, FrameIndex(f as u32)))
                        .copied()
                        .or(ticks.car_median[f])
                        .map(u64::from)
                })
                .unwrap_or(gap / 2)
                .min(gap),
            _ => 0,
        }
    }

    fn applied_ticks(
        &self,
        frame: &NetworkFrame,
        cars: &[&NetworkCar],
        simulated: bool,
        gap: u64,
    ) -> Vec<AppliedTick> {
        let (Some(ticks), true) = (self.ticks, simulated) else {
            return Vec::new();
        };
        let f = frame.index.get();
        let mut applied = Vec::new();
        if frame
            .ball
            .as_ref()
            .and_then(|b| b.position.as_ref())
            .is_some_and(|p| p.frame == frame.index)
        {
            applied.push(AppliedTick {
                car: None,
                ticks: self.ball_ticks(f, simulated, gap),
                source: if ticks.ball[f].is_some() {
                    TickSource::Chain
                } else {
                    TickSource::Default
                },
            });
        }
        for car in cars {
            if !car
                .body
                .position
                .as_ref()
                .is_some_and(|p| p.frame == frame.index)
            {
                continue;
            }
            let source = if self.inference.ticks_override(car.life, f).is_some() {
                TickSource::DodgeFit
            } else if ticks.cars.contains_key(&(car.life, frame.index)) {
                TickSource::Chain
            } else if ticks.car_median[f].is_some() {
                TickSource::FrameMedian
            } else {
                TickSource::Default
            };
            applied.push(AppliedTick {
                car: Some(car.life.actor),
                ticks: self.car_ticks(car, f, simulated, gap),
                source,
            });
        }
        applied
    }

    /// When each car's network controls take effect inside the interval: controls first seen at this frame
    /// act from `gap / 2 - 2` ticks in (a change takes about as long to be seen as a frame lasts). A car with a
    /// fitted control shift has the controls of the frames around this one, each moved by the shift from that
    /// rule: the latest in effect at the interval's start applies from its start, the later ones when due.
    fn switches(
        &self,
        cars: &[&'a NetworkCar],
        f: usize,
        replay_tick: u64,
        span: u64,
        gap: u64,
        withheld: bool,
    ) -> Vec<(u64, &'a NetworkCar)> {
        if self.ticks.is_none() || span == 0 || withheld {
            return Vec::new();
        }
        let frames = &self.network.frames;
        let first_time = f64::from(self.first_time);
        let tick_of = |x: usize| ((f64::from(frames[x].time) - first_time) * 120.0).round() as i64;
        let interval_start = replay_tick as i64 - gap as i64;
        let mut switches: Vec<(u64, &'a NetworkCar)> = Vec::new();
        for &car in cars {
            let Some(shift) = self.inference.control_shift(car.life) else {
                switches.push(((gap / 2).saturating_sub(2).min(span), car));
                continue;
            };
            let mut in_effect: Option<&'a NetworkCar> = None;
            let mut later: Vec<(u64, &'a NetworkCar)> = Vec::new();
            // The frames whose switch can fall in this interval: further back the larger the shift.
            let back = 4 + shift.unsigned_abs() as usize / 3;
            for g in f.saturating_sub(back)..=(f + 4).min(frames.len() - 1) {
                let Some(other) = frames[g].cars.iter().find(|c| c.life == car.life) else {
                    continue;
                };
                let spacing = if g == 0 {
                    4
                } else {
                    tick_of(g) - tick_of(g - 1)
                };
                let switch = tick_of(g) - 2 - spacing / 2 + shift - interval_start;
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
        }
        switches.sort_by_key(|(switch, _)| *switch);
        switches
    }

    /// Steps the arena to `target` ticks after the interval's start, applying the control switches due.
    fn advance_to(&mut self, interval: &mut Interval<'a>, target: u64, events: &mut Vec<SimEvent>) {
        let target = target.min(interval.span);
        while interval.next_switch < interval.switches.len()
            && interval.switches[interval.next_switch].0 <= target
        {
            let (switch, car) = interval.switches[interval.next_switch];
            interval.next_switch += 1;
            let elapsed = interval.span - interval.remaining;
            if switch > elapsed {
                self.step(switch - elapsed, events);
                interval.remaining -= switch - elapsed;
            }
            let Some(&(player, created)) = self.players.by_actor.get(&car.life.actor) else {
                continue;
            };
            if created != car.life.created {
                continue;
            }
            let next = updates::network_controls(car);
            let mut controls = *self.arena.get_car_controls(player.get());
            // In the air only the steer switches: the air fits solve from the car's update with the other controls
            // as they were (switching boost and throttle there too made the held-out errors worse; RESULTS.md,
            // "Inputs between frames").
            if !self.arena.get_car_state(player.get()).is_on_ground() {
                controls.steer = next.steer;
                if self.leads(car.life) {
                    controls.throttle = next.throttle;
                    controls.boost = next.boost;
                    // Air controls from the steer (no solved ones): the shifted steer too.
                    if self
                        .control_sources
                        .get(player.get())
                        .is_some_and(|s| s.0 == AirControlSource::Steer)
                    {
                        if next.handbrake {
                            controls.roll = next.steer;
                        } else {
                            controls.yaw = next.steer;
                        }
                        if next.handbrake {
                            controls.yaw = 0.0;
                        } else {
                            controls.roll = 0.0;
                        }
                    }
                    // The handbrake too: it picks roll or yaw, and RocketSim carries its value into the landing.
                    controls.handbrake = next.handbrake;
                }
                self.arena.set_car_controls(player.get(), controls);
                continue;
            }
            controls.throttle = next.throttle;
            controls.steer = next.steer;
            controls.handbrake = next.handbrake;
            controls.boost = next.boost;
            self.arena.set_car_controls(player.get(), controls);
            sources(&mut self.control_sources, player.get()).1 = GroundControlSource::Network;
        }
        let elapsed = interval.span - interval.remaining;
        if target > elapsed {
            self.step(target - elapsed, events);
            interval.remaining -= target - elapsed;
        }
    }

    /// Steps `ticks` ticks, driving the air and ground schedules and pressing the pending dodges, then limits
    /// the reported velocities.
    fn step(&mut self, ticks: u64, events: &mut Vec<SimEvent>) {
        for _ in 0..ticks {
            let sim_tick = self.arena.tick_count() + 1;
            self.pending_dodges
                .retain(|dodge| dodge.end_tick >= sim_tick);
            self.ground_schedules
                .retain(|(_, schedule)| schedule.end_tick >= sim_tick);
            self.air_schedules
                .retain(|schedule| schedule.end_tick >= sim_tick);
            for schedule in &self.air_schedules {
                let Some(&(_, air)) = schedule.entries.iter().rev().find(|e| e.0 <= sim_tick)
                else {
                    continue;
                };
                let slot = schedule.player.get();
                let mut controls = *self.arena.get_car_controls(slot);
                // A jump press in the air is a double jump or a flip, whose kind RocketSim takes from the
                // same controls: the press keeps its own. A held jump is no press and flies the schedule.
                let state = self.arena.get_car_state(slot);
                if controls.jump && !state.prev_controls().jump && !state.is_on_ground() {
                    continue;
                }
                controls.pitch = air.pitch;
                controls.yaw = air.yaw;
                controls.roll = air.roll;
                self.arena.set_car_controls(slot, controls);
                sources(&mut self.control_sources, slot).0 = AirControlSource::Schedule;
            }
            self.drive_ramps(sim_tick);
            self.drive_ground_schedules(sim_tick);
            self.switch_air_boost(sim_tick);
            self.switch_air_throttle(sim_tick);
            self.press_dodges(sim_tick);
            self.press_early(sim_tick);
            self.press_deferred(sim_tick);
            self.release_jumps(sim_tick);
            self.release_lead_jumps(sim_tick);
            self.release_before_presses(sim_tick);
            self.infer_flip_resets();
            self.record_tick();
            events.extend(
                self.arena
                    .step_tick()
                    .iter()
                    .copied()
                    .map(|event| SimEvent { sim_tick, event }),
            );
        }
        updates::limit_velocities(&mut self.arena, self.players.len());
    }

    /// A fitted or deferred dodge is planned for the player.
    fn plans_dodge(&self, player: PlayerIndex) -> bool {
        self.pending_dodges.iter().any(|d| d.player == player)
            || self.deferred_dodges.iter().any(|d| d.player == player)
    }

    /// Releases the jump the tick before a planned dodge or double-jump press, so that the press is a press: a
    /// takeoff jump held to its update (`press_early`) could otherwise run into a fitted dodge's press tick.
    fn release_before_presses(&mut self, sim_tick: u64) {
        let next = sim_tick + 1;
        let slots = self
            .pending_dodges
            .iter()
            .filter(|d| d.start_tick == next)
            .map(|d| d.player.get())
            .chain(
                self.early_presses
                    .iter()
                    .filter(|p| p.tick == next && p.kind != EarlyKind::Jump)
                    .map(|p| p.player.get()),
            )
            .collect::<Vec<_>>();
        for slot in slots {
            let mut controls = *self.arena.get_car_controls(slot);
            if controls.jump {
                controls.jump = false;
                self.arena.set_car_controls(slot, controls);
            }
        }
    }

    /// A jump press RocketSim would refuse: an airborne car whose double jump and flip are used up or expired.
    /// Every press in the air comes from the replay's action counters (or a fit of them), so the car did double
    /// jump or dodge in the game, which it can only after a flip reset the simulation missed: its wheels touched
    /// the ball, but in the simulation fewer of them than the three RocketSim needs. The reset is applied before
    /// the press, as an observed one is, so that the press double jumps or flips (with its direction). Only the
    /// presses of a double jump or dodge count (`action_presses`): the game counts those on their own counters, and
    /// never after a flip reset on the jump counter, whose presses in the air are first jumps the simulation has
    /// not landed for yet.
    fn infer_flip_resets(&mut self) {
        for slot in std::mem::take(&mut self.action_presses) {
            let state = self.arena.get_car_state(slot);
            let pressed = state.controls.jump && !state.prev_controls().jump;
            if !pressed
                || state.is_on_ground()
                || state.is_demoed
                || state.is_auto_flipping
                || state.has_flip_or_jump()
            {
                continue;
            }
            let mut state = *state;
            reports::reset_flip(&mut state);
            self.arena.set_car_state_full(slot, state);
            self.diagnostics.flip_resets_inferred += 1;
            self.inferred_resets.insert(slot);
        }
    }

    /// An analog stick's throttle and steer at `sim_tick`: linear across each observed change that involves a value
    /// other than -1, 0 or 1 (over one frame gap centred on its switch tick), held elsewhere; a change between
    /// those values is a step. In the air only the steer ramps, as only it switches there. The replay sends one
    /// value per frame, so the ramp is a guess at the stick's path between two; against the true inputs of the
    /// RLBot and RocketSim recordings it halves the steer error of a stick player (RESULTS.md).
    fn drive_ramps(&mut self, sim_tick: u64) {
        if self.ramp_cars.is_empty() {
            return;
        }
        let t = sim_tick as i64 + self.ramp_offset;
        let frames = &self.network.frames;
        let first_time = f64::from(self.first_time);
        let tick_of = |x: usize| ((f64::from(frames[x].time) - first_time) * 120.0).round() as i64;
        for i in 0..self.ramp_cars.len() {
            let (slot, car) = self.ramp_cars[i];
            let airborne = !self.arena.get_car_state(slot).is_on_ground();
            let shift = self.inference.control_shift(car.life).unwrap_or(0);
            let f = self.ramp_frame;
            // (switch tick, half gap, throttle, steer) of this car life's frames around this one.
            let mut points: Vec<(i64, i64, f32, f32)> = Vec::new();
            for g in f.saturating_sub(8)..=(f + 8).min(frames.len() - 1) {
                let Some(other) = frames[g].cars.iter().find(|c| c.life == car.life) else {
                    continue;
                };
                let spacing = if g == 0 {
                    4
                } else {
                    tick_of(g) - tick_of(g - 1)
                };
                let c = updates::network_controls(other);
                points.push((
                    tick_of(g) - 2 - spacing / 2 + shift,
                    (spacing / 2).max(1),
                    c.throttle,
                    c.steer,
                ));
            }
            let Some(k) = points.iter().rposition(|p| p.0 <= t) else {
                continue;
            };
            let ramp = |a: f32, b: f32, frac: f64| -> Option<f32> {
                let digital = |v: f32| v == -1.0 || v == 0.0 || v == 1.0;
                if a == b || (digital(a) && digital(b)) {
                    return None;
                }
                Some(a + (b - a) * frac.clamp(0.0, 1.0) as f32)
            };
            let mut value = (points[k].2, points[k].3);
            if let Some(next) = points.get(k + 1)
                && t >= next.0 - next.1
            {
                let frac = (t - (next.0 - next.1)) as f64 / (2 * next.1) as f64;
                value.0 = ramp(points[k].2, next.2, frac).unwrap_or(value.0);
                value.1 = ramp(points[k].3, next.3, frac).unwrap_or(value.1);
            } else if k >= 1 && t <= points[k].0 + points[k].1 {
                let (prev, here) = (points[k - 1], points[k]);
                let frac = 0.5 + (t - here.0) as f64 / (2 * here.1) as f64;
                value.0 = ramp(prev.2, here.2, frac).unwrap_or(value.0);
                value.1 = ramp(prev.3, here.3, frac).unwrap_or(value.1);
            }
            let mut controls = *self.arena.get_car_controls(slot);
            if !airborne {
                controls.throttle = value.0;
            }
            controls.steer = value.1;
            self.arena.set_car_controls(slot, controls);
        }
    }

    /// The presses this frame's updates already show: a jump, double-jump or dodge counter that turns odd in this
    /// frame while the car's update in it carries the impulse. Without a press here the update would set the car's
    /// flags and the controls would never show the press. A jump is placed by the time since it started (from the
    /// update's rise speed: the impulse of 291.7 UU/s, then 1,458.3 UU/s^2 of hold force against 650 of gravity); a
    /// double jump or dodge at the midpoint rule, before the update (a leading car's double jump that the update
    /// does not fit is deferred instead, `lead_double_jump_delay`, and pressed where a later update places it).
    /// Cars a fit already plans are left to it.
    fn plan_early_presses(
        &mut self,
        cars: &[&'a NetworkCar],
        f: usize,
        span: u64,
        gap: u64,
        switches: &[(u64, &'a NetworkCar)],
    ) {
        if f == 0 {
            return;
        }
        let start = self.arena.tick_count();
        let frames = &self.network.frames;
        for &car in cars {
            let Some(&(player, created)) = self.players.by_actor.get(&car.life.actor) else {
                continue;
            };
            if created != car.life.created
                || self.ground_schedules.iter().any(|(p, _)| *p == player)
            {
                continue;
            }
            // A dodge fit plans the dodge only: the takeoff's jump below is still this pass's.
            let dodge_fitted = self.inference.dodge_handled(car.life, f);
            let dodge_planned = self.pending_dodges.iter().any(|d| d.player == player);
            let Some(previous) = frames[f - 1].cars.iter().find(|c| c.life == car.life) else {
                continue;
            };
            let frame = FrameIndex(f as u32);
            let rose = |now: &Option<crate::decode::NetworkValue<u8>>,
                        before: &Option<crate::decode::NetworkValue<u8>>| {
                now.as_ref()
                    .is_some_and(|v| v.frame == frame && v.value % 2 == 1)
                    && before.as_ref().is_none_or(|v| v.value % 2 == 0)
            };
            let Some(velocity) = car
                .body
                .linear_velocity
                .as_ref()
                .filter(|v| v.frame == frame)
            else {
                continue;
            };
            let update = span - self.car_ticks(car, f, true, gap).min(span);
            if update == 0 {
                // An update from before the interval that shows a deferred double jump's rise: it is behind.
                if let Some(i) = self
                    .deferred_dodges
                    .iter()
                    .position(|d| d.life == car.life && d.from > 0)
                {
                    let state = self.arena.get_car_state(player.get());
                    let rise = (glam::Vec3A::from(velocity.value) - state.phys.vel)
                        .dot(state.phys.rot_mat.z_axis);
                    if rise > 200.0 {
                        self.deferred_dodges.remove(i);
                    }
                }
                continue;
            }
            let state = self.arena.get_car_state(player.get());
            let airborne = !state.is_on_ground() || state.phys.pos.z > 50.0;
            let rule = (gap / 2).saturating_sub(2).min(update - 1);
            // A leading car's double jump deferred from an earlier frame: pressed where this update places it, or
            // at the midpoint rule when the update shows the rise but not where; else it waits past this update.
            if airborne
                && let Some(i) = self
                    .deferred_dodges
                    .iter()
                    .position(|d| d.life == car.life && d.from > 0)
            {
                let up = state.phys.rot_mat.z_axis;
                let rise = (glam::Vec3A::from(velocity.value) - state.phys.vel).dot(up);
                let timed = self
                    .options
                    .air_boost_timing
                    .then(|| {
                        self.impulse_tick(car, f, EarlyKind::DoubleJump, update, rule, switches)
                    })
                    .flatten();
                let tick = timed.or((rise > 200.0).then_some(rule));
                if let Some(tick) = tick {
                    self.deferred_dodges.remove(i);
                    if timed.is_some() {
                        self.diagnostics.air_presses_timed += 1;
                    }
                    self.early_presses.push(EarlyPress {
                        player,
                        life: car.life,
                        kind: EarlyKind::DoubleJump,
                        tick: start + tick + 1,
                        update_tick: start + update,
                        applied: false,
                    });
                    continue;
                }
                let dodge = &mut self.deferred_dodges[i];
                dodge.from = dodge.from.max(start + update + 1);
                dodge.until = dodge.until.max(dodge.from + 60);
            }
            // A dodge or double jump the counters show, pressed before the update; when that does not apply (a
            // fit plans the dodge, or the simulated car is still on the ground), the frame may still show the
            // takeoff of the jump before it.
            let air_press = if rose(
                &car.inputs.dodge_active_raw,
                &previous.inputs.dodge_active_raw,
            ) {
                updates::dodge_torque(frames, f, car).and_then(|[tx, ty, _]| {
                    let (pitch, yaw) = (-ty / 2.24, -tx / 2.60);
                    (!dodge_fitted
                        && !dodge_planned
                        && airborne
                        && (pitch * pitch + yaw * yaw).sqrt() > 0.01)
                        .then_some(EarlyKind::Dodge { pitch, yaw })
                })
            } else if rose(
                &car.inputs.double_jump_active_raw,
                &previous.inputs.double_jump_active_raw,
            ) {
                (!dodge_fitted && !dodge_planned && airborne).then_some(EarlyKind::DoubleJump)
            } else {
                None
            };
            let kind = if let Some(kind) = air_press {
                let timed = self
                    .options
                    .air_boost_timing
                    .then(|| self.impulse_tick(car, f, kind, update, rule, switches))
                    .flatten();
                if timed.is_some() {
                    self.diagnostics.air_presses_timed += 1;
                }
                // A leading car's double jump the update does not show: it comes after the counter's press.
                if timed.is_none()
                    && kind == EarlyKind::DoubleJump
                    && self.leads(car.life)
                    && let Some(shift) = self.inference.control_shift(car.life)
                {
                    let from = start + rule + 1 + lead_double_jump_delay(shift);
                    self.deferred_dodges.push(DeferredDodge {
                        player,
                        life: car.life,
                        pitch: 0.0,
                        yaw: 0.0,
                        from,
                        until: from + 60,
                    });
                    continue;
                }
                (kind, timed.unwrap_or(rule))
            } else if (rose(
                &car.inputs.jump_active_raw,
                &previous.inputs.jump_active_raw,
            ) && !dodge_planned
                && !dodge_fitted)
                || (state.is_on_ground()
                    && !state.has_jumped
                    && car.body.position.as_ref().is_some_and(|p| {
                        p.frame == frame && (p.value[2] < 150.0 || state.is_on_ground())
                    })
                    && recent_jump(
                        frames,
                        f,
                        car,
                        self.cars.get(&car.life).and_then(|t| t.airborne_frame),
                    ))
            {
                // A jump counter that rose in this frame, or a takeoff in this frame's update while the simulated
                // car is still on the ground (the counters can lead the updates by frames, and an update showing
                // the car at rest undoes the simulation's own jump): the jump that took off. Also when a fit plans
                // the car's dodge or the dodge counter rose with it (a jump-and-dodge whose takeoff the updates
                // show late): the dodge waits for the car to be airborne, and without this press the takeoff came
                // from the update alone.
                // The rise along the car's up axis, so that a jump off a wall or the ceiling counts as one off
                // the floor does: the impulse (291.7 UU/s) and the hold force (1,458.3 UU/s^2) push along it,
                // gravity's share depends on the surface.
                let up = state.phys.rot_mat.z_axis;
                let rise = glam::Vec3A::from(velocity.value).dot(up);
                if !state.is_on_ground() || rise <= 150.0 {
                    continue;
                }
                let accel = 1458.333 - 650.0 * up.z;
                let since = ((rise - 291.667) / accel * 120.0).round().clamp(1.0, 23.0) as u64;
                (EarlyKind::Jump, update.saturating_sub(since))
            } else {
                continue;
            };
            self.early_presses.push(EarlyPress {
                player,
                life: car.life,
                kind: kind.0,
                tick: start + kind.1 + 1,
                update_tick: start + update,
                applied: false,
            });
        }
    }

    /// The step (from the interval's start) at which a double jump or dodge the frame's update shows was pressed,
    /// from where the update puts the car. Both are a velocity impulse `J` (291.7 UU/s along the car's up axis, or
    /// the dodge's along the ground plane) on top of forces that are steady over the interval (gravity, a boost that
    /// does not switch): with RocketSim's integration (velocity, then position), an impulse in step `k` of `n`
    /// leaves the position `dt * J * ((n - 1) / 2 - k)` past `dt * ((n - 1) * v0 + (n + 1) * v1) / 2`, the path of
    /// the velocities `v0` at the start and `v1` at the update, whatever the steady forces are. The dodge's impulse
    /// is the velocity change less gravity and boost. A car that can touch the ball or another car in the interval
    /// (`may_touch`), whose update holds the contact too, or near the speed limit (2,100 UU/s), is timed by
    /// simulation instead (`best_by_simulation`, with the interval's switches; not with another dodge planned for it),
    /// and the midpoint rule confirmed there is a timing too. Not for a car with a wheel on
    /// a surface, still holding its jump (whose force ends in the interval), whose boost switches in it, or whose
    /// evidence does not fit one impulse.
    fn impulse_tick(
        &self,
        car: &NetworkCar,
        f: usize,
        kind: EarlyKind,
        update: u64,
        rule: u64,
        switches: &[(u64, &NetworkCar)],
    ) -> Option<u64> {
        const BOOST: f32 = 2975.0 / 3.0;
        const AIR_THROTTLE: f32 = 200.0 / 3.0;
        const DOUBLE_JUMP: f32 = 875.0 / 3.0;
        let frame = FrameIndex(f as u32);
        let (player, _) = *self.players.by_actor.get(&car.life.actor)?;
        let slot = player.get();
        let state = *self.arena.get_car_state(slot);
        let controls = *self.arena.get_car_controls(slot);
        let fresh = |v: Option<&crate::decode::NetworkValue<[f32; 3]>>| {
            v.filter(|v| v.frame == frame)
                .map(|v| glam::Vec3A::from(v.value))
        };
        let p1 = fresh(car.body.position.as_ref())?;
        let v1 = fresh(car.body.linear_velocity.as_ref())?;
        let rotation = car.body.rotation.as_ref().filter(|v| v.frame == frame)?;
        let boost_switches = car
            .inputs
            .boost_active_raw
            .as_ref()
            .is_some_and(|v| v.frame == frame && (v.value % 2 == 1) != controls.boost);
        if update < 2
            || state.is_on_ground()
            || state.wheels_with_contact().iter().any(Option::is_some)
            || state.is_demoed
            || state.is_jumping
            || boost_switches
        {
            return None;
        }
        // Near the speed limit RocketSim scales the velocity back to 2,300 UU/s, which one impulse does not
        // explain: simulated too.
        let near_limit = state.phys.vel.length() > 2100.0 || v1.length() > 2100.0;
        if near_limit || self.may_touch(slot, v1, update) {
            // The update holds the contact too: the press by simulation, the jump released before it. A dodge
            // planned for the car would be missing from the simulation.
            // (A leading car's deferred double jump is this press, not another.)
            let other_dodge = self.pending_dodges.iter().any(|d| d.player == player)
                || self
                    .deferred_dodges
                    .iter()
                    .any(|d| d.player == player && !(d.life == car.life && d.from > 0));
            if other_dodge {
                return None;
            }
            let pressed = |c: &mut CarControls, step: u64, k: u64| {
                c.jump = step == k + 1;
                if c.jump {
                    let (pitch, yaw) = match kind {
                        EarlyKind::Dodge { pitch, yaw } => (pitch, yaw),
                        _ => (0.0, 0.0),
                    };
                    (c.pitch, c.yaw, c.roll) = (pitch, yaw, 0.0);
                }
            };
            // The rule confirmed is a timing too: `None` would read as an update that does not show it.
            let mut target = UpdateTarget::new(update, p1, v1, car, switches);
            target.leads = self.leads(car.life);
            let (k, cost) = self.best_by_simulation(slot, &target, 0..update, rule, pressed);
            return (cost < 10.0).then_some(k);
        }
        let [x, y, z, w] = rotation.value;
        let end = glam::Mat3A::from_quat(glam::Quat::from_xyzw(x, y, z, w).normalize());
        let n = update as f32;
        let seconds = n / 120.0;
        let change = v1 - state.phys.vel;
        let impulse = match kind {
            EarlyKind::DoubleJump => {
                DOUBLE_JUMP * (state.phys.rot_mat.z_axis + end.z_axis).normalize_or_zero()
            }
            EarlyKind::Dodge { .. } => {
                let forward = (state.phys.rot_mat.x_axis + end.x_axis).normalize_or_zero();
                let throttle = if controls.boost {
                    BOOST + AIR_THROTTLE
                } else {
                    controls.throttle.clamp(-1.0, 1.0) * AIR_THROTTLE
                };
                let steady = glam::Vec3A::new(0.0, 0.0, -650.0) + throttle * forward;
                // The dodge impulse lies in the ground plane.
                (change - steady * seconds).with_z(0.0)
            }
            EarlyKind::Jump => return None,
        };
        // A dodge without a direction has no impulse.
        if impulse.length() < 250.0 {
            return None;
        }
        let offset =
            (p1 - state.phys.pos) * 120.0 - ((n - 1.0) * state.phys.vel + (n + 1.0) * v1) / 2.0;
        let steps = offset.dot(impulse) / impulse.length_squared();
        let k = (n - 1.0) / 2.0 - steps;
        // The part of the offset one impulse does not explain (UU/s, one tick of the impulse is |J|).
        let rest = (offset - steps * impulse).length();
        if !(-0.6..=n - 0.4).contains(&k) || rest > 0.5 * impulse.length() {
            return None;
        }
        Some((k.round().max(0.0) as u64).min(update - 1))
    }

    /// Whether the car shows its controls ahead of their effect: its control shift is at least `LEAD_MIN_SHIFT`
    /// now and was in most of its frames so far.
    fn leads(&self, life: CarLife) -> bool {
        // The votes first: most cars fail them, and each question is recorded for the resimulation.
        self.lead_votes
            .get(&life)
            .is_some_and(|&(lead, all)| 2 * lead > all)
            && self
                .inference
                .control_shift(life)
                .is_some_and(|s| s >= LEAD_MIN_SHIFT)
    }

    /// Whether the car in `slot` can touch the ball or another car in the next `ticks`: unless the distance at the
    /// start less how far each can move (its speed at the start, and the car's at the update `v1`, plus gravity on
    /// the ball and boost on another car) keeps them apart by their bounding spheres (the hitbox's half diagonal plus
    /// its offset, the ball's radius), a copy of the arena is stepped with the controls held, and the car may touch
    /// if on any tick its hitbox comes within 15 UU of the ball or its bounding sphere within 10 UU of another car's.
    fn may_touch(&self, slot: usize, v1: glam::Vec3A, ticks: u64) -> bool {
        const BALL_RADIUS: f32 = 91.25;
        const MARGIN: f32 = 10.0;
        let seconds = ticks as f32 / 120.0;
        let radius = |slot: usize| {
            let config = &self.arena.get_car_info(slot).config;
            config.hitbox_size.length() * 0.5 + config.hitbox_pos_offset.length()
        };
        let state = self.arena.get_car_state(slot);
        let reach = radius(slot) + state.phys.vel.length().max(v1.length()) * seconds + MARGIN;
        let others: Vec<usize> = (0..self.players.len())
            .filter(|&other| {
                let o = self.arena.get_car_state(other);
                other != slot
                    && !o.is_demoed
                    && (o.phys.pos - state.phys.pos).length()
                        < reach + radius(other) + (o.phys.vel.length() + 1100.0 * seconds) * seconds
            })
            .collect();
        let ball = self.arena.get_ball_state().phys;
        let ball_reach = BALL_RADIUS + (ball.vel.length() + 650.0 * seconds) * seconds;
        let near_ball = (state.phys.pos - ball.pos).length() < reach + ball_reach;
        if others.is_empty() && !near_ball {
            return false;
        }
        let config = self.arena.get_car_info(slot).config;
        let touches = |arena: &Arena| {
            let car = arena.get_car_state(slot).phys;
            let local = car.rot_mat.transpose() * (arena.get_ball_state().phys.pos - car.pos)
                - config.hitbox_pos_offset;
            let q = local.abs() - config.hitbox_size * 0.5;
            let ball_gap =
                q.max(glam::Vec3A::ZERO).length() + q.max_element().min(0.0) - BALL_RADIUS;
            (near_ball && ball_gap < 15.0)
                || others.iter().any(|&other| {
                    (arena.get_car_state(other).phys.pos - car.pos).length()
                        < radius(slot) + radius(other) + MARGIN
                })
        };
        let mut copy = self.arena.clone();
        for _ in 0..ticks {
            if touches(&copy) {
                return true;
            }
            copy.step_tick();
        }
        touches(&copy)
    }

    /// When an airborne car's boost started or stopped in this interval. The network boost switches in the air only
    /// at the car's update; the update's velocity shows when it did: from the interval's start to the update, the
    /// forward velocity changes by the boost (991.7 UU/s^2, with the air throttle at 1, 66.7) on the boosting ticks
    /// and by the air throttle on the others, beside gravity. The forward direction is the mean of the car's at the
    /// start and at the update. A car that can touch the ball or another car in the interval (`may_touch`), whose
    /// update holds the contact too, or near the speed limit (2,200 UU/s), is timed by simulation instead
    /// (`switch_by_simulation`). Not for a car with a wheel on a surface or without boost.
    fn plan_air_boost(
        &mut self,
        cars: &[&'a NetworkCar],
        f: usize,
        span: u64,
        gap: u64,
        switches: &[(u64, &'a NetworkCar)],
    ) {
        const BOOST: f32 = 2975.0 / 3.0;
        const AIR_THROTTLE: f32 = 200.0 / 3.0;
        let start = self.arena.tick_count();
        let frame = FrameIndex(f as u32);
        for &car in cars {
            let Some(&(player, created)) = self.players.by_actor.get(&car.life.actor) else {
                continue;
            };
            if created != car.life.created {
                continue;
            }
            if self.leads(car.life) {
                continue;
            }
            // A ground schedule writes the throttle and boost on every tick: a one-tick switch would not hold.
            if self
                .ground_schedules
                .iter()
                .any(|(p, s)| *p == player && s.end_tick > start && !s.jump_only)
            {
                continue;
            }
            let slot = player.get();
            let state = *self.arena.get_car_state(slot);
            let controls = *self.arena.get_car_controls(slot);
            let Some(counter) = car
                .inputs
                .boost_active_raw
                .as_ref()
                .filter(|v| v.frame == frame)
            else {
                continue;
            };
            let boost = counter.value % 2 == 1;
            let (Some(velocity), Some(rotation)) = (
                car.body
                    .linear_velocity
                    .as_ref()
                    .filter(|v| v.frame == frame),
                car.body.rotation.as_ref().filter(|v| v.frame == frame),
            ) else {
                continue;
            };
            let v1 = glam::Vec3A::from(velocity.value);
            if boost == controls.boost
                || state.is_on_ground()
                || state.wheels_with_contact().iter().any(Option::is_some)
                || state.is_demoed
                || state.boost <= 1.0
            {
                continue;
            }
            let update = span - self.car_ticks(car, f, true, gap).min(span);
            if update < 2 {
                continue;
            }
            // Near the speed limit RocketSim scales the velocity back to 2,300 UU/s, and the boost shows only in
            // its direction: simulated too.
            let near_limit = state.phys.vel.length() > 2200.0 || v1.length() > 2200.0;
            if near_limit || self.may_touch(slot, v1, update) {
                // A press planned for the car would be missing from the simulation.
                if self.plans_dodge(player) || self.early_presses.iter().any(|p| p.player == player)
                {
                    continue;
                }
                let Some(position) = car.body.position.as_ref().filter(|p| p.frame == frame) else {
                    continue;
                };
                let position = glam::Vec3A::from(position.value);
                let target = UpdateTarget::new(update, position, v1, car, switches);
                let switched =
                    |c: &mut CarControls, step: u64, k: u64| c.boost = boost == (step > k);
                if let Some(k) =
                    self.switch_by_simulation(slot, &target, 0..update, update, switched)
                {
                    self.air_boost.push((start + k + 1, slot, boost));
                    self.diagnostics.air_boost_switches += 1;
                }
                continue;
            }
            let [x, y, z, w] = rotation.value;
            let end_forward =
                glam::Mat3A::from_quat(glam::Quat::from_xyzw(x, y, z, w).normalize()).x_axis;
            let forward = (state.phys.rot_mat.x_axis + end_forward).normalize_or_zero();
            let n = update as f32;
            let seconds = n / 120.0;
            let change = v1 - state.phys.vel - glam::Vec3A::new(0.0, 0.0, -650.0) * seconds;
            let throttle = controls.throttle.clamp(-1.0, 1.0);
            // Forward change (UU/s) = (k * (BOOST + AIR_THROTTLE) + (n - k) * throttle * AIR_THROTTLE) / 120.
            let per_tick = BOOST + AIR_THROTTLE - throttle * AIR_THROTTLE;
            let k = ((change.dot(forward) * 120.0 - n * throttle * AIR_THROTTLE) / per_tick)
                .round()
                .clamp(0.0, n) as u64;
            // Boost on for the last `k` ticks before the update, or off after the first `k` (at least RocketSim's
            // 0.1 s once begun, which it enforces itself).
            let switch = if boost { update - k } else { k };
            if switch >= update {
                continue;
            }
            self.air_boost.push((start + switch + 1, slot, boost));
            self.diagnostics.air_boost_switches += 1;
        }
    }

    /// When an airborne car's throttle changed in this interval. In the air the throttle switches only at the car's
    /// update; without boost the throttle pushes the car forward by 66.7 UU/s^2 times its value, so the forward
    /// velocity change to the update, beside gravity, shows how many ticks had the old value. Not for a car with a
    /// wheel on a surface or that can touch the ball or another car in the interval (`may_touch`: the throttle's
    /// force is too small to time beside a contact), near the speed limit, boosting or switching its boost, with a
    /// planned press, flipping, or for a change of less than 0.5.
    fn plan_air_throttle(&mut self, cars: &[&'a NetworkCar], f: usize, span: u64, gap: u64) {
        const AIR_THROTTLE: f32 = 200.0 / 3.0;
        let start = self.arena.tick_count();
        let frame = FrameIndex(f as u32);
        for &car in cars {
            let Some(&(player, created)) = self.players.by_actor.get(&car.life.actor) else {
                continue;
            };
            if created != car.life.created {
                continue;
            }
            if self.leads(car.life) {
                continue;
            }
            // A ground schedule writes the throttle and boost on every tick: a one-tick switch would not hold.
            if self
                .ground_schedules
                .iter()
                .any(|(p, s)| *p == player && s.end_tick > start && !s.jump_only)
            {
                continue;
            }
            let slot = player.get();
            let state = *self.arena.get_car_state(slot);
            let controls = *self.arena.get_car_controls(slot);
            let (Some(velocity), Some(rotation)) = (
                car.body
                    .linear_velocity
                    .as_ref()
                    .filter(|v| v.frame == frame),
                car.body.rotation.as_ref().filter(|v| v.frame == frame),
            ) else {
                continue;
            };
            let next = updates::network_controls(car);
            let old = controls.throttle.clamp(-1.0, 1.0);
            let new = next.throttle.clamp(-1.0, 1.0);
            let v1 = glam::Vec3A::from(velocity.value);
            if (new - old).abs() < 0.5
                || next.boost
                || controls.boost
                || state.is_boosting
                || state.is_flipping
                || state.is_on_ground()
                || state.wheels_with_contact().iter().any(Option::is_some)
                || state.is_demoed
                || state.phys.vel.length() > 2200.0
                || v1.length() > 2200.0
                || self.early_presses.iter().any(|p| p.player == player)
                || self.air_boost.iter().any(|b| b.1 == slot)
            {
                continue;
            }
            let update = span - self.car_ticks(car, f, true, gap).min(span);
            if update < 2 || self.may_touch(slot, v1, update) {
                continue;
            }
            let [x, y, z, w] = rotation.value;
            let end_forward =
                glam::Mat3A::from_quat(glam::Quat::from_xyzw(x, y, z, w).normalize()).x_axis;
            let forward = (state.phys.rot_mat.x_axis + end_forward).normalize_or_zero();
            let n = update as f32;
            let change = v1 - state.phys.vel - glam::Vec3A::new(0.0, 0.0, -650.0) * n / 120.0;
            // Forward change (UU/s) = (k * old + (n - k) * new) * AIR_THROTTLE / 120.
            let k = ((change.dot(forward) * 120.0 / AIR_THROTTLE - n * new) / (old - new))
                .round()
                .clamp(0.0, n) as u64;
            if k >= update {
                continue;
            }
            self.air_throttle.push((start + k + 1, slot, next.throttle));
            self.diagnostics.air_throttle_switches += 1;
        }
    }

    /// When an airborne car's held jump was released in this interval. The jump counter turns even when the jump
    /// ends (released, or at RocketSim's 24-tick limit), and the release was applied at the car's update; the
    /// update's velocity shows how many of the interval's ticks the jump force (1,458.3 UU/s^2 along the car's up
    /// axis) still pushed: the velocity along that axis (the mean of the car's at the interval's start and at the
    /// update) changes by it beside gravity; boost and air throttle push along the forward axis, across it. A car
    /// with a wheel on a surface at the start (a jump a few ticks old, whose suspension still pushes) or that can
    /// touch the ball or another car in the interval (`may_touch`) is timed by simulation instead
    /// (`switch_by_simulation`). A release at the limit, or one the evidence puts at the update, is left as it
    /// was.
    fn plan_jump_release(
        &mut self,
        cars: &[&'a NetworkCar],
        f: usize,
        span: u64,
        gap: u64,
        switches: &[(u64, &'a NetworkCar)],
    ) {
        const HOLD: f32 = 4375.0 / 3.0;
        const MIN_TICKS: u32 = 3;
        const MAX_TICKS: u32 = 24;
        let start = self.arena.tick_count();
        let frame = FrameIndex(f as u32);
        for &car in cars {
            let Some(&(player, created)) = self.players.by_actor.get(&car.life.actor) else {
                continue;
            };
            if created != car.life.created {
                continue;
            }
            let slot = player.get();
            let state = *self.arena.get_car_state(slot);
            let controls = *self.arena.get_car_controls(slot);
            let released = car
                .inputs
                .jump_active_raw
                .as_ref()
                .is_some_and(|v| v.frame == frame && v.value % 2 == 0);
            let (Some(velocity), Some(rotation)) = (
                car.body
                    .linear_velocity
                    .as_ref()
                    .filter(|v| v.frame == frame),
                car.body.rotation.as_ref().filter(|v| v.frame == frame),
            ) else {
                continue;
            };
            if !released || !controls.jump || !state.is_jumping || state.is_demoed {
                continue;
            }
            // A car showing its controls ahead of their effect: `plan_lead_releases`.
            if self.leads(car.life) {
                continue;
            }
            // A ground schedule that sets the jump writes it on every tick: the release would be pressed again.
            if self.ground_schedules.iter().any(|(p, s)| {
                *p == player
                    && s.end_tick > start
                    && (s.jump_only || s.entries.iter().any(|e| e.5.is_some()))
            }) {
                continue;
            }
            let update = span - self.car_ticks(car, f, true, gap).min(span);
            // The ticks the jump can still push before RocketSim's limit, and the ones it must.
            let left = u64::from(MAX_TICKS.saturating_sub(state.jump_ticks));
            let least = u64::from(MIN_TICKS.saturating_sub(state.jump_ticks));
            if update < 1 || left == 0 {
                continue;
            }
            let v1 = glam::Vec3A::from(velocity.value);
            let on_surface =
                state.is_on_ground() || state.wheels_with_contact().iter().any(Option::is_some);
            if on_surface || self.may_touch(slot, v1, update) {
                // A fit's schedule drives the jump itself; a press or boost switch planned for the car would be
                // missing from the simulation.
                if self.ground_schedules.iter().any(|(p, _)| *p == player)
                    || self.plans_dodge(player)
                    || self.early_presses.iter().any(|p| p.player == player)
                    || self.air_boost.iter().any(|b| b.1 == slot)
                {
                    continue;
                }
                let Some(position) = car.body.position.as_ref().filter(|p| p.frame == frame) else {
                    continue;
                };
                let position = glam::Vec3A::from(position.value);
                let target = UpdateTarget::new(update, position, v1, car, switches);
                let last = update.min(left);
                let held = |c: &mut CarControls, step: u64, k: u64| c.jump = step <= k;
                if let Some(k) = self.switch_by_simulation(slot, &target, least..last, update, held)
                {
                    self.jump_releases.push((start + k + 1, slot));
                    self.diagnostics.jump_releases_timed += 1;
                }
                continue;
            }
            let [x, y, z, w] = rotation.value;
            let end_up =
                glam::Mat3A::from_quat(glam::Quat::from_xyzw(x, y, z, w).normalize()).z_axis;
            let up = (state.phys.rot_mat.z_axis + end_up).normalize_or_zero();
            // The jump must push past this update: no release in the interval (`clamp` needs `least <= n`).
            if least >= update {
                continue;
            }
            let n = update as f32;
            let change = (glam::Vec3A::from(velocity.value) - state.phys.vel).dot(up);
            // Along `up`: change (UU/s) = (k * HOLD - n * 650 * up.z) / 120.
            let k = ((change * 120.0 + n * 650.0 * up.z) / HOLD)
                .round()
                .clamp(least as f32, n) as u64;
            if k >= update || k >= left {
                continue;
            }
            self.jump_releases.push((start + k + 1, slot));
            self.diagnostics.jump_releases_timed += 1;
        }
    }

    /// The step `k` of `candidates` (a control's change after step `k`, or a press on step `k + 1`) whose
    /// simulation to the car's update comes closest to it (position error plus 0.1 x velocity error), when it is
    /// closer than `default`'s and below 10. `set(controls, step, k)` sets the control on each step. Steps an arena
    /// copy, the car's controls held as at the start but for its network switches (in the air only the steer, as in
    /// `advance_to`) and its air schedule's pitch, yaw and roll; the other cars hold theirs.
    fn switch_by_simulation(
        &self,
        slot: usize,
        target: &UpdateTarget,
        candidates: std::ops::Range<u64>,
        default: u64,
        set: impl Fn(&mut CarControls, u64, u64),
    ) -> Option<u64> {
        let (k, cost) = self.best_by_simulation(slot, target, candidates, default, set);
        // A best change that still misses the update by 10 or more (an update that puts the car back at rest on
        // the ground, say) is no evidence of when it happened.
        (k != default && cost < 10.0).then_some(k)
    }

    /// The step of `candidates` or `default` whose simulation comes closest to the update, and its cost
    /// (`switch_by_simulation`); `default` unless another is strictly better.
    fn best_by_simulation(
        &self,
        slot: usize,
        target: &UpdateTarget,
        candidates: std::ops::Range<u64>,
        default: u64,
        set: impl Fn(&mut CarControls, u64, u64),
    ) -> (u64, f32) {
        let start = self.arena.tick_count();
        let update = target.update;
        // The last schedule wins, as in `step`.
        let schedule = self
            .air_schedules
            .iter()
            .rev()
            .find(|s| s.player.get() == slot && s.end_tick > start);
        let cost = |k: u64| {
            let mut copy = self.arena.clone();
            for step in 1..=update {
                let mut controls = *copy.get_car_controls(slot);
                for &(_, next) in target.switches.iter().filter(|(at, _)| step == at + 1) {
                    controls.steer = next.steer;
                    if copy.get_car_state(slot).is_on_ground() {
                        controls.throttle = next.throttle;
                        controls.handbrake = next.handbrake;
                        controls.boost = next.boost;
                    } else if target.leads {
                        controls.throttle = next.throttle;
                        controls.boost = next.boost;
                        if self
                            .control_sources
                            .get(slot)
                            .is_some_and(|s| s.0 == AirControlSource::Steer)
                        {
                            (controls.yaw, controls.roll) = if next.handbrake {
                                (0.0, next.steer)
                            } else {
                                (next.steer, 0.0)
                            };
                        }
                        controls.handbrake = next.handbrake;
                    }
                }
                if let Some(schedule) = schedule
                    && schedule.end_tick >= start + step
                    && let Some(&(_, air)) =
                        schedule.entries.iter().rev().find(|e| e.0 <= start + step)
                {
                    controls.pitch = air.pitch;
                    controls.yaw = air.yaw;
                    controls.roll = air.roll;
                }
                set(&mut controls, step, k);
                copy.set_car_controls(slot, controls);
                copy.step_tick();
            }
            let phys = copy.get_car_state(slot).phys;
            (phys.pos - target.position).length() + 0.1 * (phys.vel - target.velocity).length()
        };
        let mut best = (default, cost(default));
        for k in candidates.filter(|&k| k != default) {
            let c = cost(k);
            if c < best.1 - 1e-4 {
                best = (k, c);
            }
        }
        best
    }

    /// When a car that shows its controls ahead of their effect (`leads`) releases a held jump: its control shift
    /// after the midpoint-rule switch of the frame whose jump counter turned even, held across the updates until
    /// then (`lead_releases`). RocketSim ends the jump's force at its 24-tick limit itself.
    fn plan_lead_releases(&mut self, cars: &[&'a NetworkCar], f: usize, gap: u64) {
        let start = self.arena.tick_count();
        let frame = FrameIndex(f as u32);
        for &car in cars {
            let Some(&(player, created)) = self.players.by_actor.get(&car.life.actor) else {
                continue;
            };
            if created != car.life.created || !self.leads(car.life) {
                continue;
            }
            let slot = player.get();
            let state = self.arena.get_car_state(slot);
            let released = car
                .inputs
                .jump_active_raw
                .as_ref()
                .is_some_and(|v| v.frame == frame && v.value % 2 == 0);
            if !released
                || !self.arena.get_car_controls(slot).jump
                || !state.is_jumping
                || state.is_demoed
            {
                continue;
            }
            let Some(shift) = self.inference.control_shift(car.life) else {
                continue;
            };
            let at = (gap / 2).saturating_sub(2) + shift as u64;
            self.lead_releases.retain(|&(_, s)| s != slot);
            self.lead_releases.push((start + at + 1, slot));
        }
    }

    /// Releases the held jumps of leading cars due at `sim_tick` (`lead_releases`).
    fn release_lead_jumps(&mut self, sim_tick: u64) {
        for i in 0..self.lead_releases.len() {
            let (tick, slot) = self.lead_releases[i];
            if sim_tick == tick {
                let mut controls = *self.arena.get_car_controls(slot);
                if controls.jump && self.arena.get_car_state(slot).prev_controls().jump {
                    controls.jump = false;
                    self.arena.set_car_controls(slot, controls);
                }
            }
        }
        self.lead_releases.retain(|&(tick, _)| tick > sim_tick);
    }

    /// Releases the jumps due at `sim_tick` (`plan_jump_release`), unless the jump was already released: a jump
    /// down now is then a new press (a dodge's).
    fn release_jumps(&mut self, sim_tick: u64) {
        for i in 0..self.jump_releases.len() {
            let (tick, slot) = self.jump_releases[i];
            if sim_tick == tick {
                let mut controls = *self.arena.get_car_controls(slot);
                if controls.jump && self.arena.get_car_state(slot).prev_controls().jump {
                    controls.jump = false;
                    self.arena.set_car_controls(slot, controls);
                }
            }
        }
    }

    /// Applies the throttle switches in the air due at `sim_tick` (`plan_air_throttle`).
    fn switch_air_throttle(&mut self, sim_tick: u64) {
        for i in 0..self.air_throttle.len() {
            let (tick, slot, throttle) = self.air_throttle[i];
            if tick == sim_tick && !self.arena.get_car_state(slot).is_on_ground() {
                let mut controls = *self.arena.get_car_controls(slot);
                controls.throttle = throttle;
                self.arena.set_car_controls(slot, controls);
            }
        }
    }

    /// Applies the boost switches in the air due at `sim_tick` (`plan_air_boost`).
    fn switch_air_boost(&mut self, sim_tick: u64) {
        for i in 0..self.air_boost.len() {
            let (tick, slot, boost) = self.air_boost[i];
            if tick == sim_tick && !self.arena.get_car_state(slot).is_on_ground() {
                let mut controls = *self.arena.get_car_controls(slot);
                controls.boost = boost;
                self.arena.set_car_controls(slot, controls);
            }
        }
    }

    /// Applies the early presses due at `sim_tick`: the press (released the tick before), and a jump held from
    /// its press to the update.
    fn press_early(&mut self, sim_tick: u64) {
        for i in 0..self.early_presses.len() {
            let press = self.early_presses[i];
            let slot = press.player.get();
            let state = *self.arena.get_car_state(slot);
            let mut controls = *self.arena.get_car_controls(slot);
            if sim_tick + 1 == press.tick {
                controls.jump = false;
                self.arena.set_car_controls(slot, controls);
            } else if sim_tick == press.tick {
                let ready = match press.kind {
                    EarlyKind::Jump => state.is_on_ground() && !state.has_jumped,
                    EarlyKind::DoubleJump => {
                        !state.is_on_ground() && !state.has_double_jumped && !state.has_flipped
                    }
                    EarlyKind::Dodge { .. } => !state.is_on_ground() && !state.has_flipped,
                };
                if ready && press.kind != EarlyKind::Jump && state.prev_controls().jump {
                    // The jump is still held: release now, press a tick later.
                    controls.jump = false;
                    self.arena.set_car_controls(slot, controls);
                    self.early_presses[i].tick += 1;
                    continue;
                }
                if !ready {
                    continue;
                }
                controls.jump = true;
                if let EarlyKind::Dodge { pitch, yaw } = press.kind {
                    controls.pitch = pitch;
                    controls.yaw = yaw;
                    // The dodge direction is (-pitch, yaw + roll): a roll left over would turn it.
                    controls.roll = 0.0;
                } else if press.kind == EarlyKind::DoubleJump {
                    // A jump press with no direction is RocketSim's double jump.
                    (controls.pitch, controls.yaw, controls.roll) = (0.0, 0.0, 0.0);
                }
                self.arena.set_car_controls(slot, controls);
                sources(&mut self.control_sources, slot).0 = AirControlSource::Press;
                self.early_presses[i].applied = true;
                if press.kind != EarlyKind::Jump {
                    self.action_presses.insert(slot);
                }
            } else if press.applied
                && press.kind == EarlyKind::Jump
                && sim_tick > press.tick
                && sim_tick <= press.update_tick
            {
                controls.jump = true;
                self.arena.set_car_controls(slot, controls);
            } else if press.applied && sim_tick == press.tick + 1 && press.kind != EarlyKind::Jump {
                controls.jump = false;
                self.arena.set_car_controls(slot, controls);
            }
        }
    }

    /// Presses the deferred dodges whose car is airborne and has not flipped (released the tick before when jump is
    /// still held); drops those past their time.
    fn press_deferred(&mut self, sim_tick: u64) {
        self.deferred_dodges.retain(|d| d.until >= sim_tick);
        let mut done = Vec::new();
        for (i, dodge) in self.deferred_dodges.iter().enumerate() {
            if sim_tick < dodge.from {
                continue;
            }
            let slot = dodge.player.get();
            let mut state = *self.arena.get_car_state(slot);
            if state.is_on_ground() || state.has_flipped || state.is_demoed {
                continue;
            }
            let mut controls = *self.arena.get_car_controls(slot);
            if state.prev_controls().jump {
                controls.jump = false;
                self.arena.set_car_controls(slot, controls);
                continue;
            }
            if !state.has_jumped {
                // Airborne without a jump the simulation saw (its update came after the counter): the jump happened.
                state.has_jumped = true;
                self.arena.set_car_state_full(slot, state);
            }
            controls.jump = true;
            controls.pitch = dodge.pitch;
            controls.yaw = dodge.yaw;
            // The dodge direction is (-pitch, yaw + roll): a roll left over would turn it.
            controls.roll = 0.0;
            self.arena.set_car_controls(slot, controls);
            sources(&mut self.control_sources, slot).0 = AirControlSource::Press;
            self.action_presses.insert(slot);
            done.push(i);
        }
        for i in done.into_iter().rev() {
            self.deferred_dodges.remove(i);
        }
    }

    /// Records the current tick's state with the controls about to be applied.
    fn record_tick(&mut self) {
        let slots = self.players.len();
        let cars = (0..slots)
            .map(|slot| {
                let mut car = *self.arena.get_car_state(slot);
                car.controls = *self.arena.get_car_controls(slot);
                if u8::try_from(slot)
                    .is_ok_and(|p| self.holds.spawning.contains_key(&PlayerIndex(p)))
                {
                    car.is_demoed = false;
                    car.demo_respawn_timer = 0.0;
                }
                car
            })
            .collect();
        let control_sources = (0..slots)
            .map(|slot| *sources(&mut self.control_sources, slot))
            .collect();
        self.stepped.push(TickRecord {
            sim_tick: self.arena.tick_count(),
            ball: *self.arena.get_ball_state(),
            cars,
            control_sources,
        });
    }

    /// The ground schedules' controls at `sim_tick`; from its press tick a pending dodge drives the car.
    fn drive_ground_schedules(&mut self, sim_tick: u64) {
        for (player, schedule) in &self.ground_schedules {
            if self
                .pending_dodges
                .iter()
                .any(|dodge| dodge.player == *player && sim_tick >= dodge.start_tick)
            {
                continue;
            }
            let Some(entry) = schedule.entries.iter().rev().find(|e| e.0 <= sim_tick) else {
                continue;
            };
            let slot = player.get();
            let mut controls = *self.arena.get_car_controls(slot);
            if schedule.jump_only {
                controls.jump = entry.5.unwrap_or(controls.jump);
                self.arena.set_car_controls(slot, controls);
                continue;
            }
            controls.throttle = entry.1;
            controls.steer = entry.2;
            controls.handbrake = entry.3;
            controls.boost = entry.4;
            if let Some(jump) = entry.5 {
                controls.jump = jump;
            }
            self.arena.set_car_controls(slot, controls);
            sources(&mut self.control_sources, slot).1 = GroundControlSource::Schedule;
        }
    }

    /// The pending dodges at `sim_tick`: jump released before the press (so the press is a new edge), jump with
    /// the direction on the press tick, then the pitch cancel. An air schedule solved around the dodge owns the
    /// controls except on the press tick.
    fn press_dodges(&mut self, sim_tick: u64) {
        let mut deferred = Vec::new();
        let mut pressed = Vec::new();
        // A press while the jump is still held (a takeoff jump held into the dodge's first tick) would not be a
        // press: release now and press a tick later.
        for dodge in &mut self.pending_dodges {
            if sim_tick == dodge.start_tick
                && self
                    .arena
                    .get_car_state(dodge.player.get())
                    .prev_controls()
                    .jump
                && !self.arena.get_car_state(dodge.player.get()).is_on_ground()
            {
                dodge.start_tick += 1;
                dodge.end_tick = dodge.end_tick.max(dodge.start_tick);
                let mut controls = *self.arena.get_car_controls(dodge.player.get());
                controls.jump = false;
                self.arena.set_car_controls(dodge.player.get(), controls);
            }
        }
        for dodge in &self.pending_dodges {
            let slot = dodge.player.get();
            if sim_tick == dodge.start_tick && self.arena.get_car_state(slot).is_on_ground() {
                // The car is not yet airborne (its takeoff can reach the updates after the counters): pressing now
                // would jump, not dodge. The dodge waits for the car to leave the ground.
                deferred.push(DeferredDodge {
                    player: dodge.player,
                    life: dodge.life,
                    pitch: dodge.pitch,
                    yaw: dodge.yaw,
                    from: 0,
                    until: sim_tick + 60,
                });
                continue;
            }
            if sim_tick != dodge.start_tick
                && self.air_schedules.iter().any(|s| s.player == dodge.player)
            {
                if sim_tick < dodge.start_tick {
                    let mut current = *self.arena.get_car_controls(slot);
                    current.jump = false;
                    self.arena.set_car_controls(slot, current);
                }
                continue;
            }
            // The air controls from the plan; throttle, steer, boost and handbrake as the car's switches have them
            // (a boost switch timed inside the plan).
            let current = *self.arena.get_car_controls(slot);
            let mut controls = CarControls {
                throttle: current.throttle,
                steer: current.steer,
                boost: current.boost,
                handbrake: current.handbrake,
                ..dodge.base
            };
            controls.jump = false;
            if sim_tick < dodge.start_tick {
                // A ground schedule of the same car sets the jump input itself.
                if !self
                    .ground_schedules
                    .iter()
                    .any(|(p, _)| *p == dodge.player)
                {
                    self.arena.set_car_controls(slot, controls);
                    *sources(&mut self.control_sources, slot) =
                        (AirControlSource::Dodge, GroundControlSource::Dodge);
                }
            } else if sim_tick == dodge.start_tick {
                controls.jump = true;
                controls.pitch = dodge.pitch;
                controls.yaw = dodge.yaw;
                // The dodge direction is (-pitch, yaw + roll): a roll left in `base` would turn it.
                controls.roll = 0.0;
                self.arena.set_car_controls(slot, controls);
                pressed.push(slot);
                *sources(&mut self.control_sources, slot) =
                    (AirControlSource::Dodge, GroundControlSource::Dodge);
            } else {
                let sign = self.arena.get_car_state(slot).flip_rel_torque.y.signum();
                controls.pitch = dodge.cancel * sign;
                self.arena.set_car_controls(slot, controls);
                *sources(&mut self.control_sources, slot) =
                    (AirControlSource::Dodge, GroundControlSource::Dodge);
            }
        }
        self.deferred_dodges.extend(deferred);
        self.action_presses.extend(pressed);
    }

    fn update_ball(&mut self, ctx: &mut FrameContext, body: &crate::decode::NetworkBody) {
        let mut ball = *self.arena.get_ball_state();
        if ctx.simulated && self.ball_initialized {
            ctx.predictions.push(Prediction {
                car: None,
                phys: ball.phys,
                is_on_ground: None,
            });
        }
        let mut applied =
            updates::apply_update(&mut ball.phys, body, ctx.index, !self.ball_initialized);
        if let Some(changed) = updates::zero_sleeping_velocity(&mut ball.phys, body, ctx.index) {
            applied |= changed;
            ctx.sleeping_velocity_zeroed.push(None);
            self.diagnostics.sleeping_ball_updates += 1;
        }
        if applied {
            self.arena.set_ball_state(ball);
        }
        self.ball_initialized = true;
    }

    fn output(
        &mut self,
        frame: &NetworkFrame,
        ctx: FrameContext,
        replay_tick: u64,
        events: Vec<SimEvent>,
        applied_ticks: Vec<AppliedTick>,
    ) -> SimulatedFrame {
        // Sim ticks to the replay timeline: the offset at the end of this frame.
        let offset = replay_tick as i64 - self.arena.tick_count() as i64;
        let fitted = ctx
            .fitted
            .iter()
            .map(|&(player, tick, kind)| FittedInput {
                player,
                replay_tick: (tick as i64 + offset).max(0) as u64,
                kind,
            })
            .collect();
        let mut state = self.arena.get_arena_state();
        // A car held out of collisions on its spawn pose is not demolished.
        for (info, car) in state.cars.iter_mut() {
            if u8::try_from(info.idx)
                .is_ok_and(|p| self.holds.spawning.contains_key(&PlayerIndex(p)))
            {
                car.is_demoed = false;
                car.demo_respawn_timer = 0.0;
            }
        }
        let lives = frame
            .current_cars()
            .into_iter()
            .chain(frame.cars.iter().filter(|car| car.player.is_none()))
            .filter_map(|car| match self.players.by_actor.get(&car.life.actor) {
                Some(&(player, created)) if created == car.life.created => Some((player, created)),
                _ => None,
            })
            .collect();
        let pickups = frame
            .pad_records
            .iter()
            .filter(|r| !r.repeat && r.picked_up_raw != 255)
            .filter_map(|r| {
                let instigator = r.instigator_car?;
                Some(PickupMatch {
                    pad: r.pad,
                    pad_index: self.pads.index_of(r.pad),
                    player: self
                        .players
                        .by_actor
                        .get(&instigator)
                        .map(|&(player, _)| player),
                })
            })
            .collect();
        let car_players = frame
            .cars
            .iter()
            .filter_map(|car| Some((car.life.actor, self.players.index_of(car.player.as_ref()?)?)))
            .collect();
        let mut updated_players: Vec<PlayerIndex> = frame
            .current_cars()
            .into_iter()
            .filter(|car| {
                car.body
                    .position
                    .as_ref()
                    .is_some_and(|p| p.frame == frame.index)
            })
            .filter_map(|car| self.players.index_of(car.player.as_ref()?))
            .collect();
        updated_players.sort_unstable();
        updated_players.dedup();
        SimulatedFrame {
            index: frame.index,
            replay_tick,
            state,
            events,
            applied_ticks,
            sleeping_velocity_zeroed: ctx.sleeping_velocity_zeroed,
            wrecks_inferred: ctx.wrecks_inferred,
            wrecks_held: self
                .holds
                .wrecks
                .iter()
                .map(|(&player, &(_, source))| (player, source))
                .collect(),
            spawning: self.holds.spawning.keys().copied().collect(),
            car_players,
            ball_updated: frame
                .ball
                .as_ref()
                .and_then(|b| b.position.as_ref())
                .is_some_and(|p| p.frame == frame.index),
            updated_players,
            fitted,
            lives,
            pickups,
            predictions: ctx.predictions,
            control_sources: self.control_sources.clone(),
            ticks: std::mem::take(&mut self.stepped),
        }
    }
}
