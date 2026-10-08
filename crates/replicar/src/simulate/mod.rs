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
use rocketsim::{Arena, ArenaConfig, ArenaEvent, ArenaState, DemoMode, GameMode};

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
}

impl Default for SimulationOptions {
    fn default() -> Self {
        Self {
            seed: 0,
            loadout_hitboxes: true,
            simulated_pad_pickups: false,
            simulated_demolitions: false,
            withheld: None,
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
/// carries the other controls.
#[derive(Debug, Clone, Copy)]
struct PendingDodge {
    player: PlayerIndex,
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
/// updates, which can lag the counters by frames): pressed at the first tick the car is airborne, unless the
/// counter turns even or `until` passes first.
#[derive(Debug, Clone, Copy)]
pub(super) struct DeferredDodge {
    pub(super) player: PlayerIndex,
    pub(super) life: CarLife,
    pub(super) pitch: f32,
    pub(super) yaw: f32,
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
    /// Dodges waiting for their car to leave the ground (`DeferredDodge`).
    pub(super) deferred_dodges: Vec<DeferredDodge>,
    /// The ticks stepped in the current frame's interval.
    stepped: Vec<TickRecord>,
    /// The interval's cars whose throttle and steer ramp across a change (by player slot), the replay tick of sim
    /// tick 0 (replay tick = sim tick + offset), and the frame.
    ramp_cars: Vec<(usize, &'a NetworkCar)>,
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
        deferred_dodges: Vec::new(),
        stepped: Vec::new(),
        ramp_cars: Vec::new(),
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
        let gap = replay_tick - self.previous_tick;
        let state = frame.game_state.as_ref().map(|s| &s.value);
        let active = state == Some(&GameState::Active);
        // The frame that reports a goal ends the play before it: the ball crossed the line in its interval, so the
        // interval is simulated like any other (it is the last of its play segment); play stops after it.
        let goal_frame =
            !active && self.previous_in_play && state == Some(&GameState::PostGoalScored);
        let in_play = active || goal_frame;
        let simulated = in_play && self.previous_in_play && gap > 0 && gap <= MAX_GAP_TICKS;
        self.previous_tick = replay_tick;
        self.previous_in_play = active;
        if !simulated {
            self.diagnostics.skipped_replay_ticks += gap;
        }
        if let (Some(ticks), true) = (self.ticks, simulated) {
            self.diagnostics.ball_tick_frames += usize::from(ticks.ball[f].is_some());
            self.diagnostics.car_tick_frames += usize::from(ticks.car_median[f].is_some());
        }
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
            switches: self.switches(&frame_cars, f, replay_tick, span, gap, withheld),
            next_switch: 0,
        };
        self.ramp_cars.clear();
        if simulated && !withheld && self.ticks.is_some() {
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
        if simulated && !withheld && self.ticks.is_some() {
            self.plan_early_presses(&frame_cars, f, span, gap);
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
            if !self.arena.get_car_state(player.get()).is_on_ground {
                controls.steer = next.steer;
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
                if controls.jump && !state.prev_controls.jump && !state.is_on_ground {
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
            self.press_dodges(sim_tick);
            self.press_early(sim_tick);
            self.press_deferred(sim_tick);
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
            let airborne = !self.arena.get_car_state(slot).is_on_ground;
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
    /// double jump or dodge at the midpoint rule, before the update. Cars a fit already plans are left to it.
    fn plan_early_presses(&mut self, cars: &[&'a NetworkCar], f: usize, span: u64, gap: u64) {
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
                || self.pending_dodges.iter().any(|d| d.player == player)
                || self.ground_schedules.iter().any(|(p, _)| *p == player)
                || self.inference.dodge_handled(car.life, f)
            {
                continue;
            }
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
                continue;
            }
            let state = self.arena.get_car_state(player.get());
            let airborne = !state.is_on_ground || state.phys.pos.z > 50.0;
            let rule = (gap / 2).saturating_sub(2).min(update - 1);
            let kind = if rose(
                &car.inputs.dodge_active_raw,
                &previous.inputs.dodge_active_raw,
            ) {
                let Some([tx, ty, _]) = updates::dodge_torque(frames, f, car) else {
                    continue;
                };
                let (pitch, yaw) = (-ty / 2.24, -tx / 2.60);
                if !airborne || (pitch * pitch + yaw * yaw).sqrt() <= 0.01 {
                    continue;
                }
                (EarlyKind::Dodge { pitch, yaw }, rule)
            } else if rose(
                &car.inputs.double_jump_active_raw,
                &previous.inputs.double_jump_active_raw,
            ) {
                if !airborne {
                    continue;
                }
                (EarlyKind::DoubleJump, rule)
            } else if rose(
                &car.inputs.jump_active_raw,
                &previous.inputs.jump_active_raw,
            ) {
                if !state.is_on_ground || velocity.value[2] <= 150.0 {
                    continue;
                }
                let since = ((velocity.value[2] - 291.667) / 808.333 * 120.0)
                    .round()
                    .clamp(1.0, 23.0) as u64;
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
                    EarlyKind::Jump => state.is_on_ground && !state.has_jumped,
                    EarlyKind::DoubleJump => {
                        !state.is_on_ground && !state.has_double_jumped && !state.has_flipped
                    }
                    EarlyKind::Dodge { .. } => !state.is_on_ground && !state.has_flipped,
                };
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
            let slot = dodge.player.get();
            let mut state = *self.arena.get_car_state(slot);
            if state.is_on_ground || state.has_flipped || state.is_demoed {
                continue;
            }
            let mut controls = *self.arena.get_car_controls(slot);
            if state.prev_controls.jump {
                controls.jump = false;
                self.arena.set_car_controls(slot, controls);
                continue;
            }
            if !state.has_jumped {
                // Airborne without a jump the simulation saw (its update came after the counter): the jump happened.
                state.has_jumped = true;
                self.arena.set_car_state(slot, state);
            }
            controls.jump = true;
            controls.pitch = dodge.pitch;
            controls.yaw = dodge.yaw;
            // The dodge direction is (-pitch, yaw + roll): a roll left over would turn it.
            controls.roll = 0.0;
            self.arena.set_car_controls(slot, controls);
            sources(&mut self.control_sources, slot).0 = AirControlSource::Press;
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
        for dodge in &self.pending_dodges {
            let slot = dodge.player.get();
            if sim_tick == dodge.start_tick && self.arena.get_car_state(slot).is_on_ground {
                // The car is not yet airborne (its takeoff can reach the updates after the counters): pressing now
                // would jump, not dodge. The dodge waits for the car to leave the ground.
                if let Some((actor, (_, created))) = self
                    .players
                    .by_actor
                    .iter()
                    .find(|(_, (player, _))| *player == dodge.player)
                {
                    deferred.push(DeferredDodge {
                        player: dodge.player,
                        life: CarLife {
                            actor: *actor,
                            created: *created,
                        },
                        pitch: dodge.pitch,
                        yaw: dodge.yaw,
                        until: sim_tick + 60,
                    });
                }
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
            let mut controls = dodge.base;
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
