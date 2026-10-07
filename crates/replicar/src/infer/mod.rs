//! The inference: what the replay does not say, chosen where the simulator needs it (docs/v2-plan.md,
//! section 4.2). The simulator asks; an `Inference` answers. `FittedInference` computes its answers, from
//! earlier and later updates.

mod air_schedule;
mod fits;
pub mod recorded;

use std::collections::{HashMap, HashSet};

use rocketsim::{BallState, CarControls, CarState};

pub use air_schedule::{AirSchedule, AirScheduleQuery};
pub use fits::{DodgePlan, GroundSchedule};

use replicar_format::AirControlSource;

use crate::air::{self, AirControls};
use crate::decode::{CarLife, NetworkCar, NetworkReplay};
use crate::hitbox::Hitbox;
use crate::update_ticks::{UpdateTicks, Withheld};
use fits::{FitContext, ScratchArenas};

/// Below this height (UU) a car's updates do not count as airborne for its air controls.
const AIR_MIN_Z: f32 = 50.0;

/// A pending dodge press, as the simulator holds it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PressInFlight {
    pub start_tick: u64,
    pub pitch: f32,
    pub yaw: f32,
}

/// What the simulator knows when it asks about a car's update.
pub struct FitQuery<'q> {
    /// The frame of the car's update.
    pub index: usize,
    pub car: &'q NetworkCar,
    /// The car's simulated state (see each question for when).
    pub state: &'q CarState,
    /// The controls the car drives on with at this point.
    pub controls: &'q CarControls,
    /// The update's ticks before its frame.
    pub ticks_before: u64,
    /// The current sim tick and the simulation's ball.
    pub now: u64,
    pub ball: BallState,
    /// The player's hitbox, for the scratch arenas.
    pub hitbox: Hitbox,
    pub in_play: bool,
    pub new_life: bool,
}

/// A ground schedule, and the dodge that follows a fitted ground jump.
#[derive(Debug, Clone, PartialEq)]
pub struct GroundChoice {
    pub schedule: GroundSchedule,
    pub dodge: Option<DodgePlan>,
}

/// What the simulator asks.
pub trait Inference {
    /// The pitch, yaw and roll of an airborne car for the interval that starts at frame `index`, given the
    /// network controls it drives on (`controls`), and how they were found (`Lookahead` or `Persisted`). `None`:
    /// no evidence, and the simulator uses the steer.
    fn air_controls(
        &mut self,
        index: usize,
        car: &NetworkCar,
        controls: &CarControls,
    ) -> Option<(AirControls, AirControlSource)>;

    /// The pitch of a flipping car (its flip cancel); `None` leaves the pitch as it is. Asked for every car
    /// update (the inference also learns here that a car stopped flipping).
    fn flip_pitch(&mut self, query: &FitQuery, airborne: bool, pressing: bool) -> Option<f32>;

    /// A dodge press for an airborne car whose dodge counter is about to turn odd.
    fn dodge_start(&mut self, query: &FitQuery) -> Option<DodgePlan>;

    /// The air controls of an airborne car tick by tick to its next update, with the shift of a flip's start
    /// the solution chose (for a flipping car or a pending press).
    fn air_schedule(
        &mut self,
        query: &AirScheduleQuery,
        press: Option<PressInFlight>,
    ) -> Option<(AirSchedule, i32)>;

    /// The ground controls of a car tick by tick to its next update, and a dodge after a fitted ground jump.
    fn ground_schedule(&mut self, query: &FitQuery, dodge_pending: bool) -> Option<GroundChoice>;

    /// The update tick a fit placed this car's update at frame `index` at (ticks before its frame).
    fn ticks_override(&self, life: CarLife, index: usize) -> Option<u64>;

    /// The dodge activated at frame `index` was already planned by a fit.
    fn dodge_handled(&self, life: CarLife, index: usize) -> bool;

    /// The shift, in ticks, by which the car's network controls take effect later than the midpoint rule.
    fn control_shift(&self, life: CarLife) -> Option<i64>;
}

/// What the inference counted.
#[derive(Debug, Clone, Default, PartialEq, Eq, serde::Serialize)]
pub struct InferenceDiagnostics {
    /// Airborne updates whose interval got an air schedule, and airborne ones refused one.
    pub air_schedules_planned: usize,
    pub air_schedules_refused: usize,
    /// Dodge starts fitted.
    pub dodge_starts_fitted: usize,
}

/// Which inferences `FittedInference` makes.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct InferenceOptions {
    /// Solve an airborne car's air controls over the span to its next update (offline).
    pub air_lookahead: bool,
    /// Solve an airborne car's air controls tick by tick to its next update's rotation and angular velocity
    /// (offline).
    pub air_schedules: bool,
    /// Fit when the network controls took effect, jump presses, dodge starts and flip cancels (offline).
    pub input_fits: bool,
    /// Fit the timings against the next update instead of the one after it (in sample there).
    pub fit_on_next_update: bool,
    /// Leave the first interval out of the flip-cancel fit when later updates exist (held out).
    pub flip_cancel_holdout: bool,
    /// Infer the tick of the first update after a dodge from the fitted path.
    pub dodge_first_update_tick: bool,
    /// The scratch arenas' random seed.
    pub seed: u64,
}

impl Default for InferenceOptions {
    fn default() -> Self {
        Self {
            air_lookahead: true,
            air_schedules: true,
            input_fits: true,
            fit_on_next_update: true,
            flip_cancel_holdout: false,
            dodge_first_update_tick: true,
            seed: 0,
        }
    }
}

/// The inference that computes its answers.
pub struct FittedInference<'a> {
    network: &'a NetworkReplay,
    ticks: Option<&'a UpdateTicks>,
    options: InferenceOptions,
    withheld: Withheld<'a>,
    scratch: ScratchArenas,
    /// The fitted cancel per car life and update frame (`None`: refused), and the last cancel per car life.
    flip_cancels: HashMap<(CarLife, usize), Option<f32>>,
    last_cancel: HashMap<CarLife, f32>,
    /// The informative ground timing shifts per car life.
    shifts: HashMap<CarLife, Vec<i64>>,
    /// Dodges planned by a fit, by car life and activation frame.
    handled: HashSet<(CarLife, usize)>,
    /// Update ticks a fit set, by car life and frame.
    overrides: HashMap<(CarLife, usize), u64>,
    pub diagnostics: InferenceDiagnostics,
}

impl<'a> FittedInference<'a> {
    /// `ticks` places the updates the fits reach for; without it the fits that need update ticks make no
    /// choice.
    #[must_use]
    pub fn new(
        network: &'a NetworkReplay,
        ticks: Option<&'a UpdateTicks>,
        options: InferenceOptions,
        withheld: Withheld<'a>,
    ) -> Self {
        Self {
            network,
            ticks,
            options,
            withheld,
            scratch: ScratchArenas::new(options.seed),
            flip_cancels: HashMap::new(),
            last_cancel: HashMap::new(),
            shifts: HashMap::new(),
            handled: HashSet::new(),
            overrides: HashMap::new(),
            diagnostics: InferenceDiagnostics::default(),
        }
    }

    fn fit_context(&self) -> FitContext<'a> {
        FitContext {
            frames: &self.network.frames,
            ticks: self.ticks,
            withheld: self.withheld,
            fit_on_next_update: self.options.fit_on_next_update,
            flip_cancel_holdout: self.options.flip_cancel_holdout,
            dodge_first_update_tick: self.options.dodge_first_update_tick,
        }
    }

    /// Records a fitted dodge: planned, its cancel the car's last, and the first update's tick.
    fn record_dodge(
        &mut self,
        life: CarLife,
        plan: &DodgePlan,
        first_update: Option<(usize, u64)>,
    ) {
        self.handled.insert((life, plan.activation_frame));
        if let Some((frame, ticks)) = first_update {
            self.overrides.insert((life, frame), ticks);
        }
        self.last_cancel.insert(life, plan.cancel);
        self.diagnostics.dodge_starts_fitted += 1;
    }
}

impl Inference for FittedInference<'_> {
    /// The constant control over the span to the next update (with `air_lookahead`), else the persistence of
    /// the control over the last span: per axis the calibrated median share of it, the steer standing in for
    /// yaw (or roll with the handbrake) when the replay has one.
    fn air_controls(
        &mut self,
        index: usize,
        car: &NetworkCar,
        controls: &CarControls,
    ) -> Option<(AirControls, AirControlSource)> {
        let frames = &self.network.frames;
        if self.options.air_lookahead
            && let Some(solved) =
                air::lookahead_controls(frames, index, car, AIR_MIN_Z, self.withheld)
        {
            return Some((solved, AirControlSource::Lookahead));
        }
        let (solved, lag) = air::past_controls(frames, index, car, AIR_MIN_Z)?;
        let keep = |axis: usize, value: f32| value * air::persistence(axis, lag, value.abs());
        let pitch = keep(0, solved.pitch);
        let controls = if car.inputs.steer.is_none() {
            AirControls {
                pitch,
                yaw: keep(1, solved.yaw),
                roll: keep(2, solved.roll),
            }
        } else if controls.handbrake {
            AirControls {
                pitch,
                yaw: keep(1, solved.yaw),
                roll: controls.steer,
            }
        } else {
            AirControls {
                pitch,
                yaw: controls.steer,
                roll: keep(2, solved.roll),
            }
        };
        Some((controls, AirControlSource::Persisted))
    }

    /// A flipping car's cancel is fitted once per update with an angular velocity (cached) and held until the
    /// next; `pressing`: a press from this frame's counters drives the car instead.
    fn flip_pitch(&mut self, query: &FitQuery, airborne: bool, pressing: bool) -> Option<f32> {
        if !self.options.input_fits {
            return None;
        }
        let FitQuery {
            index, car, state, ..
        } = *query;
        let life = car.life;
        if !(airborne && !pressing && state.is_flipping && state.flip_rel_torque.y != 0.0) {
            if !state.is_flipping {
                self.last_cancel.remove(&life);
            }
            return None;
        }
        let sign = state.flip_rel_torque.y.signum();
        let mut cancel = self.last_cancel.get(&life).copied().unwrap_or(0.0);
        if let Some(update_frame) = car
            .body
            .angular_velocity_raw
            .as_ref()
            .map(|v| v.frame.get())
        {
            let key = (life, update_frame);
            if !self.flip_cancels.contains_key(&key) && update_frame == index {
                let mut base = *query.controls;
                if query.controls.handbrake {
                    base.roll = query.controls.steer;
                } else {
                    base.yaw = query.controls.steer;
                }
                let fitted = if query.in_play && !query.new_life {
                    let ctx = self.fit_context();
                    fits::fit_flip_cancel(ctx, query, &base, self.scratch.flip(query.hitbox))
                } else {
                    None
                };
                self.flip_cancels.insert(key, fitted);
            }
            if let Some(Some(fitted)) = self.flip_cancels.get(&key) {
                cancel = *fitted;
                self.last_cancel.insert(life, cancel);
            }
        }
        Some(cancel * sign)
    }

    fn dodge_start(&mut self, query: &FitQuery) -> Option<DodgePlan> {
        if !self.options.input_fits {
            return None;
        }
        let ctx = self.fit_context();
        let plan = fits::fit_dodge_start(ctx, query, self.scratch.flip(query.hitbox))?;
        self.record_dodge(query.car.life, &plan, plan.first_update);
        Some(plan)
    }

    fn air_schedule(
        &mut self,
        query: &AirScheduleQuery,
        press: Option<PressInFlight>,
    ) -> Option<(AirSchedule, i32)> {
        if !self.options.air_schedules {
            return None;
        }
        let flip_path = self.options.input_fits && (query.state.is_flipping || press.is_some());
        let ctx = self.fit_context();
        let scratch = if flip_path {
            Some(self.scratch.flip(query.hitbox))
        } else {
            None
        };
        let planned = self.ticks.and_then(|ticks| {
            air_schedule::plan(
                ctx.frames,
                ticks,
                &self.overrides,
                ctx.withheld,
                query,
                press,
                scratch,
            )
        });
        if planned.is_some() {
            self.diagnostics.air_schedules_planned += 1;
        } else if !query.state.is_on_ground {
            self.diagnostics.air_schedules_refused += 1;
        }
        planned
    }

    fn ground_schedule(&mut self, query: &FitQuery, dodge_pending: bool) -> Option<GroundChoice> {
        if !self.options.input_fits {
            return None;
        }
        let ctx = self.fit_context();
        let life = query.car.life;
        let timing = fits::fit_ground_timing(ctx, query, self.scratch.ground(query.hitbox));
        if let Some(shift) = timing.as_ref().and_then(|s| s.shift) {
            self.shifts.entry(life).or_default().push(shift);
        }
        if let Some(schedule) = timing {
            return Some(GroundChoice {
                schedule,
                dodge: None,
            });
        }
        if let Some(schedule) = fits::fit_jump_timing(ctx, query, self.scratch.ground(query.hitbox))
        {
            return Some(GroundChoice {
                schedule,
                dodge: None,
            });
        }
        if dodge_pending {
            return None;
        }
        let flip = fits::fit_ground_flip(ctx, query, self.scratch.ground(query.hitbox))?;
        if let Some(plan) = &flip.dodge {
            self.record_dodge(life, plan, None);
        }
        if let Some((frame, ticks)) = flip.first_update {
            self.overrides.insert((life, frame), ticks);
        }
        Some(GroundChoice {
            schedule: flip.schedule,
            dodge: flip.dodge,
        })
    }

    fn ticks_override(&self, life: CarLife, index: usize) -> Option<u64> {
        self.overrides.get(&(life, index)).copied()
    }

    fn dodge_handled(&self, life: CarLife, index: usize) -> bool {
        self.handled.contains(&(life, index))
    }

    /// The median of the car's last 15 informative shifts, once it has 5; 0 is no shift.
    fn control_shift(&self, life: CarLife) -> Option<i64> {
        self.shifts
            .get(&life)
            .filter(|v| v.len() >= 5)
            .map(|v| {
                let mut recent: Vec<i64> = v[v.len().saturating_sub(15)..].to_vec();
                recent.sort_unstable();
                recent[recent.len() / 2]
            })
            .filter(|&m| m != 0)
    }
}
