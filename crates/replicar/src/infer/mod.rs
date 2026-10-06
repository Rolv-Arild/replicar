//! The inference: what the replay does not say, chosen where the simulator needs it (docs/v2-plan.md,
//! section 4.2). The simulator asks; an `Inference` answers. `FittedInference` computes its answers, from
//! earlier and later updates.

mod air_schedule;

use rocketsim::CarControls;

pub use air_schedule::{AirSchedule, AirScheduleQuery};

use crate::air::{self, AirControls};
use crate::decode::{NetworkCar, NetworkReplay};
use crate::update_ticks::{UpdateTicks, Withheld};

/// Below this height (UU) a car's updates do not count as airborne for its air controls.
const AIR_MIN_Z: f32 = 50.0;

/// What the simulator asks.
pub trait Inference {
    /// The pitch, yaw and roll of an airborne car for the interval that starts at frame `index`, given the
    /// network controls it drives on (`controls`). `None`: no evidence, and the simulator uses the steer.
    fn air_controls(
        &mut self,
        index: usize,
        car: &NetworkCar,
        controls: &CarControls,
    ) -> Option<AirControls>;

    /// The air controls of an airborne car tick by tick over the interval from its update at
    /// `query.index` to its next one. `None`: the simulator keeps the interval's controls.
    fn air_schedule(&mut self, query: &AirScheduleQuery) -> Option<AirSchedule>;
}

/// What the inference counted.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct InferenceDiagnostics {
    /// Airborne updates whose interval got an air schedule, and airborne ones refused one.
    pub air_schedules_planned: usize,
    pub air_schedules_refused: usize,
}

/// Which inferences `FittedInference` makes.
#[derive(Debug, Clone, Copy)]
pub struct InferenceOptions {
    /// Solve an airborne car's air controls over the span to its next update (offline).
    pub air_lookahead: bool,
    /// Solve an airborne car's air controls tick by tick to its next update's rotation and angular velocity
    /// (offline).
    pub air_schedules: bool,
}

impl Default for InferenceOptions {
    fn default() -> Self {
        Self {
            air_lookahead: true,
            air_schedules: true,
        }
    }
}

/// The inference that computes its answers.
pub struct FittedInference<'a> {
    network: &'a NetworkReplay,
    ticks: Option<&'a UpdateTicks>,
    options: InferenceOptions,
    withheld: Withheld<'a>,
    pub diagnostics: InferenceDiagnostics,
}

impl<'a> FittedInference<'a> {
    /// `ticks` places the updates the fits reach for; without it the offline fits that need update ticks
    /// make no choice.
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
            diagnostics: InferenceDiagnostics::default(),
        }
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
    ) -> Option<AirControls> {
        let frames = &self.network.frames;
        if self.options.air_lookahead
            && let Some(solved) =
                air::lookahead_controls(frames, index, car, AIR_MIN_Z, self.withheld)
        {
            return Some(solved);
        }
        let (solved, lag) = air::past_controls(frames, index, car, AIR_MIN_Z)?;
        let keep = |axis: usize, value: f32| value * air::persistence(axis, lag, value.abs());
        let pitch = keep(0, solved.pitch);
        Some(if car.inputs.steer.is_none() {
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
        })
    }

    fn air_schedule(&mut self, query: &AirScheduleQuery) -> Option<AirSchedule> {
        if !self.options.air_schedules {
            return None;
        }
        let planned = self.ticks.and_then(|ticks| {
            air_schedule::plan(&self.network.frames, ticks, self.withheld, query)
        });
        if planned.is_some() {
            self.diagnostics.air_schedules_planned += 1;
        } else if !query.state.is_on_ground {
            self.diagnostics.air_schedules_refused += 1;
        }
        planned
    }
}
