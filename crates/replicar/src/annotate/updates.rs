//! Which bodies were updated in a frame, and how old their last update is (docs/glossary.md, "Updated", "Ticks
//! since update", "Seconds since update"). Outputs only: nothing here feeds the simulation.
//!
//! - `ball_updated`, `car_updated`: the simulator applied an update of the body at this frame. A player whose
//!   car is not in the frame's state, or whose current car cannot be resolved (between car lives), has no
//!   value: unknown. It counts updates in frames the simulator does not step too (goal pause, countdown, the
//!   first kickoff frame).
//! - `*_seconds_since_update`: the frame's replay time minus that of the frame that carried the body's last
//!   update (frame cadence, not the update's tick; 0 at an updated frame). Observed. None before the first
//!   update, for a player without a resolved car, and for a respawned car until its first update.
//! - `*_ticks_since_update`: the frame's replay tick minus the update tick of the last applied update, carried
//!   forward (usually 0-4 at an updated frame, growing until the next). Uses the offline update-tick
//!   inference: none before the first update, when the update's tick was not inferred (no inference, a frame
//!   the simulator does not step, or the `Default` source, which is half the frame window and not an
//!   inference), and for a car from its respawn until its first update.

use replicar_format::FrameIndex;

use crate::decode::{CarLife, NetworkFrame};
use crate::simulate::{AppliedTick, SimulatedFrame, TickSource};

/// The updates of one frame, per player where a list.
#[derive(Debug, Clone, PartialEq)]
pub struct Updates {
    pub ball_updated: bool,
    pub car_updated: Vec<Option<bool>>,
    pub ball_seconds_since_update: Option<f32>,
    pub car_seconds_since_update: Vec<Option<f32>>,
    pub ball_ticks_since_update: Option<u32>,
    pub car_ticks_since_update: Vec<Option<u32>>,
}

/// The update tick of an applied update, or `None` when it was not inferred.
fn update_tick(replay_tick: u64, applied: Option<&AppliedTick>) -> Option<i64> {
    let applied = applied.filter(|a| a.source != TickSource::Default)?;
    Some(replay_tick as i64 - applied.ticks as i64)
}

/// Carries the last update's tick forward from frame to frame. Use one per simulation, in frame order.
#[derive(Debug, Default)]
pub struct UpdateTracker {
    /// The ball's last applied update: its update tick (`None`: not inferred), or no update yet.
    ball: Option<Option<i64>>,
    /// Per player: the car life of the last update and its update tick.
    cars: Vec<Option<(CarLife, Option<i64>)>>,
}

impl UpdateTracker {
    /// The updates of `simulated`, the simulation of `network_frame`; `frames` are the replay's frames.
    pub fn frame(
        &mut self,
        frames: &[NetworkFrame],
        network_frame: &NetworkFrame,
        simulated: &SimulatedFrame,
    ) -> Updates {
        let index = network_frame.index.get();
        let replay_tick = simulated.replay_tick;
        let players = simulated.state.cars.len();
        if self.cars.len() < players {
            self.cars.resize(players, None);
        }
        let ticks_since = |tick: Option<i64>| tick.map(|t| (replay_tick as i64 - t).max(0) as u32);
        let seconds_since = |update: FrameIndex| {
            (f64::from(network_frame.time) - f64::from(frames[update.get()].time)) as f32
        };
        if simulated.ball_updated {
            let applied = simulated.applied_ticks.iter().find(|a| a.car.is_none());
            self.ball = Some(update_tick(replay_tick, applied));
        }
        let ball_seconds_since_update = network_frame
            .ball
            .as_ref()
            .and_then(|body| body.position.as_ref())
            .filter(|position| position.frame.get() <= index)
            .map(|position| seconds_since(position.frame));
        // Each player's current car, among the cars the simulation linked to a player.
        let mut player_cars = vec![None; players];
        for car in network_frame.current_cars() {
            let Some(&(_, player)) = simulated
                .car_players
                .iter()
                .find(|(actor, _)| *actor == car.life.actor)
            else {
                continue;
            };
            if let Some(slot) = player_cars.get_mut(player.get()) {
                *slot = Some(car);
            }
        }
        let mut car_updated = Vec::with_capacity(players);
        let mut car_seconds_since_update = Vec::with_capacity(players);
        let mut car_ticks_since_update = Vec::with_capacity(players);
        for (player, car) in player_cars.into_iter().enumerate() {
            let Some(car) = car else {
                car_updated.push(None);
                car_seconds_since_update.push(None);
                car_ticks_since_update.push(None);
                continue;
            };
            // A car of another life (a respawn) does not inherit the previous car's update.
            if self.cars[player].is_some_and(|(life, _)| life != car.life) {
                self.cars[player] = None;
            }
            let updated = simulated.updated_players.iter().any(|p| p.get() == player);
            if updated {
                let applied = simulated
                    .applied_ticks
                    .iter()
                    .find(|a| a.car == Some(car.life.actor));
                self.cars[player] = Some((car.life, update_tick(replay_tick, applied)));
            }
            car_updated.push(Some(updated));
            car_seconds_since_update.push(
                car.body
                    .position
                    .as_ref()
                    .filter(|position| position.frame.get() <= index)
                    .map(|position| seconds_since(position.frame)),
            );
            car_ticks_since_update.push(ticks_since(self.cars[player].and_then(|(_, t)| t)));
        }
        Updates {
            ball_updated: simulated.ball_updated,
            car_updated,
            ball_seconds_since_update,
            car_seconds_since_update,
            ball_ticks_since_update: ticks_since(self.ball.flatten()),
            car_ticks_since_update,
        }
    }
}
