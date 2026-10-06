//! What the replay reports at the end of an interval: demolitions and flip resets, applied to the simulation.

use super::{HoldSource, Simulator};
use crate::decode::{DemolitionReport, NetworkCar, NetworkEvent, NetworkFrame};

impl Simulator<'_> {
    /// Observed demolitions, after the interval (so that the bump of the demolition is still simulated). A
    /// repeated report is skipped, as is a victim that is gone from the frame or is not its player's current
    /// car. A goal explosion demolishes only a wreck (a car whose player link is inactive, or sleeping) and
    /// holds it until the player's next car life or a live update; another demolition holds the car
    /// demolished for 3 s (360 ticks), longer when its car is a wreck.
    pub(super) fn apply_demolitions(
        &mut self,
        frame: &NetworkFrame,
        frame_cars: &[&NetworkCar],
        replay_tick: u64,
    ) {
        for event in &frame.events {
            let NetworkEvent::Demolition {
                report,
                victim_car: Some(victim),
                repeat,
                ..
            } = event
            else {
                continue;
            };
            let goal_explosion = *report == DemolitionReport::GoalExplosion;
            if (!goal_explosion && *repeat) || !frame.cars.iter().any(|c| c.life.actor == *victim) {
                continue;
            }
            let Some(&(player, created)) = self.players.by_actor.get(victim) else {
                continue;
            };
            let Some(victim_car) = frame_cars.iter().find(|c| {
                c.life.actor == *victim && c.life.created == created && c.player.is_some()
            }) else {
                continue;
            };
            let sleeping = victim_car.body.sleeping.as_ref().is_some_and(|s| s.value);
            if goal_explosion && victim_car.player_link_active && !sleeping {
                continue;
            }
            // A demolition of a car still held on its spawn pose is an ordinary demolition.
            if self.holds.spawning.remove(&player).is_some() {
                self.cars
                    .entry(victim_car.life)
                    .or_default()
                    .spawn_demolished = true;
            }
            let slot = player.get();
            let mut state = *self.arena.get_car_state(slot);
            if !state.is_demoed || goal_explosion {
                state.is_demoed = true;
                state.demo_respawn_timer = 3.0;
                self.arena.set_car_state(slot, state);
                self.arena.refresh_car_sticky_gate(slot);
            }
            if goal_explosion {
                // Counted once per hold (the replay re-sends the report); a hold begun by a sleeping update
                // becomes observed.
                let new_hold = !self
                    .holds
                    .wrecks
                    .get(&player)
                    .is_some_and(|&(life, source)| {
                        life == victim_car.life && source == HoldSource::Observed
                    });
                self.holds
                    .wrecks
                    .insert(player, (victim_car.life, HoldSource::Observed));
                if new_hold {
                    self.diagnostics.goal_explosion_demolitions += 1;
                }
            } else {
                self.holds
                    .demolished_until
                    .insert(player, replay_tick + 360);
                // A demolished car whose body is a wreck stays out of the simulation until the player's next
                // car life or a live update, not just 3 s.
                if (!victim_car.player_link_active || sleeping)
                    && !self
                        .holds
                        .wrecks
                        .get(&player)
                        .is_some_and(|&(life, _)| life == victim_car.life)
                {
                    self.holds
                        .wrecks
                        .insert(player, (victim_car.life, HoldSource::Observed));
                    self.diagnostics.wrecks_after_demolition += 1;
                }
            }
        }
    }

    /// Observed flip resets: when the simulation has not reset the car's flip itself, do what RocketSim does on
    /// the tick a wheel touches something. The controls survive the state change.
    pub(super) fn apply_flip_resets(&mut self, frame: &NetworkFrame, frame_cars: &[&NetworkCar]) {
        for event in &frame.events {
            let NetworkEvent::FlipReset { car, .. } = event else {
                continue;
            };
            let Some(&(player, created)) = self.players.by_actor.get(car) else {
                continue;
            };
            if !frame_cars
                .iter()
                .any(|c| c.life.actor == *car && c.life.created == created && c.player.is_some())
            {
                continue;
            }
            self.diagnostics.flip_resets_observed += 1;
            let slot = player.get();
            let mut state = *self.arena.get_car_state(slot);
            if state.is_demoed
                || !(state.has_jumped
                    || state.has_double_jumped
                    || state.has_flipped
                    || state.is_flipping)
            {
                continue;
            }
            state.has_jumped = false;
            state.has_double_jumped = false;
            state.has_flipped = false;
            state.is_flipping = false;
            state.is_jumping = false;
            state.flip_time = 0.0;
            state.air_time = 0.0;
            state.air_time_since_jump = 0.0;
            let controls = *self.arena.get_car_controls(slot);
            self.arena.set_car_state(slot, state);
            self.arena.set_car_controls(slot, controls);
            self.diagnostics.flip_resets_applied += 1;
        }
    }
}
