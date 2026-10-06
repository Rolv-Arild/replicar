//! The input fits: offline choices of what the replay does not say, each made by simulating candidates in a
//! scratch arena and scoring them on later updates (docs/glossary.md, "Fit"). Ported from v1's `fits.rs`;
//! the longer fits are split into steps in a later change, with parity guarding the split.

mod dodge_start;
mod flip_cancel;
mod ground_flip;
mod ground_timing;
mod jump_timing;

use std::collections::HashMap;

use glam::Vec3A;
use rocketsim::{Arena, ArenaConfig, BoostPadConfig, CarControls, CarState, GameMode, Team};

pub(super) use dodge_start::fit_dodge_start;
pub(super) use flip_cancel::fit_flip_cancel;
pub(super) use ground_flip::fit_ground_flip;
pub(super) use ground_timing::fit_ground_timing;
pub(super) use jump_timing::fit_jump_timing;

use crate::decode::{GameState, NetworkCar, NetworkFrame};
use crate::hitbox::Hitbox;
use crate::update_ticks::{UpdateTicks, Withheld};

/// A dodge to press: `jump` with the dodge direction `start_offset` ticks after the update it was planned
/// at, then `cancel` of the flip's pitch torque cancelled until `duration` ticks after it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DodgePlan {
    /// The frame whose dodge counter turned odd.
    pub activation_frame: usize,
    pub start_offset: u64,
    pub duration: u64,
    pub pitch: f32,
    pub yaw: f32,
    pub cancel: f32,
    /// The first update after the activation and its ticks before its frame, as fitted.
    pub first_update: Option<(usize, u64)>,
}

/// Ground controls tick by tick for one car until `end_tick`: (first sim tick, throttle, steer, handbrake,
/// boost, jump), in order; a jump of `None` leaves the jump control as it is.
#[derive(Debug, Clone, PartialEq)]
pub struct GroundSchedule {
    pub end_tick: u64,
    pub entries: Vec<(u64, f32, f32, bool, bool, Option<bool>)>,
    /// The timing shift the ground timing fit chose, when it was strictly better than the midpoint rule.
    pub shift: Option<i64>,
}

/// A ground jump followed by a dodge, fitted together.
#[derive(Debug, Clone, PartialEq)]
pub struct GroundFlip {
    pub schedule: GroundSchedule,
    pub dodge: Option<DodgePlan>,
    /// The first update after the activation and its ticks before its frame, as fitted.
    pub first_update: Option<(usize, u64)>,
}

/// What every fit reads: the frames, the update ticks, the withheld frames and the fit switches.
#[derive(Clone, Copy)]
pub(super) struct FitContext<'a> {
    pub(super) frames: &'a [NetworkFrame],
    /// `None` without update ticks: the fits that need them refuse, the flip cancel takes 0 ticks before.
    pub(super) ticks: Option<&'a UpdateTicks>,
    pub(super) withheld: Withheld<'a>,
    /// Fit the timings against the next update instead of the one after it.
    pub(super) fit_on_next_update: bool,
    /// Leave the first interval out of the flip-cancel fit when later updates exist.
    pub(super) flip_cancel_holdout: bool,
    /// Infer the tick of the first update after a dodge from the fitted path.
    pub(super) dodge_first_update_tick: bool,
}

impl FitContext<'_> {
    /// A frame's replay tick.
    pub(super) fn timeline(&self, frame: usize) -> i64 {
        let first = f64::from(self.frames.first().map_or(0.0, |f| f.time));
        ((f64::from(self.frames[frame].time) - first) * 120.0).round() as i64
    }

    pub(super) fn in_play(&self, frame: usize) -> bool {
        self.frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|s| s.value == GameState::Active)
    }

    pub(super) fn withheld(&self, frame: usize) -> bool {
        self.withheld.contains(frame)
    }

    /// The replay tick from which the controls first seen in frame `g` act: `2 + spacing / 2` ticks before
    /// the frame time.
    pub(super) fn control_change_tick(&self, g: usize) -> i64 {
        let spacing = if g == 0 {
            4
        } else {
            self.timeline(g) - self.timeline(g - 1)
        };
        self.timeline(g) - 2 - spacing / 2
    }
}

/// The same car in another frame: same car life and player.
pub(super) fn same_car<'c>(car: &'c NetworkCar) -> impl Fn(&&NetworkCar) -> bool + 'c {
    move |c: &&NetworkCar| c.life == car.life && c.player == car.player
}

/// The network controls of a car (throttle, steer, handbrake, boost and jump from the counters).
pub(super) fn network_controls(car: &NetworkCar) -> CarControls {
    crate::simulate::network_controls(car)
}

pub(super) fn vec3(value: [f32; 3]) -> Vec3A {
    Vec3A::from(value)
}

/// A scratch arena for the fits: one car with the given hitbox and no reachable boost pad. The arenas are
/// reused for the whole replay, so a pad one fit picked up would stay on cooldown into later fits; RocketSim
/// needs at least one pad, so the only one lies far below the floor.
fn scratch_arena(seed: u64, hitbox: Hitbox) -> Arena {
    let mut config = ArenaConfig::new(GameMode::Soccar);
    config.rng_seed = Some(seed);
    config.custom_boost_pads = Some(vec![BoostPadConfig {
        pos: Vec3A::new(0.0, 0.0, -10_000.0),
        is_big: false,
    }]);
    let mut scratch = Arena::new_with_config(config);
    scratch.add_car(Team::Blue, hitbox.config());
    scratch
}

/// An absolute sim tick of a state from an arena whose tick count was `source_tick`, as the same age before
/// `target_tick`; `None` for no tick, a tick after `source_tick`, or one that would be negative.
fn rebase_tick(tick: Option<u64>, source_tick: u64, target_tick: u64) -> Option<u64> {
    target_tick.checked_sub(source_tick.checked_sub(tick?)?)
}

/// Seeds a scratch arena's car with `state` from an arena whose tick count was `source_tick`, keeping the age
/// of its last extra ball hit (RocketSim grants the extra impulse only when `last_hit_tick + 1 < tick_count`).
pub(super) fn seed_car(scratch: &mut Arena, mut state: CarState, source_tick: u64) {
    state.last_extra_hit_tick =
        rebase_tick(state.last_extra_hit_tick, source_tick, scratch.tick_count());
    scratch.set_car_state(0, state);
    scratch.refresh_car_sticky_gate(0);
}

/// The scratch arenas, one per hitbox for the ground fits and one per hitbox for the flip and dodge fits:
/// each arena's tick count and ball carry over from one fit to the next, so the two kinds stay apart.
pub(super) struct ScratchArenas {
    seed: u64,
    ground: HashMap<Hitbox, Arena>,
    flip: HashMap<Hitbox, Arena>,
}

impl ScratchArenas {
    pub(super) fn new(seed: u64) -> Self {
        Self {
            seed,
            ground: HashMap::new(),
            flip: HashMap::new(),
        }
    }

    pub(super) fn ground(&mut self, hitbox: Hitbox) -> &mut Arena {
        let seed = self.seed;
        self.ground
            .entry(hitbox)
            .or_insert_with(|| scratch_arena(seed, hitbox))
    }

    pub(super) fn flip(&mut self, hitbox: Hitbox) -> &mut Arena {
        let seed = self.seed;
        self.flip
            .entry(hitbox)
            .or_insert_with(|| scratch_arena(seed, hitbox))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_hit_tick_keeps_its_age_in_another_arena() {
        assert_eq!(rebase_tick(Some(90), 100, 20), Some(10));
        assert_eq!(rebase_tick(Some(90), 100, 5), None);
        assert_eq!(rebase_tick(Some(110), 100, 50), None);
        assert_eq!(rebase_tick(None, 100, 50), None);
    }
}
