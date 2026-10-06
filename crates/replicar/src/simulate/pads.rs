//! Boost pads: matching the replay's pad records to RocketSim's pads, and their cooldowns.

use std::collections::HashMap;

use glam::Vec3A;
use rocketsim::{Arena, BoostPadState};

use super::Players;
use crate::decode::{ActorId, NetworkFrame, PadRecord};

/// Holding a pad on cooldown for this long keeps the simulation from picking it up.
pub(super) const HELD_COOLDOWN: f32 = 20.0;

pub(super) struct Pads {
    /// A pad keeps its name (`VehiclePickup_Boost_TA_14`) when its actor is created again (after a goal: 184
    /// actors for 34 pads in one game), so each name is matched to a pad by votes over the whole replay.
    by_name: HashMap<String, usize>,
    by_actor: HashMap<ActorId, usize>,
    last_counter: HashMap<ActorId, u8>,
    /// The pads' true cooldowns in seconds (0 = available), from the replay's records.
    pub(super) cooldowns: Vec<f32>,
}

/// The pad nearest to `pos` horizontally, when it is within 350 UU and at least 100 UU nearer than the
/// second nearest.
fn unambiguous_pad(arena: &Arena, pos: Vec3A) -> Option<usize> {
    let mut best = (f32::INFINITY, None);
    let mut second = f32::INFINITY;
    for index in 0..arena.num_boost_pads() {
        let pad = arena.get_boost_pad_config(index).pos;
        let d2 = (pad.x - pos.x).powi(2) + (pad.y - pos.y).powi(2);
        if d2 < best.0 {
            second = best.0;
            best = (d2, Some(index));
        } else if d2 < second {
            second = d2;
        }
    }
    (best.0 < 350.0 * 350.0 && second.sqrt() - best.0.sqrt() >= 100.0)
        .then_some(best.1)
        .flatten()
}

impl Pads {
    /// Votes for every pad name: at each new pickup, the pad nearest the instigator's position (updated in
    /// the last 0.1 s) when it is clear. A name needs at least two votes and twice the runner-up's.
    pub(super) fn new(arena: &Arena, frames: &[NetworkFrame]) -> Self {
        let mut votes: HashMap<&str, HashMap<usize, u32>> = HashMap::new();
        for frame in frames {
            for record in &frame.pad_records {
                let (Some(name), Some(instigator)) = (&record.pad_name, record.instigator_car)
                else {
                    continue;
                };
                if record.picked_up_raw == 255 || record.repeat {
                    continue;
                }
                let Some(pos) = frame
                    .cars
                    .iter()
                    .find(|c| c.life.actor == instigator)
                    .and_then(|c| c.body.position.as_ref())
                    .filter(|p| {
                        p.frame <= frame.index && frame.time - frames[p.frame.get()].time <= 0.1
                    })
                    .map(|p| Vec3A::from(p.value))
                else {
                    continue;
                };
                let mut ranked: Vec<(f32, usize)> = (0..arena.num_boost_pads())
                    .map(|index| {
                        let pad = arena.get_boost_pad_config(index).pos;
                        ((pad.x - pos.x).hypot(pad.y - pos.y), index)
                    })
                    .collect();
                ranked.sort_by(|a, b| a.0.total_cmp(&b.0));
                if ranked.len() >= 2 && ranked[0].0 < 350.0 && ranked[1].0 - ranked[0].0 >= 100.0 {
                    *votes
                        .entry(name)
                        .or_default()
                        .entry(ranked[0].1)
                        .or_default() += 1;
                }
            }
        }
        let by_name = votes
            .into_iter()
            .filter_map(|(name, counts)| {
                let mut ranked: Vec<(u32, usize)> =
                    counts.into_iter().map(|(pad, n)| (n, pad)).collect();
                ranked.sort_by(|a, b| b.cmp(a));
                let top = ranked[0];
                let second = ranked.get(1).map_or(0, |r| r.0);
                (top.0 >= 2 && top.0 >= 2 * second).then(|| (name.to_owned(), top.1))
            })
            .collect();
        Self {
            by_name,
            by_actor: HashMap::new(),
            last_counter: HashMap::new(),
            cooldowns: vec![0.0; arena.num_boost_pads()],
        }
    }

    /// Applies a frame's pad records to the arena and the true cooldowns: a pickup puts the pad on its full
    /// cooldown, 255 makes it available. A record whose counter did not change is a repeat.
    pub(super) fn apply(
        &mut self,
        arena: &mut Arena,
        frame: &NetworkFrame,
        frames: &[NetworkFrame],
        players: &Players,
    ) {
        for record in &frame.pad_records {
            let pad = self.pad_of(arena, record, frame, frames, players);
            let changed = self.last_counter.insert(record.pad, record.picked_up_raw)
                != Some(record.picked_up_raw);
            let Some(index) = pad.filter(|_| changed) else {
                continue;
            };
            if record.picked_up_raw == 255 {
                arena.set_boost_pad_state(index, BoostPadState { cooldown: 0.0 });
                self.cooldowns[index] = 0.0;
            } else if record.picked_up_raw % 2 == 1 {
                let full = if arena.get_boost_pad_config(index).is_big {
                    10.0
                } else {
                    4.0
                };
                arena.set_boost_pad_state(index, BoostPadState { cooldown: full });
                self.cooldowns[index] = full;
            }
        }
    }

    /// The pad of a record: by its name's vote, else by the actor's earlier match, else the pad nearest the
    /// instigator (its position updated in the last 0.1 s, or its simulated one).
    fn pad_of(
        &mut self,
        arena: &Arena,
        record: &PadRecord,
        frame: &NetworkFrame,
        frames: &[NetworkFrame],
        players: &Players,
    ) -> Option<usize> {
        if let Some(&index) = record
            .pad_name
            .as_ref()
            .and_then(|name| self.by_name.get(name))
        {
            self.by_actor.insert(record.pad, index);
        }
        if let Some(&index) = self.by_actor.get(&record.pad) {
            return Some(index);
        }
        let instigator = record.instigator_car?;
        let pos = frame
            .cars
            .iter()
            .find(|c| c.life.actor == instigator)
            .and_then(|c| c.body.position.as_ref())
            .filter(|p| p.frame <= frame.index && frame.time - frames[p.frame.get()].time <= 0.1)
            .map(|p| Vec3A::from(p.value))
            .or_else(|| {
                players
                    .by_actor
                    .get(&instigator)
                    .map(|&(player, _)| arena.get_car_state(player.get()).phys.pos)
            })?;
        let index = unambiguous_pad(arena, pos)?;
        self.by_actor.insert(record.pad, index);
        Some(index)
    }

    /// The pad a pad actor was matched to.
    pub(super) fn index_of(&self, pad: ActorId) -> Option<usize> {
        self.by_actor.get(&pad).copied()
    }

    /// Holds every pad of the arena on cooldown (from the first tick of an interval).
    pub(super) fn hold(arena: &mut Arena) {
        for index in 0..arena.num_boost_pads() {
            arena.set_boost_pad_state(
                index,
                BoostPadState {
                    cooldown: HELD_COOLDOWN,
                },
            );
        }
    }

    /// The true cooldowns recharge in real time.
    pub(super) fn recharge(&mut self, seconds: f32) {
        for cooldown in &mut self.cooldowns {
            *cooldown = (*cooldown - seconds).max(0.0);
        }
    }

    /// Writes the true cooldowns into the arena, for the exported state.
    pub(super) fn write(&self, arena: &mut Arena) {
        for (index, &cooldown) in self.cooldowns.iter().enumerate() {
            arena.set_boost_pad_state(index, BoostPadState { cooldown });
        }
    }
}
