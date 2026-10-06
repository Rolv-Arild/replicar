//! The player table: which player index each car belongs to (docs/glossary.md, "Player index").

use std::collections::{BTreeSet, HashMap};

use replicar_format::{FrameIndex, PlayerIndex, Team};
use rocketsim::Arena;

use crate::decode::{ActorId, NetworkCar, PlayerKey};
use crate::hitbox::Hitbox;

/// A player of the simulation: one RocketSim car for the whole replay, created the first time the player's
/// car appears with a team.
#[derive(Debug, Clone, PartialEq)]
pub struct SimPlayer {
    pub index: PlayerIndex,
    pub key: PlayerKey,
    pub team: Team,
    /// The car body known when the player's car was created.
    pub body_product_id: Option<u32>,
    pub hitbox: Hitbox,
}

#[derive(Default)]
pub(super) struct Players {
    pub(super) players: Vec<SimPlayer>,
    by_key: HashMap<PlayerKey, PlayerIndex>,
    /// The player and car life each car actor last belonged to.
    pub(super) by_actor: HashMap<ActorId, (PlayerIndex, FrameIndex)>,
    /// Loadout changes already counted: (player, team, body).
    changes_seen: BTreeSet<(PlayerIndex, Option<Team>, Option<u32>)>,
}

/// How a car was resolved to a player.
pub(super) struct Resolved {
    pub(super) player: PlayerIndex,
    /// The car starts a new life on this player in this frame (its first frame, or the player's first car).
    pub(super) new_life: bool,
}

impl Players {
    pub(super) fn index_of(&self, key: &PlayerKey) -> Option<PlayerIndex> {
        self.by_key.get(key).copied()
    }

    pub(super) fn len(&self) -> usize {
        self.players.len()
    }

    /// The player of `car` at `frame`, adding the player (and its RocketSim car) when its car is seen with a
    /// team for the first time. A car without a player link keeps the player of its car life, if any.
    /// Counts loadout changes and players without a known hitbox in `diagnostics`.
    pub(super) fn resolve(
        &mut self,
        car: &NetworkCar,
        frame: FrameIndex,
        arena: &mut Arena,
        loadout_hitboxes: bool,
        diagnostics: &mut super::SimDiagnostics,
    ) -> Option<Resolved> {
        let actor = car.life.actor;
        let Some(key) = &car.player else {
            return self
                .by_actor
                .get(&actor)
                .filter(|(_, created)| *created == car.life.created)
                .map(|&(player, _)| Resolved {
                    player,
                    new_life: false,
                });
        };
        let body_now = car.body_product_id.as_ref().map(|v| v.value);
        if let Some(&player) = self.by_key.get(key) {
            self.by_actor.insert(actor, (player, car.life.created));
            let info = &self.players[player.get()];
            let team_changed = car.team.is_some_and(|team| team != info.team);
            let body_changed = body_now.is_some()
                && info.body_product_id.is_some()
                && body_now != info.body_product_id;
            if (team_changed || body_changed)
                && self.changes_seen.insert((player, car.team, body_now))
            {
                diagnostics.player_loadout_changes += 1;
            }
            return Some(Resolved {
                player,
                new_life: car.life.created == frame,
            });
        }
        let team = car.team?;
        let known = body_now
            .filter(|_| loadout_hitboxes)
            .and_then(Hitbox::of_body);
        let hitbox = known.unwrap_or_default();
        let rocketsim_team = match team {
            Team::Blue => rocketsim::Team::Blue,
            Team::Orange => rocketsim::Team::Orange,
        };
        let player =
            PlayerIndex(u8::try_from(arena.add_car(rocketsim_team, hitbox.config())).ok()?);
        self.by_key.insert(key.clone(), player);
        self.players.push(SimPlayer {
            index: player,
            key: key.clone(),
            team,
            body_product_id: body_now,
            hitbox,
        });
        self.by_actor.insert(actor, (player, car.life.created));
        diagnostics.default_hitbox_players += usize::from(known.is_none());
        Some(Resolved {
            player,
            new_life: true,
        })
    }
}
