//! How long a jump stays held where the replay cannot show it: after a double jump or dodge press, and past the
//! 24-tick limit of a ground jump. RocketSim acts on a jump press, and on the hold only while a ground jump pushes
//! (`is_jumping`), so neither hold changes the simulation: they are labels set on the rows after it, from the
//! controls and states of every simulated tick, and a hold ends before the next press so that every press stays one.
//! The lengths fit the true inputs best (RESULTS.md, "Jump holds"): humans hold a double jump or dodge about 20 ticks
//! and release a ground jump that reaches the limit some 6 ticks past it; bots hold about 8 ticks and release at the
//! limit.

use std::collections::HashSet;

use replicar_format::record::Frame;

use crate::simulate::{SimPlayer, TickRecord};

/// RocketSim's longest ground jump, in ticks (`car_consts::jump::MAX_TICKS`).
const MAX_TICKS: usize = 24;

struct Prior {
    /// The hold of a double jump or dodge press.
    air_hold: usize,
    /// The ticks past the limit a ground jump held to it is released at.
    past_limit: usize,
    /// ...but this many ticks before a double jump or dodge press that follows.
    before_air_press: usize,
}

const HUMAN: Prior = Prior {
    air_hold: 20,
    past_limit: 6,
    before_air_press: 8,
};

const BOT: Prior = Prior {
    air_hold: 8,
    past_limit: 0,
    before_air_press: 8,
};

/// One simulated tick of a car: its controls' jump (applied by the step after the tick) and its state at the tick.
#[derive(Debug, Clone, Copy)]
struct Tick {
    segment: u32,
    sim_tick: u64,
    jump: bool,
    on_ground: bool,
    jumping: bool,
    demoed: bool,
}

/// Every simulated tick of each player's car in play, in order.
#[derive(Debug, Default)]
pub(crate) struct JumpTicks {
    players: Vec<Vec<Tick>>,
}

impl JumpTicks {
    /// Adds a tick of play segment `segment`.
    pub(crate) fn push(&mut self, segment: Option<u32>, record: &TickRecord) {
        let Some(segment) = segment else {
            return;
        };
        if self.players.len() < record.cars.len() {
            self.players.resize(record.cars.len(), Vec::new());
        }
        for (ticks, car) in self.players.iter_mut().zip(&record.cars) {
            ticks.push(Tick {
                segment,
                sim_tick: record.sim_tick,
                jump: car.controls.jump,
                on_ground: car.is_on_ground(),
                jumping: car.is_jumping,
                demoed: car.is_demoed,
            });
        }
    }

    /// Per player, the (segment, sim tick) of each tick whose jump a hold turns on. A player without a platform id
    /// (`player_id:` or `actor:` key) holds as a bot.
    fn held(&self, players: &[SimPlayer]) -> Vec<HashSet<(u32, u64)>> {
        self.players
            .iter()
            .enumerate()
            .map(|(p, ticks)| {
                let bot = players.get(p).is_some_and(|player| {
                    let key = &player.key.0;
                    key.starts_with("player_id:") || key.starts_with("actor:")
                });
                let prior = if bot { &BOT } else { &HUMAN };
                let mut held = HashSet::new();
                // Runs of consecutive ticks of one segment.
                let mut start = 0;
                while start < ticks.len() {
                    let mut end = start + 1;
                    while end < ticks.len()
                        && ticks[end].segment == ticks[start].segment
                        && ticks[end].sim_tick == ticks[end - 1].sim_tick + 1
                    {
                        end += 1;
                    }
                    hold_run(&ticks[start..end], prior, &mut held);
                    start = end;
                }
                held
            })
            .collect()
    }
}

/// The holds of one run of consecutive ticks.
fn hold_run(run: &[Tick], prior: &Prior, held: &mut HashSet<(u32, u64)>) {
    let presses: Vec<usize> = (1..run.len())
        .filter(|&i| run[i].jump && !run[i - 1].jump)
        .collect();
    for (n, &press) in presses.iter().enumerate() {
        let Some(release) = (press + 1..run.len()).find(|&i| !run[i].jump) else {
            continue;
        };
        let next = presses.get(n + 1).copied();
        let until = if !run[press].on_ground {
            // A double jump or dodge: released a tick before the car lands or presses again at the latest.
            let land = (press + 1..run.len()).find(|&i| run[i].on_ground);
            let bound = [next, land]
                .into_iter()
                .flatten()
                .min()
                .unwrap_or(run.len());
            (press + prior.air_hold).min(bound - 1)
        } else if release - press >= MAX_TICKS && !run[release].jumping {
            // A ground jump held to the limit: its push ended there, whenever it was released.
            let mut until = press + MAX_TICKS + prior.past_limit;
            if let Some(air) = presses[n + 1..].iter().find(|&&i| !run[i].on_ground) {
                until = until.min(air.saturating_sub(prior.before_air_press));
            }
            if let Some(next) = next {
                until = until.min(next - 1);
            }
            until
        } else {
            continue;
        };
        // Not while a ground jump still pushes (a press in its first ticks): RocketSim ends the push at the release
        // before the press, but a file does not keep that (`jump_hold_broken`), and a state restored from it would
        // push on with the jump held. Not across a demolition.
        let span = release..until.clamp(release, run.len());
        if run[span.clone()].iter().any(|t| t.demoed || t.jumping) {
            continue;
        }
        held.extend(run[span].iter().map(|t| (t.segment, t.sim_tick)));
    }
}

/// Sets the holds on the rows: a held tick's controls hold the jump, and so do the next tick's previous controls.
pub(crate) fn apply(frames: &mut [Frame], ticks: &JumpTicks, players: &[SimPlayer]) {
    let held = ticks.held(players);
    if held.iter().all(HashSet::is_empty) {
        return;
    }
    for frame in frames.iter_mut() {
        let Some(segment) = frame.segment else {
            continue;
        };
        for (p, held) in held.iter().enumerate() {
            let Some(Some(car)) = frame.state.cars.get_mut(p) else {
                continue;
            };
            if held.contains(&(segment, frame.sim_tick)) {
                car.controls.jump = true;
            }
            if frame.sim_tick > 0 && held.contains(&(segment, frame.sim_tick - 1)) {
                car.previous_controls.jump = true;
            }
        }
    }
}
