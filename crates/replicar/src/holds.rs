//! How long a jump stays held where the replay cannot show it, set by a convention (docs/glossary.md, "Jump
//! release"). RocketSim reads the jump control for a press (against the control of the tick before) and, while a
//! ground jump pushes (`is_jumping`), for its release (`update_jump`); nowhere else. A held jump on a tick with no
//! ground jump pushing changes nothing, so how long a double jump or dodge press is held, or a ground jump past
//! RocketSim's 24-tick limit, is not in the replay. Such ticks are released: a double jump or dodge press lasts one
//! tick, and a ground jump held to the limit ends at it. The releases are labels set on the rows after the
//! simulation, from the controls and states of every simulated tick, so the states are unchanged; a release only
//! ever ends a held stretch early, so every press stays one and none is added.

use std::collections::HashSet;

use replicar_format::record::Frame;

use crate::simulate::TickRecord;

/// RocketSim's longest ground jump, in ticks (`car_consts::jump::MAX_TICKS`).
const MAX_TICKS: u32 = 24;

/// One simulated tick of a car: its controls' jump (applied by the step after the tick) and its state at the tick.
#[derive(Debug, Clone, Copy)]
struct Tick {
    segment: u32,
    sim_tick: u64,
    jump: bool,
    /// A ground jump pushes in the step after the tick and reads its jump: the step does not end it by the limit.
    pushing: bool,
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
                pushing: car.is_jumping && car.jump_ticks < MAX_TICKS,
                demoed: car.is_demoed,
            });
        }
    }

    /// Per player, the (segment, sim tick) of each tick whose jump the convention releases.
    fn released(&self) -> Vec<HashSet<(u32, u64)>> {
        self.players
            .iter()
            .map(|ticks| {
                let mut released = HashSet::new();
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
                    release_run(&ticks[start..end], &mut released);
                    start = end;
                }
                released
            })
            .collect()
    }
}

/// The releases of one run of consecutive ticks: each held stretch from a press to a release inside the run, from
/// the tick after the press, or after the last tick in it whose jump a ground jump's push reads or the car is
/// demolished at.
fn release_run(run: &[Tick], released: &mut HashSet<(u32, u64)>) {
    for press in (1..run.len()).filter(|&i| run[i].jump && !run[i - 1].jump) {
        let Some(release) = (press + 1..run.len()).find(|&i| !run[i].jump) else {
            continue;
        };
        let from = (press + 1..release)
            .rev()
            .find(|&i| run[i].pushing || run[i].demoed)
            .map_or(press + 1, |i| i + 1);
        released.extend(run[from..release].iter().map(|t| (t.segment, t.sim_tick)));
    }
}

/// Sets the releases on the rows: a released tick's controls release the jump, and so do the next tick's previous
/// controls.
pub(crate) fn apply(frames: &mut [Frame], ticks: &JumpTicks) {
    let released = ticks.released();
    if released.iter().all(HashSet::is_empty) {
        return;
    }
    for frame in frames.iter_mut() {
        let Some(segment) = frame.segment else {
            continue;
        };
        for (p, released) in released.iter().enumerate() {
            let Some(Some(car)) = frame.state.cars.get_mut(p) else {
                continue;
            };
            if released.contains(&(segment, frame.sim_tick)) {
                car.controls.jump = false;
            }
            if frame.sim_tick > 0 && released.contains(&(segment, frame.sim_tick - 1)) {
                car.previous_controls.jump = false;
            }
        }
    }
}
