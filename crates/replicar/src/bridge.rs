//! Run bridging (docs/glossary.md): car runs joined where a simulated path fixes the interval between two updates.
//!
//! A client replay's car updates are placed on the timeline by runs of updates whose motion chains them
//! (`update_ticks`); each run is placed on its own, a few ticks off at times, and a run ends at a pair the motion
//! does not chain (a turn, a jump, a landing). On the RLBot LAN client recordings 80% of the misplaced updates
//! miss by the same amount as the update before them: whole runs are off, and joining the runs that belong
//! together places most of them (RESULTS.md, "The client replays' states against the server's"). A first
//! simulation without the expensive fits gives each car's path from an update; where that path meets the car's
//! next update clearly best at one interval, that interval joins the two updates' runs, and the update ticks are
//! inferred again. The ball keeps its first placement: its runs are placed against the cars', and placing them
//! against the joined runs made them worse.

use std::collections::{BTreeMap, HashMap};

use glam::Vec3A;

use crate::decode::{ActorId, CarLife, NetworkReplay};
use crate::infer::{FittedInference, InferenceOptions};
use crate::simulate::{SimulationOptions, simulate};
use crate::update_ticks::{UpdateTicks, Withheld, infer_bridged};
use crate::{Error, Meshes};

/// Intervals tried, in ticks off the current one.
const OFFSETS: std::ops::RangeInclusive<i64> = -3..=3;
/// The best interval's position error must be under the next best's by this factor.
const RATIO: f32 = 3.0;
/// The car must move at least this fast (UU/s) at both updates: slower, a tick is too little distance to tell.
const MIN_SPEED: f32 = 350.0;
/// Longest interval bridged, in ticks.
const MAX_TICKS: u64 = 60;

/// `ticks` (of a replay that is not lag-free) with the car updates inferred again with the bridged intervals.
/// RocketSim's meshes must be loaded.
pub fn bridge_runs(
    network: &NetworkReplay,
    ticks: &UpdateTicks,
    meshes: &Meshes,
    inference: InferenceOptions,
    options: &SimulationOptions,
) -> Result<UpdateTicks, Error> {
    let withheld = Withheld(options.withheld.as_deref());
    let mut first = FittedInference::new(
        network,
        Some(ticks),
        InferenceOptions {
            air_schedules: false,
            input_fits: false,
            ..inference
        },
        withheld,
    );
    // Each car's simulated position and velocity by sim tick, and the sim tick each update was applied at.
    let mut paths: HashMap<ActorId, BTreeMap<u64, (Vec3A, Vec3A)>> = HashMap::new();
    let mut applied: HashMap<(ActorId, usize), u64> = HashMap::new();
    simulate(
        network,
        Some(ticks),
        &mut first,
        meshes,
        options.clone(),
        |frame| {
            let now = frame.state.tick_count;
            for tick in &frame.applied_ticks {
                if let Some(actor) = tick.car {
                    applied.insert((actor, frame.index.get()), now.saturating_sub(tick.ticks));
                }
            }
            for record in &frame.ticks {
                for &(actor, player) in &frame.car_players {
                    if let Some(car) = record.cars.get(player.get()) {
                        paths
                            .entry(actor)
                            .or_default()
                            .insert(record.sim_tick, (car.phys.pos, car.phys.vel));
                    }
                }
            }
        },
    )?;

    // Each car life's position updates, in order.
    let mut updates: BTreeMap<CarLife, Vec<(usize, Vec3A)>> = BTreeMap::new();
    for frame in &network.frames {
        for car in &frame.cars {
            if let Some(position) = car
                .body
                .position
                .as_ref()
                .filter(|p| p.frame == frame.index)
            {
                updates
                    .entry(car.life)
                    .or_default()
                    .push((frame.index.get(), Vec3A::from(position.value)));
            }
        }
    }
    let mut bridges = BTreeMap::new();
    for (life, list) in &updates {
        let Some(path) = paths.get(&life.actor) else {
            continue;
        };
        for pair in list.windows(2) {
            let ((fa, _), (fb, target)) = (pair[0], pair[1]);
            let (Some(&ta), Some(&tb)) = (
                applied.get(&(life.actor, fa)),
                applied.get(&(life.actor, fb)),
            ) else {
                continue;
            };
            if tb <= ta + 1 || tb - ta > MAX_TICKS || (ta..tb).any(|t| !path.contains_key(&t)) {
                continue;
            }
            // The path from the first update; past the tick the second was applied at, it goes on at its velocity.
            let current = (tb - ta) as i64;
            let (last_pos, last_vel) = path[&(tb - 1)];
            let position = |n: i64| -> Vec3A {
                let t = ta as i64 + n;
                if t < tb as i64 {
                    path[&(t as u64)].0
                } else {
                    last_pos + last_vel * ((t - tb as i64 + 1) as f32 / 120.0)
                }
            };
            let mut errors: Vec<(i64, f32)> = OFFSETS
                .map(|d| current + d)
                .filter(|&n| n >= 1)
                .map(|n| (n, (position(n) - target).length()))
                .collect();
            errors.sort_by(|a, b| a.1.total_cmp(&b.1));
            let speed = path[&ta].1.length().min(last_vel.length());
            if speed >= MIN_SPEED
                && errors.len() >= 2
                && errors[1].1 >= RATIO * errors[0].1.max(1e-3)
            {
                bridges.insert((*life, fa, fb), errors[0].0);
            }
        }
    }
    let mut bridged = infer_bridged(network, options.loadout_hitboxes, withheld, &bridges);
    bridged.ball.clone_from(&ticks.ball);
    bridged.ball_car_offset = ticks.ball_car_offset;
    bridged.bridged_hits = ticks.bridged_hits;
    Ok(bridged)
}
