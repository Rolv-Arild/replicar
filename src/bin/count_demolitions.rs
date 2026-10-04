//! Per replay: demolitions the replay reports (`Event::Demolish`, not the post-goal explosion) against
//! the simulation's own (`car_hit_car` with `is_demo`), and how many of the simulated ones have an
//! observed demolition of the same victim within 30 ticks. Also counted: the replay's re-reports
//! (`repeat`, within `DEMOLITION_REPEAT_WINDOW` of the first), the post-goal explosions (excluded) and
//! the victims with no linked car in the frame (dropped from the observed count, listed apart).
//! The simulated count is 0 by construction while observed demolitions are applied and RocketSim's own
//! rule is disabled (the defaults); `NO_OBSERVED_DEMOS=1` turns RocketSim's rule on for the comparison.
//!
//! usage: `count_demolitions <dir or replay> [--final-assessment]`
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes};
use replay_to_rocketsim::observations::Event;
use rocketsim::ArenaEvent;

fn replay_paths(path: &Path, final_assessment: bool) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    // Every directory and file opened is resolved and refused when it is in the sealed test split (a link or
    // junction under another name included).
    replay_to_rocketsim::ensure_unsealed(path, final_assessment)?;
    if path.is_file() {
        return Ok(vec![path.to_owned()]);
    }
    let mut result = Vec::new();
    for size in ["1v1", "2v2", "3v3"] {
        replay_to_rocketsim::ensure_unsealed(&path.join(size), final_assessment)?;
        for entry in fs::read_dir(path.join(size))? {
            let path = entry?.path();
            if path.extension().is_some_and(|ext| ext == "replay") {
                replay_to_rocketsim::ensure_unsealed(&path, final_assessment)?;
                result.push(path);
            }
        }
    }
    result.sort();
    Ok(result)
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args()
            .nth(1)
            .ok_or("usage: count_demolitions <dir or replay> [--final-assessment]")?,
    );
    // The test split is sealed until the frozen assessment (TEST_PROTOCOL.md); only that run passes the flag.
    if replay_to_rocketsim::sealed_path_refused(
        &path,
        env::args().any(|arg| arg == "--final-assessment"),
    ) {
        return Err("refusing to inspect a path with a 'test' component (pass --final-assessment for the frozen run)".into());
    }
    let options = ConvertOptions::default();
    // RocketSim's own demolition rule only runs when it is not disabled (`disable_simulated_demolitions`).
    let simulated_possible = !options.disable_simulated_demolitions;
    let (mut tot_obs, mut tot_sim, mut tot_both) = (0, 0, 0);
    let (mut tot_repeat, mut tot_unlinked, mut tot_goal, mut tot_repeat_unlinked) =
        (0usize, 0usize, 0usize, 0usize);
    for replay in replay_paths(&path, env::args_os().any(|arg| arg == "--final-assessment"))? {
        let output = convert_bytes(&fs::read(&replay)?, &options)?;
        let mut observed: Vec<(u64, usize)> = Vec::new(); // (timeline tick, victim slot)
        let mut simulated: Vec<(u64, usize)> = Vec::new();
        let (mut repeats, mut unlinked, mut goals, mut repeats_unlinked) =
            (0usize, 0usize, 0usize, 0usize);
        for (frame, converted) in output.observations.frames.iter().zip(&output.frames) {
            for event in &frame.events {
                if let Event::Demolish {
                    source,
                    victim_car,
                    repeat,
                    ..
                } = event
                {
                    if *source == "goal_explosion" {
                        goals += 1;
                    } else if *repeat {
                        repeats += 1;
                        let linked = victim_car.is_some_and(|v| {
                            frame
                                .cars
                                .iter()
                                .any(|c| c.actor_id == v && c.player_key.is_some())
                        });
                        repeats_unlinked += usize::from(!linked);
                    } else {
                        let slot = victim_car.and_then(|v| {
                            output
                                .car_slots
                                .iter()
                                .find(|s| {
                                    frame.cars.iter().any(|c| {
                                        c.actor_id == v
                                            && c.player_key.as_deref()
                                                == Some(s.player_key.as_str())
                                    })
                                })
                                .map(|s| s.slot)
                        });
                        if let Some(slot) = slot {
                            observed.push((converted.timeline_tick, slot));
                        } else {
                            unlinked += 1;
                        }
                    }
                }
            }
            for e in &converted.simulated_events {
                if let ArenaEvent::CarHitCar(hit) = &e.event
                    && hit.is_demo
                {
                    let tick = converted.timeline_tick as i64
                        - (converted.state.tick_count as i64 - e.arena_tick as i64);
                    simulated.push((tick.max(0) as u64, hit.victim_car_idx));
                }
            }
        }
        let both = simulated
            .iter()
            .filter(|(t, v)| {
                observed
                    .iter()
                    .any(|(ot, ov)| ov == v && ot.abs_diff(*t) <= 30)
            })
            .count();
        tot_obs += observed.len();
        tot_sim += simulated.len();
        tot_both += both;
        tot_repeat += repeats;
        tot_unlinked += unlinked;
        tot_goal += goals;
        tot_repeat_unlinked += repeats_unlinked;
        println!(
            "{} observed {} repeat {} no-linked-car {} goal-explosion {} simulated {} both {}",
            replay
                .file_name()
                .unwrap()
                .to_string_lossy()
                .chars()
                .take(8)
                .collect::<String>(),
            observed.len(),
            repeats,
            unlinked,
            goals,
            if simulated_possible {
                simulated.len().to_string()
            } else {
                "n/a".to_owned()
            },
            if simulated_possible {
                both.to_string()
            } else {
                "n/a".to_owned()
            },
        );
    }
    println!(
        "total observed {tot_obs} (linked victim, not a repeat); repeats {tot_repeat} (same victim reported again within {} s; {} of them with no linked car); victims with no linked car in the frame {tot_unlinked} (not in observed); post-goal explosions {tot_goal} (excluded); non-goal events {}",
        replay_to_rocketsim::observations::DEMOLITION_REPEAT_WINDOW,
        tot_repeat_unlinked,
        tot_obs + tot_repeat + tot_unlinked
    );
    if simulated_possible {
        println!(
            "simulated {tot_sim}, with an observed demolition of the same victim within 30 ticks {tot_both}"
        );
    } else {
        println!(
            "simulated demolitions: not counted (observed demolitions are applied and RocketSim's own rule is disabled, disable_simulated_demolitions = true, so there are none by construction)"
        );
    }
    Ok(())
}
