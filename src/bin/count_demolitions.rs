//! Per replay: demolitions the replay reports (`Event::Demolish`, not the post-goal explosion) against
//! the simulation's own (`car_hit_car` with `is_demo`), and how many of the simulated ones have an
//! observed demolition of the same victim within 30 ticks.
//!
//! usage: count_demolitions <dir or replay>
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes};
use replay_to_rocketsim::observations::Event;
use rocketsim::ArenaEvent;

fn replay_paths(path: &Path) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    if path.is_file() {
        return Ok(vec![path.to_owned()]);
    }
    let mut result = Vec::new();
    for size in ["1v1", "2v2", "3v3"] {
        for entry in fs::read_dir(path.join(size))? {
            let path = entry?.path();
            if path.extension().is_some_and(|ext| ext == "replay") {
                result.push(path);
            }
        }
    }
    result.sort();
    Ok(result)
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(env::args().nth(1).ok_or("usage: count_demolitions <dir or replay>")?);
    if path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let mut options = ConvertOptions::default();
    options.apply_observed_demolitions = env::var_os("NO_OBSERVED_DEMOS").is_none();
    let (mut tot_obs, mut tot_sim, mut tot_both) = (0, 0, 0);
    for replay in replay_paths(&path)? {
        let output = convert_bytes(&fs::read(&replay)?, &options)?;
        let mut observed: Vec<(u64, usize)> = Vec::new(); // (timeline tick, victim slot)
        let mut simulated: Vec<(u64, usize)> = Vec::new();
        for (frame, converted) in output.observations.frames.iter().zip(&output.frames) {
            for event in &frame.events {
                if let Event::Demolish { source, victim_car: Some(v), .. } = event {
                    if *source != "goal_explosion" {
                        let slot = output
                            .car_slots
                            .iter()
                            .find(|s| {
                                frame.cars.iter().any(|c| {
                                    c.actor_id == *v && c.player_key.as_deref() == Some(s.player_key.as_str())
                                })
                            })
                            .map(|s| s.slot);
                        if let Some(slot) = slot {
                            observed.push((converted.timeline_tick, slot));
                        }
                    }
                }
            }
            for e in &converted.simulated_events {
                if let ArenaEvent::CarHitCar(hit) = &e.event {
                    if hit.is_demo {
                        let tick = converted.timeline_tick as i64
                            - (converted.state.tick_count as i64 - e.arena_tick as i64);
                        simulated.push((tick.max(0) as u64, hit.victim_car_idx));
                    }
                }
            }
        }
        let both = simulated
            .iter()
            .filter(|(t, v)| observed.iter().any(|(ot, ov)| ov == v && ot.abs_diff(*t) <= 30))
            .count();
        tot_obs += observed.len();
        tot_sim += simulated.len();
        tot_both += both;
        println!(
            "{} observed {} simulated {} both {}",
            replay.file_name().unwrap().to_string_lossy().chars().take(8).collect::<String>(),
            observed.len(),
            simulated.len(),
            both
        );
    }
    println!("total observed {tot_obs} simulated {tot_sim} both {tot_both}");
    Ok(())
}
