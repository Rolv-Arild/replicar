//! Truth-free consistency counts of a split (`TEST_PROTOCOL.md` section 5), from one default conversion of
//! every replay: boost pickups matched to RocketSim's pad list and those whose car's path reaches the pad,
//! simulated touches inside a ball-contact interval and contact intervals holding a simulated touch,
//! replays detected as lag-free, the demolition counts (as `count_demolitions`) and the scoreboard check
//! (as `check_scoreboard` with `CONVERTED`). Per replay and in total.
//!
//! usage: `consistency_counts <dir or replay> [--final-assessment]`
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes, infer_packet_lags};
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
            let p = entry?.path();
            if p.extension().is_some_and(|e| e == "replay") {
                replay_to_rocketsim::ensure_unsealed(&p, final_assessment)?;
                result.push(p);
            }
        }
    }
    result.sort();
    Ok(result)
}

#[derive(Default, Clone, Copy)]
struct Counts {
    replays: usize,
    lag_free_replays: usize,
    pickups: usize,
    pickups_on_pad_list: usize,
    pickups_path_reaches_pad: usize,
    sim_touches: usize,
    sim_touches_in_contact: usize,
    contacts: usize,
    contacts_with_sim_touch: usize,
    demolitions: usize,
    demolition_repeats: usize,
    demolition_no_linked_car: usize,
    goal_explosions: usize,
    simulated_demolitions: usize,
    running_frames: usize,
    running_off_by_one: usize,
    running_off_more: usize,
    running_without_value: usize,
}

impl Counts {
    fn add(&mut self, other: &Counts) {
        self.replays += other.replays;
        self.lag_free_replays += other.lag_free_replays;
        self.pickups += other.pickups;
        self.pickups_on_pad_list += other.pickups_on_pad_list;
        self.pickups_path_reaches_pad += other.pickups_path_reaches_pad;
        self.sim_touches += other.sim_touches;
        self.sim_touches_in_contact += other.sim_touches_in_contact;
        self.contacts += other.contacts;
        self.contacts_with_sim_touch += other.contacts_with_sim_touch;
        self.demolitions += other.demolitions;
        self.demolition_repeats += other.demolition_repeats;
        self.demolition_no_linked_car += other.demolition_no_linked_car;
        self.goal_explosions += other.goal_explosions;
        self.simulated_demolitions += other.simulated_demolitions;
        self.running_frames += other.running_frames;
        self.running_off_by_one += other.running_off_by_one;
        self.running_off_more += other.running_off_more;
        self.running_without_value += other.running_without_value;
    }

    fn print(&self, label: &str) {
        let pct = |n: usize, d: usize| {
            if d == 0 {
                "n/a".to_owned()
            } else {
                format!("{:.1}%", 100.0 * n as f64 / d as f64)
            }
        };
        println!("{label}:");
        println!(
            "  replays {}, detected as lag-free {}",
            self.replays, self.lag_free_replays
        );
        println!(
            "  boost pickups (new, with an instigator) {}: pad matched to RocketSim's list {} ({}), the car's path reaches the pad {} ({})",
            self.pickups,
            self.pickups_on_pad_list,
            pct(self.pickups_on_pad_list, self.pickups),
            self.pickups_path_reaches_pad,
            pct(self.pickups_path_reaches_pad, self.pickups)
        );
        println!(
            "  simulated touches {}: inside a ball-contact interval {} ({}); ball-contact intervals {}: holding a simulated touch {} ({})",
            self.sim_touches,
            self.sim_touches_in_contact,
            pct(self.sim_touches_in_contact, self.sim_touches),
            self.contacts,
            self.contacts_with_sim_touch,
            pct(self.contacts_with_sim_touch, self.contacts)
        );
        println!(
            "  demolitions: non-goal events {} = linked victim and not a repeat {} + repeats {} + no linked car {}; post-goal explosions {} (excluded); simulated (is_demo) {} (0 by construction while RocketSim's own rule is off)",
            self.demolitions + self.demolition_repeats + self.demolition_no_linked_car,
            self.demolitions,
            self.demolition_repeats,
            self.demolition_no_linked_car,
            self.goal_explosions,
            self.simulated_demolitions
        );
        let exact = self.running_frames - self.running_off_by_one - self.running_off_more;
        println!(
            "  scoreboard, running frames with a shown clock {}: integer equals the ceiling of the reconstruction {} ({}), differs by one {} ({}), by more {} ({}); running frames without a value {}",
            self.running_frames,
            exact,
            pct(exact, self.running_frames),
            self.running_off_by_one,
            pct(self.running_off_by_one, self.running_frames),
            self.running_off_more,
            pct(self.running_off_more, self.running_frames),
            self.running_without_value
        );
    }
}

fn count_replay(bytes: &[u8], options: &ConvertOptions) -> Result<Counts, Box<dyn Error>> {
    let output = convert_bytes(bytes, options)?;
    let mut c = Counts {
        replays: 1,
        ..Counts::default()
    };
    // Lag-free detection is part of the lag inference of the conversion; run it on the same observations.
    c.lag_free_replays = usize::from(infer_packet_lags(&output.observations, options).lag_free);
    let (mut touches, mut contacts): (Vec<u64>, Vec<(u64, u64)>) = (Vec::new(), Vec::new());
    for (frame, converted) in output.observations.frames.iter().zip(&output.frames) {
        for p in &converted.boost_pickups {
            c.pickups += 1;
            c.pickups_on_pad_list += usize::from(p.pad_index.is_some());
            c.pickups_path_reaches_pad += usize::from(p.verified);
        }
        touches.extend(converted.touches.iter().map(|t| t.tick));
        contacts.extend(
            converted
                .ball_contacts
                .iter()
                .map(|b| (b.tick_from, b.tick_to)),
        );
        c.contacts_with_sim_touch += converted
            .ball_contacts
            .iter()
            .filter(|b| b.simulated_touch)
            .count();
        for event in &frame.events {
            if let Event::Demolish {
                source,
                victim_car,
                repeat,
                ..
            } = event
            {
                if *source == "goal_explosion" {
                    c.goal_explosions += 1;
                } else if *repeat {
                    c.demolition_repeats += 1;
                } else {
                    let linked = victim_car.is_some_and(|v| {
                        frame
                            .cars
                            .iter()
                            .any(|car| car.actor_id == v && car.player_key.is_some())
                    });
                    if linked {
                        c.demolitions += 1;
                    } else {
                        c.demolition_no_linked_car += 1;
                    }
                }
            }
        }
        c.simulated_demolitions += converted
            .simulated_events
            .iter()
            .filter(|e| matches!(&e.event, ArenaEvent::CarHitCar(hit) if hit.is_demo))
            .count();
        if let Some(sb) = &converted.scoreboard
            && sb.clock_state == "running"
        {
            let overtime = sb.period == "overtime";
            let shown = frame.seconds_remaining.as_ref().map(|v| v.value);
            let clock = if overtime {
                sb.overtime_seconds
            } else {
                sb.seconds_remaining
            };
            if let (Some(shown), Some(clock)) = (shown, clock) {
                c.running_frames += 1;
                match (shown - clock.ceil() as i32).abs() {
                    0 => {}
                    1 => c.running_off_by_one += 1,
                    _ => c.running_off_more += 1,
                }
            } else {
                c.running_without_value += 1;
            }
        }
    }
    // The converter's own interval rule: a touch within 2 ticks of the contact interval.
    c.sim_touches = touches.len();
    c.sim_touches_in_contact = touches
        .iter()
        .filter(|&&t| {
            contacts
                .iter()
                .any(|&(from, to)| t + 2 >= from && t <= to + 2)
        })
        .count();
    c.contacts = contacts.len();
    Ok(c)
}

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "usage: consistency_counts <dir or replay> [--final-assessment]";
    let path = PathBuf::from(env::args().nth(1).ok_or(usage)?);
    // The test split is sealed until the frozen assessment (TEST_PROTOCOL.md); only that run passes the flag.
    if replay_to_rocketsim::sealed_path_refused(
        &path,
        env::args().any(|arg| arg == "--final-assessment"),
    ) {
        return Err("refusing to inspect a path with a 'test' component (pass --final-assessment for the frozen run)".into());
    }
    let options = ConvertOptions::default();
    let mut total = Counts::default();
    for replay in replay_paths(&path, env::args_os().any(|arg| arg == "--final-assessment"))? {
        let counts = count_replay(&fs::read(&replay)?, &options)?;
        let name = replay
            .file_name()
            .unwrap()
            .to_string_lossy()
            .chars()
            .take(8)
            .collect::<String>();
        println!(
            "{name} lag-free {} | pickups {} on-list {} reach {} | touches {} in-contact {} | contacts {} with-touch {} | demolitions {} repeat {} unlinked {} sim {} | running {} off1 {} off2+ {}",
            counts.lag_free_replays,
            counts.pickups,
            counts.pickups_on_pad_list,
            counts.pickups_path_reaches_pad,
            counts.sim_touches,
            counts.sim_touches_in_contact,
            counts.contacts,
            counts.contacts_with_sim_touch,
            counts.demolitions,
            counts.demolition_repeats,
            counts.demolition_no_linked_car,
            counts.simulated_demolitions,
            counts.running_frames,
            counts.running_off_by_one,
            counts.running_off_more
        );
        total.add(&counts);
    }
    total.print(&format!("total over {}", path.display()));
    Ok(())
}
