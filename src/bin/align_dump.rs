//! Align a BakkesMod state dump (JSON, one record per 4 physical ticks) to a replay's timeline.
//!
//! The dump holds the local player's car, its controller inputs and the ball. Free-flight ball packets
//! of the replay are exact server states at exact physical ticks (chain lags), so for each of them the
//! dump frame j and tick offset r (0..=3) that RocketSim's ball reproduces from the dump state are
//! found; the dump's tick grid is then `T0 + 4 j` on the replay timeline. Prints the matches, the
//! consistency of T0, and how car packets compare with the dump's car state where the ticks coincide.
//!
//! usage: align_dump <replay> <dump.json>

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::Vec3A;
use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes};
use rocketsim::{Arena, ArenaConfig, BallState, GameMode};
use serde_json::Value;

fn vec(v: &Value, keys: [&str; 3]) -> Vec3A {
    Vec3A::new(
        v[keys[0]].as_f64().unwrap_or(f64::NAN) as f32,
        v[keys[1]].as_f64().unwrap_or(f64::NAN) as f32,
        v[keys[2]].as_f64().unwrap_or(f64::NAN) as f32,
    )
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args_os().skip(1);
    let replay_path = PathBuf::from(
        args.next()
            .ok_or("usage: align_dump <replay> <dump.json>")?,
    );
    let dump_path = PathBuf::from(
        args.next()
            .ok_or("usage: align_dump <replay> <dump.json>")?,
    );
    if replay_to_rocketsim::sealed_path_refused(&replay_path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let output = convert_bytes(&fs::read(&replay_path)?, &ConvertOptions::default())?;
    let frames = &output.observations.frames;
    let first_time = frames[0].time;
    let tick_of =
        |f: usize| ((f64::from(frames[f].time) - f64::from(first_time)) * 120.0).round() as i64;

    let dump: Value = serde_json::from_str(&fs::read_to_string(&dump_path)?)?;
    let dump_frames = dump["frames"].as_array().ok_or("frames")?;
    let n = dump_frames.len();
    let ball_pos: Vec<Vec3A> = dump_frames
        .iter()
        .map(|f| vec(&f["ball"]["location"], ["X", "Y", "Z"]))
        .collect();
    let ball_vel: Vec<Vec3A> = dump_frames
        .iter()
        .map(|f| vec(&f["ball"]["Velocity"], ["X", "Y", "Z"]))
        .collect();
    println!(
        "replay frames {} (time {:.2}..{:.2} s); dump frames {n} ({:.1} s at 4 ticks)",
        frames.len(),
        frames[0].time,
        frames[frames.len() - 1].time,
        n as f32 * 4.0 / 120.0
    );

    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    let mut matches: Vec<(usize, i64, usize, i64)> = Vec::new(); // (frame, tick, dump j, r)
    for f in 0..frames.len() {
        let Some(ball) = frames[f].ball.as_ref() else {
            continue;
        };
        let (Some(p), Some(v)) = (
            ball.position.as_ref().filter(|x| x.frame == f),
            ball.linear_velocity.as_ref().filter(|x| x.frame == f),
        ) else {
            continue;
        };
        let Some(lag) = output.frames[f]
            .packet_lags
            .iter()
            .find(|l| l.actor_id.is_none() && l.source == "chain")
            .map(|l| l.ticks as i64)
        else {
            continue;
        };
        let (pos, vel) = (Vec3A::from_array(p.value), Vec3A::from_array(v.value));
        let speed = vel.length();
        let t_b = tick_of(f) - lag;
        for j in 0..n {
            // Within 3 ticks of travel (plus slack) of the dump position.
            if (ball_pos[j] - pos).length() > 3.0 * speed / 120.0 + 60.0 {
                continue;
            }
            let mut state = BallState::default();
            state.phys.pos = ball_pos[j];
            state.phys.vel = ball_vel[j];
            arena.set_ball_state(state);
            for r in 0..=3i64 {
                if r > 0 {
                    arena.step_tick();
                }
                let s = arena.get_ball_state();
                if (s.phys.pos - pos).length() < 0.6 && (s.phys.vel - vel).length() < 3.0 {
                    matches.push((f, t_b, j, r));
                    break;
                }
            }
        }
    }
    println!(
        "free-flight ball packets matched to a dump frame and tick offset: {}",
        matches.len()
    );
    let mut t0_counts: BTreeMap<i64, usize> = BTreeMap::new();
    for &(_, t_b, j, r) in &matches {
        *t0_counts.entry(t_b - r - 4 * j as i64).or_default() += 1;
    }
    let mut ranked: Vec<_> = t0_counts.iter().collect();
    ranked.sort_by_key(|(_, c)| std::cmp::Reverse(**c));
    println!("candidate T0 (replay timeline tick of dump frame 0), count:");
    for (t0, c) in ranked.iter().take(6) {
        println!("  T0 = {t0}: {c}");
    }
    println!("first matches (frame, physical tick, dump frame, r):");
    for m in matches.iter().take(8) {
        println!("  {m:?}");
    }
    Ok(())
}
