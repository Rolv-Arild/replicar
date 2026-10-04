//! Write a replay's observations as JSON lines, one line per frame, for analysis in other tools.
//!
//! Each line: frame index, time, game state, the ball's position/velocity with their freshness frames,
//! and for every car with a player key: actor id, player key, team, position, rotation, linear velocity,
//! angular velocity (each with the frame it was last updated), and the replicated controls (throttle,
//! steer, handbrake and the boost, jump, double jump and dodge counters, each with its freshness frame).
//! Also the players' names by key on the first line, and (unless `--no-lags`) the converter's inferred
//! packet lags of each frame (`lags`: actor id or null for the ball, ticks, source).
//!
//! usage: dump_replay_packets <replay> <out.jsonl> [--no-lags]

use std::env;
use std::error::Error;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::PathBuf;

use replay_to_rocketsim::conversion::{ConvertOptions, convert_observations};
use replay_to_rocketsim::observations::{Value, extract};
use serde_json::{Value as Json, json};

fn stamped<T: serde::Serialize>(v: &Option<Value<T>>) -> Json {
    match v {
        Some(v) => json!([v.value, v.frame]),
        None => Json::Null,
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let replay_path = PathBuf::from(
        args.next()
            .ok_or("usage: dump_replay_packets <replay> <out>")?,
    );
    let out_path = PathBuf::from(
        args.next()
            .ok_or("usage: dump_replay_packets <replay> <out>")?,
    );
    if replay_to_rocketsim::sealed_path_refused(&replay_path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let replay = boxcars::ParserBuilder::new(&fs::read(&replay_path)?)
        .must_parse_network_data()
        .parse()?;
    let observed = extract(&replay).ok_or("no network frames")?;
    let with_lags = !env::args().any(|a| a == "--no-lags");
    let lags: Vec<Json> = if with_lags {
        let options = ConvertOptions::default();
        let output = convert_observations(observed.clone(), &options)?;
        output
            .frames
            .iter()
            .map(|f| {
                Json::Array(
                    f.packet_lags
                        .iter()
                        .map(|l| json!({"actor": l.actor_id, "ticks": l.ticks, "source": l.source}))
                        .collect(),
                )
            })
            .collect()
    } else {
        Vec::new()
    };
    let mut out = BufWriter::new(File::create(&out_path)?);
    let names: Vec<Json> = observed
        .frames
        .iter()
        .flat_map(|f| f.players.iter())
        .map(|p| json!({"key": p.key, "name": p.name, "team": p.team}))
        .collect();
    let mut seen = std::collections::HashSet::new();
    let names: Vec<Json> = names
        .into_iter()
        .filter(|n| seen.insert(n["key"].as_str().unwrap_or("").to_string()))
        .collect();
    writeln!(out, "{}", json!({"players": names}))?;
    for frame in &observed.frames {
        let cars: Vec<Json> = frame
            .cars
            .iter()
            .filter(|c| c.player_key.is_some())
            .map(|c| {
                json!({
                    "actor": c.actor_id,
                    "created": c.actor_created_frame,
                    "key": c.player_key,
                    "team": c.team,
                    "pos": stamped(&c.body.position),
                    "rot": stamped(&c.body.rotation_xyzw),
                    "vel": stamped(&c.body.linear_velocity),
                    "ang": stamped(&c.body.angular_velocity_replay_units),
                    "throttle": stamped(&c.inputs.throttle),
                    "steer": stamped(&c.inputs.steer),
                    "handbrake": stamped(&c.inputs.handbrake),
                    "boost": stamped(&c.inputs.boost_active_raw),
                    "jump": stamped(&c.inputs.jump_active_raw),
                    "dbl": stamped(&c.inputs.double_jump_active_raw),
                    "dodge": stamped(&c.inputs.dodge_active_raw),
                    "torque": stamped(&c.inputs.dodge_torque_replay_units),
                    "boost_amount": stamped(&c.boost_raw),
                })
            })
            .collect();
        let ball = frame
            .ball
            .as_ref()
            .map(|b| json!({"pos": stamped(&b.position), "vel": stamped(&b.linear_velocity)}));
        writeln!(
            out,
            "{}",
            json!({
                "f": frame.index,
                "t": frame.time,
                "state": frame.game_state.as_ref().map(|g| g.value.clone()),
                "ball": ball,
                "cars": cars,
                "lags": lags.get(frame.index).cloned().unwrap_or(Json::Null),
            })
        )?;
    }
    Ok(())
}
