//! Multi-car RocketSim rollout test against an RLBot state recording (`states.jsonl`, one GamePacket
//! per physics frame with every car's true state and `last_input`).
//!
//! For every run of consecutive Active frames, all cars and the ball are set from the packet at frame
//! n (flags, boost, flip state from the packet), RocketSim is stepped H ticks with the recorded inputs,
//! and every car is compared with the packet at n + H. Input alignment variants: the input listed in
//! the packet of the tick being simulated (`next`: packet n+1+t) or the previous one (`same`: n+t).
//! Separates physics and input-alignment error from what the converter has to infer.
//!
//! usage: rlbot_onestep <states.jsonl> [max_packets]

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

use glam::{Mat3A, Vec3A};
use rocketsim::{
    Arena, ArenaConfig, BallState, CarBodyConfig, CarControls, CarState, GameMode, Team,
};
use serde_json::Value;

struct Player {
    pos: Vec3A,
    vel: Vec3A,
    rot: Mat3A,
    ang: Vec3A,
    boost: f32,
    air_state: u64,
    has_jumped: bool,
    has_double_jumped: bool,
    has_dodged: bool,
    dodge_timeout: f32,
    dodge_elapsed: f32,
    dodge_dir: (f32, f32),
    demolished: bool,
    supersonic: bool,
    team: u8,
    is_bot: bool,
    controls: CarControls,
}

struct Packet {
    frame: u64,
    phase: u64,
    players: Vec<Player>,
    ball_pos: Vec3A,
    ball_vel: Vec3A,
    ball_rot: Mat3A,
    ball_ang: Vec3A,
}

fn vec3(v: &Value) -> Vec3A {
    Vec3A::new(
        v["x"].as_f64().unwrap_or(0.0) as f32,
        v["y"].as_f64().unwrap_or(0.0) as f32,
        v["z"].as_f64().unwrap_or(0.0) as f32,
    )
}

/// RLBot Euler angles (radians; pitch up, yaw counter-clockwise from +x seen from above, roll right)
/// to the rotation matrix with columns forward, right, up (the Unreal rotator convention).
fn matrix(v: &Value) -> Mat3A {
    let (p, y, r) = (
        v["pitch"].as_f64().unwrap_or(0.0) as f32,
        v["yaw"].as_f64().unwrap_or(0.0) as f32,
        v["roll"].as_f64().unwrap_or(0.0) as f32,
    );
    let (cp, sp, cy, sy, cr, sr) = (p.cos(), p.sin(), y.cos(), y.sin(), r.cos(), r.sin());
    Mat3A::from_cols(
        Vec3A::new(cp * cy, cp * sy, sp),
        Vec3A::new(sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp),
        Vec3A::new(-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp),
    )
}

fn rotation_error(a: Mat3A, b: Mat3A) -> f32 {
    let trace = a.x_axis.dot(b.x_axis) + a.y_axis.dot(b.y_axis) + a.z_axis.dot(b.z_axis);
    ((trace - 1.0) * 0.5).clamp(-1.0, 1.0).acos().to_degrees()
}

fn quantile(values: &mut [f32], q: f64) -> f32 {
    if values.is_empty() {
        return f32::NAN;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn parse(line: &str) -> Option<Packet> {
    let row: Value = serde_json::from_str(line).ok()?;
    let p = &row["packet"];
    let info = &p["match_info"];
    let mut players = Vec::new();
    for pl in p["players"].as_array()? {
        let li = &pl["last_input"];
        let f = |k: &str| li[k].as_f64().unwrap_or(0.0) as f32;
        let b = |k: &str| li[k].as_bool().unwrap_or(false);
        players.push(Player {
            pos: vec3(&pl["physics"]["location"]),
            vel: vec3(&pl["physics"]["velocity"]),
            rot: matrix(&pl["physics"]["rotation"]),
            ang: vec3(&pl["physics"]["angular_velocity"]),
            boost: pl["boost"].as_f64().unwrap_or(0.0) as f32,
            air_state: pl["air_state"].as_u64().unwrap_or(0),
            has_jumped: pl["has_jumped"].as_bool().unwrap_or(false),
            has_double_jumped: pl["has_double_jumped"].as_bool().unwrap_or(false),
            has_dodged: pl["has_dodged"].as_bool().unwrap_or(false),
            dodge_timeout: pl["dodge_timeout"].as_f64().unwrap_or(-1.0) as f32,
            dodge_elapsed: pl["dodge_elapsed"].as_f64().unwrap_or(0.0) as f32,
            dodge_dir: (
                pl["dodge_dir"]["x"].as_f64().unwrap_or(0.0) as f32,
                pl["dodge_dir"]["y"].as_f64().unwrap_or(0.0) as f32,
            ),
            demolished: pl["demolished_timeout"].as_f64().unwrap_or(-1.0) >= 0.0,
            supersonic: pl["is_supersonic"].as_bool().unwrap_or(false),
            team: pl["team"].as_u64().unwrap_or(0) as u8,
            is_bot: pl["is_bot"].as_bool().unwrap_or(true),
            controls: CarControls {
                throttle: f("throttle"),
                steer: f("steer"),
                pitch: f("pitch"),
                yaw: f("yaw"),
                roll: f("roll"),
                jump: b("jump"),
                boost: b("boost"),
                handbrake: b("handbrake"),
            },
        });
    }
    let ball = &p["balls"][0]["physics"];
    Some(Packet {
        frame: info["frame_num"].as_u64()?,
        phase: info["match_phase"].as_u64()?,
        players,
        ball_pos: vec3(&ball["location"]),
        ball_vel: vec3(&ball["velocity"]),
        ball_rot: matrix(&ball["rotation"]),
        ball_ang: vec3(&ball["angular_velocity"]),
    })
}

/// Car state from a packet; `jump_ticks` counts the consecutive frames the player was `Jumping`.
fn car_state(pl: &Player, jump_ticks: u32, handbrake_val: f32) -> CarState {
    let mut s = CarState::default();
    s.phys.pos = pl.pos;
    s.phys.vel = pl.vel;
    s.phys.rot_mat = pl.rot;
    s.phys.ang_vel = pl.ang;
    s.boost = pl.boost;
    let on_ground = pl.air_state == 0 || pl.air_state == 1;
    s.is_on_ground = on_ground;
    s.wheels_with_contact = [on_ground.then(rocketsim::RaycastHitInfo::default); 4];
    s.has_jumped = pl.has_jumped;
    s.has_double_jumped = pl.has_double_jumped;
    s.has_flipped = pl.has_dodged;
    s.is_flipping = pl.air_state == 3;
    if pl.has_dodged {
        let dir = glam::Vec2::new(pl.dodge_dir.0, pl.dodge_dir.1);
        s.flip_rel_torque = Vec3A::new(-dir.y, dir.x, 0.0);
        s.flip_time = pl.dodge_elapsed;
    }
    s.is_jumping = pl.air_state == 1;
    s.jump_ticks = if s.is_jumping { jump_ticks } else { 0 };
    if pl.dodge_timeout >= 0.0 {
        s.air_time_since_jump = 1.25 - pl.dodge_timeout;
        s.air_time = s.air_time_since_jump;
    }
    s.is_boosting = pl.controls.boost && pl.boost > 0.0;
    s.is_supersonic = pl.supersonic;
    s.handbrake_val = handbrake_val;
    s
}

#[derive(Default)]
struct Rows {
    pos: Vec<f32>,
    vel: Vec<f32>,
    rot: Vec<f32>,
    ang: Vec<f32>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let path = PathBuf::from(
        args.next()
            .ok_or("usage: rlbot_onestep <states.jsonl> [max]")?,
    );
    let max_packets: usize = args
        .next()
        .and_then(|a| a.parse().ok())
        .unwrap_or(usize::MAX);
    if path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut packets: Vec<Packet> = Vec::new();
    for line in BufReader::new(File::open(&path)?).lines() {
        if packets.len() >= max_packets {
            break;
        }
        let Some(packet) = parse(&line?) else {
            continue;
        };
        // Duplicate frame numbers (repeated packets) are dropped.
        if packets.last().is_some_and(|p| p.frame == packet.frame) {
            continue;
        }
        packets.push(packet);
    }
    println!("{} distinct-frame packets", packets.len());
    let n_players = packets[0].players.len();
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    for pl in &packets[0].players {
        arena.add_car(
            if pl.team == 0 {
                Team::Blue
            } else {
                Team::Orange
            },
            CarBodyConfig::OCTANE,
        );
    }
    // Consecutive frames the player has been Jumping, per packet and player.
    let mut jumping_run: Vec<Vec<u32>> = vec![vec![0; n_players]; packets.len()];
    for i in 1..packets.len() {
        for k in 0..n_players {
            if packets[i].players[k].air_state == 1 && packets[i - 1].players[k].air_state == 1 {
                jumping_run[i][k] = jumping_run[i - 1][k] + 1;
            }
        }
    }

    // RocketSim's smoothed handbrake, tracked along the recording with its own rates (the packet has
    // only the handbrake button): +5 per second while held, -2 per second otherwise.
    let mut handbrake: Vec<Vec<f32>> = vec![vec![0.0; n_players]; packets.len()];
    for i in 1..packets.len() {
        let ticks = (packets[i].frame - packets[i - 1].frame).min(120) as f32;
        for k in 0..n_players {
            let rate = if packets[i].players[k].controls.handbrake {
                5.0
            } else {
                -2.0
            };
            handbrake[i][k] = (handbrake[i - 1][k] + rate * ticks / 120.0).clamp(0.0, 1.0);
        }
    }
    let horizons = [1usize, 4, 12];
    let variants = ["input of packet n+t (same)", "input of packet n+t+1 (next)"];
    let mut rows: BTreeMap<String, Rows> = BTreeMap::new();
    let stride = 1;
    for n in (0..packets.len().saturating_sub(13)).step_by(stride) {
        let contiguous = |h: usize| {
            (0..=h).all(|t| packets[n + t].frame == packets[n].frame + t as u64)
                && (0..=h).all(|t| packets[n + t].phase == 3)
        };
        for &h in &horizons {
            if !contiguous(h) {
                continue;
            }
            for (vi, variant) in variants.iter().enumerate() {
                let start = &packets[n];
                for (k, pl) in start.players.iter().enumerate() {
                    let mut state = car_state(pl, jumping_run[n][k] + 1, handbrake[n][k]);
                    state.is_demoed = pl.demolished;
                    arena.set_car_state(k, state);
                }
                let mut ball = BallState::default();
                ball.phys.pos = start.ball_pos;
                ball.phys.vel = start.ball_vel;
                ball.phys.rot_mat = start.ball_rot;
                ball.phys.ang_vel = start.ball_ang;
                arena.set_ball_state(ball);
                for t in 0..h {
                    for k in 0..n_players {
                        arena.set_car_controls(k, packets[n + t + vi].players[k].controls);
                    }
                    arena.step_tick();
                }
                let end = &packets[n + h];
                for k in 0..n_players {
                    let (a, b) = (&packets[n].players[k], &end.players[k]);
                    if a.demolished || b.demolished {
                        continue;
                    }
                    let got = arena.get_car_state(k);
                    let any_boost = (0..=h).any(|t| packets[n + t].players[k].controls.boost);
                    let near_ball = (a.pos - packets[n].ball_pos).length() < 300.0;
                    let near_car = packets[n]
                        .players
                        .iter()
                        .enumerate()
                        .any(|(o, p)| o != k && (p.pos - a.pos).length() < 250.0);
                    let states: Vec<u64> = (0..=h)
                        .map(|t| packets[n + t].players[k].air_state)
                        .collect();
                    let handbrake = (0..=h).any(|t| packets[n + t].players[k].controls.handbrake);
                    let jump_label = format!("jump {}->{}", states[0], states[h]);
                    let category = if states.iter().any(|&s| s == 3) {
                        "flip"
                    } else if states.iter().any(|&s| s == 1) {
                        if h == 1 {
                            jump_label.as_str()
                        } else {
                            "jump window"
                        }
                    } else if states.iter().any(|&s| s == 2) {
                        "double jump"
                    } else if states.iter().all(|&s| s == 0) {
                        if handbrake {
                            "ground, handbrake"
                        } else if any_boost {
                            "ground, boosting"
                        } else {
                            "ground, no boost"
                        }
                    } else if states.iter().all(|&s| s == 4) {
                        if any_boost {
                            "air, boosting"
                        } else {
                            "air, no boost"
                        }
                    } else {
                        "landing or takeoff"
                    };
                    if h == 1
                        && vi == 1
                        && env::var_os("JUMP_TRACE").is_some()
                        && category == "jump 1->1"
                    {
                        static COUNT: std::sync::atomic::AtomicUsize =
                            std::sync::atomic::AtomicUsize::new(0);
                        if (got.phys.vel - b.vel).length() > 100.0
                            && COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 14
                        {
                            println!(
                                "frame {} p{k}: vel start {:?} sim {:?} true {:?} ctl {:?}
   run {} jump_ticks {} input jump next {} | z vel start {:.1} sim {:.1} true {:.1} | pos z start {:.2} sim {:.2} true {:.2} | ground? {} wheels {}",
                                packets[n].frame,
                                a.vel.to_array(),
                                got.phys.vel.to_array(),
                                b.vel.to_array(),
                                (packets[n + 1].players[k].controls.throttle, packets[n + 1].players[k].controls.steer, packets[n + 1].players[k].controls.pitch, packets[n + 1].players[k].controls.yaw, packets[n + 1].players[k].controls.roll, packets[n + 1].players[k].controls.handbrake),
                                jumping_run[n][k],
                                jumping_run[n][k] + 1,
                                packets[n + 1].players[k].controls.jump,
                                a.vel.z,
                                got.phys.vel.z,
                                b.vel.z,
                                a.pos.z,
                                got.phys.pos.z,
                                b.pos.z,
                                got.is_on_ground,
                                got.wheels_with_contact.iter().flatten().count()
                            );
                        }
                    }
                    for group in [
                        "all".to_string(),
                        if a.is_bot { "bot cars".to_string() } else { "human cars".to_string() },
                        category.to_string(),
                        if near_car {
                            "near another car".to_string()
                        } else if near_ball {
                            "near ball".to_string()
                        } else {
                            "isolated".to_string()
                        },
                    ] {
                        let r = rows
                            .entry(format!("H={h:>2} | {variant:<30} | {group}"))
                            .or_default();
                        r.pos.push((got.phys.pos - b.pos).length());
                        r.vel.push((got.phys.vel - b.vel).length());
                        r.rot.push(rotation_error(got.phys.rot_mat, b.rot));
                        let mut ang = got.phys.ang_vel;
                        if ang.length() > 5.5 {
                            ang *= 5.5 / ang.length();
                        }
                        r.ang.push((ang - b.ang).length());
                    }
                }
            }
        }
    }
    println!(
        "{:<72} {:>6} | {:>16} | {:>16} | {:>14} | {:>14}",
        "horizon | inputs | group",
        "n",
        "pos UU p50/90/99",
        "vel UU/s p50/90",
        "rot deg p50/90",
        "ang rad/s p50/90"
    );
    for (label, r) in rows.iter_mut() {
        println!(
            "{:<72} {:>6} | {:>5.2}/{:>5.2}/{:>5.1} | {:>6.1}/{:>7.1} | {:>6.3}/{:>6.3} | {:>6.3}/{:>6.3}",
            label,
            r.pos.len(),
            quantile(&mut r.pos, 0.5),
            quantile(&mut r.pos, 0.9),
            quantile(&mut r.pos, 0.99),
            quantile(&mut r.vel, 0.5),
            quantile(&mut r.vel, 0.9),
            quantile(&mut r.rot, 0.5),
            quantile(&mut r.rot, 0.9),
            quantile(&mut r.ang, 0.5),
            quantile(&mut r.ang, 0.9),
        );
    }
    Ok(())
}
