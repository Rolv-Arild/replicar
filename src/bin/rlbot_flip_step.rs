//! Headroom of a per-flip cancel *step* over a constant cancel, on RLBot recordings.
//!
//! For each forward or backward dodge in a recording, the flip is simulated from the first packet with
//! `has_dodged` (all cars, true inputs) for H ticks and compared with the recorded packet at H: (a) with
//! the true inputs, (b) with no cancel, (c) the best constant cancel over the span (0, .25 .. 1, chosen
//! against the target: in sample), (d) the best step (value c from tick t0, chosen against the target:
//! in sample). Whatever (d) gains over (c) is the most a two-parameter step could add.
//!
//! usage: rlbot_flip_step <states.jsonl> [max_packets]

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
fn car_state(pl: &Player, jump_ticks: u32, handbrake_val: f32, _boosting_time: f32) -> CarState {
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

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let path = PathBuf::from(
        args.next()
            .ok_or("usage: rlbot_flip_step <states.jsonl> [max]")?,
    );
    let max_packets: usize = args
        .next()
        .and_then(|a| a.parse().ok())
        .unwrap_or(usize::MAX);
    if replay_to_rocketsim::sealed_path_refused(&path, false) {
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
        if packets.last().is_some_and(|p| p.frame == packet.frame) {
            continue;
        }
        packets.push(packet);
    }
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
    let mut jumping_run: Vec<Vec<u32>> = vec![vec![0; n_players]; packets.len()];
    let mut handbrake: Vec<Vec<f32>> = vec![vec![0.0; n_players]; packets.len()];
    for i in 1..packets.len() {
        let ticks = (packets[i].frame - packets[i - 1].frame).min(120) as f32;
        for k in 0..n_players {
            if packets[i].players[k].air_state == 1 && packets[i - 1].players[k].air_state == 1 {
                jumping_run[i][k] = jumping_run[i - 1][k] + 1;
            }
            let rate = if packets[i].players[k].controls.handbrake {
                5.0
            } else {
                -2.0
            };
            handbrake[i][k] = (handbrake[i - 1][k] + rate * ticks / 120.0).clamp(0.0, 1.0);
        }
    }
    let horizons = [8usize, 12, 16, 24];
    let h_max = 24;
    let cancels = [0.0f32, 0.25, 0.5, 0.75, 1.0];
    let starts = [0usize, 4, 8, 12, 16];
    // rows[(group, variant, horizon)] -> (rotation errors, angular velocity errors)
    let mut rows: BTreeMap<(String, usize, usize), (Vec<f32>, Vec<f32>)> = BTreeMap::new();
    let variants = [
        "true inputs",
        "no cancel",
        "best constant (in sample)",
        "best step (in sample)",
    ];
    let mut flips = 0;
    for n in 1..packets.len().saturating_sub(h_max + 1) {
        if !(0..=h_max).all(|t| {
            packets[n + t].frame == packets[n].frame + t as u64 && packets[n + t].phase == 3
        }) {
            continue;
        }
        for k in 0..n_players {
            let (a, prev) = (&packets[n].players[k], &packets[n - 1].players[k]);
            if !(a.has_dodged && !prev.has_dodged) || a.dodge_dir.0.abs() < 0.3 || a.demolished {
                continue;
            }
            let sign = if a.dodge_dir.0 >= 0.0 { 1.0 } else { -1.0 };
            flips += 1;
            let group = if a.is_bot { "bots" } else { "human" };
            let mut run = |h: usize, pitch: &dyn Fn(usize) -> Option<f32>| -> (f32, f32) {
                for (j, pl) in packets[n].players.iter().enumerate() {
                    let mut state = car_state(pl, jumping_run[n][j] + 1, handbrake[n][j], 0.0);
                    state.is_demoed = pl.demolished;
                    arena.set_car_state(j, state);
                }
                let mut ball = BallState::default();
                ball.phys.pos = packets[n].ball_pos;
                ball.phys.vel = packets[n].ball_vel;
                ball.phys.rot_mat = packets[n].ball_rot;
                ball.phys.ang_vel = packets[n].ball_ang;
                arena.set_ball_state(ball);
                for t in 1..=h {
                    for j in 0..n_players {
                        let mut c = packets[n + t].players[j].controls;
                        if j == k {
                            if let Some(p) = pitch(t) {
                                c.pitch = p;
                            }
                        }
                        arena.set_car_controls(j, c);
                    }
                    arena.step_tick();
                }
                let got = arena.get_car_state(k);
                let target = &packets[n + h].players[k];
                let mut ang = got.phys.ang_vel;
                if ang.length() > 5.5 {
                    ang *= 5.5 / ang.length();
                }
                (
                    rotation_error(got.phys.rot_mat, target.rot),
                    (ang - target.ang).length(),
                )
            };
            for &h in &horizons {
                let score = |e: (f32, f32)| e.0 + 10.0 * e.1;
                let truth = run(h, &|_| None);
                let none = run(h, &|_| Some(0.0));
                let mut best_c = (f32::INFINITY, (0.0, 0.0));
                let mut best_s = (f32::INFINITY, (0.0, 0.0));
                for &c in &cancels {
                    let e = run(h, &|_| Some(c * sign));
                    if score(e) < best_c.0 {
                        best_c = (score(e), e);
                    }
                    for &t0 in &starts {
                        let e = run(h, &|t| Some(if t > t0 { c * sign } else { 0.0 }));
                        if score(e) < best_s.0 {
                            best_s = (score(e), e);
                        }
                    }
                }
                for (v, e) in [(0, truth), (1, none), (2, best_c.1), (3, best_s.1)] {
                    let row = rows.entry((group.to_string(), v, h)).or_default();
                    row.0.push(e.0);
                    row.1.push(e.1);
                }
            }
        }
    }
    println!("{flips} forward/backward flips");
    for group in ["bots", "human"] {
        for &h in &horizons {
            println!("{group}, H = {h} ticks after the dodge packet");
            for (v, name) in variants.iter().enumerate() {
                if let Some(row) = rows.get_mut(&(group.to_string(), v, h)) {
                    let mean = |x: &Vec<f32>| x.iter().sum::<f32>() / x.len().max(1) as f32;
                    println!(
                        "  {name:<28} n {:>3}  rot deg mean {:>5.2} p50 {:>5.2} p90 {:>5.2}   ang rad/s mean {:>5.3} p50 {:>5.3} p90 {:>5.3}",
                        row.0.len(),
                        mean(&row.0),
                        quantile(&mut row.0, 0.5),
                        quantile(&mut row.0, 0.9),
                        mean(&row.1),
                        quantile(&mut row.1, 0.5),
                        quantile(&mut row.1, 0.9)
                    );
                }
            }
        }
    }
    Ok(())
}
