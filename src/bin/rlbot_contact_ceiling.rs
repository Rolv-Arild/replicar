//! How well can RocketSim reproduce a real ball touch when it starts from the exact true state?
//!
//! From the RLBot recording (`states.jsonl`: true state and `last_input` of every car and the ball at
//! every tick), every true touch (a change of a player's `latest_touch.game_seconds`, first seen in
//! packet T) is rolled out with RocketSim from the TRUE state of the ball and all cars at packet
//! T - lead (lead 1, 2, 4, 8) with the true inputs of every tick (packet n + t + 1 for the tick
//! n + t -> n + t + 1, the alignment `rlbot_onestep` found right), through T + 12. The ball velocity
//! and position at T + H (H = 0, 1, 4, 12) are compared with the truth. The hit impulse handling is the
//! converter default (`step_tick_with_hit_impulse(arena, false)`: the pinned RocketSim applies it).
//! Also: how often a `CarHitBall` of the toucher appears at the right tick, and the same rollouts with
//! the toucher's starting position or velocity perturbed by Gaussian noise (sensitivity to car state
//! error).
//!
//! usage: rlbot_contact_ceiling <states.jsonl> [meshes dir, default collision_meshes]

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

use glam::{Mat3A, Vec3A};
use replay_to_rocketsim::conversion::step_tick_with_hit_impulse;
use rocketsim::{
    Arena, ArenaConfig, ArenaEvent, BallState, CarBodyConfig, CarControls, CarState, GameMode, Team,
};
use serde_json::Value;

struct Player {
    name: String,
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
    hitbox: [f32; 3],
    hitbox_offset: [f32; 3],
    touch_seconds: Option<f64>,
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

/// RLBot Euler angles to the rotation matrix with columns forward, right, up.
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

fn parse(line: &str) -> Option<Packet> {
    let row: Value = serde_json::from_str(line).ok()?;
    let p = &row["packet"];
    let info = &p["match_info"];
    let mut players = Vec::new();
    for pl in p["players"].as_array()? {
        let li = &pl["last_input"];
        let f = |k: &str| li[k].as_f64().unwrap_or(0.0) as f32;
        let b = |k: &str| li[k].as_bool().unwrap_or(false);
        let hb = &pl["hitbox"];
        let ho = &pl["hitbox_offset"];
        players.push(Player {
            name: pl["name"].as_str().unwrap_or("").to_string(),
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
            hitbox: [
                hb["length"].as_f64().unwrap_or(0.0) as f32,
                hb["width"].as_f64().unwrap_or(0.0) as f32,
                hb["height"].as_f64().unwrap_or(0.0) as f32,
            ],
            hitbox_offset: [
                ho["x"].as_f64().unwrap_or(0.0) as f32,
                ho["y"].as_f64().unwrap_or(0.0) as f32,
                ho["z"].as_f64().unwrap_or(0.0) as f32,
            ],
            touch_seconds: pl["latest_touch"]["game_seconds"].as_f64(),
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

fn car_state(pl: &Player, jump_ticks: u32, handbrake_val: f32, boosting_time: f32) -> CarState {
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
    s.is_boosting = boosting_time > 0.0 || (pl.controls.boost && pl.boost > 0.0);
    s.boosting_time = boosting_time;
    s.is_supersonic = pl.supersonic;
    s.handbrake_val = handbrake_val;
    s
}

/// The RocketSim hitbox preset closest to the recorded hitbox.
fn preset(hitbox: [f32; 3]) -> CarBodyConfig {
    let all = [
        CarBodyConfig::OCTANE,
        CarBodyConfig::DOMINUS,
        CarBodyConfig::PLANK,
        CarBodyConfig::BREAKOUT,
        CarBodyConfig::HYBRID,
        CarBodyConfig::MERC,
        CarBodyConfig::PSYCLOPS,
    ];
    *all.iter()
        .min_by(|a, b| {
            let d = |c: &CarBodyConfig| {
                (c.hitbox_size.x - hitbox[0]).abs()
                    + (c.hitbox_size.y - hitbox[1]).abs()
                    + (c.hitbox_size.z - hitbox[2]).abs()
            };
            d(a).total_cmp(&d(b))
        })
        .unwrap()
}

/// splitmix64 + Box-Muller.
struct Rng(u64);
impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn uniform(&mut self) -> f32 {
        ((self.next_u64() >> 40) as f32 + 0.5) / (1u64 << 24) as f32
    }
    fn gauss(&mut self) -> f32 {
        let (u1, u2) = (self.uniform(), self.uniform());
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos()
    }
    fn vec(&mut self, sigma: f32) -> Vec3A {
        Vec3A::new(self.gauss(), self.gauss(), self.gauss()) * sigma
    }
}

const LEADS: [usize; 4] = [1, 2, 4, 8];
const HORIZONS: [usize; 4] = [0, 1, 4, 12];

#[derive(Clone)]
struct Sample {
    lead: usize,
    config: &'static str,
    free_air: bool,
    car_grounded: bool,
    carrying: bool,
    is_bot: bool,
    /// No other true touch between the touch and T + 12.
    clean: bool,
    /// Ball velocity and position error at each horizon.
    vel: [f32; 4],
    pos: [f32; 4],
}

#[derive(Clone, Copy)]
struct Config {
    name: &'static str,
    pos_sigma: f32,
    vel_sigma: f32,
}

const CONFIGS: [Config; 8] = [
    Config { name: "exact", pos_sigma: 0.0, vel_sigma: 0.0 },
    Config { name: "pos 1 UU", pos_sigma: 1.0, vel_sigma: 0.0 },
    Config { name: "pos 3 UU", pos_sigma: 3.0, vel_sigma: 0.0 },
    Config { name: "pos 6 UU", pos_sigma: 6.0, vel_sigma: 0.0 },
    Config { name: "vel 10 UU/s", pos_sigma: 0.0, vel_sigma: 10.0 },
    Config { name: "vel 30 UU/s", pos_sigma: 0.0, vel_sigma: 30.0 },
    Config { name: "pos 3 + vel 30", pos_sigma: 3.0, vel_sigma: 30.0 },
    Config { name: "pos 6 + vel 30", pos_sigma: 6.0, vel_sigma: 30.0 },
];
const DRAWS: usize = 5;

fn quantile(values: &mut Vec<f32>, q: f64) -> f32 {
    if values.is_empty() {
        return f32::NAN;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn summarize(label: &str, samples: &[&Sample], lead: usize, which: usize) {
    let group: Vec<&&Sample> = samples.iter().filter(|s| s.lead == lead).collect();
    if group.is_empty() {
        return;
    }
    print!("{label:<34} lead {lead} n {:>5} |", group.len());
    for (h, name) in HORIZONS.iter().enumerate() {
        let mut v: Vec<f32> = group.iter().map(|s| if which == 0 { s.vel[h] } else { s.pos[h] }).collect();
        let f = |t: f32| 100.0 * v.iter().filter(|x| **x < t).count() as f64 / v.len() as f64;
        let (a, b, c) = if which == 0 { (f(25.0), f(50.0), f(100.0)) } else { (f(1.0), f(3.0), f(10.0)) };
        let (p50, p90) = (quantile(&mut v, 0.5), quantile(&mut v, 0.9));
        print!(" H{name:<2} {a:>4.0}/{b:>4.0}/{c:>4.0}% p50 {p50:>6.1} p90 {p90:>7.1} |");
    }
    println!();
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let path = PathBuf::from(args.next().ok_or("usage: rlbot_contact_ceiling <states.jsonl> [meshes]")?);
    let meshes = args.next().unwrap_or_else(|| "collision_meshes".to_string());
    if replay_to_rocketsim::sealed_path_refused(&path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    rocketsim::init(Path::new(&meshes), true)?;
    let mut packets: Vec<Packet> = Vec::new();
    for line in BufReader::new(File::open(&path)?).lines() {
        let Some(packet) = parse(&line?) else { continue };
        if packets.last().is_some_and(|p| p.frame == packet.frame) {
            continue;
        }
        packets.push(packet);
    }
    let max_players = packets.iter().map(|p| p.players.len()).max().unwrap_or(0);
    packets.retain(|p| p.players.len() == max_players);
    let names: Vec<String> = packets[0].players.iter().map(|p| p.name.clone()).collect();
    // Re-order every packet's players by the first packet's names.
    for p in packets.iter_mut() {
        let mut ordered = Vec::new();
        for n in &names {
            if let Some(i) = p.players.iter().position(|pl| &pl.name == n) {
                ordered.push(p.players.swap_remove(i));
            }
        }
        p.players = ordered;
    }
    packets.retain(|p| p.players.len() == names.len());
    let n_players = names.len();
    println!("{} packets, {} players", packets.len(), n_players);
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    for pl in &packets[0].players {
        arena.add_car(if pl.team == 0 { Team::Blue } else { Team::Orange }, preset(pl.hitbox));
    }
    // Trackers as in rlbot_onestep.
    let mut jumping_run: Vec<Vec<u32>> = vec![vec![0; n_players]; packets.len()];
    let mut handbrake: Vec<Vec<f32>> = vec![vec![0.0; n_players]; packets.len()];
    let mut boosting_time: Vec<Vec<f32>> = vec![vec![0.0; n_players]; packets.len()];
    for i in 1..packets.len() {
        let ticks = (packets[i].frame - packets[i - 1].frame).min(120) as f32;
        for k in 0..n_players {
            if packets[i].players[k].air_state == 1 && packets[i - 1].players[k].air_state == 1 {
                jumping_run[i][k] = jumping_run[i - 1][k] + 1;
            }
            let rate = if packets[i].players[k].controls.handbrake { 5.0 } else { -2.0 };
            handbrake[i][k] = (handbrake[i - 1][k] + rate * ticks / 120.0).clamp(0.0, 1.0);
            let fell = packets[i].players[k].boost < packets[i - 1].players[k].boost - 0.05;
            if fell && packets[i].frame == packets[i - 1].frame + 1 {
                boosting_time[i][k] = boosting_time[i - 1][k] + 1.0 / 120.0;
            }
        }
    }
    // True touches: (packet index T, player index).
    let mut touches: Vec<(usize, usize)> = Vec::new();
    let mut last_seconds: Vec<Option<f64>> = vec![None; n_players];
    for (i, p) in packets.iter().enumerate() {
        for k in 0..n_players {
            let s = p.players[k].touch_seconds;
            if s.is_some() && last_seconds[k] != s {
                touches.push((i, k));
            }
            last_seconds[k] = s;
        }
    }
    // Skip the touch already present in the first packet of the recording.
    touches.retain(|&(i, _)| i > 0);
    println!("{} true touches", touches.len());

    let mut samples: Vec<Sample> = Vec::new();
    // CarHitBall timing: offset (event step - touch step) of the first hit of the toucher, by lead, exact config.
    let mut hit_offsets: BTreeMap<usize, Vec<Option<i64>>> = BTreeMap::new();
    let mut rng = Rng(0x1234_5678_9ABC_DEF0);
    for &(t_idx, toucher) in &touches {
        for &lead in &LEADS {
            if t_idx < lead || t_idx + 12 >= packets.len() {
                continue;
            }
            let n = t_idx - lead;
            let contiguous = (0..=(lead + 12)).all(|t| {
                packets[n + t].frame == packets[n].frame + t as u64 && packets[n + t].phase == 3
            });
            if !contiguous {
                continue;
            }
            let start = &packets[n];
            let pre = &packets[t_idx - 1];
            let free_air = {
                let b = pre.ball_pos;
                b.z > 150.0 && b.z < 1850.0 && b.x.abs() < 3800.0 && b.y.abs() < 4800.0
            };
            let car_grounded = pre.players[toucher].air_state == 0;
            let carrying = touches
                .iter()
                .any(|&(j, k)| k == toucher && j < t_idx && t_idx - j <= 40);
            let clean = !touches
                .iter()
                .any(|&(j, _)| j > t_idx && j <= t_idx + 12);
            let is_bot = start.players[toucher].is_bot;
            for config in CONFIGS {
                let draws = if config.pos_sigma == 0.0 && config.vel_sigma == 0.0 { 1 } else { DRAWS };
                for _ in 0..draws {
                    for (k, pl) in start.players.iter().enumerate() {
                        let mut state = car_state(pl, jumping_run[n][k] + 1, handbrake[n][k], boosting_time[n][k]);
                        state.is_demoed = pl.demolished;
                        if k == toucher {
                            state.phys.pos += rng.vec(config.pos_sigma);
                            state.phys.vel += rng.vec(config.vel_sigma);
                        }
                        arena.set_car_state(k, state);
                    }
                    let mut ball = BallState::default();
                    ball.phys.pos = start.ball_pos;
                    ball.phys.vel = start.ball_vel;
                    ball.phys.rot_mat = start.ball_rot;
                    ball.phys.ang_vel = start.ball_ang;
                    arena.set_ball_state(ball);
                    let mut vel = [0.0f32; 4];
                    let mut pos = [0.0f32; 4];
                    let mut first_hit: Option<i64> = None;
                    for t in 0..(lead + 12) {
                        for k in 0..n_players {
                            arena.set_car_controls(k, packets[n + t + 1].players[k].controls);
                        }
                        let events = step_tick_with_hit_impulse(&mut arena, false);
                        // This step ends at packet n + t + 1; the touch step ends at packet T.
                        if first_hit.is_none()
                            && events
                                .iter()
                                .any(|e| matches!(e, ArenaEvent::CarHitBall(h) if h.car_idx == toucher))
                        {
                            first_hit = Some((n + t + 1) as i64 - t_idx as i64);
                        }
                        let at = n + t + 1;
                        for (h, &horizon) in HORIZONS.iter().enumerate() {
                            if at == t_idx + horizon {
                                let truth = &packets[at];
                                let b = arena.get_ball_state();
                                vel[h] = (b.phys.vel - truth.ball_vel).length();
                                pos[h] = (b.phys.pos - truth.ball_pos).length();
                            }
                        }
                    }
                    if config.name == "exact" {
                        hit_offsets.entry(lead).or_default().push(first_hit);
                    }
                    samples.push(Sample {
                        lead,
                        config: config.name,
                        free_air,
                        car_grounded,
                        carrying,
                        is_bot,
                        clean,
                        vel,
                        pos,
                    });
                }
            }
        }
    }

    println!("\nHit impulse handling: converter default (apply_hit_extra_impulse = false; the pinned RocketSim applies it).");
    println!("\n== CarHitBall of the toucher in the rollout: step offset of the first hit against the true touch step (0 = right tick) ==");
    for (lead, v) in &hit_offsets {
        let mut counts: BTreeMap<String, usize> = BTreeMap::new();
        for o in v {
            let key = match o {
                None => "no hit".to_string(),
                Some(d) if d.abs() <= 3 => format!("{d:+}"),
                Some(d) if *d < 0 => "earlier than -3".to_string(),
                Some(_) => "later than +3".to_string(),
            };
            *counts.entry(key).or_default() += 1;
        }
        let total = v.len() as f64;
        println!(
            "lead {lead} (n {}): {}",
            v.len(),
            counts
                .iter()
                .map(|(k, c)| format!("{k}: {:.1}%", 100.0 * *c as f64 / total))
                .collect::<Vec<_>>()
                .join(", ")
        );
    }

    let exact: Vec<&Sample> = samples.iter().filter(|s| s.config == "exact").collect();
    println!("\n== Ball VELOCITY error at the horizon after the touch (H ticks after T); columns: % of touches below 25/50/100 UU/s, p50, p90 (UU/s) ==");
    let groups: Vec<(&str, Vec<&Sample>)> = vec![
        ("all touches", exact.clone()),
        ("clean (no other touch in T..T+12)", exact.iter().copied().filter(|s| s.clean).collect()),
        ("clean, ball in free air", exact.iter().copied().filter(|s| s.clean && s.free_air).collect()),
        ("clean, ball near floor/walls", exact.iter().copied().filter(|s| s.clean && !s.free_air).collect()),
        ("clean, car grounded", exact.iter().copied().filter(|s| s.clean && s.car_grounded).collect()),
        ("clean, car airborne", exact.iter().copied().filter(|s| s.clean && !s.car_grounded).collect()),
        ("clean, carry/dribble (<=40 ticks)", exact.iter().copied().filter(|s| s.clean && s.carrying).collect()),
        ("clean, first touch", exact.iter().copied().filter(|s| s.clean && !s.carrying).collect()),
        ("clean, bot", exact.iter().copied().filter(|s| s.clean && s.is_bot).collect()),
        ("clean, human", exact.iter().copied().filter(|s| s.clean && !s.is_bot).collect()),
    ];
    for (label, g) in &groups {
        for &lead in &LEADS {
            summarize(label, g, lead, 0);
        }
    }
    println!("\n== Ball POSITION error at the horizon; columns: % below 1/3/10 UU, p50, p90 (UU) ==");
    for (label, g) in groups.iter().take(2) {
        for &lead in &LEADS {
            summarize(label, g, lead, 1);
        }
    }
    println!("\n== Sensitivity: the toucher's start position/velocity perturbed ({DRAWS} draws per touch), clean touches, ball velocity error ==");
    for config in CONFIGS {
        let g: Vec<&Sample> = samples.iter().filter(|s| s.config == config.name && s.clean).collect();
        for &lead in &LEADS {
            summarize(config.name, &g, lead, 0);
        }
    }
    println!("\n== Sensitivity, clean carry/dribble touches (the close-control case) ==");
    for config in CONFIGS {
        let g: Vec<&Sample> = samples
            .iter()
            .filter(|s| s.config == config.name && s.clean && s.carrying)
            .collect();
        for &lead in &[1usize, 4] {
            summarize(&format!("carry, {}", config.name), &g, lead, 0);
        }
    }
    Ok(())
}
