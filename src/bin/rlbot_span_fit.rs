#![allow(dead_code)]
//! Experiment: does fitting the car's trajectory to a ball contact (and to the car packet after it) improve
//! the car position at the contact and the post-hit ball velocity, for contacts with NO car packet within
//! four ticks before the hit?
//!
//! Truth: the RLBot recording (`states.jsonl`, exact car and ball state and `last_input` at every tick). The
//! replay (host or client) gives the car packets A (before the hit) and B (after it), the ball packets a and b
//! around the hit, and the observed controls. Per true touch (a change of `latest_touch`, first seen in
//! packet T; the hit step is T-1 -> T) whose toucher has no car packet in T-4..T-1:
//!  * baselines: the converter's own per-tick trajectory (`C`, `--inferred` only), the unfitted scratch
//!    simulation from packet A with the observed controls (`S0`), cubic Hermite and linear interpolation
//!    of packets A and B (`H`, `L`);
//!  * fits (`S1*`): a timing shift of the observed ground control switches chosen to land the simulated car
//!    on packet B and/or the simulated ball on ball packet b (both are future observations: offline fits);
//!  * scoring against the TRUTH: the car's position at T-1 (just before the hit step) and at T, at the
//!    ticks between the packets, and the ball's velocity at T, T+1, T+4, T+12 when RocketSim is restarted at
//!    T-1 from the method's car state (observed controls, shift of the method) with the ball rolled from
//!    ball packet a. `TS` restarts from the TRUE car state at T-1 with the observed controls, `TT` with the
//!    true controls too (the ceiling).
//! Packet ticks: `--oracle` (default) uses the true server tick of every packet (found by exact position
//! match), isolating the control question; `--inferred` uses the converter's inferred lags (the real
//! situation) and also scores the converter's own trajectory.
//!
//! usage: rlbot_span_fit <replay> <states.jsonl> <out.jsonl> [--inferred] [--meshes DIR] [--limit N]

use std::collections::{BTreeMap, HashMap};
use std::env;
use std::error::Error;
use std::fs::{self, File};
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use glam::{Mat3A, Quat, Vec3A};
use replay_to_rocketsim::conversion::{
    self, CarTraceRow, ConvertOptions, PacketLags, controls_from_observation,
    step_tick_with_hit_impulse,
};
use replay_to_rocketsim::contact_alignment::aligned_lags;
use replay_to_rocketsim::observations::{Body, ObservedReplay, extract};
use rocketsim::{
    Arena, ArenaConfig, ArenaEvent, BallState, CarBodyConfig, CarControls, CarState, GameMode, Team,
};
use serde_json::{Value, json};

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

fn key3(p: Vec3A) -> [i32; 3] {
    [(p.x * 100.0).round() as i32, (p.y * 100.0).round() as i32, (p.z * 100.0).round() as i32]
}

struct Truth {
    packets: Vec<Packet>,
    by_frame: HashMap<u64, usize>,
    pos: HashMap<(u8, [i32; 3]), Vec<usize>>,
    names: Vec<String>,
}

impl Truth {
    fn at(&self, tick: i64) -> Option<&Packet> {
        if tick < 0 {
            return None;
        }
        self.by_frame.get(&(tick as u64)).map(|&i| &self.packets[i])
    }
}

fn load_truth(path: &Path) -> Result<Truth, Box<dyn Error>> {
    let mut packets: Vec<Packet> = Vec::new();
    for line in BufReader::new(File::open(path)?).lines() {
        let Some(packet) = parse(&line?) else { continue };
        if packets.last().is_some_and(|p| p.frame == packet.frame) {
            continue;
        }
        packets.push(packet);
    }
    let max_players = packets.iter().map(|p| p.players.len()).max().unwrap_or(0);
    packets.retain(|p| p.players.len() == max_players);
    let names: Vec<String> = packets[0].players.iter().map(|p| p.name.clone()).collect();
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
    let mut by_frame = HashMap::new();
    let mut pos: HashMap<(u8, [i32; 3]), Vec<usize>> = HashMap::new();
    for (i, p) in packets.iter().enumerate() {
        by_frame.insert(p.frame, i);
        for (k, pl) in p.players.iter().enumerate() {
            pos.entry((k as u8, key3(pl.pos))).or_default().push(i);
        }
        pos.entry((255, key3(p.ball_pos))).or_default().push(i);
    }
    Ok(Truth { packets, by_frame, pos, names })
}

#[derive(Clone, Copy, Debug)]
struct Phys {
    pos: Vec3A,
    vel: Vec3A,
    rot: Mat3A,
    ang: Vec3A,
}

fn fresh_phys(body: &Body, f: usize) -> Option<Phys> {
    let pos = body.position.as_ref().filter(|v| v.frame == f)?.value;
    let vel = body.linear_velocity.as_ref().filter(|v| v.frame == f)?.value;
    let rot = body.rotation_xyzw.as_ref().filter(|v| v.frame == f)?.value;
    let ang = body
        .angular_velocity_replay_units
        .as_ref()
        .filter(|v| v.frame == f)
        .map_or([0.0; 3], |v| v.value);
    let q = conversion::quaternion(rot)?;
    Some(Phys {
        pos: Vec3A::from(pos),
        vel: Vec3A::from(vel),
        rot: Mat3A::from_quat(q),
        ang: Vec3A::from(ang) * 0.01,
    })
}

#[derive(Clone)]
struct CarPkt {
    frame: usize,
    tick: i64,
    phys: Phys,
    boost: f32,
}

struct Track {
    actor: i32,
    created: usize,
    player: usize,
    pkts: Vec<CarPkt>,
}

impl Track {
    fn tick_at(&self, frame: usize) -> Option<i64> {
        self.pkts.binary_search_by_key(&frame, |p| p.frame).ok().map(|i| self.pkts[i].tick)
    }
}

#[derive(Clone)]
struct BallPkt {
    frame: usize,
    tick: i64,
    phys: Phys,
}

struct Packets {
    tracks: Vec<Track>,
    ball: Vec<BallPkt>,
    off: Vec<i64>,
    timeline: Vec<i64>,
}

fn is_active(obs: &ObservedReplay, f: usize) -> bool {
    obs.frames[f].game_state.as_ref().is_some_and(|g| g.value == "Active")
}

fn build_packets(
    obs: &ObservedReplay,
    truth: &Truth,
    name_of_key: &HashMap<String, String>,
    lags: Option<&PacketLags>,
) -> Packets {
    let frames = &obs.frames;
    let first_time = f64::from(frames.first().map_or(0.0, |f| f.time));
    let timeline: Vec<i64> =
        frames.iter().map(|f| ((f64::from(f.time) - first_time) * 120.0).round() as i64).collect();
    struct Raw {
        g: usize,
        actor: i32,
        created: usize,
        player: usize,
        phys: Phys,
        boost: f32,
        true_tick: Option<i64>,
        lag: Option<f32>,
    }
    let mut raws: Vec<Raw> = Vec::new();
    for (g, frame) in frames.iter().enumerate() {
        if !is_active(obs, g) {
            continue;
        }
        for car in &frame.cars {
            let Some(name) = car.player_key.as_ref().and_then(|k| name_of_key.get(k)) else { continue };
            let Some(player) = truth.names.iter().position(|n| n == name) else { continue };
            let Some(phys) = fresh_phys(&car.body, g) else { continue };
            let true_tick = truth
                .pos
                .get(&(player as u8, key3(phys.pos)))
                .filter(|v| v.len() == 1)
                .map(|v| truth.packets[v[0]].frame as i64);
            let lag = lags.and_then(|l| l.car_actor.get(&(car.actor_id, car.actor_created_frame, g)).copied());
            raws.push(Raw {
                g,
                actor: car.actor_id,
                created: car.actor_created_frame,
                player,
                phys,
                boost: car.boost.as_ref().map_or(100.0, |b| b.value),
                true_tick,
                lag,
            });
        }
    }
    // The running offset between the converter's timeline and the server ticks (as rlbot_reconstruction).
    let mut matched: Vec<(usize, i64)> = Vec::new();
    for r in &raws {
        if let Some(t) = r.true_tick {
            match (lags.is_some(), r.lag) {
                (true, Some(lag)) => matched.push((r.g, timeline[r.g] - lag.round() as i64 - t)),
                (true, None) => {}
                (false, _) => matched.push((r.g, timeline[r.g] - t)),
            }
        }
    }
    let mut off = vec![0i64; frames.len()];
    if !matched.is_empty() {
        for f in 0..frames.len() {
            let i = matched.partition_point(|m| m.0 < f).min(matched.len() - 1);
            if lags.is_some() {
                let (lo, hi) = (i.saturating_sub(100), (i + 101).min(matched.len()));
                let mut counts: BTreeMap<i64, usize> = BTreeMap::new();
                for m in &matched[lo..hi] {
                    *counts.entry(m.1).or_default() += 1;
                }
                off[f] = counts.into_iter().max_by_key(|(_, c)| *c).map(|(o, _)| o).unwrap_or(0);
            } else {
                let (lo, hi) = (i.saturating_sub(150), (i + 151).min(matched.len()));
                off[f] = matched[lo..hi].iter().map(|m| m.1).min().unwrap_or(0);
            }
        }
    }
    let mut tracks: HashMap<(i32, usize), Track> = HashMap::new();
    for r in raws {
        let tick = if lags.is_some() {
            match r.lag {
                Some(lag) => timeline[r.g] - lag.round() as i64 - off[r.g],
                None => continue,
            }
        } else {
            match r.true_tick {
                Some(t) => t,
                None => continue,
            }
        };
        tracks
            .entry((r.actor, r.created))
            .or_insert_with(|| Track { actor: r.actor, created: r.created, player: r.player, pkts: Vec::new() })
            .pkts
            .push(CarPkt { frame: r.g, tick, phys: r.phys, boost: r.boost });
    }
    let mut tracks: Vec<Track> = tracks.into_values().collect();
    for t in &mut tracks {
        t.pkts.sort_by_key(|p| p.frame);
    }
    let mut ball = Vec::new();
    for (g, frame) in frames.iter().enumerate() {
        if !is_active(obs, g) {
            continue;
        }
        let Some(phys) = frame.ball.as_ref().and_then(|b| fresh_phys(b, g)) else { continue };
        let tick = if let Some(l) = lags {
            let Some(Some(lag)) = l.ball.get(g).copied() else { continue };
            timeline[g] - lag.round() as i64 - off[g]
        } else {
            match truth.pos.get(&(255, key3(phys.pos))).filter(|v| v.len() == 1) {
                Some(v) => truth.packets[v[0]].frame as i64,
                None => continue,
            }
        };
        ball.push(BallPkt { frame: g, tick, phys });
    }
    Packets { tracks, ball, off, timeline }
}

fn control_entries(
    obs: &ObservedReplay,
    options: &ConvertOptions,
    pk: &Packets,
    track: &Track,
    g_lo: usize,
    g_hi: usize,
) -> Vec<(i64, CarControls)> {
    let frames = &obs.frames;
    let mut out = Vec::new();
    for g in g_lo..=g_hi.min(frames.len() - 1) {
        if !is_active(obs, g) {
            continue;
        }
        let Some(c) = frames[g]
            .cars
            .iter()
            .find(|c| c.actor_id == track.actor && c.actor_created_frame == track.created)
        else {
            continue;
        };
        let mut ctl = controls_from_observation(c, options);
        ctl.jump = false;
        let mut tick = None;
        if let Some(s_cur) = track.tick_at(g) {
            if let Some(s_prev) = (g.saturating_sub(8)..g).rev().find_map(|h| track.tick_at(h)) {
                if s_prev < s_cur && s_cur - s_prev <= 40 {
                    tick = Some((s_prev + s_cur + 1).div_euclid(2));
                }
            }
        }
        let tick = tick.unwrap_or_else(|| {
            let spacing = if g == 0 { 4 } else { pk.timeline[g] - pk.timeline[g - 1] };
            pk.timeline[g] - pk.off[g] - 2 - spacing / 2
        });
        out.push((tick, ctl));
    }
    out.sort_by_key(|e| e.0);
    out
}

fn controls_at(entries: &[(i64, CarControls)], own: CarControls, shift: i64, tau: i64) -> CarControls {
    let i = entries.partition_point(|e| e.0 + shift <= tau);
    if i == 0 { own } else { entries[i - 1].1 }
}

fn make_car_state(p: &Phys, boost: f32) -> CarState {
    let mut s = CarState::default();
    s.phys.pos = p.pos;
    s.phys.vel = p.vel;
    s.phys.rot_mat = p.rot;
    s.phys.ang_vel = p.ang;
    s.boost = boost;
    let grounded = p.pos.z < 30.0;
    s.is_on_ground = grounded;
    s.wheels_with_contact = [grounded.then(rocketsim::RaycastHitInfo::default); 4];
    s
}

/// The ceiling tool's full-state construction from a true player record.
fn truth_car_state(pl: &Player) -> CarState {
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
    s.jump_ticks = if s.is_jumping { 1 } else { 0 };
    if pl.dodge_timeout >= 0.0 {
        s.air_time_since_jump = 1.25 - pl.dodge_timeout;
        s.air_time = s.air_time_since_jump;
    }
    s.is_boosting = pl.controls.boost && pl.boost > 0.0;
    s.is_supersonic = pl.supersonic;
    s
}

fn ball_state(p: &Phys) -> BallState {
    let mut b = BallState::default();
    b.phys.pos = p.pos;
    b.phys.vel = p.vel;
    b.phys.rot_mat = p.rot;
    b.phys.ang_vel = p.ang;
    b
}

struct RunOut {
    /// Car state at tick `t_a + i`.
    cars: Vec<CarState>,
    /// Ball (position, velocity) at tick `t_a + i`, when the ball is placed by then.
    ball: Vec<Option<(Vec3A, Vec3A)>>,
    /// Ticks of CarHitBall events.
    hits: Vec<i64>,
}

struct Sim {
    arena: Arena,
    ball_arena: Arena,
    refresh: bool,
    hit_extra: bool,
}

impl Sim {
    fn new(config: CarBodyConfig) -> Self {
        let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
        arena.add_car(Team::Blue, config);
        Sim {
            arena,
            ball_arena: Arena::new_with_config(ArenaConfig::new(GameMode::Soccar)),
            refresh: env::var_os("REFRESH").is_some(),
            hit_extra: false,
        }
    }

    /// Car from `start` at tick `t_a`; ball from its packet. The earlier object runs alone until the later
    /// one starts. Runs through tick `t_end`.
    fn run(
        &mut self,
        start: CarState,
        t_a: i64,
        ball: &BallPkt,
        ctl: &dyn Fn(i64) -> CarControls,
        t_end: i64,
    ) -> RunOut {
        let n = (t_end - t_a + 1).max(1) as usize;
        let mut out = RunOut { cars: Vec::with_capacity(n), ball: vec![None; n], hits: Vec::new() };
        let mut parked = BallState::default();
        parked.phys.pos = if start.phys.pos.distance(Vec3A::new(0.0, 0.0, 1800.0)) > 700.0 {
            Vec3A::new(0.0, 0.0, 1800.0)
        } else {
            Vec3A::new(3000.0, 4000.0, 300.0)
        };
        let ball_bs = ball_state(&ball.phys);
        out.cars.push(start);
        if ball.tick >= t_a {
            self.arena.set_ball_state(parked);
            self.arena.set_car_state(0, start);
            if self.refresh {
                self.arena.refresh_car_sticky_gate(0);
            }
            for tau in t_a + 1..=t_end {
                if tau - 1 == ball.tick {
                    self.arena.set_ball_state(ball_bs);
                    out.ball[(tau - 1 - t_a) as usize] = Some((ball.phys.pos, ball.phys.vel));
                }
                self.arena.set_car_controls(0, ctl(tau));
                let events = step_tick_with_hit_impulse(&mut self.arena, self.hit_extra);
                if events.iter().any(|e| matches!(e, ArenaEvent::CarHitBall(_))) {
                    out.hits.push(tau);
                }
                out.cars.push(*self.arena.get_car_state(0));
                if tau > ball.tick {
                    let b = self.arena.get_ball_state();
                    out.ball[(tau - t_a) as usize] = Some((b.phys.pos, b.phys.vel));
                }
            }
        } else {
            self.ball_arena.set_ball_state(ball_bs);
            for _ in ball.tick..t_a {
                self.ball_arena.step_tick();
            }
            self.arena.set_ball_state(*self.ball_arena.get_ball_state());
            self.arena.set_car_state(0, start);
            if self.refresh {
                self.arena.refresh_car_sticky_gate(0);
            }
            let b = self.arena.get_ball_state();
            out.ball[0] = Some((b.phys.pos, b.phys.vel));
            for tau in t_a + 1..=t_end {
                self.arena.set_car_controls(0, ctl(tau));
                let events = step_tick_with_hit_impulse(&mut self.arena, self.hit_extra);
                if events.iter().any(|e| matches!(e, ArenaEvent::CarHitBall(_))) {
                    out.hits.push(tau);
                }
                out.cars.push(*self.arena.get_car_state(0));
                let b = self.arena.get_ball_state();
                out.ball[(tau - t_a) as usize] = Some((b.phys.pos, b.phys.vel));
            }
        }
        out
    }
}

fn hermite(a: &Phys, b: &Phys, t_a: i64, t_b: i64, tau: i64) -> (Vec3A, Vec3A) {
    let dt = (t_b - t_a) as f32 / 120.0;
    let s = (tau - t_a) as f32 / (t_b - t_a) as f32;
    let (s2, s3) = (s * s, s * s * s);
    let h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    let h10 = s3 - 2.0 * s2 + s;
    let h01 = -2.0 * s3 + 3.0 * s2;
    let h11 = s3 - s2;
    let pos = a.pos * h00 + a.vel * (h10 * dt) + b.pos * h01 + b.vel * (h11 * dt);
    let d00 = 6.0 * s2 - 6.0 * s;
    let d10 = 3.0 * s2 - 4.0 * s + 1.0;
    let d01 = -6.0 * s2 + 6.0 * s;
    let d11 = 3.0 * s2 - 2.0 * s;
    let vel = (a.pos * d00 + a.vel * (d10 * dt) + b.pos * d01 + b.vel * (d11 * dt)) / dt;
    (pos, vel)
}

fn linear(a: &Phys, b: &Phys, t_a: i64, t_b: i64, tau: i64) -> (Vec3A, Vec3A) {
    let s = (tau - t_a) as f32 / (t_b - t_a) as f32;
    let dt = (t_b - t_a) as f32 / 120.0;
    (a.pos.lerp(b.pos, s), (b.pos - a.pos) / dt)
}

fn slerp_rot(a: Mat3A, b: Mat3A, s: f32) -> Mat3A {
    Mat3A::from_quat(Quat::from_mat3a(&a).slerp(Quat::from_mat3a(&b), s))
}

fn quantile(values: &mut Vec<f32>, q: f64) -> f32 {
    if values.is_empty() {
        return f32::NAN;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn r2(x: f32) -> f64 {
    ((x as f64) * 100.0).round() / 100.0
}

/// A method's car trajectory (position per tick from t_a to t_b), its state at T-1 and the controls it
/// continues with.
struct Method {
    pos: Vec<Option<Vec3A>>,
    state_t1: Option<CarState>,
    shift: i64,
    /// Use the true inputs after T-1 (the ceiling).
    true_controls: bool,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 3 {
        return Err("usage: rlbot_span_fit <replay> <states.jsonl> <out.jsonl> [--inferred] [--meshes DIR] [--limit N]".into());
    }
    let (replay_path, states_path, out_path) = (PathBuf::from(&args[0]), PathBuf::from(&args[1]), PathBuf::from(&args[2]));
    for p in [&replay_path, &states_path] {
        if p.to_string_lossy().contains("test") {
            return Err("refusing to inspect a path containing 'test'".into());
        }
    }
    let inferred = args.iter().any(|a| a == "--inferred");
    let meshes = args
        .iter()
        .position(|a| a == "--meshes")
        .and_then(|i| args.get(i + 1).cloned())
        .unwrap_or_else(|| "collision_meshes".to_string());
    let limit: usize = args
        .iter()
        .position(|a| a == "--limit")
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok())
        .unwrap_or(usize::MAX);
    let min_lead: i64 = args
        .iter()
        .position(|a| a == "--min-lead")
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok())
        .unwrap_or(5);
    let label = replay_path.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
    rocketsim::init(Path::new(&meshes), true)?;

    eprintln!("loading truth ...");
    let truth = load_truth(&states_path)?;
    eprintln!("{} truth packets, players {:?}", truth.packets.len(), truth.names);
    let replay = boxcars::ParserBuilder::new(&fs::read(&replay_path)?).must_parse_network_data().parse()?;
    let mut observed = extract(&replay).ok_or("no network frames")?;
    // `--thin K`: only every K-th frame keeps its car body packets (the others repeat the last kept body with
    // its old stamps), to imitate a sparser cadence (as rlbot_reconstruction).
    if let Some(at) = args.iter().position(|a| a == "--thin") {
        let k: usize = args.get(at + 1).and_then(|v| v.parse().ok()).ok_or("--thin K")?;
        let mut last: HashMap<(i32, usize), Body> = HashMap::new();
        for (f, frame) in observed.frames.iter_mut().enumerate() {
            for car in &mut frame.cars {
                let key = (car.actor_id, car.actor_created_frame);
                match last.get(&key) {
                    Some(body) if f % k != 0 => car.body = body.clone(),
                    _ => {
                        last.insert(key, car.body.clone());
                    }
                }
            }
        }
    }
    let name_of_key: HashMap<String, String> = observed
        .frames
        .iter()
        .flat_map(|f| f.players.iter())
        .filter_map(|p| p.name.clone().map(|n| (p.key.clone(), n)))
        .collect();
    let mut options = ConvertOptions::default();
    options.collision_meshes = PathBuf::from(&meshes);

    // Lags and the converter's own trajectory (inferred mode).
    let mut lags: Option<PacketLags> = None;
    let mut conv_trace: HashMap<(usize, i64), CarTraceRow> = HashMap::new();
    let mut slot_player: HashMap<usize, usize> = HashMap::new();
    if inferred {
        eprintln!("inferring lags (with the contact alignment) ...");
        let (l, summary) = aligned_lags(&observed, &options)?;
        eprintln!("alignment: {summary:?}");
        lags = Some(l.clone());
        let mut o2 = options.clone();
        o2.align_contacts = false;
        o2.external_packet_lags = Some(Arc::new(l));
        conversion::CAR_TRACE_ENABLED.store(true, std::sync::atomic::Ordering::Relaxed);
        eprintln!("converting with the car trace ...");
        let summary = conversion::convert_observations_with(&observed, &o2, |_, _, _| Ok(()))?;
        conversion::CAR_TRACE_ENABLED.store(false, std::sync::atomic::Ordering::Relaxed);
        for s in &summary.car_slots {
            if let Some(name) = name_of_key.get(&s.player_key) {
                if let Some(p) = truth.names.iter().position(|n| n == name) {
                    slot_player.insert(s.slot, p);
                }
            }
        }
        let rows = std::mem::take(&mut *conversion::CAR_TRACE.lock().unwrap());
        eprintln!("{} trace rows", rows.len());
        // The offsets are filled in below once the packets are built; keep the rows by frame for now.
        let mut pending: Vec<CarTraceRow> = Vec::new();
        let mut by_frame: Vec<(usize, i64, Vec<CarTraceRow>)> = Vec::new();
        for r in rows {
            if r.slot == u32::MAX {
                by_frame.push((r.vel[0] as usize, r.vel[1] as i64, std::mem::take(&mut pending)));
            } else {
                pending.push(r);
            }
        }
        // Stash for later: (frame, timeline offset, rows).
        TRACE_FRAMES.with(|t| *t.borrow_mut() = by_frame);
    }
    let pk = build_packets(&observed, &truth, &name_of_key, lags.as_ref());
    if inferred {
        TRACE_FRAMES.with(|t| {
            for (frame, tl_offset, rows) in t.borrow().iter() {
                for r in rows {
                    let Some(&player) = slot_player.get(&(r.slot as usize)) else { continue };
                    let timeline_tick = r.arena_tick as i64 + tl_offset;
                    let server = timeline_tick - pk.off[*frame];
                    conv_trace.insert((player, server), *r);
                }
            }
        });
    }
    eprintln!(
        "{} car tracks, {} ball packets, {} car packets",
        pk.tracks.len(),
        pk.ball.len(),
        pk.tracks.iter().map(|t| t.pkts.len()).sum::<usize>()
    );

    // True touches: (packet index T, player).
    let n_players = truth.names.len();
    let mut touches: Vec<(usize, usize)> = Vec::new();
    let mut last_seconds: Vec<Option<f64>> = vec![None; n_players];
    for (i, p) in truth.packets.iter().enumerate() {
        for k in 0..n_players {
            let s = p.players[k].touch_seconds;
            if s.is_some() && last_seconds[k] != s {
                touches.push((i, k));
            }
            last_seconds[k] = s;
        }
    }
    touches.retain(|&(i, _)| i > 0);
    eprintln!("{} true touches", touches.len());

    let mut out = File::create(&out_path)?;
    let mut counts: BTreeMap<&str, usize> = BTreeMap::new();
    let mut done = 0usize;
    let shifts: Vec<i64> = (-8..=40).collect();
    'touches: for &(ti, k) in &touches {
        let t_hit = truth.packets[ti].frame as i64; // T
        *counts.entry("touches").or_default() += 1;
        // Car packets A (before) and B (after) of this player's own tracks.
        let mut a_pkt: Option<(&Track, usize)> = None;
        let mut b_pkt: Option<(&Track, usize)> = None;
        for t in pk.tracks.iter().filter(|t| t.player == k) {
            for (i, p) in t.pkts.iter().enumerate() {
                if p.tick <= t_hit - 1 && a_pkt.is_none_or(|(tt, ii)| tt.pkts[ii].tick < p.tick) {
                    a_pkt = Some((t, i));
                }
            }
        }
        let Some((track, ia)) = a_pkt else {
            *counts.entry("no car packet A").or_default() += 1;
            continue;
        };
        for (i, p) in track.pkts.iter().enumerate().skip(ia + 1) {
            if p.tick >= t_hit {
                b_pkt = Some((track, i));
                break;
            }
        }
        let Some((_, ib)) = b_pkt else {
            *counts.entry("no car packet B").or_default() += 1;
            continue;
        };
        let (a, b) = (&track.pkts[ia], &track.pkts[ib]);
        let lead = t_hit - a.tick;
        if lead < min_lead {
            *counts.entry("lead < min").or_default() += 1;
            continue;
        }
        if b.tick - a.tick > 60 {
            *counts.entry("span > 60 ticks").or_default() += 1;
            continue;
        }
        // Ball packets around the hit.
        let ball_a = pk.ball.iter().filter(|p| p.tick <= t_hit - 1).max_by_key(|p| p.tick);
        let ball_b = pk.ball.iter().filter(|p| p.tick >= t_hit).min_by_key(|p| p.tick);
        let (Some(ball_a), Some(ball_b)) = (ball_a, ball_b) else {
            *counts.entry("no ball packets").or_default() += 1;
            continue;
        };
        if ball_b.tick - ball_a.tick > 40 || t_hit - ball_a.tick > 40 {
            *counts.entry("ball packets too far").or_default() += 1;
            continue;
        }
        let t_first = a.tick.min(ball_a.tick);
        let t_end = b.tick.max(ball_b.tick).max(t_hit + 12);
        // Contiguous active truth over the whole window.
        if !(t_first - 1..=t_end).all(|tau| truth.at(tau).is_some_and(|p| p.phase == 3)) {
            *counts.entry("truth gap or inactive").or_default() += 1;
            continue;
        }
        // No other true touch from the first packet to T + 12.
        let unclean = touches.iter().any(|&(tj, _)| {
            tj != ti && {
                let f = truth.packets[tj].frame as i64;
                f >= t_first - 1 && f <= t_hit + 12
            }
        });
        if unclean {
            *counts.entry("other touch in window").or_default() += 1;
            continue;
        }
        let tp1 = truth.at(t_hit - 1).unwrap();
        let tp0 = truth.at(t_hit).unwrap();
        let pl1 = &tp1.players[k];
        // Counter changes (jump, double jump, dodge) in the span: refuse to treat as plain ground driving.
        let frames = &observed.frames;
        let counters = |g: usize| -> [Option<u8>; 3] {
            frames[g]
                .cars
                .iter()
                .find(|c| c.actor_id == track.actor && c.actor_created_frame == track.created)
                .map_or([None; 3], |c| {
                    [
                        c.inputs.jump_active_raw.as_ref().map(|v| v.value),
                        c.inputs.double_jump_active_raw.as_ref().map(|v| v.value),
                        c.inputs.dodge_active_raw.as_ref().map(|v| v.value),
                    ]
                })
        };
        let jump_in_span = (a.frame..=b.frame).any(|g| counters(g) != counters(a.frame));
        // A dodge (flip) is in progress at packet A according to the observed counter.
        let dodge_odd_at_a = counters(a.frame)[2].is_some_and(|v| v % 2 == 1);
        let air_at_a = pk_air_state(&truth, a.tick, k);
        let entries = control_entries(&observed, &options, &pk, track, a.frame.saturating_sub(3), b.frame + 1);
        let own = frames[a.frame]
            .cars
            .iter()
            .find(|c| c.actor_id == track.actor && c.actor_created_frame == track.created)
            .map(|c| {
                let mut ctl = controls_from_observation(c, &options);
                ctl.jump = false;
                ctl
            })
            .unwrap_or_default();
        let hitbox_cfg = preset(pl1.hitbox);
        let mut sim = Sim::new(hitbox_cfg);
        let start = make_car_state(&a.phys, a.boost);
        // Candidate runs (timing shifts of the observed control switches), in families:
        //  G: observed ground controls, no air controls; A: the same plus the boundary-value air controls
        //  (the converter's schedule, from packets A and B); TA: observed controls plus the TRUE air controls
        //  (oracle bound on what rotation modelling can bring).
        let idx = |tau: i64| (tau - a.tick) as usize;
        // The boundary-value air schedule (as `plan_air_bvp`): segments of about 4 ticks.
        let total = (b.tick - a.tick) as u32;
        let bvp: Option<Vec<(i64, conversion::AirControls)>> = if a.phys.pos.z > 30.0 && b.phys.pos.z > 30.0 && total >= 2 {
            let parts = total.div_ceil(4).max(1);
            let ticks: Vec<u32> = (0..parts).map(|i| total * (i + 1) / parts - total * i / parts).collect();
            let constant = conversion::solve_span_air_controls(
                a.phys.rot,
                a.phys.ang,
                b.phys.ang,
                total,
                options.air_lookahead_refine_iterations,
            );
            let prior = vec![constant; ticks.len()];
            let (solved, rot_err, omega_err) =
                conversion::solve_air_bvp(a.phys.rot, a.phys.ang, b.phys.rot, b.phys.ang, &ticks, &prior);
            if rot_err <= 3.0f32.to_radians() && omega_err <= 0.5 {
                let mut tick = a.tick + 1;
                let mut entries = Vec::new();
                for (c, n) in solved.iter().zip(&ticks) {
                    entries.push((tick, *c));
                    tick += i64::from(*n);
                }
                Some(entries)
            } else {
                None
            }
        } else {
            None
        };
        // AR: the boundary-value air controls with RocketSim (and the ball, so the hit) as the forward model,
        // from packet A to packet B: per-segment pitch/yaw/roll that bring the simulated car to packet B's
        // rotation and angular velocity. Uses packet B (a future observation).
        let air_like = a.phys.pos.z > 30.0 && b.phys.pos.z > 30.0 && total >= 2;
        let mut ar_info = Value::Null;
        let ar_sched: Option<Vec<(i64, conversion::AirControls)>> = if air_like && !dodge_odd_at_a && !args.iter().any(|x| x == "--no-ar") {
            let parts = total.div_ceil(4).max(1);
            let ticks: Vec<u32> = (0..parts).map(|i| total * (i + 1) / parts - total * i / parts).collect();
            let constant = conversion::solve_span_air_controls(
                a.phys.rot,
                a.phys.ang,
                b.phys.ang,
                total,
                options.air_lookahead_refine_iterations,
            );
            let prior = vec![constant; ticks.len()];
            let mut forward = |segments: &[(conversion::AirControls, u32)]| -> (Mat3A, Vec3A) {
                let mut sched: Vec<(i64, conversion::AirControls)> = Vec::new();
                let mut tick = a.tick + 1;
                for (c, n) in segments {
                    sched.push((tick, *c));
                    tick += i64::from(*n);
                }
                let ctl = |tau: i64| {
                    let mut c = controls_at(&entries, own, 0, tau);
                    if let Some(x) = sched.iter().rev().find(|x| x.0 <= tau) {
                        c.pitch = x.1.pitch;
                        c.yaw = x.1.yaw;
                        c.roll = x.1.roll;
                    }
                    c
                };
                let run = sim.run(start, a.tick, ball_a, &ctl, b.tick);
                let end = run.cars.last().unwrap();
                let mut omega = end.phys.ang_vel;
                let speed = omega.length();
                if speed > 5.5 {
                    omega *= 5.5 / speed;
                }
                (end.phys.rot_mat, omega)
            };
            let (solved, rot_err, omega_err) =
                conversion::solve_bvp_with(&mut forward, b.phys.rot, b.phys.ang, &ticks, &prior);
            ar_info = json!({"rot_err_deg": r2(rot_err.to_degrees()), "omega_err": r2(omega_err), "segments": ticks.len()});
            let mut tick = a.tick + 1;
            let mut entries_out = Vec::new();
            for (c, n) in solved.iter().zip(&ticks) {
                entries_out.push((tick, *c));
                tick += i64::from(*n);
            }
            Some(entries_out)
        } else {
            None
        };
        let ar_at = |tau: i64| -> Option<conversion::AirControls> {
            ar_sched.as_ref().and_then(|e| e.iter().rev().find(|x| x.0 <= tau).map(|x| x.1))
        };
        let bvp_at = |tau: i64| -> Option<conversion::AirControls> {
            bvp.as_ref().and_then(|e| e.iter().rev().find(|x| x.0 <= tau).map(|x| x.1))
        };
        let family_ctl = |fam: &str, shift: i64, tau: i64| -> CarControls {
            let mut c = controls_at(&entries, own, shift, tau);
            match fam {
                "A" => {
                    if let Some(air) = bvp_at(tau) {
                        c.pitch = air.pitch;
                        c.yaw = air.yaw;
                        c.roll = air.roll;
                    }
                }
                "AR" => {
                    if let Some(air) = ar_at(tau) {
                        c.pitch = air.pitch;
                        c.yaw = air.yaw;
                        c.roll = air.roll;
                    }
                }
                "TA" => {
                    if let Some(tp) = truth.at(tau) {
                        let t = tp.players[k].controls;
                        c.pitch = t.pitch;
                        c.yaw = t.yaw;
                        c.roll = t.roll;
                    }
                }
                "T" => {
                    if let Some(tp) = truth.at(tau) {
                        c = tp.players[k].controls;
                        c.jump = false;
                    }
                }
                _ => {}
            }
            c
        };
        let score_run = |run: &RunOut| -> Value {
            let end = &run.cars[idx(b.tick)];
            let end_pos = (end.phys.pos - b.phys.pos).length();
            let end_vel = (end.phys.vel - b.phys.vel).length();
            let ball_res = run.ball[idx(ball_b.tick)]
                .map(|(_, v)| (v - ball_b.phys.vel).length())
                .unwrap_or(f32::NAN);
            let mut interior = Vec::new();
            for tau in a.tick + 1..b.tick {
                let t = truth.at(tau).unwrap();
                interior.push(r2((run.cars[idx(tau)].phys.pos - t.players[k].pos).length()));
            }
            let s1 = &run.cars[idx(t_hit - 1)];
            let pos_t1 = (s1.phys.pos - pl1.pos).length();
            let vel_t1 = (s1.phys.vel - pl1.vel).length();
            let rot_t1 = rotation_error_deg(s1.phys.rot_mat, pl1.rot);
            let ang_t1 = (s1.phys.ang_vel - pl1.ang).length();
            let pos_t = (run.cars[idx(t_hit)].phys.pos - tp0.players[k].pos).length();
            let mut ball_err = Vec::new();
            for h in [0i64, 1, 4, 12] {
                let tt = truth.at(t_hit + h).unwrap();
                ball_err.push(
                    run.ball[idx(t_hit + h)].map(|(_, v)| r2((v - tt.ball_vel).length())).unwrap_or(f64::NAN),
                );
            }
            let hit_rel = run.hits.iter().map(|h| h - t_hit).min();
            json!({
                "end_pos": r2(end_pos), "end_vel": r2(end_vel), "ball_res": r2(ball_res),
                "pos_t1": r2(pos_t1), "vel_t1": r2(vel_t1), "rot_t1": r2(rot_t1), "ang_t1": r2(ang_t1),
                "pos_t": r2(pos_t), "interior": interior, "ball": ball_err, "hit_rel": hit_rel,
            })
        };
        let mut fams: serde_json::Map<String, Value> = serde_json::Map::new();
        let mut runs: Vec<(i64, RunOut)> = Vec::new(); // family G, for the restarts below
        let mut fam_list = vec!["G", "A", "AR", "TA"];
        if bvp.is_none() {
            fam_list.retain(|f| *f != "A");
        }
        if ar_sched.is_none() {
            fam_list.retain(|f| *f != "AR");
        }
        for fam in fam_list {
            let mut cands: Vec<Value> = Vec::new();
            for &shift in &shifts {
                if fam == "TA" && !(-8..=8).contains(&shift) {
                    continue;
                }
                let ctl = |tau: i64| family_ctl(fam, shift, tau);
                let run = sim.run(start, a.tick, ball_a, &ctl, t_end);
                let mut v = score_run(&run);
                v["shift"] = json!(shift);
                cands.push(v);
                if fam == "G" {
                    runs.push((shift, run));
                }
            }
            fams.insert(fam.to_string(), Value::Array(cands));
        }
        // Natural run from packet A with all true controls (what a perfect control fit could reach).
        {
            let ctl = |tau: i64| family_ctl("T", 0, tau);
            let run = sim.run(start, a.tick, ball_a, &ctl, t_end);
            let mut v = score_run(&run);
            v["shift"] = json!(0);
            fams.insert("T".to_string(), Value::Array(vec![v]));
        }
        // Restart helper: ball outcome from a car state at T-1.
        let outcome = |sim: &mut Sim, state: CarState, ctl: &dyn Fn(i64) -> CarControls| -> RunOut {
            sim.run(state, t_hit - 1, ball_a, ctl, t_hit + 12)
        };
        let s0 = &runs.iter().find(|(s, _)| *s == 0).unwrap().1;
        let s0_state_t1 = s0.cars[idx(t_hit - 1)];
        let mut methods: BTreeMap<String, Method> = BTreeMap::new();
        // Hermite and linear.
        for (name, f) in [("H", 0), ("L", 1)] {
            let mut pos = Vec::new();
            for tau in a.tick..=b.tick {
                let (p, _) = if f == 0 {
                    hermite(&a.phys, &b.phys, a.tick, b.tick, tau)
                } else {
                    linear(&a.phys, &b.phys, a.tick, b.tick, tau)
                };
                pos.push(Some(p));
            }
            let (p, v) = if f == 0 {
                hermite(&a.phys, &b.phys, a.tick, b.tick, t_hit - 1)
            } else {
                linear(&a.phys, &b.phys, a.tick, b.tick, t_hit - 1)
            };
            let s = (t_hit - 1 - a.tick) as f32 / (b.tick - a.tick) as f32;
            let mut st = s0_state_t1;
            st.phys.pos = p;
            st.phys.vel = v;
            st.phys.rot_mat = slerp_rot(a.phys.rot, b.phys.rot, s);
            st.phys.ang_vel = a.phys.ang.lerp(b.phys.ang, s);
            methods.insert(name.to_string(), Method { pos, state_t1: Some(st), shift: 0, true_controls: false });
        }
        // The converter's own trajectory.
        if inferred {
            let mut pos = Vec::new();
            for tau in a.tick..=b.tick {
                pos.push(conv_trace.get(&(k, tau)).map(|r| Vec3A::from(r.pos)));
            }
            let st = conv_trace.get(&(k, t_hit - 1)).map(|r| {
                let mut st = s0_state_t1;
                st.phys.pos = Vec3A::from(r.pos);
                st.phys.vel = Vec3A::from(r.vel);
                st.phys.rot_mat = Mat3A::from_cols(
                    Vec3A::new(r.rot[0], r.rot[1], r.rot[2]),
                    Vec3A::new(r.rot[3], r.rot[4], r.rot[5]),
                    Vec3A::new(r.rot[6], r.rot[7], r.rot[8]),
                );
                st.phys.ang_vel = Vec3A::from(r.ang);
                st
            });
            methods.insert("C".to_string(), Method { pos, state_t1: st, shift: 0, true_controls: false });
        }
        // Truth state (TS: observed controls; TT: true controls).
        let ts_state = truth_car_state(pl1);
        methods.insert("TS".to_string(), Method { pos: vec![], state_t1: Some(ts_state), shift: 0, true_controls: false });
        methods.insert("TT".to_string(), Method { pos: vec![], state_t1: Some(ts_state), shift: 0, true_controls: true });
        let mut method_json = serde_json::Map::new();
        for (name, m) in &methods {
            let mut v = serde_json::Map::new();
            if !m.pos.is_empty() {
                let at = |tau: i64| m.pos[idx(tau)];
                let truth_pos = |tau: i64| truth.at(tau).unwrap().players[k].pos;
                v.insert("pos_t1".into(), json!(at(t_hit - 1).map(|p| r2((p - truth_pos(t_hit - 1)).length()))));
                v.insert("pos_t".into(), json!(at(t_hit).map(|p| r2((p - truth_pos(t_hit)).length()))));
                v.insert("end_pos".into(), json!(at(b.tick).map(|p| r2((p - b.phys.pos).length()))));
                let interior: Vec<Value> = (a.tick + 1..b.tick)
                    .map(|tau| json!(at(tau).map(|p| r2((p - truth_pos(tau)).length()))))
                    .collect();
                v.insert("interior".into(), Value::Array(interior));
            }
            if let Some(st) = m.state_t1 {
                let run = if m.true_controls {
                    let ctl = |tau: i64| {
                        truth.at(tau).map_or_else(CarControls::default, |p| p.players[k].controls)
                    };
                    outcome(&mut sim, st, &ctl)
                } else {
                    let shift = m.shift;
                    let ctl = |tau: i64| controls_at(&entries, own, shift, tau);
                    outcome(&mut sim, st, &ctl)
                };
                let mut ball_err = Vec::new();
                for h in [0i64, 1, 4, 12] {
                    let tt = truth.at(t_hit + h).unwrap();
                    ball_err.push(
                        run.ball[(h + 1) as usize]
                            .map(|(_, v)| r2((v - tt.ball_vel).length()))
                            .unwrap_or(f64::NAN),
                    );
                }
                v.insert("ball".into(), json!(ball_err));
                v.insert("hit_rel".into(), json!(run.hits.iter().map(|h| h - t_hit).min()));
            }
            method_json.insert(name.clone(), Value::Object(v));
        }
        let rec = json!({
            "game": label, "T": t_hit, "player": truth.names[k], "bot": pl1.is_bot,
            "grounded": pl1.air_state == 0, "air_state": pl1.air_state,
            "lead": lead, "t_a": a.tick, "t_b": b.tick, "span": b.tick - a.tick,
            "ball_a": ball_a.tick, "ball_b": ball_b.tick,
            "jump_in_span": jump_in_span, "dodge_odd_at_a": dodge_odd_at_a, "air_state_a": air_at_a,
            "z": pl1.pos.z, "speed": pl1.vel.length(),
            "ball_dist": (tp1.ball_pos - pl1.pos).length(),
            "methods": method_json, "fams": fams, "bvp": bvp.is_some(), "ar": ar_info,
        });
        writeln!(out, "{}", serde_json::to_string(&rec)?)?;
        *counts.entry("samples").or_default() += 1;
        done += 1;
        if done >= limit {
            break 'touches;
        }
    }
    eprintln!("{counts:?}");
    Ok(())
}


thread_local! {
    static TRACE_FRAMES: std::cell::RefCell<Vec<(usize, i64, Vec<CarTraceRow>)>> = const { std::cell::RefCell::new(Vec::new()) };
}

fn rotation_error_deg(a: Mat3A, b: Mat3A) -> f32 {
    let trace = a.x_axis.dot(b.x_axis) + a.y_axis.dot(b.y_axis) + a.z_axis.dot(b.z_axis);
    ((trace - 1.0) * 0.5).clamp(-1.0, 1.0).acos().to_degrees()
}

fn pk_air_state(truth: &Truth, tick: i64, k: usize) -> Option<u64> {
    truth.at(tick).map(|p| p.players[k].air_state)
}
