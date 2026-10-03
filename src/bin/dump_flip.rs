//! One-step RocketSim test of the flips of a BakkesMod state dump, with the true controller inputs.
//!
//! For each dodge (the record where `b_isdodging` first shows, j0) RocketSim is started from the dump's
//! state at every record j of the flip with the flip state set from the true dodge direction
//! (`flip_rel_torque = (-dir.y, dir.x, 0)`, `dir` = (DodgeForward, DodgeStrafe)) and a flip time of
//! `4 (j - j0) + s` ticks; the press tick s (0..=4 ticks before record j0) is unknown and chosen per
//! dodge to minimise the error of the first three steps. It steps 4 ticks with (a) the true inputs of j
//! held, (b) the same with pitch 0 (no cancel), (c) each cancel in 0, 0.25 .. 1 (oracle, best per
//! step), and compares with record j+1. Shows whether the true inputs explain the flip.
//!
//! usage: dump_flip <dump.json>

use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Vec3A};
use rocketsim::{
    Arena, ArenaConfig, BallState, CarBodyConfig, CarControls, CarState, GameMode, Team,
};
use serde_json::Value;

fn vec3(v: &Value, keys: [&str; 3]) -> Vec3A {
    Vec3A::new(
        v[keys[0]].as_f64().unwrap_or(0.0) as f32,
        v[keys[1]].as_f64().unwrap_or(0.0) as f32,
        v[keys[2]].as_f64().unwrap_or(0.0) as f32,
    )
}

/// Unreal rotator (pitch, yaw, roll in 1/65536 turn) to the rotation matrix with columns
/// forward, right, up.
fn matrix(v: &Value) -> Mat3A {
    let k = std::f32::consts::PI / 32768.0;
    let (p, y, r) = (
        v["Pitch"].as_f64().unwrap_or(0.0) as f32 * k,
        v["Yaw"].as_f64().unwrap_or(0.0) as f32 * k,
        v["Roll"].as_f64().unwrap_or(0.0) as f32 * k,
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

struct Sample {
    pos: Vec3A,
    vel: Vec3A,
    rot: Mat3A,
    ang: Vec3A,
}

fn sample(v: &Value) -> Sample {
    Sample {
        pos: vec3(&v["location"], ["X", "Y", "Z"]),
        vel: vec3(&v["Velocity"], ["X", "Y", "Z"]),
        rot: matrix(&v["Rotation"]),
        ang: vec3(&v["AngularVelocity"], ["X", "Y", "Z"]),
    }
}

fn controls(inputs: &Value) -> CarControls {
    let f = |k: &str| inputs[k].as_f64().unwrap_or(0.0) as f32;
    CarControls {
        throttle: f("Throttle"),
        steer: f("Steer"),
        pitch: f("Pitch"),
        yaw: f("Yaw"),
        roll: f("Roll"),
        jump: f("Jump") > 0.5,
        boost: f("ActivateBoost") > 0.5,
        handbrake: f("Handbrake") > 0.5,
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: dump_flip <dump.json>")?,
    );
    if replay_to_rocketsim::sealed_path_refused(&path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let dump: Value = serde_json::from_str(&fs::read_to_string(&path)?)?;
    let frames = dump["frames"].as_array().ok_or("frames")?;
    let st = |j: usize| &frames[j]["players"][0]["state"];
    let flag = |j: usize, key: &str| st(j)[key].as_bool().unwrap_or(false);
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    arena.add_car(Team::Blue, CarBodyConfig::OCTANE);

    // One-step (rotation degrees, angular velocity rad/s) error for the flip state at record j.
    let mut run = |j: usize, j0: usize, flip_ticks: i64, control: CarControls| -> (f32, f32) {
        let (sa, sb) = (st(j), st(j + 1));
        let car_a = sample(sa);
        let car_b = sample(sb);
        let ball_a = sample(&frames[j]["ball"]);
        let (forward, strafe) = (
            st(j0)["inputs"]["DodgeForward"].as_f64().unwrap_or(0.0) as f32,
            st(j0)["inputs"]["DodgeStrafe"].as_f64().unwrap_or(0.0) as f32,
        );
        let dir = glam::Vec2::new(forward, strafe).normalize_or_zero();
        let mut state = CarState::default();
        state.phys.pos = car_a.pos;
        state.phys.vel = car_a.vel;
        state.phys.rot_mat = car_a.rot;
        state.phys.ang_vel = car_a.ang;
        state.boost = sa["boostAmount"].as_f64().unwrap_or(0.0) as f32 * 100.0;
        state.is_on_ground = false;
        state.has_jumped = true;
        state.has_double_jumped = true;
        state.has_flipped = true;
        state.is_flipping = true;
        state.flip_rel_torque = Vec3A::new(-dir.y, dir.x, 0.0);
        state.flip_time = flip_ticks as f32 / 120.0;
        state.air_time_since_jump = sa["time_offGround"].as_f64().unwrap_or(0.0) as f32;
        arena.set_car_state(0, state);
        let mut ball = BallState::default();
        ball.phys.pos = ball_a.pos;
        ball.phys.vel = ball_a.vel;
        ball.phys.rot_mat = ball_a.rot;
        ball.phys.ang_vel = ball_a.ang;
        arena.set_ball_state(ball);
        arena.set_car_controls(0, control);
        for _ in 0..4 {
            arena.step_tick();
        }
        let mut end = *arena.get_car_state(0);
        // The game caps the angular speed at 5.5 rad/s; RocketSim's reported state can exceed it.
        let speed = end.phys.ang_vel.length();
        if speed > 5.5 {
            end.phys.ang_vel *= 5.5 / speed;
        }
        if env::var_os("FLIP_TRACE").is_some() && j0 == 138 {
            eprintln!(
                "j {j} ticks {flip_ticks} pitch {:.2} sim ang {:?} true ang {:?} start ang {:?} flip_time_after {:.3} is_flipping {}",
                control.pitch,
                end.phys
                    .ang_vel
                    .to_array()
                    .map(|x| (x * 100.0).round() / 100.0),
                car_b.ang.to_array().map(|x| (x * 100.0).round() / 100.0),
                car_a.ang.to_array().map(|x| (x * 100.0).round() / 100.0),
                end.flip_time,
                end.is_flipping
            );
        }
        (
            rotation_error(end.phys.rot_mat, car_b.rot),
            (end.phys.ang_vel - car_b.ang).length(),
        )
    };

    let max_steps = 8;
    let variants = [
        "true inputs",
        "pitch 0 (no cancel)",
        "oracle cancel (best of 5)",
    ];
    let mut rot: Vec<Vec<Vec<f32>>> = vec![vec![Vec::new(); max_steps]; variants.len()];
    let mut ang: Vec<Vec<Vec<f32>>> = vec![vec![Vec::new(); max_steps]; variants.len()];
    let mut best_cancels: Vec<Vec<f32>> = vec![Vec::new(); max_steps];
    let mut dodges = 0;
    for j0 in 1..frames.len() - 1 {
        if !(flag(j0, "b_isdodging") && !flag(j0 - 1, "b_isdodging")) {
            continue;
        }
        let forward = st(j0)["inputs"]["DodgeForward"].as_f64().unwrap_or(0.0) as f32;
        let sign = if forward >= 0.0 { 1.0 } else { -1.0 };
        let end = (j0..frames.len() - 1)
            .take_while(|&j| flag(j, "b_isdodging"))
            .last()
            .unwrap_or(j0);
        // The unknown press tick: the offset of 0..=4 ticks that best explains the first 3 steps.
        let mut best = (f32::INFINITY, 0i64);
        for s in 0..=4i64 {
            let mut total = 0.0;
            for j in j0..=(j0 + 2).min(end) {
                let c = controls(&st(j)["inputs"]);
                let (r, a) = run(j, j0, 4 * (j - j0) as i64 + s, c);
                total += r + 10.0 * a;
            }
            if total < best.0 {
                best = (total, s);
            }
        }
        dodges += 1;
        for j in j0..=end.min(j0 + max_steps - 1) {
            let idx = j - j0;
            let ticks = 4 * idx as i64 + best.1;
            let base = controls(&st(j)["inputs"]);
            let a = run(j, j0, ticks, base);
            let mut no_cancel = base;
            no_cancel.pitch = 0.0;
            let b = run(j, j0, ticks, no_cancel);
            let mut oracle = (f32::INFINITY, 0.0f32, (0.0f32, 0.0f32));
            for c in 0..=4 {
                let cancel = c as f32 * 0.25;
                let mut control = base;
                control.pitch = cancel * sign;
                let step = run(j, j0, ticks, control);
                let score = step.0 + 10.0 * step.1;
                if score < oracle.0 {
                    oracle = (score, cancel, step);
                }
            }
            for (v, step) in [(0, a), (1, b), (2, oracle.2)] {
                rot[v][idx].push(step.0);
                ang[v][idx].push(step.1);
            }
            best_cancels[idx].push(oracle.1);
        }
        println!(
            "dodge record {j0}: press {} tick(s) before the record, {} flip records, true signed pitch by record {:?}",
            best.1,
            end - j0 + 1,
            (j0..=end.min(j0 + 5))
                .map(
                    |j| (st(j)["inputs"]["Pitch"].as_f64().unwrap_or(0.0) as f32 * sign * 100.0)
                        .round()
                        / 100.0
                )
                .collect::<Vec<_>>()
        );
    }
    println!(
        "\n{dodges} dodges; one-step error at flip record index i (4 ticks each) after the dodge record; p50 / p90"
    );
    for (v, name) in variants.iter().enumerate() {
        println!("{name}");
        for i in 0..max_steps {
            println!(
                "  i={i} n {:>2}  rot deg {:>5.2} / {:>5.2}   ang rad/s {:>5.3} / {:>5.3}",
                rot[v][i].len(),
                quantile(&mut rot[v][i], 0.5),
                quantile(&mut rot[v][i], 0.9),
                quantile(&mut ang[v][i], 0.5),
                quantile(&mut ang[v][i], 0.9),
            );
        }
    }
    println!("oracle cancel by step (median / mean)");
    for i in 0..max_steps {
        let mean = best_cancels[i].iter().sum::<f32>() / best_cancels[i].len().max(1) as f32;
        println!(
            "  i={i}: {:.2} / {:.2}",
            quantile(&mut best_cancels[i], 0.5),
            mean
        );
    }
    Ok(())
}
