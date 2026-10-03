//! One-step RocketSim test against a BakkesMod state dump with the true controller inputs.
//!
//! The dump has one record per 4 physical ticks with the local car's true state and inputs, and the
//! ball. For each consecutive pair (j, j+1) RocketSim is started from the dump's car and ball state at
//! j, driven with the true inputs at j (held), at j+1, or their mean, stepped 4 ticks, and compared
//! with the dump at j+1. Separates physics and input-sampling error from everything the converter has
//! to infer. Dump rotations are Unreal rotators (1/65536 turn), velocities UU/s, angular velocity rad/s.
//!
//! usage: dump_onestep <dump.json>

use std::collections::BTreeMap;
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

fn mix(a: CarControls, b: CarControls) -> CarControls {
    CarControls {
        throttle: (a.throttle + b.throttle) / 2.0,
        steer: (a.steer + b.steer) / 2.0,
        pitch: (a.pitch + b.pitch) / 2.0,
        yaw: (a.yaw + b.yaw) / 2.0,
        roll: (a.roll + b.roll) / 2.0,
        jump: a.jump,
        boost: a.boost || b.boost,
        handbrake: a.handbrake || b.handbrake,
    }
}

#[derive(Default)]
struct Rows {
    pos: Vec<f32>,
    vel: Vec<f32>,
    rot: Vec<f32>,
    ang: Vec<f32>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: dump_onestep <dump.json>")?,
    );
    if replay_to_rocketsim::sealed_path_refused(&path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let dump: Value = serde_json::from_str(&fs::read_to_string(&path)?)?;
    let frames = dump["frames"].as_array().ok_or("frames")?;
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let mut rows: BTreeMap<String, Rows> = BTreeMap::new();

    for j in 0..frames.len() - 1 {
        let (a, b) = (&frames[j]["players"][0], &frames[j + 1]["players"][0]);
        let (sa, sb) = (&a["state"], &b["state"]);
        let car_a = sample(sa);
        let car_b = sample(sb);
        let ball_a = sample(&frames[j]["ball"]);
        let ball_b = sample(&frames[j + 1]["ball"]);
        let on_ground = sa["time_onGround"].as_f64().unwrap_or(0.0) > 0.0;
        let on_ground_next = sb["time_onGround"].as_f64().unwrap_or(0.0) > 0.0;
        let dodging = sa["b_isdodging"].as_bool().unwrap_or(false)
            || sb["b_isdodging"].as_bool().unwrap_or(false);
        let jumped =
            sa["b_jumped"].as_bool().unwrap_or(false) || sb["b_jumped"].as_bool().unwrap_or(false);
        let jump_input = sa["inputs"]["Jump"].as_f64().unwrap_or(0.0) > 0.5
            || sb["inputs"]["Jump"].as_f64().unwrap_or(0.0) > 0.5;
        let boosting = sa["inputs"]["ActivateBoost"].as_f64().unwrap_or(0.0) > 0.5
            || sb["inputs"]["ActivateBoost"].as_f64().unwrap_or(0.0) > 0.5;
        let near_ball = (car_a.pos - ball_a.pos).length() < 300.0;
        let category = if dodging {
            "flipping (dodge)"
        } else if jump_input || (jumped && sa["time_offGround"].as_f64().unwrap_or(9.0) < 0.3) {
            "jump window"
        } else if on_ground && on_ground_next {
            if boosting {
                "ground, boosting"
            } else {
                "ground, no boost"
            }
        } else if !on_ground && !on_ground_next {
            if boosting {
                "air, boosting"
            } else {
                "air, no boost"
            }
        } else {
            "landing or takeoff"
        };
        for (label, control) in [
            ("hold controls of j", controls(&sa["inputs"])),
            ("controls of j+1", controls(&sb["inputs"])),
            (
                "mean of j and j+1",
                mix(controls(&sa["inputs"]), controls(&sb["inputs"])),
            ),
        ] {
            let mut state = CarState::default();
            state.phys.pos = car_a.pos;
            state.phys.vel = car_a.vel;
            state.phys.rot_mat = car_a.rot;
            state.phys.ang_vel = car_a.ang;
            state.boost = sa["boostAmount"].as_f64().unwrap_or(0.0) as f32 * 100.0;
            state.is_on_ground = on_ground;
            state.wheels_with_contact = [on_ground.then(rocketsim::RaycastHitInfo::default); 4];
            state.has_jumped = sa["b_jumped"].as_bool().unwrap_or(false);
            state.has_double_jumped = sa["b_doubledJumped"].as_bool().unwrap_or(false);
            state.has_flipped = sa["b_isdodging"].as_bool().unwrap_or(false);
            state.is_flipping = state.has_flipped;
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
            let end = arena.get_car_state(0);
            for group in [
                "all".to_string(),
                category.to_string(),
                if near_ball {
                    "near ball".to_string()
                } else {
                    "away from ball".to_string()
                },
            ] {
                let rows = rows.entry(format!("{label:<20} | {group}")).or_default();
                rows.pos.push((end.phys.pos - car_b.pos).length());
                rows.vel.push((end.phys.vel - car_b.vel).length());
                rows.rot.push(rotation_error(end.phys.rot_mat, car_b.rot));
                rows.ang.push((end.phys.ang_vel - car_b.ang).length());
            }
        }
        let _ = ball_b;
    }
    println!(
        "{:<44} {:>5} | {:>16} | {:>16} | {:>16} | {:>16}",
        "controls | group",
        "n",
        "pos UU p50/90/99",
        "vel UU/s p50/90",
        "rot deg p50/90",
        "ang rad/s p50/90"
    );
    for (label, r) in rows.iter_mut() {
        println!(
            "{:<44} {:>5} | {:>5.2}/{:>5.2}/{:>5.1} | {:>6.1}/{:>7.1} | {:>6.2}/{:>6.2} | {:>6.3}/{:>6.3}",
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
