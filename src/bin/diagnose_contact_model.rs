//! With a nearly exact car state, does RocketSim reproduce a ball touch? Train replays only.
//!
//! Takes consecutive ball frames (A, B) with a real touch (the ball's velocity at B differs from a
//! ball-only prediction by more than 50 UU/s) where one car within 400 UU has a chain-lag packet at
//! the physical tick of A or one tick before it. In a scratch arena holding only that car (with its
//! hitbox and observed throttle, steer, handbrake and boost) and the ball, both are set from their
//! packets at A's tick and stepped to B's tick. Compares the simulated ball velocity with the packet
//! at B, for the whole set and by the number of ticks the contact had to be simulated, and reports
//! whether RocketSim produced a hit. If errors stay large here, the limit is the contact model (or
//! geometry) and not the car state.

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Quat, Vec3A};
use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes, step_tick_with_hit_impulse};
use replay_to_rocketsim::observations::Body;
use rocketsim::{
    Arena, ArenaConfig, ArenaEvent, BallState, CarBodyConfig, CarControls, CarState, GameMode, Team,
};

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

fn quantile(values: &mut [f32], q: f64) -> f32 {
    if values.is_empty() {
        return f32::NAN;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn physics(body: &Body, frame: usize) -> Option<(Vec3A, Vec3A, Mat3A, Vec3A)> {
    let pos = body.position.as_ref().filter(|v| v.frame == frame)?.value;
    let vel = body
        .linear_velocity
        .as_ref()
        .filter(|v| v.frame == frame)?
        .value;
    let rot = body
        .rotation_xyzw
        .as_ref()
        .filter(|v| v.frame == frame)?
        .value;
    let ang = body
        .angular_velocity_replay_units
        .as_ref()
        .filter(|v| v.frame == frame)?
        .value;
    let quat = Quat::from_xyzw(rot[0], rot[1], rot[2], rot[3]);
    if !quat.is_finite() || quat.length_squared() < 0.5 {
        return None;
    }
    Some((
        Vec3A::from_array(pos),
        Vec3A::from_array(vel),
        Mat3A::from_quat(quat.normalize()),
        Vec3A::from_array(ang) * 0.01,
    ))
}

fn hitbox(name: &str) -> CarBodyConfig {
    match name {
        "breakout" => CarBodyConfig::BREAKOUT,
        "dominus" => CarBodyConfig::DOMINUS,
        "hybrid" => CarBodyConfig::HYBRID,
        "merc" => CarBodyConfig::MERC,
        "plank" => CarBodyConfig::PLANK,
        "psyclops" => CarBodyConfig::PSYCLOPS,
        _ => CarBodyConfig::OCTANE,
    }
}

#[derive(Default)]
struct Group {
    error: Vec<f32>,
    impulse: Vec<f32>,
    sim_impulse: Vec<f32>,
    direction: Vec<f32>,
    hits: usize,
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: diagnose_contact_model <train dir or replay>")?,
    );
    if path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let options = ConvertOptions::default();
    // Optional second argument: RocketSim's ball_hit_extra_force_scale (default 1).
    let extra_scale: f32 = env::args_os()
        .nth(2)
        .and_then(|v| v.to_string_lossy().parse().ok())
        .unwrap_or(1.0);
    println!("ball_hit_extra_force_scale = {extra_scale}");
    let apply_hit = !env::args_os().any(|arg| arg == "--no-apply-hit-impulse");
    println!("apply reported extra hit impulse: {apply_hit}");
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut arenas: BTreeMap<String, Arena> = BTreeMap::new();
    let mut ball_only = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    let mut groups: BTreeMap<String, Group> = BTreeMap::new();

    for replay_path in replay_paths(&path)? {
        let output = convert_bytes(&fs::read(&replay_path)?, &options)?;
        let frames = &output.observations.frames;
        let first_time = frames[0].time;
        let tick =
            |f: usize| ((f64::from(frames[f].time) - f64::from(first_time)) * 120.0).round() as i64;
        let lag_of = |f: usize, actor: Option<i32>| {
            output.frames[f]
                .packet_lags
                .iter()
                .find(|l| l.actor_id == actor && l.source == "chain")
                .map(|l| l.ticks as i64)
        };
        for f in 1..frames.len() {
            let active = |i: usize| {
                frames[i]
                    .game_state
                    .as_ref()
                    .is_some_and(|s| s.value == "Active")
            };
            if !active(f) || !active(f - 1) {
                continue;
            }
            let (Some(lag_a), Some(lag_b)) = (lag_of(f - 1, None), lag_of(f, None)) else {
                continue;
            };
            let (Some(ball_a), Some(ball_b)) =
                (frames[f - 1].ball.as_ref(), frames[f].ball.as_ref())
            else {
                continue;
            };
            let (Some(a), Some(b)) = (physics(ball_a, f - 1), physics(ball_b, f)) else {
                continue;
            };
            let (tick_a, tick_b) = (tick(f - 1) - lag_a, tick(f) - lag_b);
            let k = tick_b - tick_a;
            if !(2..=10).contains(&k) {
                continue;
            }
            // Ball-only reference for the real touch.
            let mut ball_state = BallState::default();
            ball_state.phys.pos = a.0;
            ball_state.phys.vel = a.1;
            ball_state.phys.rot_mat = a.2;
            ball_state.phys.ang_vel = a.3;
            ball_only.set_ball_state(ball_state);
            for _ in 0..k {
                ball_only.step_tick();
            }
            let free = ball_only.get_ball_state().phys.vel;
            let impulse = (b.1 - free).length();
            if impulse <= 50.0 {
                continue;
            }
            // A car within 400 UU with a chain packet at tick_a or tick_a - 1, and nothing else near.
            let near: Vec<_> = frames[f - 1]
                .cars
                .iter()
                .filter(|c| {
                    c.body
                        .position
                        .as_ref()
                        .is_some_and(|p| (Vec3A::from_array(p.value) - a.0).length() < 500.0)
                })
                .collect();
            if near.len() != 1 {
                continue;
            }
            let car = near[0];
            // Find a fresh chain-lag packet of this car at tick_a or tick_a - 1 among frames f-3..=f.
            let mut chosen = None;
            for cf in f.saturating_sub(3)..=f {
                let Some(c) = frames[cf].cars.iter().find(|c| {
                    c.actor_id == car.actor_id && c.actor_created_frame == car.actor_created_frame
                }) else {
                    continue;
                };
                let Some(lag) = lag_of(cf, Some(c.actor_id)) else {
                    continue;
                };
                let t = tick(cf) - lag;
                if (tick_a - 1..=tick_a).contains(&t) {
                    if let Some(p) = physics(&c.body, cf) {
                        chosen = Some((c, p, tick_a - t));
                        break;
                    }
                }
            }
            let Some((c, packet, stale)) = chosen else {
                continue;
            };
            let Some(player) = c.player_key.as_ref() else {
                continue;
            };
            let Some(slot) = output.car_slots.iter().find(|s| &s.player_key == player) else {
                continue;
            };
            let arena = arenas.entry(slot.hitbox.clone()).or_insert_with(|| {
                let mut config = ArenaConfig::new(GameMode::Soccar);
                config.mutators.ball_hit_extra_force_scale = extra_scale;
                let mut arena = Arena::new_with_config(config);
                arena.add_car(Team::Blue, hitbox(&slot.hitbox));
                arena
            });
            let mut car_state = CarState::default();
            car_state.phys.pos = packet.0;
            car_state.phys.vel = packet.1;
            car_state.phys.rot_mat = packet.2;
            car_state.phys.ang_vel = packet.3;
            car_state.is_on_ground = packet.0.z < 30.0;
            car_state.wheels_with_contact =
                [(packet.0.z < 30.0).then(rocketsim::RaycastHitInfo::default); 4];
            arena.set_car_state(0, car_state);
            arena.set_ball_state(ball_state);
            arena.set_car_controls(
                0,
                CarControls {
                    throttle: c.inputs.throttle.as_ref().map_or(0.0, |v| v.value),
                    steer: c.inputs.steer.as_ref().map_or(0.0, |v| v.value),
                    handbrake: c.inputs.handbrake.as_ref().is_some_and(|v| v.value),
                    boost: c
                        .inputs
                        .boost_active_raw
                        .as_ref()
                        .is_some_and(|v| v.value % 2 == 1),
                    ..CarControls::default()
                },
            );
            let mut hit = false;
            for _ in 0..k {
                for event in step_tick_with_hit_impulse(arena, apply_hit) {
                    hit |= matches!(event, ArenaEvent::CarHitBall(_));
                }
            }
            let sim_velocity = arena.get_ball_state().phys.vel;
            // Also run on to the following ball packet, once the contact has finished.
            let mut later: Option<(f32, f32, f32)> = None;
            if f + 1 < frames.len() {
                if let (Some(lag_c), Some(ball_c)) =
                    (lag_of(f + 1, None), frames[f + 1].ball.as_ref())
                {
                    if let Some(c_state) = physics(ball_c, f + 1) {
                        let tick_c = tick(f + 1) - lag_c;
                        let extra = tick_c - tick_b;
                        if (1..=10).contains(&extra) {
                            for _ in 0..extra {
                                step_tick_with_hit_impulse(arena, apply_hit);
                            }
                            // ball-only reference from A to C
                            ball_only.set_ball_state(ball_state);
                            for _ in 0..(tick_c - tick_a) {
                                ball_only.step_tick();
                            }
                            let free_c = ball_only.get_ball_state().phys.vel;
                            let sim_c = arena.get_ball_state().phys.vel;
                            later = Some((
                                (sim_c - c_state.1).length(),
                                (c_state.1 - free_c).length(),
                                (sim_c - free_c).length(),
                            ));
                        }
                    }
                }
            }
            if let Some((error_c, impulse_c, sim_impulse_c)) = later {
                for label in [
                    "at the NEXT ball packet (all)".to_string(),
                    format!("at the NEXT ball packet, car {} tick(s) stale", stale),
                ] {
                    let group = groups.entry(label).or_default();
                    group.error.push(error_c);
                    group.impulse.push(impulse_c);
                    group.sim_impulse.push(sim_impulse_c);
                    group.hits += usize::from(hit);
                }
            }
            let error = (sim_velocity - b.1).length();
            let sim_impulse = (sim_velocity - free).length();
            let cosine = if impulse > 1.0 && sim_impulse > 1.0 {
                (sim_velocity - free).dot(b.1 - free) / (impulse * sim_impulse)
            } else {
                f32::NAN
            };
            let air = if a.0.z > 250.0 { "air" } else { "low" };
            let dodging = c
                .inputs
                .dodge_active_raw
                .as_ref()
                .is_some_and(|d| d.value % 2 == 1);
            let jumping = c
                .inputs
                .jump_active_raw
                .as_ref()
                .is_some_and(|d| d.value % 2 == 1)
                || c.inputs
                    .double_jump_active_raw
                    .as_ref()
                    .is_some_and(|d| d.value % 2 == 1);
            let boosting = c
                .inputs
                .boost_active_raw
                .as_ref()
                .is_some_and(|v| v.value % 2 == 1);
            let car_kind = if dodging {
                "car dodging"
            } else if jumping {
                "car jumping"
            } else if packet.0.z > 50.0 {
                "car airborne (no dodge/jump)"
            } else {
                "car on ground"
            };
            for label in [
                "all".to_string(),
                format!("{air}"),
                format!(
                    "ticks simulated {}",
                    if k <= 4 {
                        "2-4"
                    } else if k <= 7 {
                        "5-7"
                    } else {
                        "8-10"
                    }
                ),
                format!("car packet {} tick(s) stale", stale),
                if hit {
                    "sim hit".to_string()
                } else {
                    "sim NO hit".to_string()
                },
                car_kind.to_string(),
                format!("{car_kind}, boost {}", if boosting { "on" } else { "off" }),
            ] {
                let group = groups.entry(label).or_default();
                group.error.push(error);
                group.impulse.push(impulse);
                group.sim_impulse.push(sim_impulse);
                if !cosine.is_nan() {
                    group.direction.push(cosine);
                }
                group.hits += usize::from(hit);
            }
        }
    }

    println!(
        "real ball touches simulated from a nearly exact car and ball (single car near the ball):"
    );
    println!(
        "{:<26} {:>6} {:>7} | {:>24} | {:>16} | {:>16} | {:>18}",
        "group",
        "n",
        "hit%",
        "ball vel err p50/90/99",
        "real impulse p50",
        "sim impulse p50",
        "impulse direction cos p10/p50"
    );
    for (label, group) in groups.iter_mut() {
        println!(
            "{:<26} {:>6} {:>6.1}% | {:>7.1}/{:>7.1}/{:>7.1} | {:>16.1} | {:>16.1} | {:>8.2}/{:>8.2}",
            label,
            group.error.len(),
            group.hits as f64 * 100.0 / group.error.len() as f64,
            quantile(&mut group.error, 0.5),
            quantile(&mut group.error, 0.9),
            quantile(&mut group.error, 0.99),
            quantile(&mut group.impulse, 0.5),
            quantile(&mut group.sim_impulse, 0.5),
            quantile(&mut group.direction, 0.1),
            quantile(&mut group.direction, 0.5),
        );
    }
    Ok(())
}
