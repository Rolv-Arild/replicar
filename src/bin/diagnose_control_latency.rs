//! Is the delay between an observed control change and its effect a per-event random offset, or
//! something a fit can learn? Offline diagnostic on train/validation.
//!
//! Takes the flat-ground packet pairs of `diagnose_ground_driving` (same selection) where the
//! observed throttle, steer or handbrake changes within 16 ticks of the pair. For each pair, the
//! per-frame controls (switch times from the midpoint rule) are shifted later by d ticks
//! (d = -8..=8) and RocketSim is stepped from packet a to packet b; the error at b is a mix of
//! angular velocity (per 0.3 rad/s) and velocity (per 50 UU/s). The best d of a pair uses its own
//! target, so it is an in-sample fit. The held-out test: fit d on a pair and apply it to the next
//! consecutive pair of the same car (which shares packet b), compared with d = 0 and with the next
//! pair's own best d. If the delay were a stable property of the player or session, the carried d
//! would help; if it is per-event noise, it would not.

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Quat, Vec3A};
use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes};
use replay_to_rocketsim::observations::{Body, Car};
use rocketsim::{
    Arena, ArenaConfig, BallState, CarBodyConfig, CarControls, CarState, GameMode, Team,
};

const SHIFTS: std::ops::RangeInclusive<i64> = -8..=8;

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

type Phys = (Vec3A, Vec3A, Mat3A, Vec3A);

fn physics(body: &Body, frame: usize) -> Option<Phys> {
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

#[derive(Clone, Copy)]
struct Inputs {
    throttle: f32,
    steer: f32,
    handbrake: bool,
    boost: bool,
}

fn inputs_of(car: &Car) -> Inputs {
    Inputs {
        throttle: car.inputs.throttle.as_ref().map_or(0.0, |v| v.value),
        steer: car.inputs.steer.as_ref().map_or(0.0, |v| v.value),
        handbrake: car.inputs.handbrake.as_ref().is_some_and(|v| v.value),
        boost: car
            .inputs
            .boost_active_raw
            .as_ref()
            .is_some_and(|v| v.value % 2 == 1),
    }
}

struct Packet {
    frame: usize,
    tick: i64,
    phys: Phys,
    inputs: Inputs,
    counters: [Option<u8>; 4],
    hitbox: String,
}

/// (nominal frame tick, midpoint-rule start tick, inputs) for every active frame of a car.
type Timeline = Vec<(i64, i64, Inputs)>;

fn handbrake_value(timeline: &Timeline, tick: i64) -> f32 {
    let mut value = 0.0f32;
    for tau in tick - 120..=tick {
        let index = timeline.partition_point(|(_, start, _)| *start <= tau);
        let held = index > 0 && timeline[index - 1].2.handbrake;
        value = (value + if held { 5.0 } else { -2.0 } / 120.0).clamp(0.0, 1.0);
    }
    value
}

struct Pair<'a> {
    a: &'a Packet,
    b: &'a Packet,
}

/// Error at b for controls shifted later by `d` ticks: angular (per 0.3 rad/s) + velocity (per 50 UU/s).
fn evaluate(arena: &mut Arena, timeline: &Timeline, pair: &Pair, d: i64) -> (f32, f32) {
    evaluate_with_interior(arena, timeline, pair, d, None).0
}

/// As `evaluate`, and also the error at an interior packet that is not used by the fit.
fn evaluate_with_interior(
    arena: &mut Arena,
    timeline: &Timeline,
    pair: &Pair,
    d: i64,
    interior: Option<&Packet>,
) -> ((f32, f32), Option<(f32, f32)>) {
    let (a, b) = (pair.a, pair.b);
    let mut interior_error = None;
    let mut state = CarState::default();
    state.phys.pos = a.phys.0;
    state.phys.vel = a.phys.1;
    state.phys.rot_mat = a.phys.2;
    state.phys.ang_vel = a.phys.3;
    state.is_on_ground = true;
    state.wheels_with_contact = [Some(rocketsim::RaycastHitInfo::default()); 4];
    state.handbrake_val = handbrake_value(timeline, a.tick);
    arena.set_ball_state(parked_ball());
    arena.set_car_state(0, state);
    for tau in a.tick + 1..=b.tick {
        let index = timeline.partition_point(|(_, start, _)| *start + d <= tau);
        let inputs = if index == 0 {
            a.inputs
        } else {
            timeline[index - 1].2
        };
        arena.set_car_controls(
            0,
            CarControls {
                throttle: inputs.throttle,
                steer: inputs.steer,
                handbrake: inputs.handbrake,
                boost: inputs.boost,
                ..CarControls::default()
            },
        );
        arena.step_tick();
        if let Some(mid) = interior {
            if tau == mid.tick {
                let sim = arena.get_car_state(0);
                interior_error = Some((
                    (sim.phys.ang_vel - mid.phys.3).length(),
                    (sim.phys.vel - mid.phys.1).length(),
                ));
            }
        }
    }
    let sim = arena.get_car_state(0);
    (
        (
            (sim.phys.ang_vel - b.phys.3).length(),
            (sim.phys.vel - b.phys.1).length(),
        ),
        interior_error,
    )
}

/// The scratch arena's ball is parked out of reach; at kickoff it would sit in the way of cars
/// crossing the centre.
fn parked_ball() -> BallState {
    let mut ball = BallState::default();
    ball.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
    ball
}

fn cost((ang, vel): (f32, f32)) -> f32 {
    ang / 0.3 + vel / 50.0
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: diagnose_control_latency <train or validation dir or replay>")?,
    );
    if path.to_string_lossy().contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let options = ConvertOptions::default();
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut arenas: BTreeMap<String, Arena> = BTreeMap::new();

    // In-sample best shift per pair, and the carried-shift test on consecutive pairs.
    let mut best_hist: BTreeMap<i64, usize> = BTreeMap::new();
    let mut pair_count = 0usize;
    let mut own_gain: Vec<f32> = Vec::new(); // cost at d=0 minus cost at own best d
    let mut consecutive = 0usize;
    let mut same_shift = 0usize;
    let mut next_zero: Vec<[f32; 2]> = Vec::new();
    let mut next_carried: Vec<[f32; 2]> = Vec::new();
    let mut next_own: Vec<[f32; 2]> = Vec::new();
    let mut carried_better = 0usize;
    let mut carried_worse = 0usize;
    let mut carried_nonzero = 0usize;
    // Held-out interior packet: fit d on (a, c) and score at b.
    let mut held_zero: Vec<[f32; 2]> = Vec::new();
    let mut held_fit: Vec<[f32; 2]> = Vec::new();
    let mut held_gated: Vec<[f32; 2]> = Vec::new();
    let mut held_better = 0usize;
    let mut held_worse = 0usize;
    let mut held_moved = 0usize;

    for replay_path in replay_paths(&path)? {
        let output = convert_bytes(&fs::read(&replay_path)?, &options)?;
        let frames = &output.observations.frames;
        let first_time = frames[0].time;
        let tick_of = |f: usize| -> i64 {
            ((f64::from(frames[f].time) - f64::from(first_time)) * 120.0).round() as i64
        };
        let lag_of = |f: usize, actor: i32| {
            output.frames[f]
                .packet_lags
                .iter()
                .find(|l| l.actor_id == Some(actor) && l.source == "chain")
                .map(|l| l.ticks as i64)
        };
        let active = |f: usize| {
            frames[f]
                .game_state
                .as_ref()
                .is_some_and(|s| s.value == "Active")
        };
        let mut timelines: BTreeMap<(i32, usize), Timeline> = BTreeMap::new();
        let mut lifetimes: BTreeMap<(i32, usize), Vec<Packet>> = BTreeMap::new();
        for f in 0..frames.len() {
            if !active(f) {
                continue;
            }
            for car in &frames[f].cars {
                let key = (car.actor_id, car.actor_created_frame);
                let line = timelines.entry(key).or_default();
                let spacing = line.last().map_or(4, |(t, _, _)| tick_of(f) - t);
                line.push((tick_of(f), tick_of(f) - 2 - spacing / 2, inputs_of(car)));
                let (Some(phys), Some(lag)) = (physics(&car.body, f), lag_of(f, car.actor_id))
                else {
                    continue;
                };
                let Some(player) = car.player_key.as_ref() else {
                    continue;
                };
                let Some(slot) = output.car_slots.iter().find(|s| &s.player_key == player) else {
                    continue;
                };
                let raw = |x: &Option<replay_to_rocketsim::observations::Value<u8>>| {
                    x.as_ref().map(|v| v.value)
                };
                lifetimes.entry(key).or_default().push(Packet {
                    frame: f,
                    tick: tick_of(f) - lag,
                    phys,
                    inputs: inputs_of(car),
                    counters: [
                        raw(&car.inputs.jump_active_raw),
                        raw(&car.inputs.double_jump_active_raw),
                        raw(&car.inputs.dodge_active_raw),
                        raw(&car.inputs.flip_car_active_raw),
                    ],
                    hitbox: slot.hitbox.clone(),
                });
            }
        }
        for (key, packets) in &lifetimes {
            let timeline = &timelines[key];
            // Valid pairs among consecutive packets; remember which follow each other.
            let valid = |a: &Packet, b: &Packet| -> bool {
                let k = b.tick - a.tick;
                if !(2..=15).contains(&k)
                    || a.counters != b.counters
                    || a.counters.iter().flatten().any(|c| c % 2 == 1)
                    || a.inputs.boost != b.inputs.boost
                    || (a.frame..=b.frame).any(|f| !active(f))
                {
                    return false;
                }
                let flat = |p: &Packet| p.phys.0.z < 30.0 && p.phys.2.z_axis.z > 0.97;
                let clear = |p: &Packet| {
                    let ball_far = frames[p.frame].ball.as_ref().is_none_or(|ball| {
                        ball.position.as_ref().is_none_or(|bp| {
                            (Vec3A::from_array(bp.value) - p.phys.0).length() > 400.0
                        })
                    });
                    let cars_far = frames[p.frame].cars.iter().all(|o| {
                        o.body.position.as_ref().is_none_or(|op| {
                            let d = (Vec3A::from_array(op.value) - p.phys.0).length();
                            d < 1.0 || d > 400.0
                        })
                    });
                    ball_far && cars_far
                };
                flat(a) && flat(b) && clear(a) && clear(b)
            };
            // A control change within 16 ticks of the pair (throttle, steer or handbrake).
            let changes_nearby = |a: &Packet, b: &Packet| -> bool {
                let near: Vec<&Inputs> = timeline
                    .iter()
                    .filter(|(t, _, _)| *t >= a.tick - 16 && *t <= b.tick + 16)
                    .map(|(_, _, i)| i)
                    .collect();
                near.windows(2).any(|w| {
                    (w[0].throttle - w[1].throttle).abs() > 0.1
                        || (w[0].steer - w[1].steer).abs() > 0.1
                        || w[0].handbrake != w[1].handbrake
                })
            };
            let Some(first) = packets.first() else {
                continue;
            };
            let arena = arenas.entry(first.hitbox.clone()).or_insert_with(|| {
                let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
                arena.add_car(Team::Blue, hitbox(&first.hitbox));
                arena
            });
            // Per-pair error table over the shifts, for pairs that qualify.
            let mut table: Vec<Option<Vec<(f32, f32)>>> = Vec::new();
            for window in packets.windows(2) {
                let (a, b) = (&window[0], &window[1]);
                if valid(a, b) && changes_nearby(a, b) {
                    let pair = Pair { a, b };
                    table.push(Some(
                        SHIFTS
                            .map(|d| evaluate(arena, timeline, &pair, d))
                            .collect(),
                    ));
                } else {
                    table.push(None);
                }
            }
            let zero = (-*SHIFTS.start()) as usize;
            let best_of = |row: &Vec<(f32, f32)>| -> usize {
                // Ties (and an unchanged best) prefer d = 0.
                let mut best = zero;
                for (i, e) in row.iter().enumerate() {
                    if cost(*e) < cost(row[best]) - 1e-6 {
                        best = i;
                    }
                }
                best
            };
            for triple in packets.windows(3) {
                let (a, b, c) = (&triple[0], &triple[1], &triple[2]);
                if !(valid(a, b) && valid(b, c) && changes_nearby(a, c)) || c.tick - a.tick > 24 {
                    continue;
                }
                let outer = Pair { a, b: c };
                let mut best = zero;
                let mut costs = Vec::new();
                for d in SHIFTS {
                    costs.push(cost(evaluate(arena, timeline, &outer, d)));
                }
                for (i, e) in costs.iter().enumerate() {
                    if *e < costs[best] - 1e-6 {
                        best = i;
                    }
                }
                let d_fit = best as i64 - zero as i64;
                // Gated variant, fixed in advance: keep a shift only if it removes half of the
                // outer cost at d = 0.
                let d_gated = if costs[best] < 0.5 * costs[zero] {
                    d_fit
                } else {
                    0
                };
                let at = |arena: &mut Arena, d: i64| {
                    evaluate_with_interior(arena, timeline, &outer, d, Some(b))
                        .1
                        .map(|(x, y)| [x, y])
                };
                let (Some(z), Some(f), Some(g)) =
                    (at(arena, 0), at(arena, d_fit), at(arena, d_gated))
                else {
                    continue;
                };
                held_moved += usize::from(d_fit != 0);
                if d_fit != 0 {
                    let (cz, cf) = (cost((z[0], z[1])), cost((f[0], f[1])));
                    if cf < cz - 1e-6 {
                        held_better += 1;
                    } else if cf > cz + 1e-6 {
                        held_worse += 1;
                    }
                }
                held_zero.push(z);
                held_fit.push(f);
                held_gated.push(g);
            }
            for (i, row) in table.iter().enumerate() {
                let Some(row) = row else { continue };
                pair_count += 1;
                let best = best_of(row);
                *best_hist.entry(best as i64 - zero as i64).or_default() += 1;
                own_gain.push(cost(row[zero]) - cost(row[best]));
                if let Some(Some(next)) = table.get(i + 1) {
                    consecutive += 1;
                    let next_best = best_of(next);
                    same_shift += usize::from((next_best as i64 - best as i64).abs() <= 1);
                    next_zero.push([next[zero].0, next[zero].1]);
                    next_carried.push([next[best].0, next[best].1]);
                    next_own.push([next[next_best].0, next[next_best].1]);
                    if best != zero {
                        carried_nonzero += 1;
                        if cost(next[best]) < cost(next[zero]) - 1e-6 {
                            carried_better += 1;
                        } else if cost(next[best]) > cost(next[zero]) + 1e-6 {
                            carried_worse += 1;
                        }
                    }
                }
            }
        }
    }

    println!("pairs with a control change within 16 ticks: {pair_count}");
    println!("best shift d (ticks later than the midpoint rule) per pair, in sample:");
    for (d, n) in &best_hist {
        println!("  {d:+3}: {:.1}%", *n as f64 * 100.0 / pair_count as f64);
    }
    let mut gain = own_gain.clone();
    println!(
        "in-sample cost reduction from the pair's own best shift: p50 {:.3} p90 {:.3} (cost = ang/0.3 + vel/50)",
        quantile(&mut gain, 0.5),
        quantile(&mut gain, 0.9)
    );
    println!(
        "\nconsecutive pairs sharing a packet: {consecutive}; best shift within 1 tick of the previous pair's: {:.1}%",
        same_shift as f64 * 100.0 / consecutive.max(1) as f64
    );
    println!(
        "carrying the previous pair's fitted shift (nonzero in {carried_nonzero}): better than d=0 in {carried_better}, worse in {carried_worse}"
    );
    for (name, rows) in [
        ("next pair, d = 0 (midpoint rule)", &next_zero),
        ("next pair, previous pair's fitted d", &next_carried),
        ("next pair, its own best d (in sample)", &next_own),
    ] {
        let mut ang: Vec<f32> = rows.iter().map(|r| r[0]).collect();
        let mut vel: Vec<f32> = rows.iter().map(|r| r[1]).collect();
        println!(
            "  {name:<40} ang p50/p90 {:.3}/{:.3} rad/s, vel p50/p90 {:.1}/{:.1} UU/s",
            quantile(&mut ang, 0.5),
            quantile(&mut ang, 0.9),
            quantile(&mut vel, 0.5),
            quantile(&mut vel, 0.9)
        );
    }
    println!(
        "\nheld-out interior packet: shift fitted on (a, c), scored at b (not used by the fit); {} triples, shift moved in {held_moved} (better than d=0 in {held_better}, worse in {held_worse}):",
        held_zero.len()
    );
    for (name, rows) in [
        ("midpoint rule, d = 0", &held_zero),
        ("d fitted on (a, c)", &held_fit),
        ("d fitted only if it halves the cost", &held_gated),
    ] {
        let mut ang: Vec<f32> = rows.iter().map(|r| r[0]).collect();
        let mut vel: Vec<f32> = rows.iter().map(|r| r[1]).collect();
        println!(
            "  {name:<38} at b: ang p50/p90/p99 {:.3}/{:.3}/{:.3} rad/s, vel p50/p90/p99 {:.1}/{:.1}/{:.1} UU/s",
            quantile(&mut ang, 0.5),
            quantile(&mut ang, 0.9),
            quantile(&mut ang, 0.99),
            quantile(&mut vel, 0.5),
            quantile(&mut vel, 0.9),
            quantile(&mut vel, 0.99)
        );
    }
    Ok(())
}
