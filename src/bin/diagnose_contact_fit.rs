//! How far must a car packet's state move for RocketSim to reproduce a real ball touch? Train only.
//!
//! Selects the same touches as `diagnose_contact_model`: consecutive ball packets (A, B) whose
//! velocity change at B is more than 50 UU/s beyond a ball-only prediction, one car within 500 UU,
//! and a fresh chain-lag packet of that car at A's tick or one tick before it. The car and ball are
//! set from their packets and stepped to B's tick in a scratch arena, as there. Then the car's start
//! position is shifted over a small grid (along its horizontal travel, sideways, and vertically) and
//! the smallest shift whose simulated ball velocity at B is within 50 UU/s of the packet is
//! recorded. The ball packet at B is a future observation, so this is an offline fit, not a
//! prediction: it says whether the contact model can reproduce the touch with a car state that is
//! plausibly within the car's own trajectory uncertainty. That uncertainty is measured alongside:
//! the same car, simulated alone with the ball parked far away from its packet to its next fresh
//! packet, gives the position error over an ordinary packet interval.

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Quat, Vec3A};
use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes, step_arena_tick};
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

fn car_state(packet: &(Vec3A, Vec3A, Mat3A, Vec3A), shift: Vec3A) -> CarState {
    let mut state = CarState::default();
    state.phys.pos = packet.0 + shift;
    state.phys.vel = packet.1;
    state.phys.rot_mat = packet.2;
    state.phys.ang_vel = packet.3;
    let grounded = packet.0.z < 30.0;
    state.is_on_ground = grounded;
    state.wheels_with_contact = [grounded.then(rocketsim::RaycastHitInfo::default); 4];
    state
}

struct TouchRecord {
    chain: (PathBuf, i32, usize),
    stale: i64,
    /// The car packet arrived in the same replay frame as the ball packet at A.
    same_frame: bool,
    /// Whether the touch was reproduced with the car packet assumed -2..=2 ticks off.
    matched: [bool; 5],
}

/// Per-chain timing offsets chosen on half of a chain's touches and scored on the other half.
fn print_timing_check(records: &[TouchRecord]) {
    println!(
        "relative car/ball packet timing: matched touches when the car packet is assumed delta ticks later (+) or earlier (-) than the chains say"
    );
    for (stale, same_frame) in [(0i64, true), (0, false), (1, true), (1, false)] {
        let group: Vec<_> = records
            .iter()
            .filter(|r| r.stale == stale && r.same_frame == same_frame)
            .collect();
        print!(
            "  car packet {stale} tick(s) stale, {} frame as the ball packet, n {}:",
            if same_frame { "same" } else { "different" },
            group.len()
        );
        for (i, delta) in (-2..=2i64).enumerate() {
            let matched = group.iter().filter(|r| r.matched[i]).count();
            print!(
                " {delta:+}: {:.1}%",
                matched as f64 * 100.0 / group.len().max(1) as f64
            );
        }
        println!();
    }
    let any_of = |lo: usize, hi: usize| {
        records
            .iter()
            .filter(|r| r.matched[lo..=hi].iter().any(|m| *m))
            .count() as f64
            * 100.0
            / records.len().max(1) as f64
    };
    println!(
        "  per-touch oracle (best timing for each touch, which uses the ball packet at B): measured timing {:.1}%, best of -1..+1 {:.1}%, best of -2..+2 {:.1}% of {} touches",
        any_of(2, 2),
        any_of(1, 3),
        any_of(0, 4),
        records.len()
    );
    let mut chains: BTreeMap<&(PathBuf, i32, usize), Vec<&TouchRecord>> = BTreeMap::new();
    for record in records {
        chains.entry(&record.chain).or_default().push(record);
    }
    let (mut baseline, mut chosen, mut oracle, mut total) = (0usize, 0usize, 0usize, 0usize);
    let (mut chains_used, mut chains_moved) = (0usize, 0usize);
    let mut chosen_deltas = [0usize; 5];
    for touches in chains.values() {
        if touches.len() < 4 {
            continue;
        }
        chains_used += 1;
        // Best delta on all touches of the chain (in sample), for reference.
        let score =
            |subset: &[&TouchRecord], i: usize| subset.iter().filter(|r| r.matched[i]).count();
        let pick = |subset: &[&TouchRecord]| {
            // Ties prefer the measured timing (delta 0), then the smaller shift.
            (0..5usize)
                .max_by_key(|&i| (score(subset, i), std::cmp::Reverse(i.abs_diff(2))))
                .unwrap()
        };
        let all: Vec<&TouchRecord> = touches.to_vec();
        oracle += score(&all, pick(&all));
        // Split half: choose on one parity, score on the other.
        for held_out in 0..2usize {
            let (train, test): (Vec<_>, Vec<_>) = touches
                .iter()
                .enumerate()
                .partition(|(i, _)| i % 2 != held_out);
            let train: Vec<&TouchRecord> = train.into_iter().map(|(_, r)| *r).collect();
            let test: Vec<&TouchRecord> = test.into_iter().map(|(_, r)| *r).collect();
            let best = pick(&train);
            baseline += score(&test, 2);
            chosen += score(&test, best);
            total += test.len();
            chosen_deltas[best] += 1;
            chains_moved += usize::from(best != 2);
        }
    }
    println!(
        "  chains with >= 4 touches: {chains_used}; split-half (choose on one half of a chain's touches, score on the other), {total} touches: measured timing {:.1}% matched, per-chain delta {:.1}% (in-sample best per chain {:.1}%); chosen delta counts -2..+2 {:?} ({chains_moved} of {} choices moved)",
        baseline as f64 * 100.0 / total.max(1) as f64,
        chosen as f64 * 100.0 / total.max(1) as f64,
        oracle as f64 * 100.0 / total.max(1) as f64,
        chosen_deltas,
        chains_used * 2,
    );
}

#[derive(Default)]
struct Group {
    baseline_error: Vec<f32>,
    matched: usize,
    matched_at_zero: usize,
    shift: Vec<f32>,
    count: usize,
    car_base: Vec<f32>,
    car_fit: Vec<f32>,
    car_better: usize,
    local: [Vec<f32>; 3],
    gap: Vec<f32>,
    anchor_n: usize,
    anchor_base_match: usize,
    anchor_match: usize,
    anchor_base_error: Vec<f32>,
    anchor_error: Vec<f32>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: diagnose_contact_fit <train dir or replay>")?,
    );
    if replay_to_rocketsim::sealed_path_refused(&path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let options = ConvertOptions::default();
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut arenas: BTreeMap<String, Arena> = BTreeMap::new();
    let mut ball_only = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    let mut groups: BTreeMap<String, Group> = BTreeMap::new();
    // Position error of a car simulated alone over an ordinary packet interval, by interval length.
    let mut drift: BTreeMap<String, Vec<f32>> = BTreeMap::new();
    let mut touch_records: Vec<TouchRecord> = Vec::new();

    // Shift grid: along travel, sideways, vertical (UU).
    let along: [f32; 9] = [-16.0, -12.0, -8.0, -4.0, 0.0, 4.0, 8.0, 12.0, 16.0];
    let side: [f32; 5] = [-8.0, -4.0, 0.0, 4.0, 8.0];
    let up: [f32; 3] = [-4.0, 0.0, 4.0];

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
                        chosen = Some((c, p, tick_a - t, cf, t));
                        break;
                    }
                }
            }
            let Some((c, packet, stale, car_frame, car_tick)) = chosen else {
                continue;
            };
            let Some(player) = c.player_key.as_ref() else {
                continue;
            };
            let Some(slot) = output.car_slots.iter().find(|s| &s.player_key == player) else {
                continue;
            };
            let arena = arenas.entry(slot.hitbox.clone()).or_insert_with(|| {
                let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
                arena.add_car(Team::Blue, hitbox(&slot.hitbox));
                arena
            });
            let controls = CarControls {
                throttle: c.inputs.throttle.as_ref().map_or(0.0, |v| v.value),
                steer: c.inputs.steer.as_ref().map_or(0.0, |v| v.value),
                handbrake: c.inputs.handbrake.as_ref().is_some_and(|v| v.value),
                boost: c
                    .inputs
                    .boost_active_raw
                    .as_ref()
                    .is_some_and(|v| v.value % 2 == 1),
                ..CarControls::default()
            };

            // Car-alone drift to the same car's next fresh chain packet after this one.
            let mut next_car: Option<(i64, Vec3A, Vec3A)> = None;
            for nf in car_frame + 1..(car_frame + 6).min(frames.len()) {
                let Some(next) = frames[nf].cars.iter().find(|n| {
                    n.actor_id == car.actor_id && n.actor_created_frame == car.actor_created_frame
                }) else {
                    continue;
                };
                let (Some(lag), Some(next_packet)) =
                    (lag_of(nf, Some(next.actor_id)), physics(&next.body, nf))
                else {
                    continue;
                };
                let steps = tick(nf) - lag - car_tick;
                if !(1..=14).contains(&steps) || !active(nf) {
                    continue;
                }
                let mut parked = BallState::default();
                parked.phys.pos = Vec3A::new(0.0, 0.0, 1500.0);
                arena.set_ball_state(parked);
                arena.set_car_state(0, car_state(&packet, Vec3A::ZERO));
                arena.set_car_controls(0, controls);
                for _ in 0..steps {
                    step_arena_tick(arena);
                }
                let miss = next_packet.0 - arena.get_car_state(0).phys.pos;
                next_car = Some((steps, next_packet.0, miss));
                let error = (arena.get_car_state(0).phys.pos - next_packet.0).length();
                drift
                    .entry(format!("interval {steps:>2} ticks"))
                    .or_default()
                    .push(error);
                break;
            }

            let mut forward = Vec3A::new(packet.1.x, packet.1.y, 0.0);
            if forward.length() < 50.0 {
                forward = packet.2.x_axis;
                forward.z = 0.0;
            }
            let forward = forward.normalize_or_zero();
            let lateral = Vec3A::Z.cross(forward);

            // The car's own next packet is an independent check: it is not used to choose a shift.
            let follow = next_car.filter(|(steps, _, _)| *steps >= stale + k);
            let mut run = |shift: Vec3A| -> (bool, f32, Option<f32>) {
                // A car packet one tick before the ball packet first advances alone, so the car
                // and ball are both at A's tick when they start interacting.
                let mut parked = BallState::default();
                parked.phys.pos = Vec3A::new(0.0, 0.0, 1500.0);
                arena.set_ball_state(parked);
                arena.set_car_state(0, car_state(&packet, shift));
                arena.set_car_controls(0, controls);
                for _ in 0..stale {
                    step_arena_tick(arena);
                }
                arena.set_ball_state(ball_state);
                let mut hit = false;
                for _ in 0..k {
                    for event in step_arena_tick(arena) {
                        hit |= matches!(event, ArenaEvent::CarHitBall(_));
                    }
                }
                let error = (arena.get_ball_state().phys.vel - b.1).length();
                let car_error = follow.map(|(steps, target, _)| {
                    for _ in 0..(steps - stale - k) {
                        step_arena_tick(arena);
                    }
                    (arena.get_car_state(0).phys.pos - target).length()
                });
                (hit, error, car_error)
            };
            let (hit0, error0, car_error0) = run(Vec3A::ZERO);
            let mut best: Option<(f32, f32, Option<f32>)> = None; // (shift norm, error, next-packet car error)
            let mut best_shift: Option<Vec3A> = None;
            for &da in &along {
                for &ds in &side {
                    for &du in &up {
                        let shift = forward * da + lateral * ds + Vec3A::Z * du;
                        let (hit, error, car_error) = run(shift);
                        if hit && error < 50.0 {
                            let norm = shift.length();
                            if best.is_none_or(|(n, _, _)| norm < n) {
                                best = Some((norm, error, car_error));
                                best_shift = Some(shift);
                            }
                        }
                    }
                }
            }
            // Smallest gap between the car's hitbox and the ball's free-flight path over the
            // interval, both simulated without contact (UU; negative means penetration).
            let min_gap = {
                let config = hitbox(&slot.hitbox);
                let mut parked = BallState::default();
                parked.phys.pos = Vec3A::new(0.0, 0.0, 1500.0);
                arena.set_ball_state(parked);
                arena.set_car_state(0, car_state(&packet, Vec3A::ZERO));
                arena.set_car_controls(0, controls);
                ball_only.set_ball_state(ball_state);
                let mut min_gap = f32::MAX;
                for s in 0..=(stale + k) {
                    if s > 0 {
                        step_arena_tick(arena);
                    }
                    if s >= stale {
                        if s > stale {
                            ball_only.step_tick();
                        }
                        let state = arena.get_car_state(0);
                        let local = state.phys.rot_mat.transpose()
                            * (ball_only.get_ball_state().phys.pos - state.phys.pos)
                            - config.hitbox_pos_offset;
                        let q = local.abs() - config.hitbox_size * 0.5;
                        let gap = q.max(Vec3A::ZERO).length() + q.max_element().min(0.0) - 91.25;
                        min_gap = min_gap.min(gap);
                    }
                }
                min_gap
            };
            // Two-sided anchor: the car's own next packet is used to spread the miss it would
            // otherwise have there evenly over the interval (offline; the ball packet at B is not
            // used). Compared with the unanchored run on the same touches.
            let anchored = follow.map(|(steps, _, miss)| {
                let mut parked = BallState::default();
                parked.phys.pos = Vec3A::new(0.0, 0.0, 1500.0);
                arena.set_ball_state(parked);
                arena.set_car_state(0, car_state(&packet, Vec3A::ZERO));
                arena.set_car_controls(0, controls);
                let mut hit = false;
                for s in 1..=(stale + k) {
                    if s == stale + 1 {
                        arena.set_ball_state(ball_state);
                    }
                    for event in step_arena_tick(arena) {
                        hit |= s > stale && matches!(event, ArenaEvent::CarHitBall(_));
                    }
                    let mut state = *arena.get_car_state(0);
                    state.phys.pos += miss / steps as f32;
                    arena.set_car_state(0, state);
                }
                (hit, (arena.get_ball_state().phys.vel - b.1).length())
            });
            // Relative timing of the car and ball chains: re-run with the car packet assumed
            // `delta` ticks later or earlier than the chains say (delta 0 is as measured).
            let mut matched_by_delta = [false; 5];
            for (slot_index, delta) in (-2..=2i64).enumerate() {
                let pre = stale + delta;
                let mut far = BallState::default();
                far.phys.pos = Vec3A::new(0.0, 0.0, 1500.0);
                let mut hit = false;
                if pre >= 0 {
                    arena.set_ball_state(far);
                    arena.set_car_state(0, car_state(&packet, Vec3A::ZERO));
                    arena.set_car_controls(0, controls);
                    for _ in 0..pre {
                        step_arena_tick(arena);
                    }
                    arena.set_ball_state(ball_state);
                    for _ in 0..k {
                        for event in step_arena_tick(arena) {
                            hit |= matches!(event, ArenaEvent::CarHitBall(_));
                        }
                    }
                } else {
                    // The car packet is later than the ball's: the ball advances alone first.
                    let lead = -pre;
                    let mut far_car = car_state(&packet, Vec3A::ZERO);
                    far_car.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
                    far_car.phys.vel = Vec3A::ZERO;
                    far_car.is_on_ground = false;
                    far_car.wheels_with_contact = [None; 4];
                    arena.set_car_state(0, far_car);
                    arena.set_ball_state(ball_state);
                    for _ in 0..lead {
                        step_arena_tick(arena);
                    }
                    arena.set_car_state(0, car_state(&packet, Vec3A::ZERO));
                    arena.set_car_controls(0, controls);
                    for _ in 0..(k - lead) {
                        for event in step_arena_tick(arena) {
                            hit |= matches!(event, ArenaEvent::CarHitBall(_));
                        }
                    }
                }
                let error = (arena.get_ball_state().phys.vel - b.1).length();
                matched_by_delta[slot_index] = hit && error < 50.0;
            }
            touch_records.push(TouchRecord {
                chain: (replay_path.clone(), car.actor_id, car.actor_created_frame),
                stale,
                same_frame: car_frame == f - 1,
                matched: matched_by_delta,
            });
            let air = if a.0.z > 250.0 { "air" } else { "low" };
            // Smallest shift in the car's own frame (forward, right, up), for a systematic bias.
            let local_shift = best_shift.map(|shift| {
                [
                    shift.dot(packet.2.x_axis),
                    shift.dot(packet.2.y_axis),
                    shift.dot(packet.2.z_axis),
                ]
            });
            for label in [
                "all".to_string(),
                format!("hitbox {}", slot.hitbox),
                format!("{air}"),
                if hit0 && error0 < 50.0 {
                    "already matched".to_string()
                } else if hit0 {
                    "sim hit, ball velocity off".to_string()
                } else {
                    "sim NO hit".to_string()
                },
                format!("car packet {stale} tick(s) stale"),
            ] {
                let group = groups.entry(label).or_default();
                group.count += 1;
                group.baseline_error.push(error0);
                group.gap.push(min_gap);
                if let Some((anchored_hit, anchored_error)) = anchored {
                    group.anchor_n += 1;
                    group.anchor_base_match += usize::from(hit0 && error0 < 50.0);
                    group.anchor_match += usize::from(anchored_hit && anchored_error < 50.0);
                    group.anchor_base_error.push(error0);
                    group.anchor_error.push(anchored_error);
                }
                if hit0 && error0 < 50.0 {
                    group.matched_at_zero += 1;
                }
                if let Some((norm, _, car_error)) = best {
                    group.matched += 1;
                    group.shift.push(norm);
                    if norm > 0.0 {
                        if let Some(local) = local_shift {
                            for (axis, value) in local.iter().enumerate() {
                                group.local[axis].push(*value);
                            }
                        }
                    }
                    if let (Some(fitted), Some(base)) = (car_error, car_error0) {
                        group.car_base.push(base);
                        group.car_fit.push(fitted);
                        if fitted < base {
                            group.car_better += 1;
                        }
                    }
                }
            }
        }
    }

    print_timing_check(&touch_records);
    println!(
        "car-alone position drift over one ordinary packet interval (simulated from packet to next packet):"
    );
    for (label, values) in drift.iter_mut() {
        println!(
            "  {label}: n {:>6}  p50 {:>6.2}  p90 {:>6.2}  p99 {:>6.2} UU",
            values.len(),
            quantile(values, 0.5),
            quantile(values, 0.9),
            quantile(values, 0.99)
        );
    }
    println!(
        "\nreal ball touches (single car near, chain-lag packets, offline fit uses the ball packet at B):"
    );
    println!(
        "{:<28} {:>6} | {:>9} | {:>12} | {:>26}",
        "group", "n", "matched@0", "matched<=grid", "smallest shift p50/p90 UU"
    );
    for (label, group) in groups.iter_mut() {
        println!(
            "{:<28} {:>6} | {:>8.1}% | {:>11.1}% | {:>12.1}/{:>8.1}   (baseline ball velocity error p50 {:.1})",
            label,
            group.count,
            group.matched_at_zero as f64 * 100.0 / group.count as f64,
            group.matched as f64 * 100.0 / group.count as f64,
            quantile(&mut group.shift, 0.5),
            quantile(&mut group.shift, 0.9),
            quantile(&mut group.baseline_error, 0.5),
        );
        println!(
            "{:<28} min hitbox-to-ball gap over the interval (unshifted, no contact) p10/p50/p90/p99: {:.1}/{:.1}/{:.1}/{:.1} UU",
            "",
            quantile(&mut group.gap, 0.1),
            quantile(&mut group.gap, 0.5),
            quantile(&mut group.gap, 0.9),
            quantile(&mut group.gap, 0.99),
        );
        if group.anchor_n > 0 {
            println!(
                "{:<28} anchored to the car's next packet (n {}): matched {:.1}% (unanchored {:.1}%), ball velocity error p50/p90 {:.1}/{:.1} (unanchored {:.1}/{:.1})",
                "",
                group.anchor_n,
                group.anchor_match as f64 * 100.0 / group.anchor_n as f64,
                group.anchor_base_match as f64 * 100.0 / group.anchor_n as f64,
                quantile(&mut group.anchor_error, 0.5),
                quantile(&mut group.anchor_error, 0.9),
                quantile(&mut group.anchor_base_error, 0.5),
                quantile(&mut group.anchor_base_error, 0.9),
            );
        }
        if !group.local[0].is_empty() {
            let n = group.local[0].len();
            let mut text = String::new();
            for (name, values) in ["forward", "right", "up"]
                .iter()
                .zip(group.local.iter_mut())
            {
                text += &format!(
                    " {name} p10/p50/p90 {:.1}/{:.1}/{:.1}",
                    quantile(values, 0.1),
                    quantile(values, 0.5),
                    quantile(values, 0.9)
                );
            }
            println!(
                "{:<28} nonzero smallest shift in the car frame (n {n}):{text}",
                ""
            );
        }
        if !group.car_base.is_empty() {
            println!(
                "{:<28} car position error at the car's next packet, where the shift matched (n {}): unshifted p50/p90 {:.2}/{:.2}, shifted p50/p90 {:.2}/{:.2}, shifted closer in {:.1}%",
                "",
                group.car_base.len(),
                quantile(&mut group.car_base, 0.5),
                quantile(&mut group.car_base, 0.9),
                quantile(&mut group.car_fit, 0.5),
                quantile(&mut group.car_fit, 0.9),
                group.car_better as f64 * 100.0 / group.car_base.len() as f64,
            );
        }
    }
    Ok(())
}
