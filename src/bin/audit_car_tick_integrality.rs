//! Are replay car packets exact 120 Hz physics states? Same whole-tick test as the ball audit.
//!
//! For consecutive fresh packets of one car (up to three frames apart) while it is airborne, away
//! from walls, ceiling, the ball and other cars, with zero throttle and unchanged boost, jump and
//! dodge counters, the linear motion is ballistic and independent of the unknown pitch/roll
//! inputs (RocketSim cars have no air drag). Start RocketSim from the first packet, step whole
//! ticks k = 0..=24, and compare position and velocity with the second packet. Offline train
//! diagnostic; refuses paths containing "test".

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Quat, Vec3A};
use replicar_v1::{observations, parse_replay};
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarState, GameMode, Team};

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

struct Packet {
    frame: usize,
    time: f32,
    state: CarState,
    counters: [Option<u8>; 4],
    boost_raw: Option<u8>,
    throttle: Option<f32>,
}

fn quantile(values: &mut [f64], q: f64) -> f64 {
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn histogram(values: &[f64], edges: &[f64]) -> Vec<f64> {
    let mut counts = vec![0usize; edges.len() + 1];
    for &v in values {
        counts[edges.iter().position(|&e| v < e).unwrap_or(edges.len())] += 1;
    }
    counts
        .iter()
        .map(|&c| c as f64 * 100.0 / values.len() as f64)
        .collect()
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: audit_car_tick_integrality <train dir or replay>")?,
    );
    if replicar_v1::sealed_path_refused(&path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    let slot = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);

    let (mut pos_err, mut vel_err, mut along, mut grav) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let mut counts: BTreeMap<i64, usize> = BTreeMap::new();
    let mut interval_errors: Vec<f64> = Vec::new();
    let mut by_gap: [Vec<f64>; 4] = Default::default();

    for replay_path in replay_paths(&path)? {
        let replay = parse_replay(&fs::read(&replay_path)?)?;
        let observed = observations::extract(&replay).ok_or("network frames absent")?;
        let mut lifetimes: BTreeMap<(i32, usize), Vec<Packet>> = Default::default();
        for frame in &observed.frames {
            if !frame
                .game_state
                .as_ref()
                .is_some_and(|s| s.value == "Active")
            {
                continue;
            }
            let ball = frame
                .ball
                .as_ref()
                .and_then(|b| b.position.as_ref())
                .map(|p| Vec3A::from_array(p.value));
            for car in &frame.cars {
                let b = &car.body;
                let (Some(p), Some(v), Some(r), Some(w)) = (
                    b.position.as_ref().filter(|x| x.frame == frame.index),
                    b.linear_velocity
                        .as_ref()
                        .filter(|x| x.frame == frame.index),
                    b.rotation_xyzw.as_ref().filter(|x| x.frame == frame.index),
                    b.angular_velocity_replay_units
                        .as_ref()
                        .filter(|x| x.frame == frame.index),
                ) else {
                    continue;
                };
                let pos = Vec3A::from_array(p.value);
                let clear = pos.z > 300.0
                    && pos.z < 1900.0
                    && pos.x.abs() < 3600.0
                    && pos.y.abs() < 4600.0
                    && ball.is_none_or(|bp| (bp - pos).length() > 700.0)
                    && frame
                        .cars
                        .iter()
                        .filter(|o| o.actor_id != car.actor_id)
                        .all(|o| {
                            o.body.position.as_ref().is_none_or(|op| {
                                (Vec3A::from_array(op.value) - pos).length() > 700.0
                            })
                        });
                if !clear {
                    continue;
                }
                let quat = Quat::from_xyzw(r.value[0], r.value[1], r.value[2], r.value[3]);
                if !quat.is_finite() || quat.length_squared() < 0.5 {
                    continue;
                }
                let mut state = CarState::default();
                state.phys.pos = pos;
                state.phys.vel = Vec3A::from_array(v.value);
                state.phys.ang_vel = Vec3A::from_array(w.value) * 0.01;
                state.phys.rot_mat = Mat3A::from_quat(quat.normalize());
                state.is_on_ground = false;
                let raw = |x: &Option<observations::Value<u8>>| x.as_ref().map(|v| v.value);
                lifetimes
                    .entry((car.actor_id, car.actor_created_frame))
                    .or_default()
                    .push(Packet {
                        frame: frame.index,
                        time: frame.time,
                        state,
                        counters: [
                            raw(&car.inputs.jump_active_raw),
                            raw(&car.inputs.double_jump_active_raw),
                            raw(&car.inputs.dodge_active_raw),
                            raw(&car.inputs.flip_car_active_raw),
                        ],
                        boost_raw: raw(&car.inputs.boost_active_raw),
                        throttle: car.inputs.throttle.as_ref().map(|t| t.value),
                    });
            }
        }
        for packets in lifetimes.values() {
            for pair in packets.windows(2) {
                let (a, b) = (&pair[0], &pair[1]);
                let gap = b.frame - a.frame;
                if !(1..=3).contains(&gap)
                    || a.counters != b.counters
                    || a.counters.iter().flatten().any(|c| c % 2 == 1)
                    || a.boost_raw.is_none_or(|x| x % 2 == 1)
                    || b.boost_raw.is_none_or(|x| x % 2 == 1)
                    || a.throttle != Some(0.0)
                    || b.throttle != Some(0.0)
                    || (a.frame..=b.frame).any(|f| {
                        observed.frames[f]
                            .game_state
                            .as_ref()
                            .is_none_or(|s| s.value != "Active")
                    })
                    || a.state.phys.vel.length() < 300.0
                {
                    continue;
                }
                arena.set_car_state(slot, a.state);
                let mut best: Option<(usize, f32)> = None;
                let mut trace = Vec::new();
                for k in 0..=24usize {
                    if k > 0 {
                        arena.step_tick();
                    }
                    let s = arena.get_car_state(slot);
                    let err = (s.phys.pos - b.state.phys.pos).length();
                    trace.push((s.phys.pos, s.phys.vel));
                    if best.is_none_or(|(_, e)| err < e) {
                        best = Some((k, err));
                    }
                }
                let (k, err) = best.unwrap();
                let (pos, vel) = trace[k];
                let speed = b.state.phys.vel.length();
                pos_err.push(f64::from(err));
                vel_err.push(f64::from((vel - b.state.phys.vel).length()));
                along.push(
                    f64::from((pos - b.state.phys.pos).dot(b.state.phys.vel) / (speed * speed))
                        * 120.0,
                );
                grav.push(f64::from((vel - b.state.phys.vel).z) / (650.0 / 120.0));
                {
                    let mean = (a.state.phys.vel + b.state.phys.vel) * 0.5;
                    let implied_k = f64::from(
                        (b.state.phys.pos - a.state.phys.pos).dot(mean) / mean.length_squared(),
                    ) * 120.0;
                    if err < 0.05 {
                        interval_errors.push((implied_k - k as f64).abs());
                    }
                }
                *counts.entry(k as i64).or_default() += 1;
                by_gap[gap].push(f64::from(err));
                let _ = b.time - a.time;
            }
        }
    }
    let n = pos_err.len();
    println!("airborne car packet pairs (zero throttle, no boost/jump/dodge change): {n}");
    let edges = [0.01, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0];
    let labels = [
        "<0.01", "<0.05", "<0.1", "<0.25", "<0.5", "<1", "<2", "<5", "<10", ">=10",
    ];
    for (name, values) in [
        ("best-k position error (UU)", &pos_err),
        ("best-k velocity error (UU/s)", &vel_err),
    ] {
        print!("{name:<32}");
        for (l, v) in labels.iter().zip(histogram(values, &edges)) {
            print!(" {l}:{v:.1}%");
        }
        println!();
    }
    println!(
        "position error at best k: p10 {:.4} p50 {:.4} p90 {:.4} p99 {:.3} UU",
        quantile(&mut pos_err.clone(), 0.1),
        quantile(&mut pos_err.clone(), 0.5),
        quantile(&mut pos_err.clone(), 0.9),
        quantile(&mut pos_err.clone(), 0.99)
    );
    let tick_edges: Vec<f64> = (-10..=10).map(|i| f64::from(i) * 0.1).collect();
    for (name, values) in [
        ("along-velocity residual (ticks)", &along),
        ("gravity residual (ticks)", &grav),
    ] {
        let line: Vec<String> = histogram(values, &tick_edges)
            .iter()
            .map(|v| format!("{v:.1}"))
            .collect();
        println!(
            "\n{name}, share per 0.1-tick bin from -1 to +1:\n  {}",
            line.join(" ")
        );
        println!(
            "  |residual| < 0.02 tick: {:.1}%  < 0.1 tick: {:.1}%",
            values.iter().filter(|v| v.abs() < 0.02).count() as f64 * 100.0 / n as f64,
            values.iter().filter(|v| v.abs() < 0.1).count() as f64 * 100.0 / n as f64
        );
    }
    for (gap, values) in by_gap.iter_mut().enumerate().skip(1) {
        if !values.is_empty() {
            println!(
                "frame gap {gap}: n={} best-k position error p50 {:.4} p90 {:.4}",
                values.len(),
                quantile(values, 0.5),
                quantile(values, 0.9)
            );
        }
    }
    println!("\nbest whole-tick k between packets (count): {counts:?}");
    interval_errors.sort_by(|a, b| a.total_cmp(b));
    let q = |p: f64| interval_errors[((interval_errors.len() - 1) as f64 * p) as usize];
    println!(
        "car |estimated interval - exact k| (ticks), exact fits only, n={}: p50 {:.3} p90 {:.3} p99 {:.3} p99.9 {:.3} max {:.3}",
        interval_errors.len(),
        q(0.5),
        q(0.9),
        q(0.99),
        q(0.999),
        q(1.0)
    );
    Ok(())
}
