//! How much of replicar's input error the replay could show: from replicar's state at a frame, the next frames are
//! simulated twice, with replicar's own controls and with the recorded true inputs (`scripts/ml/truth_controls.py`),
//! and each car's simulated state is compared with the replay's updates of that car at their update ticks. Where
//! the true inputs reproduce the updates much better, a better search could recover them; where both reproduce
//! them equally well, the difference is invisible to the replay and only a prior over inputs can help.
//!
//! Every car of the arena gets the variant's controls (true inputs where recorded, else replicar's); a car's
//! updates count only when its true inputs cover every tick of the window. Windows start every `stride` frames and
//! end `horizon` frames later; only windows inside continuous simulated play count.
//!
//! With `[true_update_ticks.csv]` (`slot`, `frame`, `sim_tick`, from `truth_controls.py`) every update is scored at
//! its true tick instead of the inferred one, and updates without one are left out: that separates the inputs from
//! update-tick inference.
//!
//! usage: identifiability <replay> <truth.csv> <collision_meshes> <out.json> [stride] [horizon] [true_update_ticks.csv]

use std::collections::{BTreeMap, HashMap, VecDeque};
use std::error::Error;
use std::fs;

use glam::{Mat3A, Quat, Vec3A};
use replicar::decode::NetworkReplay;
use replicar::rocketsim::{Arena, ArenaConfig, ArenaState, CarControls, DemoMode, GameMode};
use replicar::simulate::SimulatedFrame;
use serde::Serialize;

/// The variants: which controls take the true input (throttle, steer, pitch, yaw, roll, jump, boost, handbrake).
const VARIANTS: [(&str, [bool; 8]); 6] = [
    ("replicar", [false; 8]),
    ("truth", [true; 8]),
    (
        "truth jump",
        [false, false, false, false, false, true, false, false],
    ),
    (
        "truth boost",
        [false, false, false, false, false, false, true, false],
    ),
    (
        "truth throttle steer handbrake",
        [true, true, false, false, false, false, false, true],
    ),
    (
        "truth pitch yaw roll",
        [false, false, true, true, true, false, false, false],
    ),
];

fn mixed(ours: CarControls, true_: CarControls, mask: [bool; 8]) -> CarControls {
    CarControls {
        throttle: if mask[0] {
            true_.throttle
        } else {
            ours.throttle
        },
        steer: if mask[1] { true_.steer } else { ours.steer },
        pitch: if mask[2] { true_.pitch } else { ours.pitch },
        yaw: if mask[3] { true_.yaw } else { ours.yaw },
        roll: if mask[4] { true_.roll } else { ours.roll },
        jump: if mask[5] { true_.jump } else { ours.jump },
        boost: if mask[6] { true_.boost } else { ours.boost },
        handbrake: if mask[7] {
            true_.handbrake
        } else {
            ours.handbrake
        },
    }
}

fn arena_from(state: &ArenaState) -> Arena {
    let mut config = ArenaConfig::new(GameMode::Soccar);
    config.mutators.demo_mode = DemoMode::Disabled;
    let mut arena = Arena::new_with_config(config);
    let mut cars: Vec<_> = state.cars.iter().collect();
    cars.sort_by_key(|(info, _)| info.idx);
    for (info, car) in cars {
        let index = arena.add_car(info.team, info.config);
        let mut car = *car;
        // The hit tick as the same age in the new arena's tick count (0).
        car.last_extra_hit_tick = car
            .last_extra_hit_tick
            .and_then(|t| 0u64.checked_sub(state.tick_count.checked_sub(t)?));
        arena.set_car_state_full(index, car);
        arena.refresh_car_sticky_gate(index);
    }
    arena.set_ball_state(state.ball);
    for (i, (_, pad)) in state.boost_pads.iter().enumerate() {
        if i < arena.num_boost_pads() {
            arena.set_boost_pad_state(i, *pad);
        }
    }
    arena
}

fn rotation_degrees(a: Mat3A, b: Mat3A) -> f32 {
    let delta = Quat::from_mat3a(&(a * b.transpose())).normalize();
    let (_, angle) = delta.to_axis_angle();
    let angle = if angle > std::f32::consts::PI {
        2.0 * std::f32::consts::PI - angle
    } else {
        angle
    };
    angle.to_degrees()
}

#[derive(Default)]
struct Samples {
    errors: Vec<Vec<[f32; 4]>>,
}

#[derive(Serialize)]
struct Variant {
    #[serde(flatten)]
    quantiles: Quantiles,
    /// The share of updates where this variant reproduces the position at least 1 UU (and 20%) better than
    /// replicar's controls, and worse by as much.
    clearly_better: f32,
    clearly_worse: f32,
    /// The same at the replay's precision: at least 0.05 UU and 20%.
    finely_better: f32,
    finely_worse: f32,
}

#[derive(Serialize)]
struct Row {
    samples: usize,
    variants: BTreeMap<&'static str, Variant>,
}

#[derive(Serialize)]
struct Quantiles {
    position_p50_p90: [f32; 2],
    velocity_p50_p90: [f32; 2],
    rotation_degrees_p50_p90: [f32; 2],
    angular_velocity_p50_p90: [f32; 2],
}

fn quantiles(values: &[[f32; 4]]) -> Quantiles {
    let q = |k: usize| {
        let mut v: Vec<f32> = values.iter().map(|e| e[k]).collect();
        v.sort_by(f32::total_cmp);
        let at = |p: f32| {
            v.get(((v.len() as f32 - 1.0) * p).round() as usize)
                .copied()
                .unwrap_or(f32::NAN)
        };
        [at(0.5), at(0.9)]
    };
    Quantiles {
        position_p50_p90: q(0),
        velocity_p50_p90: q(1),
        rotation_degrees_p50_p90: q(2),
        angular_velocity_p50_p90: q(3),
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 5 {
        return Err("usage: identifiability <replay> <truth.csv> <collision_meshes> <out.json> [stride] [horizon]".into());
    }
    let stride: usize = args.get(5).map_or(Ok(4), |s| s.parse())?;
    let horizon: usize = args.get(6).map_or(Ok(4), |s| s.parse())?;
    let true_ticks: Option<HashMap<(usize, usize), u64>> = args
        .get(7)
        .map(|path| -> Result<_, Box<dyn Error>> {
            let mut map = HashMap::new();
            for line in fs::read_to_string(path)?.lines() {
                let v: Vec<i64> = line.split(',').filter_map(|x| x.parse().ok()).collect();
                if v.len() == 3 && v[2] >= 0 {
                    map.insert((v[0] as usize, v[1] as usize), v[2] as u64);
                }
            }
            Ok(map)
        })
        .transpose()?;
    let mut truth: HashMap<(usize, u64), CarControls> = HashMap::new();
    for line in fs::read_to_string(&args[2])?.lines() {
        let v: Vec<f32> = line
            .split(',')
            .map(|x| x.parse().unwrap_or(f32::NAN))
            .collect();
        if v.len() < 10 {
            continue;
        }
        truth.insert(
            (v[0] as usize, v[1] as u64),
            CarControls {
                throttle: v[2],
                steer: v[3],
                pitch: v[4],
                yaw: v[5],
                roll: v[6],
                jump: v[7] > 0.5,
                boost: v[8] > 0.5,
                handbrake: v[9] > 0.5,
            },
        );
    }
    let bytes = fs::read(&args[1])?;
    let meshes = replicar::Meshes::load(std::path::Path::new(&args[3]))?;
    let converter = replicar::Converter::new(&meshes, replicar::Config::default());
    let network: NetworkReplay = replicar::decode::decode(&replicar::parse(&bytes)?)?;
    let frames_net = network.frames.clone();
    let mut window: VecDeque<SimulatedFrame> = VecDeque::new();
    let mut seen = 0usize;
    // (horizon frame, situation) -> samples
    let mut groups: BTreeMap<(usize, &'static str), Samples> = BTreeMap::new();
    // With IDENT_DUMP=<path>: one line per scored update (frame, slot, horizon, situation, then the position error
    // of each variant), for inspecting windows.
    let mut dump: Vec<String> = Vec::new();
    let dumping = std::env::var_os("IDENT_DUMP").is_some();
    let mut evaluate = |frames: &VecDeque<SimulatedFrame>| {
        let start = &frames[0];
        if frames.iter().skip(1).any(|f| f.ticks.is_empty()) {
            return;
        }
        let t0 = start.state.tick_count;
        let t_end = frames[horizon].state.tick_count;
        // Replicar's controls per tick and slot, from the tick records.
        let mut controls: HashMap<u64, Vec<CarControls>> = HashMap::new();
        for f in frames.iter().skip(1) {
            for record in &f.ticks {
                controls.insert(
                    record.sim_tick,
                    record.cars.iter().map(|c| c.controls).collect(),
                );
            }
        }
        if (t0..t_end).any(|t| !controls.contains_key(&t)) {
            return;
        }
        let slots = start.state.cars.len();
        let covered: Vec<bool> = (0..slots)
            .map(|s| (t0..t_end).all(|t| truth.contains_key(&(s, t))))
            .collect();
        if !covered.iter().any(|&c| c) {
            return;
        }
        // The updates to score: (tick, slot, frame, horizon index, body).
        let mut targets = Vec::new();
        for (h, f) in frames.iter().enumerate().skip(1) {
            let net = &frames_net[f.index.get()];
            for applied in &f.applied_ticks {
                let Some(actor) = applied.car else { continue };
                let Some(&(_, player)) = f.car_players.iter().find(|(a, _)| *a == actor) else {
                    continue;
                };
                let slot = player.get();
                if slot >= slots || !covered[slot] {
                    continue;
                }
                let Some(car) = net.cars.iter().find(|c| c.life.actor == actor) else {
                    continue;
                };
                let tick = match &true_ticks {
                    Some(map) => match map.get(&(slot, f.index.get())) {
                        Some(&tick) => tick,
                        None => continue,
                    },
                    None => f.state.tick_count.saturating_sub(applied.ticks),
                };
                if tick <= t0 || tick > t_end {
                    continue;
                }
                targets.push((tick, slot, h, car.body.clone(), f.index));
            }
        }
        if targets.is_empty() {
            return;
        }
        // The situation of each car at the start.
        let ball = start.state.ball.phys.pos;
        let situation = |slot: usize| -> &'static str {
            let (_, car) = start
                .state
                .cars
                .iter()
                .find(|(i, _)| i.idx == slot)
                .unwrap();
            let near_ball = (car.phys.pos - ball).length() < 400.0;
            let near_car = start
                .state
                .cars
                .iter()
                .any(|(i, c)| i.idx != slot && (c.phys.pos - car.phys.pos).length() < 300.0);
            match (car.is_on_ground(), near_ball || near_car) {
                (true, false) => "ground",
                (true, true) => "ground near ball or car",
                (false, false) if car.is_flipping => "air flipping",
                (false, false) => "air",
                (false, true) => "air near ball or car",
            }
        };
        let mut arenas: Vec<Arena> = VARIANTS.iter().map(|_| arena_from(&start.state)).collect();
        let mut at_tick: Vec<HashMap<u64, Vec<replicar::rocketsim::CarState>>> =
            VARIANTS.iter().map(|_| HashMap::new()).collect();
        for t in t0..t_end {
            for (variant, arena) in arenas.iter_mut().enumerate() {
                for slot in 0..slots {
                    let ours = controls[&t].get(slot).copied().unwrap_or_default();
                    let c = truth
                        .get(&(slot, t))
                        .map_or(ours, |&true_| mixed(ours, true_, VARIANTS[variant].1));
                    arena.set_car_controls(slot, c);
                }
                arena.step_tick();
                at_tick[variant]
                    .insert(t + 1, (0..slots).map(|s| *arena.get_car_state(s)).collect());
            }
        }
        for (tick, slot, h, body, frame) in targets {
            let fresh = |f: Option<replicar::FrameIndex>| f == Some(frame);
            let Some(position) = body.position.as_ref().filter(|p| fresh(Some(p.frame))) else {
                continue;
            };
            let mut errors = vec![[0f32; 4]; VARIANTS.len()];
            for variant in 0..VARIANTS.len() {
                let car = at_tick[variant][&tick][slot];
                let p = Vec3A::from(position.value);
                errors[variant][0] = (car.phys.pos - p).length();
                errors[variant][1] = body
                    .linear_velocity
                    .as_ref()
                    .filter(|v| fresh(Some(v.frame)))
                    .map_or(f32::NAN, |v| (car.phys.vel - Vec3A::from(v.value)).length());
                errors[variant][2] = body.rotation.as_ref().map_or(f32::NAN, |r| {
                    let [x, y, z, w] = r.value;
                    rotation_degrees(
                        car.phys.rot_mat,
                        Mat3A::from_quat(Quat::from_xyzw(x, y, z, w).normalize()),
                    )
                });
                errors[variant][3] = body
                    .angular_velocity_raw
                    .as_ref()
                    .filter(|v| fresh(Some(v.frame)))
                    .map_or(f32::NAN, |v| {
                        (car.phys.ang_vel - Vec3A::from(v.value) * 0.01).length()
                    });
            }
            if dumping {
                let errors: Vec<String> = errors.iter().map(|e| format!("{:.4}", e[0])).collect();
                dump.push(format!(
                    "{},{slot},{h},{},{}",
                    frame.get(),
                    situation(slot),
                    errors.join(",")
                ));
            }
            for key in [(h, situation(slot)), (h, "all")] {
                let group = groups.entry(key).or_default();
                group.errors.resize(VARIANTS.len(), Vec::new());
                for (variant, e) in errors.iter().enumerate() {
                    group.errors[variant].push(*e);
                }
            }
        }
    };
    converter.convert_network_with(network, |frame| {
        window.push_back(frame.clone());
        if window.len() > horizon {
            if seen.is_multiple_of(stride) {
                evaluate(&window);
            }
            seen += 1;
            window.pop_front();
        }
    })?;
    let mut out: BTreeMap<String, Row> = BTreeMap::new();
    for ((h, situation), samples) in &groups {
        let ours = &samples.errors[0];
        let n = ours.len();
        let better_by = |a: &[[f32; 4]], b: &[[f32; 4]], margin: f32| {
            a.iter()
                .zip(b)
                .filter(|(x, y)| x[0] + margin < y[0] && x[0] < 0.8 * y[0])
                .count() as f32
                / n.max(1) as f32
        };
        let better = |a: &[[f32; 4]], b: &[[f32; 4]]| better_by(a, b, 1.0);
        let variants = VARIANTS
            .iter()
            .zip(&samples.errors)
            .map(|((name, _), e)| {
                (
                    *name,
                    Variant {
                        quantiles: quantiles(e),
                        clearly_better: better(e, ours),
                        clearly_worse: better(ours, e),
                        finely_better: better_by(e, ours, 0.05),
                        finely_worse: better_by(ours, e, 0.05),
                    },
                )
            })
            .collect();
        out.insert(
            format!("h{h} {situation}"),
            Row {
                samples: n,
                variants,
            },
        );
    }
    fs::write(&args[4], serde_json::to_string_pretty(&out)?)?;
    if let Some(path) = std::env::var_os("IDENT_DUMP") {
        fs::write(path, dump.join("\n") + "\n")?;
    }
    for (k, r) in &out {
        print!("{k:28} n {:6} pos p90 /better/worse:", r.samples);
        for (name, v) in &r.variants {
            print!(
                " | {name} {:.2} {:.2} {:.2}",
                v.quantiles.position_p50_p90[1], v.clearly_better, v.clearly_worse
            );
        }
        println!();
    }
    Ok(())
}
