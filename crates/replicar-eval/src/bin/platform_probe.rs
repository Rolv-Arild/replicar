//! Is RocketSim bit-for-bit the same on two platforms? Steps an arena with four cars on scripted controls (no
//! replay, no replicar inference) and prints a hash of every body's state every 1,000 ticks; run it on both and
//! compare the lines.
//!
//! ```text
//! platform_probe <collision_meshes> [every]   # a line every `every` ticks (1000)
//! platform_probe --libm                        # f32 sin, cos, powf and atan2 over a million inputs: how many differ
//! ```

use replicar::rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};
use sha2::{Digest, Sha256};

/// A deterministic pseudo-random number in 0..1 from integers (no floating-point library calls).
fn noise(a: u64, b: u64) -> f32 {
    let mut x = a.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ b.wrapping_mul(0xC2B2_AE3D_27D4_EB4F);
    x ^= x >> 33;
    x = x.wrapping_mul(0xFF51_AFD7_ED55_8CCD);
    x ^= x >> 33;
    (x >> 40) as f32 / (1u64 << 24) as f32
}

/// A named maths function of one `f32`.
type Function = fn(f32) -> f32;

/// Hashes of `f32::sin`, `cos`, `powf` and `atan2` over a million inputs (the platform's maths library).
fn libm() {
    let inputs: Vec<f32> = (0..1_000_000u64)
        .map(|i| (noise(i, 99) - 0.5) * 8.0)
        .collect();
    let functions: [(&str, Function); 4] = [
        ("sin", f32::sin),
        ("cos", f32::cos),
        ("powf", |x| (1.0 - x.abs() / 8.0).powf(1.0 / 120.0)),
        ("atan2", |x| x.atan2(0.75)),
    ];
    for (name, f) in functions {
        let mut hash = Sha256::new();
        for &x in &inputs {
            hash.update(f(x).to_bits().to_le_bytes());
        }
        println!("{name:6} {:x}", hash.finalize());
    }
    // The bits themselves, for comparing value by value: the first inputs.
    for &x in &inputs[..8] {
        println!(
            "{:08x} sin {:08x} cos {:08x}",
            x.to_bits(),
            x.sin().to_bits(),
            x.cos().to_bits()
        );
    }
}

fn main() {
    if std::env::args().nth(1).as_deref() == Some("--libm") {
        return libm();
    }
    let meshes = std::env::args()
        .nth(1)
        .expect("usage: platform_probe <collision_meshes>");
    replicar::Meshes::load(&meshes).expect("meshes");
    let every: u64 = std::env::args()
        .nth(2)
        .map_or(1000, |e| e.parse().expect("every"));
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    let cars: Vec<usize> = [Team::Blue, Team::Blue, Team::Orange, Team::Orange]
        .into_iter()
        .map(|team| arena.add_car(team, CarBodyConfig::OCTANE))
        .collect();
    arena.reset_to_random_kickoff(Some(7));
    let mut hash = Sha256::new();
    for tick in 1..=24_000u64 {
        // New controls every 8 ticks, with jumps, boost and handbrake now and then.
        let block = tick / 8;
        for &car in &cars {
            let n = |k: u64| noise(block * 16 + k, car as u64);
            let controls = CarControls {
                throttle: n(0) * 2.0 - 1.0,
                steer: n(1) * 2.0 - 1.0,
                pitch: n(2) * 2.0 - 1.0,
                yaw: n(3) * 2.0 - 1.0,
                roll: n(4) * 2.0 - 1.0,
                jump: n(5) < 0.15,
                boost: n(6) < 0.4,
                handbrake: n(7) < 0.1,
            };
            arena.set_car_controls(car, controls);
        }
        arena.step_tick();
        let ball = arena.get_ball_state().phys;
        for v in [ball.pos, ball.vel, ball.ang_vel] {
            for c in v.to_array() {
                hash.update(c.to_bits().to_le_bytes());
            }
        }
        for &car in &cars {
            let phys = arena.get_car_state(car).phys;
            for v in [phys.pos, phys.vel, phys.ang_vel] {
                for c in v.to_array() {
                    hash.update(c.to_bits().to_le_bytes());
                }
            }
        }
        if tick.is_multiple_of(every) {
            let digest = hash.clone().finalize();
            let p = arena.get_car_state(cars[0]).phys.pos;
            println!(
                "tick {tick:6} {:x}  car0 ({:.4}, {:.4}, {:.4})",
                digest, p.x, p.y, p.z
            );
        }
    }
}
