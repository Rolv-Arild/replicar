//! Restoration from a file (story 7.5): converts each replay keeping the simulated RocketSim states, writes it
//! as float32 and as quantized, reads every row's state back (`replicar_format::read_states`), restores it
//! (`replicar::restore::arena_state`) and compares it with the simulated state. float32: bodies, boost,
//! controls, internals and pads equal, rotations within 1e-6 per matrix element, wheels as touching or not.
//! quantized: the bodies within their quanta (rotation within 1e-4 per element), the rest equal.
//!
//! usage: `restore_check <replay or folder>... [--final-assessment]`

use std::path::PathBuf;
use std::process::ExitCode;

use replicar::rocketsim::{ArenaState, CarState, PhysState};
use replicar_eval::collect_replays;

/// The largest differences of one comparison: position, velocity, angular velocity, rotation element, and the
/// count of other fields that differ.
#[derive(Debug, Default, Clone, Copy)]
struct Worst {
    position: f32,
    velocity: f32,
    angular_velocity: f32,
    rotation: f32,
    other: usize,
}

fn phys(worst: &mut Worst, a: &PhysState, b: &PhysState) {
    worst.position = worst.position.max((a.pos - b.pos).abs().max_element());
    worst.velocity = worst.velocity.max((a.vel - b.vel).abs().max_element());
    worst.angular_velocity = worst
        .angular_velocity
        .max((a.ang_vel - b.ang_vel).abs().max_element());
    for (x, y) in [
        (a.rot_mat.x_axis, b.rot_mat.x_axis),
        (a.rot_mat.y_axis, b.rot_mat.y_axis),
        (a.rot_mat.z_axis, b.rot_mat.z_axis),
    ] {
        worst.rotation = worst.rotation.max((x - y).abs().max_element());
    }
}

/// The car state without its body and with the wheels reduced to touching or not, for an exact comparison.
fn rest(car: &CarState) -> String {
    let mut car = *car;
    car.phys = CarState::default().phys;
    let wheels = car.wheels_with_contact.map(|w| w.is_some());
    car.wheels_with_contact = [None; 4];
    format!("{car:?} {wheels:?}")
}

fn compare(worst: &mut Worst, simulated: &ArenaState, restored: &ArenaState) {
    phys(worst, &simulated.ball.phys, &restored.ball.phys);
    worst.other += usize::from(simulated.tick_count != restored.tick_count);
    worst.other += usize::from(
        simulated.ball.tick_count_since_kickoff != restored.ball.tick_count_since_kickoff,
    );
    worst.other += usize::from(simulated.cars.len() != restored.cars.len());
    for ((info_a, a), (info_b, b)) in simulated.cars.iter().zip(&restored.cars) {
        phys(worst, &a.phys, &b.phys);
        worst.other += usize::from(info_a.idx != info_b.idx || info_a.team != info_b.team);
        worst.other += usize::from(rest(a) != rest(b));
    }
    for ((config_a, a), (config_b, b)) in simulated.boost_pads.iter().zip(&restored.boost_pads) {
        worst.other += usize::from(config_a.pos != config_b.pos || a.cooldown != b.cooldown);
    }
}

fn main() -> ExitCode {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let final_assessment = args.iter().any(|a| a == "--final-assessment");
    args.retain(|a| a != "--final-assessment");
    let roots: Vec<PathBuf> = args.iter().map(PathBuf::from).collect();
    let (replays, meshes) = match (
        collect_replays(&roots, final_assessment),
        replicar::Meshes::load("collision_meshes"),
    ) {
        (Ok(replays), Ok(meshes)) => (replays, meshes),
        (Err(error), _) => {
            eprintln!("error: {error}");
            return ExitCode::FAILURE;
        }
        (_, Err(error)) => {
            eprintln!("error: {error}");
            return ExitCode::FAILURE;
        }
    };
    let converter = replicar::Converter::new(&meshes, replicar::Config::default());
    let directory = std::env::temp_dir().join(format!("replicar-restore-{}", std::process::id()));
    if let Err(error) = std::fs::create_dir_all(&directory) {
        eprintln!("error: {error}");
        return ExitCode::FAILURE;
    }
    let mut failed = false;
    for path in &replays {
        let outcome = (|| -> Result<[Worst; 2], Box<dyn std::error::Error>> {
            let bytes = std::fs::read(path)?;
            let network = replicar::decode::decode(&replicar::parse(&bytes)?)?;
            let mut states: Vec<ArenaState> = Vec::new();
            let conversion = converter
                .convert_network_with(network, |frame| states.push(frame.state.clone()))?;
            let mut worst = [Worst::default(); 2];
            for (k, precision) in [
                replicar_format::Precision::Float32,
                replicar_format::Precision::Quantized,
            ]
            .into_iter()
            .enumerate()
            {
                let file = directory.join("x.parquet");
                conversion.write(
                    &file,
                    &replicar_format::WriteOptions {
                        precision,
                        ..Default::default()
                    },
                )?;
                for (frame, restored) in replicar::restore::arena_states(&file)? {
                    compare(&mut worst[k], &states[frame.get()], &restored);
                }
            }
            Ok(worst)
        })();
        match outcome {
            Ok([exact, quantized]) => {
                let ok = exact.position == 0.0
                    && exact.velocity == 0.0
                    && exact.angular_velocity == 0.0
                    && exact.rotation <= 1e-6
                    && exact.other == 0
                    && quantized.position <= 0.005_01
                    && quantized.velocity <= 0.005_01
                    && quantized.angular_velocity <= 5.01e-5
                    && quantized.rotation <= 1e-4
                    && quantized.other == 0;
                failed |= !ok;
                println!(
                    "{}  {}  float32 {exact:?}  quantized {quantized:?}",
                    if ok { "equal    " } else { "DIFFERENT" },
                    path.display()
                );
            }
            Err(error) => {
                failed = true;
                println!("FAILED     {}  {error}", path.display());
            }
        }
    }
    let _ = std::fs::remove_dir_all(&directory);
    if failed {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}
