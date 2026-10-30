//! The jump holds set on the rows after the simulation change no state (`holds`): wherever a car holds the jump on
//! two ticks in a row with no ground jump pushing, RocketSim steps a restored row to the same state with the jump
//! released. Needs the collision meshes and a train replay, which are local; skips without them.

use std::path::{Path, PathBuf};

use replicar::rocketsim::{Arena, ArenaConfig, ArenaState, DemoMode, GameMode};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
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
        car.last_extra_hit_tick = None;
        arena.set_car_state_full(index, car);
        arena.refresh_car_sticky_gate(index);
    }
    arena.set_ball_state(state.ball);
    arena
}

#[test]
fn a_held_jump_without_a_push_changes_no_state() {
    let meshes = root().join("collision_meshes");
    let train = root().join("replays/train/2v2");
    if !meshes.join("soccar").is_dir() || !train.is_dir() {
        eprintln!("skipping holds test: local meshes or replays missing");
        return;
    }
    let meshes = replicar::Meshes::load(&meshes).expect("meshes");
    let mut files: Vec<PathBuf> = std::fs::read_dir(&train)
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "replay"))
        .collect();
    files.sort();
    let bytes = std::fs::read(&files[0]).expect("replay");
    let converter = replicar::Converter::new(&meshes, replicar::Config::default());
    let conversion = converter.convert(&bytes).expect("conversion");
    let mut checked = 0;
    for frame in &conversion.frames {
        let row = replicar_format::StateRow {
            frame: frame.frame,
            sim_tick: frame.sim_tick,
            state: frame.state.clone(),
        };
        let held: Vec<usize> = frame
            .state
            .cars
            .iter()
            .enumerate()
            .filter_map(|(p, car)| {
                let car = car.as_ref()?;
                (car.controls.jump
                    && car.previous_controls.jump
                    && !car.internals.is_jumping
                    && !car.internals.is_demoed)
                    .then_some(p)
            })
            .collect();
        if held.is_empty() {
            continue;
        }
        let state = replicar::restore::arena_state(&conversion.header, &row).expect("restore");
        for p in held {
            let mut released = state.clone();
            let slot = released
                .cars
                .iter()
                .position(|(info, _)| info.idx == p)
                .expect("car");
            released.cars[slot].1.controls.jump = false;
            let (mut a, mut b) = (arena_from(&state), arena_from(&released));
            a.step_tick();
            b.step_tick();
            for car in 0..a.num_cars() {
                let (x, y) = (a.get_car_state(car), b.get_car_state(car));
                assert!(
                    x.phys.pos == y.phys.pos
                        && x.phys.vel == y.phys.vel
                        && x.phys.ang_vel == y.phys.ang_vel
                        && x.has_jumped == y.has_jumped
                        && x.is_jumping == y.is_jumping
                        && x.has_double_jumped == y.has_double_jumped
                        && x.has_flipped == y.has_flipped
                        && x.is_flipping == y.is_flipping,
                    "sim tick {}: holding player {p}'s jump changed car {car}",
                    frame.sim_tick
                );
            }
            checked += 1;
        }
    }
    assert!(checked > 100, "only {checked} held ticks checked");
}
