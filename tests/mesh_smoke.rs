use std::path::Path;

use rocketsim::{Arena, CarBodyConfig, GameMode, Team};

#[test]
fn supplied_soccar_meshes_support_a_simulation_tick() {
    let meshes = Path::new(env!("CARGO_MANIFEST_DIR")).join("collision_meshes");
    if !meshes.join("soccar").is_dir() {
        eprintln!("skipping mesh smoke test: no local collision_meshes/soccar directory");
        return;
    }

    rocketsim::init(&meshes, true).expect("load supplied collision meshes");
    let mut arena = Arena::new(GameMode::Soccar);
    let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.step_tick();
    let state = arena.get_arena_state();
    assert_eq!(state.tick_count, 1);
    assert_eq!(state.num_cars(), 1);
    assert!(arena.get_car_state(car).phys.pos.is_finite());
    assert!(state.ball.phys.pos.is_finite());
}
