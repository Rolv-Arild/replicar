use rocketsim::{Arena, CarBodyConfig, CarControls, CarState, GameMode, Team, Vec3A};
use std::path::Path;

fn main() {
    let meshes = Path::new("collision_meshes");
    rocketsim::init(&meshes, true).expect("init meshes");
    let mut arena = Arena::new(GameMode::Soccar);
    let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);

    // Test setting is_flipping directly on CarState:
    let mut state = CarState::default();
    state.phys.pos = Vec3A::new(0.0, 0.0, 300.0);
    state.is_on_ground = false;
    state.wheels_with_contact = [None; 4];
    state.has_jumped = true;
    state.air_time_since_jump = 0.2;
    state.has_flipped = true;
    state.is_flipping = true;
    state.flip_rel_torque = Vec3A::new(0.0, 1.0, 0.0); // forward flip
    state.flip_time = 0.0;
    arena.set_car_state(car, state);

    let initial_vel = arena.get_car_state(car).phys.vel;
    // Controls neutral (no jump)
    arena.set_car_controls(car, CarControls::default());

    arena.step_tick();
    let post_state = *arena.get_car_state(car);
    let d_vel = post_state.phys.vel - initial_vel;
    println!(
        "Manual is_flipping step: has_flipped={} is_flipping={} flip_time={:.3} d_vel={:?} ang_vel={:?}",
        post_state.has_flipped,
        post_state.is_flipping,
        post_state.flip_time,
        d_vel.to_array(),
        post_state.phys.ang_vel.to_array(),
    );
}
