//! Sanity probe of RocketSim's car-ball hit on a clean head-on approach (no replay data).
//!
//! An Octane driving straight at a stationary ball on the ground at several speeds. For a centered
//! hit the ball leaves near the car speed plus the extra hit impulse (about 0.6 x closing speed), so
//! roughly 1.6x the car speed at moderate speeds. Prints the ball speed after the hit for direct
//! (car set on the ground with throttle) approaches.

use std::path::Path;

use glam::{Mat3A, Vec3A};
use replay_to_rocketsim::conversion::step_arena_tick;
use rocketsim::{
    Arena, ArenaConfig, ArenaEvent, BallState, CarBodyConfig, CarControls, CarState, GameMode, Team,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    rocketsim::init(Path::new("collision_meshes"), true)?;
    for speed in [500.0f32, 1000.0, 1400.0, 2000.0] {
        let scale: f32 = std::env::var("EXTRA_SCALE")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(1.0);
        let mut config = ArenaConfig::new(GameMode::Soccar);
        config.mutators.ball_hit_extra_force_scale = scale;
        let mut arena = Arena::new_with_config(config);
        arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let mut car = CarState::default();
        car.phys.pos = Vec3A::new(0.0, -600.0, 17.0);
        car.phys.vel = Vec3A::new(0.0, speed, 0.0);
        // facing +y: forward axis (x_axis) = +y, right (y_axis) = +x... (RocketSim rot_mat columns: forward, right, up)
        car.phys.rot_mat = Mat3A::from_cols(Vec3A::Y, -Vec3A::X, Vec3A::Z);
        car.is_on_ground = true;
        car.wheels_with_contact = [Some(rocketsim::RaycastHitInfo::default()); 4];
        arena.set_car_state(0, car);
        let mut ball = BallState::default();
        ball.phys.pos = Vec3A::new(0.0, 0.0, 93.15);
        arena.set_ball_state(ball);
        arena.set_car_controls(
            0,
            CarControls {
                throttle: 1.0,
                ..CarControls::default()
            },
        );
        let mut hit_tick = None;
        let mut speed_after = 0.0f32;
        for tick in 1..=120 {
            for event in step_arena_tick(&mut arena) {
                if let ArenaEvent::CarHitBall(hit) = &event {
                    if hit_tick.is_none() {
                        hit_tick = Some(tick);
                        println!(
                            "   HIT event at tick {tick}: reported extra_hit_vel {:?} (|v| {:.1})",
                            hit.extra_hit_vel.to_array().map(|v| v.round()),
                            hit.extra_hit_vel.length()
                        );
                    }
                }
            }
            if hit_tick.is_none() && (tick == 1 || tick % 10 == 0) {
                let c = arena.get_car_state(0);
                println!(
                    "   tick {tick:3}: car speed {:7.1} y {:7.1} on_ground {} | ball speed {:.1}",
                    c.phys.vel.length(),
                    c.phys.pos.y,
                    c.is_on_ground,
                    arena.get_ball_state().phys.vel.length()
                );
            }
            if hit_tick.is_some_and(|h| tick <= h + 8 && tick >= h) {
                let c = arena.get_car_state(0);
                println!(
                    "   hit+{}: car speed {:7.1} vel {:?} | ball vel {:?}",
                    tick - hit_tick.unwrap(),
                    c.phys.vel.length(),
                    c.phys.vel.to_array().map(|v| v.round()),
                    arena
                        .get_ball_state()
                        .phys
                        .vel
                        .to_array()
                        .map(|v| v.round())
                );
            }
            if hit_tick.is_some_and(|h| tick == h + 6) {
                speed_after = arena.get_ball_state().phys.vel.length();
            }
        }
        let ball_speed = speed_after;
        println!(
            "car {speed:6.0} UU/s: hit at tick {hit_tick:?}, ball speed after {ball_speed:7.1} UU/s ({:.2}x)",
            ball_speed / speed
        );
    }
    Ok(())
}
