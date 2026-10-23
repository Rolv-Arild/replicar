//! A RocketSim state restored from a file row (`replicar_format::read_states`). The rotation comes back from
//! a unit quaternion (within 1e-6 per matrix element of the simulated one, RESULTS.md, "v2: the file format"),
//! the wheels only as touching or not (a touching wheel gets placeholder ray-hit fields), and nothing of
//! RocketSim's hidden wheel and contact-solver state: a restored state is a snapshot to inspect or to start a new
//! simulation from, not an exact continuation. For exact states, resimulate (`Converter::resimulate`). Put a car
//! into an arena with `Arena::set_car_state_full` (`set_car_state` clears the contacts); in an arena that has run
//! before, `Arena::reset_car_transient_contacts` first drops the wheel state its own past left behind.

use glam::{Mat3A, Quat, Vec3A};
use replicar_format::header::Header;
use replicar_format::read_states;
use replicar_format::record::{Body, Car, Controls};
use rocketsim::{
    ArenaState, BallState, BoostPadConfig, BoostPadState, CarControls, CarInfo, CarState, GameMode,
    PhysState,
};

use crate::Error;
use crate::hitbox::Hitbox;

fn invalid(why: String) -> Error {
    Error::Resimulation(format!("cannot restore the state: {why}"))
}

fn phys(body: &Body) -> PhysState {
    PhysState {
        pos: Vec3A::from(body.position),
        rot_mat: Mat3A::from_quat(Quat::from_array(body.rotation).normalize()),
        vel: Vec3A::from(body.velocity),
        ang_vel: Vec3A::from(body.angular_velocity),
    }
}

fn controls(c: &Controls) -> CarControls {
    CarControls {
        throttle: c.throttle,
        steer: c.steer,
        pitch: c.pitch,
        yaw: c.yaw,
        roll: c.roll,
        jump: c.jump,
        boost: c.boost,
        handbrake: c.handbrake,
    }
}

/// A car's state from its row. RocketSim keeps the contact cache private: the ground flag and the touching
/// wheels go in through `set_contact_flags` (a touching wheel gets placeholder ray-hit fields), the previous
/// controls through `set_prev_controls`, and the world-contact normal, which has no setter, through the state's
/// serde form (ROCKETSIM_NOTES.md, "The car's contact cache has no full setter").
fn car_state(car: &Car) -> Result<CarState, Error> {
    let i = &car.internals;
    // Fields the file does not store (`jump_hold_broken`, `bump_cooldown_victim`, and any a later RocketSim
    // release adds) keep their defaults. (A struct literal cannot fill the private cache.)
    let mut state = CarState::DEFAULT;
    state.phys = phys(&car.body);
    state.controls = controls(&car.controls);
    state.has_jumped = i.has_jumped;
    state.has_double_jumped = i.has_double_jumped;
    state.has_flipped = i.has_flipped;
    state.flip_rel_torque = Vec3A::from(i.flip_relative_torque);
    state.jump_ticks = i.jump_ticks;
    state.flip_time = i.flip_time;
    state.is_flipping = i.is_flipping;
    state.is_jumping = i.is_jumping;
    state.air_time = i.air_time;
    state.air_time_since_jump = i.air_time_since_jump;
    state.boost = car.boost;
    state.time_since_boosted = i.time_since_boosted;
    state.is_boosting = i.is_boosting;
    state.boosting_time = i.boosting_time;
    state.is_supersonic = i.is_supersonic;
    state.supersonic_grace_timer = i.supersonic_grace_timer;
    state.handbrake_val = i.handbrake_value;
    state.is_auto_flipping = i.is_auto_flipping;
    state.auto_flip_timer = i.auto_flip_timer;
    state.auto_flip_torque_scale = i.auto_flip_torque_scale;
    state.bump_cooldown_timer = i.bump_cooldown_timer;
    state.last_extra_hit_tick = i.last_extra_hit_tick;
    state.is_demoed = i.is_demoed;
    state.demo_respawn_timer = i.demo_respawn_timer;
    state.set_contact_flags(i.is_on_ground, i.wheels_with_contact);
    state.set_prev_controls(controls(&car.previous_controls));
    match i.world_contact_normal {
        None => Ok(state),
        Some(normal) => with_world_contact_normal(state, Vec3A::from(normal)),
    }
}

/// `state` with the world-contact normal set, through serde (the only way in while the contact cache is private).
fn with_world_contact_normal(state: CarState, normal: Vec3A) -> Result<CarState, Error> {
    let fail = |e: serde_json::Error| invalid(format!("world contact normal: {e}"));
    let mut value = serde_json::to_value(state).map_err(fail)?;
    let cache = value
        .get_mut("cache")
        .and_then(|c| c.get_mut("world_contact_normal"))
        .ok_or_else(|| {
            invalid("RocketSim's car state has no cache.world_contact_normal".to_owned())
        })?;
    *cache = serde_json::to_value(Some(normal)).map_err(fail)?;
    serde_json::from_value(value).map_err(fail)
}

/// The state of one row: its sim tick, the ball, every player with a car (absent players are left out), and
/// the pads of the header's layout.
pub fn arena_state(header: &Header, row: &replicar_format::StateRow) -> Result<ArenaState, Error> {
    let mut state = ArenaState::new_empty(GameMode::Soccar);
    state.tick_count = row.sim_tick;
    let mut ball = BallState::default();
    ball.phys = phys(&row.state.ball.body);
    ball.tick_count_since_kickoff = row.state.ball.ticks_since_kickoff;
    state.ball = ball;
    for (player, car) in header.players.iter().zip(&row.state.cars) {
        let Some(car) = car else {
            continue;
        };
        let hitbox = Hitbox::from_name(&player.hitbox)
            .ok_or_else(|| invalid(format!("unknown hitbox {}", player.hitbox)))?;
        let team = match player.team {
            0 => rocketsim::Team::Blue,
            1 => rocketsim::Team::Orange,
            other => return Err(invalid(format!("team {other}"))),
        };
        state.cars.push((
            CarInfo {
                idx: usize::from(player.index),
                team,
                config: hitbox.config(),
            },
            car_state(car)?,
        ));
    }
    for (pad, cooldown) in header.pads.iter().zip(&row.state.pad_cooldowns) {
        state.boost_pads.push((
            BoostPadConfig {
                pos: Vec3A::from(pad.position),
                is_big: pad.is_big,
            },
            BoostPadState {
                cooldown: *cooldown,
            },
        ));
    }
    Ok(state)
}

/// Every row's restored state from a file with the `state` group.
pub fn arena_states(
    path: &std::path::Path,
) -> Result<Vec<(replicar_format::FrameIndex, ArenaState)>, Error> {
    let (header, rows) = read_states(path)?;
    rows.iter()
        .map(|row| Ok((row.frame, arena_state(&header, row)?)))
        .collect()
}
