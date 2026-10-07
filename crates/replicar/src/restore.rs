//! A RocketSim state restored from a file row (`replicar_format::read_states`). The rotation comes back from
//! a unit quaternion (within 1e-6 per matrix element of the simulated one, RESULTS.md, "v2: the file format"),
//! the wheels only as touching or not (a touching wheel gets a default ray hit), and nothing of RocketSim's
//! private caches: a restored state is a snapshot to inspect or to start a new simulation from, not an exact
//! continuation. For exact states, resimulate (`Converter::resimulate`).

use glam::{Mat3A, Quat, Vec3A};
use replicar_format::header::Header;
use replicar_format::read_states;
use replicar_format::record::{Body, Controls};
use rocketsim::{
    ArenaState, BallState, BoostPadConfig, BoostPadState, CarControls, CarInfo, CarState, GameMode,
    PhysState, RaycastHitInfo,
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
        let i = &car.internals;
        state.cars.push((
            CarInfo {
                idx: usize::from(player.index),
                team,
                config: hitbox.config(),
            },
            CarState {
                phys: phys(&car.body),
                controls: controls(&car.controls),
                prev_controls: controls(&car.previous_controls),
                is_on_ground: i.is_on_ground,
                wheels_with_contact: i
                    .wheels_with_contact
                    .map(|touching| touching.then(RaycastHitInfo::default)),
                has_jumped: i.has_jumped,
                has_double_jumped: i.has_double_jumped,
                has_flipped: i.has_flipped,
                flip_rel_torque: Vec3A::from(i.flip_relative_torque),
                jump_ticks: i.jump_ticks,
                flip_time: i.flip_time,
                is_flipping: i.is_flipping,
                is_jumping: i.is_jumping,
                air_time: i.air_time,
                air_time_since_jump: i.air_time_since_jump,
                boost: car.boost,
                time_since_boosted: i.time_since_boosted,
                is_boosting: i.is_boosting,
                boosting_time: i.boosting_time,
                is_supersonic: i.is_supersonic,
                supersonic_grace_timer: i.supersonic_grace_timer,
                handbrake_val: i.handbrake_value,
                is_auto_flipping: i.is_auto_flipping,
                auto_flip_timer: i.auto_flip_timer,
                auto_flip_torque_scale: i.auto_flip_torque_scale,
                bump_cooldown_timer: i.bump_cooldown_timer,
                last_extra_hit_tick: i.last_extra_hit_tick,
                world_contact_normal: i.world_contact_normal.map(Vec3A::from),
                is_demoed: i.is_demoed,
                demo_respawn_timer: i.demo_respawn_timer,
            },
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
