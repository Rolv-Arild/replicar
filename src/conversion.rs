//! Replay frame snapshots produced by short RocketSim steps and fresh corrections.

use std::collections::HashMap;
use std::error::Error;
use std::fmt;
use std::io;
use std::path::{Path, PathBuf};

use glam::Quat;
use rocketsim::{
    Arena, ArenaConfig, ArenaEvent, ArenaState, CarBodyConfig, CarControls, CarState, GameMode,
    Mat3A, PhysState, Team, Vec3A,
};
use serde::Serialize;
use sha2::{Digest, Sha256};

use crate::observations::{self, Body, ObservedReplay, Value};
use crate::parse_replay;

#[derive(Debug)]
pub enum ConvertError {
    Parse(boxcars::ParseError),
    MissingNetworkFrames,
    UnsupportedMode(String),
    Init(io::Error),
    InvalidTime { frame: usize, time: f32 },
}

impl fmt::Display for ConvertError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Parse(error) => write!(f, "replay parse failed: {error}"),
            Self::MissingNetworkFrames => write!(f, "replay has no network frames"),
            Self::UnsupportedMode(mode) => write!(f, "unsupported replay mode: {mode}"),
            Self::Init(error) => write!(f, "RocketSim initialization failed: {error}"),
            Self::InvalidTime { frame, time } => {
                write!(f, "invalid replay time {time} at frame {frame}")
            }
        }
    }
}

impl Error for ConvertError {}

#[derive(Debug, Clone, Serialize)]
pub struct ConvertOptions {
    pub collision_meshes: PathBuf,
    pub seed: u64,
    /// Interpret odd boost-component ReplicatedActive bytes as active boost input.
    pub infer_boost_from_active: bool,
    /// Gaps larger than this are left unsimulated and recorded in diagnostics.
    pub max_gap_ticks: u64,
}

impl Default for ConvertOptions {
    fn default() -> Self {
        Self {
            collision_meshes: PathBuf::from("collision_meshes"),
            seed: 0,
            infer_boost_from_active: true,
            max_gap_ticks: 1200,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SimEvent {
    pub arena_tick: u64,
    pub event: ArenaEvent,
}

#[derive(Debug, Clone)]
pub struct ConvertedFrame {
    pub replay_frame: usize,
    pub replay_time: f32,
    /// 120 Hz tick on the elapsed replay timeline (including frozen phases).
    pub timeline_tick: u64,
    pub state: ArenaState,
    pub simulated_events: Vec<SimEvent>,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct Diagnostics {
    pub skipped_timeline_ticks: u64,
    pub unlinked_car_frames: usize,
    pub default_hitbox_players: usize,
}

#[derive(Debug, Clone)]
pub struct ConversionOutput {
    pub source_sha256: Option<String>,
    pub options: ConvertOptions,
    pub observations: ObservedReplay,
    pub frames: Vec<ConvertedFrame>,
    pub position_residuals: Vec<PositionResidual>,
    pub car_slots: Vec<CarSlot>,
    pub diagnostics: Diagnostics,
}

#[derive(Debug, Clone, Serialize)]
pub struct CarSlot {
    pub slot: usize,
    pub player_key: String,
    pub team: u8,
    pub hitbox: String,
}

/// Error before a fresh replay position is used to correct the simulation.
#[derive(Debug, Clone, Serialize)]
pub struct PositionResidual {
    pub frame: usize,
    pub actor_id: Option<i32>,
    pub seconds_since_previous_position: f32,
    pub simulated_error_uu: f32,
    pub hold_error_uu: f32,
    pub linear_extrapolation_error_uu: Option<f32>,
}

fn vec3(value: [f32; 3]) -> Vec3A {
    Vec3A::new(value[0], value[1], value[2])
}

fn position_residual(
    index: usize,
    actor_id: Option<i32>,
    body: &Body,
    previous: Option<&Body>,
    predicted: Vec3A,
    frames: &[observations::Frame],
) -> Option<PositionResidual> {
    let actual = body
        .position
        .as_ref()
        .filter(|value| value.frame == index)?;
    let previous = previous?.position.as_ref()?;
    if previous.frame >= index {
        return None;
    }
    let dt = frames[index].time - frames[previous.frame].time;
    if !dt.is_finite() || dt <= 0.0 {
        return None;
    }
    let actual = vec3(actual.value);
    let previous_pos = vec3(previous.value);
    let previous_body = if let Some(id) = actor_id {
        frames[index - 1]
            .cars
            .iter()
            .find(|car| car.actor_id == id)
            .map(|car| &car.body)
    } else {
        frames[index - 1].ball.as_ref()
    };
    let linear_extrapolation_error_uu = previous_body
        .and_then(|body| body.linear_velocity.as_ref())
        .map(|velocity| (previous_pos + vec3(velocity.value) * dt - actual).length());
    Some(PositionResidual {
        frame: index,
        actor_id,
        seconds_since_previous_position: dt,
        simulated_error_uu: (predicted - actual).length(),
        hold_error_uu: (previous_pos - actual).length(),
        linear_extrapolation_error_uu,
    })
}

fn should_apply<T>(value: &Value<T>, index: usize, new_entity: bool) -> bool {
    new_entity || value.frame == index
}

fn apply_body(state: &mut PhysState, body: &Body, index: usize, new_entity: bool) -> bool {
    let mut applied = false;
    if let Some(value) = &body.position {
        if should_apply(value, index, new_entity) {
            state.pos = vec3(value.value);
            applied = true;
        }
    }
    if let Some(value) = &body.rotation_xyzw {
        if should_apply(value, index, new_entity) {
            let [x, y, z, w] = value.value;
            let quat = Quat::from_xyzw(x, y, z, w);
            if quat.is_finite() && quat.length_squared() > 0.5 {
                state.rot_mat = Mat3A::from_quat(quat.normalize());
                applied = true;
            }
        }
    }
    if let Some(value) = &body.linear_velocity {
        if should_apply(value, index, new_entity) {
            state.vel = vec3(value.value);
            applied = true;
        }
    }
    if let Some(value) = &body.angular_velocity_replay_units {
        if should_apply(value, index, new_entity) {
            state.ang_vel = vec3(value.value) * 0.01;
            applied = true;
        }
    }
    applied
}

fn controls_from_observation(car: &observations::Car, options: &ConvertOptions) -> CarControls {
    CarControls {
        throttle: car.inputs.throttle.as_ref().map_or(0.0, |v| v.value),
        steer: car.inputs.steer.as_ref().map_or(0.0, |v| v.value),
        handbrake: car.inputs.handbrake.as_ref().is_some_and(|v| v.value),
        boost: options.infer_boost_from_active
            && car
                .inputs
                .boost_active_raw
                .as_ref()
                .is_some_and(|v| v.value % 2 == 1),
        ..CarControls::default()
    }
}

fn team(index: u8) -> Team {
    if index == 0 { Team::Blue } else { Team::Orange }
}

/// Parse and convert a soccar replay. The returned frame and observation vectors align by index.
pub fn convert_bytes(
    bytes: &[u8],
    options: &ConvertOptions,
) -> Result<ConversionOutput, ConvertError> {
    let replay = parse_replay(bytes).map_err(ConvertError::Parse)?;
    let observed = observations::extract(&replay).ok_or(ConvertError::MissingNetworkFrames)?;
    let mut output = convert_observations(observed, options)?;
    output.source_sha256 = Some(format!("{:x}", Sha256::digest(bytes)));
    Ok(output)
}

/// Convert observations with RocketSim. The mesh directory is initialized once per process.
pub fn convert_observations(
    observations: ObservedReplay,
    options: &ConvertOptions,
) -> Result<ConversionOutput, ConvertError> {
    if observations.header.game_type != "TAGame.Replay_Soccar_TA" {
        return Err(ConvertError::UnsupportedMode(
            observations.header.game_type.clone(),
        ));
    }
    rocketsim::init(Path::new(&options.collision_meshes), true).map_err(ConvertError::Init)?;
    let mut config = ArenaConfig::new(GameMode::Soccar);
    config.rng_seed = Some(options.seed);
    let mut arena = Arena::new_with_config(config);
    let mut slots: HashMap<String, usize> = HashMap::new();
    let mut car_slots = Vec::new();
    let mut actor_slots: HashMap<i32, (usize, usize)> = HashMap::new();
    let mut frames = Vec::with_capacity(observations.frames.len());
    let mut position_residuals = Vec::new();
    let mut diagnostics = Diagnostics::default();
    let first_time = observations.frames.first().map_or(0.0, |frame| frame.time);
    let mut previous_tick = 0;
    let mut previous_active = false;
    let mut ball_initialized = false;

    for frame in &observations.frames {
        if !frame.time.is_finite() || frame.time < first_time {
            return Err(ConvertError::InvalidTime {
                frame: frame.index,
                time: frame.time,
            });
        }
        let timeline_tick = (((f64::from(frame.time) - f64::from(first_time)) * 120.0).round()
            as u64)
            .max(previous_tick);
        let gap = timeline_tick - previous_tick;
        let active = frame
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active");
        let mut events = Vec::new();
        let simulated = active && previous_active && gap > 0 && gap <= options.max_gap_ticks;
        if simulated {
            for _ in 0..gap {
                let arena_tick = arena.tick_count() + 1;
                let tick_events = arena.step_tick();
                events.extend(
                    tick_events
                        .iter()
                        .copied()
                        .map(|event| SimEvent { arena_tick, event }),
                );
            }
        } else {
            diagnostics.skipped_timeline_ticks += gap;
        }
        previous_tick = timeline_tick;
        previous_active = active;

        if let Some(body) = &frame.ball {
            if simulated && ball_initialized {
                if let Some(residual) = position_residual(
                    frame.index,
                    None,
                    body,
                    observations
                        .frames
                        .get(frame.index.wrapping_sub(1))
                        .and_then(|f| f.ball.as_ref()),
                    arena.get_ball_state().phys.pos,
                    &observations.frames,
                ) {
                    position_residuals.push(residual);
                }
            }
            let mut ball = *arena.get_ball_state();
            if apply_body(&mut ball.phys, body, frame.index, !ball_initialized) {
                arena.set_ball_state(ball);
            }
            ball_initialized = true;
        }

        for car in &frame.cars {
            let slot = if let Some(key) = &car.player_key {
                if let Some(slot) = slots.get(key).copied() {
                    actor_slots.insert(car.actor_id, (slot, car.actor_created_frame));
                    Some((slot, car.actor_created_frame == frame.index))
                } else if let Some(team_idx) = car.team {
                    let slot = arena.add_car(team(team_idx), CarBodyConfig::OCTANE);
                    slots.insert(key.clone(), slot);
                    car_slots.push(CarSlot {
                        slot,
                        player_key: key.clone(),
                        team: team_idx,
                        hitbox: "octane".to_owned(),
                    });
                    actor_slots.insert(car.actor_id, (slot, car.actor_created_frame));
                    diagnostics.default_hitbox_players += 1;
                    Some((slot, true))
                } else {
                    None
                }
            } else {
                actor_slots
                    .get(&car.actor_id)
                    .copied()
                    .filter(|(_, created_frame)| *created_frame == car.actor_created_frame)
                    .map(|(slot, _)| (slot, false))
            };
            let Some((slot, new_lifetime)) = slot else {
                diagnostics.unlinked_car_frames += 1;
                continue;
            };
            if simulated && !new_lifetime {
                let previous = observations
                    .frames
                    .get(frame.index.wrapping_sub(1))
                    .and_then(|f| {
                        f.cars
                            .iter()
                            .find(|previous| previous.actor_id == car.actor_id)
                    })
                    .map(|car| &car.body);
                if let Some(residual) = position_residual(
                    frame.index,
                    Some(car.actor_id),
                    &car.body,
                    previous,
                    arena.get_car_state(slot).phys.pos,
                    &observations.frames,
                ) {
                    position_residuals.push(residual);
                }
            }
            let mut state = if new_lifetime {
                CarState::default()
            } else {
                *arena.get_car_state(slot)
            };
            let mut dirty =
                apply_body(&mut state.phys, &car.body, frame.index, new_lifetime) || new_lifetime;
            if let Some(boost) = &car.boost {
                if should_apply(boost, frame.index, new_lifetime) {
                    state.boost = boost.value;
                    dirty = true;
                }
            }
            if dirty {
                arena.set_car_state(slot, state);
            }
            arena.set_car_controls(slot, controls_from_observation(car, options));
        }
        frames.push(ConvertedFrame {
            replay_frame: frame.index,
            replay_time: frame.time,
            timeline_tick,
            state: arena.get_arena_state(),
            simulated_events: events,
        });
    }
    Ok(ConversionOutput {
        source_sha256: None,
        options: options.clone(),
        observations,
        frames,
        position_residuals,
        car_slots,
        diagnostics,
    })
}

#[cfg(test)]
mod tests {
    use rocketsim::BallState;

    use super::*;
    use crate::observations::Source;

    #[test]
    fn stale_replay_fields_do_not_overwrite_simulated_physics() {
        let mut state = BallState::default().phys;
        state.pos = Vec3A::new(10.0, 20.0, 30.0);
        let body = Body {
            position: Some(Value {
                value: [1.0, 2.0, 3.0],
                frame: 3,
                source: Source::Replay,
            }),
            angular_velocity_replay_units: Some(Value {
                value: [0.0, 0.0, 100.0],
                frame: 3,
                source: Source::Replay,
            }),
            ..Body::default()
        };

        assert!(!apply_body(&mut state, &body, 4, false));
        assert_eq!(state.pos, Vec3A::new(10.0, 20.0, 30.0));
        assert!(apply_body(&mut state, &body, 3, false));
        assert_eq!(state.pos, Vec3A::new(1.0, 2.0, 3.0));
        assert_eq!(state.ang_vel.z, 1.0);
    }
}
