//! Versioned, streaming JSON Lines projection of converted replay states.

use std::io::{self, Write};

use rocketsim::{ArenaEvent, BallState, CarControls, CarState, PhysState, Vec3A};
use serde::{Deserialize, Serialize};

use crate::conversion::{
    ConversionOutput, ConversionSummary, ConvertOptions, ConvertedFrame, PositionResidual, SimEvent,
};
use crate::observations;

pub const SCHEMA_VERSION: u32 = 1;
pub const ROCKETSIM_REVISION: &str = "79f4d22fc533614d540b88457a96352c17da6b73";

fn xyz(v: Vec3A) -> [f32; 3] {
    v.to_array()
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct PhysicsRecord {
    pub position: [f32; 3],
    /// RocketSim basis columns: forward, right, up.
    pub rotation_columns: [[f32; 3]; 3],
    pub linear_velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
}

impl From<&PhysState> for PhysicsRecord {
    fn from(state: &PhysState) -> Self {
        Self {
            position: xyz(state.pos),
            rotation_columns: [
                xyz(state.rot_mat.x_axis),
                xyz(state.rot_mat.y_axis),
                xyz(state.rot_mat.z_axis),
            ],
            linear_velocity: xyz(state.vel),
            angular_velocity: xyz(state.ang_vel),
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ControlsRecord {
    pub throttle: f32,
    pub steer: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub jump: bool,
    pub boost: bool,
    pub handbrake: bool,
}

impl From<&CarControls> for ControlsRecord {
    fn from(c: &CarControls) -> Self {
        Self {
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
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct BallRecord {
    pub physics: PhysicsRecord,
    pub tick_count_since_kickoff: u64,
    pub last_extra_hit_tick: Option<u64>,
    pub heatseeker_target_direction: i8,
    pub heatseeker_target_speed: f32,
    pub heatseeker_time_since_hit: f32,
    pub dropshot_charge_level: u8,
    pub dropshot_accumulated_hit_force: f32,
    pub dropshot_target_direction: i8,
    pub dropshot_last_damage_tick: Option<u64>,
}

impl From<&BallState> for BallRecord {
    fn from(ball: &BallState) -> Self {
        Self {
            physics: (&ball.phys).into(),
            tick_count_since_kickoff: ball.tick_count_since_kickoff,
            last_extra_hit_tick: ball.last_extra_hit_tick,
            heatseeker_target_direction: ball.hs_info.y_target_dir,
            heatseeker_target_speed: ball.hs_info.cur_target_speed,
            heatseeker_time_since_hit: ball.hs_info.time_since_hit,
            dropshot_charge_level: ball.ds_info.charge_level,
            dropshot_accumulated_hit_force: ball.ds_info.accumulated_hit_force,
            dropshot_target_direction: ball.ds_info.y_target_dir,
            dropshot_last_damage_tick: ball.ds_info.last_damage_tick,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct CarRecord {
    pub slot: usize,
    pub team: u8,
    pub physics: PhysicsRecord,
    pub controls: ControlsRecord,
    pub previous_controls: ControlsRecord,
    pub boost: f32,
    pub is_boosting: bool,
    pub boosting_time: f32,
    pub time_since_boosted: f32,
    pub is_on_ground: bool,
    pub wheels_with_contact: [bool; 4],
    pub has_jumped: bool,
    pub has_double_jumped: bool,
    pub has_flipped: bool,
    pub flip_relative_torque: [f32; 3],
    pub jump_ticks: u32,
    pub flip_time: f32,
    pub is_flipping: bool,
    pub is_jumping: bool,
    pub air_time: f32,
    pub air_time_since_jump: f32,
    pub is_supersonic: bool,
    pub supersonic_grace_timer: f32,
    pub handbrake_value: f32,
    pub is_auto_flipping: bool,
    pub auto_flip_timer: f32,
    pub auto_flip_torque_scale: f32,
    pub bump_cooldown_timer: f32,
    pub world_contact_normal: Option<[f32; 3]>,
    pub is_demoed: bool,
    pub demo_respawn_timer: f32,
}

impl CarRecord {
    fn from_state(slot: usize, team: u8, car: &CarState) -> Self {
        Self {
            slot,
            team,
            physics: (&car.phys).into(),
            controls: (&car.controls).into(),
            previous_controls: (&car.prev_controls).into(),
            boost: car.boost,
            is_boosting: car.is_boosting,
            boosting_time: car.boosting_time,
            time_since_boosted: car.time_since_boosted,
            is_on_ground: car.is_on_ground,
            wheels_with_contact: car.wheels_with_contact,
            has_jumped: car.has_jumped,
            has_double_jumped: car.has_double_jumped,
            has_flipped: car.has_flipped,
            flip_relative_torque: xyz(car.flip_rel_torque),
            jump_ticks: car.jump_ticks,
            flip_time: car.flip_time,
            is_flipping: car.is_flipping,
            is_jumping: car.is_jumping,
            air_time: car.air_time,
            air_time_since_jump: car.air_time_since_jump,
            is_supersonic: car.is_supersonic,
            supersonic_grace_timer: car.supersonic_grace_timer,
            handbrake_value: car.handbrake_val,
            is_auto_flipping: car.is_auto_flipping,
            auto_flip_timer: car.auto_flip_timer,
            auto_flip_torque_scale: car.auto_flip_torque_scale,
            bump_cooldown_timer: car.bump_cooldown_timer,
            world_contact_normal: car.world_contact_normal.map(xyz),
            is_demoed: car.is_demoed,
            demo_respawn_timer: car.demo_respawn_timer,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct PadRecord {
    pub position: [f32; 3],
    pub is_big: bool,
    pub cooldown: f32,
    pub is_active: bool,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct StateRecord {
    pub arena_tick: u64,
    pub ball: BallRecord,
    pub cars: Vec<CarRecord>,
    pub boost_pads: Vec<PadRecord>,
}

impl StateRecord {
    /// Capture the soccar RocketSim state represented by a replay frame.
    pub fn from_arena_state(state: &rocketsim::ArenaState) -> Self {
        Self {
            arena_tick: state.tick_count,
            ball: (&state.ball).into(),
            cars: state
                .cars
                .iter()
                .map(|(info, car)| {
                    CarRecord::from_state(info.idx, if info.team.is_blue() { 0 } else { 1 }, car)
                })
                .collect(),
            boost_pads: state
                .boost_pads
                .iter()
                .map(|(config, state)| PadRecord {
                    position: xyz(config.pos),
                    is_big: config.is_big,
                    cooldown: state.cooldown,
                    is_active: state.is_active(),
                })
                .collect(),
        }
    }
}

#[derive(Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SimEventRecord {
    BallHitWorld {
        contact_point: [f32; 3],
        contact_normal: [f32; 3],
    },
    CarHitBall {
        car_slot: usize,
        contact_point: [f32; 3],
        extra_hit_velocity: [f32; 3],
    },
    CarHitCar {
        bumper_slot: usize,
        victim_slot: usize,
        contact_point: [f32; 3],
        is_demo: bool,
    },
    CarHitWorld {
        car_slot: usize,
        contact_point: [f32; 3],
        contact_normal: [f32; 3],
    },
    CarPickupBoost {
        car_slot: usize,
        pad_index: usize,
    },
}

#[derive(Serialize)]
pub struct TimedSimEventRecord {
    pub arena_tick: u64,
    pub event: SimEventRecord,
}

impl From<&SimEvent> for TimedSimEventRecord {
    fn from(value: &SimEvent) -> Self {
        let event = match value.event {
            ArenaEvent::BallHitWorld(data) => SimEventRecord::BallHitWorld {
                contact_point: xyz(data.contact_point),
                contact_normal: xyz(data.contact_normal),
            },
            ArenaEvent::CarHitBall(data) => SimEventRecord::CarHitBall {
                car_slot: data.car_idx,
                contact_point: xyz(data.contact_point),
                extra_hit_velocity: xyz(data.extra_hit_vel),
            },
            ArenaEvent::CarHitCar(data) => SimEventRecord::CarHitCar {
                bumper_slot: data.bumper_car_idx,
                victim_slot: data.victim_car_idx,
                contact_point: xyz(data.contact_point),
                is_demo: data.is_demo,
            },
            ArenaEvent::CarHitWorld(data) => SimEventRecord::CarHitWorld {
                car_slot: data.car_idx,
                contact_point: xyz(data.contact_point),
                contact_normal: xyz(data.contact_normal),
            },
            ArenaEvent::CarPickupBoost(data) => SimEventRecord::CarPickupBoost {
                car_slot: data.car_idx,
                pad_index: data.boost_pad_idx,
            },
        };
        Self {
            arena_tick: value.arena_tick,
            event,
        }
    }
}

#[derive(Serialize)]
struct HeaderLine<'a> {
    record_type: &'static str,
    schema_version: u32,
    source_sha256: &'a Option<String>,
    boxcars_version: &'static str,
    rocketsim_revision: &'static str,
    conversion_options: &'a crate::conversion::ConvertOptions,
    tick_rate_hz: u32,
    units: &'static str,
    header: &'a observations::Header,
    car_slots: &'a [crate::conversion::CarSlot],
    observation_diagnostics: &'a observations::Diagnostics,
    conversion_diagnostics: &'a crate::conversion::Diagnostics,
}

#[derive(Serialize)]
struct FrameLine<'a> {
    record_type: &'static str,
    frame: usize,
    replay_time: f32,
    timeline_tick: u64,
    state: StateRecord,
    observations: &'a observations::Frame,
    simulated_events: Vec<TimedSimEventRecord>,
    position_residuals: &'a [PositionResidual],
    /// Inferred packet lags applied to this frame's fresh packets (empty when disabled).
    #[serde(skip_serializing_if = "<[_]>::is_empty")]
    packet_lag_ticks: &'a [crate::conversion::AppliedPacketLag],
}

pub(crate) fn header_json(
    observations: &observations::ObservedReplay,
    options: &ConvertOptions,
    summary: &ConversionSummary,
    source_sha256: &Option<String>,
) -> serde_json::Result<Vec<u8>> {
    serde_json::to_vec(&HeaderLine {
        record_type: "header",
        schema_version: SCHEMA_VERSION,
        source_sha256,
        boxcars_version: "0.11.5",
        rocketsim_revision: ROCKETSIM_REVISION,
        conversion_options: options,
        tick_rate_hz: 120,
        units: "Rocket League UU, seconds, radians per second for state angular velocity",
        header: &observations.header,
        car_slots: &summary.car_slots,
        observation_diagnostics: &observations.diagnostics,
        conversion_diagnostics: &summary.diagnostics,
    })
}

pub(crate) fn frame_json(
    converted: &ConvertedFrame,
    observed: &observations::Frame,
    residuals: &[PositionResidual],
) -> serde_json::Result<Vec<u8>> {
    serde_json::to_vec(&frame_line(converted, observed, residuals))
}

fn frame_line<'a>(
    converted: &'a ConvertedFrame,
    observed: &'a observations::Frame,
    residuals: &'a [PositionResidual],
) -> FrameLine<'a> {
    FrameLine {
        record_type: "frame",
        frame: converted.replay_frame,
        replay_time: converted.replay_time,
        timeline_tick: converted.timeline_tick,
        state: StateRecord::from_arena_state(&converted.state),
        observations: observed,
        simulated_events: converted.simulated_events.iter().map(Into::into).collect(),
        position_residuals: residuals,
        packet_lag_ticks: &converted.packet_lags,
    }
}

fn write_line<T: Serialize>(writer: &mut impl Write, value: &T) -> io::Result<()> {
    serde_json::to_writer(&mut *writer, value).map_err(io::Error::other)?;
    writer.write_all(b"\n")
}

/// Write a header line followed by one frame line per replay frame.
pub fn write_jsonl(output: &ConversionOutput, mut writer: impl Write) -> io::Result<()> {
    if output.frames.len() != output.observations.frames.len() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "converted and observed frame counts differ",
        ));
    }
    let summary = ConversionSummary {
        car_slots: output.car_slots.clone(),
        diagnostics: output.diagnostics.clone(),
    };
    writer.write_all(
        &header_json(
            &output.observations,
            &output.options,
            &summary,
            &output.source_sha256,
        )
        .map_err(io::Error::other)?,
    )?;
    writer.write_all(b"\n")?;
    let mut residual_index = 0;
    for (converted, observed) in output.frames.iter().zip(&output.observations.frames) {
        let start = residual_index;
        while residual_index < output.position_residuals.len()
            && output.position_residuals[residual_index].frame == converted.replay_frame
        {
            residual_index += 1;
        }
        let line = frame_line(
            converted,
            observed,
            &output.position_residuals[start..residual_index],
        );
        write_line(&mut writer, &line)?;
    }
    Ok(())
}
