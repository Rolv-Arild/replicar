//! Reading the `state` group back into rows (`record::State`), for restoring RocketSim states from a file. A
//! quantized body is decoded with its field's scale.

use std::path::Path;

use arrow_array::{
    Array, BooleanArray, Float32Array, Int16Array, Int32Array, RecordBatch, UInt32Array,
    UInt64Array,
};
use arrow_schema::DataType;

use crate::header::Header;
use crate::read::{ReadError, read};
use crate::record::{Ball, Body, Car, CarInternals, Controls, State};
use crate::{CarStatus, FrameIndex};

/// One row's state: the frame, the sim tick and the state.
#[derive(Debug, Clone, PartialEq)]
pub struct StateRow {
    pub frame: FrameIndex,
    pub sim_tick: u64,
    pub state: State,
}

fn missing(name: &str) -> ReadError {
    ReadError::MissingColumn(name.to_owned())
}

/// A float column, decoding integers with the field's `scale`.
fn floats(batch: &RecordBatch, name: &str) -> Result<Vec<Option<f32>>, ReadError> {
    let schema = batch.schema();
    let field = schema.field_with_name(name).map_err(|_| missing(name))?;
    let column = batch.column_by_name(name).ok_or_else(|| missing(name))?;
    let scale = field
        .metadata()
        .get("scale")
        .and_then(|s| s.parse::<f64>().ok());
    let values = match (field.data_type(), scale) {
        (DataType::Float32, _) => {
            let a = column
                .as_any()
                .downcast_ref::<Float32Array>()
                .ok_or_else(|| missing(name))?;
            a.iter().collect()
        }
        (DataType::Int32, Some(scale)) => {
            let a = column
                .as_any()
                .downcast_ref::<Int32Array>()
                .ok_or_else(|| missing(name))?;
            a.iter()
                .map(|v| v.map(|v| (f64::from(v) * scale) as f32))
                .collect()
        }
        (DataType::Int16, Some(scale)) => {
            let a = column
                .as_any()
                .downcast_ref::<Int16Array>()
                .ok_or_else(|| missing(name))?;
            a.iter()
                .map(|v| v.map(|v| (f64::from(v) * scale) as f32))
                .collect()
        }
        _ => return Err(missing(name)),
    };
    Ok(values)
}

fn typed<'a, A: Array + 'static>(batch: &'a RecordBatch, name: &str) -> Result<&'a A, ReadError> {
    batch
        .column_by_name(name)
        .and_then(|c| c.as_any().downcast_ref::<A>())
        .ok_or_else(|| missing(name))
}

fn bools(batch: &RecordBatch, name: &str) -> Result<Vec<Option<bool>>, ReadError> {
    Ok(typed::<BooleanArray>(batch, name)?.iter().collect())
}

/// The columns of one body, by row.
fn bodies(batch: &RecordBatch, prefix: &str) -> Result<Vec<Option<Body>>, ReadError> {
    let get = |quantity: &str, components: &[&str]| -> Result<Vec<Vec<Option<f32>>>, ReadError> {
        components
            .iter()
            .map(|c| floats(batch, &format!("{prefix}_{quantity}_{c}")))
            .collect()
    };
    let position = get("position", &["x", "y", "z"])?;
    let velocity = get("velocity", &["x", "y", "z"])?;
    let angular = get("angular_velocity", &["x", "y", "z"])?;
    let rotation = get("rotation", &["x", "y", "z", "w"])?;
    Ok((0..batch.num_rows())
        .map(|r| {
            let three = |v: &Vec<Vec<Option<f32>>>| -> Option<[f32; 3]> {
                Some([v[0][r]?, v[1][r]?, v[2][r]?])
            };
            Some(Body {
                position: three(&position)?,
                velocity: three(&velocity)?,
                angular_velocity: three(&angular)?,
                rotation: [
                    rotation[0][r]?,
                    rotation[1][r]?,
                    rotation[2][r]?,
                    rotation[3][r]?,
                ],
            })
        })
        .collect())
}

fn controls(batch: &RecordBatch, prefix: &str) -> Result<Vec<Option<Controls>>, ReadError> {
    let axes = ["throttle", "steer", "pitch", "yaw", "roll"]
        .iter()
        .map(|n| floats(batch, &format!("{prefix}_{n}")))
        .collect::<Result<Vec<_>, _>>()?;
    let buttons = ["jump", "boost", "handbrake"]
        .iter()
        .map(|n| bools(batch, &format!("{prefix}_{n}")))
        .collect::<Result<Vec<_>, _>>()?;
    Ok((0..batch.num_rows())
        .map(|r| {
            Some(Controls {
                throttle: axes[0][r]?,
                steer: axes[1][r]?,
                pitch: axes[2][r]?,
                yaw: axes[3][r]?,
                roll: axes[4][r]?,
                jump: buttons[0][r]?,
                boost: buttons[1][r]?,
                handbrake: buttons[2][r]?,
            })
        })
        .collect())
}

/// Player `p`'s cars, by row (`None` where the player has no car).
fn cars(batch: &RecordBatch, p: usize) -> Result<Vec<Option<Car>>, ReadError> {
    let prefix = format!("car_{p}");
    let name = |field: &str| format!("{prefix}_{field}");
    let body = bodies(batch, &prefix)?;
    let boost = floats(batch, &name("boost"))?;
    let control = controls(batch, &name("controls"))?;
    let previous = controls(batch, &name("previous_controls"))?;
    let flag = |f: &str| bools(batch, &name(f));
    let timer = |f: &str| floats(batch, &name(f));
    let flags = [
        "is_on_ground",
        "has_jumped",
        "has_double_jumped",
        "has_flipped",
        "is_flipping",
        "is_jumping",
        "is_boosting",
        "is_supersonic",
        "is_auto_flipping",
        "is_demoed",
    ]
    .map(flag);
    let flags = flags.into_iter().collect::<Result<Vec<_>, _>>()?;
    let wheels = (0..4)
        .map(|w| flag(&format!("wheel_{w}_contact")))
        .collect::<Result<Vec<_>, _>>()?;
    let timers = [
        "flip_time",
        "air_time",
        "air_time_since_jump",
        "time_since_boosted",
        "boosting_time",
        "supersonic_grace_timer",
        "handbrake_value",
        "auto_flip_timer",
        "auto_flip_torque_scale",
        "bump_cooldown_timer",
        "demo_respawn_timer",
    ]
    .map(timer);
    let timers = timers.into_iter().collect::<Result<Vec<_>, _>>()?;
    let torque = ["x", "y", "z"]
        .iter()
        .map(|a| timer(&format!("flip_relative_torque_{a}")))
        .collect::<Result<Vec<_>, _>>()?;
    let normal = ["x", "y", "z"]
        .iter()
        .map(|a| timer(&format!("world_contact_normal_{a}")))
        .collect::<Result<Vec<_>, _>>()?;
    let jump_ticks = typed::<UInt32Array>(batch, &name("jump_ticks"))?;
    let last_hit = typed::<UInt64Array>(batch, &name("last_extra_hit_tick"))?;
    Ok((0..batch.num_rows())
        .map(|r| {
            let f = |i: usize| flags[i][r];
            let t = |i: usize| timers[i][r];
            Some(Car {
                body: body[r]?,
                boost: boost[r]?,
                controls: control[r]?,
                previous_controls: previous[r]?,
                internals: CarInternals {
                    is_on_ground: f(0)?,
                    wheels_with_contact: [
                        wheels[0][r]?,
                        wheels[1][r]?,
                        wheels[2][r]?,
                        wheels[3][r]?,
                    ],
                    has_jumped: f(1)?,
                    has_double_jumped: f(2)?,
                    has_flipped: f(3)?,
                    flip_relative_torque: [torque[0][r]?, torque[1][r]?, torque[2][r]?],
                    jump_ticks: jump_ticks.is_valid(r).then(|| jump_ticks.value(r))?,
                    flip_time: t(0)?,
                    is_flipping: f(4)?,
                    is_jumping: f(5)?,
                    air_time: t(1)?,
                    air_time_since_jump: t(2)?,
                    time_since_boosted: t(3)?,
                    is_boosting: f(6)?,
                    boosting_time: t(4)?,
                    is_supersonic: f(7)?,
                    supersonic_grace_timer: t(5)?,
                    handbrake_value: t(6)?,
                    is_auto_flipping: f(8)?,
                    auto_flip_timer: t(7)?,
                    auto_flip_torque_scale: t(8)?,
                    bump_cooldown_timer: t(9)?,
                    last_extra_hit_tick: last_hit.is_valid(r).then(|| last_hit.value(r)),
                    world_contact_normal: (|| Some([normal[0][r]?, normal[1][r]?, normal[2][r]?]))(
                    ),
                    is_demoed: f(9)?,
                    demo_respawn_timer: t(10)?,
                },
            })
        })
        .collect())
}

/// Every row's state from a file with the `state` group.
pub fn read_states(path: &Path) -> Result<(Header, Vec<StateRow>), ReadError> {
    let (header, batch) = read(path, None)?;
    let rows = batch.num_rows();
    let frame = typed::<UInt32Array>(&batch, "frame")?;
    let sim_tick = typed::<UInt64Array>(&batch, "sim_tick")?;
    let ball = bodies(&batch, "ball")?;
    let since_kickoff = typed::<UInt64Array>(&batch, "ball_ticks_since_kickoff")?;
    let players = header.players.len();
    let mut cars_by_player = Vec::with_capacity(players);
    let mut status_by_player = Vec::with_capacity(players);
    let mut inferred_by_player = Vec::with_capacity(players);
    for p in 0..players {
        cars_by_player.push(cars(&batch, p)?);
        let status = batch
            .column_by_name(&format!("car_{p}_status"))
            .ok_or_else(|| missing("car status"))?;
        let status = arrow_cast_names(status.as_ref())?;
        status_by_player.push(status);
        inferred_by_player.push(bools(&batch, &format!("car_{p}_status_inferred"))?);
    }
    let pads = (0..header.pads.len())
        .map(|k| floats(&batch, &format!("pad_{k}_cooldown")))
        .collect::<Result<Vec<_>, _>>()?;
    let mut out = Vec::with_capacity(rows);
    for r in 0..rows {
        let ball = ball[r].ok_or_else(|| missing("ball"))?;
        out.push(StateRow {
            frame: FrameIndex(frame.value(r)),
            sim_tick: sim_tick.value(r),
            state: State {
                ball: Ball {
                    body: ball,
                    ticks_since_kickoff: since_kickoff.value(r),
                },
                car_status: status_by_player
                    .iter()
                    .map(|s| s[r].unwrap_or(CarStatus::Absent))
                    .collect(),
                car_status_inferred: inferred_by_player
                    .iter()
                    .map(|s| s[r].unwrap_or(false))
                    .collect(),
                cars: cars_by_player.iter().map(|c| c[r]).collect(),
                pad_cooldowns: pads.iter().map(|p| p[r].unwrap_or(0.0)).collect(),
            },
        });
    }
    Ok((header, out))
}

/// The values of a name column.
fn arrow_cast_names(column: &dyn Array) -> Result<Vec<Option<CarStatus>>, ReadError> {
    use arrow_array::types::UInt8Type;
    let dictionary = column
        .as_any()
        .downcast_ref::<arrow_array::DictionaryArray<UInt8Type>>()
        .ok_or_else(|| missing("car status"))?;
    let values = dictionary
        .values()
        .as_any()
        .downcast_ref::<arrow_array::StringArray>()
        .ok_or_else(|| missing("car status"))?;
    Ok(dictionary
        .keys()
        .iter()
        .map(|k| k.and_then(|k| CarStatus::from_name(values.value(usize::from(k)))))
        .collect())
}
