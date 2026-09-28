//! Direct, batched Parquet projection of schema-v1 replay frames.

use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fs::File;
use std::io;
use std::sync::Arc;

use arrow_array::{
    ArrayRef, BooleanArray, FixedSizeListArray, Float32Array, Float64Array, LargeBinaryArray,
    RecordBatch, UInt32Array, UInt64Array,
};
use arrow_schema::{DataType, Field, Schema, SchemaRef};
use parquet::arrow::ArrowWriter;
use parquet::basic::{Compression, ZstdLevel};
use parquet::file::properties::WriterProperties;
use rocketsim::Mat3A;
use sha2::{Digest, Sha256};

use crate::conversion::{self, ConvertOptions, ConvertedFrame, PositionResidual};
use crate::observations;
use crate::serialization;

const BATCH_SIZE: usize = 512;

struct Row {
    frame: u32,
    replay_time: f64,
    timeline_tick: u64,
    arena_tick: u64,
    ball_position: Vec<f32>,
    ball_rotation_columns: Vec<f32>,
    ball_velocity: Vec<f32>,
    ball_angular_velocity: Vec<f32>,
    car_position: Vec<f32>,
    car_rotation_columns: Vec<f32>,
    car_velocity: Vec<f32>,
    car_angular_velocity: Vec<f32>,
    car_boost: Vec<f32>,
    car_present: Vec<bool>,
    car_demoed: Vec<bool>,
    control_axes: Vec<f32>,
    control_buttons: Vec<bool>,
    boost_pad_active: Vec<bool>,
    boost_pad_cooldown: Vec<f32>,
    scores: Vec<f32>,
    seconds_remaining: f32,
    frame_json: Vec<u8>,
}

fn rotation(rot: Mat3A) -> Vec<f32> {
    [
        rot.x_axis.to_array(),
        rot.y_axis.to_array(),
        rot.z_axis.to_array(),
    ]
    .into_iter()
    .flatten()
    .collect()
}

fn row(
    converted: &ConvertedFrame,
    observed: &observations::Frame,
    residuals: &[PositionResidual],
    slot_index: &HashMap<usize, usize>,
    cars: usize,
    pad_count: usize,
) -> io::Result<Row> {
    let state = &converted.state;
    if state.boost_pads.len() != pad_count {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "boost pad count changed",
        ));
    }
    let mut result = Row {
        frame: u32::try_from(converted.replay_frame).map_err(io::Error::other)?,
        // Match the decimal value read from the schema-v1 JSONL frame.
        replay_time: serde_json::to_string(&converted.replay_time)
            .map_err(io::Error::other)?
            .parse()
            .map_err(io::Error::other)?,
        timeline_tick: converted.timeline_tick,
        arena_tick: state.tick_count,
        ball_position: state.ball.phys.pos.to_array().to_vec(),
        ball_rotation_columns: rotation(state.ball.phys.rot_mat),
        ball_velocity: state.ball.phys.vel.to_array().to_vec(),
        ball_angular_velocity: state.ball.phys.ang_vel.to_array().to_vec(),
        car_position: vec![f32::NAN; cars * 3],
        car_rotation_columns: vec![f32::NAN; cars * 9],
        car_velocity: vec![f32::NAN; cars * 3],
        car_angular_velocity: vec![f32::NAN; cars * 3],
        car_boost: vec![f32::NAN; cars],
        car_present: vec![false; cars],
        car_demoed: vec![false; cars],
        control_axes: vec![f32::NAN; cars * 5],
        control_buttons: vec![false; cars * 3],
        boost_pad_active: state
            .boost_pads
            .iter()
            .map(|(_, p)| p.is_active())
            .collect(),
        boost_pad_cooldown: state.boost_pads.iter().map(|(_, p)| p.cooldown).collect(),
        scores: observed
            .team_scores
            .iter()
            .map(|s| s.as_ref().map_or(f32::NAN, |v| v.value as f32))
            .collect(),
        seconds_remaining: observed
            .seconds_remaining
            .as_ref()
            .map_or(f32::NAN, |v| v.value as f32),
        frame_json: serialization::frame_json(converted, observed, residuals)
            .map_err(io::Error::other)?,
    };
    let mut seen = HashSet::new();
    for (info, car) in &state.cars {
        let Some(&index) = slot_index.get(&info.idx) else {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "state has unlisted car slot",
            ));
        };
        if !seen.insert(index) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "duplicate car slot",
            ));
        }
        result.car_position[index * 3..index * 3 + 3].copy_from_slice(&car.phys.pos.to_array());
        result.car_rotation_columns[index * 9..index * 9 + 9]
            .copy_from_slice(&rotation(car.phys.rot_mat));
        result.car_velocity[index * 3..index * 3 + 3].copy_from_slice(&car.phys.vel.to_array());
        result.car_angular_velocity[index * 3..index * 3 + 3]
            .copy_from_slice(&car.phys.ang_vel.to_array());
        result.car_boost[index] = car.boost;
        result.car_present[index] = true;
        result.car_demoed[index] = car.is_demoed;
        result.control_axes[index * 5..index * 5 + 5].copy_from_slice(&[
            car.controls.throttle,
            car.controls.steer,
            car.controls.pitch,
            car.controls.yaw,
            car.controls.roll,
        ]);
        result.control_buttons[index * 3..index * 3 + 3].copy_from_slice(&[
            car.controls.jump,
            car.controls.boost,
            car.controls.handbrake,
        ]);
    }
    Ok(result)
}

fn list_field(name: &str, dtype: DataType, width: usize) -> Field {
    Field::new(
        name,
        DataType::FixedSizeList(Arc::new(Field::new("item", dtype, true)), width as i32),
        true,
    )
}

fn schema(
    cars: usize,
    pads: usize,
    header_json: Vec<u8>,
    pad_json: Vec<u8>,
) -> io::Result<SchemaRef> {
    let f = DataType::Float32;
    let b = DataType::Boolean;
    let fields = vec![
        Field::new("frame", DataType::UInt32, true),
        Field::new("replay_time", DataType::Float64, true),
        Field::new("timeline_tick", DataType::UInt64, true),
        Field::new("arena_tick", DataType::UInt64, true),
        list_field("ball_position", f.clone(), 3),
        list_field("ball_rotation_columns", f.clone(), 9),
        list_field("ball_velocity", f.clone(), 3),
        list_field("ball_angular_velocity", f.clone(), 3),
        list_field("car_position", f.clone(), 3 * cars),
        list_field("car_rotation_columns", f.clone(), 9 * cars),
        list_field("car_velocity", f.clone(), 3 * cars),
        list_field("car_angular_velocity", f.clone(), 3 * cars),
        list_field("car_boost", f.clone(), cars),
        list_field("car_present", b.clone(), cars),
        list_field("car_demoed", b.clone(), cars),
        list_field("control_axes", f.clone(), 5 * cars),
        list_field("control_buttons", b.clone(), 3 * cars),
        list_field("boost_pad_active", b, pads),
        list_field("boost_pad_cooldown", f.clone(), pads),
        list_field("scores", f.clone(), 2),
        Field::new("seconds_remaining", f, true),
        Field::new("frame_json", DataType::LargeBinary, true),
    ];
    let metadata = HashMap::from([
        ("columnar_version".to_owned(), "1".to_owned()),
        (
            "replay_header_json".to_owned(),
            String::from_utf8(header_json).map_err(io::Error::other)?,
        ),
        (
            "pad_config_json".to_owned(),
            String::from_utf8(pad_json).map_err(io::Error::other)?,
        ),
    ]);
    Ok(Arc::new(Schema::new_with_metadata(fields, metadata)))
}

fn floats(rows: &[Row], width: usize, field: fn(&Row) -> &Vec<f32>) -> io::Result<ArrayRef> {
    let values: Vec<f32> = rows.iter().flat_map(|r| field(r).iter().copied()).collect();
    Ok(Arc::new(
        FixedSizeListArray::try_new(
            Arc::new(Field::new("item", DataType::Float32, true)),
            width as i32,
            Arc::new(Float32Array::from(values)),
            None,
        )
        .map_err(io::Error::other)?,
    ))
}

fn bools(rows: &[Row], width: usize, field: fn(&Row) -> &Vec<bool>) -> io::Result<ArrayRef> {
    let values: Vec<bool> = rows.iter().flat_map(|r| field(r).iter().copied()).collect();
    Ok(Arc::new(
        FixedSizeListArray::try_new(
            Arc::new(Field::new("item", DataType::Boolean, true)),
            width as i32,
            Arc::new(BooleanArray::from(values)),
            None,
        )
        .map_err(io::Error::other)?,
    ))
}

fn batch(rows: &[Row], schema: SchemaRef, cars: usize, pads: usize) -> io::Result<RecordBatch> {
    let columns: Vec<ArrayRef> = vec![
        Arc::new(UInt32Array::from(
            rows.iter().map(|r| r.frame).collect::<Vec<_>>(),
        )),
        Arc::new(Float64Array::from(
            rows.iter().map(|r| r.replay_time).collect::<Vec<_>>(),
        )),
        Arc::new(UInt64Array::from(
            rows.iter().map(|r| r.timeline_tick).collect::<Vec<_>>(),
        )),
        Arc::new(UInt64Array::from(
            rows.iter().map(|r| r.arena_tick).collect::<Vec<_>>(),
        )),
        floats(rows, 3, |r| &r.ball_position)?,
        floats(rows, 9, |r| &r.ball_rotation_columns)?,
        floats(rows, 3, |r| &r.ball_velocity)?,
        floats(rows, 3, |r| &r.ball_angular_velocity)?,
        floats(rows, cars * 3, |r| &r.car_position)?,
        floats(rows, cars * 9, |r| &r.car_rotation_columns)?,
        floats(rows, cars * 3, |r| &r.car_velocity)?,
        floats(rows, cars * 3, |r| &r.car_angular_velocity)?,
        floats(rows, cars, |r| &r.car_boost)?,
        bools(rows, cars, |r| &r.car_present)?,
        bools(rows, cars, |r| &r.car_demoed)?,
        floats(rows, cars * 5, |r| &r.control_axes)?,
        bools(rows, cars * 3, |r| &r.control_buttons)?,
        bools(rows, pads, |r| &r.boost_pad_active)?,
        floats(rows, pads, |r| &r.boost_pad_cooldown)?,
        floats(rows, 2, |r| &r.scores)?,
        Arc::new(Float32Array::from(
            rows.iter().map(|r| r.seconds_remaining).collect::<Vec<_>>(),
        )),
        Arc::new(LargeBinaryArray::from_iter_values(
            rows.iter().map(|r| r.frame_json.as_slice()),
        )),
    ];
    RecordBatch::try_new(schema, columns).map_err(io::Error::other)
}

/// Parse once and simulate twice: discover final fixed list widths, then write
/// frames directly to Parquet in bounded batches. Parsed replay observations
/// remain resident because the offline control model uses future packets.
pub fn write_parquet(
    bytes: &[u8],
    options: &ConvertOptions,
    file: File,
) -> Result<usize, Box<dyn Error>> {
    let replay = crate::parse_replay(bytes)?;
    let observed = observations::extract(&replay).ok_or("replay has no network frames")?;
    drop(replay);
    if observed.frames.is_empty() {
        return Err("cannot build a columnar schema for a replay without frames".into());
    }
    let mut pad_json = None;
    let expected = conversion::convert_observations_with(&observed, options, |frame, _, _| {
        if pad_json.is_none() {
            let pads: Vec<_> = frame
                .state
                .boost_pads
                .iter()
                .map(|(config, state)| serialization::PadRecord {
                    position: config.pos.to_array(),
                    is_big: config.is_big,
                    cooldown: state.cooldown,
                    is_active: state.is_active(),
                })
                .collect();
            pad_json = Some(serde_json::to_vec(&pads).map_err(io::Error::other)?);
        }
        Ok(())
    })?;
    let source_sha256 = Some(format!("{:x}", Sha256::digest(bytes)));
    let header = serialization::header_json(&observed, options, &expected, &source_sha256)?;
    let pad_json = pad_json.unwrap_or_else(|| b"[]".to_vec());
    let pads: Vec<serde_json::Value> = serde_json::from_slice(&pad_json)?;
    let schema = schema(expected.car_slots.len(), pads.len(), header, pad_json)?;
    let properties = WriterProperties::builder()
        .set_compression(Compression::ZSTD(ZstdLevel::try_new(3)?))
        .set_max_row_group_row_count(Some(BATCH_SIZE))
        .build();
    let mut writer = ArrowWriter::try_new(file, schema.clone(), Some(properties))?;
    let mut rows = Vec::with_capacity(BATCH_SIZE);
    let mut count = 0usize;
    let slot_index: HashMap<usize, usize> = expected
        .car_slots
        .iter()
        .enumerate()
        .map(|(index, slot)| (slot.slot, index))
        .collect();
    let actual = conversion::convert_observations_with(
        &observed,
        options,
        |frame, observation, residuals| {
            if frame.replay_frame != count {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "nonsequential frame index",
                ));
            }
            rows.push(row(
                frame,
                observation,
                residuals,
                &slot_index,
                expected.car_slots.len(),
                pads.len(),
            )?);
            count += 1;
            if rows.len() == BATCH_SIZE {
                writer
                    .write(&batch(
                        &rows,
                        schema.clone(),
                        expected.car_slots.len(),
                        pads.len(),
                    )?)
                    .map_err(io::Error::other)?;
                rows.clear();
            }
            Ok(())
        },
    )?;
    if actual != expected {
        return Err("conversion passes differed".into());
    }
    if !rows.is_empty() {
        writer.write(&batch(&rows, schema, expected.car_slots.len(), pads.len())?)?;
    }
    writer.close()?;
    Ok(count)
}
