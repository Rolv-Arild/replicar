//! Direct, batched Parquet projection of schema-v1 replay frames.

use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fs::File;
use std::io;
use std::path::Path;
use std::sync::Arc;

use arrow_array::{
    ArrayRef, BooleanArray, FixedSizeListArray, Float32Array, Float64Array, LargeBinaryArray,
    RecordBatch, UInt8Array, UInt32Array, UInt64Array,
};
use arrow_schema::{DataType, Field, Schema, SchemaRef};
use parquet::arrow::ArrowWriter;
use parquet::basic::{Compression, ZstdLevel};
use parquet::file::metadata::KeyValue;
use parquet::file::properties::WriterProperties;
use rocketsim::Mat3A;
use sha2::{Digest, Sha256};

use crate::conversion::{self, ConvertOptions, ConvertedFrame, PositionResidual};
use crate::freshness::{FrameFreshness, FreshnessTracker};
use crate::labels::{FrameLabels, ReplayLabels};
use crate::observations;
use crate::parquet_tables::{BATCH_SIZE, ExportSummary, Tables, dict, dict_type};
use crate::scoreboard::ScoreboardFrame;
use crate::serialization;

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
    /// Reconstructed match clock (`None`: no scoreboard for the frame, so unknown, not zero).
    scoreboard: Option<ScoreboardFrame>,
    /// Per car slot: `Some(1)` while the slot is held demolished as a dead pawn shell by an observed goal
    /// explosion, `Some(2)` by an inference from a sleeping packet of an unlinked car, `None` otherwise.
    dead_shell_held: Vec<Option<u8>>,
    /// Per car slot: the slot's car is known only from its spawn pose in this frame (no rigid-body packet yet).
    spawn_pose_held: Vec<bool>,
    /// Training labels (`labels.rs`), written as the `label_*` columns.
    labels: FrameLabels,
    /// Per car slot: the raw ping byte of the slot's player (`observations::Player::ping_raw`); `None` until the
    /// player's first update, and for a slot not created yet. An observation, not a label.
    ping_raw: Vec<Option<u8>>,
    /// Packet freshness (`freshness.rs`), written as `ball_fresh` and its sibling columns.
    freshness: FrameFreshness,
}

/// The raw ping of each slot's player in this frame. `slot_players` holds each slot's player key, learned from
/// the frame's car actors (`ConvertedFrame::car_actor_slots`) the frame the slot is created in and kept after.
fn slot_pings(
    car_actor_slots: &[(i32, usize)],
    observed: &observations::Frame,
    slot_players: &mut [Option<String>],
) -> Vec<Option<u8>> {
    for &(actor, slot) in car_actor_slots {
        let key = observed
            .cars
            .iter()
            .find(|car| car.actor_id == actor)
            .and_then(|car| car.player_key.as_ref());
        if let (Some(key), Some(known)) = (key, slot_players.get_mut(slot)) {
            known.get_or_insert_with(|| key.clone());
        }
    }
    let mut ping_of: HashMap<&str, u8> = HashMap::new();
    for player in &observed.players {
        if let Some(ping) = &player.ping_raw {
            ping_of.insert(player.key.as_str(), ping.value);
        }
    }
    slot_players
        .iter()
        .map(|key| key.as_deref().and_then(|key| ping_of.get(key).copied()))
        .collect()
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
    labels: FrameLabels,
    ping_raw: Vec<Option<u8>>,
    freshness: FrameFreshness,
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
        frame_json: serialization::frame_json(converted, observed, residuals, &labels, &freshness)
            .map_err(io::Error::other)?,
        scoreboard: converted.scoreboard.clone(),
        dead_shell_held: vec![None; cars],
        spawn_pose_held: vec![false; cars],
        labels,
        ping_raw,
        freshness,
    };
    for &slot in &converted.spawn_pose_held {
        let Some(&index) = slot_index.get(&slot) else {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "spawn pose hold on an unlisted car slot"));
        };
        result.spawn_pose_held[index] = true;
    }
    for held in &converted.dead_shells_held {
        let Some(&index) = slot_index.get(&held.slot) else {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "dead shell on an unlisted car slot"));
        };
        result.dead_shell_held[index] = Some(match held.source {
            "observed" => 1,
            _ => 2,
        });
    }
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

/// The main file's schema. The replay header (`replay_header_json`) is not in it: it holds the
/// conversion's diagnostics, known only after the single conversion pass, so the writer appends it
/// to the file's key-value metadata at close, where Arrow readers merge it into the schema metadata.
fn schema(cars: usize, pads: usize, pad_json: Vec<u8>) -> io::Result<SchemaRef> {
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
        // Appended after the original columns so existing positions and names are unchanged.
        Field::new("scoreboard_period", dict_type(), true),
        Field::new("scoreboard_clock_state", dict_type(), true),
        Field::new("scoreboard_seconds_remaining", DataType::Float32, true),
        Field::new("scoreboard_overtime_seconds", DataType::Float32, true),
        // Per car slot (null: not held): 1 held as a dead pawn shell by an observed goal explosion, 2 by an
        // inference from a sleeping packet of a car with no active link.
        list_field("dead_shell_held", DataType::UInt8, cars),
        // Per car slot: the car is known only from its spawn pose (no rigid-body packet yet); false otherwise.
        list_field("spawn_pose_held", DataType::Boolean, cars),
        // Training labels (`labels.rs`), always separate from the state and observations. Future-derived
        // (read from later frames): label_episode, label_episode_seconds_remaining, label_next_scoring_team
        // and label_seconds_until_next_goal; null when not applicable (outside play, no further goal).
        // Observed: label_update_age_seconds, per car slot (null: no car, or no packet yet).
        Field::new("label_episode", DataType::UInt32, true),
        Field::new("label_episode_seconds_remaining", DataType::Float32, true),
        Field::new("label_next_scoring_team", DataType::UInt8, true),
        Field::new("label_seconds_until_next_goal", DataType::Float32, true),
        list_field("label_update_age_seconds", DataType::Float32, cars),
        // Per car slot, an observation (not a label): the raw `Engine.PlayerReplicationInfo:Ping` byte of the
        // slot's player (null until its first update, never 0 for unknown).
        list_field("ping_raw", DataType::UInt8, cars),
        // Packet freshness (`freshness.rs`), observed or inferred, not labels. The tick ages use the offline
        // packet-lag inference and are null when no lag was inferred (see `freshness.rs`).
        Field::new("ball_fresh", DataType::Boolean, true),
        list_field("car_fresh", DataType::Boolean, cars),
        Field::new("ball_update_age_seconds", DataType::Float32, true),
        Field::new("ball_packet_age_ticks", DataType::UInt32, true),
        list_field("car_packet_age_ticks", DataType::UInt32, cars),
    ];
    let metadata = HashMap::from([
        ("columnar_version".to_owned(), "1".to_owned()),
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

fn small_ints(rows: &[Row], width: usize, field: fn(&Row) -> &Vec<Option<u8>>) -> io::Result<ArrayRef> {
    let values: Vec<Option<u8>> = rows.iter().flat_map(|r| field(r).iter().copied()).collect();
    Ok(Arc::new(
        FixedSizeListArray::try_new(
            Arc::new(Field::new("item", DataType::UInt8, true)),
            width as i32,
            Arc::new(UInt8Array::from(values)),
            None,
        )
        .map_err(io::Error::other)?,
    ))
}

/// A fixed-width list whose items (built by `values` from all rows, row by row) may be null.
fn nullable_list(
    rows: &[Row],
    width: usize,
    item: DataType,
    values: impl FnOnce(&[Row]) -> ArrayRef,
) -> io::Result<ArrayRef> {
    Ok(Arc::new(
        FixedSizeListArray::try_new(Arc::new(Field::new("item", item, true)), width as i32, values(rows), None)
            .map_err(io::Error::other)?,
    ))
}

/// A fixed-width list of nullable floats (an item is null when unknown).
fn nullable_floats(rows: &[Row], width: usize, field: fn(&Row) -> &Vec<Option<f32>>) -> io::Result<ArrayRef> {
    let values: Vec<Option<f32>> = rows.iter().flat_map(|r| field(r).iter().copied()).collect();
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
        dict(rows.iter().map(|r| r.scoreboard.as_ref().map(|s| s.period)))?,
        dict(
            rows.iter()
                .map(|r| r.scoreboard.as_ref().map(|s| s.clock_state)),
        )?,
        Arc::new(Float32Array::from(
            rows.iter()
                .map(|r| r.scoreboard.as_ref().and_then(|s| s.seconds_remaining))
                .collect::<Vec<_>>(),
        )),
        Arc::new(Float32Array::from(
            rows.iter()
                .map(|r| r.scoreboard.as_ref().and_then(|s| s.overtime_seconds))
                .collect::<Vec<_>>(),
        )),
        small_ints(rows, cars, |r| &r.dead_shell_held)?,
        bools(rows, cars, |r| &r.spawn_pose_held)?,
        Arc::new(UInt32Array::from(rows.iter().map(|r| r.labels.episode).collect::<Vec<_>>())),
        Arc::new(Float32Array::from(
            rows.iter().map(|r| r.labels.episode_seconds_remaining).collect::<Vec<_>>(),
        )),
        Arc::new(UInt8Array::from(
            rows.iter().map(|r| r.labels.next_scoring_team).collect::<Vec<_>>(),
        )),
        Arc::new(Float32Array::from(
            rows.iter().map(|r| r.labels.seconds_until_next_goal).collect::<Vec<_>>(),
        )),
        nullable_floats(rows, cars, |r| &r.labels.update_age_seconds)?,
        small_ints(rows, cars, |r| &r.ping_raw)?,
        Arc::new(BooleanArray::from(rows.iter().map(|r| r.freshness.ball_fresh).collect::<Vec<_>>())),
        nullable_list(rows, cars, DataType::Boolean, |rows| {
            Arc::new(BooleanArray::from(
                rows.iter().flat_map(|r| r.freshness.car_fresh.iter().copied()).collect::<Vec<Option<bool>>>(),
            ))
        })?,
        Arc::new(Float32Array::from(
            rows.iter().map(|r| r.freshness.ball_update_age_seconds).collect::<Vec<_>>(),
        )),
        Arc::new(UInt32Array::from(
            rows.iter().map(|r| r.freshness.ball_packet_age_ticks).collect::<Vec<_>>(),
        )),
        nullable_list(rows, cars, DataType::UInt32, |rows| {
            Arc::new(UInt32Array::from(
                rows.iter().flat_map(|r| r.freshness.car_packet_age_ticks.iter().copied()).collect::<Vec<Option<u32>>>(),
            ))
        })?,
    ];
    RecordBatch::try_new(schema, columns).map_err(io::Error::other)
}

/// Parse once and simulate once, writing frames directly to Parquet in bounded batches (the fixed list
/// widths come from a scan of the observations). Parsed replay observations remain resident because
/// the offline control model uses future packets.
pub fn write_parquet(
    bytes: &[u8],
    options: &ConvertOptions,
    file: File,
) -> Result<usize, Box<dyn Error>> {
    Ok(write_parquet_with_tables(bytes, options, file, None)?.frames)
}

/// `write_parquet` that also writes the record tables (`parquet_tables::TABLE_NAMES`) beside
/// `tables_beside`, the main file's path (see `parquet_tables::table_path`), in the same pass and
/// with the same bounded batches. The tables are always created, empty when the replay has no such
/// records, so their schemas can be relied on.
pub fn write_parquet_with_tables(
    bytes: &[u8],
    options: &ConvertOptions,
    file: File,
    tables_beside: Option<&Path>,
) -> Result<ExportSummary, Box<dyn Error>> {
    let replay = crate::parse_replay(bytes)?;
    let observed = observations::extract(&replay).ok_or("replay has no network frames")?;
    drop(replay);
    if observed.frames.is_empty() {
        return Err("cannot build a columnar schema for a replay without frames".into());
    }
    // One conversion pass. The per-slot column widths come from a scan of the observations (the
    // number of slots does not depend on the simulation), the pad configuration from the first frame,
    // and the replay header, which holds the conversion's diagnostics, is appended to the file's
    // key-value metadata once the pass is done.
    let cars = conversion::car_slot_count(&observed);
    if cars == 0 {
        return Err("the replay has no player-linked car (no car slot): there is nothing to put in the per-car Parquet columns; export it as JSONL".into());
    }
    // A slot's column is its index: slots are numbered in the order they are created.
    let slot_index: HashMap<usize, usize> = (0..cars).map(|slot| (slot, slot)).collect();
    let properties = WriterProperties::builder()
        .set_compression(Compression::ZSTD(ZstdLevel::try_new(3)?))
        .set_max_row_group_row_count(Some(BATCH_SIZE))
        .build();
    let source_sha256 = format!("{:x}", Sha256::digest(bytes));
    // The conversion options as serialized in the header; the hash lets a reader tell tables of another
    // conversion of the same replay from this one's.
    let options_sha256 = format!("{:x}", Sha256::digest(serde_json::to_vec(options)?));
    let mut tables = tables_beside
        .map(|main| {
            Tables::create(
                main,
                &properties,
                &[
                    ("source_sha256", source_sha256.clone()),
                    ("options_sha256", options_sha256.clone()),
                ],
            )
        })
        .transpose()?;
    let replay_labels = ReplayLabels::new(&observed);
    let mut slot_players: Vec<Option<String>> = vec![None; cars];
    let mut freshness = FreshnessTracker::new(cars);
    let mut file = Some(file);
    let mut writer: Option<(ArrowWriter<File>, SchemaRef, usize)> = None;
    let mut rows = Vec::with_capacity(BATCH_SIZE);
    let mut count = 0usize;
    let summary = conversion::convert_observations_with(
        &observed,
        options,
        |frame, observation, residuals| {
            if frame.replay_frame != count {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "nonsequential frame index",
                ));
            }
            if writer.is_none() {
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
                let pad_json = serde_json::to_vec(&pads).map_err(io::Error::other)?;
                let schema = schema(cars, pads.len(), pad_json)?;
                let file = file.take().expect("the file is taken once");
                let created = ArrowWriter::try_new(file, schema.clone(), Some(properties.clone()))
                    .map_err(io::Error::other)?;
                writer = Some((created, schema, pads.len()));
            }
            let (arrow_writer, schema, pad_count) = writer.as_mut().expect("created above");
            if let Some(tables) = tables.as_mut() {
                tables.add_frame(frame, observation)?;
            }
            let labels = replay_labels.frame(&observed, frame.replay_frame, frame, cars);
            let ping_raw = slot_pings(&frame.car_actor_slots, observation, &mut slot_players);
            let fresh = freshness.frame(&observed, frame.replay_frame, frame);
            rows.push(row(frame, observation, residuals, labels, ping_raw, fresh, &slot_index, cars, *pad_count)?);
            count += 1;
            if rows.len() == BATCH_SIZE {
                arrow_writer
                    .write(&batch(&rows, schema.clone(), cars, *pad_count)?)
                    .map_err(io::Error::other)?;
                rows.clear();
            }
            Ok(())
        },
    )?;
    if summary.car_slots.len() != cars
        || summary.car_slots.iter().enumerate().any(|(index, slot)| slot.slot != index)
    {
        return Err("the conversion's car slots differ from the slot scan".into());
    }
    let (mut writer, schema, pad_count) = writer.ok_or("the conversion produced no frames")?;
    if !rows.is_empty() {
        writer.write(&batch(&rows, schema, cars, pad_count)?)?;
    }
    let header = serialization::header_json(&observed, options, &summary, &Some(source_sha256.clone()))?;
    writer.append_key_value_metadata(KeyValue::new(
        "replay_header_json".to_owned(),
        String::from_utf8(header)?,
    ));
    writer.append_key_value_metadata(KeyValue::new("options_sha256".to_owned(), options_sha256));
    writer.close()?;
    let table_rows = match tables {
        Some(tables) => tables.finish(count)?,
        None => Vec::new(),
    };
    Ok(ExportSummary {
        frames: count,
        table_rows,
    })
}

#[cfg(test)]
mod tests {
    use arrow_array::cast::AsArray;
    use arrow_array::types::{Float32Type, UInt8Type};
    use arrow_array::{Array, DictionaryArray};

    use super::*;

    fn blank_row(frame: u32, scoreboard: Option<ScoreboardFrame>) -> Row {
        Row {
            frame,
            replay_time: 0.0,
            timeline_tick: 0,
            arena_tick: 0,
            ball_position: vec![0.0; 3],
            ball_rotation_columns: vec![0.0; 9],
            ball_velocity: vec![0.0; 3],
            ball_angular_velocity: vec![0.0; 3],
            car_position: vec![f32::NAN; 3],
            car_rotation_columns: vec![f32::NAN; 9],
            car_velocity: vec![f32::NAN; 3],
            car_angular_velocity: vec![f32::NAN; 3],
            car_boost: vec![f32::NAN; 1],
            car_present: vec![false; 1],
            car_demoed: vec![false; 1],
            control_axes: vec![f32::NAN; 5],
            control_buttons: vec![false; 3],
            boost_pad_active: vec![true; 1],
            boost_pad_cooldown: vec![0.0; 1],
            scores: vec![0.0, 1.0],
            seconds_remaining: f32::NAN,
            frame_json: b"{}".to_vec(),
            scoreboard,
            dead_shell_held: vec![None; 1],
            spawn_pose_held: vec![false; 1],
            labels: FrameLabels {
                episode: None,
                episode_seconds_remaining: None,
                next_scoring_team: None,
                seconds_until_next_goal: None,
                update_age_seconds: vec![None],
            },
            ping_raw: vec![None],
            freshness: FrameFreshness {
                ball_fresh: false,
                car_fresh: vec![None],
                ball_update_age_seconds: None,
                ball_packet_age_ticks: None,
                car_packet_age_ticks: vec![None],
            },
        }
    }

    #[test]
    fn original_columns_keep_their_positions_and_scoreboard_columns_follow() {
        let schema = schema(1, 1, b"[]".to_vec()).unwrap();
        let names: Vec<_> = schema.fields().iter().map(|f| f.name().as_str()).collect();
        assert_eq!(
            names[..22],
            [
                "frame",
                "replay_time",
                "timeline_tick",
                "arena_tick",
                "ball_position",
                "ball_rotation_columns",
                "ball_velocity",
                "ball_angular_velocity",
                "car_position",
                "car_rotation_columns",
                "car_velocity",
                "car_angular_velocity",
                "car_boost",
                "car_present",
                "car_demoed",
                "control_axes",
                "control_buttons",
                "boost_pad_active",
                "boost_pad_cooldown",
                "scores",
                "seconds_remaining",
                "frame_json",
            ]
        );
        assert_eq!(
            names[22..],
            [
                "scoreboard_period",
                "scoreboard_clock_state",
                "scoreboard_seconds_remaining",
                "scoreboard_overtime_seconds",
                "dead_shell_held",
                "spawn_pose_held",
                "label_episode",
                "label_episode_seconds_remaining",
                "label_next_scoring_team",
                "label_seconds_until_next_goal",
                "label_update_age_seconds",
                "ping_raw",
                "ball_fresh",
                "car_fresh",
                "ball_update_age_seconds",
                "ball_packet_age_ticks",
                "car_packet_age_ticks",
            ]
        );
        assert_eq!(schema.metadata()["columnar_version"], "1");
    }

    #[test]
    fn a_missing_scoreboard_or_clock_value_is_null_not_zero() {
        let schema = schema(1, 1, b"[]".to_vec()).unwrap();
        let rows = [
            blank_row(
                0,
                Some(ScoreboardFrame {
                    period: "regulation",
                    clock_state: "running",
                    seconds_remaining: Some(123.5),
                    overtime_seconds: None,
                }),
            ),
            blank_row(
                1,
                Some(ScoreboardFrame {
                    period: "overtime",
                    clock_state: "kickoff",
                    seconds_remaining: None,
                    overtime_seconds: Some(0.0),
                }),
            ),
            blank_row(2, None),
        ];
        let batch = batch(&rows, schema, 1, 1).unwrap();
        let strings = |name: &str| -> Vec<Option<String>> {
            let column = batch.column_by_name(name).unwrap();
            let dictionary = column
                .as_any()
                .downcast_ref::<DictionaryArray<UInt8Type>>()
                .unwrap();
            let values = dictionary.values().as_string::<i32>();
            (0..dictionary.len())
                .map(|i| dictionary.key(i).map(|k| values.value(k).to_owned()))
                .collect()
        };
        assert_eq!(
            strings("scoreboard_period"),
            [Some("regulation".into()), Some("overtime".into()), None]
        );
        assert_eq!(
            strings("scoreboard_clock_state"),
            [Some("running".into()), Some("kickoff".into()), None]
        );
        let remaining = batch
            .column_by_name("scoreboard_seconds_remaining")
            .unwrap()
            .as_primitive::<Float32Type>();
        assert_eq!(remaining.value(0), 123.5);
        assert!(remaining.is_null(1) && remaining.is_null(2));
        let overtime = batch
            .column_by_name("scoreboard_overtime_seconds")
            .unwrap()
            .as_primitive::<Float32Type>();
        assert!(overtime.is_null(0) && overtime.is_null(2));
        // An overtime clock of exactly 0 s is a value, distinct from unknown.
        assert!(overtime.is_valid(1) && overtime.value(1) == 0.0);
    }

    /// A one-slot blank row widened to `cars` slots.
    fn wide_row(frame: u32, cars: usize, labels: FrameLabels) -> Row {
        let mut row = blank_row(frame, None);
        row.car_position = vec![f32::NAN; 3 * cars];
        row.car_rotation_columns = vec![f32::NAN; 9 * cars];
        row.car_velocity = vec![f32::NAN; 3 * cars];
        row.car_angular_velocity = vec![f32::NAN; 3 * cars];
        row.car_boost = vec![f32::NAN; cars];
        row.car_present = vec![false; cars];
        row.car_demoed = vec![false; cars];
        row.control_axes = vec![f32::NAN; 5 * cars];
        row.control_buttons = vec![false; 3 * cars];
        row.dead_shell_held = vec![None; cars];
        row.spawn_pose_held = vec![false; cars];
        row.ping_raw = vec![None; cars];
        row.freshness.car_fresh = vec![None; cars];
        row.freshness.car_packet_age_ticks = vec![None; cars];
        row.labels = labels;
        row
    }

    #[test]
    fn unknown_labels_are_null_and_known_ones_keep_their_values() {
        let schema = schema(2, 1, b"[]".to_vec()).unwrap();
        let known = wide_row(
            0,
            2,
            FrameLabels {
                episode: Some(0),
                episode_seconds_remaining: Some(0.0),
                next_scoring_team: Some(0),
                seconds_until_next_goal: Some(0.0),
                update_age_seconds: vec![Some(0.0), None],
            },
        );
        let unknown = wide_row(
            1,
            2,
            FrameLabels {
                episode: None,
                episode_seconds_remaining: None,
                next_scoring_team: None,
                seconds_until_next_goal: None,
                update_age_seconds: vec![None, None],
            },
        );
        let batch = batch(&[known, unknown], schema, 2, 1).unwrap();
        let column = |name: &str| batch.column_by_name(name).unwrap().clone();
        // Episode 0, 0.0 s, team 0 (blue) are values, distinct from unknown.
        let episode = column("label_episode");
        let episode = episode.as_primitive::<arrow_array::types::UInt32Type>();
        assert!(episode.is_valid(0) && episode.value(0) == 0 && episode.is_null(1));
        for name in ["label_episode_seconds_remaining", "label_seconds_until_next_goal"] {
            let array = column(name);
            let array = array.as_primitive::<Float32Type>();
            assert!(array.is_valid(0) && array.value(0) == 0.0 && array.is_null(1), "{name}");
        }
        let team = column("label_next_scoring_team");
        let team = team.as_primitive::<UInt8Type>();
        assert!(team.is_valid(0) && team.value(0) == 0 && team.is_null(1));
        let ages = column("label_update_age_seconds");
        let values = ages.as_fixed_size_list().values().as_primitive::<Float32Type>().clone();
        assert_eq!(values.len(), 4);
        assert!(values.is_valid(0) && values.value(0) == 0.0);
        assert!(values.is_null(1) && values.is_null(2) && values.is_null(3));
    }

    fn player(actor_id: i32, key: &str, ping: Option<(u8, usize)>) -> observations::Player {
        observations::Player {
            actor_id,
            key: key.to_owned(),
            name: None,
            team: Some(0),
            body_product_ids: [None, None],
            stats: Default::default(),
            ping_raw: ping.map(|(value, frame)| observations::Value {
                value,
                frame,
                source: observations::Source::Replay,
            }),
        }
    }

    fn car(actor_id: i32, key: Option<&str>) -> observations::Car {
        observations::Car {
            actor_id,
            actor_created_frame: 0,
            player_key: key.map(str::to_owned),
            player_link_active: true,
            team: Some(0),
            body_product_id: None,
            body: Default::default(),
            boost: None,
            boost_raw: None,
            inputs: Default::default(),
            spawn_pose: None,
        }
    }

    fn frame(cars: Vec<observations::Car>, players: Vec<observations::Player>) -> observations::Frame {
        observations::Frame {
            index: 0,
            time: 0.0,
            delta: 0.0,
            ball: None,
            cars,
            players,
            team_scores: [None, None],
            seconds_remaining: None,
            overtime: None,
            game_state: None,
            events: Vec::new(),
            pad_pickups: Vec::new(),
        }
    }

    #[test]
    fn a_slots_ping_is_its_players_raw_byte_and_null_until_there_is_one() {
        let mut slot_players = vec![None; 3];
        // Slots 0 and 1 exist (cars of players "a" and "b"); slot 2 is not created yet. Only "a" has a ping.
        let first = frame(
            vec![car(10, Some("a")), car(11, Some("b")), car(12, None)],
            vec![player(1, "a", Some((0, 3))), player(2, "b", None), player(3, "c", Some((9, 1)))],
        );
        let pings = slot_pings(&[(10, 0), (11, 1)], &first, &mut slot_players);
        // A ping of 0 is a value; no update is null; a player without a slot yet shows nowhere.
        assert_eq!(pings, [Some(0), None, None]);
        // The slot keeps its player once learned, also in a frame without its car.
        let later = frame(
            vec![car(12, Some("c"))],
            vec![player(1, "a", Some((7, 8))), player(2, "b", Some((6, 9))), player(3, "c", Some((9, 1)))],
        );
        let pings = slot_pings(&[(12, 2)], &later, &mut slot_players);
        assert_eq!(pings, [Some(7), Some(6), Some(9)]);
        // And it is a typed, nullable UInt8 list in the file.
        let schema = schema(3, 1, b"[]".to_vec()).unwrap();
        let mut row = wide_row(
            0,
            3,
            FrameLabels {
                episode: None,
                episode_seconds_remaining: None,
                next_scoring_team: None,
                seconds_until_next_goal: None,
                update_age_seconds: vec![None; 3],
            },
        );
        row.ping_raw = vec![Some(0), None, Some(255)];
        let batch = batch(&[row], schema, 3, 1).unwrap();
        let column = batch.column_by_name("ping_raw").unwrap().clone();
        let values = column.as_fixed_size_list().values().as_primitive::<UInt8Type>().clone();
        assert!(values.is_valid(0) && values.value(0) == 0);
        assert!(values.is_null(1));
        assert!(values.is_valid(2) && values.value(2) == 255);
    }

    #[test]
    fn freshness_columns_keep_unknown_as_null_and_a_zero_age_as_a_value() {
        let schema = schema(2, 1, b"[]".to_vec()).unwrap();
        let blank = FrameLabels {
            episode: None,
            episode_seconds_remaining: None,
            next_scoring_team: None,
            seconds_until_next_goal: None,
            update_age_seconds: vec![None; 2],
        };
        let mut fresh = wide_row(0, 2, blank.clone());
        fresh.freshness = FrameFreshness {
            ball_fresh: true,
            // Slot 0 fresh, slot 1 has no car.
            car_fresh: vec![Some(true), None],
            ball_update_age_seconds: Some(0.0),
            ball_packet_age_ticks: Some(0),
            car_packet_age_ticks: vec![Some(3), None],
        };
        let mut stale = wide_row(1, 2, blank);
        stale.freshness = FrameFreshness {
            ball_fresh: false,
            car_fresh: vec![Some(false), Some(false)],
            ball_update_age_seconds: None,
            ball_packet_age_ticks: None,
            car_packet_age_ticks: vec![None, Some(7)],
        };
        let batch = batch(&[fresh, stale], schema, 2, 1).unwrap();
        let column = |name: &str| batch.column_by_name(name).unwrap().clone();
        let ball_fresh = column("ball_fresh");
        let ball_fresh = ball_fresh.as_boolean();
        assert!(ball_fresh.value(0) && !ball_fresh.value(1) && ball_fresh.null_count() == 0);
        let car_fresh = column("car_fresh");
        let items = car_fresh.as_fixed_size_list().values().as_boolean().clone();
        assert!(items.is_valid(0) && items.value(0));
        assert!(items.is_null(1));
        assert!(items.is_valid(2) && !items.value(2) && items.is_valid(3) && !items.value(3));
        let age = column("ball_update_age_seconds");
        let age = age.as_primitive::<Float32Type>();
        assert!(age.is_valid(0) && age.value(0) == 0.0 && age.is_null(1));
        let ticks = column("ball_packet_age_ticks");
        let ticks = ticks.as_primitive::<arrow_array::types::UInt32Type>();
        assert!(ticks.is_valid(0) && ticks.value(0) == 0 && ticks.is_null(1));
        let car_ticks = column("car_packet_age_ticks");
        let items = car_ticks.as_fixed_size_list().values().as_primitive::<arrow_array::types::UInt32Type>().clone();
        assert!(items.is_valid(0) && items.value(0) == 3 && items.is_null(1));
        assert!(items.is_null(2) && items.is_valid(3) && items.value(3) == 7);
    }
}
