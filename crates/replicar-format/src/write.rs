//! Writing a replicar file: the rows in play segments (or every frame with `all_frames`), the groups asked
//! for, the header in the key-value metadata; written under a temporary name and renamed when complete.

use std::collections::BTreeSet;
use std::fs::File;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use arrow_array::RecordBatch;
use arrow_schema::{DataType, Schema};
use parquet::arrow::ArrowWriter;
use parquet::basic::{Compression, Encoding, ZstdLevel};
use parquet::file::metadata::KeyValue;
use parquet::file::properties::WriterProperties;
use parquet::schema::types::ColumnPath;

use crate::columns::{Columns, child_bool, child_f32, child_str, child_u8, child_u16, child_u64};
use crate::header::{HEADER_KEY, Header};
use crate::record::{Body, Car, CarInternals, Controls, Event, Frame, StatEvent};
use crate::resimulation::Resimulation;

/// A column group (docs/glossary.md, "Column group"); the frame columns are always written.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Group {
    State,
    Game,
    Updates,
    Future,
    /// Opt-in: what the fitted inference chose, to resimulate without fitting.
    Resimulation,
    /// Opt-in: the replay's network feed as replicar decodes it.
    Network,
    /// Opt-in: what the reconstruction measured about itself.
    Diagnostics,
}

impl Group {
    /// The groups written by default.
    pub const DEFAULT: [Self; 4] = [Self::State, Self::Game, Self::Updates, Self::Future];
    /// Every group.
    pub const ALL: [Self; 7] = [
        Self::State,
        Self::Game,
        Self::Updates,
        Self::Future,
        Self::Resimulation,
        Self::Network,
        Self::Diagnostics,
    ];

    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::State => "state",
            Self::Game => "game",
            Self::Updates => "updates",
            Self::Future => "future",
            Self::Resimulation => "resimulation",
            Self::Network => "network",
            Self::Diagnostics => "diagnostics",
        }
    }

    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|g| g.name() == name)
    }
}

/// How the state's bodies are stored (docs/glossary.md, "Precision").
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Precision {
    /// As RocketSim has them.
    #[default]
    Float32,
    /// Integers of 0.01 UU, 0.01 UU/s, 1e-4 rad/s and 1/32767 per quaternion component; the scale is in each
    /// field's metadata (`scale`).
    Quantized,
}

impl Precision {
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Float32 => "float32",
            Self::Quantized => "quantized",
        }
    }

    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        [Self::Float32, Self::Quantized]
            .into_iter()
            .find(|p| p.name() == name)
    }
}

/// The scales of the quantized precision: position (UU), velocity (UU/s), angular velocity (rad/s), rotation.
pub const QUANTA: [f64; 4] = [0.01, 0.01, 1e-4, 1.0 / 32767.0];

/// What a file holds.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WriteOptions {
    pub groups: BTreeSet<Group>,
    pub precision: Precision,
    /// Also the frames outside play segments, with a null `segment`.
    pub all_frames: bool,
    /// zstd level.
    pub compression_level: i32,
}

impl Default for WriteOptions {
    fn default() -> Self {
        Self {
            groups: Group::DEFAULT.into_iter().collect(),
            precision: Precision::Float32,
            all_frames: false,
            compression_level: 9,
        }
    }
}

/// The error of a write.
#[derive(Debug, thiserror::Error)]
pub enum WriteError {
    #[error("I/O: {0}")]
    Io(#[from] io::Error),
    #[error("Parquet: {0}")]
    Parquet(#[from] parquet::errors::ParquetError),
    #[error("Arrow: {0}")]
    Arrow(#[from] arrow_schema::ArrowError),
    #[error("header: {0}")]
    Header(#[from] serde_json::Error),
    #[error("the {0} group was asked for, but the conversion did not make it")]
    Missing(&'static str),
}

/// Writes `frames` to `path` with `header` (its group list and frame setting are taken from `options`).
/// The file appears only when complete.
/// What a conversion has for a file: every frame's row, and the opt-in groups.
#[derive(Debug, Clone, Copy)]
pub struct Content<'a> {
    pub frames: &'a [Frame],
    pub resimulation: Option<&'a Resimulation>,
    /// The `network` and `diagnostics` columns, one row per frame of `frames` (built with `Columns`).
    pub network: Option<&'a RecordBatch>,
    pub diagnostics: Option<&'a RecordBatch>,
}

/// Writes `content` to `path` with `header` (its group list, precision and frame setting are taken from
/// `options`). The file appears only when complete.
pub fn write(
    path: &Path,
    header: &Header,
    content: &Content,
    options: &WriteOptions,
) -> Result<(), WriteError> {
    let frames = content.frames;
    let written: Vec<bool> = frames
        .iter()
        .map(|f| options.all_frames || f.segment.is_some())
        .collect();
    let rows: Vec<&Frame> = frames
        .iter()
        .zip(&written)
        .filter_map(|(f, &w)| w.then_some(f))
        .collect();
    // A written row's stat events, with those of the left-out frames after it (an assist during the goal pause):
    // each event keeps its own `updated_frame`; the
    let mut stat_events: Vec<Vec<&StatEvent>> = vec![Vec::new(); rows.len()];
    // ones before the first written row go on it.
    let mut current: Option<usize> = None;
    for (f, &w) in frames.iter().zip(&written) {
        if w {
            current = Some(current.map_or(0, |r| r + 1));
        }
        if let Some(list) = stat_events.get_mut(current.unwrap_or(0)) {
            list.extend(&f.game.stat_events);
        }
    }
    let players = header.players.len();
    let pads = header.pads.len();
    let mut columns = Columns::default();
    frame_columns(&mut columns, &rows);
    for group in &options.groups {
        match group {
            Group::State => state_columns(&mut columns, &rows, players, pads, options.precision),
            Group::Game => game_columns(&mut columns, &rows, &stat_events),
            Group::Updates => update_columns(&mut columns, &rows, players),
            Group::Future => future_columns(&mut columns, &rows),
            Group::Network | Group::Diagnostics => {
                let batch = if *group == Group::Network {
                    content.network
                } else {
                    content.diagnostics
                };
                let batch = batch.ok_or(WriteError::Missing(group.name()))?;
                let rows = arrow_select::filter::filter_record_batch(
                    batch,
                    &arrow_array::BooleanArray::from(written.clone()),
                )?;
                columns
                    .fields
                    .extend(rows.schema().fields().iter().map(|f| (**f).clone()));
                columns.arrays.extend(rows.columns().iter().cloned());
            }
            Group::Resimulation => {
                let group = content
                    .resimulation
                    .ok_or(WriteError::Missing("resimulation"))?;
                let frames: Vec<u32> = rows.iter().map(|r| r.frame.0).collect();
                crate::resimulation::columns(&mut columns, &frames, group);
            }
        }
    }
    let mut header = header.clone();
    header.groups = options.groups.iter().map(|g| g.name().to_owned()).collect();
    header.all_frames = options.all_frames;
    header.precision = options.precision.name().to_owned();
    let schema = Arc::new(Schema::new(columns.fields));
    let mut properties = WriterProperties::builder()
        .set_compression(Compression::ZSTD(ZstdLevel::try_new(
            options.compression_level,
        )?))
        .set_max_row_group_row_count(Some(1 << 20));
    for field in schema.fields() {
        for (column, encoding) in leaf_encodings(Vec::new(), field) {
            properties = properties
                .set_column_dictionary_enabled(column.clone(), false)
                .set_column_encoding(column, encoding);
        }
    }
    let batch = RecordBatch::try_new(schema.clone(), columns.arrays)?;
    let temporary = temporary_path(path);
    let result = (|| -> Result<(), WriteError> {
        let mut writer =
            ArrowWriter::try_new(File::create(&temporary)?, schema, Some(properties.build()))?;
        writer.write(&batch)?;
        writer.append_key_value_metadata(KeyValue::new(
            HEADER_KEY.to_owned(),
            serde_json::to_string(&header)?,
        ));
        writer.close()?;
        std::fs::rename(&temporary, path)?;
        Ok(())
    })();
    if result.is_err() {
        let _ = std::fs::remove_file(&temporary);
    }
    result
}

/// Writes a plain table (one row group, zstd, float columns byte-split) to `path`, with `metadata` as key-value
/// pairs; the file appears only when complete. For files beside the replicar files, such as a corpus index.
pub fn write_table(
    path: &Path,
    batch: &RecordBatch,
    metadata: &[(&str, String)],
) -> Result<(), WriteError> {
    let mut properties =
        WriterProperties::builder().set_compression(Compression::ZSTD(ZstdLevel::try_new(9)?));
    for field in batch.schema().fields() {
        for (column, encoding) in leaf_encodings(Vec::new(), field) {
            properties = properties
                .set_column_dictionary_enabled(column.clone(), false)
                .set_column_encoding(column, encoding);
        }
    }
    let temporary = temporary_path(path);
    let result = (|| -> Result<(), WriteError> {
        let mut writer = ArrowWriter::try_new(
            File::create(&temporary)?,
            batch.schema(),
            Some(properties.build()),
        )?;
        writer.write(batch)?;
        for (key, value) in metadata {
            writer.append_key_value_metadata(KeyValue::new((*key).to_owned(), value.clone()));
        }
        writer.close()?;
        std::fs::rename(&temporary, path)?;
        Ok(())
    })();
    if result.is_err() {
        let _ = std::fs::remove_file(&temporary);
    }
    result
}

/// The encoding of every leaf column under `field` that is not the writer's default: floats split into byte
/// streams compress about twice as well (RESULTS.md, "Output size and conversion cost"); quantized bodies change
/// little from frame to frame, so their deltas pack 12-23% better than plain integers (RESULTS.md, "v2
/// encodings"), and so do the frames, ticks and ordinals of the record lists, which mostly increase.
fn leaf_encodings(
    mut path: Vec<String>,
    field: &arrow_schema::Field,
) -> Vec<(ColumnPath, Encoding)> {
    path.push(field.name().clone());
    match field.data_type() {
        DataType::List(item) => {
            path.push("list".to_owned());
            leaf_encodings(path, item)
        }
        DataType::Struct(children) => children
            .iter()
            .flat_map(|child| leaf_encodings(path.clone(), child))
            .collect(),
        DataType::Float32 => vec![(ColumnPath::new(path), Encoding::BYTE_STREAM_SPLIT)],
        DataType::Int16 | DataType::Int32 if field.metadata().contains_key("scale") => {
            vec![(ColumnPath::new(path), Encoding::DELTA_BINARY_PACKED)]
        }
        DataType::UInt32 | DataType::UInt64 | DataType::Int64 if path.len() > 1 => {
            vec![(ColumnPath::new(path), Encoding::DELTA_BINARY_PACKED)]
        }
        _ => Vec::new(),
    }
}

/// `x.parquet` becomes `x.parquet.partial`, in the same directory so that the rename is atomic.
fn temporary_path(path: &Path) -> PathBuf {
    let mut name = path.file_name().unwrap_or_default().to_os_string();
    name.push(".partial");
    path.with_file_name(name)
}

const ALWAYS: &str = "frame";
const STATE: &str = "state";
const GAME: &str = "game";
const UPDATES: &str = "updates";
const FUTURE: &str = "future";
const XYZ: [&str; 3] = ["x", "y", "z"];
const XYZW: [&str; 4] = ["x", "y", "z", "w"];

/// A named value of a record.
type Named<T, R> = (&'static str, fn(&T) -> R);
/// A quantity of a body: name, components, unit, values.
type BodyPart = (
    &'static str,
    &'static [&'static str],
    &'static str,
    fn(&Body) -> &[f32],
);

fn frame_columns(columns: &mut Columns, rows: &[&Frame]) {
    columns.u32("frame", ALWAYS, None, rows.iter().map(|r| Some(r.frame.0)));
    columns.u32("segment", ALWAYS, None, rows.iter().map(|r| r.segment));
    columns.f32(
        "replay_time",
        ALWAYS,
        Some("s"),
        rows.iter().map(|r| Some(r.replay_time)),
    );
    columns.u32(
        "replay_tick",
        ALWAYS,
        Some("tick"),
        rows.iter().map(|r| Some(r.replay_tick)),
    );
    columns.u64(
        "sim_tick",
        ALWAYS,
        Some("tick"),
        rows.iter().map(|r| Some(r.sim_tick)),
    );
}

/// Position, velocity, angular velocity and rotation of a body, `prefix` naming it.
fn body_columns(
    columns: &mut Columns,
    rows: &[&Frame],
    prefix: &str,
    precision: Precision,
    body: impl Fn(&Frame) -> Option<Body>,
) {
    let parts: [BodyPart; 4] = [
        ("position", &XYZ, "UU", |b| &b.position),
        ("velocity", &XYZ, "UU/s", |b| &b.velocity),
        ("angular_velocity", &XYZ, "rad/s", |b| &b.angular_velocity),
        ("rotation", &XYZW, "quaternion", |b| &b.rotation),
    ];
    for (k, (quantity, components, unit, values)) in parts.into_iter().enumerate() {
        for (i, component) in components.iter().enumerate() {
            let name = format!("{prefix}_{quantity}_{component}");
            let values = rows.iter().map(|r| body(r).map(|b| values(&b)[i]));
            match precision {
                Precision::Float32 => columns.f32(name, STATE, Some(unit), values),
                // The rotation fits 16 bits; the rest needs 32.
                Precision::Quantized => {
                    columns.quantized(name, STATE, Some(unit), QUANTA[k], k == 3, values);
                }
            }
        }
    }
}

fn controls_columns(
    columns: &mut Columns,
    rows: &[&Frame],
    prefix: &str,
    controls: impl Fn(&Frame) -> Option<Controls>,
) {
    let axes: [Named<Controls, f32>; 5] = [
        ("throttle", |c| c.throttle),
        ("steer", |c| c.steer),
        ("pitch", |c| c.pitch),
        ("yaw", |c| c.yaw),
        ("roll", |c| c.roll),
    ];
    for (name, value) in axes {
        columns.f32(
            format!("{prefix}_{name}"),
            STATE,
            None,
            rows.iter().map(|r| controls(r).map(|c| value(&c))),
        );
    }
    let buttons: [Named<Controls, bool>; 3] = [
        ("jump", |c| c.jump),
        ("boost", |c| c.boost),
        ("handbrake", |c| c.handbrake),
    ];
    for (name, value) in buttons {
        columns.bool(
            format!("{prefix}_{name}"),
            STATE,
            rows.iter().map(|r| controls(r).map(|c| value(&c))),
        );
    }
}

fn state_columns(
    columns: &mut Columns,
    rows: &[&Frame],
    players: usize,
    pads: usize,
    precision: Precision,
) {
    body_columns(columns, rows, "ball", precision, |r| {
        Some(r.state.ball.body)
    });
    columns.u64(
        "ball_ticks_since_kickoff",
        STATE,
        Some("tick"),
        rows.iter().map(|r| Some(r.state.ball.ticks_since_kickoff)),
    );
    for p in 0..players {
        let car = |r: &Frame| -> Option<Car> { r.state.cars.get(p).copied().flatten() };
        let prefix = format!("car_{p}");
        columns.names(
            format!("{prefix}_status"),
            STATE,
            rows.iter()
                .map(|r| r.state.car_status.get(p).map(|s| s.name())),
        );
        columns.bool(
            format!("{prefix}_status_inferred"),
            STATE,
            rows.iter()
                .map(|r| r.state.car_status_inferred.get(p).copied()),
        );
        body_columns(columns, rows, &prefix, precision, |r| {
            car(r).map(|c| c.body)
        });
        columns.f32(
            format!("{prefix}_boost"),
            STATE,
            Some("0-100"),
            rows.iter().map(|r| car(r).map(|c| c.boost)),
        );
        controls_columns(columns, rows, &format!("{prefix}_controls"), |r| {
            car(r).map(|c| c.controls)
        });
        controls_columns(columns, rows, &format!("{prefix}_previous_controls"), |r| {
            car(r).map(|c| c.previous_controls)
        });
        internal_columns(columns, rows, &prefix, &car);
    }
    for pad in 0..pads {
        columns.f32(
            format!("pad_{pad}_cooldown"),
            STATE,
            Some("s"),
            rows.iter().map(|r| r.state.pad_cooldowns.get(pad).copied()),
        );
    }
}

fn internal_columns(
    columns: &mut Columns,
    rows: &[&Frame],
    prefix: &str,
    car: &impl Fn(&Frame) -> Option<Car>,
) {
    let internals = |r: &Frame| car(r).map(|c| c.internals);
    let flags: [Named<CarInternals, bool>; 9] = [
        ("is_on_ground", |c| c.is_on_ground),
        ("has_jumped", |c| c.has_jumped),
        ("has_double_jumped", |c| c.has_double_jumped),
        ("has_flipped", |c| c.has_flipped),
        ("is_flipping", |c| c.is_flipping),
        ("is_jumping", |c| c.is_jumping),
        ("is_boosting", |c| c.is_boosting),
        ("is_supersonic", |c| c.is_supersonic),
        ("is_auto_flipping", |c| c.is_auto_flipping),
    ];
    for (name, value) in flags {
        columns.bool(
            format!("{prefix}_{name}"),
            STATE,
            rows.iter().map(|r| internals(r).map(|c| value(&c))),
        );
    }
    columns.bool(
        format!("{prefix}_is_demoed"),
        STATE,
        rows.iter().map(|r| internals(r).map(|c| c.is_demoed)),
    );
    for wheel in 0..4 {
        columns.bool(
            format!("{prefix}_wheel_{wheel}_contact"),
            STATE,
            rows.iter()
                .map(|r| internals(r).map(|c| c.wheels_with_contact[wheel])),
        );
    }
    let timers: [Named<CarInternals, f32>; 11] = [
        ("flip_time", |c| c.flip_time),
        ("air_time", |c| c.air_time),
        ("air_time_since_jump", |c| c.air_time_since_jump),
        ("time_since_boosted", |c| c.time_since_boosted),
        ("boosting_time", |c| c.boosting_time),
        ("supersonic_grace_timer", |c| c.supersonic_grace_timer),
        ("handbrake_value", |c| c.handbrake_value),
        ("auto_flip_timer", |c| c.auto_flip_timer),
        ("auto_flip_torque_scale", |c| c.auto_flip_torque_scale),
        ("bump_cooldown_timer", |c| c.bump_cooldown_timer),
        ("demo_respawn_timer", |c| c.demo_respawn_timer),
    ];
    for (name, value) in timers {
        columns.f32(
            format!("{prefix}_{name}"),
            STATE,
            None,
            rows.iter().map(|r| internals(r).map(|c| value(&c))),
        );
    }
    columns.u32(
        format!("{prefix}_jump_ticks"),
        STATE,
        Some("tick"),
        rows.iter().map(|r| internals(r).map(|c| c.jump_ticks)),
    );
    for (i, axis) in XYZ.iter().enumerate() {
        columns.f32(
            format!("{prefix}_flip_relative_torque_{axis}"),
            STATE,
            None,
            rows.iter()
                .map(|r| internals(r).map(|c| c.flip_relative_torque[i])),
        );
    }
    for (i, axis) in XYZ.iter().enumerate() {
        columns.f32(
            format!("{prefix}_world_contact_normal_{axis}"),
            STATE,
            None,
            rows.iter().map(|r| {
                internals(r)
                    .and_then(|c| c.world_contact_normal)
                    .map(|n| n[i])
            }),
        );
    }
    columns.u64(
        format!("{prefix}_last_extra_hit_tick"),
        STATE,
        Some("tick"),
        rows.iter()
            .map(|r| internals(r).and_then(|c| c.last_extra_hit_tick)),
    );
}

fn game_columns(columns: &mut Columns, rows: &[&Frame], stat_events: &[Vec<&StatEvent>]) {
    columns.names(
        "period",
        GAME,
        rows.iter().map(|r| Some(r.game.period.name())),
    );
    columns.names(
        "clock_phase",
        GAME,
        rows.iter().map(|r| Some(r.game.clock_phase.name())),
    );
    columns.f32(
        "seconds_remaining",
        GAME,
        Some("s"),
        rows.iter().map(|r| r.game.seconds_remaining),
    );
    columns.f32(
        "overtime_seconds",
        GAME,
        Some("s"),
        rows.iter().map(|r| r.game.overtime_seconds),
    );
    columns.i32("blue_score", GAME, rows.iter().map(|r| r.game.scores[0]));
    columns.i32("orange_score", GAME, rows.iter().map(|r| r.game.scores[1]));

    let events: Vec<&Event> = rows.iter().flat_map(|r| &r.game.events).collect();
    let lengths: Vec<usize> = rows.iter().map(|r| r.game.events.len()).collect();
    columns.records(
        "events",
        GAME,
        &lengths,
        vec![
            child_str(
                "kind",
                events
                    .iter()
                    .map(|e| {
                        Some(match e {
                            Event::Goal { .. } => "goal",
                            Event::Demolition { .. } => "demolition",
                            Event::FlipReset { .. } => "flip_reset",
                        })
                    })
                    .collect(),
            ),
            child_u8(
                "scoring_team",
                events
                    .iter()
                    .map(|e| match e {
                        Event::Goal { scoring_team, .. } => Some(*scoring_team),
                        _ => None,
                    })
                    .collect(),
            ),
            child_u8(
                "scorer",
                events
                    .iter()
                    .map(|e| match e {
                        Event::Goal { scorer, .. } => *scorer,
                        _ => None,
                    })
                    .collect(),
            ),
            child_u8(
                "assister",
                events
                    .iter()
                    .map(|e| match e {
                        Event::Goal { assister, .. } => *assister,
                        _ => None,
                    })
                    .collect(),
            ),
            child_u8(
                "attacker",
                events
                    .iter()
                    .map(|e| match e {
                        Event::Demolition { attacker, .. } => *attacker,
                        _ => None,
                    })
                    .collect(),
            ),
            child_u8(
                "victim",
                events
                    .iter()
                    .map(|e| match e {
                        Event::Demolition { victim, .. } => *victim,
                        _ => None,
                    })
                    .collect(),
            ),
            child_bool(
                "repeat",
                events
                    .iter()
                    .map(|e| match e {
                        Event::Demolition { repeat, .. } => Some(*repeat),
                        _ => None,
                    })
                    .collect(),
            ),
            child_bool(
                "goal_explosion",
                events
                    .iter()
                    .map(|e| match e {
                        Event::Demolition { goal_explosion, .. } => Some(*goal_explosion),
                        _ => None,
                    })
                    .collect(),
            ),
            child_u8(
                "player",
                events
                    .iter()
                    .map(|e| match e {
                        Event::FlipReset { player } => *player,
                        _ => None,
                    })
                    .collect(),
            ),
        ],
    );

    let stats: Vec<_> = stat_events.iter().flatten().collect();
    let lengths: Vec<usize> = stat_events.iter().map(Vec::len).collect();
    columns.records(
        "stat_events",
        GAME,
        &lengths,
        vec![
            child_u64(
                "updated_frame",
                stats
                    .iter()
                    .map(|e| Some(u64::from(e.updated_frame)))
                    .collect(),
            ),
            child_str("kind", stats.iter().map(|e| Some(e.kind.name())).collect()),
            child_u8("player", stats.iter().map(|e| Some(e.player)).collect()),
            crate::columns::child_i32("total", stats.iter().map(|e| Some(e.total)).collect()),
        ],
    );

    let contacts: Vec<_> = rows.iter().flat_map(|r| &r.game.ball_contacts).collect();
    let lengths: Vec<usize> = rows.iter().map(|r| r.game.ball_contacts.len()).collect();
    columns.records(
        "ball_contacts",
        GAME,
        &lengths,
        vec![
            child_u64(
                "replay_tick",
                contacts.iter().map(|c| Some(c.replay_tick)).collect(),
            ),
            child_u64(
                "from_tick",
                contacts.iter().map(|c| Some(c.from_tick)).collect(),
            ),
            child_u64(
                "to_tick",
                contacts.iter().map(|c| Some(c.to_tick)).collect(),
            ),
            child_u8("player", contacts.iter().map(|c| c.player).collect()),
            child_f32("gap", contacts.iter().map(|c| c.gap).collect()),
            child_f32(
                "velocity_residual",
                contacts.iter().map(|c| Some(c.velocity_residual)).collect(),
            ),
        ],
    );

    let pickups: Vec<_> = rows.iter().flat_map(|r| &r.game.boost_pickups).collect();
    let lengths: Vec<usize> = rows.iter().map(|r| r.game.boost_pickups.len()).collect();
    columns.records(
        "boost_pickups",
        GAME,
        &lengths,
        vec![
            child_u16("pad", pickups.iter().map(|p| p.pad).collect()),
            child_bool("is_big", pickups.iter().map(|p| p.is_big).collect()),
            child_u8("player", pickups.iter().map(|p| p.player).collect()),
            child_bool(
                "verified",
                pickups.iter().map(|p| Some(p.verified)).collect(),
            ),
            child_u8(
                "suggested_player",
                pickups.iter().map(|p| p.suggested_player).collect(),
            ),
            child_u64(
                "replay_tick",
                pickups.iter().map(|p| Some(p.replay_tick)).collect(),
            ),
        ],
    );
}

fn update_columns(columns: &mut Columns, rows: &[&Frame], players: usize) {
    columns.bool(
        "ball_updated",
        UPDATES,
        rows.iter().map(|r| Some(r.updates.ball_updated)),
    );
    columns.u32(
        "ball_update_tick",
        UPDATES,
        Some("tick"),
        rows.iter().map(|r| r.updates.ball_update_tick),
    );
    columns.u32(
        "ball_ticks_since_update",
        UPDATES,
        Some("ticks"),
        rows.iter().map(|r| r.updates.ball_ticks_since_update),
    );
    columns.f32(
        "ball_seconds_since_update",
        UPDATES,
        Some("s"),
        rows.iter().map(|r| r.updates.ball_seconds_since_update),
    );
    for p in 0..players {
        columns.bool(
            format!("car_{p}_updated"),
            UPDATES,
            rows.iter().map(|r| at(&r.updates.car_updated, p)),
        );
        columns.u32(
            format!("car_{p}_update_tick"),
            UPDATES,
            Some("tick"),
            rows.iter().map(|r| at(&r.updates.car_update_tick, p)),
        );
        columns.u32(
            format!("car_{p}_ticks_since_update"),
            UPDATES,
            Some("ticks"),
            rows.iter()
                .map(|r| at(&r.updates.car_ticks_since_update, p)),
        );
        columns.f32(
            format!("car_{p}_seconds_since_update"),
            UPDATES,
            Some("s"),
            rows.iter()
                .map(|r| at(&r.updates.car_seconds_since_update, p)),
        );
        columns.u8(
            format!("player_{p}_ping_raw"),
            UPDATES,
            None,
            rows.iter().map(|r| at(&r.updates.ping_raw, p)),
        );
    }
}

/// Player `p`'s value of a per-player list; `None` past its end.
fn at<T: Copy>(values: &[Option<T>], p: usize) -> Option<T> {
    values.get(p).copied().flatten()
}

fn future_columns(columns: &mut Columns, rows: &[&Frame]) {
    columns.names(
        "future_segment_end",
        FUTURE,
        rows.iter().map(|r| r.future.map(|f| f.segment_end.name())),
    );
    columns.f32(
        "future_seconds_until_segment_end",
        FUTURE,
        Some("s"),
        rows.iter()
            .map(|r| r.future.map(|f| f.seconds_until_segment_end)),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::header::{PadInfo, PlayerInfo};
    use crate::record::{Ball, Future, Game, State, Updates};
    use crate::{CarStatus, ClockPhase, FrameIndex, Period, SegmentEnd, StatKind};
    use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

    fn header() -> Header {
        Header {
            format_version: crate::header::FORMAT_VERSION,
            replay_sha256: "00".to_owned(),
            replicar_version: "test".to_owned(),
            rocketsim_version: "test".to_owned(),
            groups: Vec::new(),
            precision: "float32".to_owned(),
            all_frames: false,
            players: vec![PlayerInfo {
                index: 0,
                key: "a".to_owned(),
                name: None,
                team: 0,
                body_product_id: None,
                hitbox: "octane".to_owned(),
                final_stats: Default::default(),
            }],
            pads: vec![PadInfo {
                position: [0.0; 3],
                is_big: true,
            }],
            segments: Vec::new(),
            final_scores: [Some(1), Some(0)],
            state_sha256: String::new(),
            configuration: serde_json::Value::Null,
            diagnostics: serde_json::Value::Null,
        }
    }

    /// A frame in segment `segment`, its car absent when `absent`.
    fn frame(index: u32, segment: Option<u32>, absent: bool) -> Frame {
        Frame {
            frame: FrameIndex(index),
            segment,
            replay_time: index as f32 / 30.0,
            replay_tick: index * 4,
            sim_tick: u64::from(index) * 4,
            state: State {
                ball: Ball::default(),
                car_status: vec![if absent {
                    CarStatus::Absent
                } else {
                    CarStatus::Active
                }],
                car_status_inferred: vec![false],
                cars: vec![(!absent).then(Car::default)],
                pad_cooldowns: vec![0.0],
            },
            game: Game {
                period: Period::Regulation,
                clock_phase: ClockPhase::Running,
                seconds_remaining: Some(100.0),
                overtime_seconds: None,
                scores: [Some(0), None],
                events: if index == 2 {
                    vec![Event::Goal {
                        scoring_team: 0,
                        scorer: Some(0),
                        assister: None,
                    }]
                } else {
                    Vec::new()
                },
                stat_events: Vec::new(),
                ball_contacts: Vec::new(),
                boost_pickups: Vec::new(),
            },
            updates: Updates::default(),
            future: segment.map(|_| Future {
                segment_end: SegmentEnd::BlueGoal,
                seconds_until_segment_end: 0.0,
            }),
        }
    }

    fn content(frames: &[Frame]) -> Content<'_> {
        Content {
            frames,
            resimulation: None,
            network: None,
            diagnostics: None,
        }
    }

    fn read(path: &Path) -> (RecordBatch, Header) {
        let builder = ParquetRecordBatchReaderBuilder::try_new(File::open(path).unwrap()).unwrap();
        let header = builder
            .metadata()
            .file_metadata()
            .key_value_metadata()
            .and_then(|kv| kv.iter().find(|kv| kv.key == HEADER_KEY))
            .and_then(|kv| kv.value.clone())
            .unwrap();
        let batch = builder.build().unwrap().next().unwrap().unwrap();
        (batch, serde_json::from_str(&header).unwrap())
    }

    #[test]
    fn a_file_holds_the_play_frames_the_groups_and_the_header() {
        let directory =
            std::env::temp_dir().join(format!("replicar-format-{}", std::process::id()));
        std::fs::create_dir_all(&directory).unwrap();
        let path = directory.join("x.parquet");
        let frames = vec![
            frame(0, None, true),
            frame(1, Some(0), false),
            frame(2, Some(0), false),
        ];
        write(
            &path,
            &header(),
            &content(&frames),
            &WriteOptions::default(),
        )
        .unwrap();
        assert!(!directory.join("x.parquet.partial").exists());
        let (batch, header) = read(&path);
        // The frame outside play is left out; the header lists the groups written.
        assert_eq!(batch.num_rows(), 2);
        assert_eq!(header.groups, ["state", "game", "updates", "future"]);
        assert_eq!(header.final_scores, [Some(1), Some(0)]);
        let schema = batch.schema();
        for name in [
            "car_0_position_x",
            "pad_0_cooldown",
            "events",
            "future_segment_end",
        ] {
            assert!(schema.field_with_name(name).is_ok(), "{name}");
        }
        assert_eq!(
            schema
                .field_with_name("car_0_position_x")
                .unwrap()
                .metadata()["unit"],
            "UU"
        );
        // An unknown value is null, not zero.
        let orange = batch.column_by_name("orange_score").unwrap();
        assert_eq!(orange.null_count(), 2);

        // Every frame, and only the groups asked for.
        let options = WriteOptions {
            groups: [Group::Game].into_iter().collect(),
            all_frames: true,
            ..WriteOptions::default()
        };
        write(&path, &header, &content(&frames), &options).unwrap();
        let (batch, header) = read(&path);
        assert_eq!(batch.num_rows(), 3);
        assert_eq!(header.groups, ["game"]);
        assert!(batch.column_by_name("car_0_position_x").is_none());
        let segment = batch.column_by_name("segment").unwrap();
        assert!(segment.is_null(0) && !segment.is_null(1));
        std::fs::remove_dir_all(&directory).unwrap();
    }

    #[test]
    fn stat_events_of_left_out_frames_go_on_the_row_before() {
        let directory =
            std::env::temp_dir().join(format!("replicar-format-s-{}", std::process::id()));
        std::fs::create_dir_all(&directory).unwrap();
        let path = directory.join("s.parquet");
        let stat = |frame: u32, kind: StatKind| StatEvent {
            updated_frame: frame,
            kind,
            player: 0,
            total: 1,
        };
        // A shot before play, a goal in it, and the assist in the goal pause after it.
        let mut frames = vec![
            frame(0, None, true),
            frame(1, Some(0), false),
            frame(2, Some(0), false),
            frame(3, None, false),
        ];
        frames[0].game.stat_events = vec![stat(0, StatKind::Shot)];
        frames[2].game.stat_events = vec![stat(2, StatKind::Goal)];
        frames[3].game.stat_events = vec![stat(3, StatKind::Assist)];
        write(
            &path,
            &header(),
            &content(&frames),
            &WriteOptions::default(),
        )
        .unwrap();
        let (batch, _) = read(&path);
        let lists = batch
            .column_by_name("stat_events")
            .unwrap()
            .as_any()
            .downcast_ref::<arrow_array::ListArray>()
            .unwrap()
            .clone();
        let updated = |row: usize| -> Vec<u64> {
            let list = lists.value(row);
            let records = list
                .as_any()
                .downcast_ref::<arrow_array::StructArray>()
                .unwrap();
            let frames = records
                .column_by_name("updated_frame")
                .unwrap()
                .as_any()
                .downcast_ref::<arrow_array::UInt64Array>()
                .unwrap();
            frames.values().to_vec()
        };
        assert_eq!(updated(0), [0]);
        assert_eq!(updated(1), [2, 3]);
        std::fs::remove_dir_all(&directory).unwrap();
    }

    #[test]
    fn quantized_bodies_are_integers_with_their_scale() {
        let directory =
            std::env::temp_dir().join(format!("replicar-format-q-{}", std::process::id()));
        std::fs::create_dir_all(&directory).unwrap();
        let path = directory.join("q.parquet");
        let mut row = frame(1, Some(0), false);
        row.state.ball.body.position = [1.234_56, -2.0, 92.75];
        row.state.ball.body.rotation = [0.0, 0.0, 0.707_106_77, 0.707_106_77];
        let options = WriteOptions {
            precision: Precision::Quantized,
            ..WriteOptions::default()
        };
        write(&path, &header(), &content(&[row]), &options).unwrap();
        let (batch, header) = read(&path);
        assert_eq!(header.precision, "quantized");
        let schema = batch.schema();
        let field = schema.field_with_name("ball_position_x").unwrap();
        assert_eq!(field.data_type(), &DataType::Int32);
        assert_eq!(field.metadata()["scale"].parse::<f64>().unwrap(), 0.01);
        let x = batch.column_by_name("ball_position_x").unwrap();
        let x = x
            .as_any()
            .downcast_ref::<arrow_array::Int32Array>()
            .unwrap();
        assert_eq!(x.value(0), 123);
        let w = batch.column_by_name("ball_rotation_w").unwrap();
        assert_eq!(w.data_type(), &DataType::Int16);
        let w = w
            .as_any()
            .downcast_ref::<arrow_array::Int16Array>()
            .unwrap();
        assert_eq!(w.value(0), 23170);
        // Everything else keeps its type.
        assert_eq!(
            schema.field_with_name("car_0_boost").unwrap().data_type(),
            &DataType::Float32
        );
        std::fs::remove_dir_all(&directory).unwrap();
    }
}
