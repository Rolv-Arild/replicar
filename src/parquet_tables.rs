//! Record tables written next to the main Parquet file: one row per touch, ball contact, boost
//! pickup, fitted input, packet lag, event (goal or demolition) and pad pickup, each with the
//! `frame` it belongs to. The main file has one row per frame with fixed-width lists, which cannot
//! hold a variable number of records, so these live in separate typed files. A null is an unknown or
//! not-applicable value, never zero.

use std::collections::HashMap;
use std::error::Error;
use std::fs::File;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use arrow_array::builder::{FixedSizeListBuilder, Float32Builder, StringDictionaryBuilder};
use arrow_array::types::UInt8Type;
use arrow_array::{
    ArrayRef, BooleanArray, Float32Array, Int32Array, RecordBatch, StringArray, UInt8Array,
    UInt32Array, UInt64Array,
};
use arrow_schema::{DataType, Field, Schema, SchemaRef};
use parquet::arrow::ArrowWriter;
use parquet::file::properties::WriterProperties;

use crate::conversion::{
    AppliedPacketLag, BallContact, BoostPickup, ConvertedFrame, FittedInput, TouchEvent,
};
use crate::observations::{self, Event, PadPickup};

/// Rows buffered per table before a row group is written (also the main file's batch size).
pub(crate) const BATCH_SIZE: usize = 512;

pub(crate) fn dict_type() -> DataType {
    DataType::Dictionary(Box::new(DataType::UInt8), Box::new(DataType::Utf8))
}

/// A dictionary-encoded string column (`None` is null). The dictionary is built per batch, so the
/// strings, not the integer keys, are the contract.
pub(crate) fn dict<'a>(values: impl Iterator<Item = Option<&'a str>>) -> io::Result<ArrayRef> {
    let mut builder = StringDictionaryBuilder::<UInt8Type>::new();
    for value in values {
        match value {
            Some(text) => {
                builder.append(text).map_err(io::Error::other)?;
            }
            None => builder.append_null(),
        }
    }
    Ok(Arc::new(builder.finish()))
}

fn vec3_type() -> DataType {
    DataType::FixedSizeList(Arc::new(Field::new("item", DataType::Float32, true)), 3)
}

/// A non-null `FixedSizeList<Float32, 3>` column. Nullable fixed-size lists are avoided: Parquet
/// stores a null list without its child values and PyArrow cannot read that back.
fn vec3(values: impl Iterator<Item = [f32; 3]>) -> ArrayRef {
    let mut builder = FixedSizeListBuilder::new(Float32Builder::new(), 3);
    for v in values {
        builder.values().append_slice(&v);
        builder.append(true);
    }
    Arc::new(builder.finish())
}

fn slot(value: Option<usize>) -> io::Result<Option<u32>> {
    value
        .map(|v| u32::try_from(v).map_err(io::Error::other))
        .transpose()
}

fn slots<T>(rows: &[(u32, T)], get: impl Fn(&T) -> Option<usize>) -> io::Result<ArrayRef> {
    Ok(Arc::new(UInt32Array::from(
        rows.iter()
            .map(|r| slot(get(&r.1)))
            .collect::<io::Result<Vec<_>>>()?,
    )))
}

fn frames<T>(rows: &[(u32, T)]) -> ArrayRef {
    Arc::new(UInt32Array::from_iter_values(rows.iter().map(|r| r.0)))
}

fn nullable(name: &str, dtype: DataType) -> Field {
    Field::new(name, dtype, true)
}

fn required(name: &str, dtype: DataType) -> Field {
    Field::new(name, dtype, false)
}

/// Names of the record tables written beside the main file, in a fixed order.
pub const TABLE_NAMES: [&str; 7] = [
    "touches",
    "ball_contacts",
    "boost_pickups",
    "fitted_inputs",
    "packet_lags",
    "events",
    "pad_pickups",
];

/// `<dir>/<stem>.<table>.parquet` beside the main file `<dir>/<stem>.parquet`.
pub fn table_path(main: &Path, table: &str) -> PathBuf {
    let stem = main
        .file_stem()
        .map_or_else(|| "frames".into(), |s| s.to_string_lossy().into_owned());
    main.with_file_name(format!("{stem}.{table}.parquet"))
}

type Rows<'a, T> = &'a [(u32, T)];

pub(crate) fn touches_fields() -> Vec<Field> {
    vec![
        required("frame", DataType::UInt32),
        required("car_slot", DataType::UInt32),
        required("tick", DataType::UInt64),
        required("contact_point", vec3_type()),
    ]
}

pub(crate) fn touches_batch(rows: Rows<TouchEvent>, schema: &SchemaRef) -> io::Result<RecordBatch> {
    let columns: Vec<ArrayRef> = vec![
        frames(rows),
        slots(rows, |v| Some(v.car_slot))?,
        Arc::new(UInt64Array::from_iter_values(rows.iter().map(|r| r.1.tick))),
        vec3(rows.iter().map(|r| r.1.contact_point)),
    ];
    RecordBatch::try_new(schema.clone(), columns).map_err(io::Error::other)
}

pub(crate) fn ball_contacts_fields() -> Vec<Field> {
    vec![
        required("frame", DataType::UInt32),
        required("frame_a", DataType::UInt32),
        required("tick", DataType::UInt64),
        required("tick_from", DataType::UInt64),
        required("tick_to", DataType::UInt64),
        nullable("car_slot", DataType::UInt32),
        nullable("gap_uu", DataType::Float32),
        required("velocity_residual", DataType::Float32),
        required("simulated_touch", DataType::Boolean),
    ]
}

pub(crate) fn ball_contacts_batch(
    rows: Rows<BallContact>,
    schema: &SchemaRef,
) -> io::Result<RecordBatch> {
    let columns: Vec<ArrayRef> = vec![
        frames(rows),
        slots(rows, |v| Some(v.frame_a))?,
        Arc::new(UInt64Array::from_iter_values(rows.iter().map(|r| r.1.tick))),
        Arc::new(UInt64Array::from_iter_values(
            rows.iter().map(|r| r.1.tick_from),
        )),
        Arc::new(UInt64Array::from_iter_values(
            rows.iter().map(|r| r.1.tick_to),
        )),
        slots(rows, |v| v.car_slot)?,
        Arc::new(Float32Array::from(
            rows.iter().map(|r| r.1.gap_uu).collect::<Vec<_>>(),
        )),
        Arc::new(Float32Array::from_iter_values(
            rows.iter().map(|r| r.1.velocity_residual),
        )),
        Arc::new(BooleanArray::from(
            rows.iter().map(|r| r.1.simulated_touch).collect::<Vec<_>>(),
        )),
    ];
    RecordBatch::try_new(schema.clone(), columns).map_err(io::Error::other)
}

pub(crate) fn boost_pickups_fields() -> Vec<Field> {
    vec![
        required("frame", DataType::UInt32),
        nullable("pad_index", DataType::UInt32),
        required("pad_actor_id", DataType::Int32),
        nullable("is_big", DataType::Boolean),
        nullable("car_slot", DataType::UInt32),
        required("verified", DataType::Boolean),
        nullable("distance_uu", DataType::Float32),
        nullable("suggested_car_slot", DataType::UInt32),
        required("tick", DataType::UInt64),
    ]
}

pub(crate) fn boost_pickups_batch(
    rows: Rows<BoostPickup>,
    schema: &SchemaRef,
) -> io::Result<RecordBatch> {
    let columns: Vec<ArrayRef> = vec![
        frames(rows),
        slots(rows, |v| v.pad_index)?,
        Arc::new(Int32Array::from_iter_values(
            rows.iter().map(|r| r.1.pad_actor_id),
        )),
        Arc::new(BooleanArray::from(
            rows.iter().map(|r| r.1.is_big).collect::<Vec<_>>(),
        )),
        slots(rows, |v| v.car_slot)?,
        Arc::new(BooleanArray::from(
            rows.iter().map(|r| r.1.verified).collect::<Vec<_>>(),
        )),
        Arc::new(Float32Array::from(
            rows.iter().map(|r| r.1.distance_uu).collect::<Vec<_>>(),
        )),
        slots(rows, |v| v.suggested_car_slot)?,
        Arc::new(UInt64Array::from_iter_values(rows.iter().map(|r| r.1.tick))),
    ];
    RecordBatch::try_new(schema.clone(), columns).map_err(io::Error::other)
}

pub(crate) fn fitted_inputs_fields() -> Vec<Field> {
    vec![
        required("frame", DataType::UInt32),
        required("slot", DataType::UInt32),
        required("kind", dict_type()),
        required("tick", DataType::UInt64),
        nullable("activation_frame", DataType::UInt32),
        nullable("pitch", DataType::Float32),
        nullable("yaw", DataType::Float32),
        nullable("cancel", DataType::Float32),
    ]
}

pub(crate) fn fitted_inputs_batch(
    rows: Rows<FittedInput>,
    schema: &SchemaRef,
) -> io::Result<RecordBatch> {
    // The dodge-only fields are null for a jump (they do not apply; the JSON record carries 0).
    let dodge = |v: &FittedInput| v.kind == "dodge";
    let columns: Vec<ArrayRef> = vec![
        frames(rows),
        slots(rows, |v| Some(v.slot))?,
        dict(rows.iter().map(|r| Some(r.1.kind)))?,
        Arc::new(UInt64Array::from_iter_values(rows.iter().map(|r| r.1.tick))),
        slots(rows, |v| dodge(v).then_some(v.activation_frame))?,
        Arc::new(Float32Array::from(
            rows.iter()
                .map(|r| dodge(&r.1).then_some(r.1.pitch))
                .collect::<Vec<_>>(),
        )),
        Arc::new(Float32Array::from(
            rows.iter()
                .map(|r| dodge(&r.1).then_some(r.1.yaw))
                .collect::<Vec<_>>(),
        )),
        Arc::new(Float32Array::from(
            rows.iter()
                .map(|r| dodge(&r.1).then_some(r.1.cancel))
                .collect::<Vec<_>>(),
        )),
    ];
    RecordBatch::try_new(schema.clone(), columns).map_err(io::Error::other)
}

pub(crate) fn packet_lags_fields() -> Vec<Field> {
    vec![
        required("frame", DataType::UInt32),
        nullable("actor_id", DataType::Int32),
        required("ticks", DataType::UInt64),
        required("source", dict_type()),
    ]
}

pub(crate) fn packet_lags_batch(
    rows: Rows<AppliedPacketLag>,
    schema: &SchemaRef,
) -> io::Result<RecordBatch> {
    let columns: Vec<ArrayRef> = vec![
        frames(rows),
        Arc::new(Int32Array::from(
            rows.iter().map(|r| r.1.actor_id).collect::<Vec<_>>(),
        )),
        Arc::new(UInt64Array::from_iter_values(
            rows.iter().map(|r| r.1.ticks),
        )),
        dict(rows.iter().map(|r| Some(r.1.source)))?,
    ];
    RecordBatch::try_new(schema.clone(), columns).map_err(io::Error::other)
}

pub(crate) fn events_fields() -> Vec<Field> {
    vec![
        required("frame", DataType::UInt32),
        required("kind", dict_type()),
        nullable("team", DataType::UInt8),
        nullable("source", dict_type()),
        nullable("attacker_car", DataType::Int32),
        nullable("victim_car", DataType::Int32),
        nullable("attacker_pri", DataType::Int32),
        nullable("self_demolish", DataType::Boolean),
        nullable("attacker_velocity_x", DataType::Float32),
        nullable("attacker_velocity_y", DataType::Float32),
        nullable("attacker_velocity_z", DataType::Float32),
        nullable("victim_velocity_x", DataType::Float32),
        nullable("victim_velocity_y", DataType::Float32),
        nullable("victim_velocity_z", DataType::Float32),
        nullable("repeat", DataType::Boolean),
        // `dodge_refreshed` only: the replay car actor id and its new DodgesRefreshedCounter total.
        nullable("car", DataType::Int32),
        nullable("refreshed_count", DataType::Int32),
    ]
}

pub(crate) fn events_batch(rows: Rows<Event>, schema: &SchemaRef) -> io::Result<RecordBatch> {
    // A goal row has no demolition fields and a demolition row has no `team`: those are null.
    let columns: Vec<ArrayRef> = vec![
        frames(rows),
        dict(rows.iter().map(|r| {
            Some(match r.1 {
                Event::GoalScoredOn { .. } => "goal_scored_on",
                Event::Demolish { .. } => "demolish",
                Event::DodgeRefreshed { .. } => "dodge_refreshed",
            })
        }))?,
        Arc::new(UInt8Array::from(
            rows.iter()
                .map(|r| match r.1 {
                    Event::GoalScoredOn { team } => Some(team),
                    Event::Demolish { .. } | Event::DodgeRefreshed { .. } => None,
                })
                .collect::<Vec<_>>(),
        )),
        dict(rows.iter().map(|r| match &r.1 {
            Event::Demolish { source, .. } => Some(*source),
            Event::GoalScoredOn { .. } | Event::DodgeRefreshed { .. } => None,
        }))?,
        Arc::new(Int32Array::from(
            rows.iter()
                .map(|r| match &r.1 {
                    Event::Demolish { attacker_car, .. } => *attacker_car,
                    Event::GoalScoredOn { .. } | Event::DodgeRefreshed { .. } => None,
                })
                .collect::<Vec<_>>(),
        )),
        Arc::new(Int32Array::from(
            rows.iter()
                .map(|r| match &r.1 {
                    Event::Demolish { victim_car, .. } => *victim_car,
                    Event::GoalScoredOn { .. } | Event::DodgeRefreshed { .. } => None,
                })
                .collect::<Vec<_>>(),
        )),
        Arc::new(Int32Array::from(
            rows.iter()
                .map(|r| match &r.1 {
                    Event::Demolish { attacker_pri, .. } => *attacker_pri,
                    Event::GoalScoredOn { .. } | Event::DodgeRefreshed { .. } => None,
                })
                .collect::<Vec<_>>(),
        )),
        Arc::new(BooleanArray::from(
            rows.iter()
                .map(|r| match &r.1 {
                    Event::Demolish { self_demolish, .. } => Some(*self_demolish),
                    Event::GoalScoredOn { .. } | Event::DodgeRefreshed { .. } => None,
                })
                .collect::<Vec<_>>(),
        )),
        Arc::new(Float32Array::from(
            rows.iter()
                .map(|r| match &r.1 {
                    Event::Demolish {
                        attacker_velocity, ..
                    } => Some(attacker_velocity[0]),
                    Event::GoalScoredOn { .. } | Event::DodgeRefreshed { .. } => None,
                })
                .collect::<Vec<_>>(),
        )),
        Arc::new(Float32Array::from(
            rows.iter()
                .map(|r| match &r.1 {
                    Event::Demolish {
                        attacker_velocity, ..
                    } => Some(attacker_velocity[1]),
                    Event::GoalScoredOn { .. } | Event::DodgeRefreshed { .. } => None,
                })
                .collect::<Vec<_>>(),
        )),
        Arc::new(Float32Array::from(
            rows.iter()
                .map(|r| match &r.1 {
                    Event::Demolish {
                        attacker_velocity, ..
                    } => Some(attacker_velocity[2]),
                    Event::GoalScoredOn { .. } | Event::DodgeRefreshed { .. } => None,
                })
                .collect::<Vec<_>>(),
        )),
        Arc::new(Float32Array::from(
            rows.iter()
                .map(|r| match &r.1 {
                    Event::Demolish {
                        victim_velocity, ..
                    } => Some(victim_velocity[0]),
                    Event::GoalScoredOn { .. } | Event::DodgeRefreshed { .. } => None,
                })
                .collect::<Vec<_>>(),
        )),
        Arc::new(Float32Array::from(
            rows.iter()
                .map(|r| match &r.1 {
                    Event::Demolish {
                        victim_velocity, ..
                    } => Some(victim_velocity[1]),
                    Event::GoalScoredOn { .. } | Event::DodgeRefreshed { .. } => None,
                })
                .collect::<Vec<_>>(),
        )),
        Arc::new(Float32Array::from(
            rows.iter()
                .map(|r| match &r.1 {
                    Event::Demolish {
                        victim_velocity, ..
                    } => Some(victim_velocity[2]),
                    Event::GoalScoredOn { .. } | Event::DodgeRefreshed { .. } => None,
                })
                .collect::<Vec<_>>(),
        )),
        Arc::new(BooleanArray::from(
            rows.iter()
                .map(|r| match &r.1 {
                    Event::Demolish { repeat, .. } => Some(*repeat),
                    Event::GoalScoredOn { .. } | Event::DodgeRefreshed { .. } => None,
                })
                .collect::<Vec<_>>(),
        )),
        Arc::new(Int32Array::from(
            rows.iter()
                .map(|r| match &r.1 {
                    Event::DodgeRefreshed { car, .. } => Some(*car),
                    _ => None,
                })
                .collect::<Vec<_>>(),
        )),
        Arc::new(Int32Array::from(
            rows.iter()
                .map(|r| match &r.1 {
                    Event::DodgeRefreshed { count, .. } => Some(*count),
                    _ => None,
                })
                .collect::<Vec<_>>(),
        )),
    ];
    RecordBatch::try_new(schema.clone(), columns).map_err(io::Error::other)
}

pub(crate) fn pad_pickups_fields() -> Vec<Field> {
    vec![
        required("frame", DataType::UInt32),
        required("pad_actor_id", DataType::Int32),
        nullable("pad_actor_name", DataType::Utf8),
        nullable("instigator_car_id", DataType::Int32),
        required("picked_up", DataType::UInt8),
        required("repeat", DataType::Boolean),
    ]
}

pub(crate) fn pad_pickups_batch(
    rows: Rows<PadPickup>,
    schema: &SchemaRef,
) -> io::Result<RecordBatch> {
    let columns: Vec<ArrayRef> = vec![
        frames(rows),
        Arc::new(Int32Array::from_iter_values(
            rows.iter().map(|r| r.1.pad_actor_id),
        )),
        Arc::new(StringArray::from(
            rows.iter()
                .map(|r| r.1.pad_actor_name.as_deref())
                .collect::<Vec<_>>(),
        )),
        Arc::new(Int32Array::from(
            rows.iter()
                .map(|r| r.1.instigator_car_id)
                .collect::<Vec<_>>(),
        )),
        Arc::new(UInt8Array::from_iter_values(
            rows.iter().map(|r| r.1.picked_up),
        )),
        Arc::new(BooleanArray::from(
            rows.iter().map(|r| r.1.repeat).collect::<Vec<_>>(),
        )),
    ];
    RecordBatch::try_new(schema.clone(), columns).map_err(io::Error::other)
}

type Build<T> = fn(Rows<T>, &SchemaRef) -> io::Result<RecordBatch>;

/// One record table: buffers at most `BATCH_SIZE` rows, then writes a row group.
struct Sink<T> {
    schema: SchemaRef,
    writer: ArrowWriter<File>,
    rows: Vec<(u32, T)>,
    build: Build<T>,
    total: usize,
}

impl<T> Sink<T> {
    fn create(
        main: &Path,
        name: &str,
        fields: Vec<Field>,
        build: Build<T>,
        properties: &WriterProperties,
    ) -> Result<Self, Box<dyn Error>> {
        let metadata = HashMap::from([
            ("columnar_version".to_owned(), "1".to_owned()),
            ("table".to_owned(), name.to_owned()),
        ]);
        let schema = Arc::new(Schema::new_with_metadata(fields, metadata));
        let file = File::create(table_path(main, name))?;
        Ok(Self {
            writer: ArrowWriter::try_new(file, schema.clone(), Some(properties.clone()))?,
            schema,
            rows: Vec::new(),
            build,
            total: 0,
        })
    }

    fn push(&mut self, frame: u32, value: T) -> io::Result<()> {
        self.rows.push((frame, value));
        self.total += 1;
        if self.rows.len() >= BATCH_SIZE {
            self.flush()?;
        }
        Ok(())
    }

    fn flush(&mut self) -> io::Result<()> {
        if !self.rows.is_empty() {
            let batch = (self.build)(&self.rows, &self.schema)?;
            self.writer.write(&batch).map_err(io::Error::other)?;
            self.rows.clear();
        }
        Ok(())
    }

    fn finish(mut self) -> io::Result<usize> {
        self.flush()?;
        self.writer.close().map_err(io::Error::other)?;
        Ok(self.total)
    }
}

/// The record tables of one export.
pub(crate) struct Tables {
    touches: Sink<TouchEvent>,
    ball_contacts: Sink<BallContact>,
    boost_pickups: Sink<BoostPickup>,
    fitted_inputs: Sink<FittedInput>,
    packet_lags: Sink<AppliedPacketLag>,
    events: Sink<Event>,
    pad_pickups: Sink<PadPickup>,
}

impl Tables {
    pub(crate) fn create(
        main: &Path,
        properties: &WriterProperties,
    ) -> Result<Self, Box<dyn Error>> {
        Ok(Self {
            touches: Sink::create(main, "touches", touches_fields(), touches_batch, properties)?,
            ball_contacts: Sink::create(
                main,
                "ball_contacts",
                ball_contacts_fields(),
                ball_contacts_batch,
                properties,
            )?,
            boost_pickups: Sink::create(
                main,
                "boost_pickups",
                boost_pickups_fields(),
                boost_pickups_batch,
                properties,
            )?,
            fitted_inputs: Sink::create(
                main,
                "fitted_inputs",
                fitted_inputs_fields(),
                fitted_inputs_batch,
                properties,
            )?,
            packet_lags: Sink::create(
                main,
                "packet_lags",
                packet_lags_fields(),
                packet_lags_batch,
                properties,
            )?,
            events: Sink::create(main, "events", events_fields(), events_batch, properties)?,
            pad_pickups: Sink::create(
                main,
                "pad_pickups",
                pad_pickups_fields(),
                pad_pickups_batch,
                properties,
            )?,
        })
    }

    pub(crate) fn add_frame(
        &mut self,
        converted: &ConvertedFrame,
        observed: &observations::Frame,
    ) -> io::Result<()> {
        let frame = u32::try_from(converted.replay_frame).map_err(io::Error::other)?;
        for v in &converted.touches {
            self.touches.push(frame, v.clone())?;
        }
        for v in &converted.ball_contacts {
            self.ball_contacts.push(frame, v.clone())?;
        }
        for v in &converted.boost_pickups {
            self.boost_pickups.push(frame, v.clone())?;
        }
        for v in &converted.fitted_inputs {
            self.fitted_inputs.push(frame, v.clone())?;
        }
        for v in &converted.packet_lags {
            self.packet_lags.push(frame, v.clone())?;
        }
        for v in &observed.events {
            self.events.push(frame, v.clone())?;
        }
        for v in &observed.pad_pickups {
            self.pad_pickups.push(frame, v.clone())?;
        }
        Ok(())
    }

    /// Close every file; returns the row count of each table in `TABLE_NAMES` order.
    pub(crate) fn finish(self) -> io::Result<Vec<(&'static str, usize)>> {
        Ok(vec![
            (TABLE_NAMES[0], self.touches.finish()?),
            (TABLE_NAMES[1], self.ball_contacts.finish()?),
            (TABLE_NAMES[2], self.boost_pickups.finish()?),
            (TABLE_NAMES[3], self.fitted_inputs.finish()?),
            (TABLE_NAMES[4], self.packet_lags.finish()?),
            (TABLE_NAMES[5], self.events.finish()?),
            (TABLE_NAMES[6], self.pad_pickups.finish()?),
        ])
    }
}

/// Frames written and the rows of each record table (empty when tables were not requested).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExportSummary {
    pub frames: usize,
    pub table_rows: Vec<(&'static str, usize)>,
}

#[cfg(test)]
mod tests {
    use std::fs;

    use arrow_array::cast::AsArray;
    use arrow_array::types::{Float32Type, Int32Type, UInt32Type};
    use arrow_array::{Array, DictionaryArray};
    use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

    use super::*;

    fn properties() -> WriterProperties {
        WriterProperties::builder()
            .set_max_row_group_row_count(Some(BATCH_SIZE))
            .build()
    }

    /// A fresh output path in the system temp directory, unique to this process and test.
    fn main_path(test: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("replay-tables-{}-{test}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        dir.join("game.parquet")
    }

    fn read(path: &Path) -> (SchemaRef, usize, Vec<RecordBatch>) {
        let builder = ParquetRecordBatchReaderBuilder::try_new(File::open(path).unwrap()).unwrap();
        let schema = builder.schema().clone();
        let groups = builder.metadata().num_row_groups();
        let batches = builder
            .build()
            .unwrap()
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        (schema, groups, batches)
    }

    fn strings(array: &ArrayRef) -> Vec<Option<String>> {
        let dictionary = array
            .as_any()
            .downcast_ref::<DictionaryArray<UInt8Type>>()
            .expect("dictionary column");
        let values = dictionary.values().as_string::<i32>();
        (0..dictionary.len())
            .map(|i| dictionary.key(i).map(|key| values.value(key).to_owned()))
            .collect()
    }

    #[test]
    fn table_files_sit_beside_the_main_file() {
        assert_eq!(
            table_path(Path::new("out/game.parquet"), "touches"),
            Path::new("out").join("game.touches.parquet")
        );
        assert_eq!(
            table_path(Path::new("game.parquet"), "events"),
            Path::new("game.events.parquet")
        );
    }

    #[test]
    fn goal_and_demolish_events_keep_inapplicable_fields_null() {
        let main = main_path("events");
        let mut sink = Sink::create(
            &main,
            "events",
            events_fields(),
            events_batch,
            &properties(),
        )
        .unwrap();
        sink.push(7, Event::GoalScoredOn { team: 1 }).unwrap();
        sink.push(
            9,
            Event::Demolish {
                source: "extended",
                attacker_car: Some(4),
                victim_car: None,
                attacker_pri: Some(2),
                self_demolish: false,
                attacker_velocity: [1.0, 2.0, 3.0],
                victim_velocity: [4.0, 5.0, 6.0],
                repeat: true,
            },
        )
        .unwrap();
        assert_eq!(sink.finish().unwrap(), 2);
        let (schema, _, batches) = read(&table_path(&main, "events"));
        assert_eq!(schema.metadata()["table"], "events");
        assert_eq!(batches.len(), 1);
        let b = &batches[0];
        assert_eq!(b.column(0).as_primitive::<UInt32Type>().values(), &[7, 9]);
        assert_eq!(
            strings(b.column(1)),
            [Some("goal_scored_on".into()), Some("demolish".into())]
        );
        let team = b.column(2).as_primitive::<UInt8Type>();
        assert!(team.is_valid(0) && team.value(0) == 1 && team.is_null(1));
        assert_eq!(strings(b.column(3)), [None, Some("extended".into())]);
        let attacker = b.column(4).as_primitive::<Int32Type>();
        assert!(attacker.is_null(0) && attacker.value(1) == 4);
        // A demolition without a victim actor is unknown, not actor 0.
        assert!(b.column(5).is_null(1));
        assert!(b.column(7).is_null(0)); // self_demolish
        let vx = b.column(8).as_primitive::<Float32Type>();
        assert!(vx.is_null(0) && vx.value(1) == 1.0);
        let repeat = b.column(14).as_boolean();
        assert!(repeat.is_null(0) && repeat.value(1));
        fs::remove_dir_all(main.parent().unwrap()).ok();
    }

    #[test]
    fn a_jump_has_no_dodge_fields() {
        let main = main_path("fitted");
        let mut sink = Sink::create(
            &main,
            "fitted_inputs",
            fitted_inputs_fields(),
            fitted_inputs_batch,
            &properties(),
        )
        .unwrap();
        let input = |kind, activation_frame| FittedInput {
            slot: 1,
            activation_frame,
            kind,
            tick: 100,
            pitch: -0.5,
            yaw: 0.25,
            cancel: 0.0,
        };
        sink.push(3, input("jump", 0)).unwrap();
        sink.push(3, input("dodge", 5)).unwrap();
        sink.finish().unwrap();
        let (_, _, batches) = read(&table_path(&main, "fitted_inputs"));
        let b = &batches[0];
        assert_eq!(
            strings(b.column(2)),
            [Some("jump".into()), Some("dodge".into())]
        );
        assert!(b.column(4).is_null(0) && !b.column(4).is_null(1));
        let pitch = b.column(5).as_primitive::<Float32Type>();
        assert!(pitch.is_null(0) && pitch.value(1) == -0.5);
        let cancel = b.column(7).as_primitive::<Float32Type>();
        assert!(cancel.is_null(0) && cancel.value(1) == 0.0);
        fs::remove_dir_all(main.parent().unwrap()).ok();
    }

    #[test]
    fn optional_contact_and_pickup_fields_stay_null() {
        let main = main_path("optional");
        let mut contacts = Sink::create(
            &main,
            "ball_contacts",
            ball_contacts_fields(),
            ball_contacts_batch,
            &properties(),
        )
        .unwrap();
        contacts
            .push(
                12,
                BallContact {
                    frame_a: 11,
                    tick: 900,
                    tick_from: 898,
                    tick_to: 902,
                    car_slot: None,
                    gap_uu: None,
                    velocity_residual: 350.0,
                    simulated_touch: false,
                },
            )
            .unwrap();
        contacts.finish().unwrap();
        let (_, _, batches) = read(&table_path(&main, "ball_contacts"));
        let b = &batches[0];
        assert!(b.column(5).is_null(0) && b.column(6).is_null(0));
        assert_eq!(b.column(1).as_primitive::<UInt32Type>().value(0), 11);

        let mut pickups = Sink::create(
            &main,
            "boost_pickups",
            boost_pickups_fields(),
            boost_pickups_batch,
            &properties(),
        )
        .unwrap();
        pickups
            .push(
                13,
                BoostPickup {
                    pad_index: None,
                    pad_actor_id: 26,
                    is_big: None,
                    car_slot: Some(0),
                    verified: false,
                    distance_uu: None,
                    suggested_car_slot: Some(1),
                    tick: 950,
                },
            )
            .unwrap();
        pickups.finish().unwrap();
        let (_, _, batches) = read(&table_path(&main, "boost_pickups"));
        let b = &batches[0];
        assert!(b.column(1).is_null(0) && b.column(3).is_null(0) && b.column(6).is_null(0));
        assert!(!b.column(4).is_null(0) && !b.column(7).is_null(0));
        assert!(!b.column(5).as_boolean().value(0));
        fs::remove_dir_all(main.parent().unwrap()).ok();
    }

    #[test]
    fn large_tables_are_written_in_bounded_row_groups_and_empty_ones_keep_their_schema() {
        let main = main_path("groups");
        let mut sink = Sink::create(
            &main,
            "touches",
            touches_fields(),
            touches_batch,
            &properties(),
        )
        .unwrap();
        let count = 2 * BATCH_SIZE + 100;
        for i in 0..count {
            sink.push(
                i as u32,
                TouchEvent {
                    car_slot: i % 2,
                    tick: i as u64,
                    contact_point: [i as f32, 0.0, 1.0],
                },
            )
            .unwrap();
            assert!(sink.rows.len() <= BATCH_SIZE);
        }
        assert_eq!(sink.finish().unwrap(), count);
        let (schema, groups, batches) = read(&table_path(&main, "touches"));
        assert_eq!(groups, 3);
        assert_eq!(
            batches.iter().map(RecordBatch::num_rows).sum::<usize>(),
            count
        );
        assert_eq!(
            schema.field_with_name("contact_point").unwrap().data_type(),
            &vec3_type()
        );

        let empty = Sink::create(
            &main,
            "pad_pickups",
            pad_pickups_fields(),
            pad_pickups_batch,
            &properties(),
        )
        .unwrap();
        assert_eq!(empty.finish().unwrap(), 0);
        let (schema, groups, batches) = read(&table_path(&main, "pad_pickups"));
        assert_eq!((groups, batches.len()), (0, 0));
        assert_eq!(
            schema
                .fields()
                .iter()
                .map(|f| f.name().as_str())
                .collect::<Vec<_>>(),
            [
                "frame",
                "pad_actor_id",
                "pad_actor_name",
                "instigator_car_id",
                "picked_up",
                "repeat"
            ]
        );
        fs::remove_dir_all(main.parent().unwrap()).ok();
    }
}
