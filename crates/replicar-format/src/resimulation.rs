//! The `resimulation` group (docs/glossary.md, "Resimulation group"): what the fitted inference chose and
//! the update ticks, so that the replay can be simulated again without fitting.
//!
//! Its entries belong to replay frames, also frames the file has no row for (the simulation runs in frames
//! outside play segments too): each entry carries its frame and is stored in the row of the first written
//! frame at or after it, or in the last row. The entries are three lists of records per row.

use std::sync::Arc;

use arrow_array::{
    Array, ArrayRef, BooleanArray, Float32Array, Int64Array, ListArray, RecordBatch, StringArray,
    StructArray, UInt8Array, UInt32Array, UInt64Array,
};
use arrow_buffer::OffsetBuffer;
use arrow_schema::{DataType, Field, Fields};

use crate::columns::{Columns, child_bool, child_f32, child_u8, child_u64};
use crate::read::ReadError;

/// The update ticks of a frame: how many ticks before the frame the ball's update and the frame's median car
/// update are.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameTicks {
    pub frame: u32,
    pub ball: Option<u32>,
    pub car_median: Option<u32>,
}

/// The update ticks of one car's update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CarTicks {
    pub frame: u32,
    /// The car life: actor id and creation frame.
    pub actor: i32,
    pub created: u32,
    pub ticks: u32,
}

/// One tick of a schedule: from `tick` on, these controls (`None`: unchanged).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ScheduleEntry {
    pub tick: u64,
    pub throttle: Option<f32>,
    pub steer: Option<f32>,
    pub pitch: Option<f32>,
    pub yaw: Option<f32>,
    pub roll: Option<f32>,
    pub jump: Option<bool>,
    pub boost: Option<bool>,
    pub handbrake: Option<bool>,
}

/// A planned dodge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Dodge {
    pub activation_frame: u32,
    pub start_offset: u64,
    pub duration: u64,
    pub pitch: f32,
    pub yaw: f32,
    pub cancel: f32,
    /// The first update after the activation and its ticks before its frame.
    pub first_update: Option<(u32, u64)>,
}

/// One answer of the inference: the question about a car life, how many times it was asked about that car
/// life before, and the answer's values. Which fields a question uses is up to the converter.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Choice {
    pub frame: u32,
    pub actor: i32,
    pub created: u32,
    pub question: String,
    pub ordinal: u32,
    /// The same answer to the next `repeat` askings too (a value kept for a car life, such as its control
    /// shift, is asked at every update).
    pub repeat: u32,
    pub values: [Option<f32>; 3],
    pub integer: Option<i64>,
    pub player: Option<u8>,
    pub end_tick: Option<u64>,
    pub entries: Vec<ScheduleEntry>,
    pub dodge: Option<Dodge>,
}

/// The whole group.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Resimulation {
    pub ticks: Vec<FrameTicks>,
    pub car_ticks: Vec<CarTicks>,
    pub choices: Vec<Choice>,
}

const GROUP: &str = "resimulation";

/// The row each frame's entries go to: the first row at or after the frame, else the last.
fn row_of(rows: &[u32], frame: u32) -> usize {
    rows.partition_point(|&r| r < frame).min(rows.len() - 1)
}

/// Lengths per row of entries placed by their frames (`frames` in order of the entries, which must be sorted).
fn lengths(rows: &[u32], frames: impl Iterator<Item = u32>) -> Vec<usize> {
    let mut lengths = vec![0; rows.len()];
    if !rows.is_empty() {
        for frame in frames {
            lengths[row_of(rows, frame)] += 1;
        }
    }
    lengths
}

fn u32s(name: &str, values: Vec<Option<u32>>) -> (Field, ArrayRef) {
    (
        Field::new(name, DataType::UInt32, true),
        Arc::new(UInt32Array::from(values)),
    )
}

fn i64s(name: &str, values: Vec<Option<i64>>) -> (Field, ArrayRef) {
    (
        Field::new(name, DataType::Int64, true),
        Arc::new(Int64Array::from(values)),
    )
}

/// A list column of records made of `children`, `lengths` per item.
fn nested(name: &str, lengths: &[usize], children: Vec<(Field, ArrayRef)>) -> (Field, ArrayRef) {
    let (fields, arrays): (Vec<Field>, Vec<ArrayRef>) = children.into_iter().unzip();
    let fields = Fields::from(fields);
    let item = Arc::new(Field::new("item", DataType::Struct(fields.clone()), false));
    let list = ListArray::new(
        item,
        OffsetBuffer::from_lengths(lengths.iter().copied()),
        Arc::new(StructArray::new(fields, arrays, None)),
        None,
    );
    (
        Field::new(name, list.data_type().clone(), true),
        Arc::new(list),
    )
}

fn dodge_children(dodges: &[Option<Dodge>]) -> Vec<(Field, ArrayRef)> {
    let d = |f: fn(&Dodge) -> Option<f32>| dodges.iter().map(|d| d.as_ref().and_then(f)).collect();
    vec![
        u32s(
            "activation_frame",
            dodges
                .iter()
                .map(|d| d.map(|d| d.activation_frame))
                .collect(),
        ),
        child_u64(
            "start_offset",
            dodges.iter().map(|d| d.map(|d| d.start_offset)).collect(),
        ),
        child_u64(
            "duration",
            dodges.iter().map(|d| d.map(|d| d.duration)).collect(),
        ),
        child_f32("pitch", d(|d| Some(d.pitch))),
        child_f32("yaw", d(|d| Some(d.yaw))),
        child_f32("cancel", d(|d| Some(d.cancel))),
        u32s(
            "first_update_frame",
            dodges
                .iter()
                .map(|d| d.and_then(|d| d.first_update).map(|f| f.0))
                .collect(),
        ),
        child_u64(
            "first_update_ticks",
            dodges
                .iter()
                .map(|d| d.and_then(|d| d.first_update).map(|f| f.1))
                .collect(),
        ),
    ]
}

/// Writes the group's columns for the written rows' frames `rows` (ascending): the entries of frames after `after`
/// and through `through` (each unbounded when `None`).
pub(crate) fn columns(
    columns: &mut Columns,
    rows: &[u32],
    group: &Resimulation,
    after: Option<u32>,
    through: Option<u32>,
) {
    // The entries in frame order, so that each row's are contiguous; none without a row to hold them.
    let mut group = group.clone();
    let keep = |frame: u32| after.is_none_or(|a| frame > a) && through.is_none_or(|t| frame <= t);
    group.ticks.retain(|t| keep(t.frame));
    group.car_ticks.retain(|t| keep(t.frame));
    group.choices.retain(|c| keep(c.frame));
    if rows.is_empty() {
        group = Resimulation::default();
    }
    group.ticks.sort_by_key(|t| t.frame);
    group.car_ticks.sort_by_key(|t| t.frame);
    group.choices.sort_by_key(|c| c.frame);
    let ticks = &group.ticks;
    columns.records(
        "resim_update_ticks",
        GROUP,
        &lengths(rows, ticks.iter().map(|t| t.frame)),
        vec![
            u32s("frame", ticks.iter().map(|t| Some(t.frame)).collect()),
            u32s("ball", ticks.iter().map(|t| t.ball).collect()),
            u32s("car_median", ticks.iter().map(|t| t.car_median).collect()),
        ],
    );
    let cars = &group.car_ticks;
    columns.records(
        "resim_car_update_ticks",
        GROUP,
        &lengths(rows, cars.iter().map(|t| t.frame)),
        vec![
            u32s("frame", cars.iter().map(|t| Some(t.frame)).collect()),
            i64s(
                "actor",
                cars.iter().map(|t| Some(i64::from(t.actor))).collect(),
            ),
            u32s("created", cars.iter().map(|t| Some(t.created)).collect()),
            u32s("ticks", cars.iter().map(|t| Some(t.ticks)).collect()),
        ],
    );
    let choices = &group.choices;
    let entries: Vec<&ScheduleEntry> = choices.iter().flat_map(|c| &c.entries).collect();
    let entry_lengths: Vec<usize> = choices.iter().map(|c| c.entries.len()).collect();
    let dodges: Vec<Option<Dodge>> = choices.iter().map(|c| c.dodge).collect();
    let value = |k: usize| choices.iter().map(|c| c.values[k]).collect();
    let mut children = vec![
        u32s("frame", choices.iter().map(|c| Some(c.frame)).collect()),
        i64s(
            "actor",
            choices.iter().map(|c| Some(i64::from(c.actor))).collect(),
        ),
        u32s("created", choices.iter().map(|c| Some(c.created)).collect()),
        (
            Field::new("question", DataType::Utf8, true),
            Arc::new(StringArray::from(
                choices
                    .iter()
                    .map(|c| Some(c.question.as_str()))
                    .collect::<Vec<_>>(),
            )) as ArrayRef,
        ),
        u32s("ordinal", choices.iter().map(|c| Some(c.ordinal)).collect()),
        u32s("repeat", choices.iter().map(|c| Some(c.repeat)).collect()),
        child_f32("value_0", value(0)),
        child_f32("value_1", value(1)),
        child_f32("value_2", value(2)),
        i64s("integer", choices.iter().map(|c| c.integer).collect()),
        child_u8("player", choices.iter().map(|c| c.player).collect()),
        child_u64("end_tick", choices.iter().map(|c| c.end_tick).collect()),
    ];
    children.push(nested(
        "entries",
        &entry_lengths,
        vec![
            child_u64("tick", entries.iter().map(|e| Some(e.tick)).collect()),
            child_f32("throttle", entries.iter().map(|e| e.throttle).collect()),
            child_f32("steer", entries.iter().map(|e| e.steer).collect()),
            child_f32("pitch", entries.iter().map(|e| e.pitch).collect()),
            child_f32("yaw", entries.iter().map(|e| e.yaw).collect()),
            child_f32("roll", entries.iter().map(|e| e.roll).collect()),
            child_bool("jump", entries.iter().map(|e| e.jump).collect()),
            child_bool("boost", entries.iter().map(|e| e.boost).collect()),
            child_bool("handbrake", entries.iter().map(|e| e.handbrake).collect()),
        ],
    ));
    let dodge_fields = dodge_children(&dodges);
    let (fields, arrays): (Vec<Field>, Vec<ArrayRef>) = dodge_fields.into_iter().unzip();
    let validity = dodges.iter().map(Option::is_some).collect::<Vec<_>>();
    let dodge = StructArray::new(Fields::from(fields), arrays, Some(validity.into()));
    children.push((
        Field::new("dodge", dodge.data_type().clone(), true),
        Arc::new(dodge),
    ));
    columns.records(
        "resim_choices",
        GROUP,
        &lengths(rows, choices.iter().map(|c| c.frame)),
        children,
    );
}

/// The items of a list column of records, flattened over the rows.
fn items(batch: &RecordBatch, name: &str) -> Result<StructArray, ReadError> {
    let column = batch
        .column_by_name(name)
        .ok_or_else(|| ReadError::MissingColumn(name.to_owned()))?;
    let list = column
        .as_any()
        .downcast_ref::<ListArray>()
        .ok_or_else(|| ReadError::MissingColumn(name.to_owned()))?;
    let values = list.values();
    let start = list.value_offsets()[0] as usize;
    let end = list.value_offsets()[list.len()] as usize;
    let records = values
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| ReadError::MissingColumn(name.to_owned()))?;
    Ok(records.slice(start, end - start))
}

fn field<'a, T: 'static>(records: &'a StructArray, name: &str) -> Result<&'a T, ReadError> {
    records
        .column_by_name(name)
        .and_then(|c| c.as_any().downcast_ref::<T>())
        .ok_or_else(|| ReadError::MissingColumn(name.to_owned()))
}

fn opt<A: Array, T>(array: &A, i: usize, value: impl Fn(&A, usize) -> T) -> Option<T> {
    array.is_valid(i).then(|| value(array, i))
}

/// The group from a file's columns `resim_update_ticks`, `resim_car_update_ticks` and `resim_choices`.
pub fn read(batch: &RecordBatch) -> Result<Resimulation, ReadError> {
    let ticks = items(batch, "resim_update_ticks")?;
    let (frame, ball, median) = (
        field::<UInt32Array>(&ticks, "frame")?,
        field::<UInt32Array>(&ticks, "ball")?,
        field::<UInt32Array>(&ticks, "car_median")?,
    );
    let ticks = (0..ticks.len())
        .map(|i| FrameTicks {
            frame: frame.value(i),
            ball: opt(ball, i, UInt32Array::value),
            car_median: opt(median, i, UInt32Array::value),
        })
        .collect();
    let cars = items(batch, "resim_car_update_ticks")?;
    let (frame, actor, created, count) = (
        field::<UInt32Array>(&cars, "frame")?,
        field::<Int64Array>(&cars, "actor")?,
        field::<UInt32Array>(&cars, "created")?,
        field::<UInt32Array>(&cars, "ticks")?,
    );
    let car_ticks = (0..cars.len())
        .map(|i| CarTicks {
            frame: frame.value(i),
            actor: actor.value(i) as i32,
            created: created.value(i),
            ticks: count.value(i),
        })
        .collect();
    Ok(Resimulation {
        ticks,
        car_ticks,
        choices: read_choices(&items(batch, "resim_choices")?)?,
    })
}

fn read_choices(records: &StructArray) -> Result<Vec<Choice>, ReadError> {
    let frame = field::<UInt32Array>(records, "frame")?;
    let actor = field::<Int64Array>(records, "actor")?;
    let created = field::<UInt32Array>(records, "created")?;
    let question = field::<StringArray>(records, "question")?;
    let ordinal = field::<UInt32Array>(records, "ordinal")?;
    let repeat = field::<UInt32Array>(records, "repeat")?;
    let values = [
        field::<Float32Array>(records, "value_0")?,
        field::<Float32Array>(records, "value_1")?,
        field::<Float32Array>(records, "value_2")?,
    ];
    let integer = field::<Int64Array>(records, "integer")?;
    let player = field::<UInt8Array>(records, "player")?;
    let end_tick = field::<UInt64Array>(records, "end_tick")?;
    let entries = field::<ListArray>(records, "entries")?;
    let entry_records = entries
        .values()
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| ReadError::MissingColumn("entries".to_owned()))?;
    let tick = field::<UInt64Array>(entry_records, "tick")?;
    let axes = ["throttle", "steer", "pitch", "yaw", "roll"]
        .map(|n| field::<Float32Array>(entry_records, n));
    let buttons = ["jump", "boost", "handbrake"].map(|n| field::<BooleanArray>(entry_records, n));
    let axes = axes.into_iter().collect::<Result<Vec<_>, _>>()?;
    let buttons = buttons.into_iter().collect::<Result<Vec<_>, _>>()?;
    let dodge = field::<StructArray>(records, "dodge")?;
    let activation = field::<UInt32Array>(dodge, "activation_frame")?;
    let start_offset = field::<UInt64Array>(dodge, "start_offset")?;
    let duration = field::<UInt64Array>(dodge, "duration")?;
    let dodge_axes = ["pitch", "yaw", "cancel"].map(|n| field::<Float32Array>(dodge, n));
    let dodge_axes = dodge_axes.into_iter().collect::<Result<Vec<_>, _>>()?;
    let first_frame = field::<UInt32Array>(dodge, "first_update_frame")?;
    let first_ticks = field::<UInt64Array>(dodge, "first_update_ticks")?;
    let offsets = entries.value_offsets();
    let mut out = Vec::with_capacity(records.len());
    for i in 0..records.len() {
        let entries = (offsets[i] as usize..offsets[i + 1] as usize)
            .map(|e| ScheduleEntry {
                tick: tick.value(e),
                throttle: opt(axes[0], e, Float32Array::value),
                steer: opt(axes[1], e, Float32Array::value),
                pitch: opt(axes[2], e, Float32Array::value),
                yaw: opt(axes[3], e, Float32Array::value),
                roll: opt(axes[4], e, Float32Array::value),
                jump: opt(buttons[0], e, BooleanArray::value),
                boost: opt(buttons[1], e, BooleanArray::value),
                handbrake: opt(buttons[2], e, BooleanArray::value),
            })
            .collect();
        out.push(Choice {
            frame: frame.value(i),
            actor: actor.value(i) as i32,
            created: created.value(i),
            question: question.value(i).to_owned(),
            ordinal: ordinal.value(i),
            repeat: repeat.value(i),
            values: values.map(|v| opt(v, i, Float32Array::value)),
            integer: opt(integer, i, Int64Array::value),
            player: opt(player, i, UInt8Array::value),
            end_tick: opt(end_tick, i, UInt64Array::value),
            entries,
            dodge: dodge.is_valid(i).then(|| Dodge {
                activation_frame: activation.value(i),
                start_offset: start_offset.value(i),
                duration: duration.value(i),
                pitch: dodge_axes[0].value(i),
                yaw: dodge_axes[1].value(i),
                cancel: dodge_axes[2].value(i),
                first_update: first_frame
                    .is_valid(i)
                    .then(|| (first_frame.value(i), first_ticks.value(i))),
            }),
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_group_reads_back_with_entries_of_frames_without_rows() {
        let group = Resimulation {
            ticks: vec![
                FrameTicks {
                    frame: 1,
                    ball: Some(2),
                    car_median: None,
                },
                FrameTicks {
                    frame: 9,
                    ball: None,
                    car_median: Some(3),
                },
            ],
            car_ticks: vec![CarTicks {
                frame: 4,
                actor: -1,
                created: 2,
                ticks: 1,
            }],
            choices: vec![
                Choice {
                    frame: 5,
                    actor: 7,
                    created: 2,
                    question: "ground_schedule".to_owned(),
                    ordinal: 3,
                    repeat: 0,
                    integer: Some(-2),
                    end_tick: Some(40),
                    entries: vec![ScheduleEntry {
                        tick: 38,
                        throttle: Some(1.0),
                        steer: Some(-0.5),
                        jump: None,
                        boost: Some(true),
                        handbrake: Some(false),
                        ..ScheduleEntry::default()
                    }],
                    dodge: Some(Dodge {
                        activation_frame: 5,
                        start_offset: 1,
                        duration: 6,
                        pitch: -1.0,
                        yaw: 0.25,
                        cancel: 0.5,
                        first_update: None,
                    }),
                    ..Choice::default()
                },
                Choice {
                    frame: 2,
                    actor: 7,
                    created: 2,
                    question: "control_shift".to_owned(),
                    ordinal: 0,
                    repeat: 12,
                    integer: Some(1),
                    ..Choice::default()
                },
            ],
        };
        // Rows for frames 3 and 6: frames 1-3 go to the first, 4-6 and the later 9 to the second.
        let mut columns = Columns::default();
        columns.u32("frame", "frame", None, [Some(3), Some(6)]);
        super::columns(&mut columns, &[3, 6], &group, None, None);
        let schema = Arc::new(arrow_schema::Schema::new(columns.fields));
        let batch = RecordBatch::try_new(schema, columns.arrays).unwrap();
        let lists = batch
            .column_by_name("resim_update_ticks")
            .unwrap()
            .as_any()
            .downcast_ref::<ListArray>()
            .unwrap();
        assert_eq!(lists.value_length(0), 1);
        assert_eq!(lists.value_length(1), 1);
        let mut expected = group.clone();
        expected.choices.sort_by_key(|c| c.frame);
        assert_eq!(read(&batch).unwrap(), expected);
    }
}
