//! Building the file's columns: every column is a nullable Arrow array with its unit and group in the field
//! metadata. Per-player and per-pad values are one column each (`car_0_position_x`), which compresses 9-11%
//! better as floats and about 30% better as integers than lists of players (RESULTS.md, "v2: file layout").

use std::collections::HashMap;
use std::sync::Arc;

use arrow_array::builder::StringDictionaryBuilder;
use arrow_array::types::UInt8Type;
use arrow_array::{
    ArrayRef, BooleanArray, Float32Array, Int16Array, Int32Array, ListArray, StructArray,
    UInt8Array, UInt16Array, UInt32Array, UInt64Array,
};
use arrow_buffer::OffsetBuffer;
use arrow_schema::{DataType, Field, Fields};

/// The columns of a file, in order.
#[derive(Default)]
pub(crate) struct Columns {
    pub(crate) fields: Vec<Field>,
    pub(crate) arrays: Vec<ArrayRef>,
}

fn metadata(group: &str, unit: Option<&str>) -> HashMap<String, String> {
    let mut metadata = HashMap::from([("group".to_owned(), group.to_owned())]);
    if let Some(unit) = unit {
        metadata.insert("unit".to_owned(), unit.to_owned());
    }
    metadata
}

impl Columns {
    fn push(&mut self, name: String, group: &str, unit: Option<&str>, array: ArrayRef) {
        self.fields.push(
            Field::new(name, array.data_type().clone(), true).with_metadata(metadata(group, unit)),
        );
        self.arrays.push(array);
    }

    pub(crate) fn f32(
        &mut self,
        name: impl Into<String>,
        group: &str,
        unit: Option<&str>,
        values: impl IntoIterator<Item = Option<f32>>,
    ) {
        let array = Float32Array::from_iter(values);
        self.push(name.into(), group, unit, Arc::new(array));
    }

    /// A float column stored as integers of `scale` (the value is the integer times `scale`, which the field
    /// metadata records): 16-bit when `narrow`, else 32-bit. A value out of range or not finite is null.
    pub(crate) fn quantized(
        &mut self,
        name: impl Into<String>,
        group: &str,
        unit: Option<&str>,
        scale: f64,
        narrow: bool,
        values: impl IntoIterator<Item = Option<f32>>,
    ) {
        let integer = |v: f32| (f64::from(v) / scale).round();
        let array: ArrayRef = if narrow {
            Arc::new(Int16Array::from_iter(values.into_iter().map(|v| {
                v.map(integer)
                    .filter(|i| i.abs() <= f64::from(i16::MAX))
                    .map(|i| i as i16)
            })))
        } else {
            Arc::new(Int32Array::from_iter(values.into_iter().map(|v| {
                v.map(integer)
                    .filter(|i| i.abs() <= f64::from(i32::MAX))
                    .map(|i| i as i32)
            })))
        };
        let mut metadata = metadata(group, unit);
        metadata.insert("scale".to_owned(), format!("{scale:e}"));
        self.fields
            .push(Field::new(name.into(), array.data_type().clone(), true).with_metadata(metadata));
        self.arrays.push(array);
    }

    pub(crate) fn bool(
        &mut self,
        name: impl Into<String>,
        group: &str,
        values: impl IntoIterator<Item = Option<bool>>,
    ) {
        let array = BooleanArray::from_iter(values);
        self.push(name.into(), group, None, Arc::new(array));
    }

    pub(crate) fn u8(
        &mut self,
        name: impl Into<String>,
        group: &str,
        unit: Option<&str>,
        values: impl IntoIterator<Item = Option<u8>>,
    ) {
        let array = UInt8Array::from_iter(values);
        self.push(name.into(), group, unit, Arc::new(array));
    }

    pub(crate) fn u32(
        &mut self,
        name: impl Into<String>,
        group: &str,
        unit: Option<&str>,
        values: impl IntoIterator<Item = Option<u32>>,
    ) {
        let array = UInt32Array::from_iter(values);
        self.push(name.into(), group, unit, Arc::new(array));
    }

    pub(crate) fn u64(
        &mut self,
        name: impl Into<String>,
        group: &str,
        unit: Option<&str>,
        values: impl IntoIterator<Item = Option<u64>>,
    ) {
        let array = UInt64Array::from_iter(values);
        self.push(name.into(), group, unit, Arc::new(array));
    }

    pub(crate) fn i32(
        &mut self,
        name: impl Into<String>,
        group: &str,
        values: impl IntoIterator<Item = Option<i32>>,
    ) {
        let array = Int32Array::from_iter(values);
        self.push(name.into(), group, None, Arc::new(array));
    }

    /// A dictionary-encoded string column: the strings, not the keys, are the contract.
    pub(crate) fn names(
        &mut self,
        name: impl Into<String>,
        group: &str,
        values: impl IntoIterator<Item = Option<&'static str>>,
    ) {
        let mut builder = StringDictionaryBuilder::<UInt8Type>::new();
        for value in values {
            match value {
                Some(text) => {
                    builder
                        .append(text)
                        .expect("a name column has fewer than 256 names");
                }
                None => builder.append_null(),
            }
        }
        self.push(name.into(), group, None, Arc::new(builder.finish()));
    }

    /// A column of lists of records: `lengths` records per row, the records' fields as `children`.
    pub(crate) fn records(
        &mut self,
        name: impl Into<String>,
        group: &str,
        lengths: &[usize],
        children: Vec<(Field, ArrayRef)>,
    ) {
        let (fields, arrays): (Vec<Field>, Vec<ArrayRef>) = children.into_iter().unzip();
        let fields = Fields::from(fields);
        let values = if arrays.is_empty() {
            StructArray::new_empty_fields(0, None)
        } else {
            StructArray::new(fields.clone(), arrays, None)
        };
        let item = Arc::new(Field::new("item", DataType::Struct(fields), false));
        let list = ListArray::new(
            item,
            OffsetBuffer::from_lengths(lengths.iter().copied()),
            Arc::new(values),
            None,
        );
        self.push(name.into(), group, None, Arc::new(list));
    }
}

/// One field of a record column, built from every record of every row in order.
pub(crate) fn child_f32(name: &str, values: Vec<Option<f32>>) -> (Field, ArrayRef) {
    (
        Field::new(name, DataType::Float32, true),
        Arc::new(Float32Array::from(values)),
    )
}

pub(crate) fn child_bool(name: &str, values: Vec<Option<bool>>) -> (Field, ArrayRef) {
    (
        Field::new(name, DataType::Boolean, true),
        Arc::new(BooleanArray::from(values)),
    )
}

pub(crate) fn child_u8(name: &str, values: Vec<Option<u8>>) -> (Field, ArrayRef) {
    (
        Field::new(name, DataType::UInt8, true),
        Arc::new(UInt8Array::from(values)),
    )
}

pub(crate) fn child_u16(name: &str, values: Vec<Option<u16>>) -> (Field, ArrayRef) {
    (
        Field::new(name, DataType::UInt16, true),
        Arc::new(UInt16Array::from(values)),
    )
}

pub(crate) fn child_u64(name: &str, values: Vec<Option<u64>>) -> (Field, ArrayRef) {
    (
        Field::new(name, DataType::UInt64, true),
        Arc::new(UInt64Array::from(values)),
    )
}

pub(crate) fn child_str(name: &str, values: Vec<Option<&'static str>>) -> (Field, ArrayRef) {
    (
        Field::new(name, DataType::Utf8, true),
        Arc::new(arrow_array::StringArray::from(values)),
    )
}
