//! Reading a replicar file: the header, checked, and the columns as Arrow arrays.

use std::fs::File;
use std::path::Path;

use arrow_array::RecordBatch;
use parquet::arrow::ProjectionMask;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

use crate::header::{FORMAT_VERSION, HEADER_KEY, Header};

/// The error of a read.
#[derive(Debug, thiserror::Error)]
pub enum ReadError {
    #[error("I/O: {0}")]
    Io(#[from] std::io::Error),
    #[error("Parquet: {0}")]
    Parquet(#[from] parquet::errors::ParquetError),
    #[error("Arrow: {0}")]
    Arrow(#[from] arrow_schema::ArrowError),
    #[error("the file has no replicar header: not a replicar file")]
    NoHeader,
    #[error("the header is not valid: {0}")]
    Header(#[from] serde_json::Error),
    #[error(
        "the file is format version {0}; this reader reads version {FORMAT_VERSION} and earlier"
    )]
    Version(u32),
    #[error("the file has no column {0}")]
    MissingColumn(String),
}

/// The header and the columns named in `columns` (every column when `None`), all rows in one batch.
pub fn read(path: &Path, columns: Option<&[&str]>) -> Result<(Header, RecordBatch), ReadError> {
    let builder = ParquetRecordBatchReaderBuilder::try_new(File::open(path)?)?;
    let header = builder
        .metadata()
        .file_metadata()
        .key_value_metadata()
        .and_then(|kv| kv.iter().find(|kv| kv.key == HEADER_KEY))
        .and_then(|kv| kv.value.as_deref())
        .ok_or(ReadError::NoHeader)?;
    let header: Header = serde_json::from_str(header)?;
    if header.format_version > FORMAT_VERSION {
        return Err(ReadError::Version(header.format_version));
    }
    let builder = match columns {
        Some(names) => {
            let schema = builder.schema().clone();
            let mut indices = Vec::with_capacity(names.len());
            for name in names {
                indices.push(
                    schema
                        .index_of(name)
                        .map_err(|_| ReadError::MissingColumn((*name).to_owned()))?,
                );
            }
            let mask = ProjectionMask::roots(builder.parquet_schema(), indices);
            builder.with_projection(mask)
        }
        None => builder,
    };
    let schema = builder.schema().clone();
    let reader = builder.with_batch_size(1 << 20).build()?;
    let batches = reader.collect::<Result<Vec<_>, _>>()?;
    let batch = match columns {
        Some(names) => {
            let batch = arrow_select_concat(&schema, &batches)?;
            batch.project(
                &names
                    .iter()
                    .map(|n| batch.schema().index_of(n))
                    .collect::<Result<Vec<_>, _>>()?,
            )?
        }
        None => arrow_select_concat(&schema, &batches)?,
    };
    Ok((header, batch))
}

/// The batches as one (a file is written as one row group, so there is usually one).
fn arrow_select_concat(
    schema: &arrow_schema::SchemaRef,
    batches: &[RecordBatch],
) -> Result<RecordBatch, ReadError> {
    match batches {
        [one] => Ok(one.clone()),
        [] => Ok(RecordBatch::new_empty(schema.clone())),
        _ => Err(ReadError::Parquet(parquet::errors::ParquetError::General(
            "a replicar file has one row group".to_owned(),
        ))),
    }
}
