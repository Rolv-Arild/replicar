//! The replicar file format and the types every replicar crate shares.
//!
//! A replicar file is one ordinary Parquet file per replay, one row per frame in a play segment, holding
//! the column groups it was asked for (docs/v2-plan.md, section 3). This crate depends on neither the
//! replay parser nor RocketSim, so reading a file needs neither. The words used here are defined in
//! docs/glossary.md.

mod columns;
mod game;
pub mod header;
mod identity;
mod read;
mod read_state;
pub mod record;
pub mod resimulation;
mod time;
mod write;

/// The Arrow array types, for reading a file's columns.
pub use arrow_array as arrow;
pub use arrow_array::RecordBatch;
pub use columns::{
    Columns, child_bool, child_f32, child_i32, child_str, child_u8, child_u16, child_u64,
};
pub use game::{
    AirControlSource, CarStatus, ClockPhase, GroundControlSource, Period, SegmentEnd, StatKind,
};
pub use identity::{PlayerIndex, Team};
pub use read::{ReadError, read};
pub use read_state::{StateRow, read_states};
pub use time::{FrameIndex, ReplayTick, SimTick, TICKS_PER_SECOND};
pub use write::{
    Content, Group, Precision, QUANTA, RowRate, WriteError, WriteOptions, write, write_table,
};
