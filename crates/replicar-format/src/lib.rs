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
pub mod record;
mod time;
mod write;

pub use game::{CarStatus, ClockPhase, Period, SegmentEnd};
pub use identity::{PlayerIndex, Team};
pub use read::{ReadError, read};
pub use time::{FrameIndex, ReplayTick, SimTick, TICKS_PER_SECOND};
pub use write::{Group, Precision, QUANTA, WriteError, WriteOptions, write};
