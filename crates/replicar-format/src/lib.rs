//! The replicar file format and the types every replicar crate shares.
//!
//! A replicar file is one ordinary Parquet file per replay, one row per frame in a play segment, holding
//! the column groups it was asked for (docs/v2-plan.md, section 3). This crate depends on neither the
//! replay parser nor RocketSim, so reading a file needs neither. The words used here are defined in
//! docs/glossary.md.

mod identity;
mod time;

pub use identity::{PlayerIndex, Team};
pub use time::{FrameIndex, ReplayTick, SimTick, TICKS_PER_SECOND};
