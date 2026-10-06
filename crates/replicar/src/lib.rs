//! Rocket League replays reconstructed as RocketSim states.
//!
//! The pipeline, one module per stage (docs/v2-plan.md, section 4): decode the replay's network feed, infer
//! each update's tick, simulate the match in RocketSim between updates while the inference supplies what
//! the replay does not say, annotate the result, and write it in the replicar file format. The words used
//! here are defined in docs/glossary.md.

pub mod air;
pub mod annotate;
pub mod decode;
mod error;
pub mod hitbox;
pub mod infer;
mod meshes;
pub mod simulate;
#[cfg(test)]
mod testkit;
pub mod update_ticks;

pub use error::Error;
pub use meshes::Meshes;
pub use replicar_format::{FrameIndex, PlayerIndex, ReplayTick, SimTick, TICKS_PER_SECOND, Team};
pub use rocketsim;

/// Parse a replay, requiring its network frames: a replay that parses only as far as its header is an
/// error, never an empty conversion.
pub fn parse(bytes: &[u8]) -> Result<boxcars::Replay, Error> {
    let replay = boxcars::ParserBuilder::new(bytes)
        .must_parse_network_data()
        .parse()?;
    if replay.network_frames.is_none() {
        return Err(Error::NoNetworkFrames);
    }
    Ok(replay)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bytes_that_are_not_a_replay_are_a_parse_error() {
        assert!(matches!(parse(b"not a replay"), Err(Error::Parse(_))));
    }
}
