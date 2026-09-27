//! Replay parsing and reconstruction into RocketSim states.
//!
//! The conversion pipeline is being built incrementally. Strict network parsing and
//! corpus auditing are available first; simulation follows after the replay field
//! map has been measured.

use boxcars::{ParseError, ParserBuilder, Replay};

/// Parse a replay, requiring the network frames needed for state conversion.
pub fn parse_replay(bytes: &[u8]) -> Result<Replay, ParseError> {
    ParserBuilder::new(bytes).must_parse_network_data().parse()
}

pub mod audit;
