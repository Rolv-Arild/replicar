//! Replay parsing and reconstruction into RocketSim states.
//!
//! The conversion pipeline is being built incrementally. Strict network parsing and
//! corpus auditing are available first; simulation follows after the replay field
//! map has been measured.

use std::path::Path;

use boxcars::{ParseError, ParserBuilder, Replay};

/// Parse a replay, requiring the network frames needed for state conversion.
pub fn parse_replay(bytes: &[u8]) -> Result<Replay, ParseError> {
    ParserBuilder::new(bytes).must_parse_network_data().parse()
}

/// The test split is sealed until the frozen assessment (TEST_PROTOCOL.md): a path containing "test" is
/// refused unless the run is the final one (`--final-assessment`). Used by the tools the protocol runs on
/// the test split; other diagnostics refuse such paths outright.
pub fn sealed_path_refused(path: &Path, final_assessment: bool) -> bool {
    path.to_string_lossy().contains("test") && !final_assessment
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_sealed_test_split_needs_the_final_assessment_flag() {
        assert!(sealed_path_refused(Path::new("replays/test"), false));
        assert!(sealed_path_refused(Path::new("replays/test/1v1/a.replay"), false));
        assert!(!sealed_path_refused(Path::new("replays/test"), true));
        assert!(!sealed_path_refused(Path::new("replays/validation"), false));
        assert!(!sealed_path_refused(Path::new("replays/train/3v3"), false));
    }
}

pub mod audit;
pub mod ball_evidence;
pub mod contact_alignment;
pub mod conversion;
pub mod observations;
pub mod parquet_export;
pub mod parquet_tables;
pub mod restoration;
pub mod scoreboard;
pub mod serialization;
