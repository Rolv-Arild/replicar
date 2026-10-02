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

/// The test split is sealed until the frozen assessment (TEST_PROTOCOL.md): a path containing "test" (any
/// case) is refused unless the run is the final one (`--final-assessment`). The path is resolved first
/// (symbolic links, `..`, an absolute spelling) and judged below the working directory, so that a directory
/// above the repository whose name contains "test" does not refuse everything; a path that cannot be
/// resolved (it does not exist yet) is judged as written. Used by the tools the protocol runs on the test
/// split; other diagnostics refuse such paths outright.
pub fn sealed_path_refused(path: &Path, final_assessment: bool) -> bool {
    if final_assessment {
        return false;
    }
    let as_written = path.to_string_lossy().to_lowercase();
    if as_written.contains("test") {
        return true;
    }
    let (Ok(resolved), Ok(cwd)) = (path.canonicalize(), std::env::current_dir().and_then(|d| d.canonicalize())) else {
        return false;
    };
    let below = resolved.strip_prefix(&cwd).unwrap_or(&resolved);
    below.to_string_lossy().to_lowercase().contains("test")
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
        assert!(sealed_path_refused(Path::new("replays/TEST"), false));
        assert!(sealed_path_refused(Path::new("replays/Test/1v1"), false));
        assert!(sealed_path_refused(Path::new("target/no_such_test_dir"), false));
        assert!(sealed_path_refused(Path::new("replays/train/../test"), false));
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
