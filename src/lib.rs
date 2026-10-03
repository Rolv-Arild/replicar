//! Replay parsing and reconstruction into RocketSim states.
//!
//! The conversion pipeline is being built incrementally. Strict network parsing and
//! corpus auditing are available first; simulation follows after the replay field
//! map has been measured.

use std::path::{Component, Path};

use boxcars::{ParseError, ParserBuilder, Replay};

/// Parse a replay, requiring the network frames needed for state conversion.
pub fn parse_replay(bytes: &[u8]) -> Result<Replay, ParseError> {
    ParserBuilder::new(bytes).must_parse_network_data().parse()
}

/// Whether some component of `path` is named `test` (any case).
fn has_sealed_component(path: &Path) -> bool {
    path.components().any(|component| match component {
        Component::Normal(name) => name.to_string_lossy().eq_ignore_ascii_case("test"),
        _ => false,
    })
}

/// The test split is sealed until the frozen assessment (TEST_PROTOCOL.md): a path with a component named
/// `test` (any case; `latest` or `contest` do not count) is refused unless the run is the final one
/// (`--final-assessment`). The path is judged as written, after resolving it (symbolic links, junctions,
/// `..`, an absolute spelling), and so is the working directory itself (`.` inside a sealed directory is
/// sealed); a path that cannot be resolved (it does not exist yet) is judged as written. Used by the tools
/// the protocol runs on the test split, whose replay collectors also call `ensure_unsealed` on every
/// directory and file they open; other diagnostics refuse paths containing "test" outright.
pub fn sealed_path_refused(path: &Path, final_assessment: bool) -> bool {
    if final_assessment {
        return false;
    }
    if has_sealed_component(path) || path.canonicalize().is_ok_and(|resolved| has_sealed_component(&resolved)) {
        return true;
    }
    std::env::current_dir()
        .and_then(|cwd| cwd.canonicalize())
        .is_ok_and(|cwd| has_sealed_component(&cwd))
}

/// For the replay collectors: refuse a directory or file that resolves into the sealed test split, even
/// when it was reached through a link or junction under another name. `Ok` when allowed.
pub fn ensure_unsealed(path: &Path, final_assessment: bool) -> Result<(), Box<dyn std::error::Error>> {
    if sealed_path_refused(path, final_assessment) {
        let resolved = path.canonicalize().map_or_else(|_| path.display().to_string(), |p| p.display().to_string());
        return Err(format!(
            "refusing to open {} (resolves to {resolved}): it is in the sealed test split (pass --final-assessment for the frozen run)",
            path.display()
        )
        .into());
    }
    Ok(())
}

/// `std::fs::read` for a replay file, refusing one that resolves into the sealed test split
/// (`ensure_unsealed`); a drop-in for the tools that read replays.
pub fn read_replay_file(path: &Path, final_assessment: bool) -> std::io::Result<Vec<u8>> {
    ensure_unsealed(path, final_assessment)
        .map_err(|error| std::io::Error::new(std::io::ErrorKind::PermissionDenied, error.to_string()))?;
    std::fs::read(path)
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
        assert!(sealed_path_refused(Path::new("target/no_such_dir/test"), false));
        assert!(sealed_path_refused(Path::new("replays/train/../test"), false));
        // A component that merely contains the letters is not the sealed split.
        assert!(!sealed_path_refused(Path::new("replays/latest"), false));
        assert!(!sealed_path_refused(Path::new("replays/contest/1v1"), false));
        assert!(!sealed_path_refused(Path::new("target/no_such_test_dir"), false));
        assert!(ensure_unsealed(Path::new("target/no_such_dir/test"), false).is_err());
        assert!(ensure_unsealed(Path::new("target/no_such_dir/test"), true).is_ok());
    }

    /// A junction (or link) under another name that resolves into a directory named `test` is refused, on a
    /// fake tree under `target/` (never the real split). Junctions are made with `mklink /J` and removed
    /// with `rmdir` (which removes only the junction), the directories one by one without recursion.
    #[cfg(windows)]
    #[test]
    fn a_junction_into_a_sealed_directory_is_refused() {
        use std::process::Command;
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("target").join(format!("fake-sealed-{}", std::process::id()));
        let sealed = root.join("test").join("1v1");
        let junction = root.join("train2");
        std::fs::create_dir_all(&sealed).unwrap();
        std::fs::create_dir_all(root.join("train").join("1v1")).unwrap();
        let made = Command::new("cmd")
            .args(["/c", "mklink", "/J"])
            .arg(&junction)
            .arg(root.join("test"))
            .output()
            .unwrap();
        assert!(made.status.success(), "mklink failed: {}", String::from_utf8_lossy(&made.stdout));
        let through = junction.join("1v1");
        assert!(sealed_path_refused(&through, false), "the junction must resolve to the sealed directory");
        assert!(ensure_unsealed(&through, false).is_err());
        assert!(ensure_unsealed(&through, true).is_ok());
        assert!(!sealed_path_refused(&root.join("train").join("1v1"), false));
        // Remove the junction itself, then the directories one by one.
        let removed = Command::new("cmd").args(["/c", "rmdir"]).arg(&junction).output().unwrap();
        assert!(removed.status.success());
        for dir in [sealed.clone(), root.join("test"), root.join("train").join("1v1"), root.join("train"), root.clone()] {
            std::fs::remove_dir(dir).unwrap();
        }
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
