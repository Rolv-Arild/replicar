//! RocketSim's collision meshes: a resource loaded once per process, not an option of each conversion.

use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use crate::Error;

/// Proof that RocketSim's soccar collision meshes are loaded. Every simulation takes one.
#[derive(Debug, Clone)]
pub struct Meshes {
    directory: PathBuf,
}

static LOADED: OnceLock<PathBuf> = OnceLock::new();

impl Meshes {
    /// Load the meshes in `directory` (`<directory>/soccar/*.cmf`, the files RocketSim's mesh dumper writes;
    /// they come from the game and are not distributed with replicar). RocketSim keeps them for the whole
    /// process: a second call returns the meshes already loaded and refuses another directory.
    pub fn load(directory: impl AsRef<Path>) -> Result<Self, Error> {
        let directory = directory.as_ref().to_path_buf();
        if let Some(loaded) = LOADED.get() {
            return if *loaded == directory {
                Ok(Self { directory })
            } else {
                Err(Error::Meshes(format!(
                    "RocketSim already loaded the meshes of {}; one process uses one mesh directory",
                    loaded.display()
                )))
            };
        }
        // RocketSim panics instead of returning an error when the directory has no soccar meshes.
        let soccar = directory.join("soccar");
        let found = std::fs::read_dir(&soccar).is_ok_and(|entries| {
            entries.filter_map(Result::ok).any(|entry| {
                entry
                    .path()
                    .extension()
                    .is_some_and(|e| e.eq_ignore_ascii_case("cmf"))
            })
        });
        if !found {
            return Err(Error::Meshes(format!(
                "no soccar collision meshes (*.cmf) in {}: put RocketSim's meshes under <directory>/soccar/",
                soccar.display()
            )));
        }
        rocketsim::init(&directory, true).map_err(|error| Error::Meshes(error.to_string()))?;
        let _ = LOADED.set(directory.clone());
        Ok(Self { directory })
    }

    /// The directory the meshes were loaded from.
    #[must_use]
    pub fn directory(&self) -> &Path {
        &self.directory
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_directory_without_soccar_meshes_is_refused() {
        let missing = std::env::temp_dir().join("replicar-no-meshes-here");
        if LOADED.get().is_none() {
            assert!(matches!(Meshes::load(&missing), Err(Error::Meshes(_))));
        }
    }
}
