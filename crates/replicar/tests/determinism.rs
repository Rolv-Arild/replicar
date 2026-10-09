//! A conversion is deterministic: the same replay converted twice gives the same states (the header's
//! `state_sha256`). Every hash map gets its own random seed, so an order dependence on one shows up even within a
//! process (2.0.1 depended on one: RESULTS.md, "User feedback from a downstream project"). Needs the collision
//! meshes and the train replays, which are local; skips without them.

use std::path::{Path, PathBuf};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

#[test]
fn converting_a_replay_twice_gives_the_same_states() {
    let meshes = root().join("collision_meshes");
    let train = root().join("replays/train");
    if !meshes.join("soccar").is_dir() || !train.is_dir() {
        eprintln!("skipping determinism test: local meshes or replays missing");
        return;
    }
    let meshes = replicar::Meshes::load(&meshes).expect("meshes");
    // The first replay of each game size.
    let mut replays = Vec::new();
    for size in ["1v1", "2v2", "3v3"] {
        let mut files: Vec<PathBuf> = std::fs::read_dir(train.join(size))
            .into_iter()
            .flatten()
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|e| e == "replay"))
            .collect();
        files.sort();
        replays.extend(files.into_iter().take(1));
    }
    assert!(!replays.is_empty(), "no train replays");
    for replay in replays {
        let bytes = std::fs::read(&replay).expect("replay");
        let checksum = || {
            let converter = replicar::Converter::new(&meshes, replicar::Config::default());
            converter
                .convert(&bytes)
                .expect("conversion")
                .header
                .state_sha256
        };
        let (first, second) = (checksum(), checksum());
        assert_eq!(
            first,
            second,
            "{} converts to different states",
            replay.display()
        );
    }
}
