use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::{observations, parse_replay};

fn collect(root: &Path, paths: &mut Vec<PathBuf>) {
    for entry in fs::read_dir(root).unwrap() {
        let entry = entry.unwrap();
        if entry.file_type().unwrap().is_dir() {
            collect(&entry.path(), paths);
        } else if entry.path().extension().is_some_and(|ext| ext == "replay") {
            paths.push(entry.path());
        }
    }
}

#[test]
fn training_observations_preserve_ball_and_final_scores() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("replays/train");
    if !root.is_dir() {
        eprintln!("skipping corpus test: no local replays/train directory");
        return;
    }
    let mut paths = Vec::new();
    collect(&root, &mut paths);
    assert!(!paths.is_empty());
    paths.sort();
    for path in paths {
        let bytes = fs::read(&path).unwrap();
        let replay = parse_replay(&bytes).unwrap();
        let observed = observations::extract(&replay).unwrap();
        assert_eq!(
            observed.frames.len(),
            replay.network_frames.as_ref().unwrap().frames.len(),
            "{}",
            path.display()
        );
        assert!(
            observed.frames.iter().all(|frame| frame.ball.is_some()),
            "{}",
            path.display()
        );
        assert_eq!(
            observed.diagnostics.unknown_actor_updates,
            0,
            "{}",
            path.display()
        );
        let final_scores = observed
            .frames
            .last()
            .unwrap()
            .team_scores
            .each_ref()
            .map(|score| score.as_ref().map_or(0, |score| score.value));
        assert_eq!(
            final_scores,
            observed
                .header
                .final_team_scores
                .map(|score| score.unwrap_or(0)),
            "{}",
            path.display()
        );
    }
}
