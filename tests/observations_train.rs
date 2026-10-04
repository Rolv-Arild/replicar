use std::fs;
use std::path::{Path, PathBuf};

use replicar::{observations, parse_replay};

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
    let mut action_component_updates = [0usize; 5];
    let mut dodge_torque_updates = 0;
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
        for frame in &observed.frames {
            for car in &frame.cars {
                let actions = [
                    &car.inputs.boost_active_raw,
                    &car.inputs.jump_active_raw,
                    &car.inputs.double_jump_active_raw,
                    &car.inputs.dodge_active_raw,
                    &car.inputs.flip_car_active_raw,
                ];
                for (count, action) in action_component_updates.iter_mut().zip(actions) {
                    *count += usize::from(action.as_ref().is_some_and(|v| v.frame == frame.index));
                }
                dodge_torque_updates += usize::from(
                    car.inputs
                        .dodge_torque_replay_units
                        .as_ref()
                        .is_some_and(|v| v.frame == frame.index),
                );
            }
        }
    }
    assert!(action_component_updates.into_iter().all(|count| count > 0));
    assert!(dodge_torque_updates > 0);
}
