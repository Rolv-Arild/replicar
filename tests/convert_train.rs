use std::fs;
use std::path::Path;

use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes};

#[test]
fn one_replay_per_train_game_size_converts_to_finite_states() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mesh_path = root.join("collision_meshes");
    let replays = root.join("replays/train");
    if !mesh_path.join("soccar").is_dir() || !replays.is_dir() {
        eprintln!("skipping conversion corpus test: local meshes or replays missing");
        return;
    }
    let options = ConvertOptions {
        collision_meshes: mesh_path,
        ..ConvertOptions::default()
    };
    for game_size in ["1v1", "2v2", "3v3"] {
        let mut paths: Vec<_> = fs::read_dir(replays.join(game_size))
            .unwrap()
            .map(|entry| entry.unwrap().path())
            .filter(|path| path.extension().is_some_and(|ext| ext == "replay"))
            .collect();
        paths.sort();
        let path = paths
            .first()
            .expect("at least one training replay per size");
        let output = convert_bytes(&fs::read(path).unwrap(), &options).unwrap();
        assert_eq!(output.frames.len(), output.observations.frames.len());
        assert!(output.frames.len() > 1000);
        for frame in &output.frames {
            assert!(frame.state.ball.phys.pos.is_finite(), "{}", path.display());
            assert!(
                frame
                    .state
                    .cars
                    .iter()
                    .all(|(_, car)| car.phys.pos.is_finite())
            );
        }
        assert!(!output.position_residuals.is_empty());
    }
}
