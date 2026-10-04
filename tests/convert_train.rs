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

#[test]
fn inactive_player_link_and_reused_actor_id_keep_distinct_lifetimes() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let path = root.join("replays/train/1v1/00b7b402-b7dc-42af-b48d-e165ed2fd47c.replay");
    let meshes = root.join("collision_meshes");
    if !path.is_file() || !meshes.join("soccar").is_dir() {
        eprintln!("skipping actor-lifetime corpus test: local replay or meshes missing");
        return;
    }
    let output = convert_bytes(
        &fs::read(path).unwrap(),
        &ConvertOptions {
            collision_meshes: meshes,
            ..ConvertOptions::default()
        },
    )
    .unwrap();
    let demoed = output.observations.frames[3996]
        .cars
        .iter()
        .find(|car| car.actor_id == 141)
        .unwrap();
    assert!(demoed.player_key.is_some());
    assert!(!demoed.player_link_active);
    let reused = output.observations.frames[4093]
        .cars
        .iter()
        .find(|car| car.actor_id == 18)
        .unwrap();
    assert_eq!(reused.actor_created_frame, 4093);
    assert!(reused.player_key.is_none());
    let respawned = output.frames[4093]
        .state
        .cars
        .iter()
        .find(|(info, _)| info.idx == 0)
        .unwrap()
        .1;
    assert!(!respawned.is_demoed);
    assert!((respawned.phys.pos.x - 256.0).abs() < 0.01);
}

#[test]
fn live_car_wins_over_retired_car_with_same_player() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let path = root.join("replays/train/3v3/0005f298-7918-4f85-97ef-7044ca0f682a.replay");
    let meshes = root.join("collision_meshes");
    if !path.is_file() || !meshes.join("soccar").is_dir() {
        eprintln!("skipping duplicate-car corpus test: local replay or meshes missing");
        return;
    }
    let output = convert_bytes(
        &fs::read(path).unwrap(),
        &ConvertOptions {
            collision_meshes: meshes,
            // This checks which actor wins, by requiring the state to equal its packet; packet lag
            // inference deliberately advances a packet to the frame time.
            infer_packet_lag: false,
            // The demolition-correction counter below counts RocketSim's own demolitions that the
            // replay contradicts; by default the simulator's rule is off and the replay's
            // demolitions are applied, so that path is exercised with the simulator's rule on.
            disable_simulated_demolitions: false,
            ..ConvertOptions::default()
        },
    )
    .unwrap();
    let observed = &output.observations.frames[6686];
    let live = observed.cars.iter().find(|car| car.actor_id == 81).unwrap();
    let retired = observed
        .cars
        .iter()
        .find(|car| car.actor_id == 110)
        .unwrap();
    assert!(live.player_link_active);
    assert!(!retired.player_link_active);
    assert_eq!(live.player_key, retired.player_key);
    let slot = output
        .car_slots
        .iter()
        .find(|slot| Some(&slot.player_key) == live.player_key.as_ref())
        .unwrap()
        .slot;
    let converted = output.frames[6686]
        .state
        .cars
        .iter()
        .find(|(info, _)| info.idx == slot)
        .unwrap()
        .1;
    assert!((converted.phys.pos.x + 2107.0).abs() < 1.0);
    assert!((converted.phys.pos.y + 4290.0).abs() < 1.0);
    assert!(output.diagnostics.shadowed_car_frames > 0);
    assert!(output.diagnostics.active_pawn_demo_corrections > 0);
}

#[test]
fn replay_loadout_products_select_hitboxes_and_preserve_nonplaying_ids() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let meshes = root.join("collision_meshes");
    if !meshes.join("soccar").is_dir() {
        eprintln!("skipping loadout corpus test: local meshes missing");
        return;
    }
    for (replay, product, expected_hitbox) in [
        (
            "replays/train/1v1/00a63e6c-8033-40f8-a423-5b713f323dab.replay",
            403,
            Some("dominus"),
        ),
        (
            "replays/train/2v2/00b6999f-908b-47b3-a571-b1104b33bbd1.replay",
            7012,
            Some("hybrid"),
        ),
        (
            "replays/train/1v1/00a29646-cf56-4e3b-b6e1-76ae43c3e8d6.replay",
            7477,
            None,
        ),
        (
            "replays/train/1v1/00a362b5-877b-4ae4-b98e-08f089ade825.replay",
            7979,
            None,
        ),
    ] {
        let path = root.join(replay);
        if !path.is_file() {
            eprintln!("skipping missing local replay: {}", path.display());
            continue;
        }
        let output = convert_bytes(
            &fs::read(&path).unwrap(),
            &ConvertOptions {
                collision_meshes: meshes.clone(),
                ..ConvertOptions::default()
            },
        )
        .unwrap();
        assert!(
            output
                .observations
                .frames
                .iter()
                .any(|frame| frame.players.iter().any(|player| player
                    .body_product_ids
                    .iter()
                    .flatten()
                    .any(|value| value.value == product)))
        );
        if let Some(expected_hitbox) = expected_hitbox {
            assert!(
                output.car_slots.iter().any(|slot| {
                    slot.body_product_id == Some(product) && slot.hitbox == expected_hitbox
                }),
                "{replay}: product {product} did not select {expected_hitbox}; slots={:?}",
                output.car_slots
            );
        } else {
            assert!(
                output
                    .car_slots
                    .iter()
                    .all(|slot| slot.body_product_id != Some(product)),
                "{replay}: expected body product {product} only on a PRI without a playing car"
            );
        }
    }
}

/// Simulated pad pickups are blocked from the first tick of every interval: a car's exported boost does not
/// jump to a big pad's worth with neither a fresh replay boost value nor a reported pickup in that frame.
/// The replay is the one where the pickup rule used to fire (frame 3055: boost 0 to 100 with the replay's
/// value still 0). Skipped when the local replay or meshes are missing.
#[test]
fn a_car_does_not_pick_up_a_pad_the_replay_never_reported() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mesh_path = root.join("collision_meshes");
    let Some(path) = fs::read_dir(root.join("replays/train/1v1"))
        .ok()
        .into_iter()
        .flatten()
        .filter_map(|e| e.ok().map(|e| e.path()))
        .find(|p| {
            p.file_name()
                .is_some_and(|n| n.to_string_lossy().starts_with("0000a984"))
        })
    else {
        eprintln!("skipping pad blocking test: replay missing");
        return;
    };
    if !mesh_path.join("soccar").is_dir() {
        eprintln!("skipping pad blocking test: meshes missing");
        return;
    }
    let output = convert_bytes(
        &fs::read(path).unwrap(),
        &ConvertOptions {
            collision_meshes: mesh_path,
            ..ConvertOptions::default()
        },
    )
    .unwrap();
    let mut previous: std::collections::HashMap<usize, f32> = Default::default();
    let mut unsupported = Vec::new();
    for (index, frame) in output.frames.iter().enumerate() {
        let observed = &output.observations.frames[index];
        for (info, car) in &frame.state.cars {
            let before = previous.insert(info.idx, car.boost);
            let Some(before) = before else { continue };
            if car.boost - before <= 50.0 {
                continue;
            }
            let key = output
                .car_slots
                .iter()
                .find(|s| s.slot == info.idx)
                .map(|s| s.player_key.as_str());
            let observed_car = observed
                .cars
                .iter()
                .find(|c| c.player_key.as_deref() == key);
            let fresh = observed_car
                .and_then(|c| c.boost.as_ref())
                .is_some_and(|b| b.frame == index);
            let picked = observed_car.is_some_and(|c| {
                observed
                    .pad_pickups
                    .iter()
                    .any(|p| p.instigator_car_id == Some(c.actor_id))
            });
            if !fresh && !picked {
                unsupported.push((index, info.idx, before, car.boost));
            }
        }
    }
    assert!(
        unsupported.is_empty(),
        "boost jumped without a replay value or a reported pickup: {unsupported:?}"
    );
}
