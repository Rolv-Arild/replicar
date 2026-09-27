//! Check observed-state extraction against the training corpus.

use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::{observations, parse_replay};

fn collect(root: &Path, paths: &mut Vec<PathBuf>) -> Result<(), Box<dyn Error>> {
    for entry in fs::read_dir(root)? {
        let entry = entry?;
        if entry.file_type()?.is_dir() {
            collect(&entry.path(), paths)?;
        } else if entry.path().extension().is_some_and(|ext| ext == "replay") {
            paths.push(entry.path());
        }
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let root = env::args_os()
        .nth(1)
        .ok_or("usage: observation_audit <replay-directory>")?;
    let root = Path::new(&root);
    let mut paths = Vec::new();
    collect(root, &mut paths)?;
    paths.sort();
    let mut mismatches = 0;
    let mut unlinked = 0;
    let mut repeated = 0;
    let mut unknown_updates = 0;
    let mut missing_ball = 0;
    let mut car_frames = 0;
    let mut action_component_updates = [0usize; 5];
    for path in &paths {
        let replay = parse_replay(&fs::read(path)?)?;
        let observed = observations::extract(&replay).ok_or("network frames missing")?;
        let last = observed.frames.last().ok_or("replay has no frames")?;
        let final_scores = last
            .team_scores
            .each_ref()
            .map(|score| score.as_ref().map(|v| v.value));
        if final_scores.map(|score| score.unwrap_or(0))
            != observed
                .header
                .final_team_scores
                .map(|score| score.unwrap_or(0))
        {
            mismatches += 1;
            println!(
                "score mismatch {}: {:?} vs {:?}",
                path.display(),
                final_scores,
                observed.header.final_team_scores
            );
        }
        unlinked += observed.diagnostics.unlinked_car_frames;
        let this_car_frames = observed
            .frames
            .iter()
            .map(|frame| frame.cars.len())
            .sum::<usize>();
        if observed.diagnostics.unlinked_car_frames * 100 > this_car_frames {
            println!(
                "unlinked cars {}: {}/{}",
                path.display(),
                observed.diagnostics.unlinked_car_frames,
                this_car_frames
            );
        }
        repeated += observed.diagnostics.repeated_actor_announcements;
        unknown_updates += observed.diagnostics.unknown_actor_updates;
        missing_ball += observed
            .frames
            .iter()
            .filter(|frame| frame.ball.is_none())
            .count();
        car_frames += this_car_frames;
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
            }
        }
    }
    println!(
        "files={} score_mismatches={} unlinked_car_frames={}/{} repeated_announcements={} unknown_body_updates={} missing_ball_frames={}",
        paths.len(),
        mismatches,
        unlinked,
        car_frames,
        repeated,
        unknown_updates,
        missing_ball
    );
    println!(
        "fresh component counters: boost={} jump={} double_jump={} dodge={} flip_car={}",
        action_component_updates[0],
        action_component_updates[1],
        action_component_updates[2],
        action_component_updates[3],
        action_component_updates[4]
    );
    Ok(())
}
