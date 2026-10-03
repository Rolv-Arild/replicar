//! Deterministic structural audit of replay files.

use std::collections::BTreeMap;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use boxcars::{Attribute, Replay};
use serde::Serialize;
use sha2::{Digest, Sha256};

use crate::{parse_replay, read_replay_file, sealed_path_refused};

#[derive(Debug, Serialize)]
pub struct CorpusAudit {
    pub files: Vec<FileAudit>,
    pub parsed_files: usize,
    pub failed_files: usize,
    pub total_frames: usize,
    pub total_rigid_bodies: usize,
    pub rigid_bodies_without_linear_velocity: usize,
    pub rigid_bodies_without_angular_velocity: usize,
    pub game_types: BTreeMap<String, usize>,
    pub actor_classes: BTreeMap<String, usize>,
    pub attributes: BTreeMap<String, usize>,
    /// Paths beneath the root that were not audited because they are in the sealed test split (a component
    /// named `test`, also when reached through a link or junction), relative to the root.
    pub skipped_sealed: Vec<String>,
}

#[derive(Debug, Serialize)]
pub struct FileAudit {
    pub path: String,
    pub sha256: String,
    pub bytes: usize,
    pub error: Option<String>,
    pub game_type: Option<String>,
    pub version: Option<(i32, i32, Option<i32>)>,
    pub levels: Vec<String>,
    pub header_keys: Vec<String>,
    pub frames: usize,
    pub duration_seconds: Option<f32>,
    pub first_time: Option<f32>,
    pub min_delta: Option<f32>,
    pub max_delta: Option<f32>,
    pub non_monotonic_frames: usize,
    pub created_actors: usize,
    pub deleted_actors: usize,
    pub updated_attributes: usize,
    pub rigid_bodies: usize,
    pub rigid_bodies_without_linear_velocity: usize,
    pub rigid_bodies_without_angular_velocity: usize,
    pub actor_classes: BTreeMap<String, usize>,
    pub attributes: BTreeMap<String, usize>,
}

impl FileAudit {
    fn new(path: String, bytes: &[u8]) -> Self {
        Self {
            path,
            sha256: format!("{:x}", Sha256::digest(bytes)),
            bytes: bytes.len(),
            error: None,
            game_type: None,
            version: None,
            levels: Vec::new(),
            header_keys: Vec::new(),
            frames: 0,
            duration_seconds: None,
            first_time: None,
            min_delta: None,
            max_delta: None,
            non_monotonic_frames: 0,
            created_actors: 0,
            deleted_actors: 0,
            updated_attributes: 0,
            rigid_bodies: 0,
            rigid_bodies_without_linear_velocity: 0,
            rigid_bodies_without_angular_velocity: 0,
            actor_classes: BTreeMap::new(),
            attributes: BTreeMap::new(),
        }
    }

    fn inspect(&mut self, replay: &Replay) {
        self.game_type = Some(replay.game_type.clone());
        self.version = Some((
            replay.major_version,
            replay.minor_version,
            replay.net_version,
        ));
        self.levels = replay.levels.clone();
        self.header_keys = replay
            .properties
            .iter()
            .map(|(key, _)| key.clone())
            .collect();

        let Some(network) = &replay.network_frames else {
            self.error = Some("network frames absent after strict parse".to_owned());
            return;
        };

        self.frames = network.frames.len();
        self.first_time = network.frames.first().map(|frame| frame.time);
        self.duration_seconds = network.frames.last().map(|frame| frame.time);
        let mut previous_time = None;
        for frame in &network.frames {
            self.min_delta = Some(self.min_delta.map_or(frame.delta, |x| x.min(frame.delta)));
            self.max_delta = Some(self.max_delta.map_or(frame.delta, |x| x.max(frame.delta)));
            if previous_time.is_some_and(|time| frame.time < time) {
                self.non_monotonic_frames += 1;
            }
            previous_time = Some(frame.time);
            self.created_actors += frame.new_actors.len();
            self.deleted_actors += frame.deleted_actors.len();
            self.updated_attributes += frame.updated_actors.len();

            for actor in &frame.new_actors {
                let name = object_name(replay, actor.object_id.0);
                *self.actor_classes.entry(name).or_default() += 1;
            }
            for update in &frame.updated_actors {
                let name = object_name(replay, update.object_id.0);
                *self.attributes.entry(name).or_default() += 1;
                if let Attribute::RigidBody(body) = &update.attribute {
                    self.rigid_bodies += 1;
                    self.rigid_bodies_without_linear_velocity +=
                        usize::from(body.linear_velocity.is_none());
                    self.rigid_bodies_without_angular_velocity +=
                        usize::from(body.angular_velocity.is_none());
                }
            }
        }
    }
}

fn object_name(replay: &Replay, id: i32) -> String {
    usize::try_from(id)
        .ok()
        .and_then(|id| replay.objects.get(id))
        .cloned()
        .unwrap_or_else(|| format!("<invalid object {id}>"))
}

fn collect_replays(root: &Path, paths: &mut Vec<PathBuf>, skipped: &mut Vec<PathBuf>) -> io::Result<()> {
    for entry in fs::read_dir(root)? {
        let entry = entry?;
        let path = entry.path();
        // The sealed test split is never entered or read from a parent directory (the tools refuse such a
        // root outright; a root above it must not reach it).
        if sealed_path_refused(&path, false) {
            skipped.push(path);
            continue;
        }
        if entry.file_type()?.is_dir() {
            collect_replays(&path, paths, skipped)?;
        } else if path
            .extension()
            .is_some_and(|ext| ext.eq_ignore_ascii_case("replay"))
        {
            paths.push(path);
        }
    }
    Ok(())
}

fn merge_counts(target: &mut BTreeMap<String, usize>, source: &BTreeMap<String, usize>) {
    for (name, count) in source {
        *target.entry(name.clone()).or_default() += count;
    }
}

/// Audit all `.replay` files beneath a directory, in sorted path order.
pub fn audit_directory(root: &Path) -> io::Result<CorpusAudit> {
    let mut paths = Vec::new();
    let mut skipped = Vec::new();
    collect_replays(root, &mut paths, &mut skipped)?;
    paths.sort();
    skipped.sort();

    let mut result = CorpusAudit {
        files: Vec::with_capacity(paths.len()),
        parsed_files: 0,
        failed_files: 0,
        total_frames: 0,
        total_rigid_bodies: 0,
        rigid_bodies_without_linear_velocity: 0,
        rigid_bodies_without_angular_velocity: 0,
        game_types: BTreeMap::new(),
        actor_classes: BTreeMap::new(),
        attributes: BTreeMap::new(),
        skipped_sealed: skipped
            .iter()
            .map(|path| path.strip_prefix(root).unwrap_or(path).to_string_lossy().replace('\\', "/"))
            .collect(),
    };

    for path in paths {
        let bytes = read_replay_file(&path, false)?;
        let name = path
            .strip_prefix(root)
            .unwrap_or(&path)
            .to_string_lossy()
            .replace('\\', "/");
        let mut file = FileAudit::new(name, &bytes);
        match parse_replay(&bytes) {
            Ok(replay) => file.inspect(&replay),
            Err(error) => file.error = Some(error.to_string()),
        }

        if file.error.is_some() {
            result.failed_files += 1;
        } else {
            result.parsed_files += 1;
            result.total_frames += file.frames;
            result.total_rigid_bodies += file.rigid_bodies;
            result.rigid_bodies_without_linear_velocity +=
                file.rigid_bodies_without_linear_velocity;
            result.rigid_bodies_without_angular_velocity +=
                file.rigid_bodies_without_angular_velocity;
            if let Some(game_type) = &file.game_type {
                *result.game_types.entry(game_type.clone()).or_default() += 1;
            }
            merge_counts(&mut result.actor_classes, &file.actor_classes);
            merge_counts(&mut result.attributes, &file.attributes);
        }
        result.files.push(file);
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A directory above the sealed split audits what is outside it and reports, not reads, what is inside:
    /// a fake tree under `target/` (never the real split) with `train/a.replay` and `test/b.replay`.
    #[test]
    fn a_parent_directory_does_not_audit_the_sealed_split() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("target")
            .join(format!("fake-audit-{}", std::process::id()));
        let (train, sealed) = (root.join("train"), root.join("test"));
        fs::create_dir_all(&train).unwrap();
        fs::create_dir_all(&sealed).unwrap();
        fs::write(train.join("a.replay"), b"not a replay").unwrap();
        fs::write(sealed.join("b.replay"), b"not a replay").unwrap();
        let audit = audit_directory(&root).unwrap();
        let names: Vec<_> = audit.files.iter().map(|f| f.path.as_str()).collect();
        assert_eq!(names, ["train/a.replay"]);
        assert_eq!(audit.skipped_sealed, ["test"]);
        for path in [train.join("a.replay"), sealed.join("b.replay")] {
            fs::remove_file(path).unwrap();
        }
        for dir in [train, sealed, root] {
            fs::remove_dir(dir).unwrap();
        }
    }
}
