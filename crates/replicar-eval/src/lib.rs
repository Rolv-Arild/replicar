//! Evaluation and parity tools for replicar.
//!
//! The parity checks hold v2 to v1, stage by stage, on the train and validation replays: a v2 stage is
//! done when its output equals v1's on every replay (docs/v2-plan.md, section 5).

pub mod v1_shape;

use std::error::Error;
use std::path::{Path, PathBuf};

use serde_json::Value;

/// Every `.replay` file under `roots` (files are taken as they are), sorted, refusing the sealed test
/// split unless `final_assessment`.
pub fn collect_replays(
    roots: &[PathBuf],
    final_assessment: bool,
) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    fn walk(
        dir: &Path,
        final_assessment: bool,
        out: &mut Vec<PathBuf>,
    ) -> Result<(), Box<dyn Error>> {
        replicar_v1::ensure_unsealed(dir, final_assessment)?;
        for entry in std::fs::read_dir(dir)? {
            let path = entry?.path();
            if path.is_dir() {
                walk(&path, final_assessment, out)?;
            } else if path.extension().is_some_and(|e| e == "replay") {
                replicar_v1::ensure_unsealed(&path, final_assessment)?;
                out.push(path);
            }
        }
        Ok(())
    }
    let mut replays = Vec::new();
    for root in roots {
        if root.is_dir() {
            walk(root, final_assessment, &mut replays)?;
        } else {
            replicar_v1::ensure_unsealed(root, final_assessment)?;
            replays.push(root.clone());
        }
    }
    replays.sort();
    Ok(replays)
}

/// The first place where two JSON values differ, as a path with both values, or `None` when they are
/// equal. Floats compare exactly.
#[must_use]
pub fn first_difference(expected: &Value, actual: &Value) -> Option<String> {
    fn walk(path: &mut String, expected: &Value, actual: &Value) -> Option<String> {
        match (expected, actual) {
            (Value::Object(a), Value::Object(b)) => {
                for (key, value) in a {
                    let len = path.len();
                    path.push('.');
                    path.push_str(key);
                    let found = match b.get(key) {
                        Some(other) => walk(path, value, other),
                        None => Some(format!("{path}: missing in v2")),
                    };
                    path.truncate(len);
                    if found.is_some() {
                        return found;
                    }
                }
                b.keys()
                    .find(|key| !a.contains_key(*key))
                    .map(|key| format!("{path}.{key}: only in v2"))
            }
            (Value::Array(a), Value::Array(b)) => {
                for (index, (x, y)) in a.iter().zip(b).enumerate() {
                    let len = path.len();
                    path.push_str(&format!("[{index}]"));
                    let found = walk(path, x, y);
                    path.truncate(len);
                    if found.is_some() {
                        return found;
                    }
                }
                (a.len() != b.len())
                    .then(|| format!("{path}: length v1 {} v2 {}", a.len(), b.len()))
            }
            _ => (expected != actual).then(|| format!("{path}: v1 {expected} v2 {actual}")),
        }
    }
    walk(&mut String::new(), expected, actual)
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    #[test]
    fn the_first_difference_names_its_path() {
        let a = json!({"frames": [{"x": 1.0}, {"x": 2.0}], "y": [1, 2]});
        assert_eq!(first_difference(&a, &a), None);
        let b = json!({"frames": [{"x": 1.0}, {"x": 2.5}], "y": [1, 2]});
        assert_eq!(
            first_difference(&a, &b).unwrap(),
            ".frames[1].x: v1 2.0 v2 2.5"
        );
        let c = json!({"frames": [{"x": 1.0}], "y": [1, 2]});
        assert_eq!(
            first_difference(&a, &c).unwrap(),
            ".frames: length v1 2 v2 1"
        );
        let d = json!({"frames": [{"x": 1.0}, {"x": 2.0}], "y": [1, 2], "z": 0});
        assert_eq!(first_difference(&a, &d).unwrap(), ".z: only in v2");
    }
}
