//! Print a hash of the extracted observations of every replay under the given directories, to check
//! that a parser change (for example a `boxcars` pin update) leaves the observations unchanged.
//!
//! usage: hash_observations <dir>... (directories are searched recursively; also single files; refuses paths containing "test")

use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::observations::extract;
use sha2::{Digest, Sha256};

fn collect(dir: &Path, out: &mut Vec<PathBuf>) -> std::io::Result<()> {
    for entry in fs::read_dir(dir)? {
        let path = entry?.path();
        if path.is_dir() {
            collect(&path, out)?;
        } else if path.extension().is_some_and(|e| e == "replay") {
            out.push(path);
        }
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut files = Vec::new();
    for dir in std::env::args().skip(1) {
        if dir.contains("test") {
            return Err("refusing to inspect a path containing 'test'".into());
        }
        let path = Path::new(&dir);
        if path.is_file() {
            files.push(path.to_path_buf());
        } else {
            collect(path, &mut files)?;
        }
    }
    files.sort();
    for path in files {
        let bytes = fs::read(&path)?;
        let line = match boxcars::ParserBuilder::new(&bytes)
            .must_parse_network_data()
            .parse()
        {
            Ok(replay) => match extract(&replay) {
                Some(observed) => {
                    let json = serde_json::to_vec(&observed)?;
                    format!(
                        "{:x} {} frames",
                        Sha256::digest(&json),
                        observed.frames.len()
                    )
                }
                None => "no network frames".to_string(),
            },
            Err(_) => "parse error".to_string(),
        };
        println!("{} {line}", path.file_name().unwrap().to_string_lossy());
    }
    Ok(())
}
