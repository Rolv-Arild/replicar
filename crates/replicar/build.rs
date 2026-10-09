//! Which RocketSim release this build simulates with: replicar accepts any compatible release (the latest is assumed
//! best) and records the one it was built with in every file's header (`rocketsim_version`). RocketSim does not
//! expose its version, so it is read from the `Cargo.lock` of the build: the workspace's own when replicar is built
//! from its repository, else (a packaged crate, marked by `.cargo_vcs_info.json`, whose own lock is from publishing)
//! the dependent project's, found above the build's output folder. "unknown" when there is none; resimulation then
//! relies on the state checksum alone.

use std::path::{Path, PathBuf};

fn main() {
    let manifest = PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let out = PathBuf::from(std::env::var_os("OUT_DIR").expect("OUT_DIR"));
    let packaged = manifest.join(".cargo_vcs_info.json").exists();
    let start = if packaged { out } else { manifest };
    let found = start
        .ancestors()
        .map(|dir| dir.join("Cargo.lock"))
        .find_map(|lock| version_in(&lock).map(|version| (lock, version)));
    let version = match found {
        Some((lock, version)) => {
            println!("cargo:rerun-if-changed={}", lock.display());
            version
        }
        None => "unknown".to_owned(),
    };
    println!("cargo:rustc-env=REPLICAR_ROCKETSIM_VERSION={version}");
    println!("cargo:rerun-if-changed=build.rs");
}

/// The version of the `rocketsim` package in a `Cargo.lock`, if it has one.
fn version_in(lock: &Path) -> Option<String> {
    let text = std::fs::read_to_string(lock).ok()?;
    let mut lines = text.lines();
    while let Some(line) = lines.next() {
        if line.trim() == "name = \"rocketsim\"" {
            let version = lines.next()?.trim();
            return version
                .strip_prefix("version = \"")
                .and_then(|v| v.strip_suffix('"'))
                .map(str::to_owned);
        }
    }
    None
}
