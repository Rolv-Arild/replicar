//! Resimulates replicar files from their `resimulation` group and the replays, and writes the result with the
//! default groups (story 7.3: the resimulated file must equal the converted one).
//!
//! usage: `resimulate_file <out dir> <file.parquet> <replay> [<file.parquet> <replay>]...`

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Instant;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some((out, pairs)) = args.split_first() else {
        eprintln!("usage: resimulate_file <out dir> <file.parquet> <replay>...");
        return ExitCode::FAILURE;
    };
    if pairs.is_empty() || pairs.len() % 2 != 0 {
        eprintln!("usage: resimulate_file <out dir> <file.parquet> <replay>...");
        return ExitCode::FAILURE;
    }
    let meshes = match replicar::Meshes::load("collision_meshes") {
        Ok(meshes) => meshes,
        Err(error) => {
            eprintln!("error: {error}");
            return ExitCode::FAILURE;
        }
    };
    let converter = replicar::Converter::new(&meshes, replicar::Config::default());
    let mut failed = 0;
    for pair in pairs.chunks(2) {
        let (file, replay) = (Path::new(&pair[0]), Path::new(&pair[1]));
        for path in [file, replay] {
            if let Err(error) = replicar_v1::ensure_unsealed(path, false) {
                eprintln!("error: {error}");
                return ExitCode::FAILURE;
            }
        }
        let target = PathBuf::from(out).join(file.file_name().unwrap_or_default());
        let start = Instant::now();
        let outcome = std::fs::read(replay)
            .map_err(|e| e.to_string())
            .and_then(|bytes| {
                converter
                    .resimulate(&bytes, file)
                    .and_then(|c| c.write(&target, &replicar_format::WriteOptions::default()))
                    .map_err(|e| e.to_string())
            });
        match outcome {
            Ok(()) => println!(
                "{}  {:.2} s",
                target.display(),
                start.elapsed().as_secs_f64()
            ),
            Err(error) => {
                failed += 1;
                println!("FAILED {}  {error}", file.display());
            }
        }
    }
    if failed == 0 {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}
