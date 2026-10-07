//! Converts replays to replicar files with the default configuration, for checking the format (story 7.1).
//!
//! usage: `write_file <out dir> <replay or folder>... [--all-frames] [--quantized] [--resimulation] [--network] [--diagnostics] [--final-assessment]`

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use replicar_eval::collect_replays;

fn main() -> ExitCode {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let final_assessment = args.iter().any(|a| a == "--final-assessment");
    let all_frames = args.iter().any(|a| a == "--all-frames");
    let quantized = args.iter().any(|a| a == "--quantized");
    let resimulation = args.iter().any(|a| a == "--resimulation");
    let network = args.iter().any(|a| a == "--network");
    let diagnostics = args.iter().any(|a| a == "--diagnostics");
    args.retain(|a| {
        !matches!(
            a.as_str(),
            "--final-assessment"
                | "--all-frames"
                | "--quantized"
                | "--resimulation"
                | "--network"
                | "--diagnostics"
        )
    });
    let Some((out, inputs)) = args.split_first() else {
        eprintln!("usage: write_file <out dir> <replay or folder>... [--all-frames]");
        return ExitCode::FAILURE;
    };
    let roots: Vec<PathBuf> = inputs.iter().map(PathBuf::from).collect();
    let replays = match collect_replays(&roots, final_assessment) {
        Ok(replays) => replays,
        Err(error) => {
            eprintln!("error: {error}");
            return ExitCode::FAILURE;
        }
    };
    let meshes = match replicar::Meshes::load("collision_meshes") {
        Ok(meshes) => meshes,
        Err(error) => {
            eprintln!("error: {error}");
            return ExitCode::FAILURE;
        }
    };
    let converter = replicar::Converter::new(&meshes, replicar::Config::default());
    let mut options = replicar_format::WriteOptions {
        all_frames,
        precision: if quantized {
            replicar_format::Precision::Quantized
        } else {
            replicar_format::Precision::Float32
        },
        ..Default::default()
    };
    if resimulation {
        options.groups.insert(replicar_format::Group::Resimulation);
    }
    if network {
        options.groups.insert(replicar_format::Group::Network);
    }
    if diagnostics {
        options.groups.insert(replicar_format::Group::Diagnostics);
    }
    let mut failed = 0;
    for path in &replays {
        let target = PathBuf::from(out).join(
            path.with_extension("parquet")
                .file_name()
                .unwrap_or_default(),
        );
        let start = Instant::now();
        let outcome = std::fs::read(path)
            .map_err(|e| e.to_string())
            .and_then(|bytes| {
                converter
                    .convert_to_file(&bytes, &target, &options)
                    .map_err(|e| e.to_string())
            });
        match outcome {
            Ok(()) => println!(
                "{}  {:.2} s  {} bytes",
                target.display(),
                start.elapsed().as_secs_f64(),
                std::fs::metadata(&target).map_or(0, |m| m.len())
            ),
            Err(error) => {
                failed += 1;
                println!("FAILED {}  {error}", path.display());
            }
        }
    }
    if failed == 0 {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}
