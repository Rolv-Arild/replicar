//! A folder of replays: one file per replay mirroring the input's subfolders, converted `jobs` at a time, and
//! `index.parquet` (`replicar::corpus`).

use std::path::Path;

use replicar::corpus::{Report, convert_jobs, folder_jobs, write_index};
use replicar_format::WriteOptions;

pub(crate) fn convert_folder(
    meshes: &replicar::Meshes,
    input: &Path,
    output: &Path,
    options: &WriteOptions,
    threads: usize,
    skip_existing: bool,
) -> Result<(), String> {
    let jobs = folder_jobs(input, output).map_err(|e| e.to_string())?;
    std::fs::create_dir_all(output).map_err(|e| e.to_string())?;
    let rows = convert_jobs(
        meshes,
        &replicar::Config::default(),
        &jobs,
        options,
        threads,
        skip_existing,
        &Report {
            progress: &|done, total, row| match &row.error {
                Some(error) => eprintln!("{done}/{total}  FAILED {}: {error}", row.replay),
                None => eprintln!("{done}/{total}  {}", row.replay),
            },
            // The index of the replays finished so far, rewritten as the run goes (complete at the end).
            checkpoint: &|rows| {
                if let Err(error) = write_index(&output.join("index.parquet"), rows, options) {
                    eprintln!("could not write the index checkpoint: {error}");
                }
            },
        },
    );
    let index = output.join("index.parquet");
    write_index(&index, &rows, options).map_err(|e| e.to_string())?;
    let failed = rows.iter().filter(|r| r.error.is_some()).count();
    eprintln!(
        "{} replays, {failed} failed; index: {}",
        rows.len(),
        index.display()
    );
    if failed == 0 {
        Ok(())
    } else {
        Err(format!("{failed} replays failed (see the index)"))
    }
}
