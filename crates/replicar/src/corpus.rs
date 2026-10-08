//! Converting many replays (docs/v2-plan.md, section 3.4): one file per replay, several at a time, and an
//! index with one row per replay. A replay that fails becomes a row with its error, not an aborted batch.

use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

use replicar_format::{Columns, WriteOptions};
use sha2::{Digest, Sha256};

use crate::{Config, Conversion, Converter, Error, Meshes};

/// One replay to convert: its input and output, and the names the index gives them.
#[derive(Debug, Clone)]
pub struct Job {
    pub input: PathBuf,
    pub output: PathBuf,
    pub replay: String,
    pub file: String,
}

/// One row of the index.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct IndexRow {
    pub replay: String,
    /// Empty when the replay failed.
    pub file: String,
    pub sha256: Option<String>,
    pub error: Option<String>,
    pub rows: Option<u32>,
    /// The replay time the written rows span.
    pub duration_seconds: Option<f32>,
    pub map: Option<String>,
    pub players: Option<u8>,
    pub blue_players: Option<u8>,
    pub orange_players: Option<u8>,
    pub blue_score: Option<i32>,
    pub orange_score: Option<i32>,
    pub segments: Option<u32>,
}

/// Converts the replay at `input` and writes it to `output`, creating its folder.
pub fn convert_file(
    converter: &Converter,
    input: &Path,
    output: &Path,
    options: &WriteOptions,
) -> Result<Conversion, Error> {
    let bytes = std::fs::read(input).map_err(|e| Error::Io(format!("{}: {e}", input.display())))?;
    let conversion = converter.convert(&bytes)?;
    if let Some(parent) = output.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent).map_err(|e| Error::Io(e.to_string()))?;
    }
    conversion.write(output, options)?;
    Ok(conversion)
}

fn replays(dir: &Path, out: &mut Vec<PathBuf>) -> std::io::Result<()> {
    for entry in std::fs::read_dir(dir)? {
        let path = entry?.path();
        if path.is_dir() {
            replays(&path, out)?;
        } else if path.extension().is_some_and(|e| e == "replay") {
            out.push(path);
        }
    }
    Ok(())
}

/// Every `.replay` under `input`, each to the same relative path under `output` with `.parquet`.
pub fn folder_jobs(input: &Path, output: &Path) -> Result<Vec<Job>, Error> {
    let mut paths = Vec::new();
    replays(input, &mut paths).map_err(|e| Error::Io(format!("{}: {e}", input.display())))?;
    paths.sort();
    Ok(paths
        .into_iter()
        .map(|path| {
            let relative = path.strip_prefix(input).unwrap_or(&path).to_path_buf();
            let target = output.join(&relative).with_extension("parquet");
            Job {
                replay: relative.to_string_lossy().replace('\\', "/"),
                file: relative
                    .with_extension("parquet")
                    .to_string_lossy()
                    .replace('\\', "/"),
                input: path,
                output: target,
            }
        })
        .collect())
}

/// The index row of a written file, from its header.
fn row_of_file(job: &Job) -> IndexRow {
    let mut row = IndexRow {
        replay: job.replay.clone(),
        file: job.file.clone(),
        ..IndexRow::default()
    };
    match replicar_format::read(&job.output, Some(&["replay_time"])) {
        Ok((header, batch)) => {
            let times = batch
                .column(0)
                .as_any()
                .downcast_ref::<replicar_format::arrow::Float32Array>()
                .map(|t| t.values().to_vec())
                .unwrap_or_default();
            row.sha256 = Some(header.replay_sha256.clone());
            row.rows = u32::try_from(batch.num_rows()).ok();
            row.duration_seconds = times.first().zip(times.last()).map(|(a, b)| b - a);
            row.map = header.diagnostics["decode"]["map_name"]
                .as_str()
                .map(str::to_owned);
            row.players = u8::try_from(header.players.len()).ok();
            let team =
                |t: u8| u8::try_from(header.players.iter().filter(|p| p.team == t).count()).ok();
            row.blue_players = team(0);
            row.orange_players = team(1);
            row.blue_score = header.final_scores[0];
            row.orange_score = header.final_scores[1];
            row.segments = u32::try_from(header.segments.len()).ok();
        }
        Err(error) => row.error = Some(format!("reading the written file: {error}")),
    }
    row
}

/// Converts `jobs`, `threads` at a time (the meshes shared), calling `progress` with the count done, the total
/// and each finished row; with `skip_existing`, a job whose output exists is only indexed. Rows in job order.
pub fn convert_jobs(
    meshes: &Meshes,
    config: &Config,
    jobs: &[Job],
    options: &WriteOptions,
    threads: usize,
    skip_existing: bool,
    progress: &(dyn Fn(usize, usize, &IndexRow) + Sync),
) -> Vec<IndexRow> {
    let next = AtomicUsize::new(0);
    let done = AtomicUsize::new(0);
    let rows = Mutex::new(vec![IndexRow::default(); jobs.len()]);
    std::thread::scope(|scope| {
        for _ in 0..threads.max(1) {
            scope.spawn(|| {
                let converter = Converter::new(meshes, config.clone()).with_rows(options.rows);
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    let Some(job) = jobs.get(i) else {
                        break;
                    };
                    let row = if skip_existing && job.output.exists() {
                        row_of_file(job)
                    } else {
                        // A panic in one replay (a bug) is that replay's error, not the end of the batch.
                        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            convert_file(&converter, &job.input, &job.output, options)
                        }))
                        .unwrap_or_else(|panic| {
                            let message = panic
                                .downcast_ref::<&str>()
                                .map(|s| (*s).to_owned())
                                .or_else(|| panic.downcast_ref::<String>().cloned())
                                .unwrap_or_else(|| "unknown".to_owned());
                            Err(Error::Io(format!("the conversion panicked: {message}")))
                        });
                        match result {
                            Ok(_) => row_of_file(job),
                            Err(error) => IndexRow {
                                replay: job.replay.clone(),
                                sha256: std::fs::read(&job.input)
                                    .ok()
                                    .map(|b| format!("{:x}", Sha256::digest(b))),
                                error: Some(error.to_string()),
                                ..IndexRow::default()
                            },
                        }
                    };
                    progress(done.fetch_add(1, Ordering::Relaxed) + 1, jobs.len(), &row);
                    if let Ok(mut rows) = rows.lock() {
                        rows[i] = row;
                    }
                }
            });
        }
    });
    rows.into_inner().unwrap_or_default()
}

/// Writes the index (`index.parquet`): the rows, and the groups and precision in its key-value metadata.
pub fn write_index(path: &Path, rows: &[IndexRow], options: &WriteOptions) -> Result<(), Error> {
    const GROUP: &str = "index";
    let mut columns = Columns::default();
    let strings = |f: fn(&IndexRow) -> Option<String>| rows.iter().map(f).collect::<Vec<_>>();
    columns.strings("replay", GROUP, strings(|r| Some(r.replay.clone())));
    columns.strings(
        "file",
        GROUP,
        strings(|r| (!r.file.is_empty()).then(|| r.file.clone())),
    );
    columns.strings("sha256", GROUP, strings(|r| r.sha256.clone()));
    columns.strings("error", GROUP, strings(|r| r.error.clone()));
    columns.u32("rows", GROUP, None, rows.iter().map(|r| r.rows));
    columns.f32(
        "duration_seconds",
        GROUP,
        Some("s"),
        rows.iter().map(|r| r.duration_seconds),
    );
    columns.strings("map", GROUP, strings(|r| r.map.clone()));
    columns.u8("players", GROUP, None, rows.iter().map(|r| r.players));
    columns.u8(
        "blue_players",
        GROUP,
        None,
        rows.iter().map(|r| r.blue_players),
    );
    columns.u8(
        "orange_players",
        GROUP,
        None,
        rows.iter().map(|r| r.orange_players),
    );
    columns.i32("blue_score", GROUP, rows.iter().map(|r| r.blue_score));
    columns.i32("orange_score", GROUP, rows.iter().map(|r| r.orange_score));
    columns.u32("segments", GROUP, None, rows.iter().map(|r| r.segments));
    let batch = columns
        .finish()
        .map_err(|e| Error::Write(replicar_format::WriteError::Arrow(e)))?;
    let groups: Vec<&str> = options.groups.iter().map(|g| g.name()).collect();
    replicar_format::write_table(
        path,
        &batch,
        &[(
            "replicar_index",
            serde_json::json!({
                "groups": groups,
                "precision": options.precision.name(),
                "all_frames": options.all_frames,
                "rows": match options.rows {
                    replicar_format::RowRate::Ticks(_) => "ticks",
                    replicar_format::RowRate::Frames => "frames",
                },
                "tick_step": match options.rows {
                    replicar_format::RowRate::Ticks(step) => step,
                    replicar_format::RowRate::Frames => 1,
                },
            })
            .to_string(),
        )],
    )?;
    Ok(())
}
