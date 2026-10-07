//! `replicar_native`: the native part of the Python package (`pip install replicar[convert]`). The pure-Python
//! `replicar` package wraps it (`replicar.convert`, `convert_many`, `resimulate`, and `read` of a file without
//! states). Each call releases the GIL while it converts.

// A Python function's keyword arguments are its Rust parameters.
#![allow(clippy::too_many_arguments)]

use std::path::PathBuf;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use replicar_format::{Group, Precision, WriteOptions};

fn runtime(error: impl std::fmt::Display) -> PyErr {
    PyRuntimeError::new_err(error.to_string())
}

/// The write options from the Python arguments.
fn options(
    precision: &str,
    groups: Option<Vec<String>>,
    with_groups: Option<Vec<String>>,
    all_frames: bool,
) -> PyResult<WriteOptions> {
    let parse = |names: Vec<String>| -> PyResult<Vec<Group>> {
        names
            .iter()
            .map(|n| {
                Group::from_name(n)
                    .ok_or_else(|| PyValueError::new_err(format!("unknown group {n}")))
            })
            .collect()
    };
    let mut set: std::collections::BTreeSet<Group> = match groups {
        Some(names) => parse(names)?.into_iter().collect(),
        None => Group::DEFAULT.into_iter().collect(),
    };
    set.extend(parse(with_groups.unwrap_or_default())?);
    Ok(WriteOptions {
        groups: set,
        precision: Precision::from_name(precision)
            .ok_or_else(|| PyValueError::new_err(format!("unknown precision {precision}")))?,
        all_frames,
        ..WriteOptions::default()
    })
}

/// RocketSim's meshes: `meshes`, else $REPLICAR_MESHES, else ./collision_meshes.
fn meshes(meshes: Option<PathBuf>) -> PyResult<replicar::Meshes> {
    let directory = meshes
        .or_else(|| std::env::var_os("REPLICAR_MESHES").map(PathBuf::from))
        .unwrap_or_else(|| PathBuf::from("collision_meshes"));
    replicar::Meshes::load(directory).map_err(runtime)
}

/// Converts a replay to a replicar file.
#[pyfunction]
#[pyo3(signature = (replay, output, *, precision = "float32", groups = None, with_groups = None, all_frames = false, meshes_dir = None))]
fn convert(
    py: Python<'_>,
    replay: PathBuf,
    output: PathBuf,
    precision: &str,
    groups: Option<Vec<String>>,
    with_groups: Option<Vec<String>>,
    all_frames: bool,
    meshes_dir: Option<PathBuf>,
) -> PyResult<()> {
    let options = options(precision, groups, with_groups, all_frames)?;
    let meshes = meshes(meshes_dir)?;
    py.detach(|| {
        let converter = replicar::Converter::new(&meshes, replicar::Config::default());
        replicar::corpus::convert_file(&converter, &replay, &output, &options).map(|_| ())
    })
    .map_err(runtime)
}

/// Converts replays to `output_dir/<name>.parquet`, `jobs` at a time, and writes `output_dir/index.parquet`.
/// Returns one dict per replay (the index row); a replay that fails has its `error`.
#[pyfunction]
#[pyo3(signature = (replays, output_dir, *, jobs = None, skip_existing = false, precision = "float32", groups = None, with_groups = None, all_frames = false, meshes_dir = None))]
fn convert_many(
    py: Python<'_>,
    replays: Vec<PathBuf>,
    output_dir: PathBuf,
    jobs: Option<usize>,
    skip_existing: bool,
    precision: &str,
    groups: Option<Vec<String>>,
    with_groups: Option<Vec<String>>,
    all_frames: bool,
    meshes_dir: Option<PathBuf>,
) -> PyResult<Vec<std::collections::BTreeMap<String, Py<PyAny>>>> {
    let options = options(precision, groups, with_groups, all_frames)?;
    let meshes = meshes(meshes_dir)?;
    let jobs_list: Vec<replicar::corpus::Job> = replays
        .iter()
        .map(|path| {
            let name = path
                .file_stem()
                .unwrap_or_default()
                .to_string_lossy()
                .into_owned();
            replicar::corpus::Job {
                input: path.clone(),
                output: output_dir.join(format!("{name}.parquet")),
                replay: path.to_string_lossy().into_owned(),
                file: format!("{name}.parquet"),
            }
        })
        .collect();
    let threads =
        jobs.unwrap_or_else(|| std::thread::available_parallelism().map_or(1, usize::from));
    let rows = py.detach(|| -> Result<_, replicar::Error> {
        std::fs::create_dir_all(&output_dir).map_err(|e| replicar::Error::Io(e.to_string()))?;
        let rows = replicar::corpus::convert_jobs(
            &meshes,
            &replicar::Config::default(),
            &jobs_list,
            &options,
            threads,
            skip_existing,
            &|_, _, _| {},
        );
        replicar::corpus::write_index(&output_dir.join("index.parquet"), &rows, &options)?;
        Ok(rows)
    });
    let rows = rows.map_err(runtime)?;
    rows.into_iter()
        .map(|row| {
            let mut out = std::collections::BTreeMap::new();
            out.insert(
                "replay".to_owned(),
                row.replay.into_pyobject(py)?.into_any().unbind(),
            );
            out.insert(
                "file".to_owned(),
                (!row.file.is_empty())
                    .then_some(row.file)
                    .into_pyobject(py)?
                    .into_any()
                    .unbind(),
            );
            out.insert(
                "sha256".to_owned(),
                row.sha256.into_pyobject(py)?.into_any().unbind(),
            );
            out.insert(
                "error".to_owned(),
                row.error.into_pyobject(py)?.into_any().unbind(),
            );
            out.insert(
                "rows".to_owned(),
                row.rows.into_pyobject(py)?.into_any().unbind(),
            );
            Ok(out)
        })
        .collect()
}

/// Rebuilds a file from its replay and its `resimulation` group, without fitting.
#[pyfunction]
#[pyo3(signature = (file, replay, output, *, precision = "float32", groups = None, with_groups = None, all_frames = false, meshes_dir = None))]
fn resimulate(
    py: Python<'_>,
    file: PathBuf,
    replay: PathBuf,
    output: PathBuf,
    precision: &str,
    groups: Option<Vec<String>>,
    with_groups: Option<Vec<String>>,
    all_frames: bool,
    meshes_dir: Option<PathBuf>,
) -> PyResult<()> {
    let options = options(precision, groups, with_groups, all_frames)?;
    let meshes = meshes(meshes_dir)?;
    py.detach(|| -> Result<(), replicar::Error> {
        let bytes = std::fs::read(&replay).map_err(|e| replicar::Error::Io(e.to_string()))?;
        let converter = replicar::Converter::new(&meshes, replicar::Config::default());
        converter
            .resimulate(&bytes, &file)?
            .write(&output, &options)
    })
    .map_err(runtime)
}

/// The version of replicar this module was built from.
#[pyfunction]
fn version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}

#[pymodule]
fn replicar_native(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(convert, module)?)?;
    module.add_function(wrap_pyfunction!(convert_many, module)?)?;
    module.add_function(wrap_pyfunction!(resimulate, module)?)?;
    module.add_function(wrap_pyfunction!(version, module)?)?;
    Ok(())
}
