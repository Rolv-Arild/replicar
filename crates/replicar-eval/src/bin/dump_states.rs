//! Prints what the Rust reader decodes from a replicar file's `state` group, one JSON object per row (frame,
//! sim tick, ball and car bodies, boost, pad cooldowns), so that other readers can be checked against it
//! (story 8.2, `scripts/check_python_reader.py`).
//!
//! usage: `dump_states <file.parquet>`

use std::io::Write;
use std::process::ExitCode;

use replicar_format::record::Body;

fn body(b: &Body) -> serde_json::Value {
    serde_json::json!([b.position, b.velocity, b.angular_velocity, b.rotation])
}

fn main() -> ExitCode {
    let Some(path) = std::env::args().nth(1) else {
        eprintln!("usage: dump_states <file.parquet>");
        return ExitCode::FAILURE;
    };
    let (_, rows) = match replicar_format::read_states(std::path::Path::new(&path)) {
        Ok(read) => read,
        Err(error) => {
            eprintln!("error: {error}");
            return ExitCode::FAILURE;
        }
    };
    let mut out = std::io::BufWriter::new(std::io::stdout().lock());
    for row in rows {
        let line = serde_json::json!({
            "frame": row.frame.0,
            "sim_tick": row.sim_tick,
            "ball": body(&row.state.ball.body),
            "cars": row.state.cars.iter().map(|c| c.map(|c| (body(&c.body), c.boost))).collect::<Vec<_>>(),
            "status": row.state.car_status.iter().map(|s| s.name()).collect::<Vec<_>>(),
            "pads": row.state.pad_cooldowns,
        });
        if writeln!(out, "{line}").is_err() {
            return ExitCode::FAILURE;
        }
    }
    ExitCode::SUCCESS
}
