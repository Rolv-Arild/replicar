use std::env;
use std::error::Error;
use std::fs::File;
use std::io::{self, BufWriter};
use std::path::Path;

use replay_to_rocketsim::audit::audit_directory;

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args_os().skip(1);
    let root = args
        .next()
        .ok_or("usage: replay_audit <directory> [output.json]")?;
    let output = args.next();
    if args.next().is_some() {
        return Err("usage: replay_audit <directory> [output.json]".into());
    }

    let audit = audit_directory(Path::new(&root))?;
    eprintln!(
        "audited {} files: {} parsed, {} failed, {} frames",
        audit.files.len(),
        audit.parsed_files,
        audit.failed_files,
        audit.total_frames
    );
    match output {
        Some(path) => serde_json::to_writer_pretty(BufWriter::new(File::create(path)?), &audit)?,
        None => serde_json::to_writer_pretty(io::stdout().lock(), &audit)?,
    }
    Ok(())
}
