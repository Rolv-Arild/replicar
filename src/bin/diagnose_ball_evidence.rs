//! Prints one line per ball interval (see `ball_evidence`): frames, physical ticks, best elapsed ticks,
//! velocity and position residuals and the two packet positions (for labelling against a recording).
//! usage: diagnose_ball_evidence <replay>
use std::env;
use std::error::Error;
use std::fs;

use replay_to_rocketsim::ball_evidence::ball_intervals;
use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes, infer_packet_lags};

fn main() -> Result<(), Box<dyn Error>> {
    let path = env::args().nth(1).ok_or("usage: diagnose_ball_evidence <replay>")?;
    if replay_to_rocketsim::sealed_path_refused(std::path::Path::new(&path), false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let options = ConvertOptions::default();
    let output = convert_bytes(&fs::read(path)?, &options)?;
    let lags = infer_packet_lags(&output.observations, &options);
    for i in ball_intervals(&output.observations, &lags, &options)? {
        println!(
            "{} {} {} {} {} {:.3} {:.3} {:.2} {:.2} {:.2} {:.2} {:.2} {:.2}",
            i.frame_a, i.frame_b, i.tick_a, i.tick_b, i.best_ticks, i.velocity_residual, i.position_residual,
            i.position_a[0], i.position_a[1], i.position_a[2], i.position_b[0], i.position_b[1], i.position_b[2]
        );
    }
    Ok(())
}
