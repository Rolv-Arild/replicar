use std::env;
use std::error::Error;
use std::fs;
use std::time::Instant;

use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes};

fn median(values: impl Iterator<Item = f32>) -> Option<f32> {
    let mut values: Vec<_> = values.filter(|value| value.is_finite()).collect();
    if values.is_empty() {
        return None;
    }
    values.sort_by(f32::total_cmp);
    Some(values[values.len() / 2])
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = env::args_os()
        .nth(1)
        .ok_or("usage: convert_preview <replay>")?;
    let bytes = fs::read(path)?;
    let start = Instant::now();
    let output = convert_bytes(&bytes, &ConvertOptions::default())?;
    let first = output.frames.first().ok_or("empty replay")?;
    let last = output.frames.last().ok_or("empty replay")?;
    let scores = output
        .observations
        .frames
        .last()
        .unwrap()
        .team_scores
        .each_ref()
        .map(|score| score.as_ref().map(|score| score.value));
    println!(
        "frames={} time={:.2}..{:.2} timeline_ticks={} arena_ticks={} cars={} ball={:?} scores={scores:?}",
        output.frames.len(),
        first.replay_time,
        last.replay_time,
        last.timeline_tick,
        last.state.tick_count,
        last.state.num_cars(),
        last.state.ball.phys.pos
    );
    println!(
        "diagnostics={:?} elapsed={:.2?}",
        output.diagnostics,
        start.elapsed()
    );
    for (kind, is_ball) in [("ball", true), ("car", false)] {
        let residuals: Vec<_> = output
            .position_residuals
            .iter()
            .filter(|residual| residual.actor_id.is_none() == is_ball)
            .collect();
        println!(
            "{kind} residuals={} median UU: sim={:?} hold={:?} linear={:?}",
            residuals.len(),
            median(residuals.iter().map(|r| r.simulated_error_uu)),
            median(residuals.iter().map(|r| r.hold_error_uu)),
            median(
                residuals
                    .iter()
                    .filter_map(|r| r.linear_extrapolation_error_uu)
            ),
        );
    }
    Ok(())
}
