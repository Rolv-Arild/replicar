//! Wall time of the conversion stages (parse, extract, packet lags, simulation without fits, full default) per
//! replay, to size a "replay recorded decisions" mode. Run from this folder:
//! `CARGO_TARGET_DIR=../../target cargo run --release -- <replay>...` (collision meshes at ../../collision_meshes).

use std::sync::Arc;
use std::time::Instant;
use replicar::conversion::{ConvertOptions, convert_observations, infer_packet_lags};

fn main() {
    for path in std::env::args().skip(1) {
        let bytes = std::fs::read(&path).unwrap();
        let t = Instant::now();
        let replay = replicar::parse_replay(&bytes).unwrap();
        let parse = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let obs = replicar::observations::extract(&replay).unwrap();
        let extract = t.elapsed().as_secs_f64();
        drop(replay);
        let mut base = ConvertOptions::default();
        base.collision_meshes = "../../collision_meshes".into();
        let t = Instant::now();
        let lags = infer_packet_lags(&obs, &base);
        let lag_s = t.elapsed().as_secs_f64();
        let run = |o: &ConvertOptions| { let t = Instant::now(); let out = convert_observations(obs.clone(), o).unwrap(); (t.elapsed().as_secs_f64(), out.frames.len()) };
        let mut bare = base.clone();
        bare.align_contacts = false; bare.input_fits = false; bare.air_bvp = false;
        bare.infer_air_controls_from_lookahead = false;
        bare.external_packet_lags = Some(Arc::new(lags.clone()));
        let (bare_s, n) = run(&bare);
        let mut look = bare.clone(); look.infer_air_controls_from_lookahead = true;
        let (look_s, _) = run(&look);
        let (full_s, _) = run(&base);
        println!("{path}: frames {n} parse {parse:.2}s extract {extract:.2}s lags {lag_s:.2}s | sim with given lags, no fits {bare_s:.2}s | + air lookahead {look_s:.2}s | full default {full_s:.2}s");
    }
}
