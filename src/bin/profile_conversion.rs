//! Wall time of the conversion of one replay with each expensive feature switched off in turn.
//! usage: profile_conversion <replay>
use std::env;
use std::error::Error;
use std::fs;
use std::time::Instant;

use replay_to_rocketsim::conversion::{ConvertOptions, convert_observations};

fn main() -> Result<(), Box<dyn Error>> {
    let path = env::args().nth(1).ok_or("usage: profile_conversion <replay>")?;
    if path.contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let bytes = fs::read(&path)?;
    let parsed = replay_to_rocketsim::parse_replay(&bytes)?;
    let t = Instant::now();
    let observed = replay_to_rocketsim::observations::extract(&parsed).ok_or("no observations")?;
    println!("{:>7.2} s  extract observations", t.elapsed().as_secs_f64());
    let variants: Vec<(&str, Box<dyn Fn(&mut ConvertOptions)>)> = vec![
        ("align_contacts off (the base conversion)", Box::new(|o| o.align_contacts = false)),
        ("  + air_bvp off", Box::new(|o| { o.align_contacts = false; o.air_bvp = false; })),
        ("  + contacts_from_ball_packets off", Box::new(|o| { o.align_contacts = false; o.contacts_from_ball_packets = false; })),
        ("  + packet lag inference off", Box::new(|o| { o.align_contacts = false; o.infer_packet_lag = false; })),
        ("  + dodge start fit off", Box::new(|o| { o.align_contacts = false; o.infer_dodge_start = false; })),
        ("  + flip cancel fit off", Box::new(|o| { o.align_contacts = false; o.infer_flip_cancel = false; })),
        ("  + ground timing fit off", Box::new(|o| { o.align_contacts = false; o.fit_ground_control_timing = false; })),
        ("  + jump timing fit off", Box::new(|o| { o.align_contacts = false; o.fit_jump_timing = false; })),
        ("  + all of those off", Box::new(|o| {
            o.align_contacts = false; o.air_bvp = false; o.contacts_from_ball_packets = false;
            o.infer_dodge_start = false; o.infer_flip_cancel = false; o.fit_ground_control_timing = false; o.fit_jump_timing = false;
        })),
        ("align_contacts on (default)", Box::new(|o| o.align_contacts = true)),
    ];
    for (label, tweak) in variants {
        let mut options = ConvertOptions::default();
        tweak(&mut options);
        let t = Instant::now();
        let output = convert_observations(observed.clone(), &options)?;
        println!("{:>7.2} s  {label} ({} frames)", t.elapsed().as_secs_f64(), output.frames.len());
    }
    Ok(())
}
