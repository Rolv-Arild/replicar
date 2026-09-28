//! Verify that direct Parquet rich frames rebuild soccar RocketSim snapshots.

use std::env;
use std::error::Error;
use std::fs::File;
use std::path::PathBuf;

use arrow_array::LargeBinaryArray;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use rocketsim::{Arena, GameMode};

use replay_to_rocketsim::restoration::{
    apply_soccar_state_to_arena, car_slots_from_header_json, restore_soccar_state,
    state_from_frame_json,
};
use replay_to_rocketsim::serialization::StateRecord;

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args_os().skip(1);
    let path = PathBuf::from(
        args.next()
            .ok_or("usage: verify_state_restoration <states.parquet> [collision_meshes]")?,
    );
    let meshes = PathBuf::from(args.next().unwrap_or_else(|| "collision_meshes".into()));
    if args.next().is_some() {
        return Err("usage: verify_state_restoration <states.parquet> [collision_meshes]".into());
    }
    rocketsim::init(&meshes, true)?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(File::open(&path)?)?;
    let metadata = builder.schema().metadata();
    if metadata.get("columnar_version").map(String::as_str) != Some("1") {
        return Err("unsupported Parquet columnar version".into());
    }
    let slots = car_slots_from_header_json(
        metadata
            .get("replay_header_json")
            .ok_or("missing replay header metadata")?
            .as_bytes(),
    )?;
    let reader = builder.with_batch_size(512).build()?;
    let mut frames = 0usize;
    let mut live_samples = 0usize;
    let mut max_pad_error = 0.0f32;
    for batch in reader {
        let batch = batch?;
        let index = batch.schema().index_of("frame_json")?;
        let payloads = batch
            .column(index)
            .as_any()
            .downcast_ref::<LargeBinaryArray>()
            .ok_or("frame_json is not large binary")?;
        for row in 0..batch.num_rows() {
            let record = state_from_frame_json(payloads.value(row))?;
            let restored = restore_soccar_state(&record, &slots)?;
            let round_trip = StateRecord::from_arena_state(&restored);
            if serde_json::to_value(&record)? != serde_json::to_value(&round_trip)? {
                return Err(format!("snapshot fields differ at frame {frames}").into());
            }
            if frames % 5000 == 0 {
                let mut arena = Arena::new(GameMode::Soccar);
                let report = apply_soccar_state_to_arena(&restored, &mut arena)?;
                max_pad_error = max_pad_error.max(report.max_pad_cooldown_error_seconds);
                let mut applied = StateRecord::from_arena_state(&arena.get_arena_state());
                applied.arena_tick = record.arena_tick;
                for (source, actual) in record.boost_pads.iter().zip(&mut applied.boost_pads) {
                    actual.cooldown = source.cooldown;
                }
                if serde_json::to_value(&record)? != serde_json::to_value(&applied)? {
                    return Err(format!("live arena fields differ at frame {frames}").into());
                }
                live_samples += 1;
            }
            frames += 1;
        }
    }
    println!(
        "{}: {frames} exact detached snapshots, {live_samples} live arena samples, max pad cooldown error {max_pad_error:.6} s",
        path.display()
    );
    Ok(())
}
