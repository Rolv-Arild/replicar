# replicar

*Replica cars from Rocket League replays.* replicar converts Rocket League replays into one [RocketSim](https://github.com/ZealanL/RocketSim) game state per replay network frame, with the scoreboard, observed controls, events and provenance alongside. Replay packets are treated as exact server states placed on their true 120 Hz tick; RocketSim simulates every tick in between, and inputs the replay does not record (jump, dodge and control timing, flip cancels, air pitch/yaw/roll) are fitted against later packets.

On 60 held-out test replays every replay converted, and the car position predicted by RocketSim just before each replay packet is off by 0.04 / 3.5 / 35 UU (median / 90th / 99th percentile). [RESULTS.md](RESULTS.md) has the full evaluation.

## Quick start

Requirements: Rust 1.97 or newer, and RocketSim's soccar collision meshes (`*.cmf`) under `collision_meshes/soccar/` in the working directory (or pass the mesh directory as an argument).

```sh
cargo build --release
./target/release/convert_replay my_match.replay my_match.parquet   # or my_match.jsonl
```

`convert_replay <input.replay> <output.jsonl|output.parquet> [collision_meshes] [--octane-hitbox] [--no-event-tables]`

- **Parquet** (recommended for Python and ML): one row per frame with typed columns for the state, controls, boost pads, scoreboard, labels and freshness, the complete record in `frame_json`, and seven record tables beside it (`my_match.touches.parquet`, `.ball_contacts`, `.boost_pickups`, `.fitted_inputs`, `.packet_lags`, `.events`, `.pad_pickups`).
- **JSON Lines**: one `header` record and one `frame` record per network frame; easy to inspect.
- The export is published only when complete: a failed run leaves an existing output unchanged.

## Reading the output in Python

```sh
python -m pip install -r python/requirements-columnar.txt   # pyarrow, numpy
```

```python
import sys
sys.path.insert(0, "python")
from replay_columnar import read_columnar_header, load_columnar_numpy, read_record_tables

header = read_columnar_header("my_match.parquet")
arrays = load_columnar_numpy("my_match.parquet")
print(arrays["car_position"].shape)        # frames x car slots x 3 (UU)
tables = read_record_tables("my_match.parquet")
events = tables["events"].to_pandas()      # needs pandas: goals, demolitions, flip resets
```

For JSON Lines, `replicar.py` offers `read_header`, `iter_frames` (standard library only) and `load_numpy`. Unknown values are NaN for floats and -1 for integers in the arrays, and null in the files, never zero.

[docs/output-format.md](docs/output-format.md) describes every field: state, observations with their freshness, events, the scoreboard, record tables, training labels and freshness columns.

## What the output contains

| Part | Source | Notes |
| --- | --- | --- |
| RocketSim state (ball, cars, boost pads) | replay packets + simulation | packets corrected at their inferred tick, states reported at frame time |
| Observed fields (bodies, controls, boost, counters, ping) | replay | each with the frame it was last updated |
| Scoreboard | replay | score, a fractional clock fitted from the integer one, and the match phase |
| Events | replay | goals, demolitions, flip resets, pad pickups; touches found from the ball's own motion |
| Fitted inputs | inferred | jump and dodge presses, air-control intervals |
| Freshness | observed / inferred | whether a packet arrived this frame, update ages, packet ages in ticks |
| Training labels (`labels`, `label_*`) | future-derived | episode, time to its end, next scoring team; never use as model input |

## Using it from Rust

```rust
use replicar::conversion::{convert_bytes, ConvertOptions};

let bytes = std::fs::read("my_match.replay")?;
let output = convert_bytes(&bytes, &ConvertOptions::default())?;
for frame in &output.frames {
    let state = &frame.state;   // RocketSim ArenaState
}
```

`restoration::state_from_frame_json` and `restoration::restore_soccar_state` rebuild a RocketSim `ArenaState` from an exported frame; `apply_soccar_state_to_arena` seeds a live arena from it (the live arena keeps its own tick and RNG, so continuing from it is not an exact replay continuation).

## Limits

- Air pitch, yaw and roll are not in the replay; the exported values are the constant control per packet gap that reproduces the motion, not the player's per-tick inputs.
- The fits use later packets: the converter is an offline reconstruction, not a causal predictor.
- Only standard soccar is supported. Other maps or mutators are reported in the header's `observation_diagnostics`, not refused.
- Ground truth for timing, inputs and events comes from two LAN games recorded with RLBot; online replays with real latency are checked only indirectly.
- Some state is inferred and labelled: demolished car shells held out of play, velocities zeroed by sleeping packets, spawn poses before a car's first packet.

## Development

```sh
cargo test --all-targets          # some tests need replays under replays/ and skip without them
cd python && python -m unittest
```

Evaluation and diagnostics: `evaluate_corpus` (held-out one-step and masked prediction against hold and linear baselines), `error_budget`, `check_scoreboard`, `count_demolitions`, `consistency_counts`; `rlbot_reconstruction`, `align_rlbot` and `rlbot_onestep` score against RLBot recordings; `dump_reconstruction` against a BakkesMod state dump; `verify_state_restoration` checks restoration from Parquet. Tools that read replays refuse paths containing a `test` component unless given `--final-assessment`. The one-off experiment tools of the development history were removed at tag `pre-cleanup`.

- [RESULTS.md](RESULTS.md): measurements, experiments (including the ones that did not work) and the test-split assessment.
- [TEST_PROTOCOL.md](TEST_PROTOCOL.md): the protocol of the single test-split run.
- [ROCKETSIM_NOTES.md](ROCKETSIM_NOTES.md): differences from the game found in RocketSim, for its developers.
- [PLAN.md](PLAN.md): plan and work log. [AGENTS.md](AGENTS.md): guidance for contributors and coding agents.
- [data/README.md](data/README.md): the car-body to hitbox mapping.

The `replays/`, `collision_meshes/`, `external/` and `target/` folders are local and ignored by Git.

## License

MIT, see [LICENSE](LICENSE). RocketSim and boxcars are MIT-licensed as well.
