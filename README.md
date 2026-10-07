# replicar

*Replica cars from Rocket League replays.* replicar reconstructs Rocket League replays as
[RocketSim](https://github.com/ZealanL/RocketSim) states: one row per 120 Hz simulation tick in play, with the ball and
every car's full physics state, the controls applied at that tick, boost and pads, the scoreboard, events, ball
contacts and boost pickups, and how each value is known. It writes one ordinary Parquet file per replay
(`--tick-step N` keeps every N-th tick, `--rows frames` one row per replay frame).

Replay packets are exact server states, placed on their inferred 120 Hz tick; RocketSim simulates every tick in
between, and the inputs a replay does not record (jump, dodge and control timing, flip cancels, air pitch, yaw and
roll) are fitted against later packets. On 60 held-out test replays every replay converted, and the car position
predicted just before each replay packet is off by 0.04 / 3.5 / 35 UU (median / 90th / 99th percentile;
[RESULTS.md](RESULTS.md)).

This is version 2 (in development on branch `v2`). It reproduces version 1's reconstruction value for value; see
"Version 1" below.

## Quick start

Requirements: Rust 1.97 or newer, and RocketSim's soccar collision meshes (`collision_meshes/soccar/*.cmf`, which
come from the game). Pass the folder with `--meshes DIR`, set `REPLICAR_MESHES`, or run where `collision_meshes/` is.

```sh
cargo build --release -p replicar-cli

./target/release/replicar convert my_match.replay -o my_match.parquet
./target/release/replicar convert replays/ -o out/ --jobs 16      # a folder: one file per replay + out/index.parquet
./target/release/replicar inspect my_match.parquet
```

`--precision quantized` makes the file about a quarter smaller (0.01 UU, 0.01 UU/s, 1e-4 rad/s). `--with
resimulation,network,diagnostics` adds the optional groups; `--groups game,updates,future,resimulation` writes a
file without states, about a third of the size, from which `replicar resimulate` rebuilds the states exactly.
`--rows frames` writes one row per replay frame (about 30 per second) instead of one per tick, `--tick-step N` every
N-th tick. `--all-frames` also writes the frames outside play (countdowns, goal pauses and replays).

## Reading the files

Any Parquet reader opens them (pyarrow, polars, DuckDB). The Python package adds NumPy arrays:

```sh
pip install ./python/v2          # pyarrow and NumPy only
```

```python
import replicar

f = replicar.read("my_match.parquet")
f.header["players"]              # index, name, team, hitbox
a = f.arrays()
a["car_position"]                # (frames, players, 3) float32, NaN when unknown
a["clock_phase"], a["future_segment_end"]
f.records("ball_contacts")       # one row per contact, with its frame
```

Converting from Python needs the native extra (`crates/replicar-python`, built with maturin): `replicar.convert`,
`replicar.convert_many`, `replicar.resimulate`.

## What a file contains

| Group | Default | Contents |
| --- | --- | --- |
| (always) | yes | `frame`, `segment`, `replay_time`, `replay_tick`, `sim_tick` |
| `state` | yes | ball and car physics, controls, boost, RocketSim's car internals, pads, car status |
| `game` | yes | period, clock phase, fractional clock, scores, events, ball contacts, boost pickups |
| `updates` | yes | which bodies were updated, their update ticks, ticks and seconds since the last update, ping |
| `future` | yes | how the play segment ends and the time until then; future-derived, never a model input |
| `resimulation` | no | what the inference chose; with the replay, enough to rebuild `state` exactly |
| `network` | no | the replay's values as decoded, each with the frame of its last change |
| `diagnostics` | no | prediction errors before each correction, RocketSim's events, simulated touches |

Per-player columns carry the player index (`car_0_position_x`); the header (JSON in the Parquet metadata) lists the
players, the pad layout, the play segments, the final score and the conversion's settings. Every column is in
[docs/v2-file-format.md](docs/v2-file-format.md); the words are defined in [docs/glossary.md](docs/glossary.md);
[docs/v2-guide.md](docs/v2-guide.md) explains how it works and how to use it from Rust.

## Using it from Rust

```rust
let meshes = replicar::Meshes::load("collision_meshes")?;
let converter = replicar::Converter::new(&meshes, replicar::Config::default());
let conversion = converter.convert(&std::fs::read("my_match.replay")?)?;
conversion.write("my_match.parquet".as_ref(), &replicar_format::WriteOptions::default())?;
```

`Converter::convert_network_with` hands every simulated frame (its RocketSim `ArenaState`) to a callback;
`replicar::restore::arena_states` rebuilds RocketSim states from a file; `Converter::resimulate` reproduces them
exactly from the replay and the `resimulation` group.

## Limits

- Air pitch, yaw and roll are not in the replay; the values are the constant control per packet gap that
  reproduces the motion, not the player's per-tick inputs.
- The fits use later packets: this is an offline reconstruction, not a causal predictor.
- Only standard soccar is supported.
- Ground truth for timing, inputs and events comes from two LAN games recorded with RLBot; online replays with real
  latency are checked only indirectly.
- Some state is inferred and labelled (`car_status_inferred`): wrecks held out of play, spawn poses before a car's
  first packet.

## Layout and development

| Path | What |
| --- | --- |
| `crates/replicar-format` | the file: rows, header, writer, reader (no parser, no simulator) |
| `crates/replicar` | the library: decode, update ticks, simulate, infer, annotate, convert, restore |
| `crates/replicar-cli` | the `replicar` command |
| `crates/replicar-python`, `python/v2` | the Python package and its native extra |
| `crates/replicar-eval` | parity with v1 (`parity`), the evaluator (`evaluate`, `error_budget`) and checks; not published |
| `src/`, `python/replicar.py` | version 1, kept as the reference v2 is checked against |

```sh
cargo test --workspace --all-targets
cd python/v2 && PYTHONPATH=src python -m pytest tests
```

The evaluation tools refuse paths with a `test` component unless given `--final-assessment`.

- [RESULTS.md](RESULTS.md): measurements, experiments (including the ones that did not work), the test-split
  assessment, and the v2 parity checks.
- [docs/v2-plan.md](docs/v2-plan.md): the design of v2 and its stories. [PLAN.md](PLAN.md): plan and work log.
- [TEST_PROTOCOL.md](TEST_PROTOCOL.md): the protocol of the test-split run.
- [ROCKETSIM_NOTES.md](ROCKETSIM_NOTES.md): differences from the game found in RocketSim, for its developers.
- [AGENTS.md](AGENTS.md): guidance for contributors and coding agents. [data/README.md](data/README.md): the
  car-body to hitbox mapping.

The `replays/`, `collision_meshes/`, `external/` and `target/` folders are local and ignored by Git.

## Version 1

Version 1 (tag `v1-final`) is still in the repository as the package `replicar-v1`: `cargo run --release
--bin convert_replay -- my_match.replay my_match.parquet` writes its format ([docs/output-format.md](docs/output-format.md):
JSON Lines, or Parquet with seven record tables beside it), and `python/replicar.py` and
`python/replay_columnar.py` read it. v2 is checked against it stage by stage (`replicar-eval`'s `parity`), so it
stays until v2 is released.

## License

MIT, see [LICENSE](LICENSE). RocketSim and boxcars are MIT-licensed as well.
