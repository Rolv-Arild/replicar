# replicar

*Replica cars from Rocket League replays.* replicar reconstructs Rocket League replays as
[RocketSim](https://github.com/ZealanL/RocketSim) states: one row per 120 Hz simulation tick in play, with the ball and
every car's full physics state, the controls applied at that tick and what set them, boost and pads, the scoreboard,
events, the players' match statistics as they happen, ball contacts and boost pickups, and how each value is known.
It writes one ordinary Parquet file per replay; `--tick-step N` keeps every N-th tick and `--rows frames` one row
per replay frame.

Replay packets are exact server states, placed on their inferred 120 Hz tick; RocketSim simulates every tick in
between, and the inputs a replay does not record (jump, dodge and control timing, flip cancels, air pitch, yaw and
roll) are fitted against later packets. On 60 held-out test replays every replay converted, and the car position predicted
just before each replay packet is off by 0.03 / 3.0 / 35 UU (median / 90th / 99th percentile; second, labelled run;
[RESULTS.md](RESULTS.md), "Test-split assessment, second run").

This is version 2: a rewrite of version 1 with a more accurate reconstruction of the inputs between frames (jump
holds, analog sticks, steer in the air; held-out car velocity p90 about 11% lower) and much more in each file (see
[CHANGELOG.md](CHANGELOG.md)).

## Quick start

Requirements: Rust 1.97 or newer on Linux, Windows or macOS (Linux and Windows are tested), and RocketSim's soccar
collision meshes (`collision_meshes/soccar/*.cmf`, which come from the game). Pass the folder with `--meshes DIR`,
set `REPLICAR_MESHES`, or run where `collision_meshes/` is.

```sh
cargo build --release -p replicar-cli

./target/release/replicar convert my_match.replay -o my_match.parquet
./target/release/replicar convert replays/ -o out/ --jobs 16      # a folder: one file per replay + out/index.parquet
./target/release/replicar inspect my_match.parquet               # --players: the players and their statistics as CSV
```

| Option | Effect |
| --- | --- |
| `--rows frames` | one row per replay frame (about 30 per second) instead of one per tick |
| `--tick-step N` | every N-th tick (`sim_tick` a multiple of N); 8 gives 15 rows per second |
| `--precision quantized` | integers for the bodies (0.01 UU, 0.01 UU/s, 1e-4 rad/s): about a quarter smaller |
| `--with resimulation,network,diagnostics` | the optional groups |
| `--groups game,updates,future,resimulation` | a file without states, from which `replicar resimulate` rebuilds them exactly |
| `--all-frames` | also the frames outside play (countdowns, goal pauses and replays) |

A long 3v3 replay (1.6 MB) takes about 5 s on one core and 330 MB of memory; 120 replays at 32 jobs on a 16-core
machine take 39 s. Files are about 8 MB per replay with tick rows, 3 MB with frame rows and 1.7 MB with
`--tick-step 8` (RESULTS.md, "v2: a row per tick").

## Reading the files

Any Parquet reader opens them (pyarrow, polars, DuckDB). The Python package adds NumPy arrays:

```sh
pip install ./python/v2          # pyarrow and NumPy only
```

```python
import replicar

f = replicar.read("my_match.parquet")
f.header["players"]              # index, name, team, hitbox, final statistics
a = f.arrays()
a["car_position"]                # (rows, players, 3) float32, NaN when unknown
a["car_controls_throttle"]       # (rows, players): the control applied at each tick
a["clock_phase"], a["future_segment_end"]
f.records("stat_events")         # one row per save, shot, goal, clear, ... with its frame
f.players_table(), f.long("car") # long tables for pandas and SQL
```

Converting from Python needs the native extra (`crates/replicar-python`, built with maturin): `replicar.convert`,
`replicar.convert_many`, `replicar.resimulate`. `replicar.rocketsim` puts a row into mtheall's RocketSim bindings
and `replicar.rlgym` makes it an RLGym `GameState`, to continue a match or start an environment from any tick.

## What a file contains

| Group | Default | Contents |
| --- | --- | --- |
| (always) | yes | `frame`, `frame_row`, `segment`, `replay_time`, `replay_tick`, `sim_tick` |
| `state` | yes | ball and car physics, controls and their sources, boost, RocketSim's car internals, pads, car status |
| `game` | yes | period, clock phase, fractional clock, scores, events (goals with scorer and assister), stat events, ball contacts, boost pickups |
| `updates` | yes | which bodies were updated, their update ticks, ticks and seconds since the last update, ping |
| `future` | yes | how the play segment ends and the time until then; future-derived, never a model input |
| `resimulation` | no | what the inference chose; with the replay, enough to rebuild `state` exactly |
| `network` | no | the replay's values as decoded, each with the frame of its last change |
| `diagnostics` | no | prediction errors before each correction, RocketSim's events, simulated touches |

Per-player columns carry the player index (`car_0_position_x`); the header (JSON in the Parquet metadata) lists the
players with their final statistics, the statistics the replay's build counts, the pad layout, the play segments, the
final score, the rows, the platform and the conversion's settings. Every column is in
[docs/v2-file-format.md](docs/v2-file-format.md); the words are defined in [docs/glossary.md](docs/glossary.md);
[docs/v2-guide.md](docs/v2-guide.md) explains how it works and how to use it from Python and Rust.

## Using it from Rust

```rust
let meshes = replicar::Meshes::load("collision_meshes")?;
let converter = replicar::Converter::new(&meshes, replicar::Config::default());
let conversion = converter.convert(&std::fs::read("my_match.replay")?)?;
conversion.write("my_match.parquet".as_ref(), &replicar_format::WriteOptions::default())?;
```

`Converter::with_rows` keeps fewer rows in memory for a frame or tick-step file; `Converter::convert_network_with`
hands every simulated frame (its RocketSim `ArenaState`) to a callback; `replicar::restore::arena_states` rebuilds
RocketSim states from a file; `Converter::resimulate` reproduces them exactly from the replay and the `resimulation`
group.

## Limits

- Air pitch, yaw and roll are not in the replay; they are fitted to reproduce the motion between packets (per
  tick where a schedule is solved, else per packet gap), not the player's stick. `car_<i>_air_controls_source`
  says which fit set them.
- The fits use later packets: this is an offline reconstruction, not a causal predictor. A tick row between two
  frames is simulated toward the next frame's packets.
- Only standard soccar is supported.
- Ground truth for timing, inputs and events comes from two LAN games recorded with RLBot; online replays with real
  latency are checked only indirectly.
- Some state is inferred and labelled (`car_status_inferred`): wrecks held out of play, spawn poses before a car's
  first packet.
- The states differ in the last bit between platforms (RocketSim uses the platform's maths library), so a file is
  resimulated only on the platform that converted it.

## Layout and development

| Path | What |
| --- | --- |
| `crates/replicar-format` | the file: rows, header, writer, reader (no parser, no simulator) |
| `crates/replicar` | the library: decode, update ticks, simulate, infer, annotate, convert, restore |
| `crates/replicar-cli` | the `replicar` command |
| `crates/replicar-python`, `python/v2` | the Python package, its native extra and its RocketSim and RLGym bridges |
| `crates/replicar-eval` | the evaluator (`evaluate`, `error_budget`), parity with v1 (`parity`) and checks; not published |
| `src/`, `python/replicar.py` | version 1, kept as a reference |

```sh
cargo test --workspace --all-targets     # building the Python module's crate needs a Python 3.12+ (PYO3_PYTHON)
cd python/v2 && PYTHONPATH=src python -m pytest tests
```

CI (`.github/workflows/ci.yml`) runs the format check, clippy and every test on Linux and Windows; the tests that need
collision meshes or replays skip there. `.github/workflows/release.yml` builds the native wheels (Linux x86_64 and
aarch64 manylinux_2_28, Windows x86_64, macOS arm64 and x86_64; one abi3 wheel each for CPython 3.12+), the reader's
wheel and sdist, and the `replicar` command per platform, as artifacts; it publishes nothing.

The evaluation tools refuse paths with a `test` component unless given `--final-assessment`.

- [RESULTS.md](RESULTS.md): measurements, experiments (including the ones that did not work), the test-split
  assessments, and the v2 checks.
- [docs/v2-plan.md](docs/v2-plan.md): the design of v2 and its stories. [PLAN.md](PLAN.md): plan and work log.
- [TEST_PROTOCOL.md](TEST_PROTOCOL.md): the protocol of the test-split runs.
- [ROCKETSIM_NOTES.md](ROCKETSIM_NOTES.md): differences from the game found in RocketSim, for its developers.
- [AGENTS.md](AGENTS.md): guidance for contributors and coding agents. [data/README.md](data/README.md): the
  car-body to hitbox mapping.

The `replays/`, `collision_meshes/`, `external/` and `target/` folders are local and ignored by Git.

## Version 1

Version 1 (tag `v1-final`) is still in the repository as the package `replicar-v1`: `cargo run --release
--bin convert_replay -- my_match.replay my_match.parquet` writes its format ([docs/output-format.md](docs/output-format.md):
JSON Lines, or Parquet with seven record tables beside it), and `python/replicar.py` and
`python/replay_columnar.py` read it. v2 was checked against it stage by stage (`replicar-eval`'s `parity`); v2's
evaluator reproduces v1's held-out reports byte for byte. v2 differs from it by design in a flipping car's air-control
segments, the inputs between frames, the per-tick rows and the controls a row holds (CHANGELOG.md).

## License

MIT, see [LICENSE](LICENSE). RocketSim and boxcars are MIT-licensed as well.
