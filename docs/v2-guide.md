# replicar v2: guide

replicar reconstructs Rocket League replays as [RocketSim](https://github.com/ZealanL/RocketSim) states: one row per
replay frame in play, with the ball and every car's full physics state, controls, boost, pads, the scoreboard, events,
ball contacts and boost pickups, and how each value is known. It writes one ordinary Parquet file per replay. The
words are defined in [glossary.md](glossary.md); every column is listed in [v2-file-format.md](v2-file-format.md).

## How it works, in one paragraph

A replay's network frames carry exact server states of the ball and the cars, but only every few ticks, a few ticks
late, and without the players' pitch, yaw and roll or the exact moments of their jumps and dodges. replicar decodes
the frames, infers the 120 Hz tick of every update from the chained motion of the bodies, and simulates the match
in RocketSim between updates; where the replay does not say what happened, an inference fits it against later
updates (control timing, presses, dodge direction and flip cancel, air controls, and a tick or two of alignment
before ball hits). The simulation applies each update at its tick, so every frame has a full state that agrees with
the replay wherever the replay says something.

## The command

You need RocketSim's soccar collision meshes (`collision_meshes/soccar/*.cmf`, which come from the game); pass the
folder with `--meshes DIR`, set `REPLICAR_MESHES`, or run where `collision_meshes/` is.

```sh
cargo build --release -p replicar-cli          # target/release/replicar

replicar convert match.replay -o match.parquet
replicar convert replays/ -o out/ --jobs 16    # one file per replay (subfolders mirrored) + out/index.parquet
replicar inspect match.parquet                 # header, players, segments
replicar verify match.parquet --replay match.replay
```

Options of `convert` and `resimulate`:

| Option | Meaning |
| --- | --- |
| `-o, --output` | the output file, or folder for a folder input |
| `--precision float32\|quantized` | how the state's bodies are stored; quantized files are about a quarter smaller |
| `--with GROUPS` | groups added to the default ones: `resimulation`, `network`, `diagnostics` |
| `--groups GROUPS` | the whole set of groups instead (for example `game,updates,future,resimulation`) |
| `--all-frames` | also the frames outside play segments (countdowns, goal pauses and replays) |
| `--jobs N` | replays converted at once (folder input; default: every core) |
| `--skip-existing` | leave replays whose output exists (folder input; resumes a run) |
| `--meshes DIR` | RocketSim's collision meshes |
| `--replay FILE` | `resimulate` and `verify`: the replay the file was converted from |

`replicar resimulate match.parquet --replay match.replay -o full.parquet` rebuilds a file from its
`resimulation` group without fitting: the same states, about nine times faster than converting. A file with
`--groups game,updates,future,resimulation` is about a third of the full one; resimulate it when you need the
states.

## Python

```sh
pip install ./python/v2                        # the reader: pyarrow and NumPy only
maturin build --release -m crates/replicar-python/Cargo.toml -o target/wheels
pip install target/wheels/replicar_native-*.whl  # the native extra: convert and resimulate from Python
```

```python
import replicar

f = replicar.read("match.parquet")
f.header["players"]                  # index, name, team, hitbox
a = f.arrays()                       # NumPy, float32 with NaN for unknown, -1 for unknown integers
a["car_position"]                    # (frames, players, 3)
a["car_rotation"]                    # (frames, players, 4): quaternion x, y, z, w
a["clock_phase"], a["future_segment_end"]
f.records("ball_contacts")           # a pyarrow table, one row per contact, with its frame

replicar.convert("match.replay", "match.parquet", precision="quantized")          # native extra
replicar.convert_many(paths, "out/", jobs=16)
f = replicar.read("light.parquet", replay="match.replay")  # a file without states: resimulated on reading
```

Players and statistics in long form: `f.players_table()` (one row per player with the final statistics) and
`f.long("car")` (one row per frame and player, `car_0_boost` as `car_boost`), both pyarrow tables
(`.to_pandas()` for pandas). From the command line, `replicar inspect --players match.parquet` prints the players
as CSV. In DuckDB the players come from the header:

```sql
WITH players AS (
  SELECT unnest(from_json(decode(value), '{"players": [{"index": "UTINYINT", "name": "VARCHAR", "team": "UTINYINT"}]}').players, recursive := true)
  FROM parquet_kv_metadata('match.parquet') WHERE decode(key) = 'replicar'
)
SELECT p.name, s.s.kind AS stat, count(*) AS n
FROM (SELECT unnest(stat_events) AS s FROM 'match.parquet') s
JOIN players p ON p.index = s.s.player
GROUP BY ALL ORDER BY ALL;
```

### Continuing in RocketSim

`replicar.rocketsim` puts a row into mtheall's RocketSim bindings (`pip install rocketsim`, the module `RocketSim`
that RLGym steps), to continue the match from any frame:

```python
import RocketSim, replicar, replicar.rocketsim

RocketSim.init("collision_meshes")
f = replicar.read("match.parquet")
arena, cars = replicar.rocketsim.arena(f, row=1200)   # a new soccar arena; cars by player index
arena.step(8)
replicar.rocketsim.set_state(arena, cars, f, row=1300)  # reuse it: ball, cars (state and controls), pads
replicar.rocketsim.car_state(f, 1200, player=0)        # or one RocketSim.CarState, BallState, CarControls
```

The cars get the players' hitboxes and teams; a player without a car in the row gets none. The bindings are the C++
RocketSim, replicar simulates with its Rust port: they model the same game but are not the same simulator. Checked with
`scripts/check_rocketsim_bridge.py` on 12 train and 12 validation files: the state reads back from the arena as the
file has it (within 0.0005), and one frame ahead (stepping the row's controls to the next row's tick, for bodies the
next row does not update and controls that are constant over the interval) the ball is within 0.05 UU of the file
(p99) and a car within 0.15-0.27 UU (p50) and 2.5 UU (p99), against 30-50 UU (p50) for not stepping. The difference
grows with time. Left at the bindings' defaults, because a row does not have them: the ball's heatseeker state, a
car's flip-reset flags, the car its bump cooldown is for, and the tick of its last extra ball-hit impulse. The
bindings have no `psyclops` hitbox: `arena(..., hitboxes={"psyclops": "OCTANE"})` accepts a stand-in.

Any Parquet reader works without the package: `pyarrow.parquet.read_table`, `polars.read_parquet`, DuckDB's
`read_parquet`; quantized columns then come as integers with their `scale` in the field metadata.

## Rust

```rust
let meshes = replicar::Meshes::load("collision_meshes")?;           // once per process
let converter = replicar::Converter::new(&meshes, replicar::Config::default());
let conversion = converter.convert(&std::fs::read("match.replay")?)?;
conversion.write("match.parquet".as_ref(), &replicar_format::WriteOptions::default())?;

// Every simulated frame with its full RocketSim state, as it is made:
let network = replicar::decode::decode(&replicar::parse(&bytes)?)?;
converter.convert_network_with(network, |frame| { /* frame.state: rocketsim::ArenaState */ })?;

// RocketSim states back from a file (rotations within 1e-6), or exactly by resimulating:
let states = replicar::restore::arena_states("match.parquet".as_ref())?;
let again = converter.resimulate(&bytes, "match.parquet".as_ref())?;
```

The crates: `replicar-format` (the file: rows, header, writer, reader; no parser, no simulator), `replicar` (decode,
update ticks, simulate, infer, annotate, convert), `replicar-cli` (the command), `replicar-python` (the native
extra) and `replicar-eval` (parity with v1 and evaluation tools; not published).

## What to rely on

- Every value of the state at a frame with an update is the replay's exact server state at the update's tick, moved
  to the frame's time by the simulation. Between updates the state is RocketSim's prediction with the inferred
  inputs: on held-out test replays the car position just before an update is off by 0.04 / 3.5 / 35 UU (median /
  90th / 99th percentile; RESULTS.md).
- The `future` columns read later frames by construction: never use them as model inputs.
- The final score is in the header, never in a row.
