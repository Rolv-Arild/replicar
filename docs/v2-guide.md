# replicar v2: guide

replicar reconstructs Rocket League replays as [RocketSim](https://github.com/ZealanL/RocketSim) states: one row per
120 Hz simulation tick in play (or per replay frame), with the ball and every car's full physics state, the controls
applied at that tick, boost, pads, the scoreboard, events, ball contacts and boost pickups, and how each value is
known. It writes one ordinary Parquet file per replay. The words are defined in [glossary.md](glossary.md); every
column is listed in [v2-file-format.md](v2-file-format.md).

## How it works, in one paragraph

A replay's network frames carry exact server states of the ball and the cars, but only every few ticks, a few ticks
late, and without the players' pitch, yaw and roll or the exact moments of their jumps and dodges. replicar decodes
the frames, infers the 120 Hz tick of every update from the chained motion of the bodies, and simulates the match
in RocketSim between updates; where the replay does not say what happened, an inference fits it against later
updates (control timing, presses, dodge direction and flip cancel, air controls, and a tick or two of alignment
before ball hits). The simulation applies each update at its tick, so every frame has a full state that agrees with
the replay wherever the replay says something.

## What a row is

By default a file has a row for every tick RocketSim simulated in play: the replay frames' own ticks (`frame_row`
true, about one in four) and the ticks between them. A row's state is the simulation's at its tick, and its controls
are the ones RocketSim applied in the step after it: the action taken at that state, with the fitted jump and dodge
presses, control timing and air controls at the ticks the fits put them. `car_<i>_air_controls_source` (`none`,
`steer`, `persisted`, `lookahead`, `schedule`, `press`, `dodge`, `flip_cancel`) and `car_<i>_ground_controls_source`
(`network`, `schedule`, `dodge`) say what set them; `lookahead` and `schedule` used the car's next update.

A tick row belongs to the frame that ends its interval (`frame`): its state is simulated toward that frame's updates,
which are applied at their update ticks inside the interval, so a tick row can show an update a few ticks before the
frame that carries it (offline reconstruction). What the replay reports per frame is not repeated early: a tick
row's scoreboard, clock and ping are the previous frame's, its events and statistics are on the frame rows, its
update ages count from the update ticks, and its seconds since update are null. `--tick-step N` keeps the rows whose
`sim_tick` is a multiple of N and passes the records and update flags of the others to the next row kept;
`--rows frames` keeps the frame rows only.

## Statistics and events

`events` holds what the replay reports: goals (with the `scorer` and `assister`), demolitions and flip resets.
`stat_events` holds each player's match counters going up, in the frame the replay updates them: goals, assists,
saves, shots and demolitions, and in replays from the builds of September 2026 on also epic saves, clears, centers,
aerial hits, first touches, crossbar, bicycle and juggle hits, flip resets and times demolished. They are the game's
own judgement, seen with the replay's delay; an assist credited during the goal pause is kept on the last row before
it, with its `updated_frame`. A player's events add up to the header's `final_stats`. The header's `counted_stats`
lists the statistics the replay's build counts: for those a player without any has 0; the others are unknown, not 0.

## Resources

On a 16-core machine (AMD 5950X, Windows), 120 train and validation replays convert at 32 jobs in 39 s with tick
rows (6.1 GB peak for the 32 conversions in flight), 35.5 s with frame rows (4.1 GB); a long 3v3 replay alone takes
about 5 s and 330 MB. Files: about 8.5 MB per replay with tick rows, 3.2 MB with frame rows, 1.7 MB with
`--tick-step 8`; quantized about a quarter less. For 140,000 replays that is about 12 h and 1.2 TB with tick rows.

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
| `--rows ticks\|frames` | a row per simulated 120 Hz tick in play (default) or per replay frame |
| `--tick-step N` | with tick rows: only the ticks whose `sim_tick` is a multiple of N (8: 15 rows per second) |
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
a["car_position"]                    # (rows, players, 3): a row per tick, or per frame
a["car_rotation"]                    # (rows, players, 4): quaternion x, y, z, w
a["clock_phase"], a["future_segment_end"]
f.records("ball_contacts")           # a pyarrow table, one row per contact, with its frame

replicar.convert("match.replay", "match.parquet", precision="quantized")          # native extra
replicar.convert_many(paths, "out/", jobs=16)
f = replicar.read("light.parquet", replay="match.replay")  # a file without states: resimulated on reading
```

Players and statistics in long form: `f.players_table()` (one row per player with the final statistics) and
`f.long("car")` (one row per file row and player, `car_0_boost` as `car_boost`), both pyarrow tables
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

`replicar.rocketsim` puts a row into mtheall's RocketSim bindings, with everything the row has (`pip install rocketsim`, the module `RocketSim`
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
`scripts/check_rocketsim_bridge.py`: the state reads back from the arena as the file has it (within 0.0005), and one
tick ahead in a file of tick rows (stepping a row's controls, for bodies the next row does not update) every car is
within 0.03 UU (p50) and 0.62 UU (p99) of the next row and the ball within 0.04 UU (p99), against 12-13 UU (p50) for
not stepping. The difference grows with time. Left at the bindings' defaults, because a row does not have them: the ball's heatseeker state, a
car's flip-reset flags, the car its bump cooldown is for, and the tick of its last extra ball-hit impulse. The
bindings have no `psyclops` hitbox: `arena(..., hitboxes={"psyclops": "OCTANE"})` accepts a stand-in.

Fields whose meaning differs between the port and the bindings (measured by stepping both, 2026-10-08): `jump_time`
(set from `car_jump_ticks` while the car has jumped, else 0; after the jump the port counts from its end, the
bindings from its start), `flip_time` (the port's stops at the end of the flip, the bindings' keeps counting while
airborne), `supersonic_time` (the port's grace timer counts time below 2,200 UU/s, the bindings' time since the car
became supersonic) and `time_since_boosted` (not in the bindings' `CarState`). A model that sees both should not rely
on these. **Air throttle while boosting:** in the game and in the port an airborne car that boosts accelerates as if
throttle were 1, whatever the throttle (LAN truth: 1,057.7 UU/s^2 at throttle 0 and 1); the bindings add the throttle
on top (1,058.3 at 0, 1,125 at 1, 991.7 at -1). To step a row in the bindings as the game would, set the throttle to
0 while an airborne car boosts.

For RLGym 2, `replicar.rlgym.game_state(f, row)` is the row as a `GameState`, for `RocketSimEngine.set_state` or a
state mutator, so an environment can start from any replay frame:

```python
import replicar.rlgym

state = replicar.rlgym.game_state(f, row=1200, agent_ids={p["index"]: p["name"] for p in f.players})
engine.set_state(state, {})       # rlgym.rocket_league.sim.RocketSimEngine
```

A `GameState` holds less than the row: no controls (the environment's actions supply them), of the previous
controls only the jump, no air time, world contact or bump cooldown. Neither bridge steps anything: they set states.
The pad timers follow RLGym's `BOOST_LOCATIONS` (matched to the file's pads by position; RLGym's table has one pad
2 UU from RocketSim's). Checked on 600 train rows: through `RocketSimEngine.set_state` and back, positions, velocities,
rotations, boost, flags and pad timers come back as set (within 0.0005).

Any Parquet reader works without the package: `pyarrow.parquet.read_table`, `polars.read_parquet`, DuckDB's
`read_parquet`; quantized columns then come as integers with their `scale` in the field metadata.

## Rust

```rust
let meshes = replicar::Meshes::load("collision_meshes")?;           // once per process
let converter = replicar::Converter::new(&meshes, replicar::Config::default());
let conversion = converter.convert(&std::fs::read("match.replay")?)?;
conversion.write("match.parquet".as_ref(), &replicar_format::WriteOptions::default())?;

// A file of fewer rows: keep only those in memory (writing rows the conversion did not keep is refused).
let rows = replicar_format::RowRate::Ticks(8);
let converter = replicar::Converter::new(&meshes, replicar::Config::default()).with_rows(rows);
let options = replicar_format::WriteOptions { rows, ..Default::default() };

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

- At an update's tick the state is the replay's exact server state; from there to the next update it is RocketSim's
  prediction with the inferred inputs: on held-out test replays the car position just before an update is off by
  0.03 / 3.0 / 35 UU (median / 90th / 99th percentile; RESULTS.md, second test run). At the tick an update lands a body can jump by
  that error.
- Observed and inferred are kept apart: the network values and the replay's events and statistics are observed;
  update ticks, air controls, presses, control timing, spawn poses and held wrecks are inferred and say so
  (`car_status_inferred`, the controls sources, the `resimulation` group).
- The `future` columns read later frames by construction: never use them as model inputs.
- A file converted on one platform (`x86_64-windows`, `x86_64-linux`, ...; the header's `platform`) resimulates only
  on the same platform: the platforms' maths libraries differ in the last bit and RocketSim uses them every tick.
  The states agree closely (Windows against Linux: about 99% of rows bit-identical, the rest within 2 UU), but not
  exactly, and resimulation refuses rather than give other states. Convert where you will resimulate.
- The final score is in the header, never in a row.
