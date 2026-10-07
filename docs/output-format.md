# Output format (v1)

This is version 1's format (`convert_replay`, package `replicar-v1`). Version 2 writes one Parquet file per replay: see
[v2-file-format.md](v2-file-format.md).

`convert_replay` writes JSON Lines (`.jsonl`) or Parquet (`.parquet`). Both carry the same information; Parquet adds typed columns for fast reads and writes seven record tables beside the main file. A null is always an unknown or inapplicable value, never zero.

## Units and timing

- Positions in Unreal units (UU), linear velocities in UU/s, angular velocities in rad/s. Rotation is the three RocketSim basis columns.
- The timeline is the replay time rounded to 120 Hz ticks from the first frame (`timeline_tick`). RocketSim's arena tick (`arena_tick`) advances only through continuous active play, so it can differ from the timeline tick.
- Replay packets are exact server states generated 0-4 ticks before the frame that carries them. The converter infers each packet's tick (offline, from chains of packets; `packet_lag_ticks` records the lag and its source), applies the packet at that tick, and reports every state at the frame time.

## JSON Lines

The first line is a `header` record: schema version, the replay's SHA-256, boxcars and RocketSim revisions, the conversion options, the car slots (slot, player key, team, body product id, hitbox), observation and conversion diagnostics (including `map_name`, announced mutator settings and `nonstandard_notes`), and the header `labels` (below). Then one `frame` record per network frame:

- `state`: the RocketSim state (ball, cars with their controls and timers, boost pads).
- `observations`: every replay field the converter reads, each as `{value, frame, source}` where `frame` is the frame of its last update: bodies, controls, boost, component counters, players (including `ping_raw`), teams, scores, clock.
- `scoreboard`: `period`, `clock_state` (pregame, countdown, kickoff = clock held until the first touch, running, expired = at 0 waiting for the ball to touch the ground, decided, goal_pause), fractional `seconds_remaining` and, in overtime, `overtime_seconds` counting up. Scores and the clock are exact; per-player stats can arrive up to about 1.8 s late on a client replay.
- `events`: replay events: `goal_scored_on` (once per team per post-goal phase), `demolish` (skip records with `repeat` true, which re-announce one), `dodge_refreshed` (a flip reset, from the car's `DodgesRefreshedCounter`).
- `pad_pickups`: the replay's pad records (skip `repeat` true); `boost_pickups`: new pickups with the pad, the car the replay names, and whether that car's path reaches the pad.
- `touches`: the first tick of each simulated car-ball contact. `ball_contacts`: contacts found from the ball packets alone (a velocity a ball-only rollout cannot reach), with the estimated tick and nearest car; prefer these as touch evidence. `simulated_events` keep RocketSim's per-tick events; do not add them to `touches`.
- `fitted_inputs`: inferred jump and dodge presses (tick, dodge direction and pitch cancel) and `air` rows for the intervals whose pitch/yaw/roll the boundary-value solve chose (`span_ticks`).
- `position_residuals`: the simulated state just before each fresh packet against that packet.
- Provenance lists: `dead_shell_held`, `spawn_pose_held`, `sleeping_velocity_inferred`, `demolition_inferred`.
- `labels` and `freshness` (below).

The controls on a car state are the action applied from that state: observed throttle, steer and handbrake (with their timing fitted offline), boost, jump and dodge inferred from the replicated counters, and in the air the per-interval pitch/yaw/roll of the boundary-value solve.

## Parquet

The main file has one row per frame, written in row groups of 512 frames:

- Time: `frame`, `replay_time`, `timeline_tick`, `arena_tick`.
- Ball and per-car-slot state: `ball_position`, `ball_velocity`, `ball_angular_velocity`, `ball_rotation_columns`, `car_position`, `car_velocity`, `car_angular_velocity`, `car_rotation_columns`, `car_boost`, `car_demoed`, `car_present`, `control_axes`, `control_buttons` (the Python loaders return their channel order as `control_axes_order` and `control_buttons_order`).
- Boost pads: `boost_pad_active`, `boost_pad_cooldown`. Scores: `scores`; clock: `seconds_remaining` (NaN convention), `scoreboard_period`, `scoreboard_clock_state`, `scoreboard_seconds_remaining`, `scoreboard_overtime_seconds`.
- `frame_json`: the complete JSON frame record.
- Provenance per car slot: `dead_shell_held` (1 observed goal-explosion hold, 2 inferred from a sleeping unlinked car, null not held) and `spawn_pose_held` (the car is shown at its spawn pose and kept out of collisions until its first packet).
- Labels, ping and freshness columns (below).

New columns are always appended; `columnar_version` stays 1. The replay header (`replay_header_json`) is in the file's key-value metadata, written at close: `read_columnar_header` reads it (PyArrow's `schema_arrow.metadata` only shows `columnar_version` and `pad_config_json`).

### Record tables

For `game.parquet`: `game.touches.parquet`, `game.ball_contacts.parquet`, `game.boost_pickups.parquet`, `game.fitted_inputs.parquet`, `game.packet_lags.parquet`, `game.events.parquet` and `game.pad_pickups.parquet`, one row per record with the `frame` it belongs to; always written, empty when a replay has none (`--no-event-tables` skips them and deletes old ones). Each table's metadata holds `source_sha256` and `options_sha256` (visible to `pq.read_table`) and `frames`; `read_record_tables` skips, with a warning, a table that does not match the main file. `events`, `packet_lags` and `pad_pickups` name cars by replay actor id and also give the car slot (`victim_slot`, `attacker_slot`, `car_slot`, `instigator_slot`), resolved through the car's linked player in that frame. [RESULTS.md](../RESULTS.md) ("Parquet columns and record tables") lists every column.

`python python/verify_direct_parquet.py game.jsonl game.parquet` checks a Parquet export, all record tables included, against the JSONL of the same replay.

## Training labels

`labels` (JSONL) and `label_*` columns (Parquet) are derived from the whole replay and are outputs only; each frame's `labels` says `"future_derived": true`. Never use them as model input; drop them as a block.

- `episode`: goal-to-goal segment index. An episode runs from the first in-play frame of a kickoff to the frame that reports its goal, or to the last in-play frame when no goal follows (including the end of regulation before overtime). Frames outside play have a null episode.
- `episode_seconds_remaining`: replay time to the end of the episode (0 at the goal frame).
- `next_scoring_team` (0 blue, 1 orange) and `seconds_until_next_goal`: the first observed goal at or after the frame, in any episode (in goal pauses and tails this may be the next episode's goal or an overtime goal); null when none follows.
- Header `labels`: `final_score`, `winning_team` (null for a draw or unknown score), `episodes`, `observed_goals`, from the replay's own last scoreboard; never per frame.

## Freshness and ping

`freshness` (JSONL) and its Parquet columns are observed or inferred, never future-derived:

- `ball_fresh`, `car_fresh` (per slot; null for a slot with no resolvable car): a fresh rigid-body packet was applied at this frame. They also count frames the converter does not simulate, so they are a superset of the `packet_lags` rows.
- `ball_update_age_seconds`, `car_update_age_seconds`: frame time minus the time of the frame with the body's last packet.
- `ball_packet_age_ticks`, `car_packet_age_ticks`: the frame's timeline tick minus the inferred server tick of the last applied packet (normally 0-4 at a fresh frame, more when a frame gap exceeds 4 ticks, growing between packets); null where no lag was inferred.
- `ping_raw` per player (Parquet per car slot), the raw `Engine.PlayerReplicationInfo:Ping` byte (probably milliseconds / 4, uncalibrated), with `ping_age_seconds`. Null until a player's first update; the host and bots of a host or LAN replay never have one.

The Python loaders return the masks as int8 (1, 0, -1 unknown), ages as float32 with NaN, and tick ages as int32 with -1.

## Restoring a RocketSim state

`restoration::car_slots_from_header_json`, `restoration::state_from_frame_json` and `restoration::restore_soccar_state` rebuild a detached RocketSim `ArenaState` from an exported frame. `apply_soccar_state_to_arena` seeds a live arena: it rebases tick-valued fields (a car's `last_extra_hit_tick`) to the live arena's clock, but a live arena keeps its own tick, RNG and private caches, so continuing from it is not an exact replay continuation. `verify_state_restoration <file.parquet>` checks every frame of a Parquet export.
