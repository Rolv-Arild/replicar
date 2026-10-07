# The replicar file format (v2, format version 1)

A replicar file is one ordinary Parquet file per replay. Any Parquet reader opens it (pyarrow, polars, DuckDB);
the `replicar` Python package adds NumPy arrays and record tables (`python/v2`). The words are defined in
[glossary.md](glossary.md); the measurements behind the choices are in RESULTS.md ("v2: the file format" and the
sections after it). `scripts/check_file_format_doc.py` checks that every column of a file is listed here.

## Rows

By default one row per simulated tick **in a play segment** (from the kickoff to the frame that reports the goal, or
to the last frame in play): the replay frames' own ticks (`frame_row` true) and the ticks RocketSim stepped between
them (glossary, "Tick row"). `--tick-step N` keeps the ticks whose `sim_tick` is a multiple of N; `--rows frames`
keeps only the frame rows. `--all-frames` adds the frames outside play, with a null `segment` (they have no ticks).
Rows are in tick order; `frame` is the replay frame a row belongs to (a tick row: the frame that ends its interval),
so gaps show where pauses were left out. The header's `rows` and `tick_step` say which. A record (event, stat event,
ball contact, boost pickup) and an update flag of a row left out pass to the next written row of its segment.

## Columns

Every column is nullable: null means unknown or not applicable, never zero. Its field metadata has its `group`
and, where it has one, its `unit` (`UU`, `UU/s`, `rad/s`, `s`, `tick`, `ticks`, `quaternion`, `0-100`). Per-player
columns carry the player index after their first word (`car_0_position_x`, `player_1_ping_raw`; the header's
`players` says who each index is), per-pad columns the pad index (`pad_3_cooldown`; the header's `pads` gives the
layout). Below, `<i>` is a player index, `<k>` a pad index, and `{x,y,z}` one column per component. Name columns
are dictionary-encoded strings.

### Always

| Column | Type | Meaning |
| --- | --- | --- |
| `frame` | uint32 | the replay frame (of a tick row: the frame whose interval it is in) |
| `frame_row` | bool | the row is its frame's own tick, not a tick between frames |
| `segment` | uint32 | the play segment, 0.. (null outside play) |
| `replay_time` | float32, s | the frame's time as the replay stores it (a tick row: its frame's, less the ticks to it) |
| `replay_tick` | uint32, tick | replay time on the 120 Hz scale from the first frame |
| `sim_tick` | uint64, tick | RocketSim's tick count; advances only while play is simulated |

### `state` (default)

| Column | Type | Meaning |
| --- | --- | --- |
| `ball_position_{x,y,z}` | float32, UU | |
| `ball_velocity_{x,y,z}` | float32, UU/s | |
| `ball_angular_velocity_{x,y,z}` | float32, rad/s | |
| `ball_rotation_{x,y,z,w}` | float32 | unit quaternion with w >= 0 |
| `ball_ticks_since_kickoff` | uint64, tick | RocketSim's ball counter |
| `car_<i>_status` | name | `absent`, `active`, `spawning`, `demolished` |
| `car_<i>_status_inferred` | bool | the status is replicar's inference |
| `car_<i>_air_controls_source` | name | what set the row's pitch, yaw and roll: `none` (not inferred, 0), `steer`, `persisted`, `lookahead`, `schedule`, `press`, `dodge`, `flip_cancel` (glossary, "Controls source") |
| `car_<i>_ground_controls_source` | name | what set the row's throttle, steer, handbrake and boost: `network`, `schedule`, `dodge` |
| `car_<i>_position_{x,y,z}`, `car_<i>_velocity_{x,y,z}`, `car_<i>_angular_velocity_{x,y,z}`, `car_<i>_rotation_{x,y,z,w}` | float32 | as for the ball |
| `car_<i>_boost` | float32, 0-100 | |
| `car_<i>_controls_{throttle,steer,pitch,yaw,roll}` | float32 | the controls RocketSim applied in the step after the row's tick |
| `car_<i>_controls_{jump,boost,handbrake}` | bool | |
| `car_<i>_previous_controls_{throttle,steer,pitch,yaw,roll}`, `car_<i>_previous_controls_{jump,boost,handbrake}` | | RocketSim's controls of the tick before |
| `car_<i>_{is_on_ground,has_jumped,has_double_jumped,has_flipped,is_flipping,is_jumping,is_boosting,is_supersonic,is_auto_flipping,is_demoed}` | bool | RocketSim's car flags |
| `car_<i>_wheel_{0,1,2,3}_contact` | bool | the wheel touches a surface |
| `car_<i>_{flip_time,air_time,air_time_since_jump,time_since_boosted,boosting_time,supersonic_grace_timer,handbrake_value,auto_flip_timer,auto_flip_torque_scale,bump_cooldown_timer,demo_respawn_timer}` | float32 | RocketSim's car timers and values |
| `car_<i>_jump_ticks` | uint32, tick | |
| `car_<i>_flip_relative_torque_{x,y,z}`, `car_<i>_world_contact_normal_{x,y,z}` | float32 | |
| `car_<i>_last_extra_hit_tick` | uint64, tick | the sim tick of the car's last extra ball-hit impulse |
| `pad_<k>_cooldown` | float32, s | seconds until the pad is available again (0: available) |

With `--precision quantized` the ball's and cars' position, velocity and angular velocity are int32 and their
rotation components int16; the field metadata `scale` (0.01 UU, 0.01 UU/s, 1e-4 rad/s, 1/32767) turns them back
into floats (the Python reader does it), and every other column is unchanged.

### `game` (default)

| Column | Type | Meaning |
| --- | --- | --- |
| `period` | name | `regulation`, `overtime` |
| `clock_phase` | name | `pregame`, `countdown`, `kickoff`, `running`, `expired`, `decided`, `goal_pause`, `other` |
| `seconds_remaining` | float32, s | the regulation clock, fractional |
| `overtime_seconds` | float32, s | overtime played |
| `blue_score`, `orange_score` | int32 | as the replay shows them at the frame |
| `events` | list of records | `kind` (`goal`, `demolition`, `flip_reset`), `scoring_team`, `scorer`, `assister`, `attacker`, `victim`, `repeat`, `goal_explosion`, `player` |
| `stat_events` | list of records | a player's match counter going up: `kind` (`goal`, `assist`, `save`, `shot`, `demolition`; since September 2026 also `epic_save`, `clear`, `center`, `aerial_hit`, `first_touch`, `crossbar_hit`, `bicycle_hit`, `juggle_hit`, `flip_reset`, `demolished`), `player`, `total`, `updated_frame` (the frame the counter went up in: this row's or, when that frame is left out, a later one) |
| `ball_contacts` | list of records | `replay_tick`, `from_tick`, `to_tick`, `player`, `gap`, `velocity_residual` |
| `boost_pickups` | list of records | `pad`, `is_big`, `player`, `verified`, `suggested_player`, `replay_tick` |

### `updates` (default)

| Column | Type | Meaning |
| --- | --- | --- |
| `ball_updated`, `car_<i>_updated` | bool | the body got an update since the previous row (a tick row: the update was applied at its tick or since the last written row) |
| `ball_update_tick`, `car_<i>_update_tick` | uint32, tick | the replay tick the body's last update shows (inferred) |
| `ball_ticks_since_update`, `car_<i>_ticks_since_update` | uint32, ticks | replay tick minus update tick |
| `ball_seconds_since_update`, `car_<i>_seconds_since_update` | float32, s | time since the frame that carried the last update (frame rows only) |
| `player_<i>_ping_raw` | uint8 | the ping byte as the replay sends it |

### `future` (default; future-derived)

| Column | Type | Meaning |
| --- | --- | --- |
| `future_segment_end` | name | `blue_goal`, `orange_goal`, `time_expired`, `replay_ended`, `other` |
| `future_seconds_until_segment_end` | float32, s | replay time to the segment's last frame |

### `resimulation` (opt-in)

Record lists that, with the replay, let `replicar resimulate` rebuild `state` exactly without fitting. Each entry
carries its replay frame and sits in the row of the first written frame at or after it.

| Column | Records |
| --- | --- |
| `resim_update_ticks` | `frame`, `ball`, `car_median`: ticks before the frame of the ball's and the median car's update |
| `resim_car_update_ticks` | `frame`, `actor`, `created`, `ticks`: one car update's ticks before its frame |
| `resim_choices` | `frame`, `actor`, `created`, `question`, `ordinal`, `repeat`, `value_0`-`value_2`, `integer`, `player`, `end_tick`, `entries` (per-tick controls), `dodge` |

### `network` (opt-in)

The replay's values as replicar decodes them, in the replay's units, each with `<name>_frame`, the frame of its
last change. Per player, the player's current car in the frame.

| Column | Meaning |
| --- | --- |
| `network_seconds_remaining`, `network_overtime`, `network_blue_score`, `network_orange_score`, `network_game_state` (and their `_frame`) | the integer clock, the overtime flag, the scores, the game state |
| `network_ball_{position,velocity,angular_velocity_raw}_{x,y,z}`, `network_ball_rotation_{x,y,z,w}`, `network_ball_sleeping` (and `_frame`), `network_ball_{position,velocity,angular_velocity_raw,rotation}_frame` (one per vector) | the ball's body |
| `network_car_<i>_{actor,created,player_link_active}` | the current car: its actor and creation frame, and whether it links to its player |
| `network_car_<i>_{position,velocity,angular_velocity_raw}_{x,y,z}`, `network_car_<i>_rotation_{x,y,z,w}`, `network_car_<i>_sleeping` (and `_frame`), `network_car_<i>_{position,velocity,angular_velocity_raw,rotation,dodge_torque_raw}_frame` (one per vector) | its body |
| `network_car_<i>_{boost,throttle,steer,handbrake,boost_raw,boost_active_raw,jump_active_raw,double_jump_active_raw,dodge_active_raw,flip_car_active_raw,body_product_id}`, `network_car_<i>_dodge_torque_raw_{x,y,z}` (and `_frame`) | its controls, boost and action counters |
| `network_player_<i>_{match_score,goals,assists,saves,shots,demolitions,epic_saves,clears,centers,aerial_hits,first_touches,crossbar_hits,bicycle_hits,juggle_hits,flip_resets,times_demolished,ping_raw}` (and `_frame`) | the player's stats and ping |
| `network_pad_records` | records: `pad`, `instigator_car`, `picked_up_raw`, `repeat` |

### `diagnostics` (opt-in)

| Column | Records |
| --- | --- |
| `prediction_errors` | per update that corrected the simulation: `car_actor` (null: the ball), `seconds_since_previous`, `position_error_{x,y,z}` (simulated minus updated), `velocity_error`, `rotation_error_degrees`, `angular_velocity_error`, the baselines `hold_position_error`, `linear_position_error`, `hold_velocity_error`, `hold_rotation_error_degrees`, `hold_angular_velocity_error`, and `is_on_ground` |
| `simulated_events` | RocketSim's events: `sim_tick`, `kind` (`ball_hit_world`, `car_hit_ball`, `car_hit_car`, `car_hit_world`, `car_pickup_boost`, `car_landed`), `player`, `other_player`, `pad`, `is_demo`, `point_{x,y,z}`, `normal_{x,y,z}`, `extra_velocity_{x,y,z}` |
| `simulated_touches` | the first tick of each car-ball contact: `player`, `replay_tick`, `point_{x,y,z}` |

## Header

JSON in the Parquet key-value metadata under `replicar`:

| Key | Meaning |
| --- | --- |
| `format_version` | 1; a reader refuses a later version |
| `replay_sha256` | the replay file's SHA-256 |
| `replicar_version`, `rocketsim_version` | the builds that wrote the file (resimulation needs the same RocketSim) |
| `groups`, `precision`, `rows`, `tick_step`, `all_frames` | what the file holds (`rows`: `ticks` or `frames`) |
| `platform` | where the states were simulated (`x86_64-linux`, ...): only the same platform resimulates them exactly |
| `players` | per player index: `index`, `key`, `name`, `team` (0 blue, 1 orange), `body_product_id`, `hitbox`, `final_stats` (each counter's last value in the replay, by `stat_events` kind and `score`: every kind of `counted_stats`, 0 when the player's counter was never sent; the other kinds are absent, unknown) |
| `counted_stats` | the `stat_events` kinds the replay's object table names: counted by its build. A kind not named is unknown (builds before September 2026 name `demolition` only once someone has one) |
| `pads` | per pad index: `position`, `is_big` |
| `segments` | per play segment: `first_frame`, `last_frame`, `end` (future-derived) |
| `final_scores` | the match result as the replay's last frame shows it; never in a row |
| `state_sha256` | the SHA-256 of every frame's bodies; a resimulation must reproduce it |
| `configuration`, `diagnostics` | the conversion's options and what it counted |

## Encoding

One row group; zstd level 9; float columns BYTE_STREAM_SPLIT; quantized bodies and the record lists' integer
fields DELTA_BINARY_PACKED; name columns dictionary-encoded. A file is written as `<name>.partial` and renamed
when complete.

## Corpus index

`replicar convert <folder> -o <folder>` also writes `index.parquet`: per replay `replay`, `file`, `sha256`,
`error` (null when converted), `rows`, `duration_seconds` (the replay time the rows span), `map`, `players`,
`blue_players`, `orange_players`, `blue_score`, `orange_score`, `segments`; the groups and precision are in its
key-value metadata under `replicar_index`.
