# Glossary

The words replicar uses, what they mean, and how each value is known. These are the **names proposed for v2**
(`docs/v2-plan.md`, 2026-10-06). The last section maps the v1 names, so RESULTS.md and v1 exports stay readable.
Every column of a replicar file, every CLI flag and every public type should be defined here; a test checks the
file schema against this page (v2 plan, story 9.2).

## Naming rules

- **Units.** Lengths are Unreal units (UU), speeds UU/s, angular velocities rad/s, everywhere and without a
  suffix. Time always carries its unit: `_seconds`, `_tick` (a point on a tick scale) or `_ticks` (a number of
  ticks).
- **Values the replay encodes in its own units** keep them and end in `_raw` (`boost_raw` 0-255, `ping_raw`).
- **Per-player columns** start with what they describe and the player index (`car_0_position_x`,
  `player_1_ping_raw`); this page writes them with the index left out (`car_position`). Per-pad columns likewise
  (`pad_3_cooldown`).
- **Provenance in the name only where it is not the group's.** A column in `state` is simulated or corrected
  state; one in `network` is what the replay sent. Inside a group, a value with another origin says so:
  `simulated_touches`, `car_status_inferred`.
- **Future-derived values** start with `future_` and live in the `future` group, so the warning is in the name.
- **Null means unknown or not applicable, never zero.** The Python arrays use NaN for floats and -1 for integers.
- **One word per concept.** If two things need two names, both are in this glossary, and each says how it
  differs from the other.

## Time

**Tick.** 1/120 s, RocketSim's physics step and the server's.

**Frame** (also *replay frame*). One network frame of the replay: the bundle of actor updates the recording
client stored at one moment. About 30 per second in online replays; other rates occur (a 10 fps LAN client). The `frame` column is the frame's index in the replay, counted from 0, so gaps show where pauses were left out.

**Tick row** (`frame_row` false). A row of a simulated tick between two frames. RocketSim steps every tick between
frames; a file has a row per tick by default (*rows*). A tick row belongs to the frame that ends its interval: its
state is simulated toward that frame's updates, which are applied at their update ticks inside the interval (so a
tick row's state can use an update its frame carries: offline reconstruction, a few ticks ahead of the frame's
time). Its game values (period, clock, scores) and ping are the previous frame's, what was known at the tick; its
update ages count from the update ticks; its seconds since update are null; its records are on frame rows.
Between frames the state is simulation, not observation; at an update tick a body can jump by the simulation's
error.

**Rows** (`rows`, `tick_step` in the header). Which rows a file has: `ticks` (default; every simulated tick in play,
or with `--tick-step N` those whose `sim_tick` is a multiple of N) or `frames` (one row per replay frame, about 30 per
second). A row's `controls` are those RocketSim applied in the step after the row's tick: the action taken at that
state.

**Replay time** (`replay_time`, seconds). The frame's timestamp as the replay stores it.

**Replay tick** (`replay_tick`). Replay time on the 120 Hz scale, counted from the first frame and rounded:
the timeline of the whole replay, pauses and goal replays included, so it jumps between play segments. It never
goes backwards.

**Sim tick** (`sim_tick`). RocketSim's own tick count for the arena. It advances only while the match is
simulated (continuous active play), so it falls behind the replay tick at every pause. Use the replay tick to
line frames up; the sim tick only relates a state to RocketSim's internal timers.

**Update** (also *body update*). A new rigid-body value (position, rotation, and usually velocities) for the ball
or one car in a frame. Most frames carry an update for most bodies, but not all, and an update can omit a
velocity. An update is an exact server state, but from a moment slightly before the frame: see *update tick*.

**Update tick** (`ball_update_tick`, `car_update_tick`, on the replay tick scale). The tick whose server state
the body's last update shows. It is 0-4 ticks before the frame's own tick at 30 frames per second. Inferred
offline from chains of updates (*update-tick inference*), not observed.

**Updated** (`ball_updated`, `car_updated`). The body got an update since the previous written row: on a frame row,
in this frame (observed); on a tick row, at its tick, the inferred update tick (or since the last written row with
`--tick-step`).

**Ticks since update** (`ball_ticks_since_update`, `car_ticks_since_update`). Replay tick minus update tick:
how old the body's last server state is at this frame. Usually 0-4 at an updated frame, growing until the next.

**Seconds since update** (`ball_seconds_since_update`, `car_seconds_since_update`). Replay time minus the replay
time of the frame that carried the body's last update. Observed (it uses no inferred tick).

**Clock phase** (`clock_phase`). Where the match clock is: `pregame`; `countdown` (before a kickoff); `kickoff`
(cars released, clock held until the first touch); `running`; `expired` (regulation at 0, waiting for the ball
to touch the ground); `decided` (the ball touched the ground after expiry); `goal_pause` (celebration and goal
replay). Within play segments only `kickoff`, `running`, `expired` and `decided` occur.

**In play.** The replay's game state is `Active`: from the frame where the kickoff countdown has ended and the
cars can move **[verify]** until play stops. The interval that ends at a goal frame (state `PostGoalScored`) is
simulated too: the ball crossed the line in it.

**Play segment** (`segment`, 0..n). A stretch of play from a kickoff to the frame that reports the goal, or to
the last frame in play when no goal follows (regulation ending, before overtime). The overtime kickoff starts a
new segment. Countdowns, goal celebrations, goal replays and everything before the first kickoff or after the
match are **outside** play segments. A replicar file holds only frames in play segments, unless it was written
with `--all-frames`; then the other frames are included with a null `segment`. Records (events, contacts,
pickups) outside play segments, such as a goal explosion's demolitions, are left out with their frames.
(v1 called these *episodes*.)

**Period** (`period`). `regulation` or `overtime`.

**Seconds remaining** (`seconds_remaining`). The regulation clock, fractional. The replay shows only the integer
(the ceiling of the true value); replicar reconstructs the fraction from when the integer changes. In overtime it
is 0, and `overtime_seconds` counts up.

## Bodies and identity

**Match.** The game the replay recorded. A *replay* is the file.

**Player.** A participant with a car: a human or a bot. A player keeps one *player index* for the whole replay.

**Player index** (`player`, 0..n). A player's position in every per-player column, so
`car_position[frame, player]` is that player's car. The header's `players` table gives each index its name,
team, platform ID, car body and hitbox. A player who leaves keeps the index (its columns become `absent`); a
player who joins gets a new one.

**Team.** 0 is blue, 1 is orange.

**Car.** The car a player drives at a frame. A player has several cars over a match: a demolished car is
replaced by a new one at the respawn. Cars are not indexed separately; the player index addresses the current one.

**Car status** (`car_status`). `absent` (the player has no car in this frame); `active`; `spawning` (the car
exists but has had no update yet, so it is shown at the spawn position the replay announced and kept out of
collisions); `demolished`.

**Car status inferred** (`car_status_inferred`). The status is replicar's inference, not the replay's word:
`spawning` (the pose is the announced spawn), or `demolished` concluded from a leftover body (a wreck the replay
still sends after a goal explosion, or a sleeping body no player is linked to).

**Hitbox.** One of RocketSim's car body configurations (Octane, Dominus, Plank, Breakout, Hybrid, Merc),
chosen from the player's car body when known, else Octane. Fixed per player index for the replay.

**Car body** (`body_product_id`). The game's product ID of the car model, from the player's loadout.

## Where a value comes from

**Network value.** A value the replay's network data carries, decoded and kept with the frame of its last change;
the `network` group (below). Ground truth as far as the replay goes, but sparse, in the replay's own units, and
carrying only what the server chose to send.

**Simulated.** Produced by RocketSim between updates.

**Corrected state.** The state after an update was applied at its update tick and simulated forward to the frame
time. What the `state` group holds at an updated frame.

**Inferred.** Estimated by replicar from the replay, not stated by it: update ticks, when an input happened, air
controls, spawn poses, ball contacts, the fractional clock.

**Future-derived.** Computed from frames after the one it describes (the `future` group). Never use as model
input. Some inferences also look ahead (the fits use later updates); that is what makes replicar an offline
reconstruction, and it is why the future group is kept apart: it says what happens next by construction.

**Future group** (`future`; v1 *labels*). Training targets read from the frames after the one they describe,
about the frame's own play segment only (never a later one):
- `future_segment_end`: how the segment ends. `blue_goal` or `orange_goal`; `time_expired` (regulation ran out:
  the clock phase at the segment's last frame is `expired` or `decided`); `replay_ended` (the replay stops during
  play, so the end is unknown); `other` (play stopped some other way). Every segment has a value: a segment
  without a goal says so instead of being null. A forfeit looks like `replay_ended`: on train and validation the
  13 segments that end without a goal or an expired clock all end at the replay's last frame, and none is `other`.
- `future_seconds_until_segment_end`: replay time from this frame to the segment's last frame (0 there). For a
  goal segment this is the time until the goal.

The match result (final score, winner) is in the header, never per frame.

**Inferred** versus **resimulation group.** Many values are inferred (air controls in `state`, update ticks in
`updates`). The `resimulation` group is a specific set: the inferred values the simulator needs to reproduce
`state` without fitting. It is named for that purpose to avoid the ambiguity.

## Reconstruction

**Reconstruction.** The whole process: decode the replay, infer what it does not say, and simulate the match
in RocketSim between updates, so that every frame has a full state.

**Controls** (`car_controls`: throttle, steer, pitch, yaw, roll, jump, boost, handbrake). The input RocketSim applied
in the step after the row's tick (the action taken at the row's state), in RocketSim's ranges. Throttle, steer, handbrake and boost are network values (their
timing inferred); jump and dodge come from the replay's action counters; pitch, yaw and roll are inferred, because
the replay does not carry them.

**Controls source** (`car_air_controls_source`, `car_ground_controls_source`). What set a row's controls, per car.
For pitch, yaw and roll: `none` (not inferred, 0: the car was on the ground at its last update), `steer` (in the air
without a fit: the steer as yaw, or as roll with the handbrake), `persisted` (solved from the car's last two updates
and decayed since), `lookahead` (solved for the interval to the next update), `schedule` (solved tick by tick to the
next update), `press` (a jump or dodge from this frame's action counters, with its direction), `dodge` (a fitted
dodge press and its flip cancel), `flip_cancel` (a flipping car's fitted cancel as the pitch; yaw and roll as
`steer`, `persisted` or `lookahead`). For throttle, steer, handbrake and boost: `network` (the replay's values from
the tick their effect is inferred at), `schedule` (a fitted ground schedule: control timing or a jump), `dodge` (a
fitted dodge press, with the controls of the update it was planned at). `lookahead` and both `schedule`s are
future-derived: they use the car's next update. The jump input is the action counters' (observed), or a fitted press
(`fitted_inputs` in the `resimulation` group).

**Previous controls** (`car_previous_controls`). The controls of the tick before; part of RocketSim's car state,
needed to restore it.

**Air controls.** Pitch, yaw and roll while airborne. Inferred as the constant (or piecewise constant) input per
update interval that reproduces the observed rotation: not the player's per-tick stick.

**Press.** The tick a jump or dodge input started, with a dodge's direction and *flip cancel* (how much of the
flip's pitch the player cancelled, 0-1). Inferred by fitting.

**Fit.** An inference that tries candidate values in a scratch simulation and keeps the one that best reproduces a
later update: control timings, presses, air controls, contact alignment.

**Contact alignment.** Moving a car's update tick by a tick or two so that a simulated hit reproduces the ball's
next update. Part of update-tick inference.

**Inference** (code: the `Inference` trait). The part of replicar that makes these choices. *Fitted inference*
computes them; *recorded inference* reads them back from a file's `resimulation` group. Each value it hands the
simulator is a **choice** (code: `Choice`): an update tick, a control timing, a press, an air-control segment.

**Simulator** (code). The part that applies updates at their ticks, steps RocketSim and applies the controls
the inference gives it. It contains no fitting.

**Resimulate** (`replicar resimulate`). Rebuild the `state` group from the replay and a file's `resimulation` group,
without fitting: the same states, about ten times faster than a conversion, needing the same replicar and
RocketSim versions.

## Events and contacts

**Event** (`events`, a list per frame). Something the replay reports: `goal` (with the `scoring_team`, and the
`scorer` and `assister` whose goal and assist counters went up for it),
`demolition` (attacker and victim player; a repeated report of one demolition is marked `repeat`), `flip_reset`
(the car's dodge was refreshed in the air, from its refresh counter). Observed.

**Stat event** (`stat_events`, a list per frame). A player's match counter going up, in the frame the replay
updates it: the game's own judgement of a `goal`, `assist`, `save`, `shot` or `demolition`, and, in replays from the
builds of September 2026 on, `epic_save`, `clear`, `center`, `aerial_hit`, `first_touch`, `crossbar_hit`,
`bicycle_hit`, `juggle_hit`, `flip_reset` and `demolished`. Observed, with the replay's delay. A counter that goes up
by two in one update (two aerial hits) is two events, with consecutive `total`s. Like every record, it
is in the file even when its frame is not: one from a left-out frame (an assist credited during the goal pause) is on
the last written row before it, and its `updated_frame` is the frame of the update. A player's events add up to the
header's `final_stats`, each counter's last value over the replay (a reconnecting player's last frames can lack it).
The game sends a counter only when it goes up, so a kind is known only where the replay's object table names it: the
header's `counted_stats`, for which a player without updates has 0. A kind not named is unknown for every player, not
0; builds before September 2026 count only goals, assists, saves, shots and demolitions, and name the demolition
counter only once someone has one.

**Ball contact** (`ball_contacts`). The ball's motion between two updates that no free flight explains: something
hit it. Found from ball updates alone, with the estimated tick and the nearest car's player (none when no car was
close: a post, a wreck). Inferred from observed motion. **The preferred touch evidence.**

**Simulated touch** (`simulated_touches`). The first tick of a car-ball contact in RocketSim's simulation.
Overlaps ball contacts; never add the two.

**Simulated event** (`simulated_events`, diagnostics). RocketSim's own per-tick event reports.

**Boost pickup** (`boost_pickups`). A boost pad the replay reports taken, matched to a pad and checked against
the named car's path (`verified`; another car's path that reaches it is `suggested_player`).

## Files

**replicar file.** One ordinary Parquet file (`.parquet`) per replay, one row per tick (or frame) in play, holding the
column groups asked for. Any Parquet reader opens it; replicar's reader adds conveniences. Its
**header** (JSON in the Parquet key-value metadata) has the format version, the replay's SHA-256, the replicar and
RocketSim versions, the configuration, the groups and precision, the players, the pad layout and diagnostics.

**Column group.** A named set of columns written together:

| Group | Default | Holds |
| --- | --- | --- |
| (always) | yes | `frame`, `segment`, `replay_time`, `replay_tick`, `sim_tick` |
| `state` | yes | ball and car physics, car internals, controls, boost, pads, car status |
| `game` | yes | scoreboard (`clock_phase`, `period`, scores, clock), events, ball contacts, boost pickups |
| `updates` | yes | updated, update tick, ticks and seconds since update, ping |
| `future` | yes | the `future_` columns: what happens next |
| `network` | no | the network values with the frame of their last change |
| `resimulation` | no | what the fitted inference chose; enough to resimulate |
| `diagnostics` | no | prediction errors before each correction, simulated touches and events |

**Network group** (`network`). The replay's network feed, decoded: for each frame, every value replicar reads
(ball and car bodies, controls, boost, action counters, demolition and pad records, scores, clock, ping), in the
replay's units, forward-filled, each with the frame of its last change. It omits what replicar does not read
(cosmetics, camera settings, most stats), so it is not a lossless copy of the replay. Its columns start with
`network_` (they would otherwise share names with the state's) and each value has a `<name>_frame` column with the
frame of its last change: `network_ball_position_x`, `network_ball_position_frame`, `network_car_0_throttle`,
`network_player_0_goals`, `network_seconds_remaining` (the integer clock), `network_pad_records`. Per player, the
values of the player's current car in that frame.

**Diagnostics group** (`diagnostics`). `prediction_errors` (per update that corrected the simulation: the simulated
minus the updated position, the velocity, rotation and angular velocity errors, and the same for holding or
linearly extrapolating the previous update, as baselines), `simulated_events` (RocketSim's own events: ball and car
contacts, pickups, landings) and `simulated_touches`.

**Resimulation group** (`resimulation`). Each update's tick, the times observed controls took effect, presses, air
controls and the other choices of the fitted inference, per frame. With the replay, this is everything RocketSim
needs to reproduce the `state` group exactly.

**Precision** (`--precision`). How the `state` group stores numbers: `float32` (as RocketSim has them; rotations as
unit quaternions, within 1e-6 of RocketSim's matrix) or `quantized` (integers: 0.01 UU, 0.01 UU/s, 1e-4 rad/s,
quaternion components to 1/32767; readers return float32). The header records it.

**Index file** (`index.parquet`). Written by a folder conversion: one row per replay with its replay and file
paths, SHA-256, error (null when converted), rows, `duration_seconds` (the replay time the rows span), map, players
per team, final score and segments; the groups and precision are in its key-value metadata (`replicar_index`).

**`--all-frames`.** Also write the frames outside play segments (countdowns, goal pauses and replays), with a null
`segment`. Off by default.

## Evaluation (for contributors)

**Split.** `train` (inspect and design), `validation` (decide), `test` (sealed: used once, for the frozen
assessment; the evaluation tools refuse it without `--final-assessment`).

**One-step residual.** The simulated state just before an update minus the update itself.

**Masked prediction.** Withhold some frames' updates and score how well the reconstruction predicts them.
**Withheld frame**: a frame whose updates are hidden; every inference must refuse to look at it.

**Held-out.** A fit that does not use the update it is scored on.

## v1 to v2

| v1 | v2 |
| --- | --- |
| `timeline_tick` | `replay_tick` |
| `arena_tick` | `sim_tick` |
| packet | update |
| packet lag (`packet_lags` table, `AppliedPacketLag`) | update tick (`car_update_tick`, `ball_update_tick`) |
| `ball_fresh`, `car_fresh` | `ball_updated`, `car_updated` |
| `*_packet_age_ticks` | `*_ticks_since_update` |
| `*_update_age_seconds` | `*_seconds_since_update` |
| car slot (`slot`) | player index (`player`) |
| `car_present`, `car_demoed`, `dead_shell_held`, `spawn_pose_held` | `car_status`, `car_status_inferred` |
| dead pawn shell | wreck (a leftover body, status `demolished`, inferred) |
| `scoreboard_clock_state` | `clock_phase` |
| observations (`frame_json.observations`) | network values, `network` group |
| `touches` | `simulated_touches` |
| `fitted_inputs` | presses and air controls in the `resimulation` group |
| episode (`label_episode`) | play segment (`segment`, a core column) |
| labels (`label_*`) | the `future` group (`future_*`) |
| `label_next_scoring_team`, `label_seconds_until_next_goal` (the next goal anywhere, null if none) | `future_segment_end` (this segment's own end, always set) |
| `label_episode_seconds_remaining` | `future_seconds_until_segment_end` |
| every frame of the replay | frames in play segments (`--all-frames` for all) |
| `pad_pickups` | pad records in the `network` group |
| `goal_scored_on`, `demolish`, `dodge_refreshed` | `goal`, `demolition`, `flip_reset` |
| `ConvertOptions` | `Config` |
| `position_residuals` | prediction errors, `diagnostics` group |
| (planner, executor in the v2 draft of 2026-10-06) | inference, simulator |
| (`Decision`, then `Inferred`, in drafts) | `Choice`: one value the inference chose |
| (plan, plan file; `inferred` in the fourth draft) | `resimulation` group |
| (`exact`, `compact` precision) | `float32`, `quantized` |
