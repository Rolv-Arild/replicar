# Reconstruction measurements

Last updated: 2026-09-29. These are development measurements, not a final accuracy claim. Current reviewed baseline machine-readable reports are `target/train-reviewed.json` and `target/validation-reviewed.json`; the latest optional low-air gate reports are `target/train-low-air-cap-gated.json` and `target/validation-low-air-cap-gated.json`. Packet timing reports are `target/train-packet-timing.json` and `target/validation-packet-timing.json`. Older experiment reports are retained under `target/*-conversion-metrics*.json`. Each evaluator report includes replay SHA-256 values, settings, errors, and per-game-size aggregates. No `test` replay has been opened or converted.

## Walls and ramps: the "air jump" was a wall jump, and the fits now accept any surface (2026-09-30)

**Windows first** (`trace_jump_windows replays/train --air`: 4,697 jump activations whose previous packet is above 50 UU). The backlog item was an "air jump" (3,672 train packets, velocity p50 296.9 UU/s, share 0.054). Every window shows the car on the ground in the exported state at heights of 330-1,100 UU and a first-packet residual of about 300 UU/s along a horizontal or oblique direction (the median event's worst vertical-velocity error is 299.6). These are jumps off walls, ramps and the ceiling: the impulse leaves along the surface normal, and the press has the same per-event timing as a floor jump. The fits refused them because they required a car flat on the floor (z < 25 UU, up axis z > 0.97).

**Change.** The jump fit and the joint jump-and-dodge fit now need only `is_on_ground` (any surface), and so does the ground control timing fit, which also stops requiring the second and third packets to be flat. The converter's arena carries the wheel contacts of the surface for the scratch runs; the counters already rule out a jump or dodge in the span, and the ball rule of the ground fit stays.

**Effect** (`error_budget`, chain-lag packets; train / validation; against the state before this change):

| Measure | Before | After |
| --- | --- | --- |
| First packet after a jump start, previous z >= 50: velocity p50 UU/s | 296.6 / 297.3 | 19.9 / 21.0 |
| Same, velocity squared-error share | 0.056 / 0.061 | 0.013 / 0.016 |
| Same, position p50 UU | 5.0 / 5.0 | 1.5 / 1.7 |
| Wall or ramp, no boost: velocity p50/p90 UU/s | 8.4/74.7 / 8.3/73.9 | 3.7/48.7 / 3.8/50.0 |
| Wall or ramp, no boost: angular velocity p90 rad/s | 0.83 / 0.84 | 0.56 / 0.59 |
| Wall or ramp, no boost: position p90 UU, rotation p90 deg | 5.2, 2.62 / 5.0, 2.59 | 3.6, 1.87 / 3.6, 1.94 |
| Wall or ramp, boosting: velocity p90 UU/s, angular p90 rad/s | 71.0, 0.94 / 66.3, 0.89 | 54.3, 0.66 / 53.7, 0.65 |
| All cars: velocity p90 UU/s, angular p90 rad/s | 50.6, 0.58 / 51.4, 0.59 | 48.1, 0.55 / 48.7, 0.56 |
| `evaluate_corpus` car velocity p50/p90/p99 UU/s | 2.048/59.266/499.298 / 2.036/60.031/494.427 | 1.875/55.571/497.757 / 1.861/56.930/489.161 |
| `evaluate_corpus` car rotation p90 deg | 2.836 / 2.826 | 2.766 / 2.764 |
| `evaluate_corpus` car angular velocity p90 rad/s | 0.648 / 0.656 | 0.621 / 0.630 |
| `evaluate_corpus` car position p90 UU | 4.545 / 4.566 | 4.309 / 4.403 |

Per replay (60 each), p90 improved for velocity in 60 and 60, rotation in 58 and 58, angular velocity in 60 and 59; worse by more than 2%: none. Floor driving, air, near ball and near another car are unchanged to within 0.1 UU (their shares rise only because the total falls); non-aligned masked prediction is unchanged bit for bit and the aligned masked numbers move by less than 1% (target change). The whole path from the start of the overnight work, one-step car velocity p90 (train): 85.9 UU/s at the original converter, 66.1 after the ground-control fits, 59.3 after the jump and dodge fits, 55.6 now.

**What is left.** The first packet after a jump start keeps a velocity p90 of about 285-295 UU/s (about 15% of the events: refused when a dodge follows within the span without an exact lag on the second-next packet, or no second-next packet within 45 ticks). The wall/ramp p90 is still 49-50 UU/s against 36-38 for the floor, and the ground fit keeps its ball rule (near-ball driving is refused; the scratch ball is parked).

## A BakkesMod state dump with the true inputs (2026-09-30)

**The data.** `replays/2024.1.13-17.14.8.replay` (102 KB) and `replays/2024.1.13-17.14.8.json` (1.3 MB), added by the user; the file is in no split (it is not under `train`, `validation` or `test`) and is used here as a diagnostic with ground truth, not as an evaluation set. The JSON has 1,214 records (`frames`), one player (the local player, "Vync62") and the ball, with `localPlayer_gamepadSettings` (air control and steering sensitivity 1.8, deadzone 0.06, dodge input threshold 0.75). Each record has the car's location, velocity, Unreal rotator (pitch, yaw, roll in 1/65536 turn), angular velocity (rad/s), the state flags (`b_jumped`, `b_doubledJumped`, `b_isdodging`, `b_canJump`, `b_superSonic`, `boostAmount`, `time_onGround`, `time_offGround`) and the true controller inputs (`Throttle`, `Steer`, `Pitch`, `Yaw`, `Roll`, `Jump`, `Jumped`, `ActivateBoost`, `HoldingBoost`, `Handbrake`, `DodgeForward`, `DodgeStrafe`), plus the ball's location, velocity, rotation and angular velocity.

**Alignment** (`align_dump replay dump`). The records are exactly 4 physical ticks apart (implied interval between consecutive records p10/p50/p90 = 3.99/4.00/4.00 ticks, from position differences along the velocity), and there is one record per replay frame: record j is the physical state at timeline tick 4j (T0 = 0 for all 389 free-flight ball packets that RocketSim's ball reproduces from a record, at tick offsets 0 to 3). The Unreal rotator convention was validated against replay quaternions: with the standard matrix (forward, right, up columns; positive pitch up and positive roll as in Unreal) the rotation error at well-matched frames is 0.03 deg p50 (the other sign choices give 16-39 deg). The dump's `Pitch`, `Yaw`, `Roll` map straight to RocketSim's `pitch`, `yaw`, `roll` controls; `DodgeForward`/`DodgeStrafe` are the unit dodge direction (`dir.x` forward and `dir.y` right, so `Pitch = -DodgeForward`).

**This replay has no replication lag.** The raw observed packets equal the dump at the same frame: car position p50/p90/p99 0.83/4.88/6.5 UU (velocity 0.58/4.16/22 UU/s, rotation 0.08/0.39/1.2 deg, angular velocity 0.006/0.034/0.17 rad/s), ball position 0.75/1.88/6.4 UU, and every frame has a fresh car packet. It looks like offline (single-player, no network) data: the packet time is the frame time. The converter's chain lags for it are 1-2 ticks (from a nominal-time drift plus an arbitrary absolute offset, since a chain of lag-free packets fixes the lags only up to a constant), so the exported car state is advanced by 1-2 ticks too far: position error vs the dump p50/p90/p99 23.3/38.7/39.6 UU (lag 1: 13.8 UU, lag 2: 28.8 UU, about the car's travel per tick), velocity 12.0/66/298 UU/s. A lag-free replay should be detected (all chain lags within a narrow band that does not span the 0-4 tick range) and get lag 0; not done yet.

**The observed controls are the true controls at the frame time.** Replay throttle at frame f equals the dump's `Throttle` at frame f exactly in 86.8% of frames (mean absolute difference 0.004; at f-1 and f+1 it is 0.038 and 0.045), steer 83.0% (0.020 versus 0.122 and 0.141), handbrake 99.9% (98.1-98.2% at the neighbouring frames). So here there is no frame delay between an input and its observed value, unlike the online assumption behind the midpoint rule ('a change is first seen a frame after it happened'). That assumption came from online replays, where replicated inputs pass through the network; this file cannot test it.

**One-step ground truth** (`dump_onestep`). RocketSim started from the dump's true car and ball state at j with the true inputs, stepped 4 ticks, compared with record j+1 (1,213 pairs; hold the inputs of j / use those of j+1 / their mean):

| Group | n | Position p50 / p90 UU | Velocity p50 / p90 UU/s | Rotation p50 / p90 deg | Angular velocity p50 / p90 rad/s |
| --- | --- | --- | --- | --- | --- |
| Ground, no boost (hold j) | 330 | 0.01 / 0.42 | 0.0 / 23.3 | 0.00 / 0.37 | 0.001 / 0.344 |
| Ground, no boost (mean) | 330 | 0.01 / 0.40 | 0.0 / 17.5 | 0.00 / 0.36 | 0.001 / 0.288 |
| Ground, boosting (mean) | 147 | 0.12 / 1.71 | 7.3 / 73.4 | 0.07 / 0.47 | 0.071 / 0.364 |
| Air, no boost (hold j) | 284 | 0.01 / 0.26 | 0.1 / 2.8 | 0.15 / 0.64 | 0.131 / 0.535 |
| Air, boosting (mean) | 152 | 0.01 / 0.30 | 0.0 / 15.8 | 0.13 / 0.34 | 0.140 / 0.288 |
| Jump window (hold j) | 124 | 1.01 / 31.3 | 48.6 / 939 | 0.26 / 8.16 | 0.248 / 6.21 |
| Flipping (approximate flip state) | 170 | 0.46 / 2.16 | 22.0 / 93.4 | 3.46 / 4.67 | 0.875 / 2.02 |

With the true state and inputs, RocketSim's ground and air physics reproduce a 4-tick step almost exactly (ground no boost position p90 0.4 UU, rotation p90 0.37 deg; the corpus one-step ground residuals with inferred timing are position p90 2.0 UU, rotation 1.3 deg, though over longer packet intervals), so the remaining ground error of the converter is inputs and their timing, not the physics. The mean of the two inputs is at least as good as holding the first for the continuous axes (boosting: velocity p90 73 versus 73, angular 0.36 versus 0.41). The jump window and flips are dominated by tick-level timing inside the 4-tick interval (a press between two records changes the impulse tick), and the flip row is unreliable: the tool sets no `flip_relative_torque` or `flip_time` (the dump has neither), so it is a lower bound on what the flip state must supply.

**Use.** The dump gives true inputs and a 4-tick truth grid, which the corpus lacks: it can score the fits that bridge packets (thin the replay's packets, reconstruct, compare the frames without a packet with the dump), and check the jump, dodge and cancel inputs against the truth. One offline replay of one player, so it validates mechanisms, not absolute numbers for online play.

## Frames between packets: a boundary-value solve of the air controls (2026-10-01)

**The idea.** Between two fresh packets the converter simulated forward from the first one and snapped to the second, so the exported interior frames drift away and jump back, and the next packet only corrected the start of the following interval. The interior error was never scored: `error_budget` and `evaluate_corpus` compare the simulated state with a packet at the packet's own tick, the end of the gap. `rlbot_reconstruction --thin K` now thins a replay's car packets (host replays, which are lag-free and have every frame fresh, thinned to every third frame, 12 ticks, give interior frames) and scores every frame without a packet against the server's true state.

**The method** (`air_bvp`, default on, `plan_air_bvp`). For an airborne car at a fresh packet, the controls of the interval to its next fresh packet are solved as a boundary-value problem: per-segment pitch, yaw and roll (segments of about four ticks, three unknowns each) carry the car to the next packet's rotation **and** angular velocity (six end conditions), as a Levenberg-Marquardt fit of a forward model, regularised toward the constant span solution the converter already had. Free flight uses the analytic air model (`air_state_forward`, equal to RocketSim to 0.04 deg and 0.008 rad/s over 12 ticks, unit test); a car that is flipping, or whose dodge press falls in the span, uses RocketSim itself in a scratch arena with the ball parked (the dodge press tick or the flip time is also shifted by up to 6 ticks, the shift that reaches the end state best is kept and moves the dodge plan or the flip time). The solution becomes a per-tick control schedule for the interval (`AirSchedule`), so the exported interior frames are what RocketSim produces under those controls: physically consistent, not a correction added afterwards. It is refused (and the old controls stay) for a car low to the ground (z < 30 UU) at either packet, a withheld or inactive frame, a dodge in the span with no planned press, or a solution that does not reach the end (more than 3 deg or 0.5 rad/s off). It uses the next packet, so the residual of the interval at that packet is no longer a prediction.

**Result** (interior frames, no fresh packet, against the server truth; host replays of both games thinned to every third frame; before / with the solve):

| Frames | Rotation p50 / p90 (deg) | Angular velocity p50 / p90 (rad/s) |
| --- | --- | --- |
| Air, no fresh packet (9,894 / 11,053 frames) | 0.69 / 2.52 and 0.72 / 2.56 -> 0.17 / 0.96 and 0.17 / 0.95 | 0.32 / 1.19 and 0.33 / 1.22 -> 0.08 / 0.45 and 0.08 / 0.46 |
| Flip, no fresh packet (4,239 / 4,720) | 1.15 / 5.35 and 1.22 / 5.52 -> 0.41 / 4.71 and 0.43 / 4.84 | 0.25 / 1.51 and 0.26 / 1.63 -> 0.10 / 1.40 and 0.11 / 1.48 |
| All frames without a fresh packet (38,038 / 40,087) | 0.06 / 1.78 and 0.06 / 1.97 -> 0.03 / 0.89 and 0.04 / 0.94 | 0.018 / 0.71 and 0.021 / 0.75 -> 0.015 / 0.36 and 0.016 / 0.39 |
| All frames | p90 1.26 and 1.39 -> 0.56 and 0.58 | p90 0.52 and 0.56 -> 0.24 and 0.25 |

The intervals the solve covers (8,429 of 9,894 air frames, 2,548 of 4,239 flip frames in game 1) have rotation p90 0.65 deg (air) and 1.19 deg (flip), angular velocity p90 0.29 and 0.28 rad/s; position and velocity are unaffected (they did not drift: position p90 0.2 UU, velocity p90 7 UU/s on all frames). The uncovered ones are what remains: flips at low altitude (contact with the ground in the span), about 1,050 'no solution' intervals in the host replay of game 1, and low flights. On the client replays thinned to every second frame (20-tick gaps, with real lags) the gain is smaller because timing dominates (position p50 12 UU): air rotation p50 4.2-4.4 -> 1.5-1.6 deg, p90 12.2-12.9 -> 8.5-9.3; all frames without a packet rotation p90 11.0-11.2 -> 9.8-10.0 deg.

**A bug this exposed.** The interior flips had the wrong direction in 7.5% of flipping frames (angle between the simulated and the true flip torque p90 18 deg, p99 67 deg): the dodge press used the controls of its car as a base, including the roll of the air controls, and RocketSim's dodge direction is (-pitch, yaw + roll), so a leftover roll turned the flip (and a double jump into a flip, for which the flip lasted 20 frames). The press now zeroes roll in every place (planned, fitted, default, double jump); flip direction error p99 0.3 deg. This also improved the plain prediction on the corpus (one-step car residuals, train / validation, before the solve, against the previous commit): rotation p90 -4.7% / -4.4%, p99 -5.8% / -5.8%, angular velocity p90 -3.5% / -3.5%, p99 -6.4% / -6.3%, velocity p90 -2.2% / -2.2%; per replay better in 58-60 of 60 for rotation p90, angular velocity p90 and velocity p90 on both splits.

**Fitting the ground and jump timings on the next packet** (`fit_on_next_packet`, default on). The ground control and jump timing fits chose their shift against the second-next packet so that the next one stayed a held-out check. The interior frames are between the first and the next packet, so the fit now targets the next packet itself (`--fit-on-next-packet` restores it in the tools; the tools keep the old behaviour by default because they score predictions at packets). Interior error against the server truth, all four replays (host thinned every third frame, client every second), base = both solves off, now = both on:

| Replay | Frames without a packet: rotation p90 (deg) | Angular velocity p90 (rad/s) | Velocity p50 / p90 (UU/s), all frames |
| --- | --- | --- | --- |
| Game 1 host | 1.78 -> 0.85 | 0.71 -> 0.34 | 0.0 / 7 -> 0.0 / 5 |
| Game 2 host | 1.97 -> 0.91 | 0.75 -> 0.37 | 0.0 / 7 -> 0.0 / 5 |
| Game 1 client | 11.0 -> 9.7 | 1.83 -> 1.59 | 19.9 / 137 -> 17.3 / 129 |
| Game 2 client | 11.2 -> 9.5 | 1.82 -> 1.56 | 17.6 / 132 -> 16.1 / 125 |

On the client replays the ground interior improves as well (game 1, plain ground: velocity p50 42 -> 34 UU/s, rotation p50 1.74 -> 1.23 deg, angular velocity p50 0.17 -> 0.10 rad/s). The remaining client error is timing, not dynamics: the exported state at a frame time is offset by the lag level of its packets (position p50 12 UU).

**How to read the two kinds of numbers.** `error_budget` and `evaluate_corpus` turn the solve off by default (`--air-bvp` turns it on): their residuals are predictions at packets and the solve fits exactly those packets. The interior error against server truth (`rlbot_reconstruction`, `scripts/run_interior_eval.sh`) is where the solve is scored. Cost: the offline conversion of a replay takes about 5 s with the solve against about 0.6 s without it (9 replays of the corpus: 20 s to 66 s in `evaluate_corpus`, whose masked runs do not use the solve); the flip intervals with their shift scans dominate. Not yet done: a solve for ground driving and jumps (interior ground rotation p90 0.23 deg and velocity p90 11 UU/s, jump window velocity p90 97 UU/s), for the translation (boost timing) and for ball contacts, and the uncovered flips above.

## The full car state, not only the physics (2026-09-30)

**The gap.** `error_budget` and `evaluate_corpus` score position, velocity, rotation and angular velocity only. The state that decides what a car can do next (boost, `has_jumped`, `has_double_jumped`, `has_flipped`, `is_flipping`, whether the flip window is open) was never scored. `rlbot_reconstruction` now scores the exported car state of every active frame against the server's true flags and boost (RLBot `has_jumped`, `has_double_jumped`, `has_dodged`, `air_state`, `dodge_timeout`, `boost`): mismatch counts per flag, boost error, and the flip window time left against `dodge_timeout`. On the four replays of the two remote-client games (host and client, game 1; game 2 alike) the first measurement found real errors that the physics metrics could not see.

| Flag or quantity | Host replay, before | after | Client replay, before | after |
| --- | --- | --- | --- | --- |
| `has_double_jumped` wrong | 9.13% of frames (every true frame) | 0.09% | 9.24% | 0.11% |
| `has_flipped` wrong (truth true in 11,405 / 4,656 frames) | 4.09% | 0.38% | 6.56% | 1.37% |
| `is_flipping` wrong (truth true in 6,308 / 2,595 frames) | 2.38% | 0.27% | 4.33% | 2.79% |
| `has_jumped` wrong (truth true in 21,974 / 8,965 frames) | 1.86% | 1.87% | 11.75% | 4.14% |
| Flip available wrong (jumped, no flip, within 1.25 s, airborne) | 8.94% | 2.09% | 9.87% | 2.89% |
| Boost error p50 / p90 / p99 (of 100) | 0.33 / 0.86 / 12.4 | 0.26 / 0.71 / 1.34 | 0.51 / 11.3 / 24.7 | 0.41 / 2.7 / 24.4 |

**Four causes, each fixed.**
1. **Double jumps were never simulated.** The double-jump counter was only used to refuse other fits, so `has_double_jumped` was false in every double-jump frame and the impulse was missing (the corpus partition 'double-jump counter odd': velocity p50 282 UU/s). Now a double-jump counter turning odd applies RocketSim's directionless jump press (`infer_double_jump`), gated like the dodge default (a fresh velocity packet at that frame already holds the impulse: flags only). The same frame-level error at truth `DoubleJumping` frames on the host replay: velocity p90 292 to 16 UU/s; corpus partition velocity p50 282 to 49 UU/s, position p90 17 to 12 UU.
2. **A dodge in the same direction as the last one was ignored.** The replay sends the dodge torque only when it changes, so a repeated direction has an old stamp, and the code required a torque stamped at the activation frame: 97 of 448 activations (22%) of the host replay of game 1 (bots repeat directions) never produced a flip at all (whole flips, 20 frames each, without a flip in the simulation). `activation_torque` takes the torque in effect a frame after the counter (handles a stale value and a torque that arrives a frame late). Host replay: flips 11,405 true frames, wrong in 2,325 to 214; fitted dodge presses 268 to 424; corpus activations 17,577 to 17,785 (repeats are rarer in human play: +1.2%).
3. **Simulated pad pickups gave boost the car did not get.** RocketSim picks boost up when a car drives over a pad; with a simulated position a few UU off the real one, the simulated car took pads the real car missed (12 or 100 boost: persistent errors of +12 and +36 to +100 until the next replay boost update). Offline reconstruction now keeps the pads on cooldown in the simulation (`block_sim_pad_pickups`, `--sim-pad-pickups` restores) and takes boost from the replay's updates; the causal masked prediction keeps the simulated pickups (it has no later update; the masked boost metric is unchanged). Host boost p99 12.4 to 1.3, client p90 11.3 to 2.7 (the remaining p99 is the one-frame delay of a pickup: the new amount is first seen about a frame after the pickup; applying an increase seen in the next frame one frame early, `boost_pickup_lookahead`, did not help, off).
4. **Flags the simulation did not set itself.** An airborne car whose jump the simulation never applied had `has_jumped` false (a car that never jumped can do different things). `flags_from_counters` sets `has_jumped`, `has_double_jumped` and `has_flipped` of an airborne car when the replay's jump, double-jump or dodge counter differs from its value when the car was last on the ground, with the flip time and the air time since the jump taken from the counter's stamp (the pitch lock after a flip ends on time), and not for an action the simulation applies this frame or has planned. Client `has_jumped` 11.75% to 4.14%.

**Physics did not suffer and mostly gained.** Against the server truth the physics rows are unchanged by the flags (client position p90 20.7 UU, rotation p90 3.66 against 3.64 deg without) and improved by the dodge fixes (host velocity p90 7 to 5 UU/s, rotation p90 0.47 to 0.41 deg). One-step car residuals on the corpus (train / validation): the double jump and torque changes lower velocity p90 by 0.4% / 0.7% and rotation p99 by 0.7% / 1.4%; the counter flags raise angular velocity p90 by 0.3-0.4% and rotation p99 by 0.7% (per replay worse in most replays by under 1%), kept because the state they fix (what a car can do next) matters more than that; `flags_from_counters` can be turned off in `ConvertOptions`.

**What is still not measured.** Flags are scored only against the RLBot recordings (bots and two humans, LAN). For the corpus they follow the replay's counters by construction, so a corpus check would only repeat them. Boost on the corpus is scored by the masked boost metric against replay updates. The flip-availability definition (`dodge_timeout > 0` in the truth against RocketSim's 1.25 s window) agrees to 0.14 s at p90 (`flip-window time left`).

## Two games with a remote client: the online replay against server truth (2026-09-30)

**The data.** `replays/2026-09-30T16-56-39Z_lan_remote_4bots_game1` and `..._17-05-59Z_lan_remote_4bots_game2`: a match hosted on the user's PC (RLBot; 2 humans, the host and a remote player, and 4 bots; game 1 5-3, game 2 4-5, about 8 min each), with the RLBot recording of the server's true state and inputs at every tick (62,442 and 58,174 packets, 6 cars and the ball), and the saved replays of the host and of the remote client (MatchType Lan; the network path and its latency were not recorded). Nothing else needed: the client's replay is an ordinary online replay whose every car packet can be placed on a server tick. Tools: `dump_replay_packets` (replay to JSON lines, with the converter's inferred lags), `scripts/rlbot_replay_match.py`, `scripts/rlbot_lag_check.py`, `scripts/rlbot_control_delay.py`, `rlbot_reconstruction`.

**Both replays contain exact server states.** 98.4-98.6% of the fresh car packets of each replay (host and client, both games) equal a recorded server position exactly (0.01 UU grid), so each maps to its server tick (`frame_num`). Host replay: a fresh packet for every car in every frame (4-tick frames), and the true lag is 0 (spread 0.0 ticks after detrending); the replay clock drifts against the physics tick by -11.5 and +48 ticks over the match. Client replay: only 4,888 and 5,113 frames in 8 min (10 frames per second, frame gap p10 / p50 / p90 6 / 10 / 11 ticks) with a fresh car packet in nearly every frame (gap between a car's packets 71 / 82 / 97 ms).

**The true lag of a client packet is uniform over its frame window, not 0-4 ticks.** Detrended against the server tick: spread p1 / p10 / p90 / p99 -4.7 / -3.4 / +3.6 / +5.4 ticks; as a lag against its running minimum: mean 5.6 (game 2: 6.9), p10 / p50 / p90 1.2 / 5 / 9.8 ticks. The 0-4 tick range of the corpus is the 4-tick frame gap of 30 fps replays. Consequence for the converter: the lag range of a fresh packet is the frame gap; the first-packet tick inference of the dodge fit used 0-4 (fixed now: the frame gap; dodge-fit lags correlate with the truth 0.40 / 0.49 with the old range, 0.55 / 0.62 with the gap).

**The chain-lag inference against the truth** (`rlbot_lag_check.py`, client replays, both games): packets with a chain lag (86-87% of the matched packets) correlate 0.92 / 0.91 with the true lag, error p10 / p50 / p90 -1.0 / +0.1 / +1.1 ticks, |error| p50 / p90 0.5 / 1.5 ticks against a true spread of |p50| 2.2 and |p90| 4.1; packets without a chain lag (4%, source `frame_median` or `default`) carry almost no information (correlation 0.15-0.26, |error| p50 / p90 2.2 / 5.1 ticks); the new dodge-fit lags (0.6-0.7% of the packets) are in between (|error| 1.4-1.5 / 3.5-3.7). Absolute level: the inferred chain lag has mean 4.6 and p10 / p50 / p90 1 / 5 / 8 (true 5.6-6.9, 1 / 5 / 9.8): a slightly compressed copy. On the host replay (true lag 0) the inferred chain lag is 1.6 on average (p50 2): a lag-free replay is not recognised, as found with the BakkesMod dump.

**How late a control change is seen** (`rlbot_control_delay.py`: the tick of each replicated throttle, steer or handbrake change against the same change in the recorded inputs; the packet at n+1 holds the input of tick n to n+1; the replay frame's server tick from the lag-0 offset). Client replay, game 1: steer, throttle and handbrake changes are seen p10 / p50 / p90 4 / 9 / 17 ticks after they happened (mean 9.8-10.9; 59-63% within 10 ticks, 35% between 10 and 20) for bots; humans 3 / 9-11 / 18-26 (mean 10.7-13.4). The delay minus the frame gap has median 0 (p25 / p75 -3 / +3). Host replay: 1 / 1 / 2-3 ticks. The midpoint rule assumes a change happened 2 + gap / 2 ticks before the frame time (7 ticks at a 10-tick gap); the measured mean is about one frame gap plus a replication latency of several ticks on this link (mean 10.9). The rule is too small by about 3-4 ticks here, and the per-event shift range of the fits (-8 to +8 around it) covers it; the latency on other links differs, which is why a rule is a prior and the fit chooses.

**Fitted jump and dodge presses against the true presses** (`rlbot_reconstruction`, fitted tick on the converter timeline mapped to the server tick with the running mode of (timeline tick - server tick - inferred lag); true press = first tick with `has_dodged`, or `Jumping`). Client replay (games 1 / 2): jump n = 252 / 244, fitted minus true p10 / p50 / p90 -4 / 0 / +1 and -4 / 0 / +2 ticks, |error| p50 / p90 1 / 5; dodge n = 197 / 209, -3 / +1 / +5 and -3 / +1 / +6 ticks, |error| 2 / 7 and 2 / 8 (with the first-packet inference; without it only 94 / 107 dodges are fitted, -4 / 0 / +3, |error| 1-2 / 5). Host replay (games 1 / 2): jump n = 597 / 601, exactly the true tick (0 / 0 / 0); dodge n = 268 / 287, 0 / 0 / +2. The online timing fits place the presses within about a tick at the median and within 5-8 ticks at p90 (a 10-tick frame window), validated against the server for the first time.

**Reconstruction against the truth at every tick** (`rlbot_reconstruction`; the exported car state of every active frame against the server state at the frame's server tick; the tick mapping uses the converter's own lag level, so the absolute offset is not scored). Host replays: position p50 / p90 / p99 0.01 / 0.1-0.3 / 18-19 UU, velocity p90 7-9 UU/s, rotation p90 0.47-0.55 deg, angular velocity p90 0.23-0.26 rad/s; with no timing fits 0.2-0.3 / 18-19 UU, 11 UU/s, 0.50-0.63 deg, 0.33 rad/s: the fits give 18-36% on velocity, 6-13% on rotation and 23-28% on angular velocity on a lag-free replay. Client replays: position p50 / p90 0.6-0.7 / 21-22 UU, velocity p90 62-63 UU/s, rotation p90 3.7-3.9 deg, angular velocity p90 1.07-1.08 rad/s; with no fits and no lookahead controls 18.2-18.5 UU (p90), 65-67 UU/s, 3.95-4.02 deg, 1.24-1.25 rad/s: on the real online replay the fits help rotation (-3 to -6% at p90), velocity (-5 to -6%) and angular velocity (-13 to -14%) and do not help position (p90 +14-20% worse, p50 0.3 to 0.7 UU), which is dominated by the ±1 tick error of the lag inference at 1000-2000 UU/s (8-16 UU per tick). The 10-tick frames here mean every frame has a fresh packet: this scores the placement of packets in time much more than bridging. A run with the true lags (the exactly matched packets, offset against the running minimum) gives position p50 0.03-0.10 UU and velocity p50 0.7-3.6 UU/s but larger tails (p90 28-37 UU) because each packet is then extrapolated through its whole true lag (up to 10 ticks) with unknown inputs: the exported state at frame time is an extrapolation whose length is set by the lag level, which no replay reveals (the absolute lag offset).

**Lag levels per chain run are the weak point, not the links** (client replays, game 1; `scripts/rlbot_lag_links.py`, `scripts/rlbot_lag_levels.py`). The interval implied by consecutive chain lags equals the true interval exactly for over 90% of the links (|error| p50 / p90 0 / 0 ticks; standard deviation 0.8 from outliers, 1.6 for intervals over 15 ticks), with no bias by speed, acceleration or height. The absolute level of a run is what goes wrong: against a common baseline (the running minimum of the true offsets) the inferred chain lag correlates only 0.51 / 0.31 with the true lag for cars (games 1 / 2; inferred minus true p10 / p50 / p90 -10 / 0 / +1 ticks) and 0.17 / 0.11 for the ball (p10 / p90 -7 / +3), because each run's offset is placed at the centre of the interval the window [0, frame gap] leaves it, and runs are short (median 19 links, p10 4). Consequences: the exported ball against the server is off by 36 UU at p50 and 64 UU at p90 on a client replay (ball and cars sit on different lag levels, so a contact's timing is wrong by several ticks), and on the lag-free host replay the ball is exact away from cars but 12 UU off at p90 within 300 UU of a car (spurious lags, as found before). Runs of a 4-tick frame window (the corpus) have less slack, so these errors are smaller there; a better level needs information across runs (contact events shared by ball and cars, or the frame-window prior across objects), not better links.

**Corpus check.** The 120 train and validation replays have a fresh car packet in 30-50% of the active car frames (one replay 89%, which may be a host replay) at about 26-29 frames per second, so the corpus is 30 fps client-style data like the client replays here but with a 4-tick frame window: lags 0-4 ticks, control delay about a frame plus latency.

**Next.** (1) Use the true lags to improve the inference for the packets without a chain lag and for the absolute level (what lag distribution does the converter assume when a packet has none: the frame-window prior); (2) recognise lag-free (host, server) replays from all cars being fresh in nearly every frame and use lag 0; (3) replace the midpoint rule's latency constant by the measured one (2 to about 5-6 ticks on this link; unknown for the corpus, so keep the fit); (4) mask and thin these replays' packets to test bridging at longer gaps against the server truth.

## Dodge coverage, the first packet after a dodge, and deferred starts (2026-09-30)

**Where the largest error was.** `error_budget` on train (before this step): the four 'first packet after dodge' partitions hold 25-35% of the squared position, velocity and rotation error of all cars (velocity p90 555-683 UU/s, position p50 13-15 UU, rotation p50 5-6 deg) although dodges are 17,577 of about 946k samples, and only 6,906 of the 17,577 dodge activations (39%) had a fitted start. `diagnose_dodge_coverage` (new; classifies every activation by what surrounds it and prints the refusal counters of `fit_dodge_start` and `fit_ground_flip_timing`) showed: airborne at the last packet 50% fitted, on the ground at the last packet (jump then dodge) 8%; and, per attempt at the last packet before the activation, as many refusals as plans (1,025 refused against 1,080 planned on 20 replays) with one reason: **the fitted start fell after the next fresh packet**.

**Why.** The fit places the dodge from the exact packet after next (the exact-lag packet) and drives only the interval to the next packet. The first fresh packet after the activation frame has no chain lag: a dodge breaks the motion the chain inference relies on, so that packet's lag is the frame median or half the frame gap (source `frame_median` or `default` in the trace windows), an error of up to 4 ticks. Two consequences: (1) a fitted start 'after the next packet' is often really a mis-placed packet (its true tick is later than assumed, or earlier), and it was refused, so the dodge fell back to the impulse at the activation frame and could be lost when the packet reset the state; (2) even when the dodge was right, injecting the packet at a wrong tick left a 15 UU position residual (p50) that persisted to the next packet.

**A negative result on the way.** Simply planning the dodge past the packet (`defer_dodge_past_next_packet`, before the next fix) made the first-packet residual worse (velocity p50 27 to 59 UU/s): at that packet the path without the dodge and the best start before it each reproduced the packet exactly (0.0-0.5 UU) in about half of the cases each (`DEFER_DEBUG` trace on a train replay: 41 refused starts, roughly equally many with err_no_dodge_at_b about 0 and best_before_b about 0). The packet itself says which, but only if its tick is known.

**The method.** With the start and cancel fitted on the exact second packet, the tick of the first fresh packet is the tick within the lag range (0-4 ticks before its frame time) at which the simulated path reproduces that packet (position and velocity): the same exactness argument as the chain-lag inference for the ball, using the flip instead of a free flight. The converter injects that packet at the inferred tick (`lag_overrides`, packet lag source `dodge_fit`) and, when the inferred tick precedes the start (the packet predates the dodge), plans the dodge for the interval after it. For a jump then dodge (`fit_ground_flip_timing`) the same inference runs on the path with the fitted jump shift, start and cancel. Options: `infer_dodge_first_packet_tick` (default on, `--no-infer-dodge-first-packet`) and `defer_dodge_past_next_packet` (default on, `--no-defer-dodge`); both are offline (later packets, packet lags required, never on withheld frames). The plan holds the controls of the packet it was made at (a variant following the observed controls after the first packet was worse: rotation p90 2.45 to 2.35 deg and dodge-window angular velocity p90 0.84 to 0.69 rad/s with the pinned controls). Unit test `dodge_start_fit_infers_the_tick_of_the_first_packet_after_the_activation`: packets generated 1, 3 and 0 ticks before their frame time are recovered with the right lag, and the 3-tick case (before the press at tick 6) is deferred.

**Coverage.** Fitted starts 6,906 to 12,847 of 17,577 activations (39% to 73%): airborne 50% to 79%, jump then dodge 8% to 55% (`diagnose_dodge_coverage`, 60 train replays). Remaining refusals are mostly conditions of the ground fit (a jump counter already odd at the last packet while the car still touches the ground, no exact second packet, inactive frames).

**Result** (`evaluate_corpus --aligned-targets`, one-step car pre-correction residuals; before is `--no-defer-dodge --no-infer-dodge-first-packet`; 60 replays per split):

| Car, simulated one-step | Train before | Train after | Validation before | Validation after |
| --- | --- | --- | --- | --- |
| Velocity p50 / p90 / p99 UU/s | 1.875 / 55.6 / 497.8 | 1.739 / 53.1 / 333.2 | 1.861 / 56.9 / 489.2 | 1.741 / 54.6 / 334.3 |
| Rotation p50 / p90 / p99 deg | 0.295 / 2.765 / 11.13 | 0.293 / 2.663 / 10.47 | 0.288 / 2.763 / 11.04 | 0.285 / 2.675 / 10.43 |
| Angular velocity p50 / p90 / p99 rad/s | 0.0759 / 0.621 / 3.06 | 0.0764 / 0.616 / 2.71 | 0.0731 / 0.630 / 3.02 | 0.0736 / 0.626 / 2.68 |

Per replay (train / validation): velocity p99 better in 60 / 60, p90 in 59 / 54 (worse in 1 / 6); rotation p90 better in 56 / 51 (worse 4 / 8), p99 in 54 / 50; angular velocity p99 better in 58 / 59, p90 in 45 / 36 (worse 15 / 24). The ball tail also fell (one-step ball velocity p99 440 to 409 UU/s train, 457 to 414 validation, from hits by cars whose flips are better placed). The first-packet partitions of `error_budget` (train): position p50 15.0 / 12.7 / 14.9 / 13.5 to 5.1 / 6.5 / 12.6 / 9.1 UU (previous z 120-300 / 50-120 / <50 / >=300), velocity p90 558 / 555 / 683 / 583 to 500 / 513 / 621 / 478 UU/s; the dodge-window partition (all other packets with the counter odd) position p99 27.4 to 24.5 UU, velocity p90 57.1 to 47.7 UU/s, rotation p90 7.50 to 6.94 deg. Masked prediction (`--aligned-targets`, withheld frames, scored against the new offline reconstruction) is flat: p50 velocity -1 to -3%, p90 within 1.8%, p99 within -2.7% to +6.8% (train h1 velocity p99 +6.8%, validation rotation p99 +1 to +3%); the fits do not apply to withheld frames, and the targets moved.

**A dodge counter that does not exist yet** (`diagnose_dodge_coverage`, then a fix). 1,850 of the 17,577 activations (10.5%) had never been fitted by either fit: a car's dodge counter is absent until its first dodge (every new car actor after a kickoff), and both fits refused 'no counter' as if it were odd. Treated as even now: fitted starts 12,846 to 14,582 of 17,577 (73% to 83%; airborne 79% to 88%, jump then dodge 55% to 68%). One-step car residuals (train / validation, before = the state at `41f0635`): velocity p99 333 to 324 / 334 to 326 UU/s, p90 -1.3% / -0.9%, rotation p99 -2.1% / -1.9%, angular velocity p99 -3.3% / -2.5%; per replay velocity p90 better in 55 / 56 of 60 (worse 4 / 4), angular velocity p99 in 60 / 57. Masked prediction unchanged within 1%. Unit test: the dodge-start tests run with an absent counter before the activation. What remains unfitted is 13% of the activations: no car packet with an exact chain lag in the 12 frames after the activation (the dodge breaks the chain of that car for the rest of the flip): those activations are fitted 0%, the others 96%.

**Caveat: the first-packet residual is now partly in sample.** The inferred tick of the first packet is chosen to match it, so the improvement of that partition (position p50 15 to 5-9 UU for airborne starts) is not a held-out prediction; it is a better placement in time of a packet whose tick was unknown. Evidence that is not in sample: the other packets with the counter odd (the dodge-window partition above), velocity p99 of every replay, and the fit's start and cancel, which use only the exact second packet. The group still largest is a dodge whose last fresh packet was near the ground (previous z < 50 UU, 7,481 samples, velocity p90 621 UU/s, position p50 12.6 UU, 55% of them fitted).

**Flip cancel headroom** (`rlbot_flip_step`, 165 forward and backward flips of the bot recording, H ticks after the first packet with `has_dodged`, all cars stepped with the true inputs, the flip's pitch replaced): the best constant cancel over the span (chosen against the target, in sample) reaches rotation error mean 1.01 deg at H = 12 (true inputs 0.88; no cancel 4.33), 1.34 at H = 16 (1.04; 7.34), 1.80 at H = 24 (1.15; 13.6). The best two-parameter step (value from tick t0, also in sample) improves that to 0.99 / 1.18 / 1.65 deg, angular velocity 0.157 / 0.177 / 0.464 to 0.141 / 0.168 / 0.386 rad/s. A step therefore adds at most 2-15% over a constant even in sample, while the constant already removes 77-87% of the no-cancel error: the per-flip step is not worth implementing, and the in-sample versus held-out gap of the cancel comes from the cancel being unobservable before the packet, which no functional form of the flip time fixes. (The keyboard player's 16 flips: no cancel at all.)

**The boost latch was the ground and air boosting tail** (`rlbot_onestep`). RocketSim holds a boost for at least 0.1 s after a tap (`boosting_time`); starting each rollout from a packet with the latch state taken from the falling recorded boost amount instead of the button, the H = 12 tails fall: ground boosting velocity p90 24.5 to 0.9 UU/s, air boosting 33.1 to 5.3, all cars position p90 0.24 to 0.12 UU, velocity p90 5.2 to 2.2 UU/s. RocketSim's latch matches the game. The converter carries the arena's own latch, so it is not affected.

## The 2018 GDC talk on Rocket League physics and networking (2026-09-30)

**Source.** A transcript of "It Is Rocket Science: The Physics and Networking of Rocket League" (Psyonix, GDC 2018), supplied by the user as `external/psyonix_presentation_transcript.txt` (git-ignored with the other user-supplied references, not committed; it is a text reconciliation of three transcripts, not audio-verified). Read in full. What follows is what bears on this project, with the checks I could make against our data.

**What it explains or confirms.**

1. **Replays hold server states, not client predictions.** The server is authoritative, clients predict everything (their car, other cars and the ball), rewind to the server's state and re-simulate the whole scene on a correction. The replay's packets are the received server states: bit-identical in two clients' replays of one match (89% of car packets, `dual_perspective`) and equal to the server-side RLBot recording to 0.00 UU, although each client predicts its own car.
2. **The physics are a deterministic 120 Hz Bullet simulation with discrete collision detection, and the custom vehicle model is simple**: no longitudinal tire friction (a force from the acceleration curve when wheels touch), lateral friction from the side-speed ratio through a two-point curve, friction applied at the height of the centre of mass, masses ignored in forces, roll-only stability torque when wheels touch but not all, and a downward force until three wheels touch. This is why RocketSim can reproduce the game to a fraction of a UU (RLBot recording, H = 12 ticks). The `landing or takeoff` rows are where these auxiliary forces act and are the least exact ground rows (`RESULTS.md`, RLBot section).
3. **Hits depend on the penetration at the contact tick.** With discrete detection the impact normal comes from the contact point at whatever penetration the tick produced (and the game adds custom forces, the 2016 talk by Corey Davis). A car or ball a few UU or a tick off at contact changes the hit; that is the sensitivity behind the contact mismatches and the per-touch tick alignment ('Ball touches').
4. **The server buffers inputs and consumes zero, one or two client inputs per physics frame** ('downstream throttle': repeat an input for two frames when the buffer is low; consume two for one frame when it is full, still catching jumps and dodges; reuse the previous input if it runs empty; the client sends its last 10 inputs). So the control the server applies at a tick is the client's input shifted by a buffer depth of a few frames that changes slowly, with occasional one-tick repeats and merges, and a stalled connection holds the last input. That matches the measured per-event, non-stable timing of control, jump, dodge and cancel changes and argues that a shift per event (our fits) is the right form, not a constant.
5. **Other players' inputs are predicted by decaying the last known input, to neutral by about 150 ms** (client display only). Not in the replay, but a practical prior for causal masked prediction that agrees with our measured persistence table (nothing beyond 0.2 s).
6. **The ball is sent at a regular interval even when nothing changed.** Consistent with a fresh ball packet in 82% of frames and a fresh car packet about every second frame.
7. **Determinism is only approximate.** The simulation is mostly deterministic; the collision order of a ball touching two meshes at once depended on a hash (fixed by them); floating point across platforms is not exact and the corrections absorb it. So RocketSim need not and cannot match to the last bit, and simultaneous contacts (ball at a floor and wall, or a car and a wall) can legitimately differ.

**Tested: state quantization.** Since the April 2018 patch the server quantizes the physics state (compress, then simulate from the quantized state, so clients and server agree). Measured: positions in the replays (corpus 2024 file, both 2026 replays) and in the RLBot recording are exactly on a 0.01 UU grid (100% of values; 0.1 UU grid 10%), the replay velocity and angular velocity on 0.01 replay units, the RLBot velocity on 0.001 UU/s and angular velocity on 1e-5 rad/s. The one-step position error of RocketSim against the game (p50 0.01 UU) is that quantum. Rounding RocketSim's position, velocity and angular velocity to those grids after every tick (`QUANTIZE=1 rlbot_onestep`) changes nothing measurable at 1, 4 or 12 ticks (H = 12, all cars: position p90 0.24 UU, velocity p90 5.2 UU/s, rotation p90 0.04 deg, identical to rounding), so quantization is real but not the cause of the remaining tails (boost, contacts, landings). Rotation quantization was not tested. Exact-tick residuals below about 0.01 UU are quantization, not model error.

**Not in the talk.** Nothing on boost, the jump and dodge rules, the flip cancel, the air control torque, or the ball model beyond the hit normal. The Rocket Science YouTube channel it recommends (deep dives into game physics) and the 2016 talk by Corey Davis on the ball-car hit model might be worth reading for the contact mismatches; the user could supply transcripts as with this one.

## One ranked match from two clients (2026-09-30)

**The data.** `replays/2026-04-05_ranked_dual_perspective/`: two replays of the same online ranked 2v2 (score 2-4, same date and map, `Id`s differ per replay; teammates 'Evhon' and 'madih'), saved by the two clients (game build 260316, network version 11; 9,840 and 9,729 frames, 357 s, 30 fps recording; the corpus has no duplicate matches: the header `Id` is unique across all 120 train and validation replays). No inputs, no server truth. Diagnostic set, not a split. `dual_perspective` compares them.

**Cadence.** Each client's replay has a fresh car packet every 2 frames typically (gap p10 / p50 / p90 1 / 2 / 3 frames, 33 / 67 / 100 ms; 16.4k car packets for 4 cars) and a fresh ball packet in 82% of frames.

**The two clients receive mostly the same server ticks.** 89% of A's car packets (14,595 of 16,383) and 66-67% of the ball packets appear bit-identically in B (a state is the same physics tick if all three position components are equal; positions that repeat, a resting car, are excluded: 14,436 unique-position car pairs, 5,310 ball pairs). So the other client supplies exact states for only about 11% of the car packets and 33% of the ball packets that a replay lacks: little interior ground truth, but nonzero (about 1,800 car and 2,700 ball packets per replay).

**Real online lags, from the same tick seen twice.** The replay-time difference of one physics tick between the two replays is the difference of the two clients' packet lags plus a slowly varying clock offset. For the first 265 s it stays within +-6 ticks in every 10 s bucket (96-100% of pairs), with a triangular shape of width 9 ticks (-5 to +4 around the median, 0 at the mode): the difference of two independent 0-4 tick lags, which is the range the corpus analysis found. A first independent confirmation of the packet-lag model on online data.

**A check of the lag inference against the other client** (`dual_perspective`, ball packets with an inferred chain lag in both replays, 5,265 pairs). For the same tick, (time in A - time in B) x 120 = (lag A - lag B) + clock offset (slow). Subtracting the converter's inferred lag difference and detrending the slowly varying clock offset with a running median:

| Ticks (detrended) | |raw| p50 / p90 | |after subtracting the inferred lag difference| p50 / p90 |
| --- | --- | --- |
| Running median of +-25 pairs | 0.2 / 3.1 | 0.1 / 0.7 |
| Running median of +-100 pairs | 0.7 / 2.5 | 0.3 / 0.9 |

The residual collapses from a spread of about 3 ticks to under 1: the inferred relative lags are right to about a tick against a second, independent copy of the same ticks. The absolute offset stays unidentified (the median absorbs it), as known. This is the first validation of the inference on real online replays; earlier checks were internal (held-out exact-tick residuals) or on offline replays.

**One clock stretches.** After about 270 s the two replays' clocks diverge at -0.4 ticks per second (the time difference for the same tick moves from -4 to -36 ticks between t = 270 s and 350 s, detrended bands 0% inside +-6 ticks). Replay B's game-clock spacing is 1.003 s per game second there against 0.999 s in A (t = 300 s bucket), so replay B's frame times run about 0.3% slow against the game clock for the last 80 s. A frame time is not a fixed multiple of the physics tick over a match; the converter's use of frame time for the timeline and of chain lags for relative timing already tolerates it, but any use of one global offset would not (as with the RLBot recording).

**A frame is not one tick.** Within one frame the inferred chain lags of the ball and car packets are equal in only 33% (replay A) and 30% (B) of the 7,150 and 7,075 frames with at least two such packets; the largest spread inside a frame is 4 ticks (A: 0, 1, 2, 3, 4 ticks in 2,374, 1,740, 2,089, 583, 364 frames). Matching objects to the other replay agrees: of the frames of A with two or more objects found in B, 78% (A to B) and 76% (B to A) have all of them in one frame of B and the rest are split over two adjacent frames, where one tick per frame would give 100%. Each object's own rigid-body update is one tick's state; the objects of a frame come from different ticks (0-4 apart).

**Use.** (1) The lag model and its inference are supported on online data. (2) The unmatched packets of one replay are exact states the other lacks; for the ball, free-flight alignment gives their exact tick (as `align_dump` does), so masked prediction of the ball can be scored online with true states at a real cadence. (3) For cars the tick of an unmatched packet is uncertain by the other replay's lag (+-4 ticks), so a fair scoring needs the ball-anchored tick or a fit that does not use the target.

## An RLBot recording with every car's true state and inputs (2026-09-30)

**The data.** `replays/2026-09-30T10-34-49Z_local_botvsbot_nexto_ripple/` (recorded by the user's `rlbot_dump` script, RLBot v5): a local 2v2 bot match (Nexto and Ripple bots, Octane hitboxes, CHN_Stadium_P) with `states.jsonl` (one RLBot `GamePacket` per physics frame: 4 cars with physics, boost, air state, jump/double-jump/dodge flags, `dodge_elapsed`, `dodge_dir`, `last_input`, and the ball; 61,465 lines covering frame numbers 23 to 60,197, 59,845 consecutive steps, 1,482 repeated lines, 137 gaps of 192 missing frames in all) and the game's saved `.replay` (11,665 network frames; game build 260918, network version 12). Treated as a diagnostic with ground truth, not a split. It is a local match, so it has no replication delay; it is not an online-timing recording (`meta.json` corrected by hand after the run says so).

**The recorder's output is usable.** `seconds_elapsed` equals `frame_num / 120` to 1.4e-5 s, so the packets are the physics ticks. The tools are `rlbot_onestep` (all cars and the ball stepped in one RocketSim arena from a packet) and `align_rlbot` (replay to recording).

**Input alignment (settled).** `last_input` of the packet at frame n+1 is the control applied during the tick n to n+1. Stepping from n with the input listed in packet n leaves errors (H = 12, ground position p90 0.8 UU and rotation p90 0.7 deg, against 0.02 UU and 0.000 deg with the packet n+1 input). The BakkesMod dump (inputs sampled every 4 ticks) could not settle this.

**RocketSim with the true state and inputs is almost exact** (`rlbot_onestep`, 139,709 car-steps at H = 12; H ticks stepped from each packet with the recorded inputs, everything compared with the recorded state at n + H; the packet supplies pose, velocity, boost, jump and flip state; RocketSim's smoothed handbrake, which the packet lacks, is tracked with its own rates, +5/s held and -2/s released):

| Group (H = 12 ticks) | n | Position p50 / p90 / p99 UU | Velocity p50 / p90 UU/s | Rotation p50 / p90 deg | Angular velocity p50 / p90 rad/s |
| --- | --- | --- | --- | --- | --- |
| All | 139,709 | 0.01 / 0.24 / 3.8 | 0.0 / 5.2 | 0.000 / 0.040 | 0.000 / 0.009 |
| Ground, no boost | 62,468 | 0.01 / 0.02 / 0.2 | 0.0 / 0.1 | 0.000 / 0.000 | 0.000 / 0.001 |
| Ground, handbrake | 11,364 | 0.01 / 0.08 / 5.4 | 0.0 / 1.7 | 0.000 / 0.000 | 0.000 / 0.001 |
| Ground, boosting | 21,966 | 0.01 / 0.62 / 4.5 | 0.1 / 24.5 | 0.000 / 0.000 | 0.000 / 0.001 |
| Air, boosting | 8,216 | 0.01 / 0.80 / 5.7 | 0.0 / 33.1 | 0.000 / 0.626 | 0.001 / 0.162 |
| Jump window | 5,133 | 0.07 / 0.34 / 4.7 | 1.0 / 8.1 | 0.000 / 0.849 | 0.001 / 0.134 |
| Flip | 15,022 | 0.01 / 1.26 / 6.0 | 0.1 / 18.6 | 0.000 / 0.660 | 0.001 / 0.067 |
| Near another car | 6,965 | 0.01 / 0.98 / 36.0 | 0.0 / 18.3 | 0.000 / 0.697 | 0.001 / 0.148 |

So the ground physics, jumps, flips, boosting and air control reproduce the game to a fraction of a UU when the inputs and RocketSim's internal state are right. **The ground-driving error of the converter is therefore inputs, their timing and the carried internal state (the smoothed handbrake), not RocketSim's ground model**; the first tool run without the handbrake tracking had ground handbrake errors of 1.2 UU p50 and 185 UU/s p90 at H = 12, all of it removed by carrying the handbrake state. What remains (boosting velocity p90 25-33 UU/s, near-car p99 36 UU) points at the boost-latch state (`boosting_time`, `time_since_boosted`, unobserved) and bumps.

**Caveats found on the way (for `ROCKETSIM_NOTES.md`).** (1) `set_car_state` with `is_on_ground = false` and no wheel contacts while the car physically touches the ground (the `Jumping` state) gave +500 UU/s of horizontal velocity in one tick (frame 1118, player 1 of the recording); setting the contacts consistently fixes it (jump-window velocity p90 1,075 to 8 UU/s at H = 12). The converter carries the arena's own contact state instead of resetting it from packets, so it is not affected. (2) The angular speed cap: the reported flip angular velocity is above 5.5 rad/s while the game's is capped (uncapped angular velocity error 1.75 rad/s p50 at H = 1, 0.000 with the cap applied, rotation identical); already logged as a speed-limit issue.

**The saved replay of a current game build could not be parsed by `boxcars` 0.11.5** (`TAGame.PRI_TA:PlayerStatus` is not implemented; the corpus is game version 868.32.10, this file 868.34.12). `boxcars` 0.12.0 parses it. To check that a pin update is safe, the new `hash_observations` prints a SHA-256 of the extracted observations of every replay: all 120 train and validation replays give identical hashes under 0.11.5 and 0.12.0, and `evaluate_corpus` and `error_budget` reports on a 9-replay subset are byte-identical. The pin is updated to `=0.12.0` (the rule allows a measured change). The converter runs on the new replay (`evaluate_corpus` 1-step car rotation sim p50 0.00 deg, angular velocity 0.0003 rad/s, against 3.3 deg and 0.22 rad/s for holding the previous packet).

**Alignment of the saved replay to the recording.** Every sampled replay frame (1,507 of them, at least three fresh car positions) matches a recorded packet exactly (nearest-car distance 0.00 UU at p10 to p99), so the replay's car packets are exact server states here too and each maps to a recorded `frame_num`. The offset between the replay's timeline (`round(time * 120)`) and `frame_num` is not constant: it drifts from -5 to +8 ticks over the match (per-frame best offset by replay frame: -5 near frame 1,100, 0 near 5,700, +8 near 10,700), so the replay's frame times and the physics tick counter run at slightly different rates in an offline match. An online-timing analysis must therefore use per-frame alignment by exact state matching, not one global offset.

**Use.** This is the recording to reconstruct against: 4 cars, contacts, boosts, flips and jumps, exact per-tick truth and inputs. Next steps: thin the replay's car packets and reconstruct with the converter against the recording (as `dump_reconstruction` does), with the per-frame alignment; test the ground timing fits with true inputs; carry boost-latch state.

### A human on keyboard and mouse against bots (2026-09-30)

**The data.** `replays/2026-09-30T11-09-41Z_local_human_kbm_vs_bots/`: a local 2v2 with the user (keyboard and mouse, digital inputs) and three bots, 22,293 packets (frames 18 to 19,631, 32 gaps of 55 missing frames in all; about 163 s of physics), and the game's saved replay. Local, no network delay.

**The human's `last_input` is the applied control, with the same alignment as the bots'** (`rlbot_onestep`, now grouped by `is_bot`): stepping with the input of packet n+t+1, human cars at H = 12: position p50 / p90 / p99 0.01 / 0.19 / 1.9 UU, velocity p90 2.7 UU/s, rotation p90 0.063 deg, angular velocity p90 0.012 rad/s (bot cars 0.01 / 0.24 / 3.9 UU, 4.6 UU/s, 0.040 deg); with the packet n+t input the human cars are 0.91 UU and 0.74 deg at p90. So an RLBot recording of a human host gives true inputs of the same quality as a bot's.

**The flip cancel is player-dependent, not a common ramp** (`scripts/rlbot_flip_cancel.py`: signed pitch input against the flip's pitch torque, per 120 Hz tick after each forward or backward dodge; positive = pull against the flip = RocketSim's cancel):

| Player | Flips | Cancel input | Timing |
| --- | --- | --- | --- |
| Human, keyboard (this recording) | 20 | never above 0.5 (holds the dodge direction, W, for 24-40 ticks, then releases to 0) | none |
| Bots (Nexto, Ripple), both recordings | 71 + 183 | binary: 44-56% of flips cancel fully, the rest hold the dodge direction | first tick above 0.5: p10 / p50 4 / 4-8; a tick-4 step (RocketSim's gate is 5 ticks, 0.041 s) |
| Human, controller (BakkesMod dump, 9 flips) | 9 | ramp from the dodge direction through neutral to full pull-back | 8-16 ticks |

So the cancel is closer to a per-flip step (a value c from a start tick t0) than a universal ramp: the controller player ramps because an analogue stick returns through neutral, a keyboard player never cancels, a bot cancels at tick 4 or not at all. A prior for the cancel as a function of the flip time cannot be taken from one player, and the mixture is wide (0 for keyboard, 0.5 for bots, a ramp for the controller); the per-flip fit against packets (on the previous interval when causal) stays the right tool, and the fitted constant is a fair approximation whenever t0 is before the fit's span. The next-action item 'cancel as a function of the flip time' should be a per-flip step (value and start tick) fitted from the packets, judged held out, not a fixed curve.

### The fitted inputs against the dump's true inputs (2026-09-30)

**Method.** `ConvertedFrame::fitted_inputs` (new) lists the jump presses and dodge presses (with the dodge's pitch and yaw controls and the pitch cancel) that the timing fits chose at each fresh packet, on the replay timeline; `dump_inputs` thins the dump replay's car packets to every K-th frame (as `dump_reconstruction`), converts with `zero_packet_lag`, and matches each fitted event to the dump's true event (the latest fit whose tick is within 16 ticks of the record where `b_jumped` or `b_isdodging` first shows; the true press lies in the 4-tick window before that record). `dump_flip` runs one-step RocketSim from each dump record of a flip with the true inputs.

**Jump press.** Fitted press minus the tick the wheels leave the ground (the dump's `time_offGround` counts 1/120 s exactly, so the first record with air time fixes the wheels-off tick): -4 ticks at p10, p50 and p90 for K = 2 (11 of 14 jumps fitted) and -5 / -4 / -4 for K = 3 (10 of 14). A constant offset with no spread means the fit locates the press to within about a tick (a constant press-to-wheels-off delay is then a property of the game/RocketSim, not of the fit). Relative to the record grid the fitted press is -1 / 0 / +2 ticks from the first record with `b_jumped` (the true window is the 4 ticks before it). At K = 4 only 3 of 14 jumps are fitted: the jump fit refuses spans over 30 ticks (the second-next packet is 32 ticks away); K = 2-3 is the online cadence. The unfitted jumps at K = 2-3: the first jump of the replay (the dodge counter has no value yet, and the ground-start dodge fit refuses a counter that does not exist), and a few chained or ball-near cases not examined.

**Dodge press.** 8 of 9 dodges are fitted (K = 2 and 3); the fitted press is -1 / 0 / +1 ticks from the record where `b_isdodging` first shows (the true window is the 4 ticks before it, so at most one tick late). One dodge is not fitted (the first of the replay, no dodge counter yet, same reason).

**Dodge direction** (from the replay's `DodgeTorque`, an observation, not a fit). The dump's `DodgeForward`/`DodgeStrafe` are simply the live stick (pitch = -DodgeForward, yaw = DodgeStrafe) at the record, which lags the press by 0-4 ticks. Fitted pitch minus (-DodgeForward): |.| p50 0.08, p90 0.10; yaw minus DodgeStrafe: |.| p50 0.11, p90 0.18, consistent with the stick moving between the press and the record. The direction is right.

**The cancel, in the true inputs.** The player's pitch during a flip is not constant: signed by the dodge's forward component (positive = pull back against the flip's own torque, which cancels it in RocketSim), the true input at successive records after the dodge is, for example, -0.79, -0.25, +0.52, +0.91, 1.0, 1.0 (dodge 138) and -0.59, -0.09, +0.50, +0.91, 1.0, 1.0 (215): the stick goes from the dodge direction through neutral to a full pull-back within 8-16 ticks and holds it; some flips are slower (665 and 775: the pull-back starts 16-20 ticks after the dodge), one holds neutral (405), one stops at 0.5 (1077). Time-averaged over the fitted plan's ticks (press to the next packet, 4-12 ticks) the true cancel is 0.00-0.01 in all cases (K = 2 spans end before the pull-back), while the fit chose 0.5 in 2 of 8 dodges at K = 2 and 1.0 / 0.25 / 0.5 / 0.5 in 4 of 8 at K = 3. RocketSim's own gate (`PITCH_CANCEL_GATE_MIN_TIME` 0.041 s, five ticks) means a cancel in the first ticks of a span has almost no effect, so the fit's value there is chosen by tiny cost differences (its rule takes a candidate only if strictly better by 1e-4) and is not evidence of a real cancel. The reconstruction gains (flip rotation p90 13.5 to 7.8 deg) are real, but the fitted constant is a proxy for the ramp, not the input.

**Physics with the true inputs** (`dump_flip`, 9 flips, one-step from each record with the flip state set from the dodge direction; the angular velocity capped at 5.5 rad/s as the game does, which RocketSim's reported state does not):

| Step i after the dodge record (4 ticks each) | Rotation p50 / p90 deg, true inputs | Angular velocity p50 / p90 rad/s, true inputs | Rotation p50, pitch 0 | Angular velocity p50, pitch 0 |
| --- | --- | --- | --- | --- |
| 0 | 0.53 / 0.83 | 0.264 / 0.540 | 0.53 | 0.264 |
| 1 | 0.39 / 1.01 | 0.130 / 0.377 | 0.39 | 0.130 |
| 2 | 0.58 / 0.97 | 0.427 / 0.627 | 2.22 | 1.379 |
| 3 | 0.46 / 0.80 | 0.157 / 0.433 | 3.54 | 1.985 |
| 5 | 0.36 / 1.00 | 0.053 / 0.528 | 3.74 | 2.073 |
| 7 | 0.23 / 0.47 | 0.106 / 0.334 | 4.91 | 2.804 |

With the true inputs RocketSim reproduces flips to about 0.5 deg and 0.1-0.4 rad/s per 4-tick step: the flip torque, its direction (`flip_rel_torque` from the dodge direction) and the pitch-cancel rule are right, and a step where the pitch is dropped is 4-10 times worse. The best of five cancels per step (oracle) has median 0.00 / 0.00 / 0.75 / 1.00 / 1.00 ... for steps 0, 1, 2, 3, 4 ...: the cancel is a ramp from 0 to about 1 over the first 8-12 ticks, then held. Without the cap the one-step angular velocity was 1.8 rad/s off in every flip step: the game caps the angular speed at 5.5 rad/s and the pinned RocketSim's reported state exceeds it (the converter already caps reported states at 5.5; not yet reproduced for `ROCKETSIM_NOTES.md`).

**What follows.** (1) The jump and dodge timing fits recover the true press ticks to within about a tick when the second-next packet is within their span limits. (2) The cancel should be modelled as a function of the flip time rather than as one constant per interval: none for the first ~8 ticks, then rising to full, which is what the true inputs and the oracle show for 7 of 9 flips (n = 9, one player: a shape, not constants to fit); the fit on the previous interval and the held-out check remain the way to choose the parameter per flip. (3) Spans over 30 ticks get no jump fit; online cadence is mostly within it.

### Thinned reconstruction against the dump (2026-09-30)

**Protocol** (`dump_reconstruction replays/2024.1.13-17.14.8.replay replays/2024.1.13-17.14.8.json 1 2 3 4`). The dump replay has a fresh car packet in every frame and no lag. Only every K-th frame keeps its car body fields (the others repeat the last kept body with its old stamps, so they are not fresh), the thinned replay is converted with the new `zero_packet_lag` option (every fresh packet is at its frame time; the chain lags of this replay are spurious), and the exported car state of each frame is compared with the dump's true state. 210 frames outside active play (goal replay and countdown, which the converter does not simulate) are left out; 1,004 scored frames. `K = 1` is the unthinned replay and shows the floor.

| K | Frames | Position p50 / p90 / p99 UU | Velocity p50 / p90 UU/s | Rotation p50 / p90 deg | Angular velocity p50 / p90 rad/s |
| --- | --- | --- | --- | --- | --- |
| 1 (floor: packet vs dump) | 1,004 | 0.83 / 4.89 / 6.5 | 0.6 / 4.1 | 0.08 / 0.39 | 0.006 / 0.034 |
| 2, dropped frames, all fits | 502 | 0.85 / 5.00 / 6.6 | 1.1 / 15.5 | 0.18 / 0.89 | 0.059 / 0.397 |
| 3, dropped frames, all fits | 670 | 0.89 / 5.05 / 7.3 | 1.7 / 19.0 | 0.32 / 1.59 | 0.115 / 0.615 |
| 4, dropped frames, all fits | 753 | 1.03 / 5.56 / 12.1 | 2.6 / 35.0 | 0.50 / 2.98 | 0.155 / 0.878 |
| 4, 12 ticks after the packet | 251 | 1.21 / 6.14 / 15.1 | 5.4 / 46.1 | 0.88 / 4.48 | 0.178 / 0.882 |

**Position is at the floor.** The replay's own packets differ from the dump by 0.8 UU p50 / 4.9 UU p90 (packet quantization and the dump's precision), and bridging 1-3 dropped frames (4-12 ticks) adds almost nothing to that (p90 5.6 UU at K = 4, worst frame 20 UU). Rotation and angular velocity are within 1-3 degrees p90 at K = 2-4; velocity is the largest remaining error, as in the corpus (ground p90 22-42 UU/s, jump window 48-112 UU/s at K = 2 / 4).

**Effect of the timing fits on this replay** (K = 4, dropped frames; `all fits` / `no timing fits` (ground, jump, dodge and cancel fits off, lookahead controls kept) / `no fits and no lookahead controls`):

| Group | Velocity p90 UU/s | Rotation p90 deg | Angular velocity p90 rad/s |
| --- | --- | --- | --- |
| Flip | 15.0 / 16.5 / 16.5 | 7.78 / 13.49 / 13.49 | 1.68 / 3.44 / 3.44 |
| Ground | 42.1 / 40.1 / 30.2 | 1.04 / 1.04 / 0.98 | 0.336 / 0.336 / 0.330 |
| Jump window | 112 / 104 / 104 | 2.02 / 2.02 / 1.97 | 0.88 / 0.88 / 0.88 |
| Air | 9.3 / 8.9 / 8.9 | 2.74 / 3.84 / 3.84 | 0.78 / 1.15 / 1.15 |

The flip (dodge start and cancel) and airborne rotation fits help on the true-input replay (flip rotation p90 13.5 to 7.8 deg, flip angular velocity 3.44 to 1.68 rad/s, all K = 2-4), independent of the online delay assumption. The ground control timing fit and the lookahead controls **hurt** here (ground velocity p90 30 without them, 40 with lookahead, 42 with the fit), and the jump fit gains nothing. This is expected, not a contradiction of the online result: both encode the online finding that a control change is first seen a frame late (the midpoint rule), and this replay's observed controls are the true controls at the frame time (see above), so the assumption is false for it. It shows that the ground and jump timing assumptions are properties of online replays and would be wrong for offline files; a lag-free replay should be recognised (the `zero_packet_lag` option is the packet part, not automatic yet) and given no lookahead and no ground or jump timing fit. The online timing of controls cannot be tested against this file.

**What the dump cannot test.** Online replication delay (lag 0-4 ticks per actor, controls seen a frame late). Only the physics of the bridging and the flip and cancel fits are validated by it.

## The flip-cancel rule of `external/RLCarInputSolver` on sparse packets (2026-09-30)

**Question.** Does the flip-cancel compensation in the supplied solver (`AirSolver.cpp`, and `inverse_aerial_controls.py`) work as a way to choose the cancel? The earlier comparison ('RLCarInputSolver comparison') covered only its aerial orientation formula and excluded pairs with a dodge, so it had not been tested. Its rule, per state pair: a full cancel (`pitch = sign(local pitch angular velocity)`) when the local pitch angular speed fell by more than 0.05 rad/s per tick, else none; partial cancels are a TODO there, and stalls (yaw and roll rates of opposite sign) are handled separately.

**Protocol.** `flip_cancel_source` (`--flip-cancel-source`): `next-fit` (the default: simulate the candidate cancels and match the next packet, in sample), `external-next` (the rule on the interval to the next packet, in sample), `previous-fit` (the same simulation fit on the previous interval, used for the next; causal) and `external-previous` (the rule on the previous interval, used for the next; causal). Matched dodge events (`trace_dodge_windows --event-lines`; train 14,078 / validation 14,709), squared error of rotation and angular velocity at packets 2-4 after the activation, relative to no cancel fit (`--no-infer-flip-cancel`):

| Cancel choice | Rotation | Angular velocity | Uses |
| --- | --- | --- | --- |
| `next-fit` (default) | 0.678 / 0.672 | 0.626 / 0.618 | next packet (in sample) |
| `external-next` | 1.054 / 1.058 | 1.032 / 1.031 | next packet (in sample) |
| `previous-fit` | 0.872 / 0.869 | 0.945 / 0.937 | past only |
| `external-previous` | 0.970 / 0.979 | 1.047 / 1.051 | past only |
| Two-packet held-out fit | 0.846 / 0.831 | 0.825 / 0.805 | future, first interval left out |
| No cancel fit | 1.000 | 1.000 | |

(Packets 3-4 only, train: `next-fit` 0.484 and 0.428, `external-next` 1.119 and 1.085, `previous-fit` 0.700 and 0.787, `external-previous` 0.884 and 0.978.)

**Findings.** (1) The external rule as translated is not usable on our packets: in sample it is worse than fitting no cancel (1.05 rotation, 1.03 angular velocity), and causally it gains nothing (0.97 and 1.05). (2) A simulation fit on the previous interval, using only past packets, does help: 13% of the rotation error and 6% of the angular velocity error, 30% and 21% at packets 3-4, on both splits. (3) Likely cause of (1), not isolated: the rule is a single-tick heuristic (its threshold scales with `tickDelta`); our packets are 4-12 ticks apart and a flipping car turns through a large angle in that time, so the local pitch axis of the two states differs, and it is binary. That it needs adjacent states is a property of the method, not of its code.

**Use.** `previous-fit` is a causal cancel estimate (past packets only), so it is the natural estimate for a flip interval with no later packet (masked prediction, the last interval of a flip); the converter already holds the last fitted cancel there (`flip_last`), which is the same idea. Not wired into the masked path as a separate step. The default is unchanged.

## The replay's dodge torque is RocketSim's `flip_rel_torque` times (2.60, 2.24) (2026-09-30)

`check_dodge_torque replays/train`: on 5,643 dodge activations with a fresh torque, `(tx / 2.60)^2 + (ty / 2.24)^2` has radius 0.998 to 1.002 for every one (p1/p50/p99 0.998/1.000/1.002; none near the origin, none inside or outside). RocketSim builds a unit dodge direction `normalize(-pitch, yaw + roll)` (x forward, y right) and sets `flip_rel_torque = (-dir.y, dir.x, 0)`, applied times `flip::TORQUE = (260, 224, 0)` (X left/right, Y forward/backward) in the car frame, so the replicated torque is that vector in units of 1/100. The converter's `pitch = -ty / 2.24` and `yaw = -tx / 2.60` follow from it (the x component goes on yaw; RocketSim sums yaw and roll for the side direction, so the dodge is the same either way, and the air-control torque is off on the press tick because the press pitch has the opposite sign to `flip_rel_torque.y` and a side dodge has none). Forward dodges are 5,176 of 5,642 (forward-left 2,690, forward-right 2,486, backward 466); pure-axis dodges forward 382, backward 85, right 54, left 64. The left/right sign is not checked by this distribution.

## Flip cancel: how much of the flip-window accuracy is in sample (2026-09-30)

**Question.** After the dodge start fit, packets 3 and 4 after a fitted start were slightly worse in rotation and angular velocity (1.03-1.09 of baseline pooled per packet). I read the events with the largest harm.

**Windows.** Harm is concentrated in rare events: angular-velocity error rises by more than 3 rad/s in 61, 43 and 21 of about 13,000 events at packets 2, 3 and 4 (about 4% of the baseline squared error there). In the worst events the exported pitch control (the cancel) alternates between 0.00 and 1.00 from packet to packet in the fit-on run, while the baseline holds 0.75-1.00. The per-packet flip-cancel fit (`fit_flip_cancel`) is fitted in sample on the next packet's angular velocity; once the flip's speed saturates at 5.5 rad/s the candidates barely differ, and a small change of the state (an earlier fitted start moves `flip_time` by a few ticks) flips the choice.

**The in-sample fit flatters the flip window.** The cancel fit uses the next packet as its target, so the residual at that packet (and every "dodge counter odd" residual in `error_budget` and `evaluate_corpus`) is not a check. `flip_cancel_holdout` fits the cancel on the *second*-next packet and holds it for the interval to the next one. With the dodge start fit off in both, held-out versus in-sample cancel, matched by dodge event (train): squared error over the first four packets after the activation, position 1.000, velocity 0.998, **rotation 1.113 and angular velocity 1.079** (packet by packet rotation 1.00, 1.07, 1.28, 1.38; angular 1.00, 1.15, 1.30, 1.48). Across the corpus (`evaluate_corpus`, chain-lag packets, train / validation) car rotation p90 goes 2.766 to 2.990 deg / 2.764 to 2.963 and angular velocity p90 0.621 to 0.690 rad/s / 0.630 to 0.694, worse by more than 2% in 58 and 54 of 60 replays, velocity unchanged (55.571 to 55.601 UU/s); "dodge counter odd" rotation squared-error share 0.364 to 0.436, angular 0.209 to 0.284; first packet after a dodge from below 50 UU rotation p50 5.07 to 5.36 deg, angular 0.64 to 0.84 rad/s. So the earlier flip-cancel gains (RESULTS.md 'Flip-cancel inference') are real in the sense of the exported states matching the future packet, but about 8% of the rotation p90 and 11% of the angular velocity p90 of the whole corpus is that in-sample fit, and it is not evidence of prediction.

**Dodge start fit under the held-out cancel.** The comparison that matters for the dodge start (fit on versus off, both with the held-out cancel): train position 0.958, velocity 0.753, rotation 0.969, angular velocity 0.836 of the fit off pooled over the first four packets (packets 3-4: rotation 1.03-1.05, angular 1.04-1.06); validation 0.963, 0.738, 0.986, 0.843 (packets 3-4: rotation 1.03-1.07, angular 1.03-1.10). Under the in-sample cancel it is 0.961, 0.765, 0.958, 0.832 (validation 0.968, 0.752, 0.981, 0.842). So the dodge start conclusion does not depend on which cancel is used, and the small later-packet regression is not removed by holding the cancel out; it stays at 3-10% of rotation and angular error at packets 3-4.

**A constant cancel over the flip is not better (2026-09-30, later the same day).** `flip_cancel_packets` (default 1) fits one cancel jointly over the next N fresh packets of a flip, the state reset to each packet as the converter does, and `flip_cancel_holdout` now leaves the first interval (the one the cancel is used for) out of the sum when later packets exist (it replaces the second-next-packet variant described above, whose numbers stand as measured). Squared error of rotation and angular velocity at packets 2-4 after the activation, relative to no cancel fit at all (matched events, chain-lag packets; train, 14,078 events / validation, 14,709):

| Cancel fit | Rotation | Angular velocity |
| --- | --- | --- |
| Per packet, next packet only, in sample (the default) | 0.678 / 0.672 | 0.626 / 0.618 |
| Joint over 4 packets, in sample | 0.828 | 0.828 |
| Joint over 4 packets, first interval held out | 0.894 / 0.889 | 0.920 / 0.896 |
| Joint over 3 packets, first interval held out | 0.875 | 0.878 |
| Joint over 2 packets, first interval held out | 0.846 / 0.831 | 0.825 / 0.805 |
| No cancel fit | 1.000 | 1.000 |

(Packets 3-4 only: 0.484 / 0.462 and 0.428 / 0.410 for the default, 0.691 / 0.648 and 0.630 / 0.596 for the two-packet held-out fit.) So (1) the honest, predictable part of the cancel is 15-20% of the flip-window rotation and angular-velocity error, against 32-38% for the in-sample fit; (2) the shorter the horizon the better the held-out prediction (0.92, 0.88, 0.83 for 4, 3, 2 packets), so the player's cancel changes during the flip and a single constant over the flip is a worse description than a value per interval; (3) the joint fit is not a better reconstruction either (in sample it is worse than per packet at every packet, 0.83 against 0.68). The default stays the per-packet in-sample fit; `--flip-cancel-packets 2 --flip-cancel-holdout` is the honest predictor to report next to it (and what a cancel guess for a flip with no later packet can be worth: about 17%). A time-varying model of the cancel (for example a value that depends on the time since the start) is the next thing to try, scored the same way.

**Decision.** The default stays the in-sample cancel (the converter is an offline reconstruction and the exported states match the packet they are fitted to), and `--flip-cancel-holdout` (`ConvertOptions::flip_cancel_holdout`) is the honest check to report next to it. Numbers in the flip window of any earlier section (rotation and angular velocity residuals in dodge windows) are in sample for the cancel. Not done: choosing the cancel with hysteresis or jointly over the flip (the player's cancel is roughly constant over a flip), which would remove the 0.00/1.00 alternation; that needs a measurement against the held-out check, not the in-sample residual.

## Ground-start dodge, and dropping the other-car rule from the timing fits (2026-09-30)

**Joint jump and dodge fit.** The largest partition after the airborne dodge fit was the first packet after a dodge whose previous packet is on the ground (7,588 train packets, velocity share 0.21): a jump from the ground followed by a dodge needs the jump shift and the dodge press fitted together (the jump fit refuses a dodge before the second-next packet, the dodge fit needs an airborne start). `fit_ground_flip_timing` (used where the ground and jump fits decline; on with `infer_dodge_start` and `fit_jump_timing`): for a car flat on the ground with even counters, whose jump and dodge counters turn odd before the second-next fresh packet (exact chain lag, up to 45 ticks), one shift of the jump counter's switches and the dodge press tick (relative to the midpoint-rule tick of the activation frame) are searched on that packet (position + 0.1 x velocity) from a saved no-dodge path per jump shift, then the pitch cancel from its angular velocity. The plan drives only the interval to the next fresh packet (the jump input per tick, and a `PendingDodge` if the press falls inside it; the dodge press wins over the schedule and does not cancel the first jump's hold), so that packet stays held out. Unit test `ground_flip_fit_recovers_the_jump_shift_and_the_dodge_press_together`. Measured against the airborne-only fit with the other-car rule at 400 UU: 1,355 more events changed (1,125 with a low start), first packet of the changed low-start events velocity p50 313 to 104 UU/s and rotation p50 4.07 to 3.68 deg, pooled squared error over the first four packets 0.985 (velocity) and 0.978 (angular velocity) of that baseline.

**The other-car rule did not protect the fits.** The fits refused any span with another car within 400 UU at the packets, to avoid contacts that the scratch arena does not model. Dodges and jumps are mostly at the ball and near opponents, so this removed coverage. Varying the radius (train, dodge fits, matched by event against the fit off, squared error pooled over the first four packets after the activation): 400 UU 5,825 events changed and velocity 0.832, angular 0.861, rotation 0.947, position 0.962 of the fit off; 200 UU 6,877 and 0.781, 0.839, 0.955, 0.960; none 7,275 and 0.765, 0.832, 0.958, 0.961 (validation, 400 UU vs none: 6,078 and 7,616 events; 0.823 and 0.752, 0.865 and 0.842, 0.965 and 0.981, 0.966 and 0.967). For the jump fit (train, `error_budget`), first packet after a ground jump start: position p50 2.0 to 1.5 UU, velocity squared-error share 0.045 to 0.037, and "jump counter odd" velocity p90 243.5 to 226.2 UU/s. For the ground fit: near another car (26,266 packets) velocity p90 65.5 to 54.5 UU/s, angular velocity p90 0.93 to 0.82 rad/s, rotation p90 3.28 to 2.93 deg, ground partitions slightly better, nothing worse. So the other-car check is removed from the ground, jump, dodge and joint fits (the ground fit keeps its ball rule, because its scratch ball is parked; the jump and dodge fits simulate the real ball). The cost is that rotation is about 1.5 points worse in the pooled dodge ratio and the later-packet regression grows.

**Final state** (defaults: ground control timing, jump timing, airborne and ground-start dodge, all without the other-car rule; train / validation; baseline is the pre-dodge state of 2026-09-29, `lookahead_ground_controls`, `fit_ground_control_timing` and `fit_jump_timing` on):

| Measure | Before the dodge work | Final |
| --- | --- | --- |
| `evaluate_corpus` car velocity p50/p90/p99 UU/s | 2.254/62.515/561.046 / 2.241/63.408/551.359 | 2.048/59.266/499.298 / 2.036/60.031/494.427 |
| `evaluate_corpus` car rotation p90 deg | 2.881 / 2.862 | 2.836 / 2.826 |
| `evaluate_corpus` car angular velocity p90/p99 rad/s | 0.662/3.454 / 0.670/3.389 | 0.648/3.095 / 0.656/3.065 |
| `evaluate_corpus` car position p90 UU | 4.746 / 4.797 | 4.545 / 4.566 |
| First packet after ground jump start, velocity squared-error share | 0.055 | 0.036 |
| First packet after a dodge, previous z < 50: position p50 UU, rotation p50 deg | 15.5, 5.37 | 15.0, 5.17 |
| Near another car, velocity p90 UU/s | 65.4 | 54.6 |
| Matched dodge events, squared error vs fit off, velocity / angular / rotation / position | | 0.765 / 0.832 / 0.958 / 0.961 (validation 0.752 / 0.842 / 0.981 / 0.968) |

Per replay against the state before the dodge work, p90 improved for velocity in 60 of 60 replays on both splits, rotation in 49 and 52, angular velocity in 58 and 59; worse by more than 2%: rotation 2 and 1. The matched dodge events changed at the first packet: 7,297 of 14,078 (train), 7,634 of 14,709 (validation); packet 1 alone velocity 0.63 of baseline (both splits), angular 0.73 / 0.75. Later packets: rotation 1.07 / 1.09 and angular velocity 1.09 / 1.16 at packet 4, so the flip that follows a fitted start is slightly worse in its rotation and angular velocity (packet 3 about 1.03-1.06). Non-aligned masked prediction is unchanged bit for bit; aligned masked numbers move by less than 1% (target change).

**What is left.** The first packet after a dodge still has velocity p90 of 550-690 UU/s in the budget: the fit covers about half of the events, the packets without an exact lag on the second-next packet, spans over 45 ticks, and dodges after an air jump are not covered. The later-packet rotation and angular velocity regression after a fitted start (packets 3-4) is the next thing to read in real windows: matched events with the largest harm at packet 4, what the flip state (`flip_time`, cancel) is at the injection of the next packet.

## Dodge start: the same per-event timing, fitted on the second-next packet (2026-09-30)

**Windows first** (`trace_dodge_windows replays/train`: 15,938 dodge activations; frame-by-frame windows with the counters and their stamps, the dodge torque, the exported state and the converter's own residuals). The first packet at or after the counter turns odd often already shows the flip under way (|w| 5.50, or a residual of velocity 588-853 UU/s and rotation 16-28 deg) while the sim had not started it, and the residual at the next packets is smaller. The median event's worst position residual is 19 UU (p90 49.5, p99 84.8). So the start tick of a dodge is another per-event timing, as for the jump and the ground controls.

**Held-out check** (`diagnose_dodge_latency replays/train`; 1,513 airborne events with three exact packets a, b, c, the counter turning odd between a and c with a fresh torque, no other car within 400 UU; RocketSim from a with the ball parked; the dodge pressed as the converter does at the midpoint-rule tick of the activation frame plus d, d = -8..+16, and `cancel` of the pitch torque cancelled in {0, 0.25, ..., 1}; fit on c only, scored at b). Errors at b, position UU / velocity UU/s / rotation deg / angular velocity rad/s, p50 and p90: midpoint rule (d = 0, cancel 0.5) 17.0/58.8, 73.1/514.8, 7.4/31.4, 1.20/5.32; d = +2 13.6/45.2, 62.2/422.5, 7.4/23.7, 1.14/4.73; **d and cancel fitted on c 6.5/23.2, 29.6/308.3, 4.5/16.3, 0.83/3.63**. The fitted shift is concentrated 0 to +4 (12%, 15%, 13%, 11%, 7% at 0..+4) with a tail to +16, and the fitted cancel is 0 in 62% and 1 in 21%.

**Converter.** `infer_dodge_start` (default now on; `--no-infer-dodge-start` on `error_budget` and `evaluate_corpus`) existed as an in-sample fit against the next packet (in `RESULTS.md`, 'Dodge start fit on the updated RocketSim': first packet better, later angular velocity worse, so it stayed off). It now uses the held-out scheme: for an airborne car with a fresh packet and an even dodge counter that turns odd (fresh torque) before the second-next fresh packet, the start tick and the pitch cancel are fitted on the second-next packet (exact chain lag, position and velocity, then angular velocity for the cancel) in a scratch arena with the ball at its state in the main arena, and the plan drives only the interval to the next fresh packet, which is not used by the fit. Refused (the other-car rule that was here is dropped, see the next section up) without an exact lag on the fit target, with a withheld or inactive frame, or when the fitted start falls after the next packet (the normal trigger then applies). The next packet's tick is the converter's own lag for it (exact or not). Unit test `dodge_start_fit_uses_the_second_packet_and_plans_only_the_next_interval`.

**A real harm found on the way.** A first version skipped a next packet without an exact chain lag and treated a later one as the next reset. In about 7% of events a fresh packet at the activation frame (fallback lag) then reset the state under the plan, and the second packet after the activation got worse (position squared error 1.22 of baseline; worst events +75-87 UU). Using the converter's own lag for the first fresh packet fixed it (second-packet ratio 1.00). Matched by event against the fit off: 14,078 train and 14,709 validation events, squared error after/before pooled over the first four packets after the activation, train / validation: position 0.965 / 0.967, velocity 0.844 / 0.833, rotation 0.955 / 0.964, angular velocity 0.880 / 0.881. Packet 1 alone: 0.92 / 0.93, 0.76 / 0.75, 0.82 / 0.83, 0.81 / 0.80; packets 3-4 rotation and angular velocity are 1.01-1.06 (a small regression), the rest about 1.00.

**Corpus effect** (`error_budget`, chain-lag packets, train / validation; `evaluate_corpus`, one-step, aligned targets): first packet after a dodge with the previous packet 50-120 UU: position p50 14.8 / 13.9 to 12.9 / 12.1, rotation p50 6.39 / 6.30 to 5.40 / 5.16; previous packet below 50: rotation p50 6.22 / 6.09 to 5.37 / 5.26, position 16.7 / 16.0 to 15.5 / 14.7; previous packet above 300: velocity p50 38.1 / 39.2 to 27.3 / 28.2. The velocity squared-error shares barely move (0.210 to 0.209) because the tail (velocity p90 550-690 UU/s) is dominated by events the fit does not cover (4,851 of 17,577 activations fitted on train, 5,160 of 18,445 on validation, mostly airborne starts). "Dodge counter odd" (all later packets) gets slightly worse: rotation squared-error share 0.346 to 0.356, angular velocity 0.193 to 0.200 (validation the same). `evaluate_corpus` car velocity p90 62.515 to 60.954 UU/s (validation 63.408 to 61.699), angular velocity p99 3.454 to 3.198 (3.389 to 3.140), rotation p90 2.881 to 2.863 (2.862 to 2.848), position p90 4.746 to 4.612 (4.797 to 4.673). Per replay, p90 improved for velocity in 60 of 60 replays on both splits, rotation in 43 and 43, angular velocity in 47 and 49; worse by more than 2%: rotation 2 and 1, angular velocity 1 and 0. Non-aligned masked prediction is unchanged bit for bit.

**What is left.** The largest partition, the first packet after a dodge with the previous packet on the ground (7,588 packets, velocity share 0.21), is not covered: a jump from the ground followed by a dodge needs the jump shift and the dodge shift fitted together (the jump fit refuses a dodge before the second-next packet, the dodge fit needs an airborne start). Also uncovered: dodges with another car near, without an exact lag on the second-next packet, and the small later-packet rotation regression above.

## Jump timing: the counter's press is per-event noise too, and a held-out fit removes the 300 UU/s error (2026-09-29)

**Window reading first.** `trace_jump_windows replays/train` prints frame-by-frame windows around jump activations (counter values with freshness stamps, the converter's exported state and jump control, the packets, and the converter's own pre-correction residuals at the packets). Two lessons. (1) In the baseline nearly every ground jump has a ~300 UU/s velocity residual at the first packet after the counter turns odd (median 295.6 UU/s): the converter presses jump at the counter's frame time, and the packets show the impulse anywhere from before that packet to several ticks later (e.g. packet vz 6.5 at tick 6743, 302 at tick 6751, counter flip at 6740). (2) My first scoring of these windows was contaminated by air jumps: a large residual later in a window was the second (double) jump, whose counter flip is also seen after its physical press; the score now stops at the next jump press or dodge/flip change, and the budget has its own partition.

**Stable latency or per-event noise?** (`diagnose_jump_latency replays/train`: 17,891 events with three exact packets a, b, c around the start, car flat on the ground at a, jump/double-jump/dodge/flip counters otherwise unchanged, no other car within 400 UU; RocketSim stepped from a with the ball parked, midpoint-rule throttle/steer/handbrake/boost and the jump input equal to the counter's parity with every switch shifted by d = -8..+16 ticks; cost = position error + 0.1 x velocity error.) The shift fitted on c is concentrated 0 to +3 ticks after the midpoint rule (66%; 21% at +1, 18% at +2) with a flat tail to +12 (about 2% per value) and almost nothing negative. Scored at the held-out packet b, for the 8,195 events where the jump starts before b (the others are unaffected): midpoint rule cost p50/p90/p99 11.5/45.4/64.6 and vertical velocity error p50/p90 12.3/307.8 UU/s; one corpus constant (+2, chosen on c) 8.5/41.1/59.8 and 6.0/303.1; the median shift of the same player's other events in the replay 6.4/32.4/49.8 and 3.8/295.1; **fitted on this event's c 2.9/18.1/46.0 and 0.6/9.4**. So a stable per-player or corpus latency helps the typical case a little and leaves the whole 300 UU/s tail; only the per-event fit removes it (validation, 18,806 events: 11.1/45.4/64.0 and 11.8/307.5 to 3.0/19.9/47.4 and 0.6/12.3).

**Converter change.** `fit_jump_timing` (default on; `--no-fit-jump-timing` on `evaluate_corpus` and `error_budget`), used where `fit_ground_control_timing` declines: for a car flat on the ground (z < 25) with a fresh packet at a frame, an even jump counter and even double-jump/dodge/flip counters, whose jump counter turns odd before the second-next fresh packet with exact chain lags, one shift (-8..+16 ticks, the midpoint rule at 0, ties keep 0) of the counter's switches is fitted by simulating the span to that packet (spans up to 30 ticks) in a scratch arena, and the interval to the next fresh packet is driven with the shifted jump input per tick (the existing `GroundSchedule`, now with an optional jump value), so the residual at that packet is held out. Unlike the ground fit the ball is in the scratch arena (its state at the packet's time in the main arena), so jumps at the ball are fitted with their contacts; another car within 400 UU refused it in the first version (dropped later, see the section 'Ground-start dodge, and dropping the other-car rule'). Refused also without exact chain lags for both later packets, with a withheld or inactive frame, or a change of double-jump, dodge or flip counter before the second-next packet (a dodge right after the jump is common). Offline (later packets), needs `infer_jump_from_active` and the inferred packet lags. Unit test `jump_timing_fit_recovers_a_press_before_the_counter_shows_it`.

**Effect** (chain-lag packets, `error_budget`; the new partition is the first packet after a jump counter turns odd, previous packet on the ground; train / validation):

| Measure | Before | After |
| --- | --- | --- |
| First packet after ground jump start, position p50/p90 UU | 5.0/12.7 / 5.0/12.7 | 2.4/9.9 / 2.4/10.1 |
| First packet after ground jump start, velocity p50/p90 UU/s | 295.7/311.6 / 295.7/311.1 | 24.3/300.1 / 24.5/300.4 |
| Same, velocity squared-error share of all car packets | 0.131 / 0.135 | 0.054 / 0.057 |
| "Jump counter odd" (later packets), velocity p50/p90 UU/s | 109.3/268.9 / 109.3/263.4 | 97.2/243.5 / 97.2/199.0 |
| All cars, velocity p90 UU/s | 55.9 / 56.5 | 53.0 / 53.6 |
| `evaluate_corpus` car velocity p50/p90/p99 UU/s | 2.321/66.105/560.599 / 2.308/66.951/550.573 | 2.254/62.515/561.046 / 2.241/63.408/551.359 |

Velocity p90 improves in 60 of 60 replays on both splits, rotation p90 in 32 and 32, angular p90 in 47 and 48, and none gets worse by more than 2%. Every other partition (ground, air, dodge, near ball) is unchanged to within 0.1 UU (their shares rise only because the total falls). Non-aligned masked prediction is unchanged bit for bit; aligned masked numbers move by less than 0.5%.

**What is left.** The first-jump partition keeps a velocity p90 of about 300 UU/s (roughly 15% of its packets): the fit is refused when a dodge or double jump follows within the span, another car is within 400 UU, or there is no second-next exact packet in 30 ticks. The air jump (previous packet above 50 UU, mostly double jumps: 3,672 packets, share 0.049) has the same per-event timing and is unchanged, with velocity p50 296.9 UU/s. Both are candidates for the same held-out fit (a dodge-aware version would reuse the flip machinery). A third of the best shifts fall outside 0 to +3, so the counter/physics offset is not a small constant.

## Ground control timing is per-event noise, and a held-out fit removes most of it (2026-09-29)

**Why this step.** The ground tail was first read from aggregates, and I attributed it to unobserved handbrake taps. I then read 10 real train windows frame by frame (5 from each unexplained hard-steer class, `diagnose_ground_driving --trace 5 --trace-stride 397`: observed controls with freshness stamps, packet ticks, true yaw at packets, and the per-tick simulated yaw and lateral velocity with the observed and with a forced handbrake). The windows do not show a missed tap. They show (a) steady slides at 2.5-3.4 rad/s where the handbrake was pressed or released 8-20 ticks before or after the pair (taps every ~8 frames in one window) and the sim's yaw decays differently from the real yaw, in both directions (holds where the sim decays in some, decays faster than the sim in another), and (b) a steer onset where steer 0.9-1.0 was observed for two frames before packet a while the true yaw was still 0.01 at a and built about 3 ticks later than the sim's. Both are timing of an observed control change relative to its effect, off by several ticks per event, which is what the 0-4 tick lag plus the up-to-4-tick reporting delay predicts.

**Is the offset a stable latency or per-event noise?** (`diagnose_control_latency`; the same flat-ground packet pairs as `diagnose_ground_driving`, restricted to pairs with a throttle, steer or handbrake change within 16 ticks: 324,443 on train. Every control switch is shifted by d ticks relative to the midpoint rule, d = -8..+8, RocketSim is stepped from packet a to packet b, and the error at b is angular velocity per 0.3 rad/s plus velocity per 50 UU/s; the scratch arena's ball is parked out of reach.)
- The pair's own best d is spread over the whole range (34.3% at 0, 9.7% at -8, 10.0% at +8, 1.5-5% at every other value), and the best d of consecutive pairs agrees within one tick only 32.6% of the time. Carrying the previous pair's fitted d to the next pair is worse than the midpoint rule (better in 65,925 pairs, worse in 74,366; angular velocity p90 0.700 to 0.753 rad/s). So it is not a stable latency of the player or session.
- **Held-out test.** Fit d on the span a to c (two packet intervals) and score at the interior packet b, which the fit never sees. Train, 294,537 triples: angular velocity p50/p90/p99 0.085/0.656/1.565 to 0.033/0.398/1.067 rad/s, velocity 8.3/63.3/174.1 to 3.9/38.0/117.0 UU/s (d moved in 225,036, better in 134,277, worse in 46,584). Validation, 314,119 triples: 0.082/0.648/1.567 to 0.032/0.407/1.091 rad/s and 8.1/62.0/171.3 to 3.8/38.5/117.3 UU/s. Keeping d only when it halves the cost is worse (0.442 rad/s, 41.9 UU/s on train), so no gate is used.
So the timing of each control change is an unobserved per-event quantity that the next fresh packets identify, like the flip cancel or dodge start; unlike those, the held-out check shows it generalizes.

**Converter change.** `fit_ground_control_timing` (default on; `--no-fit-ground-control-timing` on `evaluate_corpus` and `error_budget`): for a car that is flat on the ground with a fresh packet at a frame, one common shift (-8..+8 ticks, midpoint rule at 0, ties keep 0) of every observed control switch is fitted by simulating the span to the *second* next fresh packet in a scratch arena (ball parked), and the interval to the *next* fresh packet is driven with that schedule (`GroundSchedule`, applied per tick in `step_ticks`). The residual at that next packet is therefore a held-out check, and the reconstruction of the frames in between is the fitted one. Refused without exact chain lags for both later packets, spans over 24 ticks, a withheld or inactive frame, a jump/dodge counter change, a car not flat on the ground at the three packets, the ball within 400 UU (another car within 400 UU refused it in the first version and no longer does), or no control change within 16 ticks. Uses later packets, so it is offline reconstruction; withheld frames and missing packet lags disable it. Unit test `ground_control_timing_fit_recovers_a_change_seen_a_frame_late` (a throttle pressed at tick 5, shown one frame late; the fit recovers the +1 shift and the driven arena reproduces the packet). The first version of the test found a real flaw: a scratch arena's default ball sits at kickoff, where it can touch a car crossing the centre, so the ball is now parked (numbers before and after were the same to three digits).

**Effect** (one-step residual at fresh packets, chain-lag packets; baseline is the previous default, `lookahead_ground_controls` on; train / validation):

| Measure | Before | After |
| --- | --- | --- |
| Ground no boost, velocity p50/p90 UU/s | 7.3/64.6 / 7.2/64.1 | 3.1/38.0 / 3.0/39.5 |
| Ground no boost, angular p50/p90 rad/s | 0.07/0.70 / 0.06/0.70 | 0.02/0.42 / 0.02/0.44 |
| Ground no boost, rotation p90 deg | 2.25 / 2.24 | 1.37 / 1.43 |
| Ground no boost, position p90 UU | 3.4 / 3.4 | 2.1 / 2.2 |
| Ground boosting, velocity p90 UU/s | 61.1 / 60.1 | 42.5 / 42.7 |
| All cars, velocity p90 UU/s | 71.4 / 70.8 | 55.9 / 56.5 |
| All cars, angular p90 rad/s | 0.74 / 0.73 | 0.59 / 0.60 |
| All cars, rotation p90 deg | 2.95 / 2.90 | 2.55 / 2.53 |
| `evaluate_corpus` car velocity p50/p90 UU/s | 4.151/81.493 / 4.151/81.102 | 2.321/66.105 / 2.308/66.951 |
| `evaluate_corpus` car rotation p90 deg | 3.238 / 3.198 | 2.881 / 2.862 |
| `evaluate_corpus` car angular p90 rad/s | 0.804 / 0.801 | 0.665 / 0.672 |
| `evaluate_corpus` ground angular p90 rad/s | 0.786 / 0.780 | 0.567 / 0.581 |

Against the original converter (before `lookahead_ground_controls`) the ground angular p90 has gone from 1.02 to 0.57 rad/s and the car velocity p90 from 85.9 to 66.1 UU/s on train. Per replay (60 each), p90 improved for velocity, rotation and angular velocity in 60, 60 and 60 train replays and 59, 60 and 60 validation replays; none got worse by more than 2%. Unchanged to within 0.1: wall and ramp cars (not fitted), air, jump/dodge windows, near a ball or another car, ball near a car, and the Parquet/JSONL parity and restoration tests. Non-aligned masked prediction is unchanged bit for bit; aligned masked numbers move slightly because the offline target improved and the masked predictions did not (horizon 1 car velocity p50 7.583 to 7.984 train, 7.856 to 8.235 validation; horizon 4 p90 168.6 to 170.5 and 153.9 to 155.6).

**Limits.** Only flat-ground cars away from the ball and other cars are fitted (wall and ramp driving keeps its 0.83 rad/s p90). The cost weights (0.3 rad/s, 50 UU/s) are scale normalizers, and the shift range (+-8 ticks) is the 0-4 tick lag plus the up-to-4-tick reporting window. About three quarters of fits pick a nonzero shift, many for small gains, and 21% of the moved triples (46,584 of 225,036) are worse than the midpoint rule at b, so individual states are not exact; the medians and tails improve. The handbrake ramp value is carried by the converter's own arena. The remaining steady-slide differences in the traced windows are not explained (see PLAN.md).

## Ground driving: the model is unbiased; control timing was the largest error (2026-09-29)

**Question.** Ground driving is the largest single car error partition (train error budget: ground without boost 24.7% of car position and 26.6% of angular-velocity squared error; velocity p50 8.3 UU/s, rotation p90 2.4 deg). Is that RocketSim's ground model, or the unobserved controls?

**Protocol** (`diagnose_ground_driving replays/train` and `replays/validation`; 394,101 and 420,114 flat-ground pairs). Consecutive fresh packets (a, b) of one car with chain-lag ticks (so the elapsed whole ticks k are exact), both flat on the ground (center z < 30 UU, up axis z > 0.97), jump/dodge counters unchanged and even, boost parity unchanged, active play throughout, no ball or other car within 400 UU at either end. RocketSim is started from packet a, with the handbrake ramp value replayed from the observed handbrake history (a packet state does not carry it; the converter's arena keeps it), and stepped k ticks with the observed controls; compared with packet b in the car's frame. Controls come from several hypotheses, some of which use a future observation (labelled): a's frame held (causal), b's frame held, linear interpolation from a to b, and the controls of every frame between them applied with a time shift.

**Findings (train; validation agrees to the third digit).**

1. **The ground model has no bias.** Signed errors in the car frame are centred on zero for every hypothesis and interval length (forward and lateral velocity, all three angular-velocity axes); intervals of 1-2 ticks are exact (velocity p50 0.6 UU/s, angular p50 0.009 rad/s) and pairs with the same controls at both ends have position p50 0.01 UU, velocity p50 0.0 UU/s, rotation p50 0.00 deg.
2. **The error is control timing.** Pairs whose controls changed between a and b (45%) carry most of it (position p50 0.87 vs 0.01 UU, angular p90 1.19 vs 0.35 rad/s for the a-held hypothesis). Applying each frame's controls at the frame's nominal tick (what a same-frame scheme does) is *worse* than holding a's controls (velocity p90 75.6 vs 72.6 UU/s, angular p90 0.921 vs 0.899). Scanning a time shift for the per-frame controls gives an optimum near -4 ticks (velocity p90 67.3, angular p90 0.659, rotation p90 2.20 deg at -4; -2: 70.7/0.785/2.34; +2: 80.7/0.939/2.72): a replicated change is first seen in the frame after it happened, and that frame's state is itself on average 2 ticks (half the 0-4 tick lag range) older than the frame time. The **midpoint rule** derived from that (switch at `T_g - 2 - gap/2`, no tuned constant) gives 67.3 UU/s, 0.662 rad/s, 2.21 deg, i.e. the scanned optimum. It uses the next frame's controls, so it is offline reconstruction, not causal prediction.
3. **The handbrake is a state.** RocketSim ramps a handbrake value (+5/s held, -2/s released). Starting every pair from zero (as a bare `set_car_state` does) inflates errors near handbrake use; replaying the observed history lowers, for the midpoint rule, velocity p90 67.3 to 59.6 UU/s, angular p90 0.662 to 0.610 rad/s and position p90 3.59 to 3.04 UU (unchanged-control pairs: velocity p90 39.3 to 30.4, angular 0.347 to 0.221). The converter already carries the value in its arena between packets. A handbrake-specific time shift (-24 to +16 ticks) has no optimum.

**Converter change.** `lookahead_ground_controls` (default on; `--no-lookahead-ground-controls` on `evaluate_corpus` and `error_budget`): for a car that is on the ground at the start of a frame interval, throttle, steer, handbrake and boost of the frame that ends the interval are applied from `gap/2 - 2` ticks into it (at least from its start) instead of from that frame's packet time. It needs the inferred packet lags (the rule is about physical ticks), is refused for a frame withheld by an evaluator, and only touches cars whose state is on the ground at the interval start, so aerial controls are unchanged. Unit test `lookahead_ground_controls_drive_the_interval_before_a_frame_and_respect_withholding`.

**Effect** (one-step pre-correction, chain-lag packets; per-partition budgets in `error_budget`; train / validation):

| Measure | Before | After |
| --- | --- | --- |
| Ground no boost, velocity p50/p90 UU/s | 8.3/71.8 / 8.3/72.1 | 7.3/64.6 / 7.2/64.1 |
| Ground no boost, angular p90 rad/s | 0.97 / 0.98 | 0.70 / 0.70 |
| Ground no boost, rotation p90 deg | 2.39 / 2.40 | 2.25 / 2.24 |
| All cars, velocity p90 UU/s | 75.9 / 76.1 | 71.4 / 70.8 |
| All cars, angular p90 rad/s | 0.93 / 0.93 | 0.74 / 0.73 |
| `evaluate_corpus` car velocity p50/p90/p99 | 4.421/85.897/560.812 / 4.474/86.233/550.573 | 4.151/81.493/560.650 / 4.151/81.102/550.539 |
| `evaluate_corpus` car rotation p90 deg | 3.291 / 3.268 | 3.238 / 3.198 |
| `evaluate_corpus` car angular p90 rad/s | 0.990 / 1.000 | 0.804 / 0.801 |
| `evaluate_corpus` ground angular p90 rad/s | 1.021 / 1.030 | 0.786 / 0.780 |

Per replay (60 each), p90 improved for angular velocity in 57 train and 58 validation replays, velocity in 43 and 46, rotation in 37 and 36; worse by more than 2%: angular 0 and 1, velocity 7 and 3, rotation 10 and 6 (largest about 5%, mostly rotation; the air partitions are unchanged to 0.1 UU). No partition of the error budget regresses materially (position p90 changes by at most 0.1 UU; ball near a car velocity p90 8.6 to 8.7). Non-aligned masked prediction is unchanged bit for bit (the option is off without packet lags, and never applies to withheld frames). With `--aligned-targets` the target is the offline reconstruction, which improved while the masked predictions did not, so horizon-4 car velocity p90 moves from 164.9 to 168.6 UU/s (train) and 151.7 to 153.9 (validation); this is a target change, not a prediction regression.

**What is left.** Hard steering with unchanged controls keeps a tail: sim minus true yaw rate times the steer direction has p90 exactly 0.000 and p10 -0.39 (-0.57 above 1,600 UU/s), so the real car turns more than the sim in about 17% of those pairs. Those pairs are already sliding (median lateral velocity 130-216 UU/s, yaw rate about 2.3-2.7 rad/s) and sit next to observed handbrake use (handbrake observed within 32 ticks, median 1) while the handbrake reads off inside the interval; forcing the handbrake on reproduces 7% of hard-steer pairs that the observed setting misses, and 10.7% remain unexplained by either. My first reading was short handbrake taps between frame samples; reading real windows frame by frame did not support it (see the next section up: the differences are control timing per event, several ticks off, and it is now fitted). Reproduce: `diagnose_ground_driving replays/train` (add `--reset-handbrake-value` to start every pair from zero), `error_budget replays/train [--no-lookahead-ground-controls]`, `evaluate_corpus replays/<split> report.json --aligned-targets [--no-lookahead-ground-controls]`.

## Ball touches: the car state is not the limit; car/ball tick alignment is (2026-09-29)

**Question.** After the hit impulse was fixed upstream, 15-25% of real touches are still not reproduced. Is that car state error at the contact tick? Protocol (`diagnose_contact_fit`, `diagnose_contact_model`, train and validation, 1,746 and 1,750 touches): consecutive ball packets (A, B) whose velocity change at B exceeds a ball-only prediction by 50 UU/s, one car within 500 UU of the ball, a fresh chain-lag packet of that car at A's tick or one tick before it. Car and ball are set from their packets and stepped to B in a scratch arena; "matched" means RocketSim produced a `CarHitBall` and the ball velocity at B is within 50 UU/s of the packet. The fits below use the ball packet at B (a future observation, an offline fit, not a prediction).

**Correction to earlier numbers.** The diagnostic started a car packet one tick before A but stepped it only to B - 1, so the car ended a tick short. Fixed by stepping the car alone (ball parked far away) for that tick first. Corrected on train with no workaround (`diagnose_contact_model replays/train 1 --no-apply-hit-impulse`): 84.5% of touches produce a sim hit (was 78.1%, so 15.5% missed, not 22%); 89.4% when the car packet is at A's tick and 73.2% when it is one tick earlier (was 87% and 51%). Real vs sim hit impulse p50 990.3 vs 971.2 UU/s for sim hits, direction cosine 1.00; ball velocity error p50 18.0 UU/s.

**Results (train; validation in parentheses).**

| Measure | Value |
| --- | --- |
| Matched, measured timing | 65.1% (63.9%); 77.0% when the car packet is at A's tick, 37.2% when one tick earlier |
| Matched within a start-position shift grid (+-16 UU along travel, +-8 sideways, +-4 up) | 84.1% (83.5%), smallest shift p50/p90 0.0/5.7 UU (0.0/6.9) |
| Direction of the needed shift, in the car frame (n 323) | right and up centred on 0 (p50 -0.0 and 0.2 UU), forward p50 +1.0 (p10/p90 -4.0/9.7); no per-hitbox offset (octane 1,696 of 1,746 touches) |
| Car position error at the car's own next packet, unshifted vs contact-matched shift | p50 0.88 vs 1.44 UU; the shifted car is closer in only 4.8% |
| Sim NO hit (271): smallest hitbox-to-ball gap over the interval, no contact | p10/p50/p90 -6.6/2.2/99 UU (sim hits: p99 +1.4 UU) |
| Car-alone position drift over an ordinary packet interval | p50 4-5 UU at 9-11 ticks, p90 14-19 UU |

So the shift that makes a touch reproduce is about the size of ordinary drift over a packet interval, but it is not supported by the car's next packet (it worsens it) and has no systematic direction, so the converter does **not** nudge car positions to force contacts. Anchoring the car to its next packet (spreading the miss linearly) made contacts worse (matched 65.6% to 57.0% validation), but that test is confounded, since the car-alone miss at the next packet contains the real collision recoil; it says nothing either way and is kept only as a caveat. About 10% of sim misses show hitbox-ball overlap of more than 6 UU without a hit and a similar share are more than 99 UU apart at the closest approach (probably the touch was by another car or the recorded car is wrong); both remain unexplained. Near misses of a few UU are within one tick of relative motion (a car and ball close at up to 25 UU per tick).

**The one-tick puzzle: relative car/ball timing.** Re-running each touch with the car packet assumed delta ticks later or earlier than the chains say (delta -2..+2), matched touches on train (validation in parentheses):

| Car packet vs ball packet A | n | -2 | -1 | 0 (as measured) | +1 | +2 |
| --- | --- | --- | --- | --- | --- | --- |
| same tick | 1,224 (1,252) | 3.9 | 17.0 | **77.7** | 35.0 | 19.5 |
| one tick earlier | 522 (498) | 4.6 | 20.3 | 37.0 | **44.4** | 18.8 |

Same-tick pairs have a sharp optimum at the measured alignment. For one-tick-earlier pairs the optimum is one tick further (the car packet acts as two ticks early), and it is much weaker, so in roughly half of them the chains misplace the car relative to the ball by a tick. It is not a whole-chain constant: choosing a per-car-chain delta on half of a chain's touches and scoring on the other half lowers matches (71.6% to 69.1% on train, 68.7% to 65.8% validation, 129 and 122 chains), and same-frame packets do not share one tick (the one-tick-earlier group is 378 of 522 touches from the same frame as the ball packet, where "same tick" gets only 21.7%). The mismatch is per touch or per chain segment. A per-touch oracle (best delta for each touch, which uses the ball packet at B) reproduces 78.3% (78.1%) with delta -1..+1, against 65.5% (64.6%) as measured, so relative timing explains at most about 13 points of the missing 35%; the remaining ~22% is the contact model, the hitbox geometry at the tick, or the ball-side.

**Decision.** Nothing enabled. The car state at the contact tick is not what limits touch reproduction, since the unshifted car already reaches its next packet within about 1 UU for touches with short intervals. A converter feature would have to choose the car/ball tick alignment per touch from the ball packet (a discrete five-way fit that uses a future observation, provenance-labelled), and the ceiling is +13 points of matched touches on a ball error that is already small (ball position p90 0.8 UU, and the ball is re-injected at every ball packet), so it ranks below the car error sources (dodge window rotation, ground driving, jump velocity). Logged on the backlog. Reproduce: `diagnose_contact_fit replays/train` and `replays/validation`.

## Dodge start fit on the updated RocketSim: still off (2026-09-29)

**Question.** After the update, does fitting the dodge start tick (`--infer-dodge-start`) help, and does using more than one later packet fix its weakness? Protocol: `error_budget replays/{train,validation} [--infer-dodge-start]`, chain-lag packets, base is the default converter; nothing tuned on validation.

**Impulse model (`diagnose_dodge_fit`, 1,500 train events).** RocketSim's dodge impulse matches the real one: |I| p50 550 sim vs 531 real, ratio p10/p50/p90 0.68/1.00/1.16, direction cosine p10/p50 0.59/0.99 (1,327 forward, 135 backward dodges). So what limits the fit is timing and the unobserved pre-dodge orientation and pitch cancel, not the impulse. The fitted start minus the activation frame's tick has p10/p50/p90 -4/-1/8 ticks. At the next packet a fitted start beats the frame-time start in 1,038 of 1,288 events (position p50/p90 7.4/24.0 vs 19.2/46.8 UU, rotation 5.6/15.6 vs 10.9/23.9 deg), in-sample.

**Two variants in the converter.** (1) Fit against the first fresh packet only; (2) fit against up to three fresh packets in the 12 frames after the activation, position and 0.1 x velocity error summed over packets, angular velocity only counted at packets at or after the start, using exact packet ticks from the lag chains. A test that forced `jump=false` before the scheduled start changed nothing.

First packet after a dodge, position p50/p90 UU and velocity p50 UU/s (rotation p50 deg), base to three-packet fit, train (validation moves the same way):

| Previous z | Packets | Position | Velocity p50 | Rotation p50 |
| --- | --- | --- | --- | --- |
| >= 300 | 1,639 | 14.8/38.3 to 12.8/36.3 | 38.1 to 25.0 | 5.94 to 5.42 |
| 120-300 | 1,359 | 15.7/38.4 to 13.6/37.6 | 29.1 to 18.9 | 6.22 to 4.73 |
| 50-120 | 2,975 | 14.8/39.7 to 11.1/36.5 | 28.2 to 17.9 | 6.37 to 4.89 |
| < 50 | 7,588 | 16.7/48.0 to 14.7/46.7 | 36.6 to 26.8 | 6.21 to 5.12 |

**But the whole dodge window gets worse.** "Dodge counter odd" (90,621 train packets): rotation p90 7.45 to 7.77 deg, rotation squared-error share 0.313 to 0.369, angular velocity p90 0.66 to 0.88 rad/s; validation the same (0.313 to 0.361, 0.66 to 0.88). Position and velocity in that window improve slightly (p99 27.5 to 24.3 UU). The fit trades the first packet against later ones, so the option stays **off by default**. The low-altitude first packets (7,588, previous z < 50, the largest share of first-dodge error) barely move (position p50 16.7 to 14.7 UU) because the jump start, hold and dodge start are not fitted jointly and a ground packet gives no orientation. Not a usable improvement until later-window angular velocity stops regressing; candidate cause is the planned pitch cancel and roll persisting past the fitted packets.

## RocketSim update: 79f4d22 to 0b02051 (2026-09-29)

RocketSim `v3-rust` moved 141 commits (2026-08-26 to 2026-09-28, version 0.2.0 to 0.2.1). The dependency is now pinned to `0b020516c4fc633e0db09dfbfaa2026bcddb058e` (`Cargo.toml`, `Cargo.lock`, `serialization::ROCKETSIM_REVISION`). API changes handled in this repository: wheel contacts are `[Option<RaycastHitInfo>; 4]`, `last_extra_hit_tick` moved from the ball to each car (serialized per car, the ball record keeps a null field), and there is a new `CarLanded` event (serialized). The schema version stays 1, but exports made with the previous revision are rejected by the restoration check because the revision string changed. Unit tests (19), the Parquet/JSONL parity check and exact state restoration (12,485 and 13,290 snapshots) pass on the new revision.

**Re-check of the logged issues (details in `ROCKETSIM_NOTES.md`).** (1) The dropped ball-car hit impulse is **fixed upstream**: `on_hit` applies it after contact solving with `accum = false`. With no workaround, the 2,000 UU/s probe sends the ball out at 2,662 UU/s (1,531 before), the mutator scale 1, 2, 3 gives 2,662, 3,793, 4,925 UU/s, and against 1,746 real touches (`diagnose_contact_model`) the simulated hit impulse is 975.3 UU/s versus 990.3 real (cosine 0.99-1.00, median ball velocity error 11.4 UU/s, 78.1% before the 2026-09-29 diagnostic fix, 84.5% after, of touches produce a sim hit; the workaround on the old revision gave 1,001.5 vs 1,008.5, 13.8 UU/s and 75.9%). The workaround is now off by default (`apply_hit_extra_impulse`, `--apply-hit-impulse`) because it double counts on the new revision. (2) Speed limits applied at the start of the next tick **remain**: a flipping car still reports 7.45 rad/s, so `limit_reported_velocities` stays on. (3) Missed touches: 270 of 1,746 (15.5% after the 2026-09-29 diagnostic fix; 382 or 22% before it, see "Ball touches"). (4) False demolitions the converter corrected: 300 to 162 (train) and 282 to 140 (validation), consistent with upstream demo/bump cone gating. (5) API limits unchanged.

**Corpus effect** (60/60 replays each, zero failures; same defaults; hit-impulse workaround off on the new revision), one-step pre-correction, RocketSim p50 / p90 / p99:

| Split | Field | 79f4d22 | 0b02051 |
| --- | --- | --- | --- |
| train | car position UU | 0.222 / 5.486 / 37.420 | **0.127** / 5.401 / 37.055 |
| train | car velocity UU/s | 5.856 / 87.329 / 567.130 | **4.421** / 85.897 / 560.812 |
| train | car rotation deg | 0.467 / 3.418 / 11.802 | 0.431 / 3.291 / 11.282 |
| train | car angular rad/s | 0.118 / 1.005 / 3.554 | 0.113 / 0.990 / 3.443 |
| train | ball velocity UU/s | 0.010 / 0.014 / 467.473 | 0.010 / 0.014 / 446.713 |
| train | ball rotation deg | 0.000 / 0.028 / 4.184 | 0.000 / **0.000** / 4.097 |
| validation | car position UU | 0.221 / 5.510 / 37.418 | **0.125** / 5.428 / 37.066 |
| validation | car velocity UU/s | 5.825 / 87.714 / 557.815 | **4.474** / 86.233 / 550.573 |
| validation | car rotation deg | 0.459 / 3.400 / 11.754 | 0.423 / 3.268 / 11.118 |
| validation | ball velocity UU/s | 0.010 / 0.015 / 475.673 | 0.010 / 0.014 / 462.940 |

Budget (train, chain-lag packets): ball near a car velocity p90 15.2 to 8.6 UU/s (position p90 1.0 to 0.8); all-car velocity p50 5.3 to 3.3 UU/s, rotation p90/p99 3.14/10.7 to 3.01/10.2 deg. Masked prediction with aligned targets (validation): horizon 4 car position p50/p99 3.622/60.188 to 3.524/58.433 UU, car rotation p50/p90/p99 2.706/13.122/37.237 to 2.619/12.606/35.985 deg, ball position p99 56.4 to 55.5 UU; horizon 1 ball position p99 30.8 to 32.0 UU (slightly worse). The free-flight ball and airborne car tick audits still match to storage precision (ball position p50/p99 0.0051/0.014 UU; 95.1% of pairs below 0.01 UU versus 97.3%, car 89.7% versus 91.5%, unexplained but tiny). The remaining unresolved items (missed hits, flip and jump start timing, ground driving) are unchanged in kind. `replays/test` remains sealed.

## Ball-car contact: the pinned RocketSim drops the hit impulse (2026-09-29)

**Question.** After exact lag chains, ball residuals are essentially exact away from cars (position p90 0.009 UU), but ball near a car held 66% of ball position and 92% of ball velocity squared error. Is that limited by car replication, or by the contact model?

**Car age (`diagnose_contacts replays/train`).** Ball residuals with a car within 300 UU, by physical ticks from the ball packet to the car's nearest chain-lag packet: ball velocity error p90 is 9.8 UU/s at 0-2 ticks (64,438 residuals), 105.1 at 3-5 (41,500), 1,065.2 at 6-8 (4,166) and 1,075.2 at 9-11 (2,053); position p90 0.57, 1.84, 13.78, 14.55 UU. So car extrapolation limits most of the spread, but the tail persists next to a car packet (p99 velocity error 1,454 UU/s at 0-2 ticks). Classifying intervals with a car packet within 2 ticks by a *real* touch (ball velocity differs from a ball-only RocketSim prediction by more than 50 UU/s) and by whether RocketSim produced a `CarHitBall`: no touch either 53,347 (velocity error p99 11 UU/s); real touch with a sim hit 3,607 (error p50/p90 683/1,425 UU/s against a real impulse p50 of 1,279); real touch and sim no hit 856 (error p50 562); phantom sim hit 1,807 (p99 1,160). Even with a fresh car, hits were about half right.

**Contact model with a nearly exact car (`diagnose_contact_model replays/train [scale]`).** 1,773 real touches with a single car within 500 UU and a chain-lag car packet at the earlier ball packet's tick or one before; only that car (with its hitbox and observed throttle, steer, handbrake, boost) and the ball are simulated to the next ball packets. Result: RocketSim's ball impulse is about **half** the real one (median 265 vs 946 UU/s overall, 526 vs 1,007 UU/s for sim hits; 78 vs 367 on the ground) while its direction is right (median cosine 0.97), across ground, airborne, dodging and jumping cars and boost on or off. A head-on probe (`simulate_hit_probe`) shows a 2,000 UU/s car sending a stationary ball out at only 1,531 UU/s, slower than the car.

**Cause.** RocketSim's `on_hit` adds an extra ball-car impulse (0.65 x relative speed at low speed, scaled by `ball_hit_extra_force_scale`) to the ball's per-tick accumulator (`accum_lin_vel`). The `CarHitBall` event reports it (`extra_hit_vel`, 1,133, 2,267 and 3,400 UU/s at scale 1, 2, 3 for the 2,000 UU/s probe) but the ball's speed after the hit is identical in all three runs (1,530.6 UU/s): the impulse never reaches the ball, because the accumulator is cleared at the start of the next tick. All five RocketSim revisions in the local cargo cache (2026-06-22 to 2026-08-26, including the pinned `79f4d22`) have this structure. The intended impulse is part of the game's hit response.

**Fix (`apply_hit_extra_impulse`, default on; `--no-apply-hit-impulse`).** `step_tick_with_hit_impulse` adds the reported `extra_hit_vel` of every `CarHitBall` event to the ball's velocity at the end of the same tick, leaving the dependency pin unchanged. A unit test checks the head-on hit: the reported impulse is present in the ball's speed only with the workaround.

Contact-model test, sim impulse and velocity error against the real touches (impulse off, then on): all touches 261 vs 525 UU/s sim impulse (real 946) and ball velocity error p50/p90 561.5/1,457.7 to **28.5/1,024.6** UU/s; sim hits only, sim impulse 526 vs 1,001.5 (real 1,008.5), error p50/p90 534.7/1,331.7 to **13.8/199.4**; direction cosine p10/p50 0.79/0.97 to 0.99/1.00; at the next ball packet (contact finished) error p50/p90/p99 481.4/1,315.3/1,823.2 to **25.1/354.6/1,048.9**.

Corpus (60/60 replays each, zero failures): one-step ball linear-velocity p99 1,073.4 to **467.5** UU/s (train) and 1,091.0 to **475.7** (validation), ball angular p99 1.457 to 1.218 and 1.509 to 1.273 rad/s, ball rotation p99 4.372 to 4.184 deg; per replay ball velocity p99 improved in 60/60 (largest p90 regression 0.0035 train, 0.475 UU/s validation, on a scale of thousands). In the budget, ball near a car velocity p90 falls 52.4 to **15.2** UU/s and its share of velocity error 0.916 to 0.880; car metrics are unchanged. Masked prediction with aligned targets, ball position p99 at horizons 1-4: train 32.6/52.8/89.2/121.4 to 30.9/44.1/59.5/70.4 UU, validation 34.5/48.0/76.1/111.3 to 30.8/36.4/45.6/56.4 UU; velocity p99 h1/h4 train 960/1,460 to 650/1,163, validation 917/1,435 to 540/1,001 UU/s.

**What is left of ball contact.** With the impulse applied, sim hits reproduce the real impulse (median 1,001.5 vs 1,008.5 UU/s), so the remaining ball-contact error is mostly *missed* or phantom hits: 428 of 1,773 real touches (24%) still produced no sim hit (error p50 727 UU/s), and the hit rate falls from 87% with the car packet at A's tick to 51% when it is one tick stale, i.e. a few UU of car position error decides whether a touch happens. That is the car-replication limit you suspected, now isolated: better car state at the contact tick (between sparse packets, the ground driving model, flips) is what remains, and the ball packets themselves reveal contact timing and car position offline. Reproduce: `diagnose_contacts`, `diagnose_contact_model [scale] [--no-apply-hit-impulse]`, `simulate_hit_probe` (env `EXTRA_SCALE`, `NO_APPLY`), `error_budget [--no-apply-hit-impulse]`, `evaluate_corpus [--aligned-targets] [--no-apply-hit-impulse]`. `replays/test` remains sealed.

## Exact whole-tick lag chains (2026-09-29)

**Problem.** Position p90 stayed near 13 UU in every category of the budget and the ball in free flight still showed p90 7.3 UU, which I attributed to lag inference. `audit_lag_accuracy replays/train` checks that against RocketSim's exact whole-tick identification on 105,360 clean free-flight ball frame pairs: the tick count implied by the inferred chain lags was exactly right for only **91.1%** of pairs (off by one tick for 8.6%, by two for 0.2%). Physical tick differences between packets are integers, so those errors were an artifact of rounding continuous, noisy lag estimates independently.

**Measured basis for the fix.** The motion-based interval estimate is very close to the exact whole tick count: ball |estimate - exact k| p50/p90/p99/p99.9/max = 0.001/0.006/0.016/0.029/1.291 ticks (one outlier), airborne cars (5,536 exact fits) 0.004/0.016/0.045/0.068/0.094. So an estimate more than 0.25 tick from an integer is not reliable, and otherwise it can be snapped.

**Method (`exact_tick_lag_chains`, default on; `--no-exact-tick-lag-chains`).** `chain_packet_lags_exact` snaps every chained interval to an integer (pairs more than 0.25 tick from one end the run), so each packet's physical tick is `S = S0 + K` with integer `K` and lag differences are exact. A packet was generated no later than its frame time and no earlier than the previous frame's time, so on the integer timeline `tl(previous) <= S <= tl(frame)`; a run ends when no integer `S0` satisfies this for every packet (a mistaken interval shows up this way). Within the feasible starts, `S0` is the one that violates the real-time window `0 <= T - S <= window` least (ties to the middle), and the applied lag is the integer `tl(frame) - S`. A unit test with whole-tick synthetic physics recovers every lag exactly, rejects fractional lags, and checks the withheld-frame barrier.

Variants tried on train (ball |k'-k| = 0 share; car position p50/p90 UU): frame-integer chains centered on the feasible interval 99.6%, 0.223/5.566; hard real-time integer feasibility (runs cut whenever no integer fits with 0.15 tick slack) 96.6%, 0.284/10.737, worse because the window model is slightly loose and each cut loses the exact chain; no cuts at all (min-violation start only) 98.8%, 0.243/6.855, because one mistaken interval contaminates the rest of a run; **integer-timeline cuts plus min-violation start (default) 99.6%, 0.222/5.489**. The absolute tick cannot be checked with residual metrics (they are relative); the min-violation rule avoids the systematic one-tick shift that centering an interval about one tick wide can introduce, and the assigned lags come out spread over 0-4 ticks (about 10/33/29/18/10% for the ball on one replay, mean 1.85).

One-step pre-correction error, RocketSim p50 / p90 / p99 (60/60 replays each, zero failures, same samples):

| Split | Field | Before | After (default) |
| --- | --- | --- | --- |
| train | ball position UU | 0.005 / 10.208 / 25.903 | 0.005 / **0.009** / 19.231 |
| train | ball rotation deg | 0.000 / 2.782 / 5.729 | 0.000 / **0.028** / 4.372 |
| train | ball velocity UU/s | 0.010 / 5.443 / 1101.572 | 0.010 / **0.014** / 1073.418 |
| train | car position UU | 0.428 / 15.170 / 38.274 | 0.222 / **5.489** / 37.342 |
| train | car velocity UU/s | 7.400 / 89.100 / 567.531 | 5.875 / 87.378 / 566.508 |
| train | car rotation deg | 0.601 / 3.654 / 11.998 | 0.468 / 3.421 / 11.804 |
| validation | ball position UU | 0.006 / 13.862 / 26.504 | 0.005 / **0.011** / 20.434 |
| validation | car position UU | 0.667 / 16.602 / 38.320 | 0.222 / **5.513** / 37.372 |
| validation | car velocity UU/s | 8.268 / 89.895 / 559.008 | 5.846 / 87.810 / 557.026 |
| validation | car rotation deg | 0.655 / 3.693 / 11.988 | 0.460 / 3.402 / 11.770 |

Per replay (improved/worse/tied): car position p50 and p90 60/0/0 in both splits, p99 43/16/1 (train) and 49/11/0 (validation); ball position p50/p90/p99 60/0/0 in both splits (59/1 on validation p99); car rotation p50 60/0 and p90 58/2 (train), 60/0 and 56/4 (validation); car velocity p50 60/0 in both. The lag inference remains offline reconstruction; masked prediction is unchanged in method but its aligned targets are now more exact: validation masked car position p50/p90 at horizon 1 fall from 2.365/18.993 to 0.734/16.962 UU and at horizon 4 from 7.527/22.694 to 3.640/20.478 UU (train h4 7.361/23.359 to 4.294/20.926), with rotation p50 h1 1.129 to 0.978 deg and no material tail change. So the true four-frame (133 ms) forward position error of the causal model is about 3.6-4.3 UU at the median.

**What is left.** The clean budget (chain-lag packets) now has ball residuals essentially exact except in contact: ball position p90 0.0 overall, and **ball near a car holds 66% of ball position and 92% of ball velocity squared error** (p99 29.6 UU, velocity p90 52 UU/s). For cars, position p50/p90/p99 0.2/4.3/28.7 UU. Next: car-ball contacts, jump activity (18% of velocity error), ground driving and the flip start tick. Reproduce: `audit_lag_accuracy replays/train [--no-exact-tick-lag-chains]`, `audit_tick_integrality`, `audit_car_tick_integrality` (interval accuracy), `evaluate_corpus`, `error_budget`. `replays/test` remains sealed.

## Flip-cancel inference (2026-09-29)

**Problem.** Flips in the replays fade their pitch component at broad times because players cancel them (opposite pitch input); pitch is not replicated, so the converter never supplied the cancel and RocketSim's flip kept its pitch torque for the whole 0.65 s. The aerial inverse skipped every dodge interval. Dodge windows held 74% of car rotation squared error (median 6.3 deg) and 60% of angular-velocity error.

**Method (`infer_flip_cancel`, default on; `--no-infer-flip-cancel`).** RocketSim already models a cancel: an opposite pitch input scales the flip's pitch torque by `1 - |pitch|`. At each fresh car packet while the simulated car is flipping (with a relative pitch torque), `fit_flip_cancel` copies the corrected car state into a scratch arena, simulates the span to the next fresh packet of the same car at its exact physical tick (frame time minus the inferred lag) under cancels of 0, 0.25, 0.5, 0.75 and 1, applies the reported angular-speed limit, and keeps the cancel whose angular velocity is closest to that packet. The chosen cancel drives every interval in the span, and the last fit is held as a persistent input in spans with no later packet. Spans that are longer than 8 frames or 40 ticks, contain an inactive or withheld frame, or change the dodge counter are refused (the barrier keeps masked evaluation causal). The grid step is a resolution, not a tuned gate. This is offline reconstruction for fitted spans; the fitted angular velocity is partly in-sample by construction, so rotation, which is not fitted, is the independent evidence. A unit test recovers a simulated 0.75 cancel exactly and checks the withheld-frame and counter-change refusals.

One-step pre-correction car error, RocketSim p50 / p90 / p99 (60/60 replays each converted, zero failures; sample counts unchanged):

| Split | Field | Without | With flip-cancel (default) |
| --- | --- | --- | --- |
| train | rotation deg | 0.633 / 4.891 / 18.603 | **0.601 / 3.654 / 11.998** |
| train | angular rad/s | 0.139 / 1.322 / 4.363 | 0.124 / 1.017 / 3.619 |
| validation | rotation deg | 0.690 / 4.840 / 18.599 | **0.655 / 3.693 / 11.988** |
| validation | angular rad/s | 0.138 / 1.328 / 4.345 | 0.124 / 1.030 / 3.587 |

Per replay, rotation p50 and p90 improved in 60/0/0 train and 59/1/0 validation replays (improved/worse/tied); angular p50 and p90 in 60/0/0 in both. Position and velocity moved by at most 0.4 UU/s. In the dodge partition of the budget (chain-lag packets): rotation p50/p90/p99 6.31/17.17/27.9 to **2.41/9.42/22.5** deg, angular p50/p90 1.02/3.67 to 0.21/1.25 rad/s; whole-car rotation p90/p99 4.59/18.4 to 3.43/11.0 deg.

Masked prediction with aligned targets (causal: only the held cancel from earlier fitted spans can act inside a withheld window), car rotation p50/p90/p99 deg, before to after. Train: h1 1.167/7.303/22.426 to 1.126/5.768/17.704; h4 3.134/17.665/45.403 to 2.988/13.612/38.513. Validation: h1 1.175/7.305/22.892 to 1.129/5.650/16.496; h2 1.428/8.411/24.086 to 1.377/6.212/18.680; h3 2.131/12.443/38.044 to 2.058/9.379/28.777; h4 2.930/17.600/47.411 to **2.791/13.029/36.378**. Angular p90 h4 2.487 to 2.183 rad/s (validation); position unchanged.

**Remaining.** After this change the budget (chain-lag packets, car residuals) puts rotation squared error in dodge windows (56% share, p50 2.41 deg: the flip start tick inside a span and roll input are still unmodelled), ground no boost (11%, p90 2.47 deg) and air (13%); angular-velocity error in ground driving (24%, p90 0.98 rad/s) and dodges (33%); velocity error in the jump partition (18%, median 123 UU/s) and ground driving (14%). Reproduce: `error_budget replays/train [--no-infer-flip-cancel]`, `evaluate_corpus replays/<split> ... [--aligned-targets] [--no-infer-flip-cancel]`. `replays/test` remains sealed.

## Recovering input timing by fitting (jump probe, 2026-09-29)

`diagnose_jump_timing replays/train 400` tests whether an input's tick can be recovered by iterating candidates against exact packet ticks, with an out-of-sample check. For 400 jump activations (jump counter even to odd) with a chain-lag ground packet before and at least three packets after, RocketSim was started from the earlier packet with the observed throttle, steer, handbrake and boost, every (start tick D, hold length H in 1-24 ticks) was simulated, the first two later packets were used to fit, and the rest were held out (error = position UU + 0.1 x velocity UU/s). Findings: the fitted start is broad and typically *after* the counter's frame time (p10/p25/p50/p75/p90 = -4.2/-3.0/+3.0/+8.0/+11.0 ticks), the fitted hold is short (p50 6 ticks, p25 1), and the start is only weakly determined: the best start two or more ticks away scores within 1 unit of the best in 36% of events (median margin 1.62). The fit works in sample (p50 12.8, p90 55.2) but held-out error stays large (p50 64.8, p90 165.8), because air control, boost and dodge after the jump are unmodelled; pinning D to the activation frame time (fitting only H) gives 72.8 and 211.4, so fitting the start improves held-out error by about 11-21% and is better in 283 of 361 decided events and worse in 78. So the approach works in principle and is informative, but for jumps the start and hold trade off (a later start with a longer hold mimics a smoother force profile), and a start later than the counter's frame time suggests RocketSim's jump profile or the counter's meaning differs from the game's. It is a diagnostic, not a converter change. `replays/test` remains sealed.

## Re-budget after timing, reported-velocity limits and flip physics (2026-09-29)

**Cleaner budget.** `error_budget replays/train` now uses only packets whose lag came from their own motion chain (1,443,438 residuals; 59,916 without a chain lag skipped), so residuals isolate model error from timing error, and assigns each residual to one exclusive behavior partition so squared-error shares add up. Car residuals (948,684): p50/p90/p99 position 0.4/14.4/33.3 UU, velocity p50/p90 6.7/80.3 UU/s, rotation 0.60/4.59/18.4 deg, angular p50/p90 0.13/1.26 rad/s (after the fix below). Largest shares of squared error:

| Car partition (n) | Position | Velocity | Rotation | Angular velocity |
| --- | --- | --- | --- | --- |
| dodge counter odd (105,092) | 0.251 | 0.383 | **0.737** (p50 6.31 deg) | **0.596** |
| jump counter odd (30,661) | 0.048 | **0.183** (p50 123 UU/s) | 0.010 | 0.027 |
| ground, no boost (369,790) | 0.286 | 0.136 | 0.054 | 0.131 |
| air, no boost (142,518) | 0.123 | 0.027 | 0.057 | 0.043 |
| ground boosting (92,613) | 0.082 | 0.026 | 0.013 | 0.032 |
| all other partitions | 0.21 | 0.23 | 0.13 | 0.17 |

Ball residuals (494,754): ball near a car holds 0.892 of velocity error and 0.905 of angular error; ball in high air holds 0.501 of position error (p90 7.3 UU) although its flight is exact, which is residual lag-inference error. Position p90 is about 12-14 UU in nearly every partition (p50 about 0.4), so about one packet in ten still has a lag wrong by a tick; exact whole-tick identification (see the section above) should remove it.

**Reported-velocity limits (fixed, default on; `--no-limit-reported-velocities`).** RocketSim applies its speed limits (car 2300 UU/s and 5.5 rad/s, ball 6000 UU/s and 6 rad/s) at the *start of the next tick*, after which the flip torque adds up to 2.17 rad/s more, so the state it reports after a step can exceed limits that recorded server states never do (a flipping car showed 7.5 rad/s where replays show 5.50). The converter now limits the reported state after each step; trajectories are unchanged. One-step car angular velocity p50/p90/p99 on train 0.150/2.081/5.268 to **0.139/1.322/4.363** rad/s (validation 0.151/2.061/5.236 to 0.138/1.328/4.345), car velocity p50 8.233 to 7.722 UU/s (validation 8.758 to 8.422), rotation and position unchanged. In the dodge partition angular p50/p90 fell 2.39/4.70 to 1.02/3.67. Under `--aligned-targets` on validation the masked angular p90 at horizon 1/4 fell 1.940/2.835 to 1.609/2.487 rad/s; nothing else changed.

**Flip physics.** Local-frame angular velocity in 2,704 train dodges with a fresh torque (aligned to the first packet with |w| >= 2 rad/s; medians): |w| reaches the 5.50 rad/s cap within 6 ticks and the direction converges on a pure roll about the car's forward axis. Roll-dominant torque (angle < 30 deg): roll 1.99, 4.39, 5.09, 5.38, 5.46 at 0, 6, 12, 18, 24 ticks with pitch 0.2 and yaw 0. Diagonal (30-60 deg): roll 2.44, 4.13, 4.71, 5.38 and pitch 1.30, 1.34, 1.10, 0.48, 0.13 at 30 ticks. Pitch-dominant (> 60 deg): pitch 1.46, 2.02, 3.08, 2.79, 2.75 then 0.72 at 30 ticks and 0.17 at 42. RocketSim (`simulate_flip_profile`, upright motionless car): roll-dominant matches (5.50 held), but it holds a diagonal flip's pitch at 3.5 rad/s and a pitch flip at 5.4 rad/s for its whole 0.65 s torque time. The pitch component's drop time (tick at which it falls below half its peak and stays) is broad, not a fixed timescale: p10/p25/p50/p75/p90 = 6.6/14.7/25.7/61.8/90.6 ticks for pitch-dominant and 6.1/12.2/26.7/70.2/86.2 for diagonal, with a second cluster at 60-96 ticks (RocketSim's 78-tick torque end); only 14-22% show no drop within about 26 ticks. This looks like unobserved player flip cancels (opposite pitch input; RocketSim does model it, scaling the flip pitch torque by `1 - |pitch|` when the pitch input has the sign of the flip's relative torque). Pitch is not replicated, so the cancel timing must be inferred from the angular-velocity packets; the offline aerial inverse currently excludes every interval containing a dodge counter, which is where most rotation error remains.

`diagnose_dodge_start` (600 train dodges, RocketSim started from the last exact packet before the activation, dodge triggered at every candidate tick, fit against later packets at their inferred ticks): with the limit applied the RMS angular-velocity fit error at the best start is p50/p90 2.31/4.27 rad/s (3.19/4.99 without it) and grows with time after the start (packet 0: 1.03, packet 5: 2.91 rad/s p50); the best start relative to the activation frame time is broad and bimodal (p10/p50/p90 -7/0/11 ticks), so a plain RocketSim dodge with no cancel input does not fit the trajectory whatever the start. Early single-event traces show RocketSim's ramp matches the replay when started at the right tick (event 0: (2.25, 0.33, -2.05) vs (2.86, -0.16, -2.53) at the second packet).

**Next.** A flip-aware inverse for dodge intervals that infers the cancel input and its start tick by simulating candidates against the exact packet ticks; then the jump-counter partition (18% of velocity error), exact whole-tick lag identification, and ball-car contacts. Reproduce: `error_budget`, `diagnose_dodge_start replays/train [max_events]`, `simulate_flip_profile` (no replay data). `replays/test` remains sealed.

## Replay packets are exact server ticks; aligned masked targets (2026-09-29)

**Question.** Is a replay packet an exact 120 Hz physics state, and what does its frame time mean? This decides how timing is modelled, how the state at a frame time is defined, and how error is measured. The tests below use RocketSim itself as the forward model, so they do not depend on the lag-inference heuristics. All are offline **train** diagnostics (`audit_tick_integrality`, `audit_car_tick_integrality`, `audit_tick_offsets`; each refuses paths containing `test`).

**Ball packets are exact ticks.** For 101,202 consecutive-frame pairs of clean free-flight ball packets (z 200-1750 UU, away from walls and any car), RocketSim started from the first packet and stepped a whole number of ticks k in 0..12 reproduces the second packet with position error p10/p50/p90/p99 = 0.0029/0.0050/0.0069/0.013 UU (no pair above 0.1 UU), velocity error 0.0057/0.0098/0.0131/0.016 UU/s, and rotation error below 0.001 deg for 94.7% of pairs (the rest below 0.05 deg). Signed residuals converted to ticks lie within +/-0.02 tick for 99.9-100% of pairs; a packet sampled between ticks would show up to +/-0.5 tick (about 10 UU at the median 1,763 UU/s). RocketSim's ball flight matches the server to the replay's storage precision. The best whole-tick count k differs from `round(frame dt * 120)` by -2/0/+2 at p10/p50/p90.

**Car packets too, where motion is predictable.** For 5,756 airborne car packet pairs (up to three frames apart, zero throttle, unchanged boost/jump/dodge counters, clear of walls, ceiling, ball and cars; linear motion is ballistic whatever the unknown pitch/roll), 91.5% match the next packet's position to below 0.01 UU with a whole tick count (median 0.0050 UU), 98.4% within 0.02 tick along the velocity, independent of the frame gap. The 6% that miss are probably unmodelled inputs. Whole ticks between car packets cluster at 9, 11, 2, 10, 7, 8 (a roughly 10-tick replication cadence, unrelated to the frame grid).

**What the frame time is.** It only bounds the packet: on 1,007 unbroken runs of exactly identified ball ticks (median 38 packets), the offset of each packet's physical tick behind its frame time has a within-run range of **3.03/3.99/4.34 ticks (p10/p50/p90; max 7.06)**, slope 0.0009 ticks/frame (no drift; p10/p90 -0.019/+0.022), and lag-1 autocorrelation 0.06. Above the run minimum the offsets are spread roughly uniformly over 0-4 ticks (share per half tick: 17.5, 10.8, 27.3, 9.5, 15.2, 4.6, 10.1, 2.4, 2.4, then about 0), and the increments between consecutive frames' ticks range over 1-9 (mode 4). That is a stationary, independent jitter of about one frame period (4 ticks): each packet is an exact server state from a tick inside the preceding frame interval, and the frame timestamp does not identify which. It supports the window model used by `infer_packet_lag`, pins the per-run constant to about +/-0.1-0.2 tick, and makes half the window (2 ticks) the median prior for an unknown lag. Whole-tick lags are what RocketSim can represent; the frame time itself is real valued, so the state at a frame time is defined only to within about one tick.

**Consequences for error measurement.** (1) Pre-correction residuals at packet time are clean physics-fidelity measures (a free-flight ball is reproduced to 0.005 UU). (2) A raw packet is not the state at its frame time: a masked comparison against it carries an unpredictable offset of up to 4 ticks (about 11-16 UU of position at typical speeds, or several degrees of rotation for a spinning car) that no model can remove. (3) `evaluate_corpus --aligned-targets` therefore keeps lag inference on for pre-window frames of the masked conversion (chains never bridge a withheld frame, so it stays causal) and takes each target from the unmasked offline reconstruction at the same frame time (the packet advanced by its inferred lag) in place of the raw packet. `--aligned-targets --no-infer-packet-lag` reproduces the old numbers exactly.

Masked car errors on validation, RocketSim p50 / p90 / p99, raw-packet targets versus aligned targets (same default options, 10,405/10,623/10,479/10,401 samples at horizons 1-4):

| Horizon | Position UU: raw | Position UU: aligned | Rotation deg: raw | Rotation deg: aligned |
| --- | --- | --- | --- | --- |
| 1 | 16.562/41.040/75.398 | **2.378/18.995/56.487** | 1.821/7.890/20.206 | 1.175/7.305/22.892 |
| 2 | 14.959/38.364/62.182 | **3.086/18.952/48.180** | 1.806/8.183/21.953 | 1.428/8.411/24.086 |
| 3 | 16.198/40.064/76.493 | **5.215/20.057/58.624** | 2.382/11.800/34.916 | 2.131/12.443/38.044 |
| 4 | 15.953/39.604/76.639 | **7.546/22.721/61.044** | 3.019/16.438/44.443 | 2.930/17.600/47.411 |

Most of the earlier masked position error was target timing noise: the true horizon-four forward error is about 7.5 UU at the median. The aligned rotation tail is a little worse because the aligned truth itself advances the packet a few ticks with default air controls. The aerial-control decisions were tuned against the noisy target, so they were re-tested under aligned targets (horizon-four rotation p50/p90/p99 deg; validation, then train): no persistence 2.982/21.795/50.361 and 3.182/22.306/49.776; no persistence and no handbrake roll 3.005/22.915/50.599 and 3.214/23.355/50.297; calibrated persistence (default) 2.930/17.600/47.411 and 3.134/17.665/45.403; tuned legacy gate 2.879/17.038/45.411 and 3.105/16.682/42.669. The conclusions stand (persistence and handbrake roll help; the tuned gate keeps a small tail edge); only the noise floor moved. Position is unaffected by aerial controls (7.564 to 7.546 UU p50).

**Limits and follow-ups.** The chain-based lag inference is approximate; an exact identification by whole-tick RocketSim simulation (as in the audits) would be more robust for ball and airborne cars. Ground cars need the unknown inputs. Aligned targets rely on the offline reconstruction, so they are only as good as the inferred lags (tiny where identified); do not use them with `--no-infer-packet-lag` expecting new information. Remaining masked error on aligned targets is physics (contacts, driving, aerial control): position p90 about 22 UU at horizon four.

Reproduce: `audit_tick_integrality`, `audit_car_tick_integrality`, `audit_tick_offsets` on `replays/train`; `evaluate_corpus replays/<split> target/<name>.json --aligned-targets` (add the aerial flags for the re-test). `replays/test` remains sealed.

## Error budget and packet-lag inference (2026-09-29)

**Why this step.** After the aerial-control work the question was whether the largest error sources were being addressed first. `error_budget` (train, default options before this change) splits the converter's own one-step pre-correction residuals by object, packet gap, regime and direction of travel. Car position error was p50/p90/p99 17.0/42.0/67.9 UU (1.0 million samples, 68,891 above 50 UU) and ball position 10.7/34.8/64.8 UU, against car rotation p50 1.73 deg, so aerial rotation was a small part of the total. **97-99% of position error is along the direction of travel (`along` 0.97 car, 0.98 ball)**, and even a free-flight ball with 5 UU/s velocity error missed by 10.7 UU: a timing error, not a physics error. Car share of squared position error: ground 0.498, airborne 0.393, near ball 0.063, wall/ramp 0.047; frame gap 2 carried 0.408 and gap 3 0.274.

**Finding (offline train audits, `audit_ball_ticks`, `audit_shared_timing`, `audit_car_ticks`).** Replay frame times are regular (median 33.3 ms, 4 ticks), but the 120 Hz ticks implied by an object's own motion between consecutive packets are not. For 182,518 free-flight ball frame pairs with nominal 4 ticks the implied count spreads over 1-9 ticks, and the lag-1 autocorrelation of implied minus nominal is -0.42 (about -0.5 for independent per-frame jitter on a regular grid). Consistent with each packet having been generated at an arbitrary time inside its frame window `(previous frame time, frame time]`, so its state is valid up to 0-3 ticks (0-25 ms) *before* the frame time the converter assigned, which is 0-37 UU at 1500 UU/s. Car packets have their own timing: ball and car lags correlate at only r = 0.30 (`audit_shared_timing`: car consecutive-frame packets imply about 2 ticks fewer than nominal), while two cars over the same frame pair correlate at 0.82. The physical spacing between consecutive car packets is about 9-11 ticks regardless of whether the frame gap is 2 or 3, with occasional extra packets about 2 ticks after a regular one, and cumulative implied time equals frame time on average (per-lifetime drift p10/p50/p90 = -2/0/2 ticks).

**Method (`infer_packet_lag`, default on; `--no-infer-packet-lag`).** For each ball and each car actor lifetime, chains of consecutive fresh packets give the implied elapsed ticks from displacement along the mean endpoint velocity (exact for constant acceleration, biased only at second order in the turn angle). The chain fixes lag *differences* between packets; each lag must lie in its frame window, so the unknown constant is confined to an intersection interval and centered in it (`infer_packet_lags`). Pair validity: ball smooth motion (speed above 300 UU/s, velocity cosine >= 0.97, speed change <= 25%, <= 2 frames apart); cars speed above 350 UU/s, cosine >= 0.95, speed change <= 40%, <= 8 frames, no dodge counter, all frames active. Runs end at an invalid pair or infeasible window. The converter then splits each replay interval into groups by lag: it steps to each object's packet time, applies that object's correction there (so pre-correction residuals compare like with like), and steps on to the frame time, so every exported state is still the state at the frame time. Lags are per car; a car without its own chain uses the frame median car lag; without inference the default is half the frame window (the median of the roughly uniform lag distribution). Each frame record carries `packet_lag_ticks` (ticks and source: `chain`, `frame_median`, `default`) as provenance; they are inferred, not observed. On one validation replay 95% of ball and 92% of car packets used their own chain lag.

**This is offline reconstruction.** It uses the next packet of the chain, and the fitted timing makes the position residual partly in-sample. Independent evidence: rotation (never used by the inference) and ball rotation, and the ball's velocity when it is unused. One-step pre-correction error, RocketSim p50 / p90 / p99 (train and validation, 60/60 replays each converted, zero failures; sample counts unchanged):

| Split | Field | Without lag inference | With lag inference (default) |
| --- | --- | --- | --- |
| train | car position UU | 16.958 / 41.995 / 67.928 | **0.435 / 15.168 / 38.274** |
| train | car linear velocity UU/s | 22.044 / 108.365 / 579.417 | 8.233 / 89.320 / 568.022 |
| train | car rotation deg | 1.730 / 7.432 / 19.494 | **0.633 / 4.891 / 18.603** |
| train | ball position UU | 10.669 / 34.850 / 64.811 | **0.005 / 10.208 / 25.892** |
| train | ball rotation deg | 2.776 / 5.730 / 11.416 | **0.000 / 2.784 / 5.729** |
| validation | car position UU | 16.391 / 41.095 / 69.570 | **0.674 / 16.603 / 38.320** |
| validation | car linear velocity UU/s | 21.638 / 108.155 / 572.439 | 8.758 / 90.083 / 559.950 |
| validation | car rotation deg | 1.677 / 7.360 / 19.392 | **0.690 / 4.840 / 18.598** |
| validation | ball position UU | 10.680 / 33.886 / 64.281 | **0.006 / 13.862 / 26.508** |
| validation | ball rotation deg | 2.820 / 5.730 / 10.922 | **0.000 / 2.864 / 5.729** |

Car position errors above 50 UU fall from 68,891 to 5,850 on train. The along-track share drops from 0.97 to 0.86, so the remaining error is mostly physics (contacts, driving, aerial control). After the change the car squared-error share is airborne 0.444, ground 0.386, near ball 0.104, wall/ramp 0.066. Per replay, final model versus no inference: car position p50 and p90 improved in 60/60 replays in both splits, p99 in 60/60 (train) and 58/60 (validation, worst +1.10 UU); ball position p50/p90/p99 improved in 60/60 in both splits; car rotation p50 and p90 improved in 60/60 in both splits.

Development path on train (car position p50/p90/p99 UU; car rotation p50 deg): frame-shared lag, strict thresholds, default 0: 7.709/35.069/73.633, 1.154; default half window: 7.374/33.941/70.413, 1.139; relaxed car thresholds: 4.654/33.330/71.140, 1.045; **per-car lags: 0.435/15.168/38.274, 0.633**. The frame-shared version worsened 13% of packets by more than 10 UU (47% improved), which motivated per-car lags. Relaxing the ball's altitude guard gave ball position p90/p99 11.3/28.1 from 15.3/43.6 with the default half window. Threshold values were set by coverage and these train numbers; validation was run only afterwards.

**Masked benchmark.** A withheld target's own lag is unknowable (about uniform over the window), and a correctly timed state carries that full lag against a packet, whereas the old un-timed baseline benefited from two lags partly cancelling: with lag inference on, masked validation h1 position rose 16.56 to 19.49 UU. That comparison is not meaningful, so `evaluate_corpus` disables packet-lag inference for the masked conversion. Masked numbers (aerial-control benchmark) are unchanged. A causal timing model would need the roughly regular 9-11 tick car cadence; it is not implemented.

**Limits and follow-ups.** The chain needs smooth motion; slow, sharply turning or contact intervals fall back to the frame median or the half-window default. Lags are rounded to whole ticks. Earlier aerial-control fits (`solve_span_air_controls`, persistence calibration) still use frame-time deltas; using the inferred packet-time deltas is the obvious next accuracy step and could change those calibrations. Remaining budget items: car-ball contacts (ball near a car: velocity p90 173 UU/s), ground driving and aerial dynamics. The test `live_car_wins_over_retired_car_with_same_player` now pins `infer_packet_lag: false` because it asserts state equals packet.

Reproduce: `cargo run --release --bin error_budget -- replays/train [--infer-packet-lag]`; `audit_ball_ticks`, `audit_shared_timing`, `audit_car_ticks`, `lag_outliers` take `replays/train`; `evaluate_corpus replays/<split> ...` with and without `--no-infer-packet-lag`. `replays/test` remains sealed.

## Input-process calibration replaces tuned persistence gates (2026-09-29)

**Why.** The persistence gate (`max(|pitch|,|roll|) >= 0.5`, 0.15 s expiry) and the 4-frame span cap were chosen by sweeping masked error, so they were hill-climbed constants. Audit of the aerial controls added on this branch: *derived from mechanics or source* were the future-packet barrier, the forward model (mirrors RocketSim `update_air_torque`) and handbrake-as-air-roll (steer correlates with inferred roll at r = 0.64 with handbrake, 0.13 without); *tuned by error sweeps* were the 0.5 magnitude gate, the 0.15 s expiry, the 4-frame span cap and the off-by-default speed-drop gate. This section replaces the tuned constants by quantities measured on the input process itself (no state error involved); the tuned gates remain as `--legacy-persist-gates` for comparison. It supersedes the defaults named in the two sections below.

**Span cap removed.** With no cap (any bracketing pair in the active phase; a constant control over the pair beats no control), one-step train car rotation p50/p90/p99 is 1.731/7.432/19.481 deg for 12 and 40 frames alike, versus 1.743/7.438/19.475 with the tuned 4 frames / 0.15 s. The default is now effectively unbounded.

**Calibration tool.** `cargo run --release --bin calibrate_air_control_persistence -- replays/train [max_span_seconds]` fits the constant control for every pair of consecutive fresh car angular packets in the air (416,600 spans on the 60 train replays; refuses paths containing `test`) and pairs fitted controls of the same actor lifetime at increasing lag. Findings:

- Persistence is strongly axis dependent. Least-squares rho(lag) is about 0.51 / 0.59 / 0.75 (pitch / yaw / roll) at lag 0-0.033 s, and 0.32 / 0.36 / 0.72 at 0.067-0.10 s. Pitch and yaw reach about 0 by 0.15-0.2 s, roll still 0.49 at 0.27-0.30 s.
- There is no threshold structure in the conditional mean (later/earlier ratio roughly constant in magnitude), and a fitted roll of about 0.69 recurs at every magnitude: it is the roll that balances RocketSim's roll damping at the 5.5 rad/s angular speed cap (4.8 x 5.5 / 38.3), so a fit at the cap is a lower bound on a held input and persists. Regressions on the previous control and a cap flag add little (R2 0.11 for pitch/yaw, 0.54-0.57 for roll).
- Conditioning on the joint magnitude of the other axes does not raise the pitch hold probability (0.43-0.49 at every level), so the tuned gate's joint rule has no support in the input process.
- Errors are judged by quantiles of absolute error, for which the optimal point prediction of an uncertain input is its conditional median. The default (`air_persist_calibrated`) therefore multiplies each fitted control by the measured median-later-control ratio `AIR_CONTROL_MEDIAN_RATIO[axis][lag band][|u| bin]` (bands 0.033-0.083, 0.083-0.133, 0.133-0.2 s; nothing beyond 0.2 s or below 0.1 magnitude), and uses observed steer for yaw (or for roll while the handbrake is held) instead of a persisted value. The table is copied from the tool's train output.

Alternatives evaluated on the input process and then on train horizon-four rotation p50/p90/p99 (deg): least-squares shrinkage rho(lag) 3.221/16.180/42.548; persist-iff-P(held)>0.5 3.202/16.682/43.825; **conditional-median ratios 3.187/16.097/42.147**; no persistence 3.227/20.130/45.784; tuned gate 3.189/15.372/39.268. Ignoring observed steer (persisting fitted yaw/roll) made no material tail difference (16.559/43.809).

Masked car rotation (p50 / p90 / p99 deg), no persistence versus calibrated default versus tuned legacy gates. Same samples per row (train 10,120/9,931/9,877/10,053; validation 10,405/10,623/10,479/10,401 at horizons 1-4):

| Split / h | No persistence | Calibrated (default) | Tuned legacy gate |
| --- | --- | --- | --- |
| train 1 | 1.835/8.919/22.652 | 1.809/8.219/20.736 | 1.810/7.944/19.691 |
| train 2 | 1.858/9.247/23.375 | 1.806/8.474/21.059 | 1.813/8.012/20.286 |
| train 3 | 2.642/15.301/38.702 | 2.566/12.329/36.510 | 2.595/11.833/34.193 |
| train 4 | 3.227/20.130/45.784 | 3.187/16.097/42.147 | 3.189/15.372/39.268 |
| validation 1 | 1.859/8.609/22.000 | 1.821/7.890/20.206 | 1.832/7.753/19.150 |
| validation 2 | 1.845/9.175/23.292 | 1.806/8.183/21.953 | 1.791/7.830/21.645 |
| validation 3 | 2.460/14.596/38.075 | 2.382/11.800/34.916 | 2.419/11.278/32.836 |
| validation 4 | 3.060/19.916/46.867 | 3.019/16.438/44.443 | 3.013/15.631/41.734 |

The alternate mask schedule (`--mask-seed 239847`) on validation horizon four: 3.130/20.500/47.885 (previous default without roll/persistence) and 3.046/15.156/42.351 (tuned) versus 3.044/16.027/44.411 (calibrated). Position is unchanged at every quantile (validation h4 15.953/39.604/76.639 UU). Per replay at horizon four, calibrated versus no persistence: rotation p90 improved/worse/tied 54/2/4 (validation) and 59/1/0 (train); p50 32/21/7 and 31/19/10 (worst regression +0.79 deg). Calibrated versus the tuned gate is *worse* at p90 in 43/60 validation replays (worst +6.0 deg) and 40/60 train replays.

**Honest trade-off.** The measured model gives up about 5% of the tail (p90/p99) that the tuned gate captured, while matching its p50, and it needs no error-tuned constant. Diagnosis: forcing *pitch* to persist at full magnitude for |u| >= 0.5 within 0.15 s (a temporary switch, removed) recovers the tail (train h4 3.173/15.275/40.924), while forcing yaw or roll changes nothing. Fitted pitch is bimodal (held or released, about 50/50) so the control-space median cannot decide it, yet state error favors holding. A hypothesis that time-averaging over multi-frame fitted spans attenuates the calibration target was tested by restricting calibration to spans of at most 0.045 s (76,738 spans): pitch rho at lag 0-0.033 s stayed 0.51 and the |u|~1 median ratio 0.45, so the hypothesis is not supported. Open question: a calibration objective in state space (fitted on train with a held-out check on validation, low-dimensional, per axis) may explain the pitch gap.

Reproduce: `evaluate_corpus replays/<split> ...` with defaults, `--no-persist-past-air-controls`, and `--legacy-persist-gates`; `replays/test` remains sealed.

## Residual harm in causal persistence and a speed-drop gate (2026-09-29)

`python/analyze_persistence_harm.py` joins the previous-default and new-default horizon-four **train** traces (10,052 windows) and keeps the 2,541 windows where persistence changed the first-interval pitch/roll (1,417 better and 529 worse by more than 1 deg; median -2.517, mean -4.599 deg). Features use only packets before the mask window. Harm concentrates where prior angular speed had been **falling** between the two fitted packets: trend below -0.2 rad/s gave 110 better versus 140 worse (mean +1.5 deg), while flat trends gave 1,040 better versus 238 worse (median -3.785) and speeds at or above 5.48 rad/s gave 1,014 versus 192 (median -4.261). Speed 5 to 5.48 rad/s (121 windows, 44 better/62 worse) and pitch-dominant persisted controls (657/306, median -1.349, versus roll-dominant 760/223, median -3.854) were the weakest groups. Age since the latest packet (up to 0.12 s), handbrake, altitude and steer magnitude did not isolate harm.

Following that, `--air-persist-max-speed-drop s` skips persistence when angular speed dropped by more than `s` rad/s between the two fitted packets. Train horizon-four rotation p50/p90/p99 (deg) for the current default and gates: none 3.189/15.372/39.268; 0.5 3.182/15.247/39.268; **0.2 3.155/15.213/39.268**; 0.0 3.181/17.220/43.340; -0.1 3.225/19.239/45.702 (tight gates discard helpful windows whose speed merely dips). With 0.2, validation horizon four is 3.013/15.631/41.734 to 2.990/15.561/41.703, the alternate mask schedule 3.046/15.156/42.351 to 3.014/15.004/42.351 (validation) and 3.251/15.917/41.289 to 3.222/15.854/41.133 (train), while horizons one and two move by at most +/-0.015 deg at p50 (validation h2 p50 1.791 to 1.806). Per replay at horizon four, p50 improved/worse/tied 28/10/22 (train) and 26/13/21 (validation); **p90 was 20/7/33 (train) but 16/14/30 on validation with a 3.0 deg worst replay**. The pooled gain (about 0.03 deg p50, 0.06-0.15 deg p90) is small and the per-replay evidence on validation is a wash, so the gate remains **off by default** (`air_persist_max_speed_drop` = 1e6). Keep it as an ablation and revisit only with a stronger discriminator.

Reproduce: run `evaluate_corpus` on `replays/train` with `--no-infer-air-roll-from-handbrake --no-persist-past-air-controls --rotation-trace target/trace-prev.jsonl` and with defaults plus `--rotation-trace target/trace-fin.jsonl`, then `python python/analyze_persistence_harm.py target/trace-prev.jsonl target/trace-fin.jsonl`. `replays/test` remains sealed.

## Causal past-control persistence and handbrake air roll (2026-09-29)

Two changes target *masked* (causal) prediction, which the gap-spanning inverse above leaves unchanged. Both are enabled by default; `--no-infer-air-roll-from-handbrake` and `--no-persist-past-air-controls` restore the previous behavior (verified: the two flags together reproduce the earlier default report exactly).

**1. Handbrake air roll.** When airborne with the replicated handbrake held, steer now drives RocketSim `roll` instead of `yaw`. Evidence (five train replays, 13,426 airborne samples with an inferred pitch/roll control and |steer| >= 0.3): with handbrake held (1,683 samples) steer correlates with the span-inferred roll at r = 0.641 and yaw at 0.296; without handbrake (11,743) yaw 0.662 and roll 0.126. This matches RL's common "powerslide = air roll" binding. The correlation uses the offline inverse only to establish the direction of the mapping; the control itself uses observed steer and handbrake, which are available inside withheld windows. Alone it improves horizon-four train masked rotation p50/p90/p99 from 3.251/20.859/46.592 to 3.227/20.151/45.784 deg.

**2. Causal persistence.** `past_persisted_air_controls` solves the constant control (same forward-model solver) between the two most recent fresh car angular packets at or before the interval and keeps it for up to 0.15 s after the latest packet, whenever no later bracketing packet is available. It reads nothing after the interval; a unit test shows that adding a later packet leaves the controls at earlier frames unchanged. Persisted values are used only when the larger of |pitch| and |roll| is at least 0.5 (`--air-persist-min-control`), the same altitude guard applies, and dodge/flip intervals are excluded.

Paired train windows (10,052 horizon-four rotation targets, joined by replay hash, actor lifetime and window start; `python/analyze_persisted_air_controls.py`) with **ungated** persistence showed why a magnitude gate is needed. Delta = persistence minus default rotation error at horizon four:

| Persisted max(abs pitch, abs roll) | Windows | Better by >1 deg | Worse by >1 deg | Median / mean delta (deg) |
| --- | ---: | ---: | ---: | --- |
| none | 6,664 | 13 | 11 | 0.000 / 0.000 |
| below 0.3 | 703 | 71 | 202 | +0.150 / +0.611 |
| 0.3 to 0.7 | 1,364 | 676 | 305 | -0.933 / -2.584 |
| 0.7 and above | 1,321 | 755 | 297 | -3.352 / -5.710 |

Prior angular speed 5.48 rad/s or higher (the apparent cap) gained most (median -1.757 deg, mean -5.715, 989 better versus 187 worse); 5 to 5.48 rad/s was net harmful (+0.899 mean). Small fitted controls are noise; large ones are sustained inputs. Threshold sweep on **train** (horizon-four rotation p50/p90/p99 deg; roll routing on): no gate 3.305/15.432/39.233, 0.2 3.278/15.296/39.236, 0.3 3.251/15.322/39.236, **0.5 3.189/15.372/39.268**, 0.7 3.185/17.202/42.064. Gain 0.5 without a gate was worse at p50 (3.327) than gain 1 (3.299), and a 0.08 s expiry was similar to 0.15 s, so neither was adopted. The 0.5 threshold was chosen on train before validation was run.

Masked car errors, RocketSim p50 / p90 / p99, previous default (roll routing and persistence off) versus new default. Samples are identical (train 10,120/9,931/9,877/10,053 rotation targets at horizons 1-4; validation 10,405/10,623/10,479/10,401):

| Split / horizon | Rotation deg: previous | Rotation deg: new | Angular rad/s: previous | Angular rad/s: new |
| --- | --- | --- | --- | --- |
| train h1 | 1.840/8.994/22.616 | 1.810/7.944/19.691 | 0.415/2.543/5.835 | 0.355/2.260/5.307 |
| train h2 | 1.872/9.482/23.586 | 1.813/8.012/20.286 | 0.456/2.597/5.947 | 0.408/2.283/5.385 |
| train h3 | 2.659/15.985/39.032 | 2.595/11.833/34.193 | 0.565/3.194/6.391 | 0.528/2.683/6.008 |
| train h4 | 3.251/20.859/46.592 | **3.189/15.372/39.268** | 0.580/3.467/6.382 | 0.579/2.972/6.215 |
| validation h1 | 1.861/8.855/22.029 | 1.832/7.744/19.150 | 0.391/2.517/5.848 | 0.337/2.242/5.462 |
| validation h2 | 1.850/9.339/23.352 | 1.791/7.830/21.645 | 0.436/2.677/5.941 | 0.392/2.366/5.503 |
| validation h3 | 2.475/15.155/38.429 | 2.419/11.264/32.836 | 0.516/3.131/6.209 | 0.487/2.661/5.873 |
| validation h4 | 3.078/20.512/47.126 | **3.013/15.631/41.734** | 0.579/3.511/6.396 | 0.561/3.044/6.174 |

Position is unchanged to within 0.15 UU at every quantile (validation h4 15.967/39.609/76.588 to 15.952/39.587/76.639 UU). The independent mask schedule (`--mask-seed 239847`) agrees: validation h4 rotation 3.130/20.500/47.885 to 3.046/15.156/42.351 deg and angular p50/p90 0.587/3.487 to 0.574/2.960; train h4 3.324/21.430/46.687 to 3.251/15.917/41.289 deg. Horizon-four rotation by validation game size (p50/p90/p99): 1v1 4.142/25.089/47.902 to 3.938/19.773/44.102, 2v2 3.028/20.812/48.140 to 2.924/16.235/42.626, 3v3 2.899/18.923/45.813 to 2.850/13.962/39.851; train sizes improve likewise (1v1 4.661/28.164/52.239 to 4.530/21.202/42.746). Per replay at horizon four, rotation **p90 improved in 60/60 train and 60/60 validation replays**. Rotation **p50 is mixed per replay**: improved/worse/tied 33/19/8 on train (largest regression 1.078 deg) and 36/16/8 on validation (0.342 deg) even though the pooled p50 improves. Angular p50 was 26/29/5 (train) and 37/18/5 (validation), p90 58/1/1 and 53/4/3 (largest validation regression 1.034 rad/s). Unmasked one-step car error changed slightly and positively (validation rotation 1.721/7.545/19.833 to 1.703/7.382/19.356 deg; linear velocity p99 571.3 to 572.3 UU/s, effectively noise).

This is the first low-air candidate that improves p50, p90 and p99 pooled at every horizon and both mask schedules, unlike the earlier holds and feedback controls. Limits: the masked protocol still supplies observed steer, throttle, handbrake and boost inputs at withheld frames (only physics fields are withheld), persistence assumes a player keeps an input for at most about 0.15 s, per-replay medians are not uniformly better, and the fitted pitch/roll remain RocketSim-equivalent estimates rather than recovered player inputs. `replays/test` remains sealed. Reproduce with `cargo run --release --bin evaluate_corpus -- replays/<split> target/<split>-final.json` (and the two `--no-*` flags for the previous default); for the window table run the ungated variant (`--persist-past-air-controls --air-persist-min-control 0`) and the default with `--rotation-trace` for each, then `python python/analyze_persisted_air_controls.py BASE.jsonl PERSIST.jsonl`.

## Gap-spanning aerial inverse (2026-09-29)

**Finding.** The offline aerial-control inverse used only the *next replay frame*, but car rigid-body packets usually arrive every 2-3 frames (see the raw-cadence audit). Most airborne intervals therefore kept zero pitch/roll while RocketSim's damping decayed spin the player was actually sustaining, which explains much of the long-standing full-match airborne angular deficit against hold. `span_lookahead_air_controls` now finds the fresh car angular packets that bracket each interval (same actor lifetime and player link, active phase, fresh position above the altitude guard at both ends, no odd dodge counter inside the span) and applies one constant control solved over the whole span to every interval inside it. Defaults are now spans up to 4 frames / 0.15 s, one forward-model refinement pass, and the 50-100 UU band included (`--air-lookahead-frames`, `--air-lookahead-seconds`, `--air-lookahead-refine`, `--no-infer-transition-air-lookahead`; `--air-lookahead-frames 1 --air-lookahead-seconds 0.05 --air-lookahead-refine 0 --no-infer-transition-air-lookahead` reproduces the previous behavior; the one-frame configuration was verified to give an identical train report).

The refinement replays RocketSim's per-tick air torque and damping (`air_angular_velocity_forward`, which mirrors `update_air_torque` in the pinned `rocketsim` source: torque only when a control is nonzero, `(1-|input|)` damping scaling for pitch and yaw, unscaled roll damping, 5.5 rad/s cap) and corrects the analytic inverse, because the analytic solve freezes damping at the span start. A unit test checks the round trip.

**This is offline reconstruction, not causal prediction.** It uses a packet from after the interval. The evaluator therefore hands the masked conversion a `withheld_frames` barrier; a span that contains any withheld frame is refused, and a unit test covers it. Masked four-frame results are consequently unchanged (below). The controls are RocketSim-equivalent estimates, not recovered player inputs.

**Which numbers are independent.** The inverse is fitted so that simulated angular velocity at the span end matches the observed packet, so the one-step *angular* error at those packets is partly **in-sample**. The inverse uses only the start orientation and the two angular velocities, so end **rotation**, position and velocity are *not* fitted. The rotation gains are the evidence.

One-step pre-correction car error, all fresh packets, RocketSim p50 / p90 / p99 (train 60/60 and validation 60/60 replays converted, zero failures; `replays/test` untouched):

| Split | Setting | Rotation deg | Angular rad/s (in-sample) | Air angular | 50-100 UU angular |
| --- | --- | --- | --- | --- | --- |
| train | previous default (1-frame) | 1.885 / 8.864 / 22.064 | 0.372 / 2.552 / 5.782 | 0.891 / 2.994 / 6.560 | 2.013 / 4.616 / 6.638 |
| train | span 4 + refine + 50 UU (new default) | **1.751 / 7.514 / 19.723** | 0.160 / 2.170 / 5.379 | 0.106 / 2.410 / 6.088 | 1.165 / 4.050 / 6.102 |
| validation | previous default | 1.855 / 8.750 / 21.954 | 0.366 / 2.528 / 5.782 | 0.887 / 2.979 / 6.495 | 1.999 / 4.661 / 6.785 |
| validation | new default | **1.721 / 7.545 / 19.833** | 0.169 / 2.186 / 5.394 | 0.126 / 2.436 / 6.035 | 1.297 / 4.154 / 6.255 |

Hold baselines are 8.03 / 28.34 / 39.47 deg rotation (train), and on train air angular 0.826 / 2.449 / 6.099 and 50-100 UU angular 0.688 / 2.789 / 5.679 rad/s. Sample counts are identical across settings (998,832 train and 1,056,577 validation rotation samples; 298,706 / 314,236 air; 142,390 / 148,464 transition). Car linear-velocity one-step quantiles moved by under 0.2%.

Train span sweep (rotation p50/p90; refine 0 and 50 UU guard off unless stated): 1 frame 1.885/8.864, 2 frames 1.825/8.372, 3 frames 1.786/8.109; 4 frames + 50 UU guard + refine 1: 1.751/7.514; 6 frames (0.22 s) + guard: 1.747/7.501, a negligible extra gain, so 4 was kept. The 50 UU guard alone at 4 frames lowers validation rotation p90 from 8.063 to 7.545 (rotation p50 1.749 to 1.721). Refinement mainly reduces the in-sample angular fit (train air p50 0.221 to 0.106) and only marginally the rotation (1.755 to 1.751); one versus three passes made no material difference.

Per-replay behavior (new default vs previous, one-step car rotation): p50 improved/worse/tied in 60/0/0 train and 59/0/1 validation replays; p90 60/0/0 and 59/0/1. Angular p50 and p90 improved in every replay in both splits. Per game size (p50/p90/p99 deg): train 1v1 2.442/11.63/24.93 to 2.204/9.284/22.85, 2v2 1.806/8.921/22.50 to 1.673/7.627/20.31, 3v3 1.790/8.072/20.56 to 1.666/6.914/18.18; validation 1v1 2.359/11.55/24.79 to 2.152/9.518/23.07, 2v2 1.847/8.978/22.58 to 1.709/7.657/20.46, 3v3 1.746/7.957/20.24 to 1.615/6.899/18.07. Velocity per-replay medians moved in both directions by at most 1.5 UU/s (p90), noise relative to 100+ UU/s errors. The 50 UU guard was chosen after the train sweep and its per-replay rotation effect checked on both splits (p50 improved in 60/0/0 train and 59/0/1 validation replays).

**Masked (causal) prediction is unchanged**, as intended: horizon-four car rotation p50/p90/p99 remains 3.251/20.859/46.592 deg on train and 3.078/20.512/47.126 on validation (validation angular p90 3.521 to 3.511 rad/s), so this change does not improve forward prediction across withheld intervals. Remaining limits: the 50-100 UU one-step angular p50 (1.17 train, 1.30 validation) is still worse than hold (0.69 / 0.67), dominated by recent dodge/flip intervals that the inverse deliberately excludes; spans assume constant input; and pitch/roll cannot be verified against real player inputs.

Reproduce (ignored outputs under `target/`): `cargo run --release --bin evaluate_corpus -- replays/<split> target/<split>-span-default.json`, and the previous behavior with the four flags above. `cargo test --all-targets` and the Python tests pass; a validation 1v1 replay exported to Parquet restored 8,126/8,126 snapshots.

## Prior-speed low-air hold gate (2026-09-28)

The frame-level train traces below identified two regimes within 50–100 UU: low-air angular speed below the replay's apparent 5.5 rad/s ceiling often decays or responds to known steering, while near-ceiling speed often persists across masked packets. A paired train analysis joined each horizon-four fresh-rotation target to its horizon-one pre-mask row using replay SHA-256, actor ID, actor lifetime, and mask-window start. It matched 10,052 scored windows; one of the 10,053 aggregate rotation samples had no matching horizon-one car row and was excluded from the feature analysis. Among matched low-air windows with prior observed angular speed in [4, 5) rad/s, angular hold made rotation worse by over 1° in 46 and better by over 1° in 11. At [5.48, 5.51) rad/s, it made rotation better by over 1° in 660 and worse by over 1° in 45. These are retrospective **train** labels; the fresh target rotation was used only to score outcomes, never as a gate input.

`--gated-low-air-angular` now tests the old low-air hold only when the most recent angular packet available before the simulated interval has magnitude at least **5.48 rad/s** (replay raw angular units ×0.01). The old altitude, freshness, simulated-contact, and jump/dodge gates still apply. This is an off-by-default ablation. The threshold came from inspecting train packets near the apparent 5.5 rad/s ceiling; it is an empirical regime marker, not proof of player input or engine physics. The hold still overwrites angular velocity after RocketSim integrates the preceding interval's orientation.

An initial 5.0 rad/s gate was checked on validation, then revised using train packet sequences to 5.48. Validation was therefore used twice during this development step; the final test split remains sealed. At train replay `00a0da63`, the angular speed at the last pre-mask packet was 5.075 rad/s for actor 8 at frame 204 and 5.284 for actor 30 at frame 1704; both had fallen from 5.5 in earlier packets. The gate now leaves their default errors at 1.12° and 2.02° instead of the ungated hold's 8.05° and 18.53°. Actor 149 at frame 11404 remained at 5.5 rad/s and retains the hold's improvement from 28.19° to 11.88°. Counterexample: `00b3d382` actor 137 at frame 6704 also had three prior packets at 5.5 rad/s, but zero recorded steer and subsequently falling angular velocity; the gate still applies the harmful hold (default 0.56°, hold 12.83°). Prior packet saturation does not reveal whether an unobserved pitch/roll input was released at the boundary.

Four-frame masked car errors on matched samples (p50 / p90 / p99):

| Split | Setting | Rotation ° | Angular rad/s | Position UU |
| --- | --- | --- | --- | --- |
| train | default | 3.251 / 20.859 / 46.592 | 0.579 / 3.468 / 6.382 | 16.776 / 40.259 / 75.562 |
| train | ungated hold | 3.306 / 18.913 / 41.514 | 0.580 / 3.196 / 6.207 | 16.776 / 40.269 / 75.654 |
| train | speed gated | 3.217 / 18.868 / 41.639 | 0.558 / 3.196 / 6.207 | 16.776 / 40.259 / 75.562 |
| validation | default | 3.078 / 20.512 / 47.163 | 0.579 / 3.521 / 6.399 | 15.967 / 39.609 / 76.639 |
| validation | ungated hold | 3.129 / 18.741 / 42.880 | 0.568 / 3.233 / 6.188 | 15.973 / 39.594 / 76.668 |
| validation | speed gated | 3.056 / 18.760 / 42.923 | 0.548 / 3.245 / 6.190 | 15.973 / 39.607 / 76.668 |

All three settings converted 60/60 train and 60/60 validation replays with zero failures and identical replay SHA-256 sets and sample counts. Horizon four contains 10,053 train and 10,401 validation rotation/position samples, and 10,049/10,398 angular samples. The gated option improved validation rotation p50 and p90 in each game size: 1v1 4.142/25.239→4.101/22.552°, 2v2 3.028/20.812→3.007/19.051°, and 3v3 2.899/18.923→2.866/17.305°. Its horizon-four replay rotation p50 improved/worsened/tied in 27/7/26 train and 25/6/29 validation replays; p90 improved in 56/60 train and 54/60 validation with no regressions. The largest replay-median rotation regression was 0.641° on train and 0.287° on validation. Validation angular p50 improved/worsened/tied in 41/7/12 replays; angular p90 improved/worsened/tied in 52/1/7. The option also improved pooled one-step validation rotation p50/p90/p99 from 1.855/8.750/21.954° to 1.845/8.282/20.171°.

The gate is promising but remains off by default because it still fails a known train decay window, regresses some replay medians, and leaves a post-step orientation/angular-velocity mismatch. Next work should infer whether an orientation-consistent control or prior-trend model can capture sustained spin without that mismatch. Reproduce the reports with `cargo run --release --bin evaluate_corpus -- replays/train target/train-low-air-cap-gated.json --gated-low-air-angular` and the same command with `replays/validation`; omit the flag for baseline and use `--hold-low-air-angular` for ungated hold. For the paired train feature analysis, run the baseline and ungated hold evaluator commands with distinct `--rotation-trace target/train-base-trace.jsonl` and `--rotation-trace target/train-hold-trace.jsonl` paths, then `python python/analyze_low_air_regimes.py target/train-base-trace.jsonl target/train-hold-trace.jsonl`. Trace rows include up to three fresh angular packets strictly before each mask window. All generated files stay ignored under `target/`.

## Masked rotation frame inspection (train only, 2026-09-28)

The evaluator now accepts one `.replay` or a split directory and optionally writes `--rotation-trace <path.jsonl>`. It emits one row per masked primary-linked car frame from the evaluator's existing four-frame schedule. Rows include both the fresh original body and the body supplied to the converter after masking, each field's source frame, original controls at the target and preceding frame, the previous and current simulated car states, and simulator events for the target interval. `previous_simulated.controls_for_next_interval` drove the **preceding** interval; `predicted.controls_for_next_interval` is the control selected for the **next** interval. Event labels beginning `sim_` are RocketSim estimates, not replay-authored contacts. `hold_rotation_error_degrees` is the evaluator's hold-last-observed-rotation baseline; the low-air angular-hold ablation requires its own run. Rotation error fields are null when the target has no fresh eligible rotation packet.

I inspected three train replays frame by frame under the default converter and the off-by-default `--hold-low-air-angular` ablation; the first was also inspected under `--feedback-low-air-angular`. Trace rotation-error keys and existing metric sample counts matched exactly: replay `00a0da63-492e-4ab7-8a07-16cd5d14dcb4` had 1,080 trace rows and 354 eligible rotation samples, `00b3d382-c4f8-403c-a166-5a7afcd1dd90` had 1,034/342, and `00a02f8f-83f1-4307-9005-3948a9153c0a` had 928/382. A final rerun of the first replay also matched all 353 eligible angular samples. Baseline and hold had identical sample keys in all three; no traced actor-lifetime mismatch was found. These are selected diagnostic windows, not a corpus-level performance estimate.

| Train replay / masked actor frames | Evidence in the packet and simulated interval | Default at last frame: rotation / angular error | Angular hold at last frame: rotation / angular error |
| --- | --- | ---: | ---: |
| `00a0da63` actor 8, frames 201–204 | Prior rotation packet at 200; replay steer near +0.79, yaw control +0.79, no simulated contact; height 68 UU | 1.12° / 0.29 rad/s | 8.05° / 2.70 rad/s |
| `00b3d382` actor 137, frames 6701–6704 | Prior angular packet at 6699; all controls zero, no simulated contact; observed angular Y falls from 5.28 to 2.41 rad/s by frame 6704 | 0.56° / 0.20 rad/s | 12.83° / 2.12 rad/s |
| `00a0da63` actor 149, frames 11401–11404 | Prior packet at 11398; replay steer about −0.83 to −0.93, no simulated contact; observed angular X remains high at 4.50 rad/s at frame 11404 | 28.19° / 3.68 rad/s | 11.88° / 1.59 rad/s |

The first window briefly favors hold at frame 202 (rotation 1.20° versus default 1.82°), then strongly favors default by frame 204. Feedback control also failed there: its inferred pitch −0.918 and clipped roll +1.0 preceded frame 204, which ended at 6.86° rotation and 2.09 rad/s angular error. A second known-steer no-contact interval in the same replay, actor 30 at frame 1704, ended at default 2.02° versus hold 18.53° and feedback 15.32°. The second table row isolates natural angular decay with zero controls; RocketSim tracks its observed decline. The third is a counterexample: default damps an angular component the fresh replay packet shows staying high. Missing pitch/roll or an unobserved impulse is plausible, but neither can be inferred uniquely from these traces. The traces also do not establish real collision absence because their contact labels come from the simulator. These mixed regimes explain why a blanket hold can improve tails while damaging typical masked rotation. Keep both low-air options disabled until a discriminator using only information available before the target packet improves matched validation behavior.

Reproduce a focused trace from the repository root with `cargo run --release --bin evaluate_corpus -- replays/train/1v1/00a0da63-492e-4ab7-8a07-16cd5d14dcb4.replay target/trace-00a0da63-report.json --rotation-trace target/trace-00a0da63.jsonl`. Add `--hold-low-air-angular` or `--feedback-low-air-angular` and use distinct output paths to compare the same masked frames. Both outputs are ignored under `target/`. Inspect `frame`, `actor_id`, `horizon`, field source frames, previous interval controls, and errors together; pooling alone obscures the reversal within a four-frame window.

## Low-air control and packet-time follow-up (2026-09-28)

The previous low-air angular hold changes angular velocity **after** RocketSim integrates orientation for that interval. That creates an inconsistent immediate state and is a plausible contributor to its masked rotation-median regression. On train, its four-frame masked rotation p50/p90/p99 changed 3.251/20.859/46.592→3.306/18.913/41.515 degrees, with p50 worse in 33/60 replays. Angular p50/p90/p99 changed 0.579/3.468/6.382→0.580/3.196/6.207 rad/s. The p50 rotation regression occurred in every game size; train examples `00a0da63` and `00b3d382` had +1.109° and +1.002° replay-median regression despite p90 gains. The original hold remains disabled.

`diagnose_air_rotation --low-air` now applies its fresh adjacent-packet, unchanged actor, active-phase, and no-fresh-dodge filters to cars whose **both** observed heights are 50–100 UU. It processed 60/60 train and validation replays, retaining 20,737/21,216 angular pairs. For nominal four-tick gaps, the replay orientation change projected onto mean angular velocity has p50 scale 0.637/0.610 on train/validation; the corresponding translation scale has p50 0.500/0.500. On the subset with a usable target position and velocity at both ends, integrating mean angular velocity over the full replay-frame delta yields rotation p50/p90 3.721°/10.743° on 19,424 train pairs and 3.826°/10.999° on 19,455 validation pairs. Replacing that duration with the **target-position-derived** interval yields 0.826°/4.040° and 0.727°/4.018° respectively. This is a cross-field offline consistency check: the target position and velocity choose the interval, so it is not a masked prediction or an observed packet timestamp. The low-air scales vary widely (four-tick orientation-scale p90 2.072/2.102), making a uniform half-time correction unsound.

To test a consistent simulator update, `--feedback-low-air-angular` is an off-by-default control ablation. It never overwrites angular velocity after a step. At an active low-air frame with a same-lifetime previous car, a position and angular observation no older than 0.15 s, airborne contact-free RocketSim state, no recent odd jump/dodge component packet, and no simulated car collision in the interval, it compares simulated pitch/roll angular velocity with the last available replay angular packet. Only when the pitch/roll difference exceeds 0.75 rad/s does it solve bounded RocketSim pitch/roll controls to approach that packet over a fixed four-tick response interval; it preserves the existing steer-to-yaw rule. Current packets are used only to set controls for *subsequent* ticks. The 0.75 threshold and four-tick response were fixed before validation. Other default controls and the 120 Hz timeline are unchanged.

| Split | Horizon-four field | Matched samples | Default p50 / p90 / p99 | Feedback p50 / p90 / p99 | Replay p50 better / worse / tied |
| --- | --- | ---: | ---: | ---: | ---: |
| train | Angular velocity (rad/s) | 10,049 | 0.579 / 3.468 / 6.382 | 0.601 / 3.393 / 6.394 | 10 / 36 / 14 |
| train | Rotation (degrees) | 10,053 | 3.251 / 20.859 / 46.592 | 3.293 / 20.237 / 46.428 | 17 / 29 / 14 |
| validation | Angular velocity (rad/s) | 10,398 | 0.579 / 3.521 / 6.399 | 0.598 / 3.437 / 6.384 | 10 / 34 / 16 |
| validation | Rotation (degrees) | 10,401 | 3.078 / 20.512 / 47.163 | 3.110 / 19.985 / 46.843 | 14 / 24 / 22 |

Validation horizon-four angular p50 rose in 1v1/2v2/3v3 from 0.893/0.577/0.519 to 0.916/0.598/0.532 rad/s; rotation p50 rose from 4.142/3.028/2.899 to 4.250/3.065/2.916 degrees. Rotation p90 improved in 37/60 validation replays, with none worse and 23 ties, but the median regressions fail the typical-plus-tail gate. Four-frame car-position p50/p90 stayed 15.967/39.609 UU; p99 moved 76.639→76.668 UU. All 60 replays converted successfully in each setting and split, with matched replay SHA-256 values and metric sample counts. The feedback option remains disabled. A physical control can improve an angular tail without matching the replay's apparent car motion interval; the packet-time interpretation remains a hypothesis.

Reproduce ignored reports with `cargo run --release --bin diagnose_air_rotation -- replays/train --low-air > target/train-low-air-motion.txt` and the analogous validation command; compare `cargo run --release --bin evaluate_corpus -- replays/train target/train-low-air-hold-control.json` with `cargo run --release --bin evaluate_corpus -- replays/train target/train-low-air-feedback.json --feedback-low-air-angular`, then substitute `validation` for the fixed check. Do not use the target-position-derived interval as a causal prediction score. `replays/test` remains sealed.

## Low-air contact, cadence, and angular-hold follow-up (2026-09-28)

The replay pad-pickup label requires an odd, non-255 `picked_up` counter and the matching instigator actor; `255` availability updates are excluded.

`diagnose_low_air` groups the existing fresh, one-step, pre-correction car angular-velocity pairs when the **target** altitude is 50–100 UU. It compares RocketSim to holding the previous fresh angular packet, with replay angular velocity scaled by 0.01. Every context is a marginal label over the same samples, so groups overlap. Packet gaps are raw replay frame and time gaps between fresh angular fields, **not** inferred simulator ticks. Prior geometry uses the previous synchronized RocketSim snapshot; `recent_sim_*` covers the preceding 0.15 s including the target interval and is an offline diagnostic. Replay pad pickups and jump/dodge component packets are labeled separately. No replay-authored contact field exists. The report includes per-replay quantiles, hashes, and the 100 worst angular regrets. `persistent_low_air` means prior carried position and current target are in the band, the predicted car is airborne, and no odd jump/dodge component packet occurred in the preceding 0.15 s; this is descriptive rather than contact proof.

Train and validation each converted 60/60 with zero failures. The same 142,390 train and 148,464 validation angular pairs as the existing altitude diagnostic were retained. Train RocketSim p50/p90/p99 was 2.013/4.616/6.638 rad/s versus hold 0.688/2.789/5.679; validation was 1.999/4.661/6.785 versus hold 0.671/2.804/5.650. RocketSim median exceeded hold in **60/60 replays in each split**. Persistent low air accounted for 97,873 train and 101,851 validation pairs. In validation that group had RocketSim p50/p90 2.063/4.641 versus hold 0.453/1.656. Thus the typical discrepancy is broad and persists away from a fresh takeoff signal. The diagnostic does not establish whether missing pitch/roll input, aerodynamics, packet timing, or an unreported contact dominates.

Contact and proximity enrich the tail but do not explain the majority of samples. On validation, previous RocketSim world contact appeared in 2,502/148,464 pairs (sim p90/p99 6.026/8.507 versus 4.641/6.670 without), and a recent simulated car-world hit appeared in 12,473 pairs (5.436/8.224 versus 4.581/6.506). A previous simulated ball distance below 350 UU appeared in 10,524 pairs (sim p99 12.401 versus 6.487 farther away). Previous simulated pad proximity below 250 UU appeared in 19,968 pairs, but its p99 was 6.484 versus 6.833 farther away. These are correlated simulation labels, not attributed replay collisions or causal effects.

An experimental `--hold-low-air-angular` converter option carries the last fresh angular packet across a low-air, airborne interval only when the prior observed and current predicted heights are both 50–100 UU, both prior position and angular fields are at most 0.15 s old, the predicted car has no wheel/world contact, no car contact event occurs in the simulated interval, and no fresh odd jump/dodge packet occurs in the prior 0.15 s. It reads no target rigid-body field; it changes the post-step angular state while leaving that interval's orientation as simulated. It is **off by default**. The intended test was whether a causal hold could improve both typical and tail masked rotation/angular errors without material per-replay regressions.

On validation, one-step transition angular p50/p90/p99 changed from 1.999/4.661/6.785 to 0.673/3.132/6.093 rad/s. Four-frame masked angular p50/p90/p99 changed from 0.579/3.521/6.399 to 0.568/3.233/6.188; rotation changed from 3.078/20.512/47.163 to **3.129/18.741/42.880** degrees. Masked rotation p50 worsened at horizons 2–4 and in every game size at horizon four: 1v1 4.142→4.231, 2v2 3.028→3.077, 3v3 2.899→2.920 degrees. At horizon four, replay rotation p50 improved/worsened/tied in 13/33/14 replays; p90 improved/worsened/tied in 54/0/6. Angular p50 improved/worsened/tied in 27/25/8; p90 in 53/2/5. One-step rotation p50 worsened in 40/60 replays. Four-frame masked car position p50 shifted 15.967→15.973 UU and p99 76.639→76.668 UU. The 60 validation replay SHA-256 values and sample counts matched between baseline and ablation; both converted 60/60 with no failures. On train, four-frame masked angular p50 changed 0.5793→0.5796 and rotation p50 3.251→3.306 degrees, despite better p90 tails. The candidate fails the typical-plus-tail gate and remains disabled. The post-step hold also creates a temporary mismatch between angular velocity and the orientation just integrated, which is a plausible source of rotation regression, not a proven cause.

Reproduce ignored reports with `cargo run --release --bin diagnose_low_air -- replays/train target/train-low-air-context.json` (replace `train` with `validation` for the frozen check), and `cargo run --release --bin evaluate_corpus -- replays/validation target/validation-low-air-hold-control.json` versus the same command with `target/validation-low-air-hold-ablation.json --hold-low-air-angular`. The evaluator now includes masked kinematics by horizon in each per-replay record. Train ablation: `cargo run --release --bin evaluate_corpus -- replays/train target/train-low-air-hold-ablation.json --hold-low-air-angular`. All reports remain ignored under `target/`. The `test` split remains sealed.

The early sections record historical experiments with actor-lifetime, boost, demolition, hitbox, jump, and dodge inference. The later reviewed sections use corrected pad handling, guarded aerial lookahead, and field-specific kinematic residuals. Historical ablation reports remain under `target/*-conversion-metrics*.json`; their figures should be read with the implementation described beside them.

## Protocol

`cargo run --release --bin evaluate_corpus -- replays/train target/train-conversion-metrics.json` measures the position immediately before a fresh replay packet corrects it. When multiple car actors share a player key, only the selected primary car is evaluated. The evaluator also hides **all ball and car rigid-body fields, as well as fresh car boost amounts,** at frame offsets 1–4 in each 100-frame block, runs conversion again, and compares the resulting positions, kinematics, and boost amounts with fresh original replay positions. Replay boost activation evidence (`boost_active_raw`) remains available to the simulator during masked frames so boost consumption can be simulated. Only active-phase comparisons with a last observed value at most 0.5 seconds old are counted. Hold-last-value baselines share the same last unmasked observation. All position values below are pooled absolute position errors in Rocket League unreal units (UU). This test measures short-horizon prediction at network frames; it does not verify every RocketSim field or long unobserved intervals. An independent replay-specific mask schedule confirmed the direction of the car-position gains below.

## Four-frame masked prediction

Each cell is median / p90 error in UU at mask horizon 4. The simulated and linear columns use the same observed target positions. Replays converted: 60/60 in each split; failures: zero.

| Split | Game size | Body | RocketSim | Linear | Samples |
| --- | --- | --- | ---: | ---: | ---: |
| train | 1v1 | ball | 12.2 / 40.5 | 14.4 / 51.9 | 1,635 |
| train | 1v1 | car | 17.5 / 44.3 | 30.1 / 69.6 | 1,535 |
| train | 2v2 | ball | 12.3 / 38.7 | 13.2 / 51.3 | 1,532 |
| train | 2v2 | car | 15.7 / 38.7 | 26.7 / 63.0 | 2,853 |
| train | 3v3 | ball | 14.6 / 41.7 | 16.5 / 57.9 | 1,930 |
| train | 3v3 | car | 17.1 / 40.4 | 30.3 / 64.3 | 5,665 |
| validation | 1v1 | ball | 10.6 / 38.1 | 12.1 / 46.4 | 1,720 |
| validation | 1v1 | car | 14.8 / 41.6 | 25.5 / 66.8 | 1,592 |
| validation | 2v2 | ball | 11.7 / 37.0 | 13.1 / 45.6 | 1,733 |
| validation | 2v2 | car | 15.3 / 38.3 | 26.5 / 63.1 | 3,203 |
| validation | 3v3 | ball | 14.2 / 41.7 | 15.3 / 56.0 | 1,945 |
| validation | 3v3 | car | 16.5 / 41.2 | 28.5 / 65.6 | 5,606 |

## One-step pre-correction prediction on train

| Game size | Body | RocketSim p50 / p90 / p99 | Linear p50 / p90 / p99 | Fresh positions |
| --- | --- | ---: | ---: | ---: |
| 1v1 | ball | 9.38 / 34.13 / 64.37 | 9.38 / 35.13 / 67.95 | 164,616 |
| 1v1 | car | 17.85 / 42.29 / 69.33 | 19.06 / 45.16 / 75.59 | 158,055 |
| 2v2 | ball | 11.20 / 33.72 / 63.56 | 10.87 / 34.77 / 69.13 | 151,225 |
| 2v2 | car | 15.99 / 41.14 / 62.79 | 17.36 / 43.78 / 69.68 | 285,343 |
| 3v3 | ball | 11.41 / 36.23 / 66.10 | 11.14 / 37.84 / 74.45 | 189,151 |
| 3v3 | car | 17.19 / 42.33 / 69.30 | 18.69 / 44.66 / 72.23 | 555,544 |

One-step simulated car p99 errors are now below linear extrapolation in all three game sizes, and gated dodge inference further reduced one-step car error across all game sizes and quantiles (overall train car p50/p90/p99 improved from 17.01/42.23/68.37 UU to 16.96/41.99/68.01 UU). The evaluator's top training car error over linear extrapolation fell from about 10,724 UU before demolition correction to 932 UU after selecting primary cars; the metric also excludes retired duplicate actors. Across train and validation, 32,751 overlapping or otherwise shadowed car-frame records were skipped, and active replay pawn evidence corrected 592 simulated demolition flags. Validation one-step car p99 fell from 76.68/73.38/76.39 UU to 66.60/61.89/72.19 UU across 1v1/2v2/3v3. Collision, kickoff, hitbox, and unknown input cases still need investigation. The masked results do not establish accurate pad-synchronization, event, or scoreboard reconstruction.

## Boost activation check

On `train`, 2,818 of 2,824 short observed boost-amount intervals ending in an odd-to-even boost activation counter transition showed boost depletion. On `validation`, 3,011 of 3,014 did. This supports interpreting odd counter values as active boost input. At the time of that ablation, enabling the signal improved four-frame validation car median/p90 from 16.0/46.8 to 15.4/45.7 UU in 1v1, 16.1/41.3 to 16.0/40.0 in 2v2, and 17.7/47.0 to 17.4/46.8 in 3v3. The initial small one-step p99 regression was overtaken by the later actor fixes above. The counter interpretation is an inference, not a direct action field.

## Four-frame masked boost prediction

The evaluator withholds fresh car boost amounts during the same four-frame rigid-body mask intervals, while leaving replay boost activation evidence (`boost_active_raw`) available to the simulator. RocketSim simulates continuous boost depletion during active boosting and boost-pad pickups if crossed. Predicted boost amounts (0–100 scale) are compared against fresh original replay boost values and a hold-last-observed-boost baseline. As with kinematics, samples require primary linked cars, active game state, and a last observation gap $\le 0.5$ seconds. All 60 train and 60 validation replays converted without failure.

### Boost error by mask horizon (all game sizes pooled)

Each entry is sample count and absolute boost error quantiles (p50 / p90 / p99) on the 0–100 boost scale.

| Split | Horizon | Samples | RocketSim p50 / p90 / p99 | Hold p50 / p90 / p99 |
| --- | ---: | ---: | ---: | ---: |
| train | 1 | 220 | 0.59 / 12.16 / 88.95 | 7.84 / 14.12 / 95.69 |
| train | 2 | 209 | 0.92 / 12.48 / 100.00 | 8.63 / 16.47 / 100.00 |
| train | 3 | 214 | 0.52 / 12.16 / 96.08 | 7.84 / 17.65 / 100.00 |
| train | 4 | 202 | 0.65 / 12.75 / 97.65 | 8.24 / 22.75 / 100.00 |
| validation | 1 | 251 | 0.72 / 12.65 / 100.00 | 6.67 / 32.55 / 100.00 |
| validation | 2 | 211 | 0.49 / 12.21 / 100.00 | 8.24 / 20.39 / 100.00 |
| validation | 3 | 253 | 0.85 / 12.49 / 100.00 | 9.02 / 28.63 / 100.00 |
| validation | 4 | 197 | 0.47 / 12.21 / 100.00 | 7.45 / 15.69 / 100.00 |

### Boost error by game size at horizon 4

| Split | Game size | Samples | RocketSim p50 / p90 / p99 | Hold p50 / p90 / p99 |
| --- | --- | ---: | ---: | ---: |
| train | 1v1 | 39 | 0.57 / 17.25 / 97.65 | 11.37 / 17.25 / 97.65 |
| train | 2v2 | 47 | 0.64 / 12.46 / 76.86 | 8.24 / 12.16 / 92.55 |
| train | 3v3 | 116 | 0.76 / 12.26 / 100.00 | 7.45 / 44.31 / 100.00 |
| validation | 1v1 | 36 | 1.06 / 13.18 / 100.00 | 9.02 / 74.51 / 100.00 |
| validation | 2v2 | 71 | 0.33 / 12.16 / 100.00 | 6.67 / 12.94 / 100.00 |
| validation | 3v3 | 90 | 0.49 / 12.21 / 85.10 | 7.06 / 16.86 / 100.00 |

### Alternate mask schedule (`--mask-seed 239847`) on validation

An independent check using the deterministic pseudo-random offset schedule confirms the boost metrics:

| Horizon | Samples | RocketSim p50 / p90 / p99 | Hold p50 / p90 / p99 |
| ---: | ---: | ---: | ---: |
| 1 | 191 | 0.46 / 12.16 / 27.83 | 8.63 / 17.25 / 100.00 |
| 2 | 223 | 0.49 / 12.16 / 100.00 | 7.45 / 14.51 / 100.00 |
| 3 | 223 | 0.47 / 12.16 / 100.00 | 7.84 / 17.65 / 100.00 |
| 4 | 189 | 0.64 / 12.16 / 83.92 | 8.63 / 15.29 / 100.00 |

At horizon 4 by game size with `--mask-seed 239847`: 1v1 (43 samples) RocketSim 0.64 / 12.16 / 100.00 vs Hold 8.63 / 33.33 / 100.00; 2v2 (58 samples) RocketSim 0.59 / 12.16 / 44.90 vs Hold 7.06 / 12.55 / 41.57; 3v3 (88 samples) RocketSim 0.64 / 12.16 / 21.45 vs Hold 10.20 / 15.29 / 100.00.

### Boost findings and limitations

1. **Continuous depletion tracking (p50):** RocketSim median boost error is under 1.0 boost unit across all horizons and game sizes (0.33 to 1.06 boost units, representing $\le 1\%$ of total boost capacity). In contrast, the hold-last-observed baseline median error is 6.3 to 11.4 boost units. This confirms that simulating boost depletion from inferred active boost input (`boost_active_raw`) closely matches ground-truth consumption.
2. **Small boost-pad pickups (p90):** Across nearly every horizon and game-size slice, RocketSim p90 error clusters tightly around **12.16 boost units**. A small boost pad provides 31 raw boost ticks ($31 \times 100 / 255 \approx 12.156863$ boost units), so missed or spurious pickups are a plausible contributor. The quantile alone cannot attribute individual errors; trajectory divergence, pad cooldown, and other events require event-level checks.
3. **Orb pickups and respawns (p99):** Extreme tail errors reach 80–100 boost units, corresponding to 100-boost orb pickups or kickoff/respawn refills that occurred during the withheld window.
4. **Limitations:** Only frames with fresh original boost packets are evaluated (~200–250 per horizon per split); replay boost updates are replicated at network rates rather than 120 Hz ticks. Simultaneous body masking also couples car position error to boost-pad collision detection.

## Four-frame masked kinematics

The same mask measures fresh linear velocity (UU/s), rotation angle (degrees), and angular velocity (radians/s). Each field uses its own last unmasked observation and is counted only when that field is fresh in the original frame and its observation gap is at most 0.5 seconds in active play. The comparison baseline holds that field's last value. Replay angular velocity is scaled by 0.01 before comparison. The primary-car filter applies; sample counts can differ because replay rigid-body velocity fields are optional.

Validation results at mask horizon 4 with default motion-gated dodge flip inference are below. Each value is median / p90 error. All 60 validation replays converted without failure.

| Size | Body | Field | RocketSim | Hold | Samples |
| --- | --- | --- | ---: | ---: | ---: |
| 1v1 | ball | Velocity (UU/s) | 5.46 / 21.84 | 86.99 / 347.79 | 1,720 |
| 1v1 | ball | Rotation (degrees) | 2.86 / 5.77 | 40.11 / 51.57 | 1,720 |
| 1v1 | ball | Angular velocity (rad/s) | 0.00 / 0.07 | 0.00 / 1.14 | 1,720 |
| 1v1 | car | Velocity (UU/s) | 36.82 / 210.52 | 180.27 / 648.79 | 1,592 |
| 1v1 | car | Rotation (degrees) | 4.37 / 24.95 | 20.32 / 59.16 | 1,592 |
| 1v1 | car | Angular velocity (rad/s) | 0.97 / 4.01 | 1.28 / 4.19 | 1,592 |
| 2v2 | ball | Velocity (UU/s) | 5.44 / 21.65 | 86.79 / 527.24 | 1,733 |
| 2v2 | ball | Rotation (degrees) | 2.86 / 5.73 | 40.11 / 51.57 | 1,733 |
| 2v2 | ball | Angular velocity (rad/s) | 0.00 / 0.06 | 0.00 / 1.65 | 1,733 |
| 2v2 | car | Velocity (UU/s) | 32.25 / 171.74 | 198.85 / 628.28 | 3,201 |
| 2v2 | car | Rotation (degrees) | 3.16 / 20.55 | 17.53 / 53.98 | 3,203 |
| 2v2 | car | Angular velocity (rad/s) | 0.61 / 3.58 | 1.09 / 3.59 | 3,201 |
| 3v3 | ball | Velocity (UU/s) | 5.46 / 27.26 | 88.73 / 987.46 | 1,945 |
| 3v3 | ball | Rotation (degrees) | 2.86 / 6.63 | 43.31 / 51.57 | 1,945 |
| 3v3 | ball | Angular velocity (rad/s) | 0.00 / 0.07 | 0.00 / 2.70 | 1,945 |
| 3v3 | car | Velocity (UU/s) | 31.07 / 154.61 | 209.35 / 641.28 | 5,605 |
| 3v3 | car | Rotation (degrees) | 2.96 / 18.78 | 17.53 / 52.84 | 5,606 |
| 3v3 | car | Angular velocity (rad/s) | 0.54 / 3.34 | 1.09 / 3.52 | 5,605 |

With gated dodge inference, car linear velocity median dropped from 51.9 / 43.3 / 39.7 UU/s to 36.8 / 32.3 / 31.1 UU/s across 1v1/2v2/3v3 (a 22–29% median error reduction), and p90 dropped by 28–34 UU/s. Car rotation p90 dropped by 4.1–5.1 degrees (e.g. from 29.06 to 24.95 deg in 1v1, 25.68 to 20.55 deg in 2v2, and 22.94 to 18.78 deg in 3v3).

## Rotation calibration and independent mask

`cargo run --release --bin calibrate_rotation -- replays/train` compared fresh car angular-velocity packets against the quaternion change over short active-play intervals. On 367,746 ground intervals, the world-frame interpretation had median/p90 vector error 0.31/1.14 rad/s, versus 0.33/1.86 when rotated from car-local coordinates and 3.75/6.06 when negated. On 262,200 air intervals, the corresponding errors were 0.62/2.64, 4.20/9.76, and 7.26/12.49 rad/s. The median direction alignment of world-frame angular velocity with quaternion motion was approximately 1.00 in both groups. This confirms the existing 0.01 scale and world-coordinate mapping; the remaining masked error is unlikely to come from a coordinate transform.

`evaluate_corpus --mask-seed 239847` uses each replay's SHA-256 and a fixed seed to select one four-frame gap at a different deterministic offset in every 100-frame block. Reports are `target/train-conversion-metrics-alt-mask.json` and `target/validation-conversion-metrics.json`. All 60 replays converted in each split with zero failures; the `test` split remains sealed. At horizon 4, car position RocketSim median/p90 versus constant-velocity extrapolation (UU) with gated dodge is:

| Split | Size | RocketSim | Linear |
| --- | --- | ---: | ---: |
| train | 1v1 | 17.3 / 47.4 | 27.5 / 66.8 |
| train | 2v2 | 16.2 / 39.8 | 26.4 / 62.9 |
| train | 3v3 | 18.1 / 47.3 | 30.2 / 65.8 |
| validation | 1v1 | 15.8 / 41.8 | 24.7 / 61.7 |
| validation | 2v2 | 15.1 / 38.7 | 26.3 / 64.2 |
| validation | 3v3 | 16.1 / 42.2 | 28.6 / 64.1 |

Under this independent schedule, validation horizon-4 car position p90 improved from 42.01 to 41.8 UU in 1v1, from 39.77 to 38.7 UU in 2v2, and from 45.30 to 42.2 UU in 3v3 (-3.1 UU!).

## Dodge torque calibration and motion-gated flip inference

### Replay torque calibration (`src/bin/calibrate_dodge.rs`)

Inspection across all 60 training replays identified 14,090 dodge counter activations on primary linked cars in active play. Of these, 13,914 (98.8%) carry a fresh `TAGame.CarComponent_Dodge_TA:DodgeTorque` vector in the same frame.

1. **Torque vector coordinate system:**
   - $T_x \in [-2.60, +2.60]$ (median absolute value 2.597): roll/yaw torque component with constant scale factor 2.60.
   - $T_y \in [-2.24, +2.24]$ (median absolute value 2.236): pitch torque component with constant scale factor 2.24.
   - $T_z = 0.000$ strictly in 100.0% of samples (no vertical torque).
   - Inverted stick mapping: $j_x = -T_x / 2.60$ and $j_y = -T_y / 2.24$.
   - The normalized stick magnitude $\sqrt{j_x^2 + j_y^2}$ has median exactly 1.000, with 100.0% of samples falling in $[0.90, 1.05]$.
   - In RocketSim car controls, `pitch = -T_y / 2.24` and `yaw = -T_x / 2.60`.

2. **Dodge impulse timing & duplicate impulse hazard:**
   - When a fresh rigid-body packet arrives at dodge activation frame $F$ (`linear_velocity.frame == F`):
     - The velocity change from frame $F-1$ to $F$ along the dodge direction has median **+461.25 UU/s** (interquartile range +365.4 to +524.8 UU/s).
     - The velocity change from frame $F$ to $F+1$ has median **+3.61 UU/s**.
     - This indicates that when velocity is freshly reported at the activation frame, the linear dodge impulse has usually already taken effect in the replay observation.
   - Injecting an active jump control into RocketSim while linear velocity is already present causes an immediate, duplicate 500 UU/s impulse, blowing out velocity and position prediction on subsequent frames.

3. **Motion-gated flip rule:**
   - When a dodge activation edge occurs while airborne:
     - If velocity is **unobserved** (`!car.body.linear_velocity.as_ref().is_some_and(|v| v.frame == frame)`), as happens during masked evaluation intervals or missing network frames, pass `controls.jump = true`, `controls.pitch = pitch`, and `controls.yaw = yaw` to trigger the dodge impulse and flip rotation in RocketSim.
     - If velocity is **already observed**, do not inject a duplicate jump control; instead, directly mark the car's state as flipping (`state.has_flipped = true`, `state.is_flipping = true`, `state.flip_rel_torque = Vec3A::new(tx / 2.60, ty / 2.24, 0.0)`, `state.flip_time = 0.0`). RocketSim simulates rotational flip torque without an extra linear impulse.

### Ablation results on train

| Metric | Baseline (`--no-inferred-dodge`) | Ungated (`--inferred-dodge`) | Gated default (`--gated-dodge`) |
| --- | ---: | ---: | ---: |
| One-step 1v1 car p50/p90/p99 (UU) | 17.86 / 42.33 / 69.43 | 17.98 / 43.54 / 72.46 | 17.85 / 42.29 / 69.33 |
| One-step 2v2 car p50/p90/p99 (UU) | 16.05 / 41.39 / 63.49 | 16.07 / 42.01 / 67.10 | 15.99 / 41.14 / 62.79 |
| One-step 3v3 car p50/p90/p99 (UU) | 17.25 / 42.61 / 69.74 | 17.26 / 43.03 / 70.93 | 17.19 / 42.33 / 69.30 |
| One-step all car p50/p90/p99 (UU) | 17.01 / 42.23 / 68.37 | 17.04 / 42.83 / 70.36 | 16.96 / 41.99 / 68.01 |
| Masked car pos h=4 p50/p90 (UU) | 17.13 / 42.41 | 16.83 / 41.12 | 16.77 / 40.25 |
| Masked car vel h=4 p50/p90 (UU/s) | 45.16 / 216.77 | 35.25 / 186.07 | 34.99 / 180.46 |
| Masked car rot h=4 p50/p90 (deg) | 3.45 / 26.05 | 3.39 / 20.96 | 3.39 / 20.96 |

Ungated dodge worsens one-step prediction significantly (p99 rises by +2.0 UU). In contrast, motion-gated dodge improves one-step prediction across every game size and quantile, while achieving the largest reduction in masked position, velocity (-36.3 UU/s at p90), and rotation error (-5.1 deg at p90).

### Validation generalization

On the 60 validation replays, the gains hold across both fixed-mask and pseudo-random (`--mask-seed 239847`) schedules:
- Validation one-step all car p50/p90/p99 improved from 16.45/41.33/69.87 to 16.39/41.09/69.61 UU.
- Validation fixed-mask car position h=4 p90 dropped from 41.63 to 39.55 UU (-2.08 UU).
- Validation fixed-mask car velocity h=4 p90 dropped from 202.04 to 168.55 UU/s (-33.5 UU/s, -16.6%).
- Validation fixed-mask car rotation h=4 p90 dropped from 24.77 to 20.47 degrees (-4.30 deg).
- Validation pseudo-random mask seed 239847 car position h=4 p90 dropped from 43.44 to 40.66 UU (-2.78 UU).

Gated dodge inference is enabled by default (`infer_dodge_from_active = true`, `gate_dodge_on_observed_impulse = true`). Baseline and ungated modes are accessible via `--no-inferred-dodge` and `--inferred-dodge`.

## Loadout body products and hitboxes

`TAGame.PRI_TA:ClientLoadouts` supplies a body product ID for each team. The extractor preserves both values on each player and attaches the currently selected one to a linked car. The user's August 2026 `items.csv` supplies product names; [Rocket League's official hitbox list](https://www.epicgames.com/help/c-37599050/c-32343914/a20257614?lang=en-US), additional [Season 22](https://www.rocketleague.com/news/rocket-league-season-22-training-rivalries-and-rewards) and [Season 23](https://www.rocketleague.com/news/hit-the-pitch-for-the-world-cup-in-rocket-league-season-23) announcements, and a localized official listing supply families. RocketSim's dedicated Psyclops preset covers that special body. The checked-in inputs, aliases, source URLs, generator, and unresolved rows are documented in [data/README.md](data/README.md). RocketSim's matching preset is used at car-slot creation.

| Product ID | Body | Hitbox | Playing slots in train |
| ---: | --- | --- | ---: |
| 21 | Backfire | Octane | 1 |
| 22 | Breakout | Breakout | 1 |
| 23 | Octane | Octane | 53 |
| 26 | Gizmo | Octane | 1 |
| 403 | Dominus | Dominus | 5 |
| 4284 | Fennec | Octane | 178 |
| 7012 | Tesla Cybertruck | Hybrid | 1 |
| 7477 | Nomad GXT | Merc | 0 |
| 7979 | Stampede | Merc | 0 |

The generated map covers 213 of 238 `Body` product rows; 11 of the 25 unresolved rows are generic drop or mystery labels, and 14 are named vehicles without a verified assignment in the checked sources. Unknown IDs remain raw and use an Octane fallback if selected by a playing car. The two rare training PRI products 7477 and 7979 were attached to non-playing actors; 7979 is now identified as Stampede, which the official list puts in Merc. All 240 playing train slots and all 240 playing validation slots have mapped IDs. Validation now has 237 Octane, two Plank, and one Hybrid slot. Its four formerly unresolved playing IDs are 25 Road Hog (Octane), 1691 Mantis (Plank), 1919 Centio (Plank), and 10900 Shokunin (Octane).

Compared with the previous eight-ID map, full-corpus train position metrics are unchanged. Validation gains two Plank slots. Four-frame car position median/p90 is unchanged in 1v1 and 2v2 to three decimals; 3v3 moves from 17.221/45.252 to 17.233/45.235 UU. One-step p90 in the Mantis replay improves by 0.008 UU, while the Centio replay worsens by 0.136 UU; other percentiles move in both directions. This is a state-fidelity correction, and the small sample does not establish a broad prediction gain. `--octane-hitbox` reproduces the original all-Octane setup.


## Boost pad pickup reconciliation and cooldown tracking

### Replay pickup extraction and spatial matching

Rocket League replays replicate vehicle pickups via TAGame.VehiclePickup_TA:NewReplicatedPickupData, which contains:
- pad_actor_id: The transient actor ID of the pickup entity.
- pad_actor_name: Name from the object table (e.g. cs_p.TheWorld:PersistentLevel.VehiclePickup_Boost_TA_0).
- instigator_car_id: The actor ID of the car that picked up the pad.
- picked_up: Pickup counter byte; non-255 train values were odd, while 255 marked an available/inactive pickup and had no instigator.

In observations.rs, these are captured per frame as pad_pickups: Vec<PadPickup>.

In conversion.rs, when options.sync_boost_pad_pickups is enabled (default true):
1. **Pad index mapping:** On an instigator event, the car's xy position is matched to the nearest RocketSim pad within 350 UU, provided the next nearest candidate is at least 100 UU farther away. Positions older than 0.1s are not used. The mapping is cached for that pad actor.
2. **Cooldown synchronization:**
   - On `picked_up == 255`: Resets cooldown to zero. This case is checked first because 255 is odd.
   - On any other odd pickup counter: Sets RocketSim cooldown to 10.0s for big pads or 4.0s for small pads.
3. **No future data leakage:** In evaluate_corpus.rs, frame.pad_pickups is explicitly cleared during masked evaluation windows, so pad events are only known when observed before the withheld gap.

### Evaluation metrics and ablation results

After fixing the 255 sentinel order and tightening spatial mapping, the default and `--no-sync-pads` pipelines each converted all 60 validation replays without failures. The same four-frame mask was used in both runs.

#### Masked boost error on validation: sync pads vs no sync pads

| Horizon | Samples | Pad sync sim p50 / p90 / p99 | No pad sync sim p50 / p90 / p99 | Hold baseline p50 / p90 / p99 |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 251 | **0.556** / 12.810 / 100.000 | 0.719 / **12.654** / 100.000 | 6.667 / 32.549 / 100.000 |
| 2 | 211 | **0.392** / **12.157** / 100.000 | 0.490 / 12.212 / 100.000 | 8.235 / 20.392 / 100.000 |
| 3 | 253 | **0.588** / **12.190** / 100.000 | 0.850 / 12.490 / 100.000 | 9.020 / 28.627 / 100.000 |
| 4 | 197 | **0.327** / **12.157** / 100.000 | 0.458 / 12.212 / 100.000 | 7.451 / 15.686 / 100.000 |

Across all horizons, pad synchronization improves median simulated boost error on validation:
- Horizon 1: 0.719 -> 0.556 boost units; p90 slightly worsens (12.654 -> 12.810).
- Horizon 2: 0.490 -> 0.392 boost units.
- Horizon 3: 0.850 -> 0.588 boost units.
- Horizon 4: 0.458 -> 0.327 boost units.

Car position errors generalize consistently without regression:
- Train all car position p50 / p90 / p99: 16.96 / 41.99 / 68.01 UU.
- Validation all car position p50 / p90 / p99: 16.39 / 41.09 / 69.56 UU.

Enabled by default with --sync-pads and ablatable with --no-sync-pads.


## Aerial steer control routing (`infer_air_steer_controls`)

### Physics analysis and calibration (`src/bin/calibrate_air_steer.rs`)

In RocketSim (update_air_torque), when a car is airborne (`!state.is_on_ground`), `CarControls.steer` only steers wheels on ground surfaces. In the air, RocketSim ignores steer and solely evaluates `CarControls.yaw`, `CarControls.roll`, and `CarControls.pitch`. When these controls are left at zero, RocketSim applies heavy aerodynamic angular damping (`air_control::DAMPING`), bringing rotational velocity to a halt.

In Rocket League replays, players replicate `ReplicatedSteer` (horizontal stick X) continuously. Pitch stick movements (stick Y) are not replicated in standard soccar network frames. Calibration on all 60 
`replays/train` (`src/bin/calibrate_air_steer.rs`) revealed:
- 709,925 total airborne frames across the training corpus.
- 68.1% of air frames (483,261) have non-neutral steer (|steer| > 0.1).
- Steer and local car yaw have the same sign in 283,213 frames vs opposite sign in 94,195 frames (a 3:1 majority), directly matching RocketSim's internal air torque coordinate convention (`dir_yaw = up_dir`).
- Routing steer to `controls.yaw` when airborne (`!state.is_on_ground`) allows RocketSim to accurately simulate player-guided aerial yaw rotation and cancel unwanted aerodynamic damping during active steering.

### Evaluation metrics and ablation results

Evaluated on all 60 `train` and 60 `validation` replays with `--no-infer-air-steer` ablation. Zero failures.

#### Masked car rotation and angular velocity p50 by horizon (Train)

| Metric | Horizon | No air steer (Baseline) | Air steer (`infer_air_steer_controls`) | Hold baseline |
| --- | ---: | ---: | ---: | ---: |
| Rotation angle (deg) | 1 | 1.87 | **1.84** | 8.10 |
| Rotation angle (deg) | 2 | 1.90 | **1.87** | 9.54 |
| Rotation angle (deg) | 3 | 2.74 | **2.66** | 13.36 |
| Rotation angle (deg) | 4 | 3.39 | **3.25** | 18.16 |
| Angular velocity (rad/s) | 1 | 0.442 | **0.414** | 0.511 |
| Angular velocity (rad/s) | 2 | 0.485 | **0.456** | 0.668 |
| Angular velocity (rad/s) | 3 | 0.599 | **0.565** | 0.937 |
| Angular velocity (rad/s) | 4 | 0.618 | **0.579** | 1.186 |

- Masked airborne angular velocity error p50 on train dropped from 1.343 rad/s to **1.255 rad/s**.
- One-step car position p50 / p90 / p99: 16.956 / 41.991 / 68.012 -> **16.955 / 41.991 / 68.033 UU**.

#### Masked car rotation and angular velocity p50 by horizon (Validation Generalization)

| Metric | Horizon | No air steer (Baseline) | Air steer (`infer_air_steer_controls`) | Hold baseline |
| --- | ---: | ---: | ---: | ---: |
| Rotation angle (deg) | 1 | 1.89 | **1.86** | 8.15 |
| Rotation angle (deg) | 2 | 1.89 | **1.85** | 9.71 |
| Rotation angle (deg) | 3 | 2.53 | **2.47** | 12.99 |
| Rotation angle (deg) | 4 | 3.18 | **3.08** | 17.89 |
| Angular velocity (rad/s) | 1 | 0.417 | **0.391** | 0.484 |
| Angular velocity (rad/s) | 2 | 0.473 | **0.436** | 0.650 |
| Angular velocity (rad/s) | 3 | 0.545 | **0.516** | 0.898 |
| Angular velocity (rad/s) | 4 | 0.612 | **0.578** | 1.125 |

- Masked airborne angular velocity error p50 on validation dropped from 1.300 rad/s to **1.218 rad/s**.
- One-step car position p50 / p90 / p99: 16.391 / 41.092 / 69.611 -> **16.391 / 41.092 / 69.613 UU**.

Enabled by default (`infer_air_steer_controls = true`) with `--infer-air-steer` and ablatable via `--no-infer-air-steer`.


## Look-ahead inverse aerial control inference (`infer_air_controls_from_lookahead`)

### Motivation and physics derivation

Rocket League replays replicate horizontal stick input (`ReplicatedSteer`) continuously, but completely omit vertical stick input (pitch) and directional air roll. Consequently, prior baseline conversions left simulated pitch and roll at `0.0` for 100% of frames. In mid-air, RocketSim applies heavy aerodynamic damping (`air_control::DAMPING`), bringing rotational angular velocities to zero unless counteracted by player controls.

Using RocketSim's air torque and damping equations from `rocketsim/src/sim/car/base.rs` and the local analytical formulation in `external/inverse_aerial_controls.py`, we estimate model-equivalent 3D aerial controls. These estimates use the next replay frame; they are not observed player inputs and are intended for offline conversion.

$$\text{dir\_pitch} = -\text{right\_dir}, \quad \text{dir\_yaw} = \text{up\_dir}, \quad \text{dir\_roll} = -\text{forward\_dir}$$

$$\text{TORQUE\_APPLY\_SCALE} = \frac{2\pi}{65536} \times 1000.0 \approx 0.0958738$$
$$\text{TORQUE} = (130, 95, 400) \times \text{SCALE} = (12.4636, 9.1080, 38.3495)$$
$$\text{DAMPING} = (30, 20, 50) \times \text{SCALE} = (2.8762, 1.9175, 4.7937)$$

For each control axis $i \in \{\text{pitch}, \text{yaw}, \text{roll}\}$:
$$\tau_i = \Delta \omega_i / \Delta t$$
$$\text{RHS}_i = \tau_i + \omega_i \cdot D_i$$
$$u_i = \text{clamp}\left(\frac{\text{RHS}_i}{T_i + \text{sign}(\text{RHS}_i) \cdot \omega_i \cdot D_i}, -1.0, 1.0\right)$$

### Corpus calibration on train (`src/bin/calibrate_inverse_air.rs`)

The original calibration tool estimated nonzero pitch on 36,423 of 53,565 airborne frame pairs (68.0%), yaw on 38,336 (71.6%), and roll on 33,483 (62.5%). Of 30,265 pairs with an active steer signal, estimated yaw had the same sign in 25,957 (85.8%). This is a consistency check, not independent controller ground truth. The calibration tool does not yet apply all continuity guards used by conversion.

### Unmasked physical fidelity (`src/bin/measure_air_fidelity.rs`)

`measure_air_fidelity` samples all 60 replays per split, matches each primary replay car to its RocketSim slot, requires the same car lifetime and fresh airborne endpoints without a dodge activation, and uses an isolated one-car simulation from each starting packet. This directly tests one-step control inversion with the future endpoint available. It does not include full match collisions or test causal prediction. These figures share the matched sample in the three-way comparison below.

| Dataset | Metric | Without Lookahead (Baseline) | With Lookahead (`infer_air_controls_from_lookahead`) | Improvement |
| --- | --- | ---: | ---: | ---: |
| **Train (51,045 pairs)** | Angular velocity p50 / p90 (rad/s) | 0.507 / 1.150 | **0.024 / 0.670** | Lower angular error |
| Train | Rotation p50 / p90 (deg) | **2.739 / 6.999** | 3.041 / 7.582 | Higher rotation error |
| **Validation (53,458 pairs)** | Angular velocity p50 / p90 (rad/s) | 0.449 / 1.123 | **0.028 / 0.672** | Lower angular error |
| Validation | Rotation p50 / p90 (deg) | **2.731 / 7.111** | 3.011 / 7.755 | Higher rotation error |

### Masked evaluation and leakage prevention

For the current four-frame mask in `evaluate_corpus.rs`, the next fresh angular packet is unavailable to the lookahead solver at the immediate masked boundary:
- Within a masked interval, `masked_observations` carries forward the prior rigid-body value and its original frame provenance. The lookahead solver requires a fresh next-frame packet, so it falls back to inferred aerial steering at that boundary.
- The converter also requires active play at both endpoints, the same car actor lifetime and owner, fresh positions above 100 UU, and no fresh next-frame dodge activation. These guards reduced unmasked angular fidelity in some cases but avoid inferring air controls from a contact, respawn, or play-state transition.
- A 60-replay validation ablation against `--no-infer-air-lookahead` produced identical masked car-position counts and p50/p90/p99 at horizons 1–4. Unmasked one-step car angular velocity p50 improved from 0.3985 to 0.3664 rad/s with lookahead; p90 improved from 2.5413 to 2.5277. Unmasked car-position quantiles changed by at most 0.001 UU. This establishes equality for the current four-frame mask, not for every possible masking protocol.

Enabled by default (`infer_air_controls_from_lookahead = true`), and ablatable via `--no-infer-air-lookahead` and `--infer-air-lookahead`.


## RLCarInputSolver comparison (supplied C++ code)

`external/RLCarInputSolver` estimates controls from two states. Its `Framework.h` expects a separate C++ RocketSim checkout, so the full ground solver is not directly executable against this project's native Rust RocketSim build. The replay stream directly supplies throttle, steer, and handbrake; it also supplies boost amount and boost/jump/dodge component counters. The C++ solver infers throttle, steer, handbrake, boost activation, and jump from motion. Those estimates may help when packets are absent, but they are not independent ground truth for recorded controls.

I ported the aerial orientation formula from `AirSolver.cpp` into `measure_air_fidelity` for a controlled comparison. All three columns use the same primary car pairs: active play at both ends, unchanged actor lifetime and player, fresh position/rotation/angular velocity at both ends, altitude above 100 UU, $0 < \Delta t \le 0.05$s, and no fresh dodge activation. Each pair starts a single Octane car from the replay state, applies controls for the elapsed RocketSim ticks, and compares the result with the next replay packet. “Replay-only” uses observed controls plus the project's steer-to-yaw fallback; “current inverse” uses the converter's offline aerial estimate; “RLCarInputSolver” replaces its pitch/yaw/roll with the supplied formula, including its deadzones and angular-speed correction. The full C++ boost, jump, flip, handbrake, and ground-steer heuristics are not tested here.

| Split | Pairs | Aerial controls | Angular velocity p50 / p90 (rad/s) | Rotation p50 / p90 (deg) |
| --- | ---: | --- | ---: | ---: |
| Train | 51,045 | Replay-only | 0.507 / 1.150 | **2.739 / 6.999** |
| Train | 51,045 | Current inverse | **0.024 / 0.670** | 3.041 / **7.582** |
| Train | 51,045 | RLCarInputSolver inverse | 0.113 / 0.673 | 3.041 / 7.630 |
| Validation | 53,458 | Replay-only | 0.449 / 1.123 | **2.731 / 7.111** |
| Validation | 53,458 | Current inverse | **0.028 / 0.672** | **3.011 / 7.755** |
| Validation | 53,458 | RLCarInputSolver inverse | 0.111 / 0.676 | 3.012 / 7.816 |

The supplied aerial formula improves angular-velocity fit over replay-only steering, but the current RocketSim-specific inverse is better at p50 and p90 on both splits. Neither inferred method improves isolated rotation error. Because both formulas use the next state, the low angular error is an offline fit, not evidence that they recover the player's actual stick input or improve masked forward prediction. Keep recorded replay controls as observations; use inverse estimates only for missing channels with provenance, and do not replace the current aerial formula on these results.

## Aerial rotation and replay-packet timing diagnosis

The inverse controls greatly reduce isolated next-packet angular-velocity error but increase orientation error. `src/bin/diagnose_air_rotation.rs` tests whether adjacent observed positions, orientations, and velocities describe motion over the elapsed replay-frame time. It uses active, unchanged primary car lifetimes with fresh fields at both ends, airborne positions above 100 UU, no fresh dodge activation, and $0 < \Delta t \le 0.05$s. For each pair, it projects observed displacement onto mean linear velocity and observed quaternion rotation onto mean world angular velocity. A scale of 1 means motion consistent with the full replay timestamp gap; a scale of 0.5 means the observed motion is half that implied by the reported velocity over that gap. The projection is a diagnostic, not a verified packet timestamp.

| Split | Nominal four-tick car translation scale p50 | Car rotation scale p50 | Ball translation scale p50 | Same-frame car/car scale difference p50 |
| --- | ---: | ---: | ---: | ---: |
| Train | 0.500 (47,431 pairs) | 0.511 (44,749 pairs) | 0.999 (448,308 pairs) | 0.001 (95,314 comparisons) |
| Validation | 0.500 (45,507 pairs) | 0.507 (42,801 pairs) | 0.997 (425,566 pairs) | 0.001 (89,268 comparisons) |

The nominal frame gap is usually four RocketSim ticks. Individual replays contain other projected factors, including approximately 1.0 and 1.75; the tool reports per-replay medians. `RecordFPS` is 30 in the inspected examples, and the match clock decrements about once per replay second even where car motion scale is 0.5. Ball packets track the nominal time much more closely. This points to car-specific packet or field timing/velocity semantics, but does not yet distinguish them from replication interpolation or another encoding rule. Applying a uniform half-time correction would hurt some replay intervals.

As a cross-field check, the tool estimates an effective interval from fresh car position and velocity, then integrates the *same pair's* mean angular velocity for that interval without using its target orientation to choose the interval. On pairs with a finite position-derived scale from 0.25 to 2.5:

| Split | Pairs | Full replay-gap rotation error p50 / p90 | Position-scaled rotation error p50 / p90 | Angular versus translation scale difference p50 / p90 |
| --- | ---: | ---: | ---: | ---: |
| Train | 47,227 | 2.904° / 6.726° | **0.105° / 1.549°** | 0.014 / 0.166 (44,456 pairs) |
| Validation | 48,646 | 2.881° / 6.798° | **0.097° / 1.560°** | 0.015 / 0.170 (45,687 pairs) |

This strongly links the orientation regression to a mismatch between the replay-frame gap and car motion implied by packet velocities. The diagnostic uses the next position and velocity, so it is **offline** and cannot be cited as causal masked-prediction improvement. It also does not establish the physical cause or justify changing the converter's 120 Hz replay timeline. The next experiment is to inspect raw actor update cadence, establish a packet-time model, and validate an explicit offline correction across all relevant state fields before enabling it.

The isolated RocketSim test also depends on its initial hidden car state. With `measure_air_fidelity --preserve-car-state`, the validation replay-only/current-inverse rotation p50 changes from 2.930° to 3.077° (the default isolated setup gives 2.731° to 3.011°), while p90 changes from 8.714° to 8.213°. Retaining simulated jump/flip flags changes the absolute errors and narrows the median regression, but does not remove it. This option is diagnostic; the converter was not changed.

## One-step pre-correction kinematic residuals

### Methodology and scope

The primary unmasked conversion pipeline (`convert_observations` / `convert_bytes`) steps RocketSim forward tick-by-tick between network observations. At each frame $F$ where a fresh rigid-body packet arrives (`actual.frame == F`), the simulation state immediately prior to applying the observation's rigid-body correction represents RocketSim's uncorrected 1-step prediction.

`src/conversion.rs` now records pre-correction kinematic residuals on `PositionResidual` for every fresh update in active play ($dt \le 0.5$s, primary linked cars for vehicles):
1. **Linear velocity error (UU/s):** Euclidean distance between pre-correction simulated velocity and fresh replay velocity, compared against a hold-last-observed-velocity baseline.
2. **Rotation error (degrees):** Geodesic angular distance $\theta = \arccos((\text{Tr}(R_{\text{sim}}^T R_{\text{replay}}) - 1) / 2)$ between pre-correction simulated orientation matrix and replay orientation matrix, invariant to quaternion sign ambiguities ($q \equiv -q$).
3. **Angular velocity error (rad/s):** Euclidean distance between pre-correction simulated angular velocity and replay angular velocity (scaled by 0.01 to convert from replay units to rad/s).
4. **Altitude stratification:** Cars are categorized by altitude: `ground` ($z < 50$ UU), `air` ($z > 100$ UU), and `transition` ($50 \le z \le 100$ UU).

`src/bin/evaluate_corpus.rs` pools and reports these quantiles across the entire corpus. All 60 train and 60 validation replays converted with zero failures.

The reviewed reports are `target/train-reviewed.json` and `target/validation-reviewed.json`, with validation ablations in `target/validation-reviewed-no-pads.json` and `target/validation-reviewed-no-lookahead.json`. These ignored files can be regenerated with the corresponding `evaluate_corpus` flags. The table reflects the final guarded converter and field-specific freshness limits.

### Aggregate 1-step kinematics: Train vs Validation

| Split | Body | Kinematic Field | Samples | RocketSim p50 / p90 / p99 | Hold Baseline p50 / p90 / p99 | Error Reduction (p50) |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| **train** | ball | Linear velocity (UU/s) | 504,518 | **5.39** / 15.52 / 1,237.34 | 21.66 / 37.39 / 2,321.68 | **-75.1%** |
| train | ball | Rotation (degrees) | 504,522 | **2.78** / 5.73 / 11.39 | 10.87 / 17.19 / 22.92 | **-74.4%** |
| train | ball | Angular velocity (rad/s) | 504,518 | 0.00 / 0.00 / 4.33 | 0.00 / 0.00 / 6.43 | Neutral |
| train | car | Linear velocity (UU/s) | 998,544 | **22.03** / 108.04 / 578.43 | 82.26 / 329.00 / 820.30 | **-73.2%** |
| train | car | Rotation (degrees) | 998,832 | **1.88** / 8.86 / 22.06 | 8.03 / 28.34 / 39.47 | **-76.5%** |
| train | car | Angular velocity (rad/s) | 998,544 | **0.372** / 2.552 / 5.782 | 0.514 / 2.086 / 5.556 | **-27.7%** |
| **validation** | ball | Linear velocity (UU/s) | 543,265 | **5.39** / 15.22 / 1,240.70 | 21.58 / 36.95 / 2,325.30 | **-75.0%** |
| validation | ball | Rotation (degrees) | 543,266 | **2.82** / 5.73 / 10.91 | 10.13 / 17.19 / 22.92 | **-72.2%** |
| validation | ball | Angular velocity (rad/s) | 543,265 | 0.00 / 0.00 / 4.38 | 0.00 / 0.00 / 6.46 | Neutral |
| validation | car | Linear velocity (UU/s) | 1,056,312 | **21.63** / 107.82 / 571.12 | 81.61 / 326.69 / 810.86 | **-73.5%** |
| validation | car | Rotation (degrees) | 1,056,577 | **1.85** / 8.75 / 21.95 | 7.92 / 28.10 / 39.12 | **-76.6%** |
| validation | car | Angular velocity (rad/s) | 1,056,312 | **0.366** / 2.530 / 5.780 | 0.506 / 2.073 / 5.548 | **-27.6%** |

### 1-step car angular velocity error by altitude

| Split | Altitude Region | Samples | RocketSim p50 / p90 / p99 (rad/s) | Hold Baseline p50 / p90 / p99 (rad/s) |
| --- | --- | ---: | ---: | ---: |
| **train** | Ground ($z < 50$ UU) | 557,448 | **0.152** / 1.121 / 2.657 | 0.330 / 1.651 / 5.245 |
| train | Transition ($50 \le z \le 100$ UU) | 142,390 | 2.013 / 4.616 / 6.638 | **0.688** / 2.789 / 5.679 |
| train | Air ($z > 100$ UU) | 298,706 | 0.891 / 2.994 / 6.560 | **0.826** / 2.449 / 6.099 |
| **validation** | Ground ($z < 50$ UU) | 593,612 | **0.150** / 1.127 / 2.673 | 0.324 / 1.642 / 5.235 |
| validation | Transition ($50 \le z \le 100$ UU) | 148,464 | 1.999 / 4.661 / 6.785 | **0.671** / 2.804 / 5.650 |
| validation | Air ($z > 100$ UU) | 314,236 | 0.887 / 2.979 / 6.495 | **0.825** / 2.444 / 6.074 |

### Key takeaways and diagnostics

1. **Substantial kinematic prediction gains over hold:**
   - On over 1 million car frame updates, RocketSim reduces median orientation error from ~8.0 degrees to **1.85 degrees (-76.6% error)** and linear velocity error from ~82 UU/s to **21.6 UU/s (-73.5% error)**.
   - For the ball, linear velocity error is reduced by **75%** (5.39 vs 21.58 UU/s) and rotation error by **72%** (2.82 vs 10.12 deg).
2. **Ground vs airborne dynamics:**
   - On the ground (over 55% of car updates), physical wheel contact and steering simulation reduce angular velocity error by more than half (**0.150 vs 0.324 rad/s** on validation).
   - In airborne flight, holding the prior angular velocity slightly outperforms the converter's full-match forward simulation (0.825 vs 0.887 rad/s on validation). This differs from the isolated one-car inversion diagnostic above, which uses the future endpoint directly.
   - In transition zones ($50 \le z \le 100$ UU), contact and takeoff impulses create the largest instantaneous angular discrepancies (2.00 vs 0.67 rad/s on validation).

## Offline car motion interval diagnostic (2026-09-28)

The earlier aerial analysis found that, on adjacent fresh airborne car packets, translation and rotation often imply about half the interval between replay frame timestamps when compared with their reported velocities. The ball usually implies the full interval. See the preceding **Aerial rotation and replay-packet timing diagnosis** for paired train and validation measurements. This does not identify the source of the discrepancy.

Commit `b2965e9` added `estimate_car_packet_interval`. Given two car positions and their linear velocities, it projects displacement onto mean velocity:

`t_projected = (p1 - p0) · mean(v0, v1) / |mean(v0, v1)|²`

The implementation rounds this result to a multiple of 1/120 s and attaches it to each eligible pre-correction car residual as `offline_interval`. It now returns no estimate when mean speed is below 100 UU/s, the fields are invalid, or the starting velocity and position came from different replay frames. The rounded count is a **motion-derived hypothesis**, not an observed server or client physics tick count. The replay frame timestamp and RocketSim's 120 Hz simulation schedule are unchanged.

The `offline_projection_fit` metric measures how closely the starting velocity times that fitted interval reaches the second position. **It is an in-sample fit:** the second position was already used to choose the interval, and the second velocity also enters the estimate. Consequently, this metric must not be compared with causal extrapolation as a prediction gain. The 5× and 225× position improvements claimed in the original branch commit and section do not establish better reconstruction or prediction. The sample sets also differ. We retain the fit only as a diagnostic of how well a quantized scalar interval describes observed translation.

There is independent, but narrower, cross-field evidence: the preceding diagnostic uses the position-derived interval to integrate angular velocity and evaluates against the *unused* target orientation. On its selected airborne pairs, validation median orientation error fell from 2.881° to 0.097°. That supports a shared translation/rotation timing or velocity-semantics effect on those pairs. It still uses future data, is not a causal test, and does not prove that the inferred tick count is the actual packet age.

The prior branch reported modes at 2 and 7 rounded ticks, but its raw actor-cadence analysis and isolated RocketSim experiment were not committed as reproducible source or reports. Those modes should be treated as **inferred motion intervals**, not raw packet timestamps. The following section measures actual per-actor update gaps and compares projected intervals by gap length, replay, and contact state. Any correction to the converter still needs independent validation, with the original replay timeline preserved.

## Raw actor cadence and frozen gap-scale check (2026-09-28)

`audit_packet_timing` counts consecutive fresh car position updates from the same primary linked actor lifetime during continuous `Active` play. It counts ball updates separately. These are **observed replay-frame gaps**, not inferred physics ticks. The ignored reports can be reproduced with:

```powershell
cargo run --release --bin audit_packet_timing -- replays/train target/train-packet-timing.json
cargo run --release --bin audit_packet_timing -- replays/validation target/validation-packet-timing.json target/train-packet-timing.json
```

| Split | Size | Car gap 1 / 2 / 3 / 4+ replay frames | Ball gap 1 frame |
| --- | --- | ---: | ---: |
| Train | 1v1 | 13,236 / 83,463 / 60,601 / 752 | 159,862 |
| Train | 2v2 | 53,861 / 122,466 / 78,094 / 30,908 | 135,325 |
| Train | 3v3 | 109,358 / 274,715 / 144,983 / 26,488 | 174,792 |
| Validation | 1v1 | 10,887 / 73,073 / 49,871 / 23,321 | 143,087 |
| Validation | 2v2 | 48,522 / 145,147 / 92,423 / 36,908 | 154,521 |
| Validation | 3v3 | 126,153 / 282,491 / 116,769 / 51,089 | 177,057 |

Two or three replay frames between car updates are common, so the converter often spans roughly 0.067-0.100 seconds (8-12 RocketSim ticks) before seeing a fresh car body. Ball updates usually appear every replay frame. Long car gaps are unevenly distributed: the largest train 4+ counts are 23,038, 15,432, and 14,351 in three individual replays. The raw-gap counts cover active primary cars, including known owners whose current link is inactive; an independent active-link-only train audit changed counts only slightly.

The same tool groups motion-derived intervals by raw gap. The training median **rounded interval / replay timestamp interval** is 0.50/0.75/0.50 for one-frame gaps in 1v1/2v2/3v3, about 1.125 for two-frame gaps, and 0.88-0.92 for three-frame gaps. Validation shows the same broad pattern, though its one-frame 2v2 median is 0.50. This explains why pooling all gap lengths obscures timing structure. These ratios are inferred from both endpoint positions and velocities; their rounded ticks are still not observed packet timestamps.

As a separate-field check, the position-derived interval is used to integrate angular velocity and compared with the unused target orientation. On validation one-frame airborne pairs, median orientation error falls from 3.406°/2.825°/3.094° using the nominal interval to 0.125°/0.097°/0.097° using the fitted interval for 1v1/2v2/3v3. This is an offline cross-field fit; target position and velocity are already known when the interval is chosen.

To test whether a timing scale transfers without using the *scored* endpoint position, the tool takes the median projected scale for each game size and raw gap of 1-3 frames from the **train** report, freezes those nine values, and scores simple start-velocity position and start-angular-velocity orientation extrapolations on **validation**. Each baseline and model value uses the same eligible pairs. The raw gap becomes known when the next packet arrives, so this is an offline packet-endpoint model, not a 120 Hz forward simulation policy.

| Validation size | Position pairs | Nominal / frozen-gap position p50; p90 (UU) | Air rotation pairs | Nominal / frozen-gap rotation p50; p90 (degrees) |
| --- | ---: | ---: | ---: | ---: |
| 1v1 | 133,831 | 18.99 / **11.94**; 44.61 / **33.73** | 36,047 | 4.04 / **3.50**; 11.09 / **10.58** |
| 2v2 | 286,091 | 18.25 / **13.09**; 45.39 / **37.21** | 79,153 | 3.56 / **3.09**; 10.01 / **9.37** |
| 3v3 | 525,412 | 18.87 / **12.97**; 45.05 / **38.22** | 148,491 | 3.33 / **2.76**; 9.43 / **9.15** |

Despite the pooled gains, **8 of 60 validation replays regress** in median position and median air rotation. The worst per-replay median position regression is 8.58 UU. A naive alternative that carries the previous pair's fitted scale forward is worse across all game sizes (validation pooled-by-size median position error rises from 17.56/17.34/17.85 to 28.46/28.95/31.63 UU). The gap-scale model is therefore a diagnostic candidate, not a converter correction. Replay-specific modes, long-gap cases, contacts, and masked RocketSim behavior need further work; `replays/test` remains sealed.

## Earlier-packet replay calibration (2026-09-28)

The eight validation replays where the frozen train gap scale worsened both median position and air rotation have mostly nominal-looking motion intervals. Their frozen-scale median position regressions range from 2.24 to 8.58 UU. This motivated a replay-specific selector in `audit_packet_timing`: for each raw gap of 1–3 frames, it starts with the frozen train scale and counts whether nominal or frozen start-velocity extrapolation wins on **completed earlier pairs in that replay**. After 64 scored pairs for that gap, it selects nominal time if nominal wins exceed frozen wins by 10%. It scores the current endpoint before adding its result to the history. This is an offline endpoint extrapolation, not a RocketSim or replay-tick correction. The selector was fixed before the validation run.

Reproduce the paired reports with:

```powershell
cargo run --release --bin audit_packet_timing -- replays/train target/train-packet-timing-adaptive.json target/train-packet-timing.json
cargo run --release --bin audit_packet_timing -- replays/validation target/validation-packet-timing-adaptive.json target/train-packet-timing.json
```

All 60 train and 60 validation replays parsed. The following validation numbers compare **the same eligible pairs**; columns are nominal / frozen train scale / earlier-packet selector, with position in UU and airborne orientation in degrees. The orientation endpoint is excluded from scale selection.

| Size | Position pairs | Position p50 / p90: nominal · frozen · selector | Air rotation pairs | Air rotation p50 / p90: nominal · frozen · selector |
| --- | ---: | --- | ---: | --- |
| 1v1 | 133,831 | 18.99 / 44.61 · 11.94 / 33.73 · 11.71 / 34.06 | 36,047 | 4.04 / 11.09 · 3.50 / 10.58 · 3.49 / 10.64 |
| 2v2 | 286,091 | 18.25 / 45.39 · 13.09 / 37.21 · 12.96 / 37.18 | 79,153 | 3.56 / 10.01 · 3.09 / 9.37 · 3.08 / 9.35 |
| 3v3 | 525,412 | 18.87 / 45.05 · 12.97 / 38.22 · 12.43 / 38.26 | 148,491 | 3.33 / 9.43 · 2.76 / 9.15 · 2.70 / 9.19 |

Relative to the frozen model, validation replay median position improves on 11 replays, worsens on 3, and is unchanged on 46; air rotation improves on 10, worsens on 2, and is unchanged on 48. The largest added regression versus frozen is 0.13 UU in replay median position. Relative to nominal time, **8/60 validation replay median positions and 7/60 air rotations still regress**. For the eight original position failures, the selector cuts the worst replay median regression from 8.58 to 1.05 UU, but does not eliminate it. The 1v1/3v3 pooled p90 position and rotation also rise slightly over frozen. Validation position p99 is 73.50/68.54/72.18 UU with nominal time, 89.61/75.71/95.57 UU with frozen scale, and 89.14/75.65/95.54 UU with the selector across 1v1/2v2/3v3. Train results have the same directional pooled median gains but are descriptive because the frozen scale was derived from train. This selector therefore remains diagnostic and is **not enabled in conversion**. A later timing correction needs paired masked RocketSim validation, contact and long-gap analysis, and no material per-replay regression; the 120 Hz replay timeline remains unchanged. `replays/test` remains sealed.

## Low-air transition diagnosis and control ablations (2026-09-28)

`evaluate_corpus` now subdivides field-fresh one-step car angular residuals whose **current observed center height is 50–100 UU**. Origin is the preceding observed position carried in the previous replay frame; `sim_ground`/`sim_air` is RocketSim's pre-correction ground flag. The event labels mean a fresh odd replay jump, double-jump, or dodge component packet within the preceding 0.15 s, searched with the same actor lifetime; they are packet proxies, not verified player inputs. The categories are separate marginal views, so their counts should not be added together. Each metric compares the same fresh angular packets with the pre-correction simulator and held previous observed angular velocity.

| Validation context | Paired samples | RocketSim / hold p50 (rad/s) |
| --- | ---: | ---: |
| Origin below 50 UU | 19,265 | 1.490 / 1.649 |
| Origin already 50–100 UU | 119,381 | 2.116 / 0.523 |
| Origin above 100 UU | 9,818 | 0.692 / 0.947 |
| RocketSim airborne | 139,221 | 2.088 / 0.634 |
| No recent jump or dodge packet | 121,847 | 1.770 / 0.545 |
| Recent jump packet | 6,045 | 0.840 / 0.882 |
| Recent dodge packet | 19,754 | 3.244 / 2.936 |

The corresponding train split has 114,598/142,390 samples already in the band, 133,217/142,390 simulated airborne, and 117,127/142,390 without a recent jump or dodge packet. Thus the large transition-band error is mostly **persistent low-air motion**, rather than a single takeoff impulse. This is an inference from the marginal groups; collision proximity and the exact replay input remain unobserved.

Two opt-in ablations were fixed on train and then checked on all 60 validation replays, with the default converter unchanged:

1. `--infer-transition-air-lookahead` lowers the existing offline inverse-control height guard from 100 to 50 UU while retaining its fresh adjacent endpoint, active-play, actor-lifetime, and dodge guards. Validation transition angular p50/p90/p99 changes from 1.999/4.661/6.785 to 1.986/4.643/6.783 rad/s. Overall one-step angular p50 improves in all 60 replays, but overall rotation p50 worsens in 42/60, and all four masked angular horizons are unchanged. The inverse uses the future angular endpoint; this result measures offline fit, not causal prediction.
2. `--compensate-transition-air-damping` uses current simulated orientation and angular velocity to solve RocketSim aerial controls for zero angular acceleration while airborne at 50–100 UU. It uses no future packet values. Validation transition angular p50/p90/p99 changes from 1.999/4.661/6.785 to **1.657/4.633/6.764** rad/s (train 2.013/4.616/6.638 to 1.668/4.589/6.616). Overall validation one-step angular p90 improves from 2.528 to 2.495 rad/s, and rotation p90 improves from 8.750 to 8.458 degrees; both p90 fields improve in all 60 validation replays. But rotation p50 rises from 1.855 to 1.873 degrees and worsens in 55/60 replay medians.

The causal damping ablation also has a consistent masked tradeoff at horizon four:

| Validation size | Angular p50 / p90: default → damping (rad/s) | Rotation p50 / p90: default → damping (degrees) |
| --- | --- | --- |
| 1v1 | 0.893 / 4.053 → 0.925 / 3.896 | 4.142 / 25.239 → 4.255 / 23.045 |
| 2v2 | 0.577 / 3.581 → 0.582 / 3.474 | 3.028 / 20.812 → 3.116 / 19.675 |
| 3v3 | 0.519 / 3.347 → 0.543 / 3.307 | 2.899 / 18.923 → 2.974 / 18.437 |

Both ablations remain **off by default** because neither improves typical and tail masked errors together. The damping controls are physically motivated simulator compensation, not recovered player inputs. The diagnostic does not establish that RocketSim damping is the only cause; raw car packet timing, wheel contact, and unobserved pitch/roll remain possible contributors. Reproduce ignored reports with `cargo run --release --bin evaluate_corpus -- replays/<split> target/<split>-takeoff-context.json` and the same command with `--infer-transition-air-lookahead` or `--compensate-transition-air-damping` and distinct output names. Train/validation each converted 60/60 under all three settings; `replays/test` remains sealed.

## Compact columnar format prototype (2026-09-28)

`python/replay_columnar.py` converts the Rust schema-v1 JSONL output in bounded batches to Arrow IPC (`.arrow`, zstd) or Parquet (`.parquet`, zstd level 3). Each replay frame has typed time/tick, ball, car, control, pad, score, and clock columns with the same dense shapes, NaN missing values, slot order, and masks as `load_numpy`. The full frame JSON is also retained as a compressed binary column, so typed access does not discard hidden RocketSim fields, replay observations/provenance, events, or residuals. The original header, dependency revisions, source hash, options, car slots, and diagnostics are stored in schema metadata. This is a **Python second stage after Rust JSONL**, not a direct Rust writer or a restored RocketSim arena.

The optional Python dependencies used here were Python 3.11.8, NumPy 2.4.4, and PyArrow 25.0.1 on Windows. PyArrow is pinned in `python/requirements-columnar.txt`. Reproduce one benchmark for each size with the named training replay, for example:

```powershell
python -m pip install --target target/pydeps -r python/requirements-columnar.txt
$env:PYTHONPATH = 'target/pydeps'
cargo run --release --bin convert_replay -- replays/train/1v1/0000a984-75af-4b24-b5a6-cb3663fc4efa.replay target/columnar-sample.jsonl
python python/benchmark_columnar.py target/columnar-sample.jsonl --repeats 3 --report target/columnar-benchmark.json
```

The same commands used `replays/train/2v2/000c0390-fc3c-4a68-8360-3979d9d88aaf.replay` and `replays/train/3v3/00054e5d-90dd-4e96-a922-bf28ae08513a.replay`, with `target/columnar-2v2.jsonl` and `target/columnar-3v3.jsonl`. The ignored reports are `target/columnar-benchmark.json`, `target/columnar-2v2-benchmark.json`, and `target/columnar-3v3-benchmark.json`. Source SHA-256 values are in those reports. The script regenerates both columnar files and gzip JSONL, times three complete dense-array loads for each format, and compares every NumPy array and **every complete rich frame** against the Rust JSONL. All three replays passed exact Python dictionary and NumPy array equality, including missing values. Synthetic tests also cover both formats and gzip JSONL.

| Train size | Frames | Raw JSONL MB / read s | gzip JSONL MB / read s | Arrow IPC MB / read s | Parquet MB / read s |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1v1 | 11,663 | 129.29 / 2.51 | **7.57** / 2.96 | 9.30 / 0.051 | 10.50 / **0.018** |
| 2v2 | 6,786 | 118.67 / 2.26 | **7.46** / 2.62 | 9.16 / 0.048 | 10.26 / **0.011** |
| 3v3 | 10,935 | 262.82 / 5.61 | **17.46** / 6.39 | 21.58 / 0.107 | 24.07 / **0.017** |

Sizes are decimal MB; read times are median seconds over three warm local reads of the same dense outputs, excluding conversion/write time. Parquet's projected loader skips the rich JSON column, whereas the prototype Arrow loader reads the whole IPC table before selecting columns; this difference is implementation-specific. Gzip JSONL is smallest, but still needs JSON parsing for dense arrays. On the 1v1 sample, a single rich-frame pass took 1.16 s JSONL, 1.35 s gzip, 1.13 s Arrow, and 1.22 s Parquet; rich access is therefore much closer across formats. Columnar write time was about 3.3–8.0 s across these samples, excluding Rust replay conversion. These are development machine measurements on one replay per size, with OS-cache effects; they do not establish corpus-wide throughput.

**Format decision for the next implementation step:** target Parquet for Python ML array access while retaining JSONL as the direct, inspectable Rust output until a native streaming writer exists. Arrow IPC remains available when its somewhat smaller files or batch semantics are preferable. The hybrid payload duplicates some typed values; a direct Rust Parquet writer, bounded-memory conversion, and a state-restoration check are still needed before declaring Phase 5 complete. `replays/test` remains sealed.

## Direct Rust Parquet export (2026-09-28)

`convert_replay` now writes `.parquet` directly from Rust using pinned Apache Arrow/Parquet 60.0.0 with zstd level 3. It preserves the Python prototype's columnar version 1 schema: final JSONL-equivalent header in metadata, first-frame pad configuration, typed state/action/score/clock columns, and a complete `frame_json` payload per row. A callback emits each corrected RocketSim snapshot and its residuals without keeping all snapshots in the Parquet path. Final car slots and conversion diagnostics are learned in a first simulation pass; a second pass writes 512-frame row groups. Both passes' complete car-slot metadata and diagnostics must agree. `boxcars` parsing, replay bytes, and the extracted observations remain replay-sized in memory because offline control inference can inspect future observations. This is bounded **snapshot/output** memory, not bounded total conversion memory. No peak-memory or throughput claim has yet been measured.

The reusable `python/verify_direct_parquet.py` check compares the complete header, every dense NumPy array (including NaN missing values), every rich frame dictionary, and the 512-row-group bound against a Rust JSONL export. Run, for example:

```powershell
cargo run --bin convert_replay -- replays/train/1v1/0000a984-75af-4b24-b5a6-cb3663fc4efa.replay target/direct-1v1.parquet
$env:PYTHONPATH = 'target/pydeps'
python python/verify_direct_parquet.py target/columnar-sample.jsonl target/direct-1v1.parquet
```

The checked training replays were the same source files and JSONL references as in the columnar prototype section above. The validation file was `replays/validation/1v1/1a3ac92c-961e-469b-9d1a-29d210a7b5d9.replay`, independently exported to both JSONL and Parquet with the same default options. All four comparisons passed:

| Split and size | Frames | Direct Parquet bytes | Header / arrays / rich frames |
| --- | ---: | ---: | --- |
| Train 1v1 | 11,663 | 10,541,260 | Exact |
| Train 2v2 | 6,786 | 10,324,582 | Exact |
| Train 3v3 | 10,935 | 24,304,160 | Exact |
| Validation 1v1 | 12,724 | 11,882,204 | Exact |

An initial parity failure found that promoting the replay's binary `f32` time directly to `f64` changed nearly every Python time value compared with reading JSONL's shortest decimal representation. The writer now parses that same decimal representation for the typed time column, and the full-array comparisons pass. `cargo test --all-targets` and Python loader tests pass. The format remains opt-in via the `.parquet` output extension. Restoring a serialized snapshot into a RocketSim arena, total peak-memory measurement, and wider validation checks remain open before making Parquet the default ML export. `replays/test` remains sealed.

## Soccar state restoration and output resource use (2026-09-28)

`src/restoration.rs` reads schema-v1 header slots and rich frame payloads, then rebuilds a detached native RocketSim `ArenaState` with the serialized arena tick, ball, car controls and timers, hitbox/team configuration, and boost-pad configuration/cooldowns. It rejects unsupported header revisions/modes, nonsequential or mismatched car slots, unknown hitboxes, and inconsistent pad activity. `verify_state_restoration` checked every field after JSON parse → native state → transfer record on four prior direct Parquet exports (train 1v1/2v2/3v3 and validation 1v1), plus the largest validation 3v3 replay by source-file size: **80,246/80,246 exact detached snapshot round trips**. It also seeded fresh live arenas at every 5,000th frame (19 samples); all public ball/car fields matched immediately, and maximum observed pad cooldown quantization error was 0 seconds. The validation 3v3 file was `replays/validation/3v3/1a0b10f9-c7b7-428a-b745-b5e7607a2d8a.replay` (38,138 frames). Reproduce with `cargo run --release --bin verify_state_restoration -- target/<export>.parquet` after direct conversion.

**Live continuation is not exact.** The pinned RocketSim API cannot set a live arena's absolute tick, RNG state, or private physics/contact/wheel caches. Applying a snapshot leaves the arena's own tick unchanged, and pad cooldown is translated through a rounded 120 Hz pickup tick. Soccar has no tile state; the current transfer schema omits Dropshot tile state and supports only the validated soccar path. A detached `ArenaState` faithfully represents the public snapshot, but stepping a newly seeded live arena is not guaranteed to reproduce the original future trajectory. This limitation is returned in `ArenaApplyReport` as source tick, live arena tick, and maximum pad cooldown error.

`python/benchmark_conversion.py` launched the same prebuilt release converter for paired JSONL and direct Parquet exports, with one warmup then three measured repetitions per format/replay and alternating format order. It recorded child-process wall time, Windows peak working set via psutil, frame count, and file size. The ignored raw reports with source hashes, executable hash, all repetitions, and machine details are `target/conversion-benchmark.json` and `target/conversion-benchmark-large.json`. Reproduce with `cargo build --release --bin convert_replay`, install `python/requirements-benchmark.txt`, then pass the paths listed below to the benchmark script; it rejects `replays/test` and removes generated outputs after measurement unless `--keep-files` is supplied.

```powershell
python python/benchmark_conversion.py replays/train/1v1/0000a984-75af-4b24-b5a6-cb3663fc4efa.replay replays/train/2v2/000c0390-fc3c-4a68-8360-3979d9d88aaf.replay replays/train/3v3/00054e5d-90dd-4e96-a922-bf28ae08513a.replay replays/validation/1v1/1a3ac92c-961e-469b-9d1a-29d210a7b5d9.replay replays/validation/2v2/1a00cbd5-38db-4309-a695-648f1b82792a.replay replays/validation/3v3/1a1c065f-dbfd-4c62-ac87-05f27b3c7d97.replay --report target/conversion-benchmark.json
python python/benchmark_conversion.py replays/train/3v3/001c4769-124f-4954-ba1f-45216cb7f165.replay replays/validation/3v3/1a0b10f9-c7b7-428a-b745-b5e7607a2d8a.replay --report target/conversion-benchmark-large.json
```

| Split / size / replay prefix | Frames | JSONL seconds / peak MB / output MB | Parquet seconds / peak MB / output MB |
| --- | ---: | ---: | ---: |
| Train 1v1 `0000a984` | 11,663 | 0.518 / 95.7 / 129.29 | 0.951 / 82.3 / 10.54 |
| Train 2v2 `000c0390` | 6,786 | 0.436 / 72.4 / 118.67 | 0.867 / 77.8 / 10.32 |
| Train 3v3 `00054e5d` | 10,935 | 0.929 / 137.4 / 262.82 | 1.812 / 119.2 / 24.30 |
| Validation 1v1 `1a3ac92c` | 12,724 | 0.560 / 103.2 / 147.00 | 1.053 / 87.6 / 11.88 |
| Validation 2v2 `1a00cbd5` | 14,137 | 0.889 / 136.8 / 250.05 | 1.770 / 104.8 / 22.05 |
| Validation 3v3 `1a1c065f` | 9,978 | 0.867 / 130.7 / 244.23 | 1.709 / 116.3 / 22.88 |
| Largest train 3v3 `001c4769` | 17,001 | 1.443 / 215.6 / 427.24 | 2.940 / 160.0 / 39.51 |
| Largest validation 3v3 `1a0b10f9` | 38,138 | 2.714 / 423.0 / 907.19 | 5.220 / 273.8 / 73.28 |

All figures are medians of three measured runs, decimal MB, on one Windows development machine. Parquet saves peak memory on seven of eight sampled replays, but **uses 5.4 MB more on the sampled train 2v2**. It takes about twice as long to export because it simulates the replay twice to discover final fixed-width car slots before writing; this is end-to-end export time, not a serializer-only comparison. Peak working set includes parsing, extracted observations, RocketSim, output buffers, and allocator behavior, but excludes filesystem cache and other processes. Cache, disk, and antivirus effects remain possible. The largest sampled Parquet peak is 273.8 MB; observations and input bytes still scale with replay size. A separate streaming parser/observation redesign is worthwhile for very large or highly concurrent workloads, but is not a prerequisite for recommending Parquet for ML reads of the supplied soccar corpus. JSONL remains useful for inspection and faster one-off exports. `replays/test` remains sealed.

## Car-ball consistency and run-level lag error (2026-10-01)

**Metric.** `rlbot_reconstruction` now scores, for every frame with a car within 350 UU of the exported ball, the exported relative position (car minus ball) against the server truth at the single best-matching true tick, and prints that tick minus the frame's tick. It isolates the mismatch between the lag levels of ball and car chains. Client game 1 (inferred lags): relative position error p50 / p90 / p99 25.7 / 54.1 / 709.8 UU, best tick minus frame tick p10 / p50 / p90 -16 / 1 / 7; with the true lags (ORACLE) 0.0 / 24.7. Host replay (spurious lags of 1-2 ticks): 0.0 / 10.1 / 17.4; oracle 0.0 / 0.0 / 0.1.

**Run levels** (`scripts/rlbot_run_levels.py STATES.jsonl REPLAY_DUMP.jsonl`; client game 1; chain-lag packets matched to a unique server tick; a run is consecutive packets of one object at most 3 frames apart whose inferred-minus-true error stays constant; truth is the offset against a running minimum, so only differences between runs are meaningful). Ball: 166 runs of 3+ packets (median 17), run-level error |p50| 3 ticks, p90 8.5, independent of run length (3 ticks at 3-6 packets, 3 at 30+). Cars: 979 runs, |p50| 1, p90 11, 0 / 10 for runs over 30 packets. Longer runs are not better placed: the level is set by the window prior at the run's ends, so it needs information from outside the run (car-ball contacts). Not yet implemented.

## Ball runs joined across hits and placed against the cars (2026-10-01)

**Problem.** Chain runs of the ball and of each car sit on separate lag levels (previous section): the ball's feasible level range is about 5.6 ticks wide even for runs of 8+ packets (its lag is concentrated, so the frame window barely bounds it), whereas a car run of 8+ packets is pinned to a median 1.4 ticks. The exported ball was 36 UU (p50) from the truth on a client replay and the car-ball relative position 26 UU.

**Two changes** (both offline, both use packets after the frame).
1. *Ball runs continue across a hit* (`ball_hit_chains`, `ball_hit_interval_ticks`). Smooth chains stopped at every hit. The ball follows the exact free-flight map (gravity then damping `0.97^dt`, position moved with the new velocity; 0.008 UU/s and 0.005 UU against server states) before and after a hit, and the two paths meet at the hit; the ticks between the two packets are the candidate for which the cross-track miss of the meeting is smallest. The displacement of the hit tick itself lies along the velocity change by a fraction of one tick of it (p10 / p50 / p90 -0.39 / 0.00 / +0.41 of dv*dt; 91% of the miss is along dv), so only the component across the velocity change counts and the fraction must lie in [-0.6, 1.2]. A pair is used only if its best candidate beats the second by more than 2 UU and the ball is in free flight (away from floor, ceiling, walls, goals) over both paths. Validation on server truth (client game 1, 208 hits; game 2 not tuned): exact elapsed ticks in 89% (95.5% of those with margin above 2 UU, 100% above 10 UU), within one tick 96%; the miss alone (no along-dv freedom) was exact in 77%. The bridged pair is one more link in the exact chain, whose window bounds must still hold (an inconsistent link ends the run).
2. *Ball runs placed against the car runs* (`place_ball_runs`). In one frame the ball's server tick minus a car's has mean 3.1 ticks, standard deviation 2.55 (both games, stable per car and per minute; car minus car: mean 0.03, standard deviation 3.46), so a ball run starts at the mean over its frames of (car ticks + offset - own K), rounded and clamped to its feasible range. Car runs do not move: a least-squares joint solve that also pulled cars toward the ball made the cars worse (position p50 0.68 to 1.7 UU) and was dropped.

**The offset is estimated from the hits, not set** (`estimate_ball_car_lag_offset`, default on, `estimate_ball_car_offset`). At the last free-flight state before each bridged hit, the gap between the nearest car's hitbox (position, velocity, gravity and angular velocity extrapolated from its nearest packet within 12 ticks; the RocketSim hitbox of its body) and the ball surface is computed; the offset is where the median gap over the hits first drops to zero. It needs 20 hits. Estimates against the truth 3.1: game 1 2.75, game 2 3.05-3.3 (varies slightly with the placement); host replay (true 0, no real lag) -0.6; none on most corpus replays (fewer than 20 bridged hits in a 30 fps replay), 0.2-0.4 where defined. Gap profile on game 1: 23 UU at offset 0, 5.6 at 2, 0.9 at 2.5, -0.9 at 3, -2.5 at 3.5, then flat around -3.9. A fixed offset 3.1 gives the same result as the estimate within noise, an offset 1 tick away costs about 3 UU at p50 (1: 20.2, 2: 12.1, 2.5: 8.2, 3.1: 5.6, 3.7: 7.7, 4.5: 9.2 UU in the car-ball metric).

**Result** (`rlbot_reconstruction`, client replays, exported state against the server truth, before / after; `NO_AIR_BVP NO_FIT_NEXT` base): car-ball relative position p50 / p90 25.6 / 54.3 to 8.4 / 29.2 UU (game 1), 25.3 / 53.3 to 7.7 / 23.7 (game 2); ball position p50 / p90 35.8 / 63.9 to 6.0 / 20.1 and 36.0 / 64.3 to 0.01 / 18.4; ball velocity p90 18 / 21 to 6 / 6 UU/s; cars unchanged (position p90 20.7 / 21.4 to 20.6 / 21.4). Host replays are unaffected. Corpus check (`error_budget replays/train` and `replays/validation`, ball and near-ball rows): neutral to slightly better (ball residuals +0.15% to +0.2% usable, CAR near ball p99 22.1 to 21.6 UU on train, 20.7 to 20.3 on validation, ball near car p99 27.7 to 26.6 / 26.2 to 25.6). Flags: `error_budget --no-ball-hit-chains --estimate-ball-car-offset`, `evaluate_corpus` the same, `rlbot_reconstruction` env `NO_BALL_HITS`, `NO_EST_MU`, `LAG_MU=x` (fixed offset). `scripts/rlbot_run_levels.py` prints the run-level error, feasible widths and same-frame offsets.

**What is left.** The remaining car-ball error (p50 8, p90 25-29 UU) is the whole-tick placement of single runs (one tick is 17-25 UU at 1000-1500 UU/s); individual hits could refine the ball run next to them, and the offset is a per-replay constant that was only validated on two LAN games (a 10 fps client). The estimator failed (no result) on replays with fewer than 20 hits, which leaves them with the box midpoints as before.

## Lag-free replays are recognised (2026-10-01)

A replay saved by the host or server has every car and the ball fresh in every frame at the frame's own tick, but the chain inference gave 1-2 ticks of spurious lag (the BakkesMod dump and the host replays of the remote-client games). `detect_lag_free_replays` (default on) counts the exact chain links whose elapsed ticks equal the gap of the frame timeline: 99.5% and 99.7% on the two host replays (62,757 and 66,417 links), 24.5% and 23.9% on the two remote-client replays, 13-47% on 36 client replays of the training split (12 of each size). A replay with at least 200 links of which 90% match gets lag 0 for every fresh packet (`zero_packet_lags`). Host replay of game 1, exported state against the server truth, before (inferred lags) / after: car position p50 / p90 / p99 0.01 / 0.1 / 18 to 0.00 / 0.0 / 0 UU, velocity p99 5 to 0 UU/s, rotation p90 0.29 to 0.00 deg, ball position p50 13.6 to 0.00 UU, car-ball relative position p50 9.1 to 0.0 UU. No corpus replay is detected (all client replays), so the corpus results are unchanged. Disable with `ConvertOptions::detect_lag_free_replays`; `rlbot_reconstruction` env `NO_LAG_FREE`.

## Actions against the server's applied input, and when a control change takes effect (2026-10-01)

**Score.** `rlbot_reconstruction` now compares the exported controls of every active frame (`state.cars[].controls`) with the input the server applied in the tick that ended at the frame's server tick (RLBot `last_input`; pitch, yaw and roll only while airborne). Host replay of game 1 (lag-free; |error| > 0.1 in): throttle 0.02%, steer 0.05%, handbrake 0.01% (observed controls, exact), jump 0.5% on the ground and 14% airborne (the held jump button: the simulation releases it early), boost 1.5%, pitch / yaw / roll 47% / 41% / 53% of airborne frames (mean |error| 0.39 / 0.31 / 0.39: the air controls come from the boundary-value solve, constant per segment of about four ticks, and the true inputs of bots and humans are mostly full deflections, so these are model controls that reproduce the states, not recovered inputs). Client replay: throttle 6.2% (ground), steer 18% on the ground and 32% airborne (air steer is yaw/roll input, unused on the ground), handbrake 2.5% / 8.7%, boost 3.4%, jump 1.1% / 12%.

**The recording client's own car dominates the steer error.** Frames with a ground steer error above 0.5 on the client replay of game 1: remote human (the player who recorded the replay) 802, host human 109, four bots 155-215 each. A player's own client applies its input at once and the server later, so for the recorder's own car the replay shows a change before the server's state reflects it (STEER_TRACE shows observed changes 3-6 ticks before the truth's); for everyone else the change is replicated with the car's next update.

**Where a change took effect** (`target/ctl_packet.py` analysis, client replays; truth: the first tick of the new input in `last_input`). 97.0% / 97.1% of 10,130 / 11,109 observed throttle, steer and handbrake changes lie inside the interval `(S_prev, S_cur]` between the server ticks of the car's own previous packet and its packet in the frame where the change is first seen, uniformly (position in the interval p10 / p50 / p90 0.125 / 0.57 / 1.0; interval p10 / p50 / p90 8 / 9 / 17 ticks). The middle of that interval predicts the change tick within 2 ticks for 51% of changes (the `2 + gap / 2` rule before the frame time: 37%, and 4.2 ticks late on average). `packet_interval_control_rule` (option, default OFF; env `PACKET_CONTROL_RULE` in `rlbot_reconstruction`) applies it in the converter, also with the next frame's controls acting inside this interval when the middle of its interval falls before this frame's time. Negative result: ground control mismatches rose (steer > 0.1 on the ground 17.8% to 18.9%, throttle 6.2% to 6.5%, handbrake 2.5% to 2.9%) and positions did not improve: the run levels of the chain lags are uncertain by a few ticks, which is as large as the gain, and the recording client's own car does not follow the rule. Kept off.

**An ordering bug fixed on the way.** The ground lookahead stepped the arena to its switch tick *before* the interval's packets were applied, so a packet whose time was earlier than the switch was applied late. The control switches are now merged with the packet times (`advance_to!`): client replay of game 1 vs the server truth, all frames, position p50 / p90 0.52 / 20.6 to 0.13 / 18.3 UU, velocity p50 5.4 to 3.3 UU/s, rotation p50 0.27 to 0.18 deg, angular velocity p50 0.049 to 0.039 rad/s. Corpus (`error_budget`, train and validation, car rows): unchanged to 0.1 UU (the 4-tick frames leave little to reorder).

## The recording client's own controls lead the server: wider ground timing shifts (2026-10-01)

`fit_ground_control_timing` chose its common shift of a car's control switches among -8..+8 ticks around the midpoint rule. Logging the chosen shifts per car (`rlbot_reconstruction`, env `SHIFT_LOG`; `GROUND_SHIFT_LOG`) on the client replay of game 1 showed that for the five cars the recorder only sees through the server the shifts are centred on 0 (p10 / p50 / p90 -7 / 0 / +4 to +10), while for the recording client's own car (the remote human) the median is +16 and the p90 +31 ticks (a range of -8..+24 left 18% at the bound): a player's own inputs reach the replay when pressed, the server's state follows later (input latency and buffering of about 130 ms here). The range is now -8..+40. Client replay of game 1 against the server truth (all frames): position p50 / p90 0.13 / 18.3 to 0.07 / 18.3 UU, velocity p50 3.3 to 1.7 UU/s, rotation p50 / p90 0.18 / 3.20 to 0.14 / 3.11 deg, angular velocity p90 0.78 to 0.68 rad/s; ground steer wrong (> 0.1) 17.8% to 13.3%, frames with a steer error above 0.5 for the remote human 802 to 383, other cars unchanged. Game 2: position p50 0.44 to 0.04 UU, velocity p50 5.3 to 0.7, ground steer 13.0%, throttle 5.5%. Corpus (`error_budget`, held-out residuals), train / validation: car velocity p90 44.1 / 45.2 to 41.0 / 41.9 UU/s, rotation p90 2.21 / 2.22 to 2.12 / 2.14 deg, angular velocity p90 0.53 / 0.54 to 0.50 / 0.51, ground-no-boost velocity p90 36.3 / 37.9 to 31.6 / 33.2; position and ball rows unchanged. Host replays unchanged (exact). So the recorder's car in a corpus replay has the same lead, and the old range cut off its fits. Not done: the shifts are per event; unfit intervals (air, near the ball, jumps) still use the midpoint rule for all cars, so a per-car median shift applied there is the next step.

**Negative results of the same step.** (1) Applying the median fitted shift of each car (last 15 informative fits, at least 5) to the intervals the ground fit does not cover did not help: client replays, ground steer wrong 13.3% to 14.8% (game 1) and 13.0% to 14.3% (game 2), steer-error frames of the remote human 383 to 379 and 425 to 409, physics unchanged; reverted. Most steer error of that car is therefore in fitted intervals or not a fixed lead. (2) Widening the jump fit's shifts from -8..+16 to -8..+40 changed nothing (has_jumped wrong 4.14% to 4.20%). (3) The airborne jump-button mismatches (12-14% of airborne frames, all truth pressed and exported released) are mostly the button held through a flip (truth air_state Dodging 1,121 of 3,500 frames, double jump 151, after the jump 210) and after the release inside RocketSim's minimum jump time (570): the replay carries only the press counters, so a held button that has no physical effect is not identifiable.

## What the exported air controls are (2026-10-01)

The exported air controls look wrong against the server's input of the same tick (pitch / yaw / roll mean |error| 0.39 / 0.31 / 0.39 on the host replay) for two reasons that are not errors of the solve. (1) The controls on a frame's state are the ones the converter applies *from* that state (the boundary-value solution for the interval to the next packet), i.e. the action taken from the state, one interval later than the server's `last_input` of the frame's own tick. (2) The solve gives one constant per interval of about four ticks, while the true inputs of bots and humans change inside it (bang-bang): the truth's own spread around its interval mean is 0.13 / 0.14 / 0.14. Against the mean of the true inputs over the next interval (host replay of game 1, airborne frames, `rlbot_reconstruction`): median |error| 0.012 / 0.010 / 0.019, within 0.1 for 61% / 66% / 61% of frames (mean 0.29 / 0.21 / 0.25, the tail being intervals the solve does not cover: flips near the ground, low flight, refused solutions). Where the solve covers the interval the per-interval action is recovered; per-tick inputs are not identifiable from packets four ticks apart (a trace of 24 consecutive frames shows exported 1.00, 0.47, 0.00, 0.48, 1.00, 0.51 against true runs of +1 and 0 that change mid-interval). The refused intervals are 'no solution' in 1,059 cases (host replay of game 1 thinned to every third frame): 80% flips at z below 150 UU (369 of 899 unique frames; flip age uniform over 0-0.6 s, rotation error of the best candidate median 10 deg, angular velocity 1.3 rad/s), then low free flight, mostly with the angular speed at the 5.5 rad/s cap. Not an early-flip or flip-direction problem; a contact with the ground by the rotating car (z 64-103 UU) or a state the scratch arena does not reproduce is the likely cause, not investigated. `scripts`: `rlbot_reconstruction` env `AIR_NOSOL=1` prints each refused interval.

**Robustness.** The default converter (all offline fits and the boundary-value solves) ran over all 120 train and validation replays without an error: 1,089 s in total with four in parallel, at most 9 s per replay.

## Fitted presses in the export (2026-10-01)

Frame records now carry `fitted_inputs` (the inferred jump and dodge presses fitted at that frame's packets, with the timeline tick they take effect, the dodge's pitch/yaw controls and pitch cancel; `air` entries record the boundary-value solve's intervals). Against the server's true presses (`rlbot_reconstruction`; a scoring bug had mixed the `air` entries into the jump statistic, giving a spurious median of +15 ticks): host replay of game 1, 576 jumps and 386 dodges, fitted minus true 0 / 0 / 0 ticks at p10 / p50 / p90 for both; client replay, 231 jumps (-3 / 0 / +1, |error| p50 / p90 1 / 4) and 299 dodges (-4 / 0 / +4, |error| 1 / 7). A jump press is now recorded only on a rising edge of the car's jump control.

## Why the near-ground flips are refused: RocketSim itself (2026-10-01)

Follow-up on the 1,059 'no solution' intervals of the air boundary-value solve (host replay of game 1, thinned to every third frame). (1) Landing is not the cause: only 9% of the refused spans touch the ground in the server truth (`air_state` 0 at any tick of the span); the minimum height over the span has p10 / p50 / p90 44 / 86 / 702 UU. (2) Relaxing the acceptance tolerances does not help: rotation / angular velocity tolerance 3 deg / 0.5 rad/s (default), 6 / 1.0, 12 / 2.0, 30 / 5 give flip-frame rotation p90 4.70 / 4.54 / 4.88 / 6.32 deg (all frames without a fresh packet 0.85 / 0.88 / 0.92 / 0.95 deg): accepting worse solutions adds interior error. (3) RocketSim with the true state and the true inputs of every tick (`rlbot_onestep`, 12-tick rollouts of both games' all cars; now works for recordings that start with fewer cars) has rotation error p90 1.56 deg and angular velocity p90 0.29 rad/s in flip frames, 1.4 / 2.4 deg in air with and without boost (p99 29-56 UU in position near contacts), 1.6-2.2 deg in landing or takeoff and near other cars; so a part of the refused intervals is the physics model's own deviation over twelve ticks, not something better controls can fix. Unchanged: the tolerances stay at 3 deg and 0.5 rad/s. Related coverage check: lowering the car chain's minimum speed (350 UU/s) to 150 recovers only 119 of the 2,285 train activations without an exact chain lag in the next twelve frames (83% to 84% fitted); link accuracy on the LAN client stays exact at low speed (|error| p90 0 in the 0-900 UU/s bin), but the missing chains are not a speed problem, so the threshold stays.

**Correction to the dodge coverage figure (diagnose_dodge_coverage, validation).** The 17% of activations without a fit are mostly not play: 2,077 of the roughly 2,650 activations with no exact chain packet in the next twelve frames occur in the `PostGoalScored` phase (cars flipping during the celebration), which is never simulated. Counting only `Active` frames: 16,703 activations, 15,330 fitted (91.8%); with exact chain packets before and after, 96% (14,238 of 14,829). What remains unfitted in play is 562 activations with no chain packet after (459 of them with chain packets before), 3% of the total. `diagnose_dodge_coverage` now prints these classes (`diagnostic:` rows).

## The recorder's lead in the real online replays, and the per-car shift (2026-10-01)

**Correction.** The corpus is real online play (ranked and private matches), so the properties first measured on the two LAN games can be checked on it without server truth. `diagnose_online_timing` (train, first 24 replays; per car actor lifetime, the median shift the ground timing fit chose, at least 20 fits) shows the same split as the LAN client in nearly every replay: a group of actors (the recording player's cars, one actor per respawn) with median shifts of +6 to +15 ticks (typically +9 to +14) against 0 to -3 for everyone else; two replays have no such group (all 0: probably recorded from a server or a spectator view). The old range of -8..+8 truncated most of them. That is the likely source of the corpus gain of the wider range (car velocity p90 -7%).

**The per-car shift helps on the corpus** (`per_car_control_shift`, now default on): the intervals the ground fit does not cover use the midpoint rule moved by the median of the car's last 15 informative fitted shifts (at least 5 so far). `error_budget`, held-out residuals, without / with, validation: car velocity p90 41.9 / 40.8 UU/s, rotation p90 2.14 / 2.08 deg, angular velocity p90 0.51 / 0.49; near the ball velocity p90 70.5 / 60.0, rotation p90 3.63 / 3.20 deg; train: velocity p90 41.0 / 39.4, rotation p90 2.12 / 2.06, near the ball 68.2 / 57.8 and 3.45 / 3.04. On the two LAN client replays it was neutral for physics and made ground steer worse (13.3% to 14.8% and 13.0% to 14.3% wrong); the corpus is 120 replays against 2, so it is on (`--no-per-car-control-shift`, env `NO_CAR_SHIFT`).

**The ball-car offset is moot at 30 fps.** The 36-99 bridged hits per replay are enough for the estimator, but with a 4-tick frame window the ball and car runs are already tight: the median hit gap is about 0-1 UU at every offset from 1 to 10 (it never crosses zero), so no offset is estimated (`none`; `0.2-0.4` where one is) and the old placement stands. The offset matters for wide windows (the 10 fps LAN client). `diagnose_online_timing` prints bridged hits, the offset and the per-car shifts per replay.

## Scoreboard, clock, events and touches against the server truth (2026-10-01)

**Protocol.** `scripts/rlbot_events_check.py <states.jsonl> <convert_replay output>` compares the converted frame records of the four remote-client replays with the RLBot recording (team scores, game time, per-player score_info, demolished timers, latest_touch). A frame's server tick is the timeline tick minus the running minimum offset of the exactly matched car packets (exact on the lag-free host replays, about +-5 ticks on the 10 fps clients). Replay-side stats are absent until first replicated, so 'equal' is reported with an absent value read as 0 as well; absent is unknown, and here the first value is always 0 (equal 99.6-100% that way).

| Field | Host replay (games 1 / 2) | Client replay (games 1 / 2) |
| --- | --- | --- |
| Team scores | exact; changes shown 0 ticks after the truth (p90 0-2) | exact; delay p50 0-10, p90 6-12 ticks |
| Game time remaining | equal on 99.9% / 97% of frames, every one of 300 changes shown with 0 ticks delay | 98.4% / 94.5%; delay p50 0, p90 10 |
| Player stats (score, goals, assists, saves, shots, demolishes) | exact, changes shown at once (p90 0-4 ticks) | exact, but shown late: score changes p50 / p90 76 / 107 ticks, shots 40 / 90, assists 32, demolishes 113 (PRI attributes replicate slowly); 3 of 23 score changes of one player never shown separately |
| Goal events (`goal_scored_on`) | 8 of 8 and 9 of 9 goals, team right, 1-4 ticks after the truth's score change | same, 3-20 ticks after |
| Touches (converter `car_hit_ball` with an impulse vs `latest_touch`) | 96.3% / 98.4% of 243 / 247 reproduced within 20 ticks, tick offset p10 / p50 / p90 0 / 0 / 1; 2.8% / 1.2% converter hits without a truth touch | 84.4% / 90.7%; offset -3 / 0 / 5; 3.1% / 5.3% extra |
| Demolitions in play (events) | all, 0 / 1-2 / 3 ticks after the truth onset | all but name mix-ups at a simultaneous pair; 3 / 7 / 16 ticks after |
| Demolished window | 360 ticks (3 s) in truth and converter (difference 0 / 0 / 2) | 354-363 ticks (-10 / -6 / 4) |

**Findings and fixes.**
1. *Demolitions were never observed.* The converter only had RocketSim's own bump detection (10 of 12 demolitions on the host replay of game 1, 8 of 12 on the client replay). The replay carries them (`TAGame.Car_TA:ReplicatedDemolishExtended`, 16 updates in game 1 with attacker and victim car, attacker PRI, both velocities; older replays `ReplicatedDemolish`; `ReplicatedDemolishGoalExplosion` for the celebration). They are now observed events (`Event::Demolish` with `source` extended / plain / goal_explosion; attacker or victim car ids are replay car actors, velocities in replay units). 29 of 60 train replays contain extended demolish events. The converter applies them (`apply_observed_demolitions`, default on): the victim is demolished for 3 s even while its replay actor is still linked. All demolitions in play of the four replays are now reproduced; the remaining differences are name mix-ups when two cars are demolished in the same tick. Demolitions during the post-goal celebration (attacker none, 3 of 17 in game 2 plus most of the rest) are in the replay and in the truth but the converter does not simulate that phase. The same event can be re-sent 200-530 ticks later in 3 of 16 cases (the victim is still demolished): merge repeats of one victim within the respawn time. A demolished car is no longer scored as a residual (753 samples on validation that compared a frozen car with a moving packet; the other residual statistics are unchanged).
2. *Duplicate goal events.* `ReplicatedScoredOnTeam` is sometimes sent again 113-135 ticks after the first (2 of 11 events in game 2). Fixed: one event per team per post-goal phase (9 events for 9 goals).
3. *Overtime clock.* In overtime the replay's `seconds_remaining` counts up from 0 (seconds of overtime played) where the server's remaining time is negative; the `overtime` flag (seen about 100 ticks after the phase starts) tells the two apart. Not normalised in the export.
4. *Replicated stats are late, not wrong.* On a client, player stats lag the truth by 30-110 ticks (up to 1.8 s) and a quick pair of increments can arrive as one. The two-sided option would be to date a stat change from an event (a shot or save at a touch, a goal at the score change); not done.
5. *Boost pickups are not checked:* the recording was made without boost pads (`include_boost_pads = false`).

Corpus: all 120 train and validation replays convert without error with the demolition handling (about 9 s at most); `error_budget` validation unchanged to the digit except 753 fewer demolished-car residuals.

## Is each event counted once? Touches, demolitions, boost pickups (2026-10-01)

**Question.** Events can come from the replay (observed) or from the simulation. Are they reported once each, and how accurate are they? Protocol: `scripts/rlbot_events_check.py` (touches, demolitions, goals) and `scripts/rlbot_pad_check.py` (pad pickups against boost jumps in the truth, since the RLBot recordings were made with `include_boost_pads = false`: that is the recording, not the replays, which have their pads) on the four remote-client replays. A correction first: the earlier touch and demolition numbers of this branch lost every event of the human with a non-ASCII name (a Windows default-encoding mismatch in the script, not a converter problem); with names fixed the touch recall below is higher.

| Event | Source | Counted twice? | Result |
| --- | --- | --- | --- |
| Goal | replay only (`goal_scored_on`) | the attribute is re-sent 113-135 ticks later (2 of 11 in game 2): fixed, one per team per post-goal phase | 9 of 9 and 8 of 8, 1-4 ticks after the truth (client 3-20) |
| Demolition | replay (`demolish`, `ReplicatedDemolishExtended`), applied to the simulation as the victim's `is_demoed` | RocketSim has `car_hit_car` with `is_demo` (corrected below): both sources reported it. The replay re-sends the same demolition 200-530 ticks later in 3 of 16 cases (`demolish` events are not merged) | all demolitions in play observed; window 3 s |
| Ball touch | simulation only (the replay has no touch event) | RocketSim reports `car_hit_ball` every tick of a contact (520 events for 243 true touches on one replay) and repeats the extra impulse (up to 14 per touch). New: `ConvertedFrame::touches` / `touches` in the record, one per contact (first tick, no event of that car for 2 ticks before) | host: 232 of 243 (95.5%) and 240 of 247 (97.2%) reproduced, 0 surplus, tick offset 0 / 0 / 1; client: 187 (77.0%) and 221 (89.5%), surplus 9 (4.6%) and 39 (15.0%), offset -2 / 0 / 3-5 |
| Boost pickup | replay (`pad_pickups`) for the state; RocketSim's pickups are blocked offline | the replay re-announces earlier pickups (the pad's counter value again, with its instigator) at resets: 189 of 646 records on game 1's host replay. New: `PadPickup::repeat`, 457 new records. The two or three simulated `car_pickup_boost` events that still leaked are now removed whenever the pads are blocked | see below |

**Boost pickups against the truth (boost jumps above 5, kickoff and respawn resets excluded; no pad truth exists).** New replay records (repeat false): 457 / 424 / 491 / 463 against 383 / 383 / 407 / 407 truth jumps (host and client of games 1 and 2). One-to-one matching on car and time (+-40 ticks), after merging repeats: 352 of 383 and 342 of 383 (game 1), 302 and 354 of 407 (game 2) truth jumps have a replay record, tick offset 0 / 1 / 3 on hosts and 2 / 7 / 13 on clients. Unmatched truth pickups are mostly small pads (24-82 against 7-23 big). Surplus replay records (76-181) are only partly explained: 17-38 are cars already at 88 or more boost (a pickup with no visible jump). The rest cannot be judged without pad data, and attribution of the car was not verified (concurrent pickups by two cars get crossed in a time-only match). To score pickups properly the recorder needs `include_boost_pads = true`.

**What is not counted twice and what still is.** In the export, an observed event appears once (`events`, `pad_pickups` with `repeat` false, `demolish` merging repeats is left to the consumer), and a simulated one appears once in `touches`. The raw `simulated_events` stream still holds the per-tick `car_hit_ball` records (kept as the simulation reports them). Do not combine `simulated_events` hits with `touches`.

## Correction: RocketSim does report demolitions; the simulated ones were duplicates and half invented (2026-10-01)

The previous section said RocketSim has no demolition event. That was wrong: `CarHitCarEvent` has `is_demo` (the victim was demolished), and the export carries it as `car_hit_car` with `is_demo`. So a demolition was reported by both sources. On the four remote-client replays the simulation reported 11 / 9 / 7 / 6 demolitions (host and client of games 1 and 2) against the truth: 10 / 7 / 5 / 4 matched a true demolition (83% recall in play, tick offset 0 on the host replays) and 1-2 per replay did not exist; 10 / 6 / 5 / 4 of them coincided with an observed event of the same victim within 30 ticks (one demolition, two reports). Over the train split (`count_demolitions`): 276 observed demolitions in play, 254 simulated, only 139 of those with an observed event of the same victim; 27 of 57 replays that have no demolition in the replay at all (only goal explosions, no demolition counters) still got 1-3 simulated demolitions each. The simulated ones also put the car in the demolished state for 3 s, which was an invented state. Fix: with `apply_observed_demolitions` the arena's demolition rule is switched off (`disable_simulated_demolitions`, default on, `DemoMode::Disabled`), so the observed demolitions are the only ones: no `is_demo` in `car_hit_car` any more (0 of 276 on train), the demolished state follows the replay (game 1: 12 of 12 onsets in the host replay, window 360 ticks), and the corpus is unchanged within noise (`error_budget` validation: near other car position p99 30.8 to 29.3 UU, the rest identical). Observed demolitions at a frame an evaluator withholds are no longer applied. Bumps without a demolition are still simulated. Provenance: demolition = replay evidence only.

## Touches from the ball packets (2026-10-01)

**Idea.** Two fresh ball packets a few ticks apart are exact server states. If nothing touched the ball in between, RocketSim rolling the ball alone from the first packet (`ball_evidence::ball_intervals`, no cars in the arena) reproduces the second; a velocity it cannot reach at any elapsed tick the packet lags allow (widened to the frame windows when the lags disagree) is a contact. That is replay evidence for a touch, independent of the car simulation.

**Detection** (`scripts/rlbot_ball_evidence.py`, intervals labelled by the server's `latest_touch` changes, packet ticks from exact ball position matches; game 1, host / client): free flight 7,056 / 2,794 intervals with 140 / 127 true touches, quiet-interval residual p50 / p99 0.01 / 0.02 UU/s (floor and walls on the client: p99 4 UU/s), weakest 1% of touches 46-67 UU/s (near surfaces 113-153): every threshold from 2 to 40 UU/s finds all of them (`CONTACT_VELOCITY_THRESHOLD` = 10). Near the floor, ceiling and walls: 81 / 53 touches, all found. The 10-23 'false alarms' per replay all have a car within 130-175 UU (centre to centre) of the ball: real contacts that `latest_touch` did not register, with changes up to 1,000 UU/s.

**Export.** `ball_contacts` on the frame that ends the interval (`contacts_from_ball_packets`, default on, offline): the interval's two physical ticks, the estimated contact tick and the car, placed with the cars' exported poses: the first tick the nearest car's hitbox reaches the ball's no-touch path (gap 150 UU or more: no car, `car_slot` null), the gap, the velocity residual, and whether a simulated touch falls in the same interval. Runtime +0.5 s per replay (all 120 train and validation replays convert, at most 9 s).

**Against the truth** (`rlbot_events_check.py`, games 1 / 2; host, client): contact intervals 243 / 249, 221 / 227; truth touches inside some contact interval 96.7% / 97.6% (host), 93.8% / 88.3% (client); the car right in 98.6% / 98.2% (host), 96.5% / 96.5% (client); contact tick minus truth p10 / p50 / p90 -1 / 0 / 1 (host), -3 / 0 / 3 (client); intervals holding a truth touch 242 of 243 and 242 of 249 (host), 209 of 221 and 208 of 227 (client), the rest being real unregistered contacts. Intervals hold two or more true touches in 12-18% of cases (a dribble inside four ticks): the evidence is per interval, the simulated `touches` separate them. For comparison the simulated touches cover 98.8% / 100% (host) and 86% / 92% (client) of the truth touches, with 5-15% surplus on a client; the union of the two covers 99.6% / 100% (host) and 98.8% / 98.4% (client). Use `ball_contacts` as the touch evidence (it does not invent touches), and `touches` for ticks inside an interval and for double touches; do not add them.

## Boost pickups with a checked car, and repeated demolitions (2026-10-01)

**The earlier pad numbers were mostly a script bug.** The boost-pickup match of the previous section lost the human with a non-ASCII name and merged records wrongly. With the replay's instigator taken as reported, `scripts/rlbot_pad_check.py` finds a record with the right car within 40 ticks for 382 of 383 truth pickups (games 1, host and client), 402 / 407 (game 2 host) and 397 / 407 (game 2 client); tick offset p10 / p50 / p90 0 / 1 / 3 on the hosts, 1-2 / 5-6 / 11-13 on the clients. The 34-48 surplus records per replay are what full-boost pickups look like (no boost jump; 17-38 of them have a car at 88 or more boost in the truth).

**Pad names, not actor ids, identify a pad.** A pad's actor is created again after each goal (184 actors for 34 pads in game 1) but keeps its name (`VehiclePickup_Boost_TA_14`). The converter now learns each name's index in RocketSim's pad list from the whole replay (nearest pad to the instigator at every non-repeat pickup of that name; at least two votes and twice the runner-up), instead of from the first pickup of each actor. Pads matched to an index: 432 / 457 to 457 / 457 (game 1 host), 418 to 424 of 424, 465 to 491 of 491, 453 to 463 of 463.

**`boost_pickups` in the export** (`ConvertedFrame::boost_pickups`, new pickups only): pad index and size, the instigator's slot, the instigator's closest horizontal distance to the pad (straight lines between the exported poses of the last eight frames), `verified` when it is within the trigger radius (144 small, 208 big) plus 60 UU for the car's size, a `suggested_car_slot` when another car's path reaches the pad instead, and the tick of closest approach. Verified: 416 of 457, 414 of 424, 445 of 491, 443 of 463 on the four replays (before the name mapping only 174 of 457 on the host replay of game 1 passed, the rest were pad mix-ups); 94-98% on six train replays. A different car's path reaches the pad in 1-9 cases per replay and the instigator was the right one in none that mattered (truth coverage is the same with the suggestion).

**Repeated demolitions.** `Event::Demolish` has `repeat`: the same victim car actor reported again within 3 s of its last report (a car cannot be demolished while demolished). 3 of 16 on game 1, 0 of 19 on game 2 (the repeats there are later). The converter ignores repeats and any report whose victim actor is no longer in the frame, so a respawned car is never demolished a second time. Train: 276 reported, 224 not repeats.

## The scoreboard lifecycle (2026-10-01)

**Rules checked against the server** (`scripts/rlbot_clock_trace.py`, the four remote-client games, 18 kickoffs, one game ending at 0 and one in overtime).
- The clock stays at 5:00 until the first touch of the kickoff and starts at that touch: the server's kickoff phase (2) ends at the first touch (`latest_touch`) in all 18 kickoffs, with 0 ticks difference. The 5-second fallback was never exercised (every kickoff was touched after 1.9 s), so it is unverified.
- It then counts down in real time (host replay: the shown integer equals the ceiling of the server's clock in all but 8 of 11,863 frames, each a single frame at a change), is frozen during the goal pause, the replay and the next countdown and kickoff (the server's value does not move over those phases, e.g. 263.83 throughout), and resumes at the next first touch.
- At 0 the server clock keeps going to -1.0 over the next second and then holds there while the ball is in the air (an engine detail); the replay shows 0 throughout, and nothing in `game_state` says the game is waiting.
- The decision comes when the ball touches the ground: game 1 ends (phase 7) 0.37 s after expiry, at the ball's floor contact, and the host replay's last frame is 3 ticks before that. Game 2's expiry ended by a goal in the air (4-4, so overtime).
- Overtime: the replay counts up from 0 from the overtime kickoff touch (the integer is again a ceiling); the server's clock is negative there (-1.0, -4.0, ...) and keeps its post-expiry offset.

**What the export lacked.** Only the replay's integer clock, a two-valued game state (`Active` covers the kickoff wait, running time and the wait after expiry) and an overtime flag. `scoreboard` (new, `src/scoreboard.rs`) reconstructs the lifecycle: `period` (regulation / overtime), `clock_state` (`pregame`, `countdown`, `kickoff` = clock held, `running`, `expired` = at 0 waiting for the ball, `decided` = the ball has touched the ground or the replay ends at the final whistle, `goal_pause`), the regulation clock `seconds_remaining` and the overtime `overtime_seconds` as fractional seconds. Within a running stretch the true clock is a line in the 120 Hz timeline and every change of the integer brackets one tick between two frames, so the intersection of the brackets fits it (a few ticks); the clock is held at its frozen value until the line falls below it, so the start of running time at the touch comes from the fit and not from a one-second-late first change. The floor contact after expiry is the first fresh ball packet below 100 UU or the simulation's first floor contact of the ball.

**Accuracy against the server** (`scripts/rlbot_clock_check.py`; regulation `seconds_remaining` against the server's clock clamped at 0): host replays |error| p50 / p99 / max 0.006 / 0.029 / 0.03 s (game 1) and 0.008 / 0.024 / 0.07 s (game 2), 99.9-100% of frames within 0.05 s; client replays 0.009 / 0.119 / 0.13 s and 0.011 / 0.118 / 0.55 s (a client's clock arrives late by the replication delay, signed mean +0.017 s). Overtime seconds against the time since the overtime kickoff touch: 0.016-0.017 s p50, max 0.03 s. Phase against the server's `match_phase` (running, countdown, kickoff, goal pause, expired): 98.3-99.1% of frames; the remainder are the server's pre-game phase 6 (the first countdown, 119 frames on game 1) and single frames at a boundary. On the real online replays (no server data; `check_scoreboard` on train and validation): the integer the replay shows equals the ceiling of the reconstruction in 98.4% and 98.6% of 540,215 and 636,600 running frames and differs by one in the rest (frames at a change), never by more.

## Car/ball tick alignment from the contact intervals: no gain (2026-10-01)

**What was tried** (`contact_alignment`, `align_contacts`, default off; `--align-contacts`, env `ALIGN_CONTACTS`, `ALIGN_DEBUG`). Offline, two passes. Pass 1 is the normal conversion (its `ball_contacts` give the interval and the car). For each contact the converter's own exported car and ball at the last frame before the hit (at most 12 ticks before it) are copied into a scratch arena with the car placed -3..+3 ticks later or earlier than the ball, the hit is simulated to the second ball packet and the ball velocity is compared with it. A shift is kept when it lowers the residual by 30 UU/s and reproduces the packet within 100 UU/s, and moves the lag of the last car packet before the hit; pass 2 runs with those lags.

**Result: neutral to slightly worse.** The scratch simulation reproduces the observed post-hit ball velocity within 100 UU/s for only 11% of 216 contacts unshifted (20% when the start frame is 1-4 ticks before the hit, 5-7% at 9-12) and 19% for the best shift, so timing is not what separates a simulated hit from the real one; 16 shifts passed the criteria. LAN client replays (games 1 / 2): touches reproduced 77.0% / 89.5% to 79.0% / 89.5%, car-ball relative position p90 21.3 / 18.8 to 21.2 / 18.8 UU, ball velocity near cars p90 11 / 20 UU/s unchanged. A looser first version (shifts accepted whenever they helped by 30 UU/s, 106 of 209 contacts) raised touch recall (84.4% / 92.3%) but worsened the car-ball position p90 (25.5 vs 21.3 UU) and the ball velocity near cars in game 2 (60 vs 20 UU/s p90): it bought simulated touches by fitting noise. Corpus (`error_budget` validation, 30 fps): ball near a car velocity p90 7.1 to 7.3 UU/s, car near the ball position p99 20.1 to 22.1 UU, nothing better. Left off.

**Why, and what is left.** The earlier contact study found the contact reproduced for 65% of touches with the car packet at the ball packet's tick, and a shift of the car's start position of a few UU along its travel (84%) explains more than the timing grid (78%). The geometry and the car's controls in the last ticks before the contact (steering, throttle, boost, pitch while a car carries the ball) set the impulse; a one-tick timing fit cannot make up for car state errors of a few UU. If hits at the control level matter, the next experiment is a joint fit of the car's start position and controls over the last frames before a contact against the ball's post-hit velocity, with the same held-out checks, not a timing shift.

## The ceiling of hit reproduction, and how sensitive it is (2026-10-01)

**Question.** Is RocketSim's contact model itself the limit on reproducing a ball touch? `rlbot_contact_ceiling <states.jsonl> <collision_meshes>` starts RocketSim from the exact true state of the ball and all cars 1, 2, 4 or 8 ticks before each true touch (`latest_touch` change; 243 and 247 touches in games 1 and 2), drives it with the true inputs of every tick (the next-packet alignment, converter default hit-impulse handling) and compares the ball with the truth 0, 1, 4 and 12 ticks after the touch. Raw outputs: `target/ceiling_outputs/` (ignored).

**Ceiling from the exact state** (181 and 189 touches with no other touch within 12 ticks, lead 1 tick): ball velocity error below 25 / 50 / 100 UU/s for 95 / 95 / 97% (game 1) and 95 / 95 / 96% (game 2), p50 / p90 0.8 / 2.9 and 0.7 / 4.7 UU/s, ball position error 0 UU at p50 and under 1 UU at p90 through 12 ticks. The toucher's `CarHitBall` comes on the true tick in 100% / 99.6% of cases at a lead of 1 or 2 ticks, 94% at 4 and 87% at 8 (the rest are the car grazing the ball before the recorded touch). Splits at lead 1 (below 50 UU/s): grounded cars 97%, airborne 95%, ball in free air 95-98%, near floor or walls 91-95%, bots 96%, humans 88% (n 25 and 24), carries (the same car again within 40 ticks) 95% and 89% (n 44, 37); at lead 4 carries fall to 80-81% with p90 330-1,020 UU/s. So RocketSim's contact model is not the limit; the hit impulse and geometry are reproduced.

**The sensitivity is the finding.** Jitter the toucher's start state only (5 Gaussian draws per touch), share of touches below 25 / 50 / 100 UU/s 12 ticks after the touch (game 1, lead 1): exact 95 / 95 / 98; position sigma 1 UU 63 / 79 / 89; 3 UU 24 / 47 / 74; 6 UU 8 / 21 / 46; velocity sigma 10 UU/s 92 / 94 / 97; 30 UU/s 53 / 84 / 95 (game 2 and lead 4 alike). The median error is 13-17 UU/s at 1 UU, 50-65 at 3 UU and 105-135 at 6 UU; carries are more sensitive. The outcome of a touch is set by the car's position at the contact to a few UU; velocity error matters far less. Against that, the converter's car position error over a packet interval is p50 0.5 and p90 18-20 UU on the LAN client replays and the exported states used by the first alignment attempt reproduced 11% of contacts within 100 UU/s.

**Consequence for the next experiment.** The place to improve hits is the car's position at the contact tick, which is known exactly at its packets (0.01 UU) and drifts between them with the unobserved controls and the uncertain packet tick. Two readings: (a) start from the exact car packet when one lies within about three ticks before the contact and fit only the timing (the earlier study found 78% reproduced that way against 65% unfitted, with the car packet at the ball packet's tick); (b) for contacts with a car packet further away, fit the controls of the last ticks to land the car on the packet after the contact as well. Both need a held-out check (the next car packet) because a position fit can match any hit.

## Car/ball alignment from the contacts, redone: it works (2026-10-01)

**Correction of the earlier 'no gain' section.** The ceiling check (previous section) showed RocketSim reproduces 95% of touches from the exact state, so the 11% reproduced by my scratch fit pointed at the scratch setup. Two defects: the car started from the converter's *exported* state and controls (the exported controls include air controls, fitted jumps and flags that do not describe the driver's input at the packet), and from a frame up to 12 ticks before the hit. On the lag-free host replay of game 1, where every packet is exact and the lead is 1-2 ticks, that setup reproduced 14% of 159 contacts within 50 UU/s; the setup of the earlier contact study (the packet's physics on a default car, grounded when low, the observed throttle, steer, handbrake and boost as controls) reproduces 77% unshifted and 80% with the best shift, as in that study.

**What `contact_alignment` does now.** For each ball contact with a car packet at most four ticks before it: car from that packet (exact physics), ball from the packet before the contact, the car placed -3..+3 ticks off, the hit simulated to the next ball packet. A shift counts only when it reproduces the ball's velocity to 100 UU/s and beats the unshifted one by 30 UU/s (a fit that needs no shift votes for 0). The car and the ball were placed by chains whose levels are uncertain *per run*, so the votes are per run: a car's exact-chain run (`PacketLags::car_runs`, every packet of a run shares one start) moves by the median of its contacts' shifts when they agree within one tick, as far as its feasible range `[lo, hi]` allows, and the converter runs again with the moved lags. Shifting only the packet before the hit (the first run-less version) broke the run's consistency and made the car position near the ball worse on the corpus.

**Evidence.** Reproduction of the contacts on the LAN client replays within 50 UU/s: 30% / 37% unshifted (host replay: 77%) to 66% / 65% with the best shift (games 1 / 2; 93 and 98 contacts with a car packet within four ticks). Against the truth (shifted contacts of game 1, 39 with truth ticks for both packets): the chosen shift equals the true correction in 69% and is within one tick in 95%; the remaining relative timing error falls from p50 / p90 1 / 3 ticks to 0 / 1. Touches reproduced (one-to-one, +-20 ticks): 77.0% to 82.3% (game 1), 89.5% to 91.1% (game 2), surplus unchanged (4.6% to 3.4%, 15.0% to 14.8%). `rlbot_reconstruction`, client: car-ball relative position p50 / p90 7.6 / 21.3 to 5.9 / 18.5 UU (game 1) and 5.7 / 18.8 to 2.5 / 18.6 UU (game 2), ball velocity near cars p90 20 to 15 UU/s (game 2), cars unchanged (position p90 18.3 / 17.9 to 18.2 / 18.0). Corpus (`error_budget` validation, 30 fps): ball near a car velocity p90 7.1 to 6.6 UU/s and position p90 0.7 to 0.6, ball position p99 17.1 to 17.0; car near the ball position p99 20.1 to 19.9, velocity p90 60.2 to 59.8, rotation p99 10.0 to 9.9; nothing worse. 21 and 23 car runs were moved on the LAN clients.

**Default and cost.** On (`align_contacts`; `--no-align-contacts`, env `NO_ALIGN_CONTACTS`, `ALIGN_DEBUG`). It runs the conversion twice (a client replay takes about 27 s instead of 6 s), only on replays with inferred lags (a lag-free host replay is skipped). The remaining limit is the car's geometry at the contact: the position sensitivity measured by the ceiling check (1 UU of position error costs a third of the reproducible hits) is untouched, and about half the contacts have no car packet within four ticks.

## Code review before the freeze, and what a conversion costs (2026-10-01)

**Review.** A read-only reviewer went through the modules added this session; each finding was checked against the code (and RocketSim's source) before acting.
Fixed: (1) `contact_alignment` set the car's controls *before* `set_car_state`, which replaces the whole state (RocketSim `Car::set_state` copies `CarState`, controls included), so the scratch car coasted; the controls are now set after it (the fidelity did not change much: 31% unshifted within 50 UU/s against 30% before). (2) **The exported boost pad state was wrong in every frame**: with `block_sim_pad_pickups` (default) every pad was held at cooldown 20 s before the step and never restored, so every pad not picked up in that frame was exported as inactive. The converter now tracks the true pad cooldowns (full cooldown at an observed pickup, 0 at a reset, decaying in real time) and writes them into the arena before each export (83% of sampled pad states are active on the host replay of game 1). (3) The demolition repeat window is 5 s (the replay re-sends a demolition up to 4.4 s later; the comment said so, the code used 3 s). (4) Adjacent exact-chain runs of a car share their boundary packet: the alignment votes are now ordered (a deterministic result) and a run moves only the packets that `car_run_of` assigns to it. (5) `per_car_control_shift` looked at frames within four of the current one while the shift can be 40 ticks; the window now grows with the shift. (6) A victim slot is demolished only when the car actor of this frame belongs to the slot's lifetime. (7) The overtime clock holds at 0 and `kickoff` when the integer never changed. (8) `GROUND_SHIFT_LOG` grew without bound: it is only kept when a diagnostic tool switches it on. (9) `evaluate_corpus` keeps RocketSim's own demolition rule on and the contact alignment off in its masked (causal) conversion, which has no later information. (10) A doc comment had landed on the wrong function.
Not changed, noted: the `jumping` edge of a recorded jump press starts from the current frame's jump control (a press in a frame whose control is already set is not recorded in `fitted_inputs`); `pad_reported` and `demolished_at` are keyed by actor id and not cleared when an id is recycled; the per-frame median lag of a car does not follow a moved run (only used when a car has no chain of its own).

**Where the time goes** (`profile_conversion`, a 3v3 validation replay of 11,023 frames, one process, wall time; the figures in the second column are after the fixes below). Base conversion 19.3 s: the simulation itself 0.6 s; the air boundary-value solve about 12 s (the flip scans try 13 start offsets, each with a Levenberg-Marquardt solve in RocketSim); the ground timing fit about 4 s (49 candidate shifts since the range was widened to -8..+40, from 17); the other fits about 3 s. Contact alignment added a complete second conversion (19 s) plus its fits, which made the default 42 s. Fixes: the alignment's first pass reuses the lags and leaves out the expensive fits (it only needs the contacts and rough car poses), and the flip scans try offsets up to +-3 ticks instead of +-6 (interior flip rotation p90 4.70 to 4.73 deg on the thinned host replay, ball and air rows unchanged). Result: 3v3 11.4 s (from 42.9 s), 1v1 3.3 s (from 13.2 s); over the 120 train and validation replays with four converting in parallel the mean is 9 s and the maximum 24 s per replay (was 25 s and 65 s). Left: the air solve is still half of the base cost, and the ground shift search could be coarse-to-fine.


## Parquet columns and record tables (2026-10-01)

The direct Rust Parquet export (`convert_replay x.replay out.parquet`) now exposes the scoreboard and the list-valued records as typed columns. Nothing existing changed: the 22 original columns keep their names, types and positions (`columnar_version` stays 1), the new columns are appended, and `python/verify_direct_parquet.py` still passes against the JSONL of the same replay. `frame_json` remains the complete record (it also carries `packet_lag_ticks` and the observed fields with their freshness).

**Design.** The main file has one row per frame with fixed-width lists, which cannot hold a variable number of records per frame, and nullable fixed-size lists do not survive a PyArrow read (Parquet drops the child values of a null list). So the records are written as seven small side files next to the main one, `<stem>.<table>.parquet` (`out.parquet` gives `out.touches.parquet`, ...), each with a `frame` column (the replay frame, join to the main file's `frame`) and the schema metadata `table` and `columnar_version`. They are written in the same pass, buffered 512 rows at a time like the main file (row groups of at most 512 rows), always created (a replay without such records gives a valid empty file), and skipped with `--no-event-tables` (library: `parquet_export::write_parquet_with_tables`, `write_parquet` is unchanged and writes the main file only). Strings are dictionary-encoded `Utf8` (`dictionary<uint8, string>`); the strings, not the integer keys, are the contract.

**Null semantics.** Null means unknown or not applicable, never zero or false: a slot or pad index that could not be matched, a velocity/cancel that does not exist for that record kind, a clock value the reconstruction does not have. The older `seconds_remaining` column keeps its NaN convention; the new scoreboard columns use real nulls.

| Where | Column (type) | Meaning |
|---|---|---|
| main | `scoreboard_period` (string, null when the frame has no scoreboard) | `regulation`, `overtime` |
| main | `scoreboard_clock_state` (string) | `pregame`, `countdown`, `kickoff` (clock held until the first touch), `running`, `expired`, `decided`, `goal_pause`, `other` (`scoreboard.rs`) |
| main | `scoreboard_seconds_remaining` (f32, nullable) | regulation clock in seconds, fractional while running, 0 after expiry |
| main | `scoreboard_overtime_seconds` (f32, nullable) | overtime time played in seconds (null in regulation; an overtime 0.0 is a value) |
| `touches` | `frame` u32, `car_slot` u32, `tick` u64, `contact_point` fixed list f32[3] | simulated touches (first tick of a contact), tick on the 120 Hz replay timeline |
| `ball_contacts` | `frame` u32 (frame whose packet ends the interval), `frame_a` u32, `tick`, `tick_from`, `tick_to` u64, `car_slot` u32 null, `gap_uu` f32 null, `velocity_residual` f32, `simulated_touch` bool | contacts found from the ball packets; null car when no car was within 150 UU |
| `boost_pickups` | `frame`, `pad_index` u32 null, `pad_actor_id` i32, `is_big` bool null, `car_slot` u32 null, `verified` bool, `distance_uu` f32 null, `suggested_car_slot` u32 null, `tick` u64 | new pad pickups checked against the cars' paths; the replay's own records are in `pad_pickups` |
| `fitted_inputs` | `frame`, `slot` u32, `kind` (`jump`, `dodge`), `tick` u64, `activation_frame` u32 null, `pitch`, `yaw`, `cancel` f32 null | inferred jump and dodge presses (not observed); the last four are null for a jump |
| `packet_lags` | `frame`, `actor_id` i32 null, `ticks` u64, `source` (`chain`, `frame_median`, `default`) | inferred packet lags applied to the frame's fresh packets (empty when `infer_packet_lag` is off) |
| `events` | `frame`, `kind` (`goal_scored_on`, `demolish`), `team` u8 null, `source` (`extended`, `plain`, `goal_explosion`) null, `attacker_car`, `victim_car`, `attacker_pri` i32 null, `self_demolish` bool null, `attacker_velocity_{x,y,z}`, `victim_velocity_{x,y,z}` f32 null, `repeat` bool null | observed events; `team` is set only for goals, the rest only for demolitions. Cars are replay actor ids, velocities are in replay units. Count only demolitions with `repeat` false |
| `pad_pickups` | `frame`, `pad_actor_id` i32, `pad_actor_name` string null, `instigator_car_id` i32 null, `picked_up` u8, `repeat` bool | observed pad records; count only `repeat` false (`picked_up` 255 marks available) |

**Check.** One validation 1v1 replay (8,126 frames, converted in 7 s in release): the table rows (touches 140, ball_contacts 177, boost_pickups 147, fitted_inputs 1,749, packet_lags 9,922, events 12, pad_pickups 1,102) equal the counts in the JSONL of the same conversion, and the scoreboard columns, the event kinds with `repeat`, and the fitted input kinds and ticks match the JSONL record by record (PyArrow 4-column check). Unit tests (`cargo test --lib parquet`) cover the column order, null handling per record kind, row-group bounds and the empty-table schema.
