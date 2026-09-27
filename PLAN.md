# Replay to RocketSim plan

Last updated: 2026-09-27. Status: Phase 0 complete; Phases 1 and 3 in progress; preliminary Phase 5 JSONL/Python path implemented.

Next action: investigate one-step car tail errors, remaining transient car actors, jump/aerial semantics and hitbox mapping, then improve reconstruction against the masked-observation baseline. Use `validation` to check chosen changes; keep `test` untouched until the final freeze.

## Goal and scope

Convert Rocket League replay network frames into timestamp-aligned RocketSim game states, plus match information such as players, teams, score, clock, events, and provenance. Replay observations are the primary evidence. RocketSim supplies physically plausible states between observations and estimates of fields absent from the replay. Expose a Rust API and a stable, Python-readable serialized dataset.

The first target is standard soccar in the supplied 1v1, 2v2, and 3v3 corpus. Other modes, mutators, unusual arenas, and replay versions must be detected and reported; add support only after the soccar pipeline is measured. Do not consult the existing `rlgym-tools` Python converter or `rust-carball` implementation until the user provides them for comparison.

## Current repository and data

- `Cargo.toml` pins parser and simulator dependencies; `src/lib.rs` exposes parsing, observations, conversion, audit, and serialization modules.
- `replays/` is ignored by `.gitignore`. It contains 180 `.replay` files: 20 per game size in each of `train`, `validation`, and `test` (60 per split; about 212 MB total).
- The split names are an evaluation contract: inspect and optimize on `train`; use `validation` to decide whether changes generalize; run `test` only for a final, frozen assessment. Keep corpus paths configurable, and never commit replay contents or generated datasets.
- The user added `collision_meshes/` with `.cmf` files for soccar, hoops, and dropshot. It is ignored as a local asset directory. RocketSim's `init_from_default` expects `./collision_meshes/` when run from the repository root.
- The installed Rust compiler is 1.97.1. `boxcars` 0.11.5 and native `rocketsim` commit `79f4d22fc533614d540b88457a96352c17da6b73` are pinned in `Cargo.toml`/`Cargo.lock` and compile locally.

## Crate findings and dependency decision

### `boxcars`

Use a pinned `boxcars` release (the inspected docs are 0.11.5) and `ParserBuilder::new(bytes).must_parse_network_data().parse()` so a header-only success never masquerades as a converted replay. Record parse errors with replay ID and version; expose optional header-only diagnostics separately.

`Replay` supplies header properties, object and name tables, keyframes, tick marks, and optional `NetworkFrames`. Each `Frame` has `time`, `delta`, actor creations, deletions, and attribute updates. `UpdatedAttribute.object_id` resolves through `Replay.objects` to a property name; actor IDs identify instances, not players. `Attribute` contains `RigidBody`, `ReplicatedBoost`, `ActiveActor`, `Pickup`, numeric and string values, and event variants. `RigidBody` has location and quaternion, while linear and angular velocity are `Option` values. A missing value means **unobserved**, not zero. Header properties can contain duplicate keys, so preserve their ordered representation until their semantics are established.

### `rocketsim`

Use the native Rust `rocketsim` crate from the upstream [RocketSim `v3-rust` branch](https://github.com/ZealanL/RocketSim/tree/v3-rust), pinned to a tested commit in `Cargo.lock` (local inspected checkout: `79f4d22fc533614d540b88457a96352c17da6b73`, 2026-08-26). This is a working dependency choice pending the user's clarification about the crate name. The separately published `rocketsim_rs` crate provides C++ bindings and has a different API.

The inspected native API has `Arena::new/new_with_config`, `add_car`, `step_tick`, per-object state setters and getters, `get_arena_state`, boost-pad access, and per-tick `ArenaEvent` output. `ArenaState` holds tick count, cars, ball, pads, and optional tiles. `CarState` has physics, controls, boost, jump/flip/demo and timing fields; `BallState` has physics and mode-specific fields. RocketSim state structs do not derive Serde, so serialization needs explicit project-owned transfer types. `init(path, silent)` loads collision meshes and should be called once before creating arenas. Inspect and test the actual pinned revision at implementation time; the native API is still evolving.

### Source references

- [boxcars parser and error behavior](https://github.com/nickbabcock/boxcars#quick-start)
- [boxcars `Replay`](https://docs.rs/boxcars/latest/boxcars/struct.Replay.html), [`Frame`](https://docs.rs/boxcars/latest/boxcars/struct.Frame.html), [`UpdatedAttribute`](https://docs.rs/boxcars/latest/boxcars/struct.UpdatedAttribute.html), [`Attribute`](https://docs.rs/boxcars/latest/boxcars/enum.Attribute.html), [`RigidBody`](https://docs.rs/boxcars/latest/boxcars/struct.RigidBody.html)
- [native RocketSim source](https://github.com/ZealanL/RocketSim/tree/v3-rust/rocketsim/src) and [RocketSim accuracy caveat](https://github.com/ZealanL/RocketSim#accuracy)

## Output contract

Provide `convert_bytes(&[u8], &ConvertOptions) -> Result<ConversionOutput, ConvertError>` and a streaming or callback variant for large batches. File IO belongs in a small CLI wrapper. `ConversionOutput` contains replay metadata, an ordered frame sequence, and diagnostics. Each frame contains replay frame index/time/delta, matched RocketSim tick, an `ArenaState` snapshot or explicit unavailable status, scoreboard and clock, stable player/team identities, observed replay events, simulated events with source and tick, and per-field provenance/quality. A missing value remains optional; inference must not be presented as observed truth.

Define a versioned, project-owned transfer schema separate from RocketSim's structs. Include schema version, dependency revisions, replay fingerprint, units, coordinate conventions, tick rate, options, and any unsupported-feature diagnostics. Preserve enough car/ball/pad state to rebuild a useful RocketSim arena when possible; document fields that cannot be restored exactly. Offer newline-delimited JSON for inspection and a compact columnar format (Arrow IPC or Parquet, selected after a size/read-speed prototype) for Python through `pyarrow`. Provide a small Python loader returning typed metadata plus frame columns/NumPy arrays, with a round-trip test. Avoid encoding Rust memory layouts or `Debug` output as a storage format.

## Reconstruction model

1. **Replay inventory and field map.** On `train` only, parse every file strictly and report game modes, versions, frame rates, duration, actor/property names and frequencies, missing velocities, parse failures, and examples of goals, demos, pickups, kickoff and overtime. Build a documented mapping from network class/property names and `boxcars::Attribute` variants to typed observations. Unknown attributes go to diagnostics rather than being silently interpreted.
2. **Actor graph and identity.** Apply create/update/delete in frame order. Track actor ID lifetimes, ownership links among player replication info, car, components, team, ball, game event and pads; handle ID reuse, delayed ownership, spectators, joins and leaves. Assign stable player IDs using replay identity fields and keep actor IDs as transient references. Reconcile header metadata with network timeline without leaking final score into earlier frames.
3. **Time and coordinate calibration.** Verify replay `time`/`delta`, pauses and non-monotonic cases. Map elapsed replay time to 120 Hz ticks with a documented rounding rule and carry residual timing error; keep the original timestamp. Verify axis signs, quaternion order/handedness, angular velocity units, and boost encoding using observed trajectories. Add focused unit tests for transforms before simulation.
4. **Observed baseline.** Build a replay-frame state accumulator first: ball/car transform and velocity, boost, flags, pads, scoreboard and events. Track freshness separately for every property and subfield. A new rigid-body packet may update position/rotation while omitting either velocity. Support an observation-only baseline that forward-fills only truly persistent values and marks stale physical values as stale.
5. **Simulation bridge.** Initialize a seeded arena for the detected mode and car hitbox, map replay cars to arena indices, step each intermediate tick, and capture events. Apply fresh replay observations at their mapped tick as corrections; preserve simulated values for unobserved fields. Resolve conflicting observations deterministically and log correction magnitudes. Recreate or reset only at real lifecycle boundaries (goal freeze, kickoff, demolition/respawn, actor changes), preserving stable identities. Do not infer goals or scoreboard solely from simulated ball crossings.
6. **Missing state estimation.** Start with conservative controls and short-horizon simulation. Derive discrete actions from replicated component state, infer ground controls and aerial input only when evidence supports them, and attach confidence. Estimate boost-pad cooldown from observed pickups plus RocketSim timing; reconcile any replay pad state. Infer jump/flip/ground/contact timers from observation history and simulation, reset uncertainty at authoritative events, and avoid claiming exact recovery of unrecorded inputs. Consider bidirectional interpolation or smoothing for offline ML output, with a separate causal mode, only if validation metrics improve and future-data use is labeled.
7. **Match metadata.** Derive score, match clock, overtime, kickoff phase, players, teams, and goals from their network/header fields and event timeline. Distinguish final header totals from per-frame score. Keep replay-authored and simulator-generated events in separate streams; attach source, replay frame and sim tick.

## Accuracy and test protocol

Build a deterministic corpus runner with per-replay JSON summary and aggregate report by split and game size. Pin parser/simulator revisions, options, seed, corpus manifest of hashes, and metric definitions. The ignored replay corpus stays local; tests skip with a clear message if absent, while small synthetic unit tests run everywhere.

Use held-out observations within `train` to measure reconstruction where truth exists: mask selected ball/car observations, reconstruct them from earlier observations, and compare position (UU), velocity (UU/s), orientation angle, angular velocity, boost, flags, pads, and event timing at the withheld frame. Report median, p90/p99, missingness, coverage, and error versus gap length and event proximity. Evaluate full-state internal consistency (finite values, unit rotations, plausible bounds, stable IDs, monotonic ticks, legal score/clock transitions) on every parsed frame. Compare simulation against an observation-only baseline; report both raw replay agreement and prediction accuracy so snapping to observed states cannot hide a bad model. Investigate outliers on `train`, approve changes on `validation`, then freeze code and settings before the one final `test` run. Record test results in a separate `RESULTS.md` or generated report, with dates and hashes.

Target acceptance gates after baselines are known: strict parse/conversion success on supported soccar corpus, zero unexplained identity/scoreboard invariant failures, reproducible output, serialization round-trip and Python load, and a material held-out improvement over the baseline on validation without major regressions by game size. Set numerical error thresholds from measured baselines rather than guessing values now. Any unsupported replay gets an explicit error or partial-result status.

### Phase 0 measured baseline (train only, 2026-09-27)

- `cargo run --bin replay_audit -- replays/train target/train-audit.json` strictly parsed 60/60 files, all `TAGame.Replay_Soccar_TA`, totaling 677,609 network frames. File hashes and individual diagnostics are in the generated, ignored `target/train-audit.json`.
- There were 1,577,611 rigid-body updates. Linear and angular velocity were absent in 1,547 of them (about 0.098% each). The fields must still be modeled as optional, since absence is concentrated in particular updates and will affect state correctness.
- No non-monotonic frame times were found. The audit should next record distributions of gaps and phase transitions, since monotonic time alone does not prove continuous physics time.
- Frequent relevant attributes include `ReplicatedRBState` (1,577,611), `ReplicatedSteer` (800,401), `ReplicatedThrottle` (317,093), `ReplicatedActive` (190,244), `NewReplicatedPickupData` (112,989), `ReplicatedBoost` (55,576), `bReplicatedHandbrake` (50,673), `SecondsRemaining` (18,877), and `MatchScore` (18,652). Actions and scoreboard belong in the conversion output even where they are not part of RocketSim's `ArenaState`.
- The local soccar collision meshes load and support a RocketSim arena tick (`cargo test --test mesh_smoke`).

### Phase 1 observations (train only, 2026-09-27)

- A typed observation extractor now tracks actor lifetimes, car-to-PRI, player-to-team and boost-component-to-car links, ball/car rigid bodies, replicated steering/throttle/handbrake, boost amount, team score, game clock, overtime, player match stats, and goal-scored-on events. Each observed field carries its last source frame. The final header score stays separate from the frame timeline.
- Network keyframes announce the same live actors repeatedly, often about every 300 frames. The extractor preserves their state on same-ID/same-class announcements, and resets only after deletion or a class replacement. All 60 training replays produced 149,190 such repeat announcements.
- Across all train frames, extraction found zero unknown rigid-body actors and zero frames missing the ball. Final network scores agree with header totals after treating an omitted zero-valued header side as zero. This is checked by `cargo test --test observations_train` (12 seconds locally).
- Initial extraction left 49,472 of 2,740,265 car-frame records unlinked (1.81%). Most arose when the pawn-to-player link became inactive at demolition; preserving the last known owner until actor deletion reduced the count to 3,162 (0.12%). The remaining transient actors stay explicitly unlinked. Actor creation frame now distinguishes ID reuse, and a new actor lifetime resets its RocketSim car state; a real replay regression covers the demolition and respawn sequence.
- A raw boost byte of 85 corresponds to the kickoff amount of about 33.3, consistent with `raw * 100 / 255`. Steering/throttle byte 128 is neutral; the provisional normalization maps 0 to -1 and 255 to 1.
- Ground-car motion calibration on three 1v1 and one 3v3 training replay gave a median displacement-to-linear-velocity ratio of about 1.00. The median yaw-rate-to-replay-angular-velocity ratio was about 0.0100, so multiply boxcars angular velocity by **0.01** for RocketSim radians per second. This corrects an earlier degrees-per-second hypothesis. Recheck against airborne rotation and other replay versions.

### Preliminary simulation bridge (train samples only, 2026-09-27)

- `conversion::convert_bytes` now produces one `ArenaState` per replay frame, aligned to a 120 Hz timeline. It advances RocketSim only through intervals where both adjacent frames are in the replay's `Active` phase. Countdown and post-goal time is recorded as skipped timeline ticks. The arena's own tick count therefore differs from the full replay-timeline tick.
- Ball/car position, quaternion, linear velocity, and calibrated angular velocity are applied only when newly observed; boost is similarly corrected only on a fresh update. Unknown replay inputs remain absent from observation output; simulator controls use observed throttle/steer/handbrake, inferred boost activity from the replicated counter, and neutral values for missing jump/aerial controls. All cars currently receive the Octane hitbox until loadout-to-hitbox mapping is implemented.
- On the first replay of each train game size, median pre-correction position error (UU) was: 1v1 ball 10.21/car 16.48, 2v2 ball 11.35/car 17.01, 3v3 ball 11.19/car 17.33. The corresponding hold-last-position baselines were ball 42.44/50.12/51.63 and car 116.59/114.68/112.54. Linear extrapolation medians were ball 10.48/11.87/11.60 and car 18.21/18.58/18.84. These are in-sample short-gap checks, not held-out validation metrics; report full distributions and event-specific errors before drawing broad conclusions.
- `cargo test` passes the fresh-field regression test, the 1v1/2v2/3v3 train conversion smoke test, the soccar mesh smoke test, and the 60-file training observation test.

### Preliminary serialization and Python access (2026-09-27)

- Schema-v1 JSONL writes a header with SHA-256 fingerprint, pinned dependency revisions, conversion options, player slots, and diagnostics; each frame includes a projected soccar RocketSim state (ball, cars, boost pads), observed replay fields with provenance, separate simulated events, and prediction residuals. `convert_replay` is the command-line writer.
- The standard-library Python reader streams full frame records. An optional NumPy function makes dense time, ball, car, boost, score, and clock arrays with a car-presence mask and NaN for missing numeric fields. A synthetic loader test and a real 12,292-frame training replay read passed. That replay's JSONL is 115 MB, motivating the planned columnar prototype. The current Rust conversion holds all frames in memory.
- Exported RocketSim fields absent from replay are simulator estimates, and many have not been calibrated yet. Schema version 1 is provisional until a state-restoration check and compact-format comparison are complete.

### Corpus-wide development evaluation (2026-09-27)

- `evaluate_corpus` converted all 60 `train` and all 60 `validation` replays with zero failures. It saves per-replay hashes, options, errors, and aggregate quantiles in ignored JSON reports. The `test` split has not been touched.
- A deterministic withheld-physics check masks all ball/car rigid-body fields at offsets 1–4 of every 100-frame block, then compares simulated positions to fresh original packets in active play. With inferred boost, four-frame validation car median/p90 errors (UU) are 15.4/45.7 in 1v1, 16.0/40.0 in 2v2, and 17.4/46.8 in 3v3; linear extrapolation gives 25.5/66.8, 26.5/63.1, and 28.5/65.6 respectively. Ball gains are smaller. See `RESULTS.md` for all groups, protocol, and caveats.
- On one-step train comparisons, RocketSim improves median car position over linear extrapolation but loses at p99 in all three game sizes. Investigate collision, demolition, kickoff, hitbox and missing-input cases before claiming full-state accuracy. The fixed periodic mask should be cross-checked against a different deterministic schedule.
- `calibrate_boost` found that 2,818/2,824 short train intervals ending with an odd-to-even boost activation counter transition show boost depletion; validation has 3,011/3,014. Interpreting odd counter values as active boost input improves four-frame masked car median/p90 across validation sizes, though one-step car p99 increases slightly. The option is enabled by default and can be ablated with `--no-inferred-boost` in the CLIs. The signal is inferred rather than an explicit boolean action field.

## Implementation sequence and deliverables

| Phase | Deliverable | Exit check |
| --- | --- | --- |
| 0 | Pin dependencies, initialize meshes, add one replay inspection CLI and corpus manifest | Strict parse and arena smoke run on a `train` replay; dependency/asset setup documented |
| 1 | Typed actor graph, observation mapping, identity and scoreboard timeline | Audits on all `train` replays; no unexplained ID/score/clock transitions |
| 2 | Observation-only frame snapshots and provenance | Deterministic fixtures, transform checks, complete train diagnostics |
| 3 | 120 Hz simulation bridge and replay corrections | Held-out metrics beat observation baseline on `validation` |
| 4 | Controls, boost pads, jump/flip/demo and event refinements | Ablations and outlier reports show each refinement helps validation |
| 5 | Versioned serialized format, CLI and Python loader | Rust round-trip and Python read test, bounded file size/read time |
| 6 | Freeze options, run final `test` evaluation, document limits | Reproducible report with split/game-size metrics and failure inventory |

Keep this file current after each phase: update the status, dependency revisions, decisions, measured results, outstanding risks and next action. Record changes that fail validation too, so later agents do not repeat them.

## Open decisions and needed resources

- Native Rust `rocketsim` is the current target. Revisit only if the user specifies the C++ bindings.
- Determine the replay actor links and encoding/scaling of controls, boost, clock, and score on `train` before applying them to simulator state.
- Determine desired downstream Python shape and preferred storage after a small Arrow IPC versus Parquet prototype. The schema should support both ML arrays and richer event/scoreboard inspection.
- When the independent converter is ready, request the user's Python/`rust-carball` implementation for a controlled comparison after this pipeline has its own baseline.

## Work log

- 2026-09-27: Inspected starter repository and counted replay splits. Read `boxcars` 0.11.5 public docs and native `rocketsim` source from the locally cached upstream `v3-rust` checkout. Wrote initial architecture and evaluation plan. No replay contents were parsed and no converter code was changed.
- 2026-09-27: User supplied collision meshes and requested naturally segmented commits. Began Phase 0; kept meshes and IDE files out of version control.
- 2026-09-27: Pinned both dependencies, implemented strict parser corpus audit with SHA-256 manifest, parsed all `train` replays, and passed the soccar mesh smoke test. Phase 0 complete.
- 2026-09-27: Added typed frame observations and an inspection CLI. Verified every `train` replay retains the ball and matches final score, and measured repeated actor announcements and unresolved ownership. Phase 1 remains open.
- 2026-09-27: Calibrated velocity units from train motion, corrected angular velocity scale to 0.01, implemented the first RocketSim bridge and per-observation position residuals, and verified one replay per game size. Full train metrics and action inference remain.
- 2026-09-27: Added schema-v1 JSONL state export, command-line conversion, and a Python streaming/NumPy loader. Read a real training export end to end; compact storage, state restoration, and full-corpus evaluation remain.
- 2026-09-27: Added a corpus evaluator with one-step and four-frame masked-physics metrics. Established train and validation baselines on all 120 development replays; recorded findings in `RESULTS.md`. The held-out `test` split remains sealed.
- 2026-09-27: Traced most unlinked car frames to inactive pawn links during demolition, retained the known owner until deletion, and keyed car-slot fallback to actor creation frame to block reused-ID contamination. Reset simulator car state on a genuine new lifetime. Re-ran train and validation reports; masked-position medians were stable and car p90 slightly improved on validation.
- 2026-09-27: Calibrated the boost-component activation counter against boost consumption on train, enabled odd-value boost input, and ran a no-boost ablation. All three validation game sizes improved on four-frame masked car position; documented the small one-step p99 regression and refreshed `RESULTS.md`.
