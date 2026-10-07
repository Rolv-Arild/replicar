# Test-split assessment protocol (for approval, 2026-10-03)

Status: **approved by the user on 2026-10-03 and run once** (tag `test-assessment-1` = `99f0a9d`; results in RESULTS.md, "Test-split assessment"). The test split is no longer unseen; any further run is a second, labelled run.

This protocol ran version 1's tools. Version 2's `replicar-eval` has `evaluate` and `error_budget` with the same options and reports (byte-identical to v1's on train and validation; RESULTS.md, "v2: the evaluator on v2"); a run of v2 on the test split would use them in place of `evaluate_corpus` and `error_budget`.

## 1. What the assessment is for

An unbiased estimate, on replays that were never used to design or tune anything, of
1. whether the converter runs on every replay (robustness and cost);
2. how large its held-out errors are, on the same metrics as the development numbers (section 4);
3. whether the truth-free consistency properties of the outputs (scoreboard, pads, demolitions, touches)
   hold on unseen online replays (section 5).

It cannot confirm accuracy against server truth. That exists only for the two LAN games
(`replays/2026-09-30T16-56-39Z_lan_remote_4bots_game1`, `..._17-05-59Z_..._game2`), which are
diagnostics and not a split. Everything in `RESULTS.md` about touches, demolitions, pickups, the clock,
the recorder's control lead and packet lags rests on them or on the train and validation corpus.

The converter is an offline tool whose aim is the most accurate reconstruction of states and inputs. It uses
future frames by design (offline fits, replay-wide estimates); that is not treated as a leak. The one rule the
evaluation keeps is that no fit may use the packet it is scored on (section 2).

## 2. What is frozen

| Item | Value |
| --- | --- |
| Source commit (tag `test-assessment-1`) | the commit that adds this approved document on branch `audit-and-leftovers`, working tree clean. Its conversion and evaluation code equals the reference build `509c540` except one change whose default output is byte-identical on all 120 development replays (`a30470a`); the other differences are documentation and the `replay_audit` sealed-path check |
| Rust toolchain, `Cargo.lock` | as committed (boxcars `=0.12.0`, glam `=0.33.11`, arrow/parquet `=60.0.0`) |
| RocketSim | git rev `0b020516c4fc633e0db09dfbfaa2026bcddb058e` |
| Options | `ConvertOptions::default()` of the tagged commit; no flags except the evaluation variants below |
| Mask schedule | `evaluate_corpus` defaults (every 100-frame block masks four consecutive frames, no seed) |
| Collision meshes | `./collision_meshes` as in the working directory (ignored by git) |

Defaults that matter (so a reader does not have to read the code): packet lags inferred offline
(`infer_packet_lag`, exact tick chains, a packet shared by two lag runs keeps the earlier run's lag
(`lag_boundary` earlier), ball chains across hits, the replay-wide ball-car offset estimate, lag-free replays
detected); air and ground fits against later packets (`air_bvp`, `fit_on_next_packet` off in the held-out
evaluators); ground timing shifts -8..+40 with the per-car median shift; observed demolitions applied and
RocketSim's own demolition rule off; dead pawn shells held demolished (goal-explosion events, sleeping packets of
unlinked cars); a fresh sleeping packet zeroes the simulated velocity; a car before its first packet starts at its
spawn pose and is kept out of collisions until that packet; observed dodge refreshes applied; ball contacts from
ball packets; `align_contacts` on; simulated pad pickups blocked from the first tick of every interval; dodge and
jump fits on.

What the evaluators hold out: `evaluate_corpus` scores its one-step residuals from a conversion with the fits
that choose a value using the packet being scored switched off (`flip_cancel_holdout`, no air-control lookahead,
no dodge first-packet tick, no contact alignment; `air_bvp` and `fit_on_next_packet` off); `--offline-fits` gives
the full offline conversion, whose rotation and angular velocity residuals are partly in-sample. The masked
prediction is a separate conversion in which every fit refuses the withheld frames (tested: cutting the replay
right after a window leaves the default masked predictions unchanged). Its `--aligned-targets` variant scores a
predictor that infers packet lags against targets from the full offline conversion;
`--aligned-targets-raw-predictor` scores the default predictor against the same targets. `error_budget` is an
offline diagnostic (full fits, in-sample for rotation); its numbers are not held out.

## 3. Rules for the run

* Run once, with the commands of section 3a, in that order. No parameter, threshold, option or code change is
  made after seeing any result. The default variant is the main result; the aligned variant is secondary; the
  raw-predictor, offline-fit and `error_budget` runs are reported for context and not graded.
* Every result is reported, including failures and unfavourable rows, with counts and per-size and per-replay
  behaviour. A replay that does not convert is reported with its error, not dropped.
* If the run finds a defect, it is recorded in `RESULTS.md` with the evidence and fixed on a new branch; the
  test split is not run again for the fix unless the user decides so explicitly, and a second run is labelled
  as such, since it is no longer unseen.
* Hardware and wall time are recorded.

## 3a. Commands

Every tool that reads replays refuses a path with a `test` component unless `--final-assessment` is given.

```
git checkout test-assessment-1
cargo build --release --bins
./target/release/evaluate_corpus.exe replays/test target/test-default.json --final-assessment
./target/release/evaluate_corpus.exe replays/test target/test-aligned.json --aligned-targets --final-assessment
./target/release/evaluate_corpus.exe replays/test target/test-aligned-raw.json --aligned-targets-raw-predictor --final-assessment
./target/release/evaluate_corpus.exe replays/test target/test-offline.json --offline-fits --final-assessment
./target/release/error_budget.exe replays/test --final-assessment
./target/release/check_scoreboard.exe replays/test --final-assessment
./target/release/count_demolitions.exe replays/test --final-assessment
./target/release/consistency_counts.exe replays/test --final-assessment
python scripts/summarize_reference.py target/test-default.json target/test-aligned.json target/test-aligned-raw.json target/test-offline.json
python scripts/acceptance.py check target/acceptance-bands.json target/test-default.json target/test-aligned.json
```

The bands (`target/acceptance-bands.json`) were built before the run from the four development reports of the
reference build (`target/ref-freeze2/{train,validation}-{default,aligned}.json`, made by `scripts/run_reference.sh`)
with `python scripts/acceptance.py bands target/acceptance-bands.json <train-default>,<train-aligned> <val-default>,<val-aligned>`.
`acceptance.py check` refuses report pairs that do not cover the same replays (path and SHA-256), options and
count (60).

## 4. Held-out accuracy: development reference and acceptance

Development reference: build `509c540`, reports in `target/ref-freeze2/` (train and validation, default and
aligned; all 120 replays convert; wall time train 265 / 729 s, validation 263 / 723 s for the default and aligned
runs). The table is validation (`python scripts/summarize_reference.py`). Train is close but not within a few
percent everywhere (default masked car position at 1 frame 17.1 / 44.6 / 76 UU against 16.6 / 41.0 / 75), which is
why the acceptance below is not a fixed percentage. RESULTS.md ("Candidate freeze reference", "Review of
`509c540`" and the sections before) has the comparison with the earlier reference `8acdb70`: default masked rows
identical, aligned masked car better at every horizon, one-step ball velocity p99 402 to 393 UU/s.

**How to read the masked horizon.** The label counts frames from the start of the mask window, not time. A masked
car starts from its last fresh body packet, which is often older than the frame before the window (cars are not
refreshed every frame), so each row also gives the median age of the packet the prediction starts from. Car and
ball are compared at equal age through the age-bucket rows, never by label. All three methods start from the same
packet, so the comparison between them is unaffected.

| Metric (p50 / p90 / p99) | Simulated | Hold baseline | Linear baseline |
| --- | --- | --- | --- |
| Car position error before correction, UU (n 1,056,568) | 0.05 / 3.9 / 38 | 112 / 188 / 230 | 17.6 / 44 / 71 |
| Ball position error before correction, UU (n 543,266) | 0.01 / 0.0 / 20 | 47 / 96 / 154 | 9.9 / 35 / 70 |
| Car one-step velocity residual, UU/s | 1.45 / 49 / 345 | 82 / 327 / 811 | |
| Car one-step rotation, deg | 0.30 / 3.1 / 11 | | |
| Car one-step angular velocity, rad/s | 0.09 / 0.9 / 3 | | |
| Masked car, default, 1 frame (start-packet age 0.068 s) | 16.6 / 41.0 / 75 | 112 / 187 / 230 | 17.7 / 44.1 / 77 |
| Masked car, default, 2 frames (0.072 s) | 15.0 / 38.4 / 62 | 127 / 194 / 246 | 16.4 / 41.9 / 71 |
| Masked car, default, 3 frames (0.105 s) | 16.1 / 39.7 / 76 | 173 / 327 / 396 | 21.7 / 55.8 / 109 |
| Masked car, default, 4 frames (0.167 s) | 15.8 / 39.2 / 77 | 234 / 373 / 422 | 27.4 / 64.9 / 134 |
| Masked ball, default, 1 frame (0.033 s) | 10.7 / 33.1 / 66 | 48 / 96 / 159 | 9.8 / 34.6 / 70 |
| Masked ball, default, 2 frames (0.067 s) | 12.9 / 35.3 / 66 | 96 / 177 / 256 | 12.6 / 39.3 / 113 |
| Masked ball, default, 3 frames (0.100 s) | 13.2 / 36.3 / 71 | 142 / 258 / 356 | 13.6 / 44.1 / 205 |
| Masked ball, default, 4 frames (0.134 s) | 12.1 / 37.0 / 72 | 189 / 341 / 453 | 13.7 / 50.0 / 303 |
| Masked car, default, start-packet age <= 0.08 s (n 12,923) | 13.7 / 36.6 / 58 | 117 / 173 / 211 | |
| Masked ball, default, start-packet age <= 0.08 s (n 6,123) | 12.1 / 34.3 / 66 | 98 / 174 / 249 | |
| Masked car, aligned, 1 frame | 0.52 / 15.8 / 52 | 131 / 219 / 268 | 6.9 / 23.7 / 71 |
| Masked car, aligned, 4 frames | 3.07 / 19.1 / 58 | 258 / 408 / 474 | 20.6 / 67.7 / 152 |
| Masked ball, aligned, 1 frame | 0.00 / 13.1 / 28 | 69 / 128 / 187 | 0.95 / 15.3 / 76 |
| Masked ball, aligned, 4 frames | 0.01 / 15.7 / 52 | 210 / 373 / 493 | 7.9 / 39.2 / 337 |

How to read the masked rows. The default masked predictor does not infer packet lags, so it starts from stale
packets placed at frame time and is only somewhat better than a linear extrapolation for the car, and level with it
for the ball at p50. The aligned variant's predictor infers lags and uses the lag-dependent fits, scored against
targets from the offline conversion; its linear baseline is extrapolated over the time from the stale packet's
inferred tick. Part of the aligned rows' advantage is that predictor and target share the lag inference.

**Acceptance.** Derived from the spread of the 120 development replays (`scripts/acceptance.py`; its docstring has
the method):
* **Rows and bands.** For each row (the five one-step car and ball rows and the masked car and ball position at
  horizons 1 to 4 in the default and aligned variants) and each of p50, p90 and p99, the statistic of a split is the
  median over its replays of each replay's own quantile. The band is a bootstrap prediction interval for that
  statistic over a new 60-replay split (20 per game size); the car position rows also have a band per game size.
  84 comparisons in all. Rows whose simulated position error is below 0.01 UU in the development median are not
  graded.
* **How many misses to expect.** The rows are strongly correlated, so the count outside is judged against an
  empirical null: random halves A and B of the development replays (stratified by size), bands built on A, rows
  outside counted on B as a fresh 60-replay split. Null: mean 5.5, p95 11, p99 13 (max 14) of 84. The verdict is
  "within", "above the p95" or "above the p99" of that null. Calibration: bands from train counted on validation
  give 3 of 84 outside, bands from validation on train 2.
* **Ordering.** Simulated < linear < hold of the masked rows is graded where it holds at that horizon, object and
  quantile (p50, p90) in both development splits (30 rules; it does not hold for the default masked ball at p50,
  where a linear extrapolation is as good as the simulation), and the test split must show the same order.
* **Outlier replays.** Test replays whose own car p90 (one-step position, masked horizon 1 default and aligned)
  exceeds that of every development replay of the same game size are listed, with a cause when one can be found
  from their own data.
* **Robustness.** Every test replay converts (60 of 60); wall time and hardware are reported, with no threshold.
A miss is reported as a finding, not as a failure of the assessment.

## 5. Truth-free consistency checks (reported, not graded)

Development values from the reference build (`consistency_counts`, `count_demolitions`, `check_scoreboard`).

| Property | Train | Validation |
| --- | --- | --- |
| Shown clock integer equals the ceiling of the reconstructed clock (running frames) | 98.4% (540,215), never off by more than one | 98.6% (636,600), never off by more than one |
| Boost pickups with the pad matched to RocketSim's list | 23,370 of 23,370 | 24,116 of 24,117 |
| Boost pickups whose car's path reaches the pad | 96.1% | 96.6% |
| Simulated touches inside a ball-contact interval | 97.0% (11,570 of 11,927) | 97.3% (12,527 of 12,880) |
| Ball-contact intervals holding a simulated touch | 84.9% (11,689 of 13,761) | 84.7% (12,708 of 15,012) |
| Non-goal demolition events: linked and not a repeat / repeats (5 s window) / no linked car | 197 / 84 / 17 (298) | 86 / 41 / 1 (128) |
| Post-goal explosions (excluded from demolitions) | 248 | 455 |
| Simulated demolitions | 0 (by construction: RocketSim's own rule is off) | 0 |
| Replays detected as lag-free | 0 of 60 | 0 of 60 |

A large gap between the test value and these is a finding about generalisation, not a bug by itself.

## 6. Stated limits (so the write-up does not over-claim)

* The validation split guided many choices (the per-car control shift, the shift range, the contact threshold,
  the lag-free test, the `earlier` lag rule). Its numbers are somewhat optimistic; that is the reason for this
  assessment.
* Accuracy against server truth is from two LAN games with one set of bots and two humans on a fast link. Online
  replays with real latency are not covered by any truth.
* Future frames are used by design. The masked conversions use replay-wide quantities fitted on all frames,
  including after the withheld ones: the ball-car lag offset (in the aligned variant; cutting the replay after a
  window moves its predictions by up to 29 UU on a train replay), the lag-free detection and, in every mode, the
  pad-name votes. The masked predictions are also conditional on the true observed controls of the withheld frames
  (controls are inputs). The masked rows are therefore what the offline converter does when packets are withheld,
  not a causal predictor.
* The replay-wide ball-car offset is weakly constrained on some replays (about 0.2 tick); there it can make the
  aligned masked ball worse than no estimate, and it interacts with the `earlier` lag rule (one-tick target moves on
  a few replays).
* Air pitch, yaw and roll are per-interval model controls, not per-tick inputs; held buttons without a physical
  effect are not identifiable.
* Inferred state is labelled but not observed: dead shells held demolished from a sleeping packet of an unlinked car,
  velocities zeroed by sleeping packets, the spawn pose before a car's first packet (frozen there, which costs about
  1.5 UU at those frames in active play).
* The five-second kickoff fallback of the match clock never occurred in the data and is unverified.
* The one-step rows and the full offline conversion use the frame's own packet for the correction at that frame
  (by design: the residual is measured before it). Only the fits listed in section 2 are held out.

## 7. Second run: version 2 (labelled; 2026-10-07)

The user asked on 2026-10-07 for a final check before merging v2, and allowed running the test split again. This run is **the second, labelled run**: the split was seen once (section 3), so it is not an unseen estimate, and nothing is tuned on it. It measures the converter as merged: v2 at tag `test-assessment-2` (the commit that adds this section), RocketSim crate `0.2.7`, `Cargo.lock` as committed.

**Frozen before the run.** The commands below; the acceptance bands rebuilt from the development reports of this converter, `target/ref-v2-inputs/{train,validation}-{default,aligned}.json` (after the input changes of RESULTS.md, "v2: inputs between frames"), into `target/acceptance-bands-2.json` with `scripts/acceptance.py bands`, before any test result. The first run's bands (`target/acceptance-bands.json`, from RocketSim `0b02051`) are also checked, for comparison with the first run. The method and the null of section 4 are unchanged.

**Commands** (`target/test-run-2/run.sh`, outputs and logs in `target/test-run-2/`):

```
cargo build --release -p replicar-eval -p replicar-cli
target/release/evaluate.exe replays/test target/test-run-2/test-default.json --final-assessment
target/release/evaluate.exe replays/test target/test-run-2/test-aligned.json --aligned-targets --final-assessment
target/release/evaluate.exe replays/test target/test-run-2/test-aligned-raw.json --aligned-targets-raw-predictor --final-assessment
target/release/evaluate.exe replays/test target/test-run-2/test-offline.json --offline-fits --final-assessment
target/release/error_budget.exe replays/test --final-assessment
python scripts/summarize_reference.py <the four reports>
python scripts/acceptance.py check target/acceptance-bands-2.json <default> <aligned>
python scripts/acceptance.py check target/acceptance-bands.json <default> <aligned>
target/release/replicar convert replays/test -o target/test-run-2/files --with resimulation --jobs 32
python scripts/check_test_files.py target/test-run-2/files replays/test
```

**What v2 adds to the checks** (reported, not graded; `scripts/check_test_files.py`): every test replay converts with the `replicar` command in the default output (tick rows) and with the `resimulation` group; each file resimulates to the same states; per player and counted statistic the stat events add up to `final_stats`; every goal report has a scorer; consecutive tick rows are one tick apart within a segment. The v1 consistency tools of section 3a (`check_scoreboard`, `count_demolitions`, `consistency_counts`) run v1's converter and are not repeated.

**Rules.** As in section 3: run once, in this order; nothing is changed after a result; every result is reported; a defect found is recorded and fixed on a new branch without running the split again.
