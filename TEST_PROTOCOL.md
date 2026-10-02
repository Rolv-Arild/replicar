# Test-split assessment protocol (draft, 2026-10-01)

Status: **draft for approval; the test split has not been touched.** `replays/test` stays sealed until the
user approves this document and the frozen commit below is tagged. The test split is run once.

## 1. What the assessment is for

An unbiased estimate, on replays that were never used to design or tune anything, of
1. whether the converter runs on every replay (robustness and cost);
2. how large its held-out errors are, on the same metrics as the development numbers (section 4);
3. whether the truth-free consistency properties of the newer outputs (scoreboard, pads, demolitions,
   touches) hold on unseen online replays (section 5).

It cannot confirm accuracy against server truth. That exists only for the two LAN games
(`replays/2026-09-30T16-56-39Z_lan_remote_4bots_game1`, `..._17-05-59Z_..._game2`), which are
diagnostics and not a split. Everything in `RESULTS.md` about touches, demolitions, pickups, the clock,
the recorder's control lead and packet lags rests on them or on the train and validation corpus.

## 2. What is frozen

Fill in before the run (all of it must be identical for the whole assessment):

| Item | Value |
| --- | --- |
| Source commit (tag `test-assessment-1`) | `TO BE SET` (branch `events-scoreboard-check`; the tip when approved, working tree clean) |
| Rust toolchain, `Cargo.lock` | as committed (boxcars `=0.12.0`, glam `=0.33.11`, arrow/parquet `=60.0.0`) |
| RocketSim | git rev `0b020516c4fc633e0db09dfbfaa2026bcddb058e` |
| Options | `ConvertOptions::default()` of the tagged commit; no flags except the two evaluation variants below |
| Mask schedule | `evaluate_corpus` defaults (every 100-frame block masks four consecutive frames, no seed) |
| Collision meshes | `./collision_meshes` as in the working directory (ignored by git) |

Defaults that matter (so a reader does not have to read the code): packet lags inferred offline
(`infer_packet_lag`, exact tick chains, ball chains across hits, lag-free replays detected), air and
ground fits against later packets (`air_bvp`, `fit_on_next_packet` off in the held-out evaluators),
ground timing shifts -8..+40 with the per-car median shift, observed demolitions applied and RocketSim's
own demo rule off, ball contacts from ball packets on, `align_contacts` on (car chain runs moved by the contacts' timing votes; two conversion passes), pads blocked in the offline
simulation (the replay's pickups are used, blocked from the first tick of every interval), dodge and jump fits on.

What the evaluators hold out (an independent review found the earlier text overstated it): `evaluate_corpus`
scores its one-step residuals from a conversion with the fits that choose a value using the packet being scored
switched off (`flip_cancel_holdout`, no air-control lookahead, no dodge first-packet tick, no contact alignment;
`air_bvp` and `fit_on_next_packet` are off as before); `--offline-fits` gives the full offline conversion, whose
rotation and angular velocity residuals are partly in-sample (section 4, both are reported). The masked
prediction is a separate conversion with the withheld frames refused by every fit. Its `--aligned-targets`
variant scores a predictor that infers packet lags against targets from the full offline conversion;
`--aligned-targets-raw-predictor` scores the default predictor against the same targets. `error_budget` is an
offline diagnostic breakdown (full fits, in-sample for rotation); its numbers are not held-out.

## 3. Rules for the run

* Run once, in this order: `evaluate_corpus replays/test target/test-default.json`,
  `evaluate_corpus replays/test target/test-aligned.json --aligned-targets`,
  `evaluate_corpus replays/test target/test-aligned-raw.json --aligned-targets-raw-predictor`,
  `evaluate_corpus replays/test target/test-offline.json --offline-fits`,
  `error_budget replays/test --final-assessment` (the flag lifts its guard against paths containing "test"),
  `check_scoreboard replays/test`, `count_demolitions replays/test`, and the pad and touch consistency
  counts of section 5. No parameter, threshold, option or code change is made after seeing any result.
* Every result is reported, including failures and unfavourable splits, with counts and per-size and
  per-replay behaviour. A replay that does not convert is reported with its error, not dropped.
* If the run finds a defect, it is recorded in `RESULTS.md` with the evidence and fixed on a new branch;
  the test split is not run again for the fix unless the user decides so explicitly, and the second run is
  then labelled as such, since it is no longer unseen.
* Hardware and wall time are recorded.

## 3a. Commands

```
cargo build --release --bins
./target/release/evaluate_corpus.exe replays/test target/test-default.json
./target/release/evaluate_corpus.exe replays/test target/test-aligned.json --aligned-targets
./target/release/evaluate_corpus.exe replays/test target/test-aligned-raw.json --aligned-targets-raw-predictor
./target/release/evaluate_corpus.exe replays/test target/test-offline.json --offline-fits
./target/release/error_budget.exe replays/test --final-assessment
./target/release/check_scoreboard.exe replays/test
./target/release/count_demolitions.exe replays/test
python scripts/summarize_reference.py target/test-default.json target/test-aligned.json target/test-aligned-raw.json target/test-offline.json
python scripts/acceptance.py check target/acceptance-bands.json target/test-default.json target/test-aligned.json
```

The bands are built from the development reports (60 train and 60 validation replays, default and aligned runs of the
build that is frozen) before the test run, with
`python scripts/acceptance.py bands target/acceptance-bands.json <train-default>,<train-aligned> <val-default>,<val-aligned>`;
`scripts/run_reference.sh` makes the four reports.

## 4. Held-out accuracy: development reference and acceptance

Development reference at commit `8acdb70` (reports in `target/ref-final/*.json` for train and validation, default and
aligned; a run is `evaluate_corpus replays/<split> <report> [flag]`, summarised by `python scripts/summarize_reference.py`).
It is the commit after the independent review of 2026-10-02 (RESULTS.md, "Independent review of the evaluators"),
which changed the evaluators: the one-step rows are now held out by default and the linear baseline of the aligned
variant is lag-corrected, so the earlier reference (`96d1e0a`, `3f91f6d`) is not comparable on the rows marked *.
All 120 train and validation replays convert. The pooled numbers in the table are validation; train is close but not
within a few percent everywhere (the default masked car position at horizon 1 is 17.1 / 44.6 / 76 UU on train against
16.6 / 41.0 / 75, p90 +9%; the aligned masked ball 0.00 / 14.5 / 31 against 0.00 / 13.3 / 29, p90 +9%), which is why the
acceptance below is not a fixed percentage.
The table was regenerated at the merged tip `2f5a37b` (after the second review pass and the dodge-refresh correction;
reports in `target/ref-merge/`): every row above is unchanged to the printed precision (the one-step ball velocity
p99 moves 406 to 402 UU/s).

| Metric (p50 / p90 / p99) | Simulated | Hold baseline | Linear baseline |
| --- | --- | --- | --- |
| Car position error before correction, UU (n 1,056,574) * | 0.05 / 3.9 / 38 | 112 / 188 / 230 | 17.6 / 44 / 71 |
| Ball position error before correction, UU (n 543,266) * | 0.01 / 0.0 / 20 | 47 / 96 / 154 | 9.9 / 35 / 70 |
| Car one-step velocity residual, UU/s * | 1.4 / 49 / 347 | 82 / 327 / 811 | |
| Car one-step rotation, deg * | 0.29 / 3.1 / 11 | | |
| Car one-step angular velocity, rad/s * | 0.09 / 0.9 / 3 | | |
| Masked car position, 1 frame ahead, UU (default) | 16.6 / 41 / 75 | 112 / 187 / 230 | 17.7 / 44 / 77 |
| Masked ball position, 1 frame ahead, UU (default) | 10.7 / 33 / 66 | 48 / 96 / 159 | 9.8 / 35 / 70 |
| Masked car position, 1 frame ahead, UU (`--aligned-targets`) * | 0.56 / 16.5 / 54 | 131 / 219 / 268 | 7.1 / 24 / 72 |
| Masked ball position, 1 frame ahead, UU (`--aligned-targets`) * | 0.00 / 13.3 / 29 | 69 / 128 / 188 | 1.0 / 16 / 76 |
| Masked car position, 1 frame ahead, UU (`--aligned-targets-raw-predictor`) | 19.1 / 55.5 / 85 | 131 / 219 / 268 | 22 / 56 / 91 |
| Masked ball position, 1 frame ahead, UU (`--aligned-targets-raw-predictor`) | 16.7 / 47 / 84 | 69 / 128 / 188 | 16.7 / 49 / 91 |

Offline-fit values of the starred one-step rows (`--offline-fits`, partly in-sample, the earlier reference): car
position 0.05 / 3.7 / 33, ball 0.01 / 0.0 / 19, velocity 1.4 / 48 / 315, rotation 0.25 / 2.4 / 10, angular
velocity 0.07 / 0.5 / 2. The held-out rotation p90 is 29% and the angular velocity p90 80% larger: those two
rows had been flattered by fits that choose their value from the packet being scored (flip cancel, air
control lookahead, dodge first-packet tick, contact alignment).

How to read the masked rows. The default masked predictor does not infer packet lags, so it starts from
stale packets placed at frame time: no better than a linear extrapolation (16.6 against 17.7 UU at p50).
Scoring it against the aligned (lag-corrected, offline) targets does not help (19.1 against 22.3): the cleaner
target alone is not what lowers the error. The aligned variant's 0.56 UU comes from the predictor that infers
lags and uses the lag-dependent ground and jump fits, scored against aligned targets. The linear baseline of
that variant is extrapolated over the time from the stale packet's tick, using the lags that same masked
conversion inferred (the earlier table's 22 UU extrapolated over the raw frame time and so kept the timing
error the simulation had removed). With that baseline the simulation's lead for the car is 7.1 to 0.56 UU at
p50 and 24 to 16.5 at p90; for the ball the linear baseline is within 1 UU at p50 and 15.6 against 13.3 UU
at p90 at horizon 1.

Acceptance. A fixed tolerance (such as +10% of the validation value) would be arbitrary here: two 60-replay splits
of the same corpus differ by up to 9% at p90 (above). The bands are therefore derived from the spread of the
development replays themselves (`scripts/acceptance.py`; its docstring has the method):
* **Rows and bands.** For each row (the five one-step car and ball rows and the masked car and ball position at
  horizons 1 to 4 in the default and aligned variants) and each of p50, p90 and p99, the statistic of a split is the
  median over its replays of each replay's own quantile. The band is a bootstrap prediction interval for that
  statistic over a new 60-replay split (20 per game size), from the 120 development replays; the car position rows
  also have a band per game size. A test statistic outside its band is a finding. About 4 of the 84 comparisons are
  expected outside by chance alone, so the number is judged against that and not against zero. Calibration
  check (`scripts/acceptance.py selfcheck`): bands from train against validation had 3 of 84 outside, bands from
  validation against train 3 of 84. Rows whose simulated position error is below 0.01 UU in the development
  median (the replay's own position resolution) are not graded.
* **Ordering.** Simulated < linear < hold of the masked rows is graded where it holds at that horizon, object and
  quantile (p50, p90) in both development splits (30 of the possible 32 do; it does not hold for the default
  masked ball at p50, where a linear extrapolation is as good as the simulation), and the test split must show
  the same order.
* **Outlier replays.** Test replays whose own car p90 (one-step position, masked horizon 1 default and aligned)
  exceeds that of every development replay of the same game size are listed with a cause when one can be found
  from their own data.
* **Robustness.** Every test replay converts (60 of 60). Wall time and the hardware are reported (development:
  train 253 / 701 s and validation 264 / 731 s for the default and the aligned run), with no threshold.
A miss is reported as a finding, not as a failure of the assessment.

## 5. Truth-free consistency checks (reported, not graded against validation)

| Property | Development value (train / validation) | Command |
| --- | --- | --- |
| Shown clock integer equals the ceiling of the reconstructed clock in running frames | 98.4% / 98.6%, never off by more than one | `check_scoreboard` |
| Simulated demolitions (`is_demo`) | 0 simulated (276 reported, 224 not repeats on train) | `count_demolitions` |
| Observed demolitions that are repeats (within 3 s) | 52 of 276 reported on train | `count_demolitions` (and `Event::Demolish.repeat`) |
| Boost pickups with the pad matched to RocketSim's list | 100% of the six train replays checked | `boost_pickups` records |
| Boost pickups whose car's path reaches the pad | 94-98% (six train replays) | `boost_pickups` records |
| Simulated touches inside a ball-contact interval, and contact intervals holding a simulated touch | LAN games only (host 95-97% recall, 0 surplus; client 77-92%, 5-15% surplus) | `ball_contacts` and `touches` records |
| Replays detected as lag-free | none of the 120 | `infer_packet_lags` |

The values for the unchecked rows are computed on the test split with the same scripts and compared in
the write-up; a large gap from the development value is a finding about generalisation, not a bug by
itself.

## 6. Stated limits (so the write-up does not over-claim)

* The validation split guided many choices in this branch (the per-car control shift, the shift range, the
  contact threshold, the lag-free test). Its numbers are somewhat optimistic; that is the reason for this
  assessment.
* Accuracy against server truth is from two LAN games with one set of bots and two humans on a fast link.
  Online replays with real latency are not covered by any truth.
* Causal prediction has no timing model; the masked rows are what the converter does when the packets
  are withheld, not a deployable predictor.
* Air pitch, yaw and roll are per-interval model controls, not per-tick inputs; held buttons without a
  physical effect are not identifiable.
* The five-second kickoff fallback of the match clock never occurred in the data and is unverified.
* The masked conversions of the aligned variants use replay-wide quantities fitted on all frames of the replay,
  including after the withheld ones: the ball-car lag offset (from all of a replay's hits), the lag-free
  detection, and in every mode the pad-name votes. They are global constants rather than per-target values,
  but the masked rows are causal-style, not causal.
* The one-step rows and the full offline conversion still use the frame's own packet for the correction at
  that frame (by design: the residual is measured before it). Only the fits listed in section 2 are held out.
