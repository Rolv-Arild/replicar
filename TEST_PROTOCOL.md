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
simulation (the replay's pickups are used), dodge and jump fits on. `evaluate_corpus` and `error_budget`
force the causal or held-out variant of each fit that would otherwise use the packet being scored.

## 3. Rules for the run

* Run once, in this order: `evaluate_corpus replays/test target/test-default.json` and
  `evaluate_corpus replays/test target/test-aligned.json --aligned-targets`, `error_budget replays/test`,
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
./target/release/error_budget.exe replays/test
./target/release/check_scoreboard.exe replays/test
./target/release/count_demolitions.exe replays/test
python scripts/summarize_reference.py target/test-default.json target/test-aligned.json
```

## 4. Held-out accuracy: development reference and acceptance

Development reference from the train and validation splits at commit `96d1e0a` (reports in
`target/ref/*.json`, regenerate with the same commands on those splits; the summary is
`python scripts/summarize_reference.py target/ref/{train,validation}-{default,aligned}.json`).
The numbers below are validation unless stated; train differs by at most a few percent.

| Metric (p50 / p90 / p99) | Simulated | Hold baseline | Linear baseline |
| --- | --- | --- | --- |
| Car position error before correction, UU (n 1,055,789) | 0.05 / 3.7 / 33 | 112 / 188 / 230 | 17.6 / 44 / 71 |
| Ball position error before correction, UU (n 543,266) | 0.01 / 0.0 / 19 | 47 / 96 / 154 | 9.9 / 35 / 70 |
| Car one-step velocity residual, UU/s | 1.4 / 48 / 316 | 82 / 327 / 811 | |
| Car one-step rotation, deg | 0.25 / 2.4 / 10 | | |
| Car one-step angular velocity, rad/s | 0.07 / 0.5 / 2 | | |
| Masked car position, 1 frame ahead, UU (default) | 16.6 / 41 / 76 | 112 / 187 / 230 | 17.7 / 44 / 77 |
| Masked car position, 1 frame ahead, UU (`--aligned-targets`) | 0.56 / 16.5 / 55 | 131 / 219 / 268 | 22 / 56 / 91 |
| Masked ball position, 1 frame ahead, UU (`--aligned-targets`) | 0.00 / 13.3 / 29 | 69 / 128 / 188 | 17 / 49 / 91 |

Read the first rows with the caveat of `PLAN.md`: the pre-correction residuals use offline fits against
later packets (they are reconstruction quality, with the packet being scored held out wherever the code
says so), while the masked rows are the causal-style prediction. Without the aligned targets the masked
prediction is no better than a linear extrapolation (16.6 against 17.7 UU at p50): packet timing noise,
not physics, dominates; the aligned variant removes it.

Acceptance, per metric and per game size (1v1, 2v2, 3v3), for the test split against validation:
* the p90 and p99 of every row above are within +10% of the validation values (the p50 within +0.5 UU
  or +10%, whichever is larger);
* the ordering simulated < linear < hold of the masked rows holds at every horizon and size;
* no individual replay has a car p90 above three times the validation p90 of its game size, and any that
  has is listed with its cause when it can be found from its own data;
* 60 of 60 replays convert; time per replay at most the development maximum (see below) plus 50%.
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
