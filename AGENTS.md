# Agent guide

These instructions apply throughout this repository. The user wants a reliable converter from Rocket League replay frames to RocketSim states, with scoreboard, actions, events, and other useful replay information alongside the physics state.

## Get oriented

- Read `README.md` for the current workflow, the top of `PLAN.md` for the next action, and the recent sections of `RESULTS.md` for evidence and limitations. Inspect `git status` and recent commits before changing anything.
- Treat `PLAN.md` as the handoff record. Keep its next action and work log current when a logical step finishes. Put protocols, numbers, negative results, and limitations in `RESULTS.md`.
- The Rust API lives mainly in `src/observations.rs`, `src/conversion.rs`, and `src/serialization.rs`. `boxcars` parses network frames; the pinned native `rocketsim` crate fills intermediate active-play ticks. Preserve the dependency pins unless a measured change justifies updating them (RocketSim was updated on 2026-09-29 to `0b02051`; re-run the logged issues in `ROCKETSIM_NOTES.md` after any update).
- Do not inspect the user's existing `rlgym-tools` Python converter or `rust-carball` implementation until the user supplies them for comparison.

## Replay and evaluation rules

- Use `replays/train` for inspection, calibration, and design. Use `replays/validation` to check generalization after choices are fixed. Keep `replays/test` sealed until the converter and evaluation protocol are frozen for the final assessment, unless the user explicitly changes this rule.
- Replay observations are evidence. Keep field freshness and provenance; absent values are unknown, not zero. Keep observed controls, scoreboard, clock, and events distinct from inferred controls and simulated events. Final header scores must not leak into earlier frames.
- Keep RocketSim's 120 Hz simulation timeline separate from replay packet cadence and offline motion-derived intervals. Do not treat a fitted interval or rounded tick count as an observed packet timestamp.
- Compare changes on matched samples against a meaningful baseline. Report counts, p50/p90/p99, per-game-size and per-replay behavior, and material regressions. A fit that uses the target value to choose its parameters is not a prediction result. Label any use of future observations; distinguish offline reconstruction from causal masked prediction.
- Investigate outliers on train, decide using validation, and document unsuccessful experiments. Do not enable a physics or timing correction solely because a pooled median improves.
- For material masked-prediction errors or regressions, inspect short train windows frame by frame before attributing a cause. Check the original and withheld field source frames, controls available before each simulated interval, actor lifetime, contact/event provenance, and the error trajectory across the window. Confirm that trace sample keys match the aggregate evaluator; include counterexamples when a proposed correction helps some windows and harms others.

## Working practice

- Never modify RocketSim itself; keep the dependency pin and work around differences in this repository's own code. When RocketSim disagrees with exact replay packets (and the cause is not an unobserved input), record it in `ROCKETSIM_NOTES.md` with a reproduction, the RocketSim source location at the pinned revision, the measured effect, the workaround, and a verified/suspected/open status, so it can be passed to the RocketSim developers.

- Start a new branch from the latest reviewed tip for a distinct work stream. Do not return to `master` merely because it is behind. Make naturally segmented commits that include the corresponding plan and results updates. Report branch and commit IDs when handing off.
- Keep replay files, collision meshes, generated datasets, and reports under ignored local paths such as `target/`; never commit them. Commit reproducible source and the commands needed to regenerate important reports.
- Run focused Rust tests and the relevant train/validation replay checks for behavioral changes. Use `cargo test --all-targets` before committing a completed implementation step. Leave the working tree clean when practical.
- Prefer a small, reviewable change over a broad rewrite. Preserve other agents' work in the shared workspace and inspect the working tree before editing or switching branches.

## Sub-agents

- Delegate bounded exploration, source audits, corpus summaries, and documentation checks when they can proceed independently. Prefer the smaller `gpt-6-luna` model for these tasks when available; use the stronger main agent for integration, difficult inference, and acceptance decisions.
- A model override needs a limited-context fork. Give a smaller sub-agent a self-contained task with relevant paths, the data-split rule, expected deliverable, and whether it may edit files. Do not assume it inherited the whole conversation.
- All agents share one filesystem and Git worktree. Keep exploratory sub-agents read-only or assign nonoverlapping files. Only the integrating agent should switch branches and make the final commit for a shared step.
