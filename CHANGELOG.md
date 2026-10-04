# Changelog

## 1.0.0 (2026-10-04)

First release.

- Converts standard soccar replays into one RocketSim state per network frame: replay packets placed on their inferred server tick, RocketSim (`9910c58`) simulating every 120 Hz tick in between, and unrecorded inputs (control, jump and dodge timing, flip cancels, air pitch/yaw/roll) fitted against later packets.
- Exports JSON Lines or Parquet with seven record tables, observations with their freshness, replay events, the scoreboard with a fitted fractional clock, freshness columns, observed ping, and future-derived training labels. Python loaders for both formats; exact restoration of a RocketSim state from an exported frame.
- Evaluated once on 60 sealed test replays (tag `test-assessment-1`; RESULTS.md, "Test-split assessment"): every replay converted and the errors matched the development splits. The conversion state is unchanged since that run.
- Release clean-up (since tag `pre-cleanup`): the one-off experiment tools and the measured-and-rejected options were removed, `ConvertOptions` went from 63 to 22 fields, `conversion.rs` was split into modules, and the code is formatted and clippy-clean. Default output is byte-identical to before the clean-up on all 120 development replays.
