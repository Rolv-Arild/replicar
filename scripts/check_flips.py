"""Flips against the replay's dodges: for every dodge the replay reports (a car's dodge counter turning odd on a frame
row), whether the simulation starts a flip (`is_flipping` turns on) within 30 ticks before to 15 after the frame, and
whether it instead only sets `has_flipped` (a flip without a direction); plus the airborne rows whose flags are ones
the game cannot produce (`has_flipped` and `has_double_jumped` both on: a flip after the double jump without a reset).
A file-only check of flip availability (the downstream feedback item 18); a row file with every tick and the network
group is needed.

usage: python scripts/check_flips.py <file.parquet>...   (PYTHONPATH=python/v2/src)
"""

import collections
import sys

import numpy as np

import replicar


def main() -> None:
    counts = collections.Counter()
    for path in sys.argv[1:]:
        a = replicar.read(path).arrays()
        tick = a["sim_tick"].astype(np.int64)
        frame_row = a["frame_row"].astype(bool)
        dodge = a["network_car_dodge_active_raw"]
        flipping = a["car_is_flipping"].astype(bool)
        flipped = a["car_has_flipped"].astype(bool)
        double = a["car_has_double_jumped"].astype(bool)
        ground = a["car_is_on_ground"].astype(bool)
        for p in range(dodge.shape[1]):
            starts = tick[1:][flipping[1:, p] & ~flipping[:-1, p]]
            bare = tick[1:][flipped[1:, p] & ~flipped[:-1, p] & ~flipping[1:, p]]
            last = None
            for r in np.where(frame_row)[0]:
                v = dodge[r, p]
                if v is None or v < 0:
                    continue
                v = int(v)
                if last is not None and v != last and v % 2 == 1:
                    counts["dodges"] += 1
                    t = tick[r]
                    if ((starts >= t - 30) & (starts <= t + 15)).any():
                        counts["simulated flip"] += 1
                    elif ((bare >= t - 30) & (bare <= t + 15)).any():
                        counts["flag only (no direction)"] += 1
                    else:
                        counts["no flip"] += 1
                last = v
            counts["airborne rows"] += int((~ground[:, p]).sum())
            counts["airborne rows flipped after double jump"] += int((~ground[:, p] & flipped[:, p] & double[:, p]).sum())
    d = counts["dodges"]
    print(f"{len(sys.argv) - 1} files, {d} replay dodges")
    for k in ["simulated flip", "flag only (no direction)", "no flip"]:
        print(f"  {k}: {counts[k]} ({100 * counts[k] / max(d, 1):.1f}%)")
    n = counts["airborne rows"]
    k = "airborne rows flipped after double jump"
    print(f"{k}: {counts[k]} of {n} ({100 * counts[k] / max(n, 1):.2f}%)")


if __name__ == "__main__":
    main()
