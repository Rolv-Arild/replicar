"""Takeoffs without a jump input: for every row where a car leaves the ground and `has_jumped` turns on (within 3
rows), whether `car_controls_jump` was on in the 14 rows before, split by kickoff (the first 3 s of a segment) and open
play, and by the ground controls source of the row before. A file-only check (the downstream project's metric for
"first jumps missing"); a row file with every tick is needed.

usage: python scripts/check_takeoffs.py <file.parquet>...   (PYTHONPATH=python/v2/src)
"""

import collections
import sys

import numpy as np

import replicar


def main() -> None:
    counts = collections.Counter()
    sources = collections.Counter()
    for path in sys.argv[1:]:
        f = replicar.read(path)
        a = f.arrays()
        seg = a["segment"]
        tick = a["sim_tick"].astype(np.int64)
        first = {}
        for i, s in enumerate(seg):
            first.setdefault(s, tick[i])
        for p in range(len(f.players)):
            ground = a["car_is_on_ground"][:, p] == 1
            jumped = a["car_has_jumped"][:, p] == 1
            jump = a["car_controls_jump"][:, p] == 1
            source = f.table.column(f"car_{p}_ground_controls_source").to_pylist()
            for r in np.flatnonzero(ground[:-1] & ~ground[1:]) + 1:
                if r < 15 or not jumped[r:r + 3].any() or jumped[r - 15] or seg[r] != seg[r - 14]:
                    continue
                where = "kickoff" if tick[r] - first[seg[r]] < 360 else "open play"
                has_press = bool(jump[r - 14:r + 1].any())
                counts[(where, has_press)] += 1
                if not has_press:
                    sources[(where, source[r - 1])] += 1
    for where in ("kickoff", "open play"):
        n = counts[(where, True)] + counts[(where, False)]
        print(f"{where:9}: {n:6} takeoffs, without a jump input in the 14 rows before {counts[(where, False)]:5}"
              f" ({counts[(where, False)] / max(n, 1):.1%})")
        for (w, s), v in sorted(sources.items(), key=lambda x: -x[1]):
            if w == where:
                print(f"      source on the row before: {s:10} {v}")


if __name__ == "__main__":
    main()
