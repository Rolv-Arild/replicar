"""Kickoff starts against the truth of an RLBot recording: for every play segment's first row (the release: the cars
at rest on their spawns, with the action they start with), the server tick it maps to against the server's last
tick at rest before the car moves, and its controls against the input the server applied next.

Rows map to server ticks as in `rlbot_input_ticks.py` (one offset per player from the exactly matching packets);
the truth release is the last server tick before the car's position leaves its spawn by more than 0.05 UU.

usage: python scripts/check_kickoffs.py <states.jsonl> <file.parquet> [label]   (PYTHONPATH=python/v2/src)
"""

import collections
import sys
from pathlib import Path

import numpy as np

import replicar

sys.path.insert(0, str(Path(__file__).parent))
from rlbot_input_ticks import truth  # noqa: E402


def main() -> None:
    states, path = sys.argv[1], sys.argv[2]
    label = sys.argv[3] if len(sys.argv) > 3 else path
    true, _ = truth(states)
    f = replicar.read(path)
    a = f.arrays()
    segment = a["segment"]
    starts = [i for i in range(len(segment)) if segment[i] >= 0 and (i == 0 or segment[i] != segment[i - 1])]
    replay_tick = a["replay_tick"].astype(np.int64)
    key = lambda x: (round(x[0] * 100), round(x[1] * 100), round(x[2] * 100))
    errors = collections.Counter()
    controls = collections.Counter()
    for info in f.players:
        name, p = info.get("name"), info["index"]
        if name not in true:
            continue
        ticks_true, values = true[name]
        index_of = {t: i for i, t in enumerate(ticks_true)}
        position = a["car_position"][:, p]
        rows = np.flatnonzero((segment >= 0) & (a["car_updated"][:, p] == 1) & ~np.isnan(position[:, 0]))
        where = collections.defaultdict(list)
        for i, t in enumerate(ticks_true):
            where[key(values[i, :3])].append(t)
        matched = sorted((int(replay_tick[r]), int(replay_tick[r]) - where[key(position[r])][0])
                         for r in rows if len(where.get(key(position[r]), [])) == 1)
        if len(matched) < 20:
            continue
        match_tick = np.array([m[0] for m in matched])
        match_offset = np.array([m[1] for m in matched])
        for s in starts:
            if np.isnan(position[s, 0]):
                continue
            i = np.searchsorted(match_tick, replay_tick[s])
            offset = int(round(np.median(match_offset[max(0, i - 50):i + 51])))
            server = int(replay_tick[s]) - offset
            # The truth release: from the server tick nearest the row, the last tick at the spawn before it moves.
            j = index_of.get(server)
            if j is None:
                continue
            spawn = values[j, :3]
            lo = j
            while lo > 0 and np.linalg.norm(values[lo - 1, :3] - spawn) < 0.05:
                lo -= 1
            hi = j
            while hi + 1 < len(values) and np.linalg.norm(values[hi + 1, :3] - values[lo, :3]) < 0.05:
                hi += 1
            if np.linalg.norm(position[s] - values[hi, :3]) > 5.0:
                errors["row not at the spawn"] += 1
                continue
            errors[int(server - ticks_true[hi])] += 1
            # The input the server applied from its release tick (in the packet after it), against the row's.
            k = index_of.get(int(ticks_true[hi]) + 1)
            if k is not None:
                controls["throttle"] += abs(a["car_controls_throttle"][s, p] - values[k, 3]) < 0.01
                controls["steer"] += abs(a["car_controls_steer"][s, p] - values[k, 4]) < 0.05
                controls["n"] += 1
    print(f"== {label}")
    print("  release row minus the truth release, server ticks:",
          dict(sorted(errors.items(), key=lambda x: (isinstance(x[0], str), x[0]))))
    n = max(controls["n"], 1)
    print(f"  starting action against the true input: throttle within 0.01 {controls['throttle'] / n:.2f}, "
          f"steer within 0.05 {controls['steer'] / n:.2f} (n {controls['n']})")


if __name__ == "__main__":
    main()
