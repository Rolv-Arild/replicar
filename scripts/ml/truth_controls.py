"""The true inputs of a recording aligned to a converted file's sim ticks, as CSV for the Rust tools (`slot`,
`sim_tick`, then throttle, steer, pitch, yaw, roll, jump, boost, handbrake): the input applied from that tick, aligned
as in `scripts/rlbot_input_ticks.py`; beside it (`.updates.csv`: `slot`, `frame`, `sim_tick`) the true tick of every
car update that matches exactly one server state.

usage: python scripts/ml/truth_controls.py <truth> <file.parquet> <out.csv>   (PYTHONPATH=python/v2/src)
"""

import collections
import sys
from pathlib import Path

import numpy as np

import replicar

sys.path.insert(0, str(Path(__file__).parent.parent))
from rlbot_input_ticks import truth  # noqa: E402

CONTROLS = ("throttle", "steer", "pitch", "yaw", "roll", "jump", "boost", "handbrake")
TRUTH_COLUMN = {"throttle": 3, "steer": 4, "boost": 5, "jump": 6, "handbrake": 7, "pitch": 8, "yaw": 9, "roll": 10}
LAG = 1


def main() -> None:
    truth_path, path, out = sys.argv[1], sys.argv[2], sys.argv[3]
    true, _ = truth(truth_path)
    f = replicar.read(path)
    a = f.arrays()
    key = lambda x: (round(x[0] * 100), round(x[1] * 100), round(x[2] * 100))
    replay_tick = a["replay_tick"].astype(np.int64)
    sim_tick = a["sim_tick"].astype(np.int64)
    lines, updates = [], []
    for info in f.players:
        p, name = info["index"], info.get("name")
        position = a["car_position"][:, p]
        updated = np.flatnonzero((a["segment"] >= 0) & (a["car_updated"][:, p] == 1) & ~np.isnan(position[:, 0]))
        if name not in true:
            best, count = None, 0
            for candidate, (_, values) in true.items():
                keys = {key(v) for v in values[:, :3]}
                c = sum(key(position[r]) in keys for r in updated)
                if c > count:
                    best, count = candidate, c
            if count < 20:
                continue
            name = best
        ticks_true, values = true[name]
        index_of = {t: i for i, t in enumerate(ticks_true)}
        where = collections.defaultdict(list)
        for i, t in enumerate(ticks_true):
            where[key(values[i, :3])].append(t)
        matched = sorted((int(replay_tick[r]), int(replay_tick[r]) - where[key(position[r])][0])
                         for r in updated if len(where.get(key(position[r]), [])) == 1)
        if len(matched) < 20:
            continue
        match_tick = np.array([m[0] for m in matched])
        match_offset = np.array([m[1] for m in matched])
        rows = np.flatnonzero((a["segment"] >= 0) & ~np.isnan(position[:, 0]))
        i = np.searchsorted(match_tick, replay_tick[rows])
        local = np.array([np.median(match_offset[max(0, j - 50):j + 51]) for j in i]).round().astype(np.int64)
        idx = np.array([index_of.get(s, -1) for s in replay_tick[rows] - local + LAG])
        for r, k in zip(rows, idx):
            if k >= 0:
                v = values[k]
                lines.append(f"{p},{sim_tick[r]}," + ",".join(f"{v[TRUTH_COLUMN[c]]:.6g}" for c in CONTROLS))
        # The true tick of each update that matches exactly one server tick: the row that applied it sits at the
        # inferred update tick; the server tick (with the local offset) gives the true one, as a sim tick.
        update_tick = a["car_update_tick"][:, p].astype(np.int64)
        for r in updated:
            hits = where.get(key(position[r]))
            if hits and len(hits) == 1:
                j = np.searchsorted(match_tick, replay_tick[r])
                local = int(round(np.median(match_offset[max(0, j - 50):j + 51])))
                true_replay = hits[0] + local
                updates.append(f"{p},{int(a['frame'][r])},{sim_tick[r] + (true_replay - update_tick[r])}")
    Path(out).write_text("\n".join(lines) + "\n")
    Path(out).with_suffix(".updates.csv").write_text("\n".join(updates) + "\n")
    print(out, "ticks", len(lines), "updates with a true tick", len(updates))


if __name__ == "__main__":
    main()
