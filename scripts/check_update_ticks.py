"""The inferred tick of every car update against the truth of an RLBot or RocketSim recording.

An update's position equals the server's at exactly one tick, which gives the true tick of each update (up to one
offset per player between replay and server ticks, taken as the median over the 101 nearest updates). Reported:
the error of the inferred update tick, by how it was inferred (`chain`: the car's own update-tick chain, listed in
the resimulation group; `median`: the frame's median car, or half the frame gap), and on the ground or in the air.

usage: python scripts/check_update_ticks.py <states.jsonl or .rlpr> <file.parquet> [label]   (PYTHONPATH=python/v2/src;
the file needs --with resimulation)
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
    chains = f.records("resim_car_update_ticks").to_pydict()
    chained = set(zip(chains["frame"], chains["actor"]))
    actor = a["network_car_actor"] if "network_car_actor" in a else None
    key = lambda x: (round(x[0] * 100), round(x[1] * 100), round(x[2] * 100))
    errors = collections.defaultdict(list)
    names = {name: name for name in true}
    for info in f.players:
        p = info["index"]
        name = info.get("name")
        position = a["car_position"][:, p]
        rows = np.flatnonzero((a["segment"] >= 0) & (a["car_updated"][:, p] == 1) & ~np.isnan(position[:, 0]))
        if name not in names:
            # A recording without names: the recorded car whose positions match.
            best, count = None, 0
            for candidate, (_, values) in true.items():
                keys = {key(v) for v in values[:, :3]}
                c = sum(key(position[r]) in keys for r in rows)
                if c > count:
                    best, count = candidate, c
            if count < 20:
                continue
            name = best
        ticks_true, values = true[name]
        where = collections.defaultdict(list)
        for i, t in enumerate(ticks_true):
            where[key(values[i, :3])].append(t)
        update_tick = a["car_update_tick"][:, p].astype(np.int64)
        matched = []
        for r in rows:
            hits = where.get(key(position[r]))
            if hits and len(hits) == 1:
                source = "chain" if actor is not None and (int(a["frame"][r]), int(actor[r, p])) in chained else "median"
                ground = "ground" if a["car_is_on_ground"][r, p] == 1 else "air"
                matched.append((int(update_tick[r]), int(update_tick[r]) - hits[0], source, ground))
        if len(matched) < 20:
            continue
        matched.sort()
        ticks = np.array([m[0] for m in matched])
        offsets = np.array([m[1] for m in matched])
        for i, (_, offset, source, ground) in enumerate(matched):
            local = np.median(offsets[max(0, i - 50):i + 51])
            error = int(round(offset - local))
            errors[(source, ground)].append(error)
            errors[("all", "all")].append(error)
    print(f"== {label}")
    for k in sorted(errors):
        e = np.array(errors[k])
        exact = (e == 0).mean()
        print(f"  {k[0]:6} {k[1]:6} n {len(e):6}  exact {exact:.3f}  within 1 {(np.abs(e) <= 1).mean():.3f}"
              f"  p05/p95 {np.percentile(e, 5):+.0f}/{np.percentile(e, 95):+.0f}")


if __name__ == "__main__":
    main()
