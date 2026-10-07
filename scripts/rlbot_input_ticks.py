"""The converter's per-tick controls against the true inputs of an RLBot recording (the server's `last_input` per
tick), for each player, per tick in active play.

The converter's tick rows are mapped to server ticks by one offset per player, the one at which the car's position
best matches the server's (median over ticks with the car on its own update tick): it depends on the states, not
on the controls, so every variant of the controls is scored on the same alignment. `last_input` of server packet
n+1 is the input applied from tick n to n+1, the same as a row's controls (applied in the step after its tick).

Reported per player group (bots, humans): throttle and steer mean absolute error and share within 0.01; boost,
jump and handbrake share of ticks that disagree; and the tick error of each press (false to true) of boost and
jump: the nearest converted press within 30 ticks, p10 / p50 / p90 and the share found.

usage: python scripts/rlbot_input_ticks.py <states.jsonl> <file.parquet> [label]   (PYTHONPATH=python/v2/src)
"""

import collections
import json
import os
import sys
from pathlib import Path

import numpy as np

import replicar


def truth_rlpr(path):
    """Per recorded car (`car<i>`): arrays by physics frame of position and inputs, from a RocketSim `.rlpr`."""
    sys.path.insert(0, str(Path(__file__).parent))
    import rlpr
    r = rlpr.load(path)
    out = {}
    c = r["controls"]
    for i in range(r["pos"].shape[1]):
        values = np.column_stack([r["pos"][:, i], c[:, i, 0], c[:, i, 1], c[:, i, 6], c[:, i, 5], c[:, i, 7]])
        out[f"car{i}"] = (r["frame"].astype(np.int64), values.astype(np.float64))
    return out, {}


def truth(path):
    """Per player name: arrays by server tick (frame_num) of position, inputs and is_bot."""
    if str(path).endswith(".rlpr"):
        return truth_rlpr(path)
    rows = collections.defaultdict(dict)
    bots = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            p = json.loads(line)["packet"]
            if p["match_info"].get("match_phase") not in (2, 3):  # active, kickoff
                pass
            n = p["match_info"]["frame_num"]
            for pl in p["players"]:
                li = pl["last_input"]
                loc = pl["physics"]["location"]
                rows[pl["name"]][n] = (loc["x"], loc["y"], loc["z"], li["throttle"], li["steer"],
                                       li["boost"], li["jump"], li["handbrake"])
                bots[pl["name"]] = pl["is_bot"]
    out = {}
    for name, by_tick in rows.items():
        ticks = np.array(sorted(by_tick))
        values = np.array([by_tick[t] for t in ticks], dtype=np.float64)
        out[name] = (ticks, values)
    return out, bots


def presses(values: np.ndarray) -> np.ndarray:
    v = values.astype(bool)
    return np.flatnonzero(v[1:] & ~v[:-1]) + 1


def releases(values: np.ndarray) -> np.ndarray:
    v = values.astype(bool)
    return np.flatnonzero(~v[1:] & v[:-1]) + 1


def main() -> None:
    states, path = sys.argv[1], sys.argv[2]
    label = sys.argv[3] if len(sys.argv) > 3 else path
    true, bots = truth(states)
    f = replicar.read(path)
    a = f.arrays()
    if f.header.get("rows") != "ticks" or f.header.get("tick_step", 1) != 1:
        raise SystemExit("needs a file with every tick (--rows ticks)")
    segment = a["segment"]
    play = segment >= 0
    tick = a["sim_tick"].astype(np.int64)
    groups = collections.defaultdict(lambda: collections.defaultdict(list))
    lag = int(os.environ.get("INPUT_LAG", "1"))
    for info in f.players:
        name, p = info.get("name"), info["index"]
        if name not in true:
            # A recording without names (.rlpr): the recorded car whose positions match the player's packets.
            position = a["car_position"][:, p]
            rows = np.flatnonzero(play & (a["car_updated"][:, p] == 1) & ~np.isnan(position[:, 0]))
            key = lambda x: (round(x[0] * 100), round(x[1] * 100), round(x[2] * 100))
            best_name, best_count = None, 0
            for candidate, (_, values) in true.items():
                keys = {key(v) for v in values[:, :3]}
                count = sum(key(position[r]) in keys for r in rows)
                if count > best_count:
                    best_name, best_count = candidate, count
            if best_name is None or best_count < 20:
                continue
            bots[best_name] = "bot" in (name or "").lower() or name in ("Nexto", "London (GPU)")
            name = best_name
        ticks_true, values = true[name]
        index_of = {t: i for i, t in enumerate(ticks_true)}
        position = a["car_position"][:, p]
        updated = a["car_updated"][:, p] == 1
        rows = np.flatnonzero(play & updated & ~np.isnan(position[:, 0]))
        replay_tick = a["replay_tick"].astype(np.int64)
        # The offset d (server tick = replay tick - d): at a tick row where an update was applied the position is
        # the packet's, which equals the server's at one tick; the most common replay tick minus that tick.
        key = lambda x: (round(x[0] * 100), round(x[1] * 100), round(x[2] * 100))
        where = collections.defaultdict(list)
        for i, t in enumerate(ticks_true):
            where[key(values[i, :3])].append(t)
        matched = []
        for r in rows:
            hits = where.get(key(position[r]))
            if hits and len(hits) == 1:
                matched.append((int(replay_tick[r]), int(replay_tick[r]) - hits[0]))
        if len(matched) < 20:
            continue
        matched.sort()
        match_tick = np.array([m[0] for m in matched]); match_offset = np.array([m[1] for m in matched])
        # The offset drifts through a match: the median of the 101 matches around each tick.
        def offset_at(t):
            i = np.searchsorted(match_tick, t)
            lo, hi = max(0, i - 50), min(len(match_tick), i + 51)
            return np.median(match_offset[lo:hi])
        group = "bots" if bots.get(name) else "humans"
        g = groups[group]
        g["matched packets"].append(len(matched))
        rows = np.flatnonzero(play & ~np.isnan(position[:, 0]) & (a["car_is_demoed"][:, p] != 1))
        local = np.array([offset_at(t) for t in replay_tick[rows]]).round().astype(np.int64)
        server = replay_tick[rows] - local + lag  # RLBot: the packet after the tick holds its input (lag 1)
        idx = np.array([index_of.get(s, -1) for s in server])
        ok = idx >= 0
        rows, idx = rows[ok], idx[ok]
        tv = values[idx]
        ground = a["car_is_on_ground"][rows, p] == 1
        for k, name_k in ((3, "throttle"), (4, "steer")):
            err = np.abs(a[f"car_controls_{name_k}"][rows, p] - tv[:, k])
            g[f"{name_k} |error|"].extend(err.tolist())
            g[f"{name_k} |error| air"].extend(err[~ground].tolist())
        for k, name_k in ((5, "boost"), (6, "jump"), (7, "handbrake")):
            conv = a[f"car_controls_{name_k}"][rows, p] == 1
            g[f"{name_k} disagree"].extend((conv != tv[:, k].astype(bool)).tolist())
            # Press timing: on consecutive ticks only.
            run = np.flatnonzero(np.diff(tick[rows]) != 1)
            starts = np.concatenate([[0], run + 1]); ends = np.concatenate([run + 1, [len(rows)]])
            for s0, e0 in zip(starts, ends):
                if e0 - s0 < 3:
                    continue
                for kind, edge in (("press", presses), ("release", releases)):
                    tp = edge(tv[s0:e0, k]); cp = edge(conv[s0:e0])
                    for t in tp:
                        if len(cp) == 0:
                            g[f"{name_k} {kind} missed"].append(1)
                            continue
                        j = np.argmin(np.abs(cp - t))
                        if abs(cp[j] - t) <= 30:
                            g[f"{name_k} {kind} error"].append(int(cp[j] - t))
                            if not ground[s0 + t]:
                                g[f"{name_k} {kind} error air"].append(int(cp[j] - t))
                            g[f"{name_k} {kind} missed"].append(0)
                        else:
                            g[f"{name_k} {kind} missed"].append(1)
                continue
                for t in tp:
                    if len(cp) == 0:
                        g[f"{name_k} press missed"].append(1)
                        continue
                    j = np.argmin(np.abs(cp - t))
                    if abs(cp[j] - t) <= 30:
                        g[f"{name_k} press error"].append(int(cp[j] - t))
                        g[f"{name_k} press missed"].append(0)
                    else:
                        g[f"{name_k} press missed"].append(1)
    for group, g in sorted(groups.items()):
        print(f"== {label} {group}")
        for key in sorted(g):
            v = np.array(g[key], dtype=float)
            if key.endswith("|error|") or key.endswith("|error| air"):
                print(f"  {key:30} mean {v.mean():.4f}  within 0.01 {np.mean(v <= 0.01):.3f}  p90 {np.percentile(v, 90):.3f}  n {len(v)}")
            elif "press error" in key or "release error" in key:
                p10, p50, p90 = np.percentile(v, [10, 50, 90])
                print(f"  {key:30} p10 / p50 / p90 {p10:+.0f} / {p50:+.0f} / {p90:+.0f}  |e| mean {np.abs(v).mean():.2f}  n {len(v)}")
            else:
                print(f"  {key:30} {v.mean():.4f}  n {len(v)}")


if __name__ == "__main__":
    main()
