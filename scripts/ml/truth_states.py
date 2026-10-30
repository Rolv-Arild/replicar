"""The true server state of every car and of the ball at each sim tick of a converted file, from an RLBot recording
(`states.jsonl`), as CSV for `replicar-eval identifiability` (`slot` (-1: the ball), `sim_tick`, position, velocity,
rotation matrix row-major (columns forward, right, up), angular velocity, boost). Each body's server ticks are mapped
to sim ticks through its updates that match exactly one server state (the true tick of an update as in
`truth_controls.py`), with the median offset of the 101 nearest within its play segment. Prints, per body, how well
the mapped states reproduce the file's updates and how much the offset varies within a segment.

usage: python scripts/ml/truth_states.py <states.jsonl> <file.parquet> <out.csv>   (PYTHONPATH=python/v2/src)
"""

import collections
import json
import sys
from pathlib import Path

import numpy as np

import replicar


def rotation_matrix(pitch: float, yaw: float, roll: float) -> np.ndarray:
    """Columns forward, right and up of an Unreal rotation in radians."""
    cp, sp, cy, sy, cr, sr = np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw), np.cos(roll), np.sin(roll)
    forward = np.array([cp * cy, cp * sy, sp])
    right = np.array([cy * sp * sr - cr * sy, sy * sp * sr + cr * cy, -cp * sr])
    up = np.array([-cr * cy * sp - sr * sy, -cr * sy * sp + sr * cy, cp * cr])
    return np.column_stack([forward, right, up])


def quaternion_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def physics(ph: dict, boost: float):
    l, r, v, w = ph["location"], ph["rotation"], ph["velocity"], ph["angular_velocity"]
    return (np.array([l["x"], l["y"], l["z"]]), np.array([v["x"], v["y"], v["z"]]),
            rotation_matrix(r["pitch"], r["yaw"], r["roll"]), np.array([w["x"], w["y"], w["z"]]), boost)


def main() -> None:
    states_path, path, out = sys.argv[1], sys.argv[2], sys.argv[3]
    true = collections.defaultdict(dict)
    with open(states_path, encoding="utf-8") as f:
        for line in f:
            packet = json.loads(line)["packet"]
            n = packet["match_info"]["frame_num"]
            for pl in packet["players"]:
                true[pl["name"]][n] = physics(pl["physics"], pl["boost"])
            if packet.get("balls"):
                true["__ball__"][n] = physics(packet["balls"][0]["physics"], 0.0)
    f = replicar.read(path)
    a = f.arrays()
    key = lambda x: (round(x[0] * 100), round(x[1] * 100), round(x[2] * 100))
    replay_tick = a["replay_tick"].astype(np.int64)
    sim_tick = a["sim_tick"].astype(np.int64)
    segment = a["segment"]
    lines = []
    bodies = [(info["index"], info.get("name")) for info in f.players] + [(-1, "__ball__")]
    for slot, name in bodies:
        prefix = "ball" if slot < 0 else "car"
        column = lambda c: a[f"{prefix}_{c}"] if slot < 0 else a[f"{prefix}_{c}"][:, slot]
        position, velocity, rotation, angular = (column(c) for c in ("position", "velocity", "rotation",
                                                                      "angular_velocity"))
        updated = np.flatnonzero((segment >= 0) & (column("updated") == 1) & ~np.isnan(position[:, 0]))
        update_tick = column("update_tick")
        if name not in true:
            # A recording without the name: the recorded body whose positions match the file's updates.
            best, count = None, 0
            for candidate, by_tick in true.items():
                keys = {key(v[0]) for v in by_tick.values()}
                c = sum(key(position[r]) in keys for r in updated)
                if c > count:
                    best, count = candidate, c
            if count < 20:
                continue
            name = best
        by_tick = true[name]
        where = collections.defaultdict(list)
        for t, v in by_tick.items():
            where[key(v[0])].append(t)
        matched = sorted((int(replay_tick[r]), int(replay_tick[r]) - where[key(position[r])][0], r)
                         for r in updated if len(where.get(key(position[r]), [])) == 1)
        if len(matched) < 20:
            continue
        match_tick = np.array([m[0] for m in matched])
        match_offset = np.array([m[1] for m in matched])
        # (true sim tick, server tick, row) of each matched update.
        pairs = []
        for t, _, r in matched:
            server = where[key(position[r])][0]
            j = np.searchsorted(match_tick, t)
            local = int(round(np.median(match_offset[max(0, j - 50):j + 51])))
            pairs.append((int(sim_tick[r] + (server + local - int(update_tick[r]))), server, r))
        pairs.sort()
        check = collections.defaultdict(list)
        for _, server, r in pairs[::7]:
            p, v, m, w, _ = by_tick[server]
            check["position"].append(np.linalg.norm(p - position[r]))
            check["velocity"].append(np.linalg.norm(v - velocity[r]))
            delta = m.T @ quaternion_matrix(rotation[r].astype(np.float64))
            check["rotation"].append(np.degrees(np.arccos(np.clip((np.trace(delta) - 1) / 2, -1, 1))))
            check["angular"].append(np.linalg.norm(w - angular[r]))
        spread = []
        for s in np.unique(segment[segment >= 0]):
            in_segment = [q for q in pairs if segment[q[2]] == s]
            if len(in_segment) < 5:
                continue
            true_ticks = np.array([q[0] for q in in_segment])
            offsets = np.array([q[0] - q[1] for q in in_segment])
            spread.extend(offsets - np.median(offsets))
            ticks = np.unique(sim_tick[segment == s])
            for t, j in zip(ticks, np.searchsorted(true_ticks, ticks)):
                offset = int(round(np.median(offsets[max(0, j - 50):j + 51])))
                state = by_tick.get(int(t) - offset)
                if state is None:
                    continue
                p, v, m, w, boost = state
                values = list(p) + list(v) + list(m.reshape(-1)) + list(w) + [boost]
                lines.append(f"{slot},{t}," + ",".join(f"{x:.7g}" for x in values))
        spread = np.abs(np.array(spread))
        print(f"slot {slot} ({name}): {len(pairs)} matched updates; at them |position| p50 "
              f"{np.median(check['position']):.3f}, |velocity| {np.median(check['velocity']):.3f}, rotation "
              f"{np.median(check['rotation']):.3f} deg, |angular| {np.median(check['angular']):.4f}; offset within "
              f"its segment |deviation| p50 / p90 / p99 {np.median(spread):.0f} / {np.percentile(spread, 90):.0f} / "
              f"{np.percentile(spread, 99):.0f} ticks")
    Path(out).write_text("\n".join(lines) + "\n")
    print(out, len(lines), "states")


if __name__ == "__main__":
    main()
