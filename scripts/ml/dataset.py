"""Per-tick training samples for a learned input model, from recordings with the true inputs.

One sample per car and simulated tick in play: the features are what the replay shows in a window of frames around the
tick (the car's bodies relative to its pose at the start of the tick's interval, update ages, the network inputs and
action counters) plus the simulation's flags at the start of the interval; the label is the input the server applied
from that tick (the RLBot or RocketSim recording, aligned as in `scripts/rlbot_input_ticks.py`). replicar's own
controls of the tick are kept for comparison.

usage: python scripts/ml/dataset.py <truth> <file.parquet> <out.npz>   (PYTHONPATH=python/v2/src; the file needs
`--with network` and every tick)
"""

import collections
import sys
from pathlib import Path

import numpy as np

import replicar

sys.path.insert(0, str(Path(__file__).parent.parent))
from rlbot_input_ticks import truth  # noqa: E402

CONTEXT = (-2, -1, 0, 1, 2)  # frames around the tick's interval: -1 starts it, 0 ends it
CONTROLS = ("throttle", "steer", "pitch", "yaw", "roll", "jump", "boost", "handbrake")
TRUTH_COLUMN = {"throttle": 3, "steer": 4, "boost": 5, "jump": 6, "handbrake": 7, "pitch": 8, "yaw": 9, "roll": 10}
LAG = 1  # the packet after a server tick holds the input applied from it
AIR_SOURCES = ("none", "steer", "persisted", "lookahead", "schedule", "press", "dodge", "flip_cancel")
GROUND_SOURCES = ("network", "schedule", "dodge")


def quat_to_matrix(q):
    """(n, 4) x, y, z, w -> (n, 3, 3) with the car's forward, right, up as columns."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def nan0(x):
    return np.nan_to_num(np.asarray(x, dtype=np.float32), nan=0.0, posinf=0.0, neginf=0.0)


def features(a, p, rows, frame_rows):
    """The feature matrix of `rows` (tick rows of player `p`); `frame_rows`: the file's frame-row indices in order."""
    frame_of_row = a["frame"]
    pos_in_frames = {int(frame_of_row[i]): k for k, i in enumerate(frame_rows)}
    replay_tick = a["replay_tick"].astype(np.float64)
    seg = a["segment"]
    k_end = np.array([pos_in_frames.get(int(frame_of_row[r]), -1) for r in rows])
    ok = k_end > 0
    rows, k_end = rows[ok], k_end[ok]
    ref = frame_rows[k_end - 1]  # the frame row that starts the interval
    q_ref = nan0(a["network_car_rotation"][ref, p])
    q_ref[np.linalg.norm(q_ref, axis=1) < 0.5] = [0, 0, 0, 1]
    m_ref = quat_to_matrix(q_ref)  # world <- car
    pos_ref = nan0(a["network_car_position"][ref, p])
    to_car = lambda v: np.einsum("nji,nj->ni", m_ref, v)  # car <- world
    cols = []
    for c in CONTEXT:
        k = np.clip(k_end + c, 0, len(frame_rows) - 1)
        i = frame_rows[k]
        valid = ((k_end + c) >= 0) & ((k_end + c) < len(frame_rows)) & (seg[i] == seg[rows])
        v = valid[:, None].astype(np.float32)
        fresh = (a["network_car_position_frame"][i, p] == frame_of_row[i]).astype(np.float32)
        q = nan0(a["network_car_rotation"][i, p])
        q[np.linalg.norm(q, axis=1) < 0.5] = [0, 0, 0, 1]
        m = quat_to_matrix(q)
        rel = np.einsum("nji,njk->nik", m_ref, m)  # car_ref <- car_i
        upd = a["car_update_tick"][i, p].astype(np.float64)
        cols += [
            v,
            (replay_tick[i] - replay_tick[rows])[:, None] / 10 * v,
            (np.nan_to_num(upd - replay_tick[rows]) / 10)[:, None] * v,
            fresh[:, None] * v,
            to_car(nan0(a["network_car_position"][i, p]) - pos_ref) / 100 * v,
            to_car(nan0(a["network_car_velocity"][i, p])) / 1000 * v,
            to_car(nan0(a["network_car_angular_velocity_raw"][i, p]) * 0.01) * v,
            rel[:, :, 0] * v, rel[:, :, 2] * v,  # forward and up in the reference frame
            nan0(a["network_car_throttle"][i, p])[:, None] * v,
            nan0(a["network_car_steer"][i, p])[:, None] * v,
            nan0(a["network_car_handbrake"][i, p])[:, None] * v,
        ]
        for counter in ("boost_active", "jump_active", "double_jump_active", "dodge_active"):
            raw = a[f"network_car_{counter}_raw"][i, p].astype(np.float64)
            cols.append((np.nan_to_num(raw) % 2 == 1).astype(np.float32)[:, None] * v)
            cols.append((a[f"network_car_{counter}_raw_frame"][i, p] == frame_of_row[i]).astype(np.float32)[:, None] * v)
        cols.append(nan0(a["network_car_dodge_torque_raw"][i, p]) / 100 * v)
    # The tick within its interval, and the car where the interval starts.
    start, end = replay_tick[ref], replay_tick[frame_rows[k_end]]
    cols += [
        ((replay_tick[rows] - start) / 10)[:, None],
        ((end - start) / 10)[:, None],
        m_ref[:, :, 2],  # up, in the world
        pos_ref[:, 2:3] / 1000,
    ]
    for flag in ("is_on_ground", "has_jumped", "has_double_jumped", "has_flipped", "is_flipping", "is_demoed"):
        cols.append(nan0(a[f"car_{flag}"][ref, p])[:, None])
    wheels = sum(nan0(a[f"car_wheel_{w}_contact"][ref, p]) for w in range(4))
    cols.append(wheels[:, None] / 4)
    return rows, np.concatenate(cols, axis=1).astype(np.float32)


def main() -> None:
    truth_path, path, out = sys.argv[1], sys.argv[2], sys.argv[3]
    true, _ = truth(truth_path)
    f = replicar.read(path)
    a = f.arrays()
    key = lambda x: (round(x[0] * 100), round(x[1] * 100), round(x[2] * 100))
    frame_rows = np.flatnonzero(a["frame_row"].astype(bool))
    replay_tick = a["replay_tick"].astype(np.int64)
    xs, ys, base, meta, extras = [], [], [], [], []
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
        rows = np.flatnonzero((a["segment"] >= 0) & ~np.isnan(position[:, 0]) & (a["car_is_demoed"][:, p] != 1))
        rows, x = features(a, p, rows, frame_rows)
        i = np.searchsorted(match_tick, replay_tick[rows])
        local = np.array([np.median(match_offset[max(0, j - 50):j + 51]) for j in i]).round().astype(np.int64)
        idx = np.array([index_of.get(s, -1) for s in replay_tick[rows] - local + LAG])
        keep = idx >= 0
        y = np.stack([values[idx[keep], TRUTH_COLUMN[c]] for c in CONTROLS], axis=1).astype(np.float32)
        b = np.stack([nan0(a[f"car_controls_{c}"][rows[keep], p]) for c in CONTROLS], axis=1)
        xs.append(x[keep]); ys.append(y); base.append(b)
        # replicar's own view of the tick: how its controls were set, and the simulation's flags.
        r = rows[keep]
        air_source = np.array(f.table.column(f"car_{p}_air_controls_source").to_pylist(), dtype=object)[r]
        ground_source = np.array(f.table.column(f"car_{p}_ground_controls_source").to_pylist(), dtype=object)[r]
        extra = [np.stack([(air_source == s).astype(np.float32) for s in AIR_SOURCES], axis=1),
                 np.stack([(ground_source == s).astype(np.float32) for s in GROUND_SOURCES], axis=1)]
        for flag in ("is_on_ground", "has_jumped", "has_double_jumped", "has_flipped", "is_flipping", "is_jumping"):
            extra.append(nan0(a[f"car_{flag}"][r, p])[:, None])
        extra.append(sum(nan0(a[f"car_wheel_{w}_contact"][r, p]) for w in range(4))[:, None] / 4)
        extras.append(np.concatenate(extra, axis=1))
        ground = a["car_is_on_ground"][rows[keep], p].astype(np.float32)
        meta.append(np.stack([np.full(keep.sum(), p), replay_tick[rows[keep]], ground], axis=1))
    np.savez_compressed(out, x=np.concatenate(xs), y=np.concatenate(ys), baseline=np.concatenate(base),
                        meta=np.concatenate(meta), extra=np.concatenate(extras))
    print(out, "samples", sum(len(x) for x in xs), "features", xs[0].shape[1])


if __name__ == "__main__":
    main()
