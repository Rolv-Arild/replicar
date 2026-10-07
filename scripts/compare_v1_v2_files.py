"""Compare a v2 replicar file with v1's Parquet export of the same replay, value for value (docs/v2-plan.md, story
7.1: "values equal to v1's Parquet columns on the same frames").

usage: python scripts/compare_v1_v2_files.py <v1.parquet> <v2.parquet> [<v1.parquet> <v2.parquet>]...
       (needs pyarrow and numpy)

v2 holds the frames in play segments; v1 every frame. On v2's frames every v1 column with a v2 counterpart must be
equal: exactly, except the rotation (v1 a matrix, v2 a quaternion: within 1e-6 per element) and v1's float64
replay time (the replay's float32). Prints one line per column group and exits non-zero on any difference.
"""

import json
import sys

import numpy as np
import pyarrow.parquet as pq


def fixed(table, name, width):
    return np.asarray(table.column(name).combine_chunks().flatten()).reshape(table.num_rows, width)


def column(table, name):
    return table.column(name).to_numpy(zero_copy_only=False)


def matrix(q):
    """Rotation matrices (rows, 3, 3) from quaternions (rows, 4: x, y, z, w)."""
    x, y, z, w = (q[:, i].astype(np.float64) for i in range(4))
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


class Check:
    def __init__(self):
        self.failures = 0

    def equal(self, label, expected, actual):
        expected = np.asarray(expected)
        actual = np.asarray(actual)
        if expected.dtype.kind == "f" or actual.dtype.kind == "f":
            same = (expected == actual) | (np.isnan(expected.astype(float)) & np.isnan(actual.astype(float)))
        else:
            same = expected == actual
        bad = int((~same).sum())
        if bad:
            self.failures += 1
            where = np.argwhere(~same)[0]
            print(f"  DIFFERENT {label}: {bad} values, first at {tuple(where)}: "
                  f"v1 {expected[tuple(where)]} v2 {actual[tuple(where)]}")

    def close(self, label, expected, actual, tolerance):
        worst = float(np.nanmax(np.abs(expected - actual))) if expected.size else 0.0
        if worst > tolerance:
            self.failures += 1
            print(f"  DIFFERENT {label}: worst {worst:.3g} > {tolerance}")
        return worst


def nullable(table, name, fill=np.nan):
    values = table.column(name).to_pylist()
    return np.array([fill if v is None else v for v in values], dtype=float)


def compare(v1_path, v2_path, check):
    v2 = pq.read_table(v2_path)
    header = json.loads(pq.read_metadata(v2_path).metadata[b"replicar"])
    players = len(header["players"])
    pads = len(header["pads"])
    v1 = pq.read_table(v1_path)
    frames = column(v2, "frame")
    v1 = v1.take(np.searchsorted(column(v1, "frame"), frames))
    check.equal("frame", column(v1, "frame"), frames)
    check.equal("replay_time", column(v1, "replay_time").astype(np.float32), column(v2, "replay_time"))
    check.equal("replay_tick", column(v1, "timeline_tick"), column(v2, "replay_tick"))
    check.equal("sim_tick", column(v1, "arena_tick"), column(v2, "sim_tick"))

    worst = 0.0
    for body, count in [("ball", 1), ("car", players)]:
        for quantity in ["position", "velocity", "angular_velocity"]:
            expected = fixed(v1, f"{body}_{quantity}", 3 * count).reshape(-1, count, 3)
            for p in range(count):
                prefix = "ball" if body == "ball" else f"car_{p}"
                actual = np.stack([column(v2, f"{prefix}_{quantity}_{c}") for c in "xyz"], -1)
                check.equal(f"{prefix}_{quantity}", expected[:, p], actual)
        expected = fixed(v1, f"{body}_rotation_columns", 9 * count).reshape(-1, count, 3, 3)
        for p in range(count):
            prefix = "ball" if body == "ball" else f"car_{p}"
            q = np.stack([column(v2, f"{prefix}_rotation_{c}") for c in "xyzw"], -1)
            norm = np.abs(np.linalg.norm(q.astype(np.float64), axis=-1) - 1).max()
            if norm > 1e-6:
                check.failures += 1
                print(f"  DIFFERENT {prefix}_rotation: |q| off by {norm:.3g}")
            if (q[:, 3] < 0).any():
                check.failures += 1
                print(f"  DIFFERENT {prefix}_rotation: w < 0")
            # v1 stores the matrix's columns: [x_axis, y_axis, z_axis], so element [k, i] is row i of column k.
            rebuilt = np.swapaxes(matrix(q), -1, -2)
            worst = max(worst, check.close(f"{prefix}_rotation", expected[:, p], rebuilt, 1e-6))

    boost = fixed(v1, "car_boost", players)
    demoed = fixed(v1, "car_demoed", players)
    axes = fixed(v1, "control_axes", 5 * players).reshape(-1, players, 5)
    buttons = fixed(v1, "control_buttons", 3 * players).reshape(-1, players, 3)
    spawn = fixed(v1, "spawn_pose_held", players)
    shells = v1.column("dead_shell_held").combine_chunks().flatten().to_pylist()
    shells = np.array([0 if s is None else s for s in shells]).reshape(-1, players)
    ping = v1.column("ping_raw").combine_chunks().flatten().to_pylist()
    ping = np.array([-1 if s is None else s for s in ping]).reshape(-1, players)
    for p in range(players):
        c = f"car_{p}"
        check.equal(f"{c}_boost", boost[:, p], column(v2, f"{c}_boost"))
        check.equal(f"{c}_is_demoed", demoed[:, p], column(v2, f"{c}_is_demoed"))
        for k, name in enumerate(["throttle", "steer", "pitch", "yaw", "roll"]):
            check.equal(f"{c}_controls_{name}", axes[:, p, k], column(v2, f"{c}_controls_{name}"))
        for k, name in enumerate(["jump", "boost", "handbrake"]):
            check.equal(f"{c}_controls_{name}", buttons[:, p, k], column(v2, f"{c}_controls_{name}"))
        status = np.array(v2.column(f"{c}_status").to_pylist())
        inferred = column(v2, f"{c}_status_inferred")
        check.equal(f"{c}_status spawning", spawn[:, p], status == "spawning")
        # A held wreck is demolished: inferred when v1 held it on inference (2), observed (1) like any reported
        # demolition. v2 does not keep v1's distinction between a held observed wreck and another demolition.
        check.equal(f"{c}_status inferred wreck", shells[:, p] == 2, (status == "demolished") & inferred)
        check.equal(f"{c}_status observed wreck", shells[:, p] == 1,
                    (status == "demolished") & ~inferred & (shells[:, p] == 1))
        check.equal(f"{c}_status demolished", demoed[:, p] & ~spawn[:, p].astype(bool) | (shells[:, p] > 0),
                    status == "demolished")
        check.equal(f"player_{p}_ping_raw", ping[:, p], nullable(v2, f"player_{p}_ping_raw", -1))

    cooldown = fixed(v1, "boost_pad_cooldown", pads)
    for k in range(pads):
        check.equal(f"pad_{k}_cooldown", cooldown[:, k], column(v2, f"pad_{k}_cooldown"))

    scores = fixed(v1, "scores", 2)
    check.equal("blue_score", scores[:, 0], nullable(v2, "blue_score"))
    check.equal("orange_score", scores[:, 1], nullable(v2, "orange_score"))
    check.equal("period", np.array(v1.column("scoreboard_period").to_pylist()),
                np.array(v2.column("period").to_pylist()))
    check.equal("clock_phase", np.array(v1.column("scoreboard_clock_state").to_pylist()),
                np.array(v2.column("clock_phase").to_pylist()))
    check.equal("seconds_remaining", nullable(v1, "scoreboard_seconds_remaining"),
                nullable(v2, "seconds_remaining"))
    check.equal("overtime_seconds", nullable(v1, "scoreboard_overtime_seconds"),
                nullable(v2, "overtime_seconds"))

    check.equal("segment", nullable(v1, "label_episode"), nullable(v2, "segment"))
    check.equal("future_seconds_until_segment_end", nullable(v1, "label_episode_seconds_remaining"),
                nullable(v2, "future_seconds_until_segment_end"))

    check.equal("ball_updated", column(v1, "ball_fresh"), column(v2, "ball_updated"))
    check.equal("ball_seconds_since_update", nullable(v1, "ball_update_age_seconds"),
                nullable(v2, "ball_seconds_since_update"))
    check.equal("ball_ticks_since_update", nullable(v1, "ball_packet_age_ticks"),
                nullable(v2, "ball_ticks_since_update"))
    for name_v1, name_v2, fill in [("car_fresh", "updated", -1), ("car_update_age_seconds", "seconds_since_update",
                                                                   np.nan),
                                   ("car_packet_age_ticks", "ticks_since_update", np.nan)]:
        values = v1.column(name_v1).combine_chunks().flatten().to_pylist()
        values = np.array([fill if v is None else v for v in values], dtype=float).reshape(-1, players)
        for p in range(players):
            check.equal(f"car_{p}_{name_v2}", values[:, p], nullable(v2, f"car_{p}_{name_v2}", fill))
    return v2.num_rows, worst


def main():
    paths = sys.argv[1:]
    if not paths or len(paths) % 2:
        sys.exit(__doc__)
    check = Check()
    for v1_path, v2_path in zip(paths[::2], paths[1::2]):
        before = check.failures
        rows, worst = compare(v1_path, v2_path, check)
        status = "equal" if check.failures == before else "DIFFERENT"
        print(f"{status}  {v2_path}: {rows} rows, rotation worst {worst:.2g}")
    sys.exit(1 if check.failures else 0)


if __name__ == "__main__":
    main()
