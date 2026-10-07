"""Compare a v2 file's `diagnostics` group (written with --all-frames) with v1's JSONL frames of the same replay
(docs/v2-plan.md, story 7.4): each frame's prediction errors against v1's position residuals, and RocketSim's
events against v1's simulated events.

usage: python scripts/compare_diagnostics_group.py <v1.jsonl> <v2.parquet>   (needs pyarrow)
"""

import json
import sys

import pyarrow.parquet as pq

PAIRS = [
    ("car_actor", "actor_id"),
    ("seconds_since_previous", "seconds_since_previous_position"),
    ("velocity_error", "simulated_velocity_error_uu_per_sec"),
    ("rotation_error_degrees", "simulated_rotation_error_degrees"),
    ("angular_velocity_error", "simulated_angular_velocity_error_rad_per_sec"),
    ("hold_position_error", "hold_error_uu"),
    ("linear_position_error", "linear_extrapolation_error_uu"),
    ("hold_velocity_error", "hold_velocity_error_uu_per_sec"),
    ("hold_rotation_error_degrees", "hold_rotation_error_degrees"),
    ("hold_angular_velocity_error", "hold_angular_velocity_error_rad_per_sec"),
    ("is_on_ground", "is_on_ground"),
]


def close(a, b):
    if isinstance(a, float) or isinstance(b, float):
        if a is None or b is None:
            return a is None and b is None
        return abs(a - b) <= 1e-4 * max(1.0, abs(a))
    return a == b


def main():
    v1_path, v2_path = sys.argv[1:3]
    table = pq.read_table(v2_path, columns=["frame", "prediction_errors", "simulated_events"])
    frames = table.column("frame").to_pylist()
    errors = dict(zip(frames, table.column("prediction_errors").to_pylist()))
    events = dict(zip(frames, table.column("simulated_events").to_pylist()))
    differences = []
    counted = [0, 0]
    with open(v1_path) as f:
        f.readline()
        for line in f:
            frame = json.loads(line)
            index = frame["frame"]
            expected = frame["position_residuals"]
            actual = errors.get(index, [])
            counted[0] += len(expected)
            if len(expected) != len(actual):
                differences.append((index, "count", len(expected), len(actual)))
                continue
            for e, a in zip(expected, actual):
                for ours, theirs in PAIRS:
                    if not close(e.get(theirs), a[ours]):
                        differences.append((index, ours, e.get(theirs), a[ours]))
                for i, axis in enumerate("xyz"):
                    if not close(e["simulated_error_vector_uu"][i], a[f"position_error_{axis}"]):
                        differences.append((index, f"position_error_{axis}", e["simulated_error_vector_uu"][i],
                                            a[f"position_error_{axis}"]))
            expected = frame["simulated_events"]
            actual = events.get(index, [])
            counted[1] += len(expected)
            if len(expected) != len(actual) or any(
                    e["arena_tick"] != a["sim_tick"] or e["event"]["kind"] != a["kind"]
                    or not all(close(x, a[f"point_{axis}"]) for x, axis in zip(e["event"].get("contact_point", []), "xyz"))
                    for e, a in zip(expected, actual)):
                differences.append((index, "events", len(expected), len(actual)))
    print(f"{v2_path}: {counted[0]} prediction errors, {counted[1]} events, {len(differences)} differences")
    for d in differences[:10]:
        print("  DIFFERENT", d)
    sys.exit(1 if differences else 0)


if __name__ == "__main__":
    main()
