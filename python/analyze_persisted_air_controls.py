"""Join matched masked rotation traces from the default and causal-persistence runs.

Usage: python python/analyze_persisted_air_controls.py BASE.jsonl PERSIST.jsonl

Rows are joined by replay hash, actor lifetime, and mask-window start. Labels are the
original horizon-four rotation errors; features are the control chosen for the first
withheld interval and packet state strictly before the window. Nothing about the
target packet is used as a feature.
"""

import argparse
import json
import math
import statistics


def window_key(row):
    return (
        row["replay_sha256"],
        row["actor_id"],
        row["actor_created_frame"],
        row["frame"] - row["horizon"],
    )


def load(path):
    starts, finals = {}, {}
    with open(path, encoding="utf-8") as file:
        for line in file:
            if '"horizon":1,' not in line and '"horizon":4,' not in line:
                continue
            row = json.loads(line)
            key = window_key(row)
            if row["horizon"] == 1:
                sim = row["previous_simulated"]
                controls = sim["controls_for_next_interval"]
                position = row["masked_body"]["position"]
                packets = row["prior_angular_packets_before_mask"]
                speed = (
                    math.sqrt(sum(a * a for a in packets[0]["angular_velocity_radians_per_second"]))
                    if packets
                    else None
                )
                starts[key] = {
                    "pitch": controls["pitch"],
                    "yaw": controls["yaw"],
                    "roll": controls["roll"],
                    "z": position["value"][2] if position else None,
                    "airborne": not sim["is_on_ground"],
                    "speed": speed,
                }
            elif row["rotation_error_degrees"] is not None:
                finals[key] = row["rotation_error_degrees"]
    return starts, finals


def bucket(value, edges, labels):
    for edge, label in zip(edges, labels):
        if value < edge:
            return label
    return labels[-1]


def report(title, groups):
    print(f"\n{title}")
    print(f"{'group':28s} {'n':>6s} {'better>1':>9s} {'worse>1':>8s} {'median d':>9s} {'mean d':>8s}")
    for label in sorted(groups):
        deltas = groups[label]
        print(
            f"{label:28s} {len(deltas):6d} {sum(d < -1 for d in deltas):9d} "
            f"{sum(d > 1 for d in deltas):8d} {statistics.median(deltas):9.3f} "
            f"{statistics.fmean(deltas):8.3f}"
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("base_trace")
    parser.add_argument("persist_trace")
    args = parser.parse_args()
    base_starts, base_final = load(args.base_trace)
    persist_starts, persist_final = load(args.persist_trace)
    by_control, by_altitude, by_speed, by_airborne = {}, {}, {}, {}
    all_deltas = []
    for key, base_error in base_final.items():
        persist_error = persist_final.get(key)
        start = persist_starts.get(key)
        if persist_error is None or start is None or key not in base_starts:
            continue
        delta = persist_error - base_error
        all_deltas.append(delta)
        magnitude = max(abs(start["pitch"]), abs(start["roll"]))
        label = bucket(magnitude, [1e-6, 0.3, 0.7], ["0 none", "1 |p,r|<0.3", "2 |p,r|<0.7", "3 |p,r|>=0.7"])
        by_control.setdefault(label, []).append(delta)
        if start["z"] is not None:
            alt = bucket(start["z"], [50, 100, 300, 800], ["0 <50", "1 50-100", "2 100-300", "3 300-800", "4 >800"])
            by_altitude.setdefault(alt, []).append(delta)
        if start["speed"] is not None:
            spd = bucket(start["speed"], [1, 3, 5, 5.48], ["0 <1", "1 1-3", "2 3-5", "3 5-5.48", "4 >=5.48"])
            by_speed.setdefault(spd, []).append(delta)
        by_airborne.setdefault("airborne" if start["airborne"] else "ground", []).append(delta)
    print(f"matched windows: {len(all_deltas)}; delta = persistence error minus default error (deg, horizon 4)")
    report("by persisted pitch/roll magnitude", by_control)
    report("by pre-window altitude (UU)", by_altitude)
    report("by prior angular speed (rad/s)", by_speed)
    report("by simulated ground state", by_airborne)


if __name__ == "__main__":
    main()
