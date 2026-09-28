"""Join matched masked rotation traces and summarize prior angular-speed regimes.

Usage: python python/analyze_low_air_regimes.py BASE.jsonl HOLD.jsonl
Only the original target rotation-error fields are labels. Features come from the
horizon-one row's angular packets strictly before that mask window.
"""

import argparse
import json
import math


def window_key(row):
    return (
        row["replay_sha256"],
        row["actor_id"],
        row["actor_created_frame"],
        row["frame"] - row["horizon"],
    )


def eligible_horizon_four(row):
    return row["horizon"] == 4 and row["rotation_error_degrees"] is not None


def summarize(label, regrets):
    print(
        f"{label}: n={len(regrets)} "
        f"hold_better_by_over_1_deg={sum(value < -1 for value in regrets)} "
        f"hold_worse_by_over_1_deg={sum(value > 1 for value in regrets)}"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("base_trace")
    parser.add_argument("hold_trace")
    args = parser.parse_args()

    hold_errors = {}
    with open(args.hold_trace, encoding="utf-8") as file:
        for line in file:
            row = json.loads(line)
            if eligible_horizon_four(row):
                hold_errors[window_key(row)] = row["rotation_error_degrees"]

    starts = {}
    groups = {"all_matched": [], "prior_speed_4_to_5": [], "prior_speed_5_48_to_5_51": []}
    unjoined = 0
    with open(args.base_trace, encoding="utf-8") as file:
        for line in file:
            row = json.loads(line)
            key = window_key(row)
            if row["horizon"] == 1:
                starts[key] = row
                continue
            if not eligible_horizon_four(row):
                continue
            start = starts.pop(key, None)
            hold_error = hold_errors.get(key)
            if start is None or hold_error is None:
                unjoined += 1
                continue
            del hold_errors[key]
            regret = hold_error - row["rotation_error_degrees"]
            groups["all_matched"].append(regret)
            position = start["masked_body"]["position"]
            packets = start["prior_angular_packets_before_mask"]
            if position is None or not 50 <= position["value"][2] <= 100 or not packets:
                continue
            angular = packets[0]["angular_velocity_radians_per_second"]
            speed = math.sqrt(sum(axis * axis for axis in angular))
            if 4 <= speed < 5:
                groups["prior_speed_4_to_5"].append(regret)
            elif 5.48 <= speed < 5.51:
                groups["prior_speed_5_48_to_5_51"].append(regret)

    for label, regrets in groups.items():
        summarize(label, regrets)
    print(f"unjoined_base_horizon_four={unjoined} unjoined_hold_horizon_four={len(hold_errors)}")


if __name__ == "__main__":
    main()
