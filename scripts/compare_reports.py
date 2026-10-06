"""Per-replay comparison of two evaluate_corpus reports of the same split (baseline A, candidate B).

usage: python scripts/compare_reports.py <A.json> <B.json> [--threshold 0.02] [--floor 0.5]

For every replay and every `simulated` p90 of the car and ball rows (one-step position, one-step kinematics,
masked position by horizon), prints the rows where B is worse than A by more than the threshold (relative),
ignoring rows whose A value is below the floor (noise at the printed precision), and a count of rows better,
worse and equal. Sample counts that differ are reported too: a row compared on other samples is not paired.
"""

import argparse
import json


def rows(replay):
    """(name, stats) for every `simulated` row of a replay report."""
    def walk(prefix, node):
        if isinstance(node, dict):
            if "simulated" in node and isinstance(node["simulated"], dict) and "p90" in node["simulated"]:
                yield prefix, node["simulated"]
            for key, value in node.items():
                if key != "simulated":
                    yield from walk(f"{prefix}.{key}" if prefix else key, value)
    for section in ["position_uu", "kinematics", "masked_position_uu_by_horizon_frames",
                    "masked_kinematics_by_horizon_frames"]:
        yield from walk(section, replay.get(section, {}))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("a")
    parser.add_argument("b")
    parser.add_argument("--threshold", type=float, default=0.02)
    parser.add_argument("--floor", type=float, default=0.5)
    args = parser.parse_args()
    a = {r["path"]: r for r in json.load(open(args.a))["replays"]}
    b = {r["path"]: r for r in json.load(open(args.b))["replays"]}
    better = worse = equal = count_changes = much_better = 0
    flagged = []
    for path in sorted(set(a) & set(b)):
        rows_b = dict(rows(b[path]))
        for name, stats_a in rows(a[path]):
            stats_b = rows_b.get(name)
            if stats_b is None:
                continue
            if stats_a.get("count") != stats_b.get("count"):
                count_changes += 1
            va, vb = stats_a["p90"], stats_b["p90"]
            if va is None or vb is None:
                continue
            if vb == va:
                equal += 1
            elif vb < va:
                better += 1
                if va >= args.floor and (va - vb) / va > args.threshold:
                    much_better += 1
            else:
                worse += 1
                if va >= args.floor and (vb - va) / va > args.threshold:
                    flagged.append((path, name, va, vb))
    missing = sorted(set(a) ^ set(b))
    print(f"{args.a} -> {args.b}: p90 rows better {better}, worse {worse}, equal {equal}; "
          f"rows with another sample count {count_changes}; replays in only one report {len(missing)}; "
          f"beyond {args.threshold:.0%}: better {much_better}, worse {len(flagged)}")
    for path, name, va, vb in flagged:
        print(f"  worse by more than {args.threshold:.0%}: {path} {name} p90 {va:.3f} -> {vb:.3f} "
              f"({(vb - va) / va:+.1%})")


if __name__ == "__main__":
    main()
