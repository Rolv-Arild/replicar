"""Side-by-side pooled metrics of evaluate reports (a baseline and candidates): one-step position and kinematics,
and masked position and kinematics by horizon, car and ball, p50 / p90 / p99 of the simulated error.

usage: python scripts/pooled_delta.py <baseline.json> <candidate.json>...
"""

import json
import sys


def metrics(report):
    out = {}
    for body in ["car", "ball"]:
        out[f"1-step {body} position UU"] = report["all"][body]["simulated"]
        for name, rows in report["one_step_kinematics_all"][body].items():
            out[f"1-step {body} {name}"] = rows["simulated"]
        for h, rows in report["masked_position_uu_by_horizon_frames"].items():
            out[f"masked h{h} {body} position UU"] = rows[body]["simulated"]
        for h, rows in report["masked_kinematics_by_horizon_frames"].items():
            for name, field in rows[body].items():
                out[f"masked h{h} {body} {name}"] = field["simulated"]
    return out


def main():
    reports = [metrics(json.load(open(p))) for p in sys.argv[1:]]
    for name, base in reports[0].items():
        cells = []
        for r in reports:
            m = r.get(name)
            cells.append("n/a" if m is None else f"{m['p50']:.4g}/{m['p90']:.4g}/{m['p99']:.4g}")
        flag = "" if all(c == cells[0] for c in cells) else "  *"
        print(f"{name:55s} " + "   ".join(cells) + flag)


if __name__ == "__main__":
    main()
