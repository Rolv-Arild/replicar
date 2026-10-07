"""A candidate `evaluate` report against a baseline, by game size and per replay (AGENTS.md: counts, p50/p90/p99,
per-game-size and per-replay behaviour, material regressions).

For each main row (one-step car position, velocity, rotation, angular velocity; masked car position at 1 and 4
frames) it prints the pooled p50 / p90 / p99 per game size, and the replays whose own p90 changed by more than 2%
(better / worse), with the five worst.

usage: python scripts/compare_reports_by_size.py <baseline.json> <candidate.json>
"""

import json
import sys

ROWS = [
    ("1-step car position UU", lambda r: r["position_uu"]["car"]["simulated"],
     lambda r, g: r["by_game_size"][g]["car"]["simulated"]),
    ("1-step car velocity UU/s", lambda r: r["kinematics"]["car"]["linear_velocity_uu_per_second"]["simulated"],
     lambda r, g: r["one_step_kinematics_by_game_size"][g]["car"]["linear_velocity_uu_per_second"]["simulated"]),
    ("1-step car rotation deg", lambda r: r["kinematics"]["car"]["rotation_degrees"]["simulated"],
     lambda r, g: r["one_step_kinematics_by_game_size"][g]["car"]["rotation_degrees"]["simulated"]),
    ("1-step car angular rad/s", lambda r: r["kinematics"]["car"]["angular_velocity_radians_per_second"]["simulated"],
     lambda r, g: r["one_step_kinematics_by_game_size"][g]["car"]["angular_velocity_radians_per_second"]["simulated"]),
    ("masked h1 car position UU", lambda r: r["masked_position_uu_by_horizon_frames"]["1"]["car"]["simulated"],
     lambda r, g: r["masked_by_game_size"][g]["1"]["car"]["simulated"]),
    ("masked h4 car position UU", lambda r: r["masked_position_uu_by_horizon_frames"]["4"]["car"]["simulated"],
     lambda r, g: r["masked_by_game_size"][g]["4"]["car"]["simulated"]),
]


def main() -> None:
    base, cand = (json.load(open(p)) for p in sys.argv[1:3])
    by_path = {r["path"]: r for r in base["replays"]}
    for name, per_replay, per_size in ROWS:
        print(f"== {name}")
        for g in ("1v1", "2v2", "3v3"):
            try:
                a, b = per_size(base, g), per_size(cand, g)
            except KeyError:
                continue
            q = lambda x: "/".join(f"{x[k]:.4g}" for k in ("p50", "p90", "p99"))
            print(f"  {g}: {q(a)} -> {q(b)}")
        changes = []
        for r in cand["replays"]:
            old = by_path.get(r["path"])
            if old is None:
                continue
            try:
                a, b = per_replay(old)["p90"], per_replay(r)["p90"]
            except (KeyError, TypeError):
                continue
            if a and a > 0:
                changes.append(((b - a) / a, r["path"].replace("\\", "/").split("/")[-2:], a, b))
        better = sum(c[0] < -0.02 for c in changes)
        worse = [c for c in changes if c[0] > 0.02]
        worse.sort(key=lambda c: -c[0])
        print(f"  per replay p90: {better} better, {len(worse)} worse by more than 2% (of {len(changes)})")
        for c in worse[:5]:
            print(f"    {'/'.join(c[1])[:40]}: {c[2]:.4g} -> {c[3]:.4g} ({c[0]:+.1%})")


if __name__ == "__main__":
    main()
