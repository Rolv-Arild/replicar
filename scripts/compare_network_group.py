"""Compare a v2 file's `network` group with v1's JSONL observations of the same replay (docs/v2-plan.md, story
7.4): on the file's frames, the ball, each player's current car (v1's primary linked car), the players' stats
and ping, the clock, the scores and the game state, every value and the frame of its last change.

usage: python scripts/compare_network_group.py <v1.jsonl> <v2.parquet>   (needs pyarrow)
"""

import json
import math
import sys

import pyarrow.parquet as pq


def same(a, b):
    if isinstance(a, float) or isinstance(b, float):
        if a is None or b is None:
            return a is None and b is None
        return a == b or (math.isnan(a) and math.isnan(b)) or abs(a - b) <= 1e-6 * max(1.0, abs(a))
    return a == b


def main():
    v1_path, v2_path = sys.argv[1:3]
    table = pq.read_table(v2_path)
    header = json.loads(pq.read_metadata(v2_path).metadata[b"replicar"])
    columns = {name: table.column(name).to_pylist() for name in table.column_names if name.startswith("network_")}
    rows = {frame: i for i, frame in enumerate(table.column("frame").to_pylist())}
    failures = {}
    checked = 0

    def check(label, expected, actual):
        nonlocal checked
        checked += 1
        if not same(expected, actual):
            failures.setdefault(label, (expected, actual))

    def value(obs, key):
        v = obs.get(key) if obs else None
        return (None, None) if v is None else (v["value"], v["frame"])

    with open(v1_path) as f:
        f.readline()
        for line in f:
            frame = json.loads(line)
            obs = frame["observations"]
            row = rows.get(obs["index"])
            if row is None:
                continue
            col = lambda name: columns[name][row]
            for key, name in [("seconds_remaining", "network_seconds_remaining"), ("overtime", "network_overtime")]:
                v, fr = value(obs, key)
                check(name, v, col(name))
                check(name + "_frame", fr, col(name + "_frame"))
            for team, name in enumerate(["blue", "orange"]):
                v, fr = value({"s": obs["team_scores"][team]}, "s")
                check(f"network_{name}_score", v, col(f"network_{name}_score"))
            v, fr = value(obs, "game_state")
            check("network_game_state", v, col("network_game_state"))

            def body(prefix, b):
                for key, name, comps in [("position", "position", "xyz"), ("rotation_xyzw", "rotation", "xyzw"),
                                         ("linear_velocity", "velocity", "xyz"),
                                         ("angular_velocity_replay_units", "angular_velocity_raw", "xyz")]:
                    v, fr = value(b, key)
                    for i, c in enumerate(comps):
                        check(f"{prefix}_{name}_{c}", None if v is None else v[i], col(f"{prefix}_{name}_{c}"))
                    check(f"{prefix}_{name}_frame", fr, col(f"{prefix}_{name}_frame"))
                v, fr = value(b, "sleeping")
                check(f"{prefix}_sleeping", v, col(f"{prefix}_sleeping"))

            body("network_ball", obs["ball"])
            for player in header["players"]:
                p = player["index"]
                prefix = f"network_car_{p}"
                cars = [c for c in obs["cars"] if c["player_key"] == player["key"]]

                def priority(c):
                    position = c["body"].get("position")
                    return (c["player_link_active"], c["actor_created_frame"],
                            position is not None and position["frame"] == obs["index"])
                car = max(cars, key=priority) if cars else None
                check(f"{prefix}_actor", car and car["actor_id"], col(f"{prefix}_actor"))
                check(f"{prefix}_created", car and car["actor_created_frame"], col(f"{prefix}_created"))
                body(prefix, car["body"] if car else None)
                for key, name in [("boost", "boost"), ("boost_raw", "boost_raw"), ("body_product_id", "body_product_id")]:
                    v, fr = value(car, key)
                    check(f"{prefix}_{name}", v, col(f"{prefix}_{name}"))
                    check(f"{prefix}_{name}_frame", fr, col(f"{prefix}_{name}_frame"))
                inputs = car["inputs"] if car else None
                for key in ["throttle", "steer", "handbrake", "boost_active_raw", "jump_active_raw",
                            "double_jump_active_raw", "dodge_active_raw", "flip_car_active_raw"]:
                    v, fr = value(inputs, key)
                    check(f"{prefix}_{key}", v, col(f"{prefix}_{key}"))
                    check(f"{prefix}_{key}_frame", fr, col(f"{prefix}_{key}_frame"))
                v, fr = value(inputs, "dodge_torque_replay_units")
                for i, c in enumerate("xyz"):
                    check(f"{prefix}_dodge_torque_raw_{c}", None if v is None else v[i],
                          col(f"{prefix}_dodge_torque_raw_{c}"))
                players = [q for q in obs["players"] if q.get("key") == player["key"]]
                q = players[0] if players else None
                v, fr = value(q, "ping_raw")
                check(f"network_player_{p}_ping_raw", v, col(f"network_player_{p}_ping_raw"))
                stats = q["stats"] if q else None
                for key, name in [("match_score", "match_score"), ("goals", "goals"), ("assists", "assists"),
                                  ("saves", "saves"), ("shots", "shots"), ("demolishes", "demolitions")]:
                    v, fr = value(stats, key)
                    check(f"network_player_{p}_{name}", v, col(f"network_player_{p}_{name}"))
                    check(f"network_player_{p}_{name}_frame", fr, col(f"network_player_{p}_{name}_frame"))
    print(f"{v2_path}: {checked} values, {len(failures)} columns differ")
    for label, (expected, actual) in list(failures.items())[:10]:
        print(f"  DIFFERENT {label}: v1 {expected} v2 {actual}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
