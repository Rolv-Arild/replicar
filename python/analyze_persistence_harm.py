"""Explain which horizon-four windows the causal aerial persistence helps or hurts.

Usage: python python/analyze_persistence_harm.py BASE.jsonl PERSIST.jsonl

Only windows where persistence changed the first-interval pitch/roll are analysed.
Features come from rows and packets strictly before the mask window; the target
rotation errors are labels only.
"""

import argparse
import json
import math
import statistics


def key(row):
    return (row["replay_sha256"], row["actor_id"], row["actor_created_frame"], row["frame"] - row["horizon"])


def speed(v):
    return math.sqrt(sum(a * a for a in v))


def load(path):
    starts, finals = {}, {}
    with open(path, encoding="utf-8") as file:
        for line in file:
            if '"horizon":1,' not in line and '"horizon":4,' not in line:
                continue
            row = json.loads(line)
            k = key(row)
            if row["horizon"] == 1:
                starts[k] = row
            elif row["rotation_error_degrees"] is not None:
                finals[k] = row["rotation_error_degrees"]
    return starts, finals


def features(start):
    c = start["previous_simulated"]["controls_for_next_interval"]
    pk = start["prior_angular_packets_before_mask"]
    if len(pk) < 2:
        return None
    w0, w1 = pk[0]["angular_velocity_radians_per_second"], pk[1]["angular_velocity_radians_per_second"]
    inputs = start["observed_inputs_before_interval"]
    steer = inputs["steer"]["value"] if inputs.get("steer") else 0.0
    hb = bool(inputs["handbrake"]["value"]) if inputs.get("handbrake") else False
    z = start["masked_body"]["position"]["value"][2]
    age = start["replay_time"] - pk[0]["replay_time"]
    return {
        "pr": max(abs(c["pitch"]), abs(c["roll"])),
        "roll_dom": abs(c["roll"]) > abs(c["pitch"]),
        "trend": speed(w0) - speed(w1),
        "speed": speed(w0),
        "age": age,
        "steer": abs(steer),
        "hb": hb,
        "z": z,
        "yawc": abs(c["yaw"]),
        "wcontact": start["previous_simulated"]["wheel_contact_count"],
    }


def table(title, groups):
    print(f"\n{title}")
    print(f"{'group':26s} {'n':>5s} {'better>1':>9s} {'worse>1':>8s} {'median':>8s} {'mean':>8s}")
    for label in sorted(groups):
        d = groups[label]
        print(f"{label:26s} {len(d):5d} {sum(x < -1 for x in d):9d} {sum(x > 1 for x in d):8d} "
              f"{statistics.median(d):8.3f} {statistics.fmean(d):8.3f}")


def bucket(v, edges, names):
    for e, n in zip(edges, names):
        if v < e:
            return n
    return names[-1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("base")
    ap.add_argument("persist")
    args = ap.parse_args()
    bs, bf = load(args.base)
    ps, pf = load(args.persist)
    groups = {n: {} for n in ["trend", "speed", "age", "steer", "hb", "roll_dom", "yawc", "z", "wcontact"]}
    total = []
    for k, be in bf.items():
        if k not in pf or k not in ps or k not in bs:
            continue
        cb = bs[k]["previous_simulated"]["controls_for_next_interval"]
        cp = ps[k]["previous_simulated"]["controls_for_next_interval"]
        if abs(cb["pitch"] - cp["pitch"]) < 1e-6 and abs(cb["roll"] - cp["roll"]) < 1e-6:
            continue
        f = features(ps[k])
        if f is None:
            continue
        d = pf[k] - be
        total.append(d)
        groups["trend"].setdefault(bucket(f["trend"], [-1, -0.2, 0.2, 1], ["a trend<-1", "b -1..-0.2", "c flat", "d 0.2..1", "e >1"]), []).append(d)
        groups["speed"].setdefault(bucket(f["speed"], [1, 3, 5, 5.48], ["a <1", "b 1-3", "c 3-5", "d 5-5.48", "e >=5.48"]), []).append(d)
        groups["age"].setdefault(bucket(f["age"], [0.04, 0.08, 0.12], ["a <0.04s", "b 0.04-0.08", "c 0.08-0.12", "d >=0.12"]), []).append(d)
        groups["steer"].setdefault(bucket(f["steer"], [0.1, 0.5], ["a <0.1", "b 0.1-0.5", "c >=0.5"]), []).append(d)
        groups["hb"].setdefault("handbrake" if f["hb"] else "no handbrake", []).append(d)
        groups["roll_dom"].setdefault("roll dominant" if f["roll_dom"] else "pitch dominant", []).append(d)
        groups["yawc"].setdefault(bucket(f["yawc"], [0.3, 0.7], ["a yaw<0.3", "b 0.3-0.7", "c >=0.7"]), []).append(d)
        groups["z"].setdefault(bucket(f["z"], [100, 300, 800], ["a <100", "b 100-300", "c 300-800", "d >800"]), []).append(d)
        groups["wcontact"].setdefault("contact" if f["wcontact"] else "no wheel contact", []).append(d)
    print(f"windows with changed pitch/roll: {len(total)}; delta = persist - base (deg, h4); "
          f"better>1={sum(x < -1 for x in total)} worse>1={sum(x > 1 for x in total)} "
          f"median={statistics.median(total):.3f} mean={statistics.fmean(total):.3f}")
    for name, g in groups.items():
        table(f"by {name}", g)


if __name__ == "__main__":
    main()
