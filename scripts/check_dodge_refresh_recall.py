"""Flip resets seen by the converter against the replay's `DodgesRefreshedCounter` increments.

A simulated reset is a frame where a car that was airborne (not on the ground in the previous frame) has fewer
of `has_jumped`, `has_double_jumped`, `has_flipped` set than in the previous frame and the ball is within 250 UU
(a wheel-ball contact, not a landing on the floor: the car must be above 80 UU; the flags can clear over two frames, which counts once). A counter reset is an
increment of that car's `TAGame.Car_TA:DodgesRefreshedCounter` (first update with a higher value). They are matched
one to one for the same car within +-WINDOW frames (default 3).

usage: python scripts/check_dodge_refresh_recall.py <replay> <converted.jsonl> [window]
"""
import json
import math
import os
import re
import subprocess
import sys

replay, converted = sys.argv[1], sys.argv[2]
window = int(sys.argv[3]) if len(sys.argv) > 3 else 3
exe = os.path.abspath("target/release/list_attributes.exe")
out = subprocess.run([exe, replay, "dodgesrefreshed", "--trace"], capture_output=True, text=True, encoding="utf-8").stdout
last, counter = {}, []
for line in out.splitlines():
    m = re.match(r"(\d+) ([\d.]+) (\d+) \S+ Int\((\d+)\)", line)
    if not m:
        continue
    frame, actor, value = int(m[1]), int(m[3]), int(m[4])
    if value > last.get(actor, 0):
        counter.append((frame, actor))
    last[actor] = max(value, last.get(actor, 0))

sim = []
previous = {}
actor_of_slot = {}
with open(converted, encoding="utf-8") as f:
    header = json.loads(f.readline())
    slot_of = {s["player_key"]: s["slot"] for s in header["car_slots"]}
    for line in f:
        r = json.loads(line)
        for c in r["observations"]["cars"]:
            slot = slot_of.get(c["player_key"])
            if slot is not None:
                actor_of_slot[(slot, c["actor_created_frame"])] = c["actor_id"]
        ball = r["state"]["ball"]["physics"]["position"]
        for car in r["state"]["cars"]:
            slot = car["slot"]
            flags = int(car["has_jumped"]) + int(car["has_double_jumped"]) + int(car["has_flipped"])
            before = previous.get(slot)
            previous[slot] = (flags, car["is_on_ground"])
            if before and before[0] and not before[1] and flags < before[0] and not car["is_demoed"]:
                pos = car["physics"]["position"]
                # near the ball and not a landing on the floor (the car at least 80 UU up)
                if math.dist(pos, ball) < 250 and pos[2] > 80:
                    # the actor currently on this slot
                    actors = [c["actor_id"] for c in r["observations"]["cars"] if slot_of.get(c["player_key"]) == slot]
                    sim.append((r["frame"], actors[0] if actors else None))

used = set()
matched = 0
for frame, actor in counter:
    for i, (sf, sa) in enumerate(sim):
        if i not in used and sa == actor and abs(sf - frame) <= window:
            used.add(i)
            matched += 1
            break
print(
    f"{os.path.basename(replay)}: counter increments {len(counter)}, simulated ball resets {len(sim)}, "
    f"matched {matched} (recall {matched / max(1, len(counter)):.0%}, surplus simulated {len(sim) - matched})"
)
for i, (sf, sa) in enumerate(sim):
    if i not in used:
        print("  simulated reset without a counter increment: frame", sf, "actor", sa)
