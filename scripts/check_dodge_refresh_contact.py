"""What was the car touching when its `DodgesRefreshedCounter` went up? Uses the converter's simulated state.

For each increment of a car's counter (first update with a higher value) prints, at the update's frame, the car's
height, the distance from the car to the ball and to the nearest other car, and the car's flags in the previous and
current frame. The simulated state is a reconstruction (positions are packets within a tick or two), so the
distances are approximate; the point is which object is near.

usage: python scripts/check_dodge_refresh_contact.py <replay> <converted.jsonl from convert_replay>
"""
import json
import math
import os
import re
import subprocess
import sys

replay, converted = sys.argv[1], sys.argv[2]
exe = os.path.abspath("target/release/list_attributes.exe")
out = subprocess.run([exe, replay, "dodgesrefreshed", "--trace"], capture_output=True, text=True, encoding="utf-8").stdout
last, events = {}, []
for line in out.splitlines():
    m = re.match(r"(\d+) ([\d.]+) (\d+) \S+ Int\((\d+)\)", line)
    if not m:
        continue
    frame, actor, value = int(m[1]), int(m[3]), int(m[4])
    if value > last.get(actor, 0) and actor in last:
        events.append((frame, actor, value))
    last[actor] = max(value, last.get(actor, 0))

wanted = {f for frame, _, _ in events for f in (frame - 1, frame, frame + 1)}
frames = {}
with open(converted, encoding="utf-8") as f:
    header = json.loads(f.readline())
    slot_of = {s["player_key"]: s["slot"] for s in header["car_slots"]}
    for line in f:
        record = json.loads(line)
        if record["frame"] in wanted:
            frames[record["frame"]] = record
print(os.path.basename(replay), "increments:", len(events))
for frame, actor, value in events:
    record = frames[frame]
    key = [c["player_key"] for c in record["observations"]["cars"] if c["actor_id"] == actor][0]
    slot = slot_of[key]
    cars = {c["slot"]: c for c in record["state"]["cars"]}
    me = cars[slot]
    ball = record["state"]["ball"]["physics"]["position"]
    d_ball = math.dist(me["physics"]["position"], ball)
    others = [(math.dist(me["physics"]["position"], c["physics"]["position"]), s) for s, c in cars.items() if s != slot and not c["is_demoed"]]
    d_car = min(others)[0] if others else float("nan")
    flags = lambda c: "".join(ch if c[k] else "-" for ch, k in (("J", "has_jumped"), ("D", "has_double_jumped"), ("F", "has_flipped")))
    before = {c["slot"]: c for c in frames[frame - 1]["state"]["cars"]}[slot]
    print(
        f"  frame {frame} actor {actor} -> {value}: z {me['physics']['position'][2]:.0f}, ball {d_ball:.0f} UU, "
        f"nearest car {d_car:.0f} UU, flags {flags(before)} -> {flags(me)}"
    )
