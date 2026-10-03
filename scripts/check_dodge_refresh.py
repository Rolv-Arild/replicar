"""Compare the replay's per-car `TAGame.Car_TA:DodgesRefreshedCounter` (a flip reset) with the converter's
exported jump and flip flags around each increment.

For each increment of the counter (traced with `list_attributes <replay> dodgesrefreshed --trace`) prints the
car's flags in the converted JSONL from 6 frames before to 7 after: J has_jumped, D has_double_jumped,
F has_flipped, g is_on_ground ('-' / '.' = false).

usage: python scripts/check_dodge_refresh.py <replay> <converted.jsonl from convert_replay>
"""
import json
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
    if value > last.get(actor, 0):
        events.append((frame, actor, value))
    last[actor] = max(value, last.get(actor, 0))

wanted = set()
for frame, _, _ in events:
    wanted.update(range(frame - 6, frame + 8))
frames = {}
with open(converted, encoding="utf-8") as f:
    header = json.loads(f.readline())
    slot_of = {s["player_key"]: s["slot"] for s in header["car_slots"]}
    for line in f:
        record = json.loads(line)
        if record["frame"] in wanted:
            frames[record["frame"]] = record
print(os.path.basename(replay), "counter increments:", len(events))
for frame, actor, value in events:
    key = [c["player_key"] for c in frames[frame]["observations"]["cars"] if c["actor_id"] == actor][0]
    slot = slot_of[key]
    flags = []
    for g in range(frame - 6, frame + 8):
        car = [c for c in frames[g]["state"]["cars"] if c["slot"] == slot][0]
        flags.append(
            ("J" if car["has_jumped"] else "-")
            + ("D" if car["has_double_jumped"] else "-")
            + ("F" if car["has_flipped"] else "-")
            + ("g" if car["is_on_ground"] else ".")
        )
    print(f"  actor {actor} -> {value} at frame {frame}:", " ".join(flags))
