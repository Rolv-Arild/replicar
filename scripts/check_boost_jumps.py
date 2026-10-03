"""Exported boost increases that the replay does not support, in a converted JSONL (convert_replay).

An increase of more than 5 units of a car's exported boost between consecutive frames is 'supported' when the
replay's boost value for that car was updated at that frame or a pad pickup of that car is reported in it. Any
other increase is the simulation picking up a pad the replay never reported (pad blocking is meant to prevent it).

usage: python scripts/check_boost_jumps.py <converted.jsonl>
"""
import json
import sys

path = sys.argv[1]
with open(path, encoding="utf-8") as f:
    header = json.loads(f.readline())
    slot_of_key = {s["player_key"]: s["slot"] for s in header["car_slots"]}
    previous = {}
    jumps = supported = 0
    examples = []
    for line in f:
        frame = json.loads(line)
        index = frame["frame"]
        obs = {slot_of_key.get(c["player_key"]): c for c in frame["observations"]["cars"]}
        picked = {p.get("instigator_car_id") for p in frame["observations"].get("pad_pickups", [])}
        for car in frame["state"]["cars"]:
            slot, boost = car["slot"], car["boost"]
            before = previous.get(slot)
            previous[slot] = boost
            if before is None or boost - before <= 5.0:
                continue
            jumps += 1
            o = obs.get(slot)
            fresh = o is not None and o.get("boost") is not None and o["boost"]["frame"] == index
            by_pickup = o is not None and o["actor_id"] in picked
            if fresh or by_pickup:
                supported += 1
            elif len(examples) < 8:
                examples.append((index, slot, round(before, 1), round(boost, 1), o and o["boost"] and o["boost"]["frame"]))
print(f"boost increases above 5: {jumps}, supported by a fresh replay boost or a reported pickup: {supported}, unsupported: {jumps - supported}")
for e in examples:
    print("  unsupported (frame, slot, before, after, frame of the replay's boost value):", e)
