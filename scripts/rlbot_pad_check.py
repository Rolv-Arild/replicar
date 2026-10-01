"""Boost pad pickups of a converted replay against the boost jumps in the server truth (the RLBot recordings have no pad data).

A truth pickup is a tick where a car's boost rises by more than 5 (a small pad gives 12, a big one fills to 100). The replay's
pickups are `observations.pad_pickups` (instigator car, pad actor); the converter's own `car_pickup_boost` simulated events are
counted separately, so a pickup reported by both sources shows up as a surplus.

usage: python scripts/rlbot_pad_check.py <states.jsonl> <converted.jsonl>
"""
import json, sys, collections
import numpy as np
exec(open('scripts/rlbot_events_check.py').read().split("series = collections.defaultdict")[0])
slot_name = {s['slot']: keys.get(s['player_key']) for s in header['car_slots']}
# truth boost per car per frame
boost = collections.defaultdict(dict)
seen = set()
for line in open(states_path):
    p = json.loads(line)['packet']; fn = p['match_info']['frame_num']
    if fn in seen: continue
    seen.add(fn)
    for pl in p['players']:
        boost[pl['name']][fn] = pl['boost']
truth_pick = []
for n, d in boost.items():
    fs = sorted(d)
    for a, b in zip(fs[:-1], fs[1:]):
        if b == a + 1 and d[b] - d[a] > 5:
            # a kickoff or respawn reset also raises the boost (to 33): not a pad
            if truth[b]['phase'] != truth[a]['phase'] or truth[b]['phase'] in (1, 4, 5, 6, 7):
                continue
            if truth[b]['players'][n]['demolished'] != truth[a]['players'][n]['demolished'] or truth[a]['players'][n]['demolished']:
                continue
            if abs(d[b] - 33.3) < 0.5 and d[a] < 25 and truth[b]['phase'] == 2:
                continue
            truth_pick.append((b, n, round(d[b] - d[a])))
actor_name = {}
for r in recs:
    for c in r['observations']['cars']:
        nm = keys.get(c.get('player_key'))
        if nm: actor_name[c['actor_id']] = nm
rep = []
raw = 0
for r in recs:
    for pk in r['observations']['pad_pickups']:
        raw += 1
        if pk['picked_up'] in (255,) or pk['instigator_car_id'] is None:
            continue
        rep.append((r['timeline_tick'] - floor_at(r['frame']), actor_name.get(pk['instigator_car_id']), pk['pad_actor_id'], pk['picked_up'], r['frame']))
sim = []
for r in recs:
    fl = floor_at(r['frame'])
    for e in r['simulated_events']:
        ev = e['event']
        if ev['kind'] == 'car_pickup_boost':
            sim.append((r['timeline_tick'] - (r['state']['arena_tick'] - e['arena_tick']) - fl, slot_name.get(ev.get('car_slot'))))
print(f"truth pickups (boost jumps > 5): {len(truth_pick)} (small {sum(1 for t in truth_pick if t[2] < 50)}); replay pad_pickups records {raw}, with an instigator {len(rep)}; simulated car_pickup_boost events {len(sim)}")
def one_to_one(evs, window=40):
    bt = collections.defaultdict(list)
    for fn, n, _ in truth_pick: bt[n].append(fn)
    used = set(); matched = 0; offs = []; surplus = []
    for ev in sorted(evs, key=lambda e: e[0]):
        t, n = ev[0], ev[1]
        c = [(abs(t - fn), fn) for fn in bt.get(n, []) if (fn, n) not in used and abs(t - fn) <= window]
        if c:
            d, fn = min(c); used.add((fn, n)); matched += 1; offs.append(t - fn)
        else:
            surplus.append(ev)
    return matched, surplus, len(truth_pick) - len(used), offs
for label, evs in (("replay pad_pickups", rep), ("simulated pickups", sim)):
    m, s, missed, offs = one_to_one(evs)
    o = np.array(offs) if offs else np.array([0])
    print(f"  {label}: matched {m}, surplus {len(s)}, truth pickups unmatched {missed}; event minus truth tick p10/p50/p90 {np.percentile(o,10):.0f}/{np.percentile(o,50):.0f}/{np.percentile(o,90):.0f}")
# repeated records of one pickup in the replay: same pad and car within 60 ticks
def boost_at_(n, t):
    d = boost.get(n, {})
    return d.get(t, d.get(t - 1))
# a pad's picked_up counter value is re-announced later (at resets): keep the first record of each (pad, value)
firsts = {}
for e in sorted(rep, key=lambda e: e[0]):
    firsts.setdefault((e[2], e[3]), e)
rep_unique = list(firsts.values())
flagged_new = sum(1 for r in recs for pk in r['observations']['pad_pickups'] if pk['instigator_car_id'] is not None and pk['picked_up'] != 255 and not pk.get('repeat', False))
print(f"  records the converter flags as new (repeat false): {flagged_new}")
print(f"  unique (pad, counter value) pickups: {len(rep_unique)} of {len(rep)} records")
m2, s2, missed2, offs2 = one_to_one(rep_unique)
o2 = np.array(offs2) if offs2 else np.array([0])
full2 = [e for e in s2 if (boost_at_(e[1], e[0]) or 0) >= 88]
print(f"  after merging re-announcements: matched {m2}, truth pickups unmatched {missed2}, surplus {len(s2)} (of which car had >= 88 boost: {len(full2)}, unexplained {len(s2)-len(full2)}); tick offset p10/p50/p90 {np.percentile(o2,10):.0f}/{np.percentile(o2,50):.0f}/{np.percentile(o2,90):.0f}")
rep_sorted = sorted(rep, key=lambda e: e[0])
dups = sum(1 for a, b in zip(rep_sorted[:-1], rep_sorted[1:]) if a[1] == b[1] and a[2] == b[2] and b[0] - a[0] <= 60)
print(f"  replay records repeating the same car and pad within 60 ticks: {dups}")
vals = collections.Counter()
for r in recs:
    for pk in r['observations']['pad_pickups']:
        vals[(pk['picked_up'], pk['instigator_car_id'] is not None)] += 1
print("  records by (picked_up value, has instigator):", dict(sorted(vals.items(), key=lambda kv: -kv[1])[:12]))
m, s, missed, offs = one_to_one(rep)
def boost_at(n, t):
    d = boost.get(n, {})
    return d.get(t, d.get(t - 1))
full = [e for e in s if (boost_at(e[1], e[0]) or 0) >= 88]
print(f"  surplus replay pickups: {len(s)}; of those the car already had >= 88 boost at that tick (a pickup with no visible jump): {len(full)}; remaining unexplained {len(s)-len(full)}")
# name-free matching: does each replay pickup coincide in time with some truth pickup, and is the car the same?
tp = sorted(truth_pick)
used = set(); right = wrong = none = 0
for e in sorted(rep_unique, key=lambda e: e[0]):
    c = [(abs(e[0] - t[0]), i) for i, t in enumerate(tp) if i not in used and abs(e[0] - t[0]) <= 15]
    if not c:
        none += 1; continue
    d, i = min(c); used.add(i)
    if tp[i][1] == e[1]: right += 1
    else: wrong += 1
print(f"  time-only matching of the merged replay pickups (+-15 ticks): same car {right}, a different car {wrong}, no truth pickup near {none}; truth pickups not matched {len(tp)-len(used)}")
name_actors = collections.defaultdict(set)
for a, n in actor_name.items(): name_actors[n].add(a)
inst_of = {}
for r in recs:
    for pk in r['observations']['pad_pickups']:
        inst_of[(pk['pad_actor_id'], pk['picked_up'])] = pk['instigator_car_id']
used = set(); shown = 0; cnt = collections.Counter()
for e in sorted(rep_unique, key=lambda e: e[0]):
    c = [(abs(e[0] - t[0]), i) for i, t in enumerate(tp) if i not in used and abs(e[0] - t[0]) <= 15]
    if not c: continue
    d, i = min(c); used.add(i)
    if tp[i][1] != e[1]:
        inst = inst_of[(e[2], e[3])]
        # is the instigator id a known car actor of the truth car? of another car? unknown?
        cnt[("instigator is a known car of the truth car" if inst in name_actors.get(tp[i][1], ()) else "known car of another player" if inst in actor_name else "unknown actor id")] += 1
        if shown < 5:
            print("   example: replay instigator", inst, "->", e[1], "; truth car", tp[i][1], "actors of truth car", sorted(name_actors.get(tp[i][1], ())), "tick", e[0]); shown += 1
print("  wrong-car cases by instigator kind:", dict(cnt))
# which truth pickups have no replay record (name-constrained, +-40 ticks)?
bt = collections.defaultdict(list)
for e in rep_unique: bt[e[1]].append(e[0])
usedr = set(); unmatched = []
for fn, n, jump in sorted(truth_pick):
    c = [(abs(t - fn), t) for t in bt.get(n, []) if (n, t) not in usedr and abs(t - fn) <= 40]
    if c:
        usedr.add((n, min(c)[1]))
    else:
        unmatched.append((fn, n, jump))
kinds = collections.Counter(("big (jump >= 50)" if j >= 50 else "small") for _, _, j in unmatched)
alln = collections.Counter(("big (jump >= 50)" if j >= 50 else "small") for _, _, j in truth_pick)
print(f"  truth pickups without a replay record: {dict(kinds)} of {dict(alln)}; with any replay record of that car within 120 ticks: {sum(1 for fn,n,j in unmatched if any(abs(t-fn)<=120 for t in bt.get(n,[])))}")
