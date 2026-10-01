"""Scoreboard, clock, phase and event fields of a converted replay against the server truth (RLBot recording).

For each frame the replay's team scores, seconds remaining, game state and each player's stats are compared with
the server's values at the frame's server tick (from the exactly matched car packets: tick = timeline tick minus
the running minimum offset, exact on a lag-free host replay, about +-5 ticks on a 10 fps client). Reported:
agreement of the value at every frame, and, for each change in the truth, how long the replay took to show it
(in ticks and frames), and changes the replay shows that the truth does not have.

usage: python scripts/rlbot_events_check.py <states.jsonl> <converted.jsonl from convert_replay>
"""
import json, sys, collections
import numpy as np

states_path, conv_path = sys.argv[1], sys.argv[2]
key = lambda p: (round(p[0] * 100), round(p[1] * 100), round(p[2] * 100))
truth_pos = collections.defaultdict(lambda: collections.defaultdict(list))
truth = {}  # frame_num -> dict
names_truth = []
with open(states_path) as f:
    for line in f:
        p = json.loads(line)['packet']
        fn = p['match_info']['frame_num']
        if fn in truth:
            continue
        mi = p['match_info']
        rec = {
            'scores': [t['score'] for t in p['teams']],
            'remaining': mi['game_time_remaining'],
            'phase': mi['match_phase'],
            'overtime': mi['is_overtime'],
            'players': {},
        }
        for pl in p['players']:
            si = pl['score_info']
            rec['players'][pl['name']] = {
                'score': si['score'], 'goals': si['goals'], 'assists': si['assists'], 'saves': si['saves'],
                'shots': si['shots'], 'demolitions': si['demolitions'], 'demolished': pl['demolished_timeout'] >= 0,
            }
            ph = pl['physics']['location']
            truth_pos[pl['name']][key((ph['x'], ph['y'], ph['z']))].append(fn)
        truth[fn] = rec
ticks = sorted(truth)

recs = []
header = None
with open(conv_path, encoding='utf-8') as f:
    for line in f:
        d = json.loads(line)
        if d['record_type'] != 'frame':
            header = d
            continue
        recs.append(d)
# player names by key
keys = {}
for r in recs[:50]:
    for p in r['observations']['players']:
        keys[p['key']] = p['name']
for r in recs:
    for p in r['observations']['players']:
        keys.setdefault(p['key'], p['name'])

# frame -> server tick offset from matched fresh car packets
offs = []
for r in recs:
    f = r['frame']
    for c in r['observations']['cars']:
        pos = c['body'].get('position')
        key_name = keys.get(c.get('player_key'))
        if not pos or pos['frame'] != f or not key_name:
            continue
        hits = truth_pos[key_name].get(key(pos['value']))
        if hits and len(hits) == 1:
            offs.append((f, r['timeline_tick'] - hits[0]))
mf = np.array([o[0] for o in offs]); mo = np.array([o[1] for o in offs])
def floor_at(f):
    i = int(np.searchsorted(mf, f)); lo, hi = max(0, i - 150), min(len(mf), i + 151)
    return int(mo[lo:hi].min())

def val(x):
    return None if x is None else x['value']

series = collections.defaultdict(list)  # name -> [(frame, server_tick, replay value, truth value)]
phase_names = collections.Counter()
for r in recs:
    f = r['frame']
    st = r['timeline_tick'] - floor_at(f)
    # nearest truth tick
    if st not in truth:
        continue
    t = truth[st]
    o = r['observations']
    ts = o['team_scores']
    for i in range(2):
        series[f'team {i} score'].append((f, st, val(ts[i]) if len(ts) > i else None, t['scores'][i]))
    # in overtime the replay counts seconds up from 0 while the server's remaining time is negative
    series['seconds remaining'].append((f, st, val(o['seconds_remaining']), abs(t['remaining']) if t['overtime'] else t['remaining']))
    series['game state'].append((f, st, val(o['game_state']), t['phase']))
    for p in o['players']:
        nm = p['name']
        if nm not in t['players']:
            continue
        tp = t['players'][nm]
        s = p['stats']
        for rk, tk in (('match_score', 'score'), ('goals', 'goals'), ('assists', 'assists'), ('saves', 'saves'),
                       ('shots', 'shots'), ('demolishes', 'demolitions')):
            series[f'{nm}: {rk}'].append((f, st, val(s[rk]), tp[tk]))

def summarize(label, rows, numeric=True):
    # agreement at every frame, treating an absent replay value as 0 (never reported) separately
    n = len(rows)
    absent = sum(1 for r in rows if r[2] is None)
    agree = sum(1 for r in rows if r[2] == r[3])
    agree_zero = sum(1 for r in rows if (r[2] if r[2] is not None else 0) == r[3])
    # truth changes and the replay's delay
    delays = []
    missed = 0
    prev = rows[0][3]
    for i in range(1, n):
        if rows[i][3] != prev:
            target = rows[i][3]
            j = next((k for k in range(i, min(n, i + 400)) if rows[k][2] == target), None)
            if j is None:
                missed += 1
            else:
                delays.append(rows[j][1] - rows[i][1])
            prev = rows[i][3]
    d = np.array(delays) if delays else np.array([0])
    print(f"{label:<44} frames {n:>6}  equal {agree/n:6.1%}  equal(absent=0) {agree_zero/n:6.1%}  absent {absent/n:6.1%}  truth changes {len(delays)+missed:>3}, shown {len(delays):>3}, delay ticks p50/p90 {np.percentile(d,50):.0f}/{np.percentile(d,90):.0f}")

for label in ['team 0 score', 'team 1 score', 'seconds remaining']:
    rows = series[label]
    if label == 'seconds remaining':
        # whole seconds: compare against floor/ceil of the truth
        rows = [(f, st, a, int(np.ceil(b)) if b is not None else None) for f, st, a, b in rows]
    summarize(label, rows)
phase = series['game state']
print("game state vs match_phase (replay value x truth phase):")
c = collections.Counter((r[2], r[3]) for r in phase)
for k, v in sorted(c.items(), key=lambda kv: -kv[1])[:14]:
    print(f"   {k}: {v}")
for label in sorted(k for k in series if ':' in k):
    summarize(label, series[label])

# ---- touches: truth latest_touch changes vs the converter's car_hit_ball events (with a hit impulse) ----
slot_name = {s['slot']: keys.get(s['player_key']) for s in header['car_slots']}
truth_touches = []  # (server frame_num, name)
last_gs = {}
with open(states_path) as f:
    seen = set()
    for line in f:
        p = json.loads(line)['packet']
        fn = p['match_info']['frame_num']
        if fn in seen:
            continue
        seen.add(fn)
        for pl in p['players']:
            lt = pl['latest_touch']
            if lt and last_gs.get(pl['name']) != lt['game_seconds']:
                if pl['name'] in last_gs or True:
                    truth_touches.append((fn, pl['name']))
                last_gs[pl['name']] = lt['game_seconds']
# the first value seen per player is a touch from before the recording only if the recording started mid-match; keep all
sim_touches = []  # (server tick, name): the converter's `touches` (first tick of each contact)
for r in recs:
    fl = floor_at(r['frame'])
    for t in r.get('touches', []):
        sim_touches.append((t['tick'] - fl, slot_name.get(t['car_slot'])))
print(f"\ntouches: truth {len(truth_touches)}, converter touches {len(sim_touches)}")
bt = collections.defaultdict(list)
for fn, n in truth_touches:
    bt[n].append(fn)
used = set(); offs_t = []; surplus = 0
for t, n in sorted(sim_touches, key=lambda e: e[0]):
    cand = [(abs(t - fn), fn) for fn in bt.get(n, []) if (fn, n) not in used and abs(t - fn) <= 20]
    if cand:
        dd_, fn = min(cand); used.add((fn, n)); offs_t.append(t - fn)
    else:
        surplus += 1
o = np.array(offs_t) if offs_t else np.array([0])
print(f"  one-to-one (+-20 ticks): truth touches reproduced {len(offs_t)} ({len(offs_t)/max(1,len(truth_touches)):.1%}), missed {len(truth_touches)-len(offs_t)}; surplus converter touches {surplus} ({surplus/max(1,len(sim_touches)):.1%} of {len(sim_touches)})")
print(f"  converter minus truth tick p10/p50/p90: {np.percentile(o,10):.0f}/{np.percentile(o,50):.0f}/{np.percentile(o,90):.0f}")

# ---- demolitions: truth demolished_timeout onsets vs the converter's car state is_demoed ----
truth_demo_on = []  # (frame_num, name)
prev_dem = {}
with open(states_path) as f:
    seen = set()
    for line in f:
        p = json.loads(line)['packet']
        fn = p['match_info']['frame_num']
        if fn in seen:
            continue
        seen.add(fn)
        for pl in p['players']:
            dm = pl['demolished_timeout'] >= 0
            if dm and not prev_dem.get(pl['name'], False):
                truth_demo_on.append((fn, pl['name']))
            prev_dem[pl['name']] = dm
sim_demo_on = []
prev = {}
for r in recs:
    fl = floor_at(r['frame'])
    for i, c in enumerate(r['state']['cars']):
        d = bool(c.get('is_demoed'))
        if d and not prev.get(i, False):
            sim_demo_on.append((r['timeline_tick'] - fl, slot_name.get(c['slot'] if 'slot' in c else i)))
        prev[i] = d
print(f"\ndemolitions: truth {len(truth_demo_on)}, converter is_demoed onsets {len(sim_demo_on)}")
by = collections.defaultdict(list)
for t, n in sim_demo_on:
    by[n].append(t)
od = []
for fn, name in truth_demo_on:
    cand = [t for t in by.get(name, []) if abs(t - fn) <= 120]
    if cand:
        od.append(min(cand, key=lambda t: abs(t - fn)) - fn)
o = np.array(od) if od else np.array([0])
print(f"  matched {len(od)}; sim minus truth onset ticks p10/p50/p90 {np.percentile(o,10):.0f}/{np.percentile(o,50):.0f}/{np.percentile(o,90):.0f}")

# ---- goals: replay goal events vs the truth's team score changes ----
truth_goal = []
prev_s = None
for fn in ticks:
    s = truth[fn]['scores']
    if prev_s is not None:
        for i in range(2):
            if s[i] != prev_s[i]:
                truth_goal.append((fn, i))
    prev_s = s
replay_goal = []
for r in recs:
    for e in r['observations']['events']:
        if e['kind'] == 'goal_scored_on':
            replay_goal.append((r['timeline_tick'] - floor_at(r['frame']), e['team']))
print(f"\ngoals: truth score changes {truth_goal}\n       replay goal_scored_on events (tick, team scored ON) {replay_goal}")
if len(sys.argv) > 3:
    print("truth onsets:", truth_demo_on)
    print("sim onsets:", sorted(sim_demo_on))
    tot = collections.Counter()
    last = truth[ticks[-1]]['players']
    print("truth demolitions stat:", {n: p['demolitions'] for n, p in last.items()})

# ---- observed demolish events (replay attribute) vs truth onsets ----
actor_name = {}
for r in recs:
    for c in r['observations']['cars']:
        n = keys.get(c.get('player_key'))
        if n:
            actor_name[c['actor_id']] = n
obs_demo = []
for r in recs:
    for e in r['observations']['events']:
        if e['kind'] == 'demolish' and e['source'] != 'goal_explosion':
            obs_demo.append((r['timeline_tick'] - floor_at(r['frame']), actor_name.get(e['victim_car']), actor_name.get(e['attacker_car']),
                             tuple(e['victim_velocity']), e['self_demolish']))
# dedupe repeated updates of one demolition (same victim within 60 ticks)
dd = []
for t, v, a, vv, sd in sorted(obs_demo):
    if dd and dd[-1][1] == v and t - dd[-1][0] <= 60:
        continue
    dd.append((t, v, a, vv, sd))
print(f"\nobserved demolish events: {len(obs_demo)} raw, {len(dd)} after merging repeats; truth onsets {len(truth_demo_on)}")
od2 = []
unmatched_truth = []
for fn, name in truth_demo_on:
    cand = [d for d in dd if d[1] == name and abs(d[0] - fn) <= 120]
    if cand:
        od2.append(min(cand, key=lambda d: abs(d[0] - fn))[0] - fn)
    else:
        unmatched_truth.append((fn, name))
o = np.array(od2) if od2 else np.array([0])
print(f"  truth onsets with an observed event: {len(od2)}; event minus truth onset ticks p10/p50/p90 {np.percentile(o,10):.0f}/{np.percentile(o,50):.0f}/{np.percentile(o,90):.0f}; unmatched truth {unmatched_truth}")
print(f"  observed events with no truth onset: {[d[:3] for d in dd if not any(d[1]==n and abs(d[0]-fn)<=120 for fn,n in truth_demo_on)]}")
for d in dd:
    if not any(d[1] == n and abs(d[0] - fn) <= 120 for fn, n in truth_demo_on):
        prev_on = [d[0] - fn for fn, n in truth_demo_on if n == d[1] and fn <= d[0]]
        print(f"  extra event {d[:3]}: ticks since this victim's last truth onset {min(prev_on) if prev_on else None}")

# ---- demolition duration: truth demolished window vs the converter's is_demoed window ----
def windows(flags):  # list of (tick, bool) -> list of (on, off)
    out = []; on = None
    for t, b in flags:
        if b and on is None: on = t
        if not b and on is not None: out.append((on, t)); on = None
    return out
tw = collections.defaultdict(list)
per = collections.defaultdict(list)
for fn in ticks:
    for n, p in truth[fn]['players'].items():
        per[n].append((fn, p['demolished']))
for n, fl in per.items():
    tw[n] = windows(fl)
sw = collections.defaultdict(list)
perm = collections.defaultdict(list)
for r in recs:
    fl_ = floor_at(r['frame'])
    for i, c in enumerate(r['state']['cars']):
        perm[slot_name.get(c.get('slot', i))].append((r['timeline_tick'] - fl_, bool(c.get('is_demoed'))))
for n, fl in perm.items():
    sw[n] = windows(fl)
dur = []
for n, w in tw.items():
    for a, b in w:
        c = [(x, y) for x, y in sw.get(n, []) if abs(x - a) <= 120]
        if c:
            x, y = min(c, key=lambda z: abs(z[0] - a))
            dur.append(((y - x) - (b - a), b - a, y - x))
if dur:
    dd_ = np.array(dur)
    print(f"demolition durations: matched {len(dur)}; truth ticks p50 {np.percentile(dd_[:,1],50):.0f}, converter p50 {np.percentile(dd_[:,2],50):.0f}; converter minus truth p10/p50/p90 {np.percentile(dd_[:,0],10):.0f}/{np.percentile(dd_[:,0],50):.0f}/{np.percentile(dd_[:,0],90):.0f}")

# ---- simulated demolitions (car_hit_car with is_demo) vs truth onsets and vs the observed events ----
sim_demo_ev = []
for r in recs:
    fl_ = floor_at(r['frame'])
    for e in r['simulated_events']:
        ev = e['event']
        if ev['kind'] == 'car_hit_car' and ev.get('is_demo'):
            t_ = r['timeline_tick'] - (r['state']['arena_tick'] - e['arena_tick']) - fl_
            sim_demo_ev.append((t_, slot_name.get(ev['victim_slot']), slot_name.get(ev['bumper_slot'])))
used_t = set(); m_ = 0; sur = []
offs_s = []
for t_, v_, a_ in sorted(sim_demo_ev, key=lambda e: e[0]):
    c = [(abs(t_ - fn), (fn, n)) for fn, n in truth_demo_on if n == v_ and (fn, n) not in used_t and abs(t_ - fn) <= 120]
    if c:
        _, k = min(c); used_t.add(k); m_ += 1; offs_s.append(t_ - k[0])
    else:
        sur.append((t_, v_, a_))
o = np.array(offs_s) if offs_s else np.array([0])
both = sum(1 for t_, v_, a_ in sim_demo_ev if any(d[1] == v_ and abs(d[0] - t_) <= 30 for d in dd))
print(f"\nsimulated demolitions (car_hit_car is_demo): {len(sim_demo_ev)}; matched to a truth onset {m_} (tick offset p10/p50/p90 {np.percentile(o,10):.0f}/{np.percentile(o,50):.0f}/{np.percentile(o,90):.0f}), surplus {sur}")
print(f"  of the simulated demolitions, {both} also have an observed event for the same victim within 30 ticks (the same demolition reported by both sources); observed events {len(dd)}")
