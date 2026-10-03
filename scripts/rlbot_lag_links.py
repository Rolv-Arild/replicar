"""Accuracy of the intervals implied by consecutive chain lags (links) against the server truth, by speed, acceleration, interval and height.

usage: python scripts/rlbot_lag_links.py <states.jsonl> <replay_packets.jsonl>
"""
import json, sys, collections
import numpy as np
states_path, replay_path = sys.argv[1], sys.argv[2]
key = lambda p: (round(p[0]*100), round(p[1]*100), round(p[2]*100))
truth = collections.defaultdict(lambda: collections.defaultdict(list))
with open(states_path) as f:
    for line in f:
        p = json.loads(line)['packet']; fn = p['match_info']['frame_num']
        for pl in p['players']:
            ph = pl['physics']['location']; truth[pl['name']][key((ph['x'],ph['y'],ph['z']))].append(fn)
rows = [json.loads(l) for l in open(replay_path, encoding='utf-8')]
names = {p['key']: p['name'] for p in rows[0]['players']}; rows = rows[1:]
t0 = rows[0]['t']
per = collections.defaultdict(list)  # actor -> list of (frame, tick_frame, fn, inferred lag, pos, vel)
for row in rows:
    f = row['f']; tk = round((row['t']-t0)*120)
    lags = {l['actor']: (l['ticks'], l['source']) for l in (row.get('lags') or []) if l['actor'] is not None}
    for c in row['cars']:
        pos = c['pos']
        if not pos or pos[1] != f: continue
        nm = names.get(c['key']); hits = truth[nm].get(key(pos[0])) if nm else None
        if not hits or len(hits) != 1: continue
        l = lags.get(c['actor'])
        if not l or l[1] != 'chain': continue
        per[c['actor']].append((f, tk, hits[0], l[0], np.array(pos[0]), np.array(c['vel'][0])))
links = []
for a, lst in per.items():
    lst.sort(key=lambda x: x[0])
    for p, q in zip(lst[:-1], lst[1:]):
        if q[0] - p[0] > 2: continue
        true_dt = q[2] - p[2]                  # true ticks between the packets
        nominal = q[1] - p[1]                   # replay clock ticks between frames
        inf_dt = nominal - (q[3] - p[3])        # interval implied by inferred lags
        speed = np.linalg.norm(q[5] + p[5]) / 2
        acc = np.linalg.norm(q[5] - p[5]) / max(true_dt, 1) * 120
        links.append((inf_dt - true_dt, speed, acc, true_dt, q[4][2]))
L = np.array(links)
print('links', len(L), 'signed error mean', L[:,0].mean().round(3), 'std', L[:,0].std().round(3))
def show(name, mask):
    m = mask
    if m.sum() > 50: print(f"  {name:<24} n {m.sum():>6} signed mean {L[m,0].mean():+.3f} std {L[m,0].std():.3f} |e| p50/p90 {np.round(np.percentile(np.abs(L[m,0]),[50,90]),2)}")
for lo, hi in ((0,900),(900,1500),(1500,2300)): show(f'speed {lo}-{hi}', (L[:,1]>=lo)&(L[:,1]<hi))
for lo, hi in ((0,300),(300,800),(800,1500),(1500,1e9)): show(f'|accel| {lo}-{hi:.0f}', (L[:,2]>=lo)&(L[:,2]<hi))
for lo, hi in ((1,7),(7,11),(11,15),(15,30)): show(f'true interval {lo}-{hi}', (L[:,3]>=lo)&(L[:,3]<hi))
show('z<25', L[:,4]<25); show('z>=25', L[:,4]>=25)
