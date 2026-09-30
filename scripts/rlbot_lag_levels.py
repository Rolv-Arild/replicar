"""Absolute lag level of the converter against the server truth, for the ball and the cars (chain lags vs a common baseline: the running minimum of the true offsets).

usage: python scripts/rlbot_lag_levels.py <states.jsonl> <replay_packets.jsonl>
"""
import json, sys, collections
import numpy as np
states_path, replay_path = sys.argv[1], sys.argv[2]
key = lambda p: (round(p[0]*100), round(p[1]*100), round(p[2]*100))
car_truth = collections.defaultdict(lambda: collections.defaultdict(list)); ball_truth = collections.defaultdict(list)
with open(states_path) as f:
    for line in f:
        p = json.loads(line)['packet']; fn = p['match_info']['frame_num']
        for pl in p['players']:
            ph = pl['physics']['location']; car_truth[pl['name']][key((ph['x'],ph['y'],ph['z']))].append(fn)
        if p['balls']:
            b = p['balls'][0]['physics']['location']; ball_truth[key((b['x'],b['y'],b['z']))].append(fn)
rows = [json.loads(l) for l in open(replay_path, encoding='utf-8')]
names = {p['key']: p['name'] for p in rows[0]['players']}; rows = rows[1:]
t0 = rows[0]['t']
recs = []  # (frame, kind, off, inferred, source)
for row in rows:
    f = row['f']; tick = round((row['t'] - t0) * 120)
    lags = {l['actor']: (l['ticks'], l['source']) for l in (row.get('lags') or [])}
    b = row.get('ball')
    if b and b['pos'] and b['pos'][1] == f:
        hits = ball_truth.get(key(b['pos'][0]))
        if hits and len(hits) == 1:
            l = lags.get(None)
            recs.append((f, 'ball', tick - hits[0], l[0] if l else None, l[1] if l else 'none'))
    for c in row['cars']:
        pos = c['pos']
        if not pos or pos[1] != f: continue
        nm = names.get(c['key'])
        hits = car_truth[nm].get(key(pos[0])) if nm else None
        if hits and len(hits) == 1:
            l = lags.get(c['actor'])
            recs.append((f, 'car', tick - hits[0], l[0] if l else None, l[1] if l else 'none'))
recs.sort(key=lambda r: r[0])
fr = np.array([r[0] for r in recs]); off = np.array([r[2] for r in recs], float)
w = 300
floor = np.array([off[max(0,i-w):i+w+1].min() for i in range(len(off))])
true = off - floor
for kind in ('ball', 'car'):
    idx = [i for i, r in enumerate(recs) if r[1] == kind and r[3] is not None and r[4] == 'chain']
    if not idx: continue
    inf = np.array([recs[i][3] for i in idx], float); tr = true[idx]
    print(f"{kind}: chain packets {len(idx)}; inferred mean {inf.mean():.2f}, true mean {tr.mean():.2f}; inferred - true: mean {np.mean(inf-tr):+.2f} p10/p50/p90 {np.round(np.percentile(inf-tr,[10,50,90]),2)}; corr {np.corrcoef(inf,tr)[0,1]:.3f}")
# per source for ball
for kind in ('ball',):
    srcs = collections.Counter(r[4] for r in recs if r[1] == kind)
    print(kind, 'sources', dict(srcs))
