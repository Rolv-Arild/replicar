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
recs = []  # (frame, obj, off, lag)
for row in rows:
    f = row['f']; tick = round((row['t'] - t0) * 120)
    lags = {l['actor']: (l['ticks'], l['source']) for l in (row.get('lags') or [])}
    b = row.get('ball')
    if b and b['pos'] and b['pos'][1] == f:
        h = ball_truth.get(key(b['pos'][0]))
        l = lags.get(None)
        if h and len(h) == 1 and l and l[1] == 'chain': recs.append((f, 'ball', tick - h[0], l[0]))
    for c in row['cars']:
        pos = c['pos']
        if not pos or pos[1] != f: continue
        nm = names.get(c['key']); h = car_truth[nm].get(key(pos[0])) if nm else None
        l = lags.get(c['actor'])
        if h and len(h) == 1 and l and l[1] == 'chain': recs.append((f, ('car', c['actor']), tick - h[0], l[0]))
recs.sort(key=lambda r: r[0])
off = np.array([r[2] for r in recs], float); fr = np.array([r[0] for r in recs])
w = 300
floor = np.array([off[max(0,i-w):i+w+1].min() for i in range(len(off))])
true = off - floor
inf = np.array([r[3] for r in recs], float)
err = inf - true
# runs: consecutive packets of one object whose inferred interval equals the true one (error difference 0) and gap<=3 frames
byobj = collections.defaultdict(list)
for i, r in enumerate(recs): byobj[r[1]].append(i)
runs = []
for obj, idx in byobj.items():
    idx.sort(key=lambda i: recs[i][0])
    cur = [idx[0]]
    for p, q in zip(idx[:-1], idx[1:]):
        if recs[q][0]-recs[p][0] <= 3 and abs((err[q]-err[p])) < 0.5: cur.append(q)
        else:
            runs.append((obj, cur)); cur = [q]
    runs.append((obj, cur))
for kind in ('ball', 'car'):
    rl = [(len(c), np.mean(err[c])) for o, c in runs if (o == 'ball') == (kind == 'ball') and len(c) >= 3]
    if not rl: continue
    L = np.array([x[0] for x in rl]); E = np.array([x[1] for x in rl])
    print(f"{kind}: {len(rl)} runs (>=3 packets); run length p10/p50/p90 {np.percentile(L,[10,50,90])}; run-level error (inferred - true) mean {E.mean():+.2f}, |err| p50/p90 {np.round(np.percentile(np.abs(E),[50,90]),2)}")
    for lo,hi in ((3,6),(6,12),(12,30),(30,1000)):
        m=(L>=lo)&(L<hi)
        if m.sum()>3: print(f"   length {lo}-{hi}: n {m.sum()}  |err| p50/p90 {np.round(np.percentile(np.abs(E[m]),[50,90]),2)}")
