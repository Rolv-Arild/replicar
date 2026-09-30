"""Check the converter's inferred packet lags against the server's true ticks (RLBot recording).

For a replay's fresh car packets that match a server tick exactly, the true lag (up to a slowly varying
clock offset) is replay_time*120 - frame_num. The inferred chain lag of the same packet (from
dump_replay_packets) should track it: after removing the slow offset from both, the difference is the
inference error. Reports the spread of the true lag, the inferred lag, and the error, per lag source.

usage: python scripts/rlbot_lag_check.py <states.jsonl> <replay_packets.jsonl>
"""
import json, sys, collections
import numpy as np

states_path, replay_path = sys.argv[1], sys.argv[2]
key = lambda p: (round(p[0] * 100), round(p[1] * 100), round(p[2] * 100))
truth = collections.defaultdict(lambda: collections.defaultdict(list))
with open(states_path) as f:
    for line in f:
        p = json.loads(line)['packet']
        fn = p['match_info']['frame_num']
        for pl in p['players']:
            ph = pl['physics']['location']
            truth[pl['name']][key((ph['x'], ph['y'], ph['z']))].append(fn)
rows = [json.loads(l) for l in open(replay_path, encoding='utf-8')]
names = {p['key']: p['name'] for p in rows[0]['players']}
rows = rows[1:]
recs = []  # (t, off, inferred lag or None, source)
for row in rows:
    f = row['f']
    lag_by_actor = {l['actor']: (l['ticks'], l['source']) for l in (row.get('lags') or []) if l['actor'] is not None}
    for c in row['cars']:
        pos = c['pos']
        if not pos or pos[1] != f:
            continue
        name = names.get(c['key'])
        hits = truth[name].get(key(pos[0])) if name else None
        if not hits or len(hits) != 1:
            continue
        lag = lag_by_actor.get(c['actor'])
        recs.append((row['t'], row['t'] * 120 - hits[0], lag[0] if lag else None, lag[1] if lag else 'none'))
recs.sort()
t = np.array([r[0] for r in recs]); off = np.array([r[1] for r in recs])
w = 150
def detrend(x):
    return x - np.array([np.median(x[max(0, i - w):i + w + 1]) for i in range(len(x))])
d_true = detrend(off)
print(f"{len(recs)} matched car packets; true lag spread (detrended) p1/p10/p50/p90/p99: {np.round(np.percentile(d_true, [1,10,50,90,99]),2)}")
by_source = collections.defaultdict(list)
for i, r in enumerate(recs):
    by_source[r[3]].append(i)
for src, idx in sorted(by_source.items()):
    idx = np.array(idx)
    lag = np.array([recs[i][2] if recs[i][2] is not None else np.nan for i in idx], dtype=float)
    ok = ~np.isnan(lag)
    if ok.sum() < 20:
        print(f"  source {src}: {len(idx)} packets")
        continue
    # detrend the inferred lag over the same sequence positions (keep global order)
    full = np.full(len(recs), np.nan)
    full[idx] = lag
    lag_d = np.full(len(recs), np.nan)
    for i in idx:
        lo, hi = max(0, i - w), min(len(recs), i + w + 1)
        seg = full[lo:hi]; seg = seg[~np.isnan(seg)]
        lag_d[i] = full[i] - np.median(seg)
    tr = d_true[idx]; inf = lag_d[idx]
    err = tr - inf
    corr = np.corrcoef(tr, inf)[0, 1]
    print(f"  source {src}: {len(idx)} packets; inferred lag (detrended) p10/p50/p90 {np.round(np.percentile(inf,[10,50,90]),2)}; corr with the true lag {corr:.3f}; error p10/p50/p90 {np.round(np.percentile(err,[10,50,90]),2)}, |error| p50/p90 {np.round(np.percentile(np.abs(err),[50,90]),2)}; |true| p50/p90 {np.round(np.percentile(np.abs(tr),[50,90]),2)}")

# Absolute level: the true lag is at least 0 and spans the frame window; its running minimum is the
# lag-0 offset. Compare the mean true lag with the mean inferred lag.
floor = np.array([off[max(0, i - w):i + w + 1].min() for i in range(len(off))])
true_lag = off - floor
inferred = np.array([r[2] if r[2] is not None and r[3] == 'chain' else np.nan for r in recs], dtype=float)
ok = ~np.isnan(inferred)
print(f"absolute level: true lag (offset minus its running minimum) mean {true_lag.mean():.2f} p10/p50/p90 {np.round(np.percentile(true_lag,[10,50,90]),1)}; inferred chain lag mean {np.nanmean(inferred):.2f} p10/p50/p90 {np.round(np.nanpercentile(inferred,[10,50,90]),1)}")
