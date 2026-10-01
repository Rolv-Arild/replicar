"""Ball-only rollout residuals between consecutive ball packets (diagnose_ball_evidence) against the server's touches.

An interval is labelled 'touch' if a truth touch (latest_touch change of any car) falls inside the server ticks of its two
packets (matched by exact ball position). Reports detection against the velocity-residual threshold, split by whether the
interval is free flight (ball high, away from walls) or not.

usage: python scripts/rlbot_ball_evidence.py <states.jsonl> <diagnose_ball_evidence output>
"""
import json, sys, collections
import numpy as np
states_path, ev_path = sys.argv[1], sys.argv[2]
key = lambda x: (round(x[0] * 100), round(x[1] * 100), round(x[2] * 100))
bt = collections.defaultdict(list); last_gs = {}; touches = []; seen = set(); ball = {}
for line in open(states_path):
    p = json.loads(line)['packet']; fn = p['match_info']['frame_num']
    if fn in seen: continue
    seen.add(fn)
    if p['balls']:
        b = p['balls'][0]['physics']['location']; bt[key((b['x'], b['y'], b['z']))].append(fn)
    for pl in p['players']:
        lt = pl['latest_touch']
        if lt and last_gs.get(pl['name']) != lt['game_seconds']:
            touches.append(fn); last_gs[pl['name']] = lt['game_seconds']
touches = np.array(sorted(touches))
rows = []
for l in open(ev_path):
    v = l.split()
    fa, fb, ta, tb, d = map(int, v[:5]); rv, rp = float(v[5]), float(v[6]); pa = [float(x) for x in v[7:10]]; pb = [float(x) for x in v[10:13]]
    ha, hb = bt.get(key(pa)), bt.get(key(pb))
    if not ha or not hb or len(ha) != 1 or len(hb) != 1: continue
    sa, sb = ha[0], hb[0]
    if sb <= sa: continue
    has = bool(np.any((touches > sa) & (touches <= sb)))
    free = 150 < pa[2] < 1850 and 150 < pb[2] < 1850 and abs(pa[0]) < 3800 and abs(pb[0]) < 3800 and abs(pa[1]) < 4800 and abs(pb[1]) < 4800
    rows.append((rv, rp, has, free, sb - sa, d))
r = np.array(rows, float)
print(f"intervals {len(r)}; with a truth touch {int(r[:,2].sum())}; elapsed ticks from the lags equal the true gap in {np.mean(r[:,4]==r[:,5]):.1%} (within 1: {np.mean(abs(r[:,4]-r[:,5])<=1):.1%})")
for label, m in (("free flight", r[:, 3] == 1), ("near floor, ceiling or walls", r[:, 3] == 0)):
    s = r[m]; quiet = s[s[:, 2] == 0]; hit = s[s[:, 2] == 1]
    print(f"{label}: {len(s)} intervals, {len(hit)} with a touch; quiet residual p50/p99/p99.9/max {np.percentile(quiet[:,0],50):.2f}/{np.percentile(quiet[:,0],99):.2f}/{np.percentile(quiet[:,0],99.9):.1f}/{quiet[:,0].max():.0f}; touch residual p1/p5/p50 {np.percentile(hit[:,0],1):.1f}/{np.percentile(hit[:,0],5):.1f}/{np.percentile(hit[:,0],50):.0f}")
    for thr in (1, 2, 5, 10, 20, 50):
        det = s[:, 0] > thr
        print(f"   threshold {thr:>3} UU/s: touches found {int((det & (s[:,2]==1)).sum())} of {len(hit)}, false alarms {int((det & (s[:,2]==0)).sum())} of {len(quiet)}")
# what are the false alarms (residual above 10 UU/s, no truth touch)? closest car-ball distance in the interval
import glob
closest = {}
with open(states_path) as f:
    seen = set()
    for line in f:
        p = json.loads(line)['packet']; fn = p['match_info']['frame_num']
        if fn in seen or not p['balls']: continue
        seen.add(fn)
        b = p['balls'][0]['physics']['location']
        dmin = min((((pl['physics']['location']['x'] - b['x']) ** 2 + (pl['physics']['location']['y'] - b['y']) ** 2 + (pl['physics']['location']['z'] - b['z']) ** 2) ** 0.5 for pl in p['players']), default=9e9)
        closest[fn] = (dmin, b['z'], p['match_info']['match_phase'])
fa_rows = []
for l in open(ev_path):
    v = l.split()
    fa, fb, ta, tb, d = map(int, v[:5]); rv = float(v[5]); pa = [float(x) for x in v[7:10]]; pb = [float(x) for x in v[10:13]]
    ha, hb = bt.get(key(pa)), bt.get(key(pb))
    if not ha or not hb or len(ha) != 1 or len(hb) != 1 or hb[0] <= ha[0] or rv <= 10: continue
    if np.any((touches > ha[0]) & (touches <= hb[0])): continue
    dm = min(closest[t][0] for t in range(ha[0], hb[0] + 1) if t in closest)
    ph = {closest[t][2] for t in range(ha[0], hb[0] + 1) if t in closest}
    fa_rows.append((rv, dm, sorted(ph)))
print(f"false alarms above 10 UU/s: {len(fa_rows)}; closest car (centre to ball centre, UU) within the interval: {[ (round(r),round(d),p) for r,d,p in fa_rows][:25]}")
