"""Replay clock fields next to the server's clock around chosen server ticks (host replay, exact alignment).

usage: python scripts/rlbot_clock_trace.py <states.jsonl> <converted.jsonl> <tick from> <tick to> [every n frames]
"""
import json, sys, collections
import numpy as np
states_path, conv_path, lo, hi = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
step = int(sys.argv[5]) if len(sys.argv) > 5 else 1
key = lambda p: (round(p[0] * 100), round(p[1] * 100), round(p[2] * 100))
truth = {}; tpos = collections.defaultdict(lambda: collections.defaultdict(list))
for line in open(states_path):
    p = json.loads(line)['packet']; fn = p['match_info']['frame_num']
    if fn in truth: continue
    m = p['match_info']
    bz = p['balls'][0]['physics']['location']['z'] if p['balls'] else None
    truth[fn] = (m['match_phase'], m['game_time_remaining'], m['is_overtime'], bz, [t['score'] for t in p['teams']])
    for pl in p['players']:
        l = pl['physics']['location']; tpos[pl['name']][key((l['x'], l['y'], l['z']))].append(fn)
recs = [json.loads(l) for l in open(conv_path, encoding='utf-8')]
keys = {}
for r in recs[1:60]:
    for p in r['observations']['players']: keys[p['key']] = p['name']
offs = []
for r in recs[1:]:
    for c in r['observations']['cars']:
        pos = c['body'].get('position'); nm = keys.get(c.get('player_key'))
        if pos and nm and pos['frame'] == r['frame']:
            h = tpos[nm].get(key(pos['value']))
            if h and len(h) == 1: offs.append((r['frame'], r['timeline_tick'] - h[0]))
mf = np.array([o[0] for o in offs]); mo = np.array([o[1] for o in offs])
def floor_at(f):
    i = int(np.searchsorted(mf, f)); a, b = max(0, i - 150), min(len(mf), i + 151); return int(mo[a:b].min())
n = 0
for r in recs[1:]:
    st = r['timeline_tick'] - floor_at(r['frame'])
    if lo <= st <= hi:
        n += 1
        if n % step: continue
        o = r['observations']; sr = o['seconds_remaining']; gs = o['game_state']; ov = o['overtime']
        t = truth.get(st)
        print(f"tick {st} frame {r['frame']}: replay clock {sr and sr['value']} (set at frame {sr and sr['frame']}) state {gs and gs['value']} overtime {ov and ov['value']} | truth phase {t and t[0]} remaining {t and round(t[1],2)} overtime {t and t[2]} ball z {t and t[3] and round(t[3])}")
