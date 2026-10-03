"""How late does a replay show a control change, against the server's true inputs (RLBot recording)?

For each change of a car's replicated throttle, steer or handbrake seen in the replay, the tick of the
same change in the recorded inputs (`last_input`; the packet of tick n+1 holds the input applied in tick
n to n+1) is found, and the delay is the replay frame's server tick (time of the frame minus the lag-0
offset, from the exactly matched car packets) minus that tick. Reported in ticks, with the frame gap.

usage: python scripts/rlbot_control_delay.py <states.jsonl> <replay_packets.jsonl> [bots|humans|all]
"""
import json, sys, collections
import numpy as np

states_path, replay_path = sys.argv[1], sys.argv[2]
who = sys.argv[3] if len(sys.argv) > 3 else 'all'
key = lambda p: (round(p[0] * 100), round(p[1] * 100), round(p[2] * 100))
q = lambda x: round(x * 127) / 127.0
truth_pos = collections.defaultdict(lambda: collections.defaultdict(list))
inputs = collections.defaultdict(dict)   # name -> frame_num -> (throttle, steer, handbrake)
is_bot = {}
with open(states_path) as f:
    for line in f:
        p = json.loads(line)['packet']
        fn = p['match_info']['frame_num']
        for pl in p['players']:
            ph = pl['physics']['location']
            truth_pos[pl['name']][key((ph['x'], ph['y'], ph['z']))].append(fn)
            li = pl['last_input']
            inputs[pl['name']][fn] = (q(li['throttle']), q(li['steer']), bool(li['handbrake']))
            is_bot[pl['name']] = pl['is_bot']
rows = [json.loads(l) for l in open(replay_path, encoding='utf-8')]
names = {p['key']: p['name'] for p in rows[0]['players']}
rows = rows[1:]

# timeline tick (as the converter: round((t - t0) * 120)) and offsets from matched car packets
t0 = rows[0]['t']
tick = lambda t: round((t - t0) * 120)
matched = []  # (frame index, offset)
for row in rows:
    f = row['f']
    for c in row['cars']:
        pos = c['pos']
        if not pos or pos[1] != f:
            continue
        name = names.get(c['key'])
        hits = truth_pos[name].get(key(pos[0])) if name else None
        if hits and len(hits) == 1:
            matched.append((f, tick(row['t']) - hits[0]))
matched.sort()
mf = np.array([m[0] for m in matched]); mo = np.array([m[1] for m in matched])
w = 150
def floor_at(f):
    i = int(np.searchsorted(mf, f)); lo, hi = max(0, i - w), min(len(mf), i + w + 1)
    return mo[lo:hi].min()

delays = collections.defaultdict(list)
delay_minus_gap = collections.defaultdict(list)
frame_gap = collections.defaultdict(list)
last = {}
prev_t = {}
for row in rows:
    f = row['f']
    S = tick(row['t']) - floor_at(f)
    gap_prev = tick(row['t']) - tick(rows[f - 1]['t']) if f > 0 else 0
    for c in row['cars']:
        name = names.get(c['key'])
        if name is None or (who == 'bots' and not is_bot[name]) or (who == 'humans' and is_bot[name]):
            continue
        for ci, field in enumerate(('throttle', 'steer', 'handbrake')):
            v = c[field]
            if not v or v[1] != f:
                continue
            new = q(v[0]) if ci < 2 else bool(v[0])
            old = last.get((name, ci))
            last[(name, ci)] = new
            if old is None or old == new:
                continue
            # the true change old -> new, latest one at or before tick S within 80 ticks
            seq = inputs[name]
            best = None
            for T in range(int(S), int(S) - 80, -1):
                a, b = seq.get(T - 1), seq.get(T)
                if a is None or b is None:
                    continue
                if a[ci] == old and b[ci] == new:
                    best = T
                    break
            if best is not None:
                delays[field].append(S - best)
                delay_minus_gap[field].append((S - best) - gap_prev)
for field, d in delays.items():
    d = np.array(d)
    print(f"{who} {field}: {len(d)} matched changes; delay (ticks) p10/p50/p90 {np.percentile(d,[10,50,90])}, mean {d.mean():.1f}; fraction within 0-10 ticks {np.mean((d>=0)&(d<=10)):.2f}, within 10-20: {np.mean((d>10)&(d<=20)):.2f}")
for field, d in delay_minus_gap.items():
    d = np.array(d)
    print(f"{who} {field}: delay minus the gap of the frame interval (should be >= the replication latency L if a change is first seen in the next frame): p5/p25/p50/p75/p95 {np.percentile(d,[5,25,50,75,95])}")
gaps = np.diff([tick(r['t']) for r in rows])
print('frame gap (ticks) p10/p50/p90', np.percentile(gaps, [10, 50, 90]))
