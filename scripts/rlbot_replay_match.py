"""Match a replay's fresh car packets to an RLBot recording of the same match (exact 0.01 UU positions).

The RLBot recording (states.jsonl, server truth) and the replay (from dump_replay_packets) are matched per
player name: a packet is on the server's tick frame_num if its position equals the recorded position at
that tick exactly. Reports the fraction matched, the cadence of the replay's fresh packets and, for the
matched ones, the lag of the packet against the replay's own clock (time*120 minus frame_num, detrended).

usage: python scripts/rlbot_replay_match.py <states.jsonl> <replay_packets.jsonl>
"""
import json, sys, collections
import numpy as np

states_path, replay_path = sys.argv[1], sys.argv[2]

def key(p):
    return (round(p[0] * 100), round(p[1] * 100), round(p[2] * 100))

truth = collections.defaultdict(lambda: collections.defaultdict(list))  # name -> pos key -> [frame_num]
frames_seen = []
with open(states_path) as f:
    for line in f:
        r = json.loads(line)
        p = r['packet']
        fn = p['match_info']['frame_num']
        frames_seen.append(fn)
        for pl in p['players']:
            ph = pl['physics']['location']
            truth[pl['name']][key((ph['x'], ph['y'], ph['z']))].append(fn)

rows = [json.loads(l) for l in open(replay_path, encoding='utf-8')]
names = {p['key']: p['name'] for p in rows[0]['players']}
rows = rows[1:]

n_fresh = 0
n_match = 0
per_name = collections.Counter()
matched = []  # (time, frame_num, name, frame)
gaps_frames, gaps_ms = [], []
last_fresh = {}
for row in rows:
    f = row['f']
    for c in row['cars']:
        pos = c['pos']
        if not pos or pos[1] != f:
            continue
        name = names.get(c['key'])
        if name is None:
            continue
        n_fresh += 1
        if (name, c['actor']) in last_fresh:
            lf, lt = last_fresh[(name, c['actor'])]
            gaps_frames.append(f - lf)
            gaps_ms.append((row['t'] - lt) * 1000)
        last_fresh[(name, c['actor'])] = (f, row['t'])
        hits = truth[name].get(key(pos[0]))
        if hits and len(hits) == 1:
            n_match += 1
            per_name[name] += 1
            matched.append((row['t'], hits[0], name, f))
print(f"replay frames {len(rows)} ({rows[-1]['t']:.1f} s); fresh car packets {n_fresh}; matched uniquely to a server tick: {n_match} ({100*n_match/max(n_fresh,1):.1f}%)")
print('gap between a car\'s fresh packets: frames p10/p50/p90 %s, ms %s' % (np.percentile(gaps_frames, [10, 50, 90]), np.round(np.percentile(gaps_ms, [10, 50, 90]), 1)))
print('matched per player:', dict(per_name))
if matched:
    m = np.array([(t, fn) for t, fn, _, _ in matched])
    # replay clock in ticks minus server frame: offset = t*120 - frame_num, detrended by a running median
    off = m[:, 0] * 120 - m[:, 1]
    order = np.argsort(m[:, 0])
    off = off[order]
    w = 50
    med = np.array([np.median(off[max(0, i - w):i + w + 1]) for i in range(len(off))])
    d = off - med
    print('offset (replay clock ticks - server frame_num) range %.1f .. %.1f; drift over the recording %.1f ticks' % (off.min(), off.max(), med[-1] - med[0]))
    print('detrended lag spread (ticks) p1/p10/p50/p90/p99:', np.round(np.percentile(d, [1, 10, 50, 90, 99]), 2))
