"""How well does the replay's replicated boost amount track the server's true boost?

For every fresh car packet matched to a server tick, the replay's last boost amount (u8 / 255 * 100, with
the frame it was last updated) is compared with the true boost at that tick, by staleness (frames since
the amount was updated) and by whether the car was boosting or picking up boost around that tick.

usage: python scripts/rlbot_boost_check.py <states.jsonl> <replay_packets.jsonl>
"""
import json, sys, collections
import numpy as np

states_path, replay_path = sys.argv[1], sys.argv[2]
key = lambda p: (round(p[0] * 100), round(p[1] * 100), round(p[2] * 100))
truth = collections.defaultdict(lambda: collections.defaultdict(list))
boost = collections.defaultdict(dict)   # name -> frame_num -> boost (0..100)
with open(states_path) as f:
    for line in f:
        p = json.loads(line)['packet']
        fn = p['match_info']['frame_num']
        for pl in p['players']:
            ph = pl['physics']['location']
            truth[pl['name']][key((ph['x'], ph['y'], ph['z']))].append(fn)
            boost[pl['name']][fn] = pl['boost']
rows = [json.loads(l) for l in open(replay_path, encoding='utf-8')]
names = {p['key']: p['name'] for p in rows[0]['players']}
rows = rows[1:]
errs = collections.defaultdict(list)
for row in rows:
    f = row['f']
    for c in row['cars']:
        pos = c['pos']
        if not pos or pos[1] != f:
            continue
        name = names.get(c['key'])
        hits = truth[name].get(key(pos[0])) if name else None
        if not hits or len(hits) != 1:
            continue
        ba = c.get('boost_amount')
        if not ba:
            continue
        fn = hits[0]
        true_b = boost[name].get(fn)
        if true_b is None:
            continue
        obs = ba[0] / 255.0 * 100.0
        stale = f - ba[1]
        # boosting around this tick: boost changed within +-6 ticks
        around = [boost[name].get(fn + d) for d in range(-6, 7)]
        around = [a for a in around if a is not None]
        rng = max(around) - min(around) if around else 0
        kind = 'pickup' if (around and around[-1] - around[0] > 5) else ('boosting' if rng > 0.5 else 'idle')
        errs[('all',)].append(obs - true_b)
        errs[('stale', min(stale, 6))].append(obs - true_b)
        errs[(kind,)].append(obs - true_b)
print(f"{len(errs[('all',)])} matched car packets with a boost amount")
for k in sorted(errs, key=str):
    e = np.array(errs[k])
    print(f"  {str(k):<22} n {len(e):>6}  error (replay - true, boost units) mean {e.mean():+.2f}  |e| p50/p90/p99 {np.round(np.percentile(np.abs(e),[50,90,99]),2)}")
