"""The reconstructed scoreboard (`scoreboard` in the frame records) against the server's clock and phase.

Regulation: seconds_remaining vs the truth's game_time_remaining (clamped at 0; the server runs on to -1.01 after
expiry and holds there). Overtime: overtime_seconds vs the time since the overtime kickoff touch. Phase: the clock_state
vs the server's match_phase (1 countdown, 2 kickoff, 3 running, 4 and 5 goal pause, 6 pregame, 7 ended).

usage: python scripts/rlbot_clock_check.py <states.jsonl> <converted.jsonl>
"""
import json, sys, collections
import numpy as np
exec(open('scripts/rlbot_events_check.py').read().split("series = collections.defaultdict")[0])
# the kickoff touch that starts each running stretch: first tick of a phase 3 run
phase = {fn: truth[fn]['phase'] for fn in truth}
ks = sorted(truth)
starts = [b for a, b in zip(ks[:-1], ks[1:]) if phase[a] != 3 and phase[b] == 3]
ot_starts = [s for s in starts if truth[s]['overtime']]
err_reg = []; err_ot = []; conf = collections.Counter(); n = 0
for r in recs:
    sb = r.get('scoreboard')
    st = r['timeline_tick'] - floor_at(r['frame'])
    if sb is None or st not in truth: continue
    t = truth[st]; n += 1
    if t['overtime']:
        if sb['overtime_seconds'] is not None and phase[st] == 3:
            s0 = max([s for s in ot_starts if s <= st], default=None)
            if s0 is not None: err_ot.append(sb['overtime_seconds'] - (st - s0) / 120)
    else:
        if sb['seconds_remaining'] is not None:
            err_reg.append(sb['seconds_remaining'] - max(0.0, t['remaining']))
    exp = {1: 'countdown', 2: 'kickoff', 3: 'running', 4: 'goal_pause', 5: 'goal_pause', 6: 'pregame', 7: 'ended'}[t['phase']]
    if t['phase'] == 3 and not t['overtime'] and t['remaining'] <= 0: exp = 'expired'
    conf[(exp, sb['clock_state'])] += 1
def q(a): a = np.abs(np.array(a)); return f"p50/p90/p99/max {np.percentile(a,50):.3f}/{np.percentile(a,90):.3f}/{np.percentile(a,99):.3f}/{a.max():.2f} s (n {len(a)}); signed mean {np.mean(a if False else err_reg if a is err_reg else a):.3f}"
if err_reg:
    a = np.array(err_reg); print(f"regulation clock error |sb - truth|: p50/p90/p99/max {np.percentile(abs(a),50):.3f}/{np.percentile(abs(a),90):.3f}/{np.percentile(abs(a),99):.3f}/{abs(a).max():.2f} s over {len(a)} frames; signed mean {a.mean():+.3f} s; within 0.05 s: {np.mean(abs(a)<=0.05):.1%}, within 0.1 s: {np.mean(abs(a)<=0.1):.1%}")
if err_ot:
    a = np.array(err_ot); print(f"overtime clock error: p50/p90/p99/max {np.percentile(abs(a),50):.3f}/{np.percentile(abs(a),90):.3f}/{np.percentile(abs(a),99):.3f}/{abs(a).max():.2f} s over {len(a)} frames; signed mean {a.mean():+.3f} s")
tot = collections.Counter()
for (e, g), v in conf.items(): tot[e] += v
agree = sum(v for (e, g), v in conf.items() if e == g)
print(f"phase agreement (server phase -> clock_state): {agree} of {n} frames ({agree/n:.1%})")
for (e, g), v in sorted(conf.items(), key=lambda kv: -kv[1])[:14]:
    print(f"   server {e:<11} -> {g:<11} {v:>6} {'' if e == g else '  <-- differs'}")
