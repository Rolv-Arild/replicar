"""Compact summary of evaluate_corpus reports (reference runs): per split and variant, the pre-correction position
errors, the masked-prediction errors against the hold and linear baselines, and the one-step kinematic residuals of
cars and the ball, overall and by game size.

usage: python scripts/summarize_reference.py <report.json> [<report2.json> ...]
"""
import json, sys
def q(x): return f"{x['p50']:.2f}/{x['p90']:.1f}/{x['p99']:.0f}"
for path in sys.argv[1:]:
    d = json.load(open(path))
    print(f"== {path}  ({d['split_directory']}, rocketsim {d['rocketsim_revision'][:7]}, {len(d['replays'])} replays, {len(d['failures'])} failures)")
    for obj in ('car', 'ball'):
        a = d['all'][obj]
        print(f"  {obj} pre-correction position error (one-step, at fresh packets) UU p50/p90/p99: simulated {q(a['simulated'])} | hold {q(a['hold'])} | linear {q(a['linear'])}  (n {a['simulated']['count']})")
    k = d['one_step_kinematics_all']
    for obj in ('car', 'ball'):
        v = k[obj]
        print(f"  {obj} one-step residual: velocity UU/s {q(v['linear_velocity_uu_per_second']['simulated'])} (hold {q(v['linear_velocity_uu_per_second']['hold'])}); rotation deg {q(v['rotation_degrees']['simulated'])}; angular velocity rad/s {q(v['angular_velocity_radians_per_second']['simulated'])}")
    mp = d['masked_position_uu_by_horizon_frames']
    def label(h, obj):
        # The horizon counts frames from the mask window's start; the stale packet a prediction starts
        # from can be older (cars are not refreshed every frame), so the label carries its age.
        age = mp[h][obj].get('start_packet_age_seconds')
        unit = 'frame' if h == '1' else 'frames'
        return f"{h} {unit}" + (f" (start-packet age p50 {age['p50']:.3f} s)" if age and age['p50'] is not None else '')
    for obj in ('car', 'ball'):
        print(f"  masked {obj} position UU p50/p90/p99 by horizon: " + '; '.join(f"{label(h, obj)}: sim {q(mp[h][obj]['simulated'])} hold {q(mp[h][obj]['hold'])} lin {q(mp[h][obj]['linear'])}" for h in sorted(mp)))
    buckets = d.get('masked_position_uu_by_start_packet_age_bucket')
    if buckets:
        for obj in ('car', 'ball'):
            print(f"  masked {obj} position UU p50/p90/p99 by start-packet age (all horizons): " + '; '.join(f"{b} s (n {v[obj]['simulated']['count']}): sim {q(v[obj]['simulated'])} hold {q(v[obj]['hold'])}" for b, v in buckets.items() if v[obj]['simulated']['count']))
    for size, v in d['by_game_size'].items():
        print(f"  {size}: car simulated {q(v['car']['simulated'])} hold {q(v['car']['hold'])} (n {v['car']['simulated']['count']}); ball simulated {q(v['ball']['simulated'])}")
