"""Signed pitch input by tick after each forward/back dodge in RLBot recordings (states.jsonl), split
by bots and humans; the pitch cancel of RocketSim is the input against the flip (signed > 0).

usage: python scripts/rlbot_flip_cancel.py <states.jsonl> [more states.jsonl ...]
"""
import json, sys, collections
import numpy as np
def load(path):
    rows=[]  # per packet: frame, list of players (name,is_bot,has_dodged,dodge_dir,pitch,air_state,dodge_elapsed)
    with open(path) as f:
        for line in f:
            r=json.loads(line); p=r['packet']
            rows.append((p['match_info']['frame_num'],p['match_info']['match_phase'],[(i,pl['is_bot'],pl['has_dodged'],pl['dodge_dir']['x'],pl['dodge_dir']['y'],pl['last_input']['pitch'],pl['last_input']['yaw'],pl['last_input']['roll'],pl['air_state'],pl['dodge_elapsed'],pl['last_input']['jump']) for i,pl in enumerate(p['players'])]))
    return rows
def analyse(path,label,humans_only=None):
    rows=load(path)
    # dedupe frames
    seq=[];last=None
    for r in rows:
        if r[0]==last: continue
        seq.append(r);last=r[0]
    curves={}  # (player) -> list of arrays
    for n in range(1,len(seq)-45):
        if seq[n][0]!=seq[n-1][0]+1: continue
        for i,pl in enumerate(seq[n][2]):
            prev=seq[n-1][2][i]
            if pl[2] and not prev[2]:   # dodge starts at packet n
                dx=pl[3]
                if abs(dx)<0.2: continue     # side dodge: no pitch cancel definition
                sign=1.0 if dx>0 else -1.0
                # need contiguous 40 frames
                if any(seq[n+t][0]!=seq[n][0]+t for t in range(0,41)): continue
                curve=[seq[n+t][2][i][5]*sign for t in range(0,41)]
                curves.setdefault((pl[1],),[]).append((curve,dx))
    print(label)
    for key,cs in curves.items():
        arr=np.array([c for c,_ in cs])
        print(' ','bots' if key[0] else 'human','n flips (fwd/back)',len(arr))
        for t in (0,2,4,6,8,10,12,16,20,24,30,40):
            col=arr[:,t]
            print(f'    tick {t:>2}: cancel input (signed pitch) mean {col.mean():+.2f}  p10/p50/p90 {np.percentile(col,10):+.2f}/{np.percentile(col,50):+.2f}/{np.percentile(col,90):+.2f}   frac>0.5: {(col>0.5).mean():.2f}')
        # per-flip time when signed pitch first > 0.5
        first=[ (np.argmax(c>0.5) if (c>0.5).any() else 99) for c in arr]
        print('    first tick with cancel>0.5: p10/p50/p90', np.percentile(first,[10,50,90]), 'never:',sum(1 for f in first if f==99))
        vals=np.unique(np.round(arr.flatten(),2)); print('    distinct pitch values',len(vals), vals[:12])
import sys
for path in sys.argv[1:]:
    analyse(path, path)
