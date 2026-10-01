"""Report on `rlbot_span_fit` output: car state at the contact and post-hit ball velocity, by method and group.

usage: python scripts/span_fit_report.py <out.jsonl> [<out2.jsonl> ...] [--all] [--methods A,B,...]

Every error is against the server truth (RLBot recording). Methods:
  L, H    linear / cubic Hermite interpolation of car packets A and B (restart at T-1 for the ball)
  C       the converter's own per-tick trajectory (--inferred runs only)
  G0      unfitted scratch simulation from packet A, observed controls with the midpoint rule (shift 0), no air controls
  A0      the same with the boundary-value air controls (the converter's air schedule) when the span has a solution
  <fam>:<rule>   a fit: the timing shift of the observed control switches (family G or A) picked by a rule that uses only
          replay observations (car packet B, ball packet b); `AUTO` uses A when it has a solution, else G
  ORACLE  the shift with the smallest TRUE position error at T-1 (not a method: the most any shift choice can give)
  TA0     observed controls plus the TRUE air controls (bound for rotation modelling)
  SAT     simulation from packet A with all the TRUE controls (bound for control modelling)
  TS      restart at T-1 from the TRUE car state with the observed controls; TT with the true controls (ceiling)
"""
import json
import sys
import numpy as np


def car_cost(c):
    return c["end_pos"] + c["end_vel"] / 50.0


def sel_car(cands):
    return min(cands, key=lambda c: (car_cost(c), abs(c["shift"])))


def sel_ball(cands):
    ok = [c for c in cands if c["ball_res"] == c["ball_res"]]
    return min(ok, key=lambda c: (c["ball_res"], abs(c["shift"]))) if ok else sel_car(cands)


def sel_joint(w):
    def f(cands):
        ok = [c for c in cands if c["ball_res"] == c["ball_res"]]
        if not ok:
            return sel_car(cands)
        return min(ok, key=lambda c: (car_cost(c) + c["ball_res"] / w, abs(c["shift"])))
    return f


def sel_car_then_ball(tol):
    def f(cands):
        best = min(car_cost(c) for c in cands)
        near = [c for c in cands if car_cost(c) <= best + tol and c["ball_res"] == c["ball_res"]]
        return min(near, key=lambda c: (c["ball_res"], abs(c["shift"]))) if near else sel_car(cands)
    return f


def sel_oracle(cands):
    return min(cands, key=lambda c: (c["pos_t1"], abs(c["shift"])))


def sel_zero(cands):
    return next(c for c in cands if c["shift"] == 0)


RULES = {
    "car": sel_car,
    "ball": sel_ball,
    "joint100": sel_joint(100.0),
    "joint30": sel_joint(30.0),
    "car+ball5": sel_car_then_ball(5.0),
    "car+ball2": sel_car_then_ball(2.0),
}


def load(paths):
    out = []
    for p in paths:
        with open(p) as f:
            for line in f:
                out.append(json.loads(line))
    return out


def get(rec, name):
    """The candidate-like dict of a method for a record, or None."""
    fams = rec["fams"]
    if name == "G0":
        return sel_zero(fams["G"])
    if name == "A0":
        return sel_zero(fams["A"]) if "A" in fams else None
    if name == "AUTO0":
        return sel_zero(fams["A"] if "A" in fams else fams["G"])
    if name == "AR0":
        return sel_zero(fams["AR"]) if "AR" in fams else None
    if name == "COMB0":
        return sel_zero(fams["AR"] if "AR" in fams else fams["G"])
    if name.startswith("COMB:") or name.startswith("COMBA:"):
        # AR when it has a solution (COMBA: also within a rotation error tolerance), else the ground family
        rule = name.split(":")[1]
        tol = 1e9
        if len(name.split(":")) > 2:
            tol = float(name.split(":")[2])
        ar = rec.get("ar") or {}
        use_ar = "AR" in fams and ar and ar.get("rot_err_deg", 1e9) <= tol
        return RULES[rule](fams["AR"] if use_ar else fams["G"])
    if name == "TA0":
        return sel_zero(fams["TA"])
    if name == "SAT":
        return fams["T"][0]
    if name == "ORACLE":
        return sel_oracle(fams["G"])
    if ":" in name:
        fam, rule = name.split(":")
        if fam == "AUTO":
            fam = "A" if "A" in fams else "G"
        if fam not in fams:
            return None
        return RULES[rule](fams[fam])
    m = rec["methods"].get(name)
    if m is None:
        return None
    if name in ("TS", "TT"):
        return {"pos_t1": 0.0, "ball": m.get("ball"), "interior": [], "shift": 0}
    return {"pos_t1": m.get("pos_t1"), "ball": m.get("ball"), "interior": m.get("interior") or [], "shift": 0}


def pct(v, q):
    v = [x for x in v if x is not None and x == x]
    return float(np.percentile(v, q)) if v else float("nan")


def fb(v, t):
    v = [x for x in v if x is not None and x == x]
    return 100.0 * sum(1 for x in v if x < t) / len(v) if v else float("nan")


def lead_bin(l):
    return "5-8" if l <= 8 else ("9-12" if l <= 12 else ">12")


def summarize(recs, names, title):
    print(f"\n== {title} (n {len(recs)}) ==")
    print(f"{'method':<14} {'pos@T-1 p50/p90/p99':>21} {'<1/<3':>7} | {'rot p50/p90 deg':>15} | {'ball H0 <25/50/100 %':>21} {'p50/p90':>10} | {'H12 <25/50/100 %':>17} {'p50/p90':>10} | {'interior p50/p90':>16}")
    for nm in names:
        pos1, rot, b0, b12, inter = [], [], [], [], []
        for r in recs:
            c = get(r, nm)
            if c is None or c.get("pos_t1") is None:
                continue
            pos1.append(c["pos_t1"])
            if c.get("rot_t1") is not None:
                rot.append(c["rot_t1"])
            if c.get("ball"):
                b0.append(c["ball"][0])
                b12.append(c["ball"][3])
            inter.extend(x for x in c.get("interior", []) if x is not None)
        if not pos1:
            continue
        print(
            f"{nm:<16} n{len(pos1):<4}{pct(pos1,50):6.2f}/{pct(pos1,90):5.2f}/{pct(pos1,99):5.1f} {fb(pos1,1):3.0f}/{fb(pos1,3):3.0f} | "
            f"{pct(rot,50):6.2f}/{pct(rot,90):6.2f} | "
            f"{fb(b0,25):6.0f}/{fb(b0,50):4.0f}/{fb(b0,100):4.0f} {pct(b0,50):8.1f}/{pct(b0,90):5.0f} | "
            f"{fb(b12,25):5.0f}/{fb(b12,50):4.0f}/{fb(b12,100):4.0f} {pct(b12,50):8.1f}/{pct(b12,90):5.0f} | "
            f"{pct(inter,50):6.2f}/{pct(inter,90):6.2f}"
        )


def paired(recs, base, new, metric="pos_t1", label=None):
    d, e = [], []
    for r in recs:
        a, b = get(r, base), get(r, new)
        if a is None or b is None or a.get(metric) is None or b.get(metric) is None:
            continue
        d.append(b[metric] - a[metric])
        if metric == "ball":
            pass
    d = np.array(d)
    if not len(d):
        return
    print(
        f"  {new} vs {base} ({metric}, n {len(d)}): better by >1 {100*np.mean(d<-1):.0f}%, worse by >1 {100*np.mean(d>1):.0f}%, "
        f"median change {np.median(d):+.2f}, p10/p90 {np.percentile(d,10):+.1f}/{np.percentile(d,90):+.1f}"
    )


def paired_ball(recs, base, new, h=3, thresh=50.0):
    """Counts of samples reproduced (ball error below thresh at horizon index h) by base only / new only / both."""
    both = only_a = only_b = neither = 0
    for r in recs:
        a, b = get(r, base), get(r, new)
        if a is None or b is None or not a.get("ball") or not b.get("ball"):
            continue
        ra, rb = a["ball"][h] < thresh, b["ball"][h] < thresh
        both += ra and rb
        only_a += ra and not rb
        only_b += rb and not ra
        neither += (not ra) and (not rb)
    print(f"  ball H{[0,1,4,12][h]} < {thresh:g}: {base} only {only_a}, {new} only {only_b}, both {both}, neither {neither}")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = [a for a in sys.argv[1:] if a.startswith("--")]
    recs = load(args)
    names = ["L", "H", "C", "G0", "A0", "G:car", "G:ball", "G:joint100", "G:car+ball2", "A:car", "A:ball", "A:joint100", "A:car+ball2",
             "AUTO:car", "AUTO:joint100", "AUTO:car+ball2", "ORACLE", "TA0", "SAT", "TS", "TT"]
    for f in flags:
        if f.startswith("--methods="):
            names = f.split("=", 1)[1].split(",")
    clean = [r for r in recs if not r["jump_in_span"]]
    if "--all" in flags:
        clean = recs
    if "--no-dodge" in flags:
        clean = [r for r in clean if not r.get("dodge_odd_at_a")]
    if "--dodge" in flags:
        clean = [r for r in clean if r.get("dodge_odd_at_a")]
    if "--matched" in flags:
        need = [n for n in names if n not in ("C",) or any(r["methods"].get("C") for r in clean)]
        clean = [r for r in clean if all(get(r, n) is not None and get(r, n).get("pos_t1") is not None for n in need)]
    print(f"{len(recs)} samples, {len(clean)} used (no jump/dodge counter change in the span)")
    summarize(clean, names, "all")
    for grp, key in (("grounded (truth air_state OnGround)", lambda r: r["grounded"]), ("airborne", lambda r: not r["grounded"])):
        summarize([r for r in clean if key(r)], names, grp)
    for b in ("5-8", "9-12", ">12"):
        summarize([r for r in clean if lead_bin(r["lead"]) == b], names, f"lead {b}")
    for grp, key in (("bots", lambda r: r["bot"]), ("humans", lambda r: not r["bot"])):
        summarize([r for r in clean if key(r)], names, grp)
