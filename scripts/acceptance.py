"""Acceptance bands for the test-split assessment, from the spread of per-replay statistics.

A fixed tolerance (for example +10% of the validation value) is arbitrary and, on our own splits, as wide as the
normal difference between two 60-replay splits (train against validation differs by up to 9% at p90). Instead the
expected spread of a 60-replay split is taken from the development replays themselves: for each row (a metric and a
quantile) the statistic of a split is the median over its replays of that replay's own quantile; the band is the
2.5th to 97.5th percentile of that statistic over splits resampled from the development replays (20 replays per game
size, with replacement, stratified; 5,000 draws, fixed seed so the band is reproducible). The band is a
prediction interval: the development median plus the 2.5th to 97.5th percentile of (the median of a new split
resampled from the development replays) minus (the median of a development-sized resample), so it includes both
the new split's sampling noise and the uncertainty of the development estimate (a plain bootstrap of the
development statistic would be too narrow by about the square root of two: `selfcheck` showed 7 and 4 of 24
rows outside, against 1.2 expected). Rows whose simulated error is zero in practically every replay (the
one-step ball position at p50 and p90) carry no information and are not graded. A test split whose statistic
falls outside a band is reported as a finding; with about 70 comparisons about 3 are expected outside by chance
alone, so the count is judged against that, not against zero.

The ordering simulated < linear < hold of the masked rows is graded only where it holds in both development
splits at that horizon, object and quantile (it does not for the default masked ball at p50, where a linear
extrapolation is as good as the simulation), and the test split must show the same order.

Reports are `evaluate_corpus` JSON files (the build with per-replay masked position quantiles).

usage:
  python scripts/acceptance.py bands <bands.json> <dev-default.json>[,<dev-aligned.json>] ...   # one group per
         development split, files of one split joined by commas: default first, aligned second
  python scripts/acceptance.py check <bands.json> <test-default.json> <test-aligned.json>
  python scripts/acceptance.py selfcheck <train-default.json> <train-aligned.json> <val-default.json> <val-aligned.json>
         (bands from train checked against validation, and the reverse: shows how often a split falls outside)
"""
import json
import re
import sys
import numpy as np

QUANTILES = ("p50", "p90", "p99")
DRAWS = 5000
SEED = 20261002  # reproducibility only


def size_of(path):
    m = re.search(r"(1v1|2v2|3v3)", path.replace("\\", "/"))
    return m.group(1) if m else "?"


def rows_of(replay, variant):
    """row name -> {quantile: value} for one replay report."""
    out = {}
    p = replay["position_uu"]
    out["car position, one-step (UU)"] = p["car"]["simulated"]
    out["ball position, one-step (UU)"] = p["ball"]["simulated"]
    k = replay["kinematics"]["car"]
    out["car velocity, one-step (UU/s)"] = k["linear_velocity_uu_per_second"]["simulated"]
    out["car rotation, one-step (deg)"] = k["rotation_degrees"]["simulated"]
    out["car angular velocity, one-step (rad/s)"] = k["angular_velocity_radians_per_second"]["simulated"]
    masked = replay.get("masked_position_uu_by_horizon_frames", {})
    for h in ("1", "2", "3", "4"):
        if h in masked:
            out[f"masked car position h{h} ({variant}) (UU)"] = masked[h]["car"]["simulated"]
            out[f"masked ball position h{h} ({variant}) (UU)"] = masked[h]["ball"]["simulated"]
    return out


def load_split(default_path, aligned_path):
    """list of (size, {row: {q: value}}) over the replays of one split (both variants merged per replay)."""
    d = json.load(open(default_path))
    a = json.load(open(aligned_path))
    by_path = {r["path"]: r for r in a["replays"]}
    replays = []
    for r in d["replays"]:
        rows = rows_of(r, "default")
        if r["path"] in by_path:
            rows.update({k: v for k, v in rows_of(by_path[r["path"]], "aligned").items() if k.startswith("masked")})
        replays.append((size_of(r["path"]), rows))
    return replays, d, a


def statistic(replays, row, q, size=None):
    vals = [rows[row][q] for s, rows in replays if (size is None or s == size) and row in rows and q in rows[row]]
    return float(np.median(vals)) if vals else float("nan")


def bootstrap(dev, row, q, size=None, per_size=20):
    """Prediction band for the median over a new split of `per_size` replays per game size."""
    rng = np.random.default_rng(SEED)
    sizes = [size] if size else ["1v1", "2v2", "3v3"]
    pools = {s: np.array([rows[row][q] for sz, rows in dev if sz == s and row in rows], dtype=float) for s in sizes}
    centre = float(np.median(np.concatenate([pools[s] for s in sizes if len(pools[s])])))
    diffs = []
    for _ in range(DRAWS):
        new = np.concatenate([rng.choice(pools[s], size=per_size, replace=True) for s in sizes if len(pools[s])])
        old = np.concatenate([rng.choice(pools[s], size=len(pools[s]), replace=True) for s in sizes if len(pools[s])])
        diffs.append(np.median(new) - np.median(old))
    return centre + float(np.percentile(diffs, 2.5)), centre + float(np.percentile(diffs, 97.5))


def all_rows(replays):
    names = []
    for _, rows in replays:
        for k in rows:
            if k not in names:
                names.append(k)
    return names


def degenerate(row, q):
    """The simulated ball position error is zero in practically every replay except in its tail."""
    return row.startswith("ball position, one-step") and q in ("p50", "p90")


def build_bands(dev):
    bands = {}
    for row in all_rows(dev):
        for q in QUANTILES:
            if degenerate(row, q):
                continue
            bands[f"{row} | {q} | all"] = bootstrap(dev, row, q)
            if row.startswith(("car position, one-step", "masked car position h1")):
                for s in ("1v1", "2v2", "3v3"):
                    bands[f"{row} | {q} | {s}"] = bootstrap(dev, row, q, s)
    return bands


def pooled_order(report, horizon):
    """(sim, lin, hold) pooled quantiles of a report for a horizon, by object and quantile."""
    out = {}
    mp = report["masked_position_uu_by_horizon_frames"][horizon]
    for obj in ("car", "ball"):
        for q in ("p50", "p90"):
            out[(obj, q)] = tuple(mp[obj][b][q] for b in ("simulated", "linear", "hold"))
    return out


def ordering_rules(dev_reports):
    """The (variant, horizon, object, quantile) where sim < linear < hold in every development report given."""
    rules = []
    for variant_index, variant in enumerate(("default", "aligned")):
        for h in ("1", "2", "3", "4"):
            tables = [pooled_order(rep[variant_index], h) for rep in dev_reports]
            for key in tables[0]:
                if all(t[key][0] < t[key][1] < t[key][2] for t in tables):
                    rules.append((variant, h, key[0], key[1]))
    return rules


def report_check(label, bands, test, rules, test_reports):
    outside = 0
    total = 0
    print(f"== {label}")
    for key, (lo, hi) in bands.items():
        row, q, scope = key.rsplit(" | ", 2)
        value = statistic(test, row, q, None if scope == "all" else scope)
        if value != value:
            continue
        total += 1
        flag = "ok" if lo <= value <= hi else ("LOW" if value < lo else "HIGH")
        if flag != "ok":
            outside += 1
            print(f"  {flag:4s} {row} {q} {scope}: {value:.4g} outside [{lo:.4g}, {hi:.4g}]")
    print(f"  {total} comparisons, {outside} outside their band (about {0.05 * total:.1f} expected by chance)")
    broken = []
    for variant, h, obj, q in rules:
        i = 0 if variant == "default" else 1
        sim, lin, hold = pooled_order(test_reports[i], h)[(obj, q)]
        if not sim < lin < hold:
            broken.append((variant, h, obj, q, round(sim, 2), round(lin, 2), round(hold, 2)))
    print(f"  ordering rules from development: {len(rules)}, broken in this split: {len(broken)}")
    for b in broken:
        print("   ", b)
    return outside, total


def main():
    mode = sys.argv[1]
    if mode == "bands":
        out = sys.argv[2]
        splits = [tuple(arg.split(",")) for arg in sys.argv[3:]]
        loaded = [load_split(d, a) for d, a in splits]
        dev = [r for replays, _, _ in loaded for r in replays]
        bands = build_bands(dev)
        rules = ordering_rules([(d, a) for _, d, a in loaded])
        json.dump({"bands": bands, "ordering": rules, "dev_replays": len(dev)}, open(out, "w"), indent=1)
        print(f"{len(bands)} bands from {len(dev)} development replays, {len(rules)} ordering rules -> {out}")
    elif mode == "check":
        spec = json.load(open(sys.argv[2]))
        test, d, a = load_split(sys.argv[3], sys.argv[4])
        report_check(f"{sys.argv[3]} against {sys.argv[2]}", {k: tuple(v) for k, v in spec["bands"].items()},
                     test, [tuple(r) for r in spec["ordering"]], (d, a))
    elif mode == "selfcheck":
        t_def, t_al, v_def, v_al = sys.argv[2:6]
        train, td, ta = load_split(t_def, t_al)
        val, vd, va = load_split(v_def, v_al)
        for name, dev, other, devrep, otherrep in (("train bands, validation checked", train, val, (td, ta), (vd, va)),
                                                    ("validation bands, train checked", val, train, (vd, va), (td, ta))):
            bands = build_bands(dev)
            rules = ordering_rules([devrep])
            report_check(name, bands, other, rules, otherrep)
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
