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
rows outside, against 1.2 expected). Rows whose simulated position error is below 0.01 UU (the replay's own
position resolution) in the development median (the one-step ball position at p50 and p90, the aligned masked
ball at p50) carry no information and are not graded. A test split whose statistic
falls outside a band is reported as a finding; with about 70 comparisons about 3 are expected outside by chance
alone, so the count is judged against that, not against zero.

The ordering simulated < linear < hold of the masked rows is graded only where it holds in both development
splits at that horizon, object and quantile (it does not for the default masked ball at p50, where a linear
extrapolation is as good as the simulation), and the test split must show the same order.

The row statistics are strongly correlated (they come from the same replays), so the number of rows outside is
judged against an empirical null, not the binomial. The null is built from disjoint partitions of the development
replays: the replays are split at random (stratified by game size) into A and B, bands are built on A with the same
code `bands` uses, and the rows outside their bands are counted on pseudo-splits of B (20 replays per game size,
with replacement, like a 60-replay test split). Drawing the pseudo-splits from the replays that built the bands
would leave out the noise of the development estimate and give a null that is too tight. Two A sizes are used
(half of the development replays, and a quarter), so the trend is visible: the real bands are built on all
development replays, which is more than either A, so their development estimate is less noisy; if the count rises
as A shrinks the null is conservative, an upper bound for the real bands (the printed counts show which way). Mean, p95, p99 and max are
printed with every `check` and stored in the bands file. A test split is a finding when its count exceeds the p95/p99.
(The bootstrap of the null's bands is vectorised and uses fewer draws than `bands`; the construction is the same.)

Reports are `evaluate_corpus` JSON files (the build with per-replay masked position quantiles). A quantile that is
null in a report (an empty sample) counts as missing. Every `check` prints the number of replays behind each row
and a message for each row that could not be compared.

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
NULL_PARTITIONS = 60
NULL_PSEUDO_SPLITS = 5
NULL_BOOTSTRAP_DRAWS = 1000


def value_of(rows, row, q):
    """One replay's quantile of a row as a float, or None when the row or quantile is missing or null/NaN."""
    v = rows.get(row, {}).get(q)
    return None if v is None or v != v else float(v)


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
    vals = [value_of(rows, row, q) for s, rows in replays if size is None or s == size]
    vals = [v for v in vals if v is not None]
    return float(np.median(vals)) if vals else float("nan")


def replay_count(replays, row, q, size=None):
    """How many replays of the split (of the game size, if given) have a value for the row's quantile."""
    return sum(value_of(rows, row, q) is not None for s, rows in replays if size is None or s == size)


def bootstrap(dev, row, q, size=None, per_size=20, fast_draws=None):
    """Prediction band for the median over a new split of `per_size` replays per game size. `fast_draws`
    (the null's many band builds) draws that many resamples as matrices instead of `DRAWS` one by one."""
    rng = np.random.default_rng(SEED)
    sizes = [size] if size else ["1v1", "2v2", "3v3"]
    pools = {}
    for s in sizes:
        values = [value_of(rows, row, q) for sz, rows in dev if sz == s]
        pools[s] = np.array([v for v in values if v is not None], dtype=float)
    centre = float(np.median(np.concatenate([pools[s] for s in sizes if len(pools[s])])))
    if fast_draws:
        used = [pools[s] for s in sizes if len(pools[s])]
        new = np.concatenate([p[rng.integers(0, len(p), (fast_draws, per_size))] for p in used], axis=1)
        old = np.concatenate([p[rng.integers(0, len(p), (fast_draws, len(p)))] for p in used], axis=1)
        diffs = np.median(new, axis=1) - np.median(old, axis=1)
        return centre + float(np.percentile(diffs, 2.5)), centre + float(np.percentile(diffs, 97.5))
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


def degenerate(dev, row, q):
    """A position row whose development median is below 0.01 UU (the replay's own position resolution)."""
    return row.endswith("(UU)") and statistic(dev, row, q) < 0.01


OUTLIER_ROWS = (
    "car position, one-step (UU)",
    "masked car position h1 (default) (UU)",
    "masked car position h1 (aligned) (UU)",
)


def outlier_limits(dev):
    """Per game size, the largest per-replay p90 of each car row among the development replays."""
    limits = {}
    for row in OUTLIER_ROWS:
        for size, rows in dev:
            if value_of(rows, row, "p90") is not None:
                limits.setdefault(row, {})
                limits[row][size] = max(limits[row].get(size, 0.0), value_of(rows, row, "p90"))
    return limits


def outlier_replays(limits, reports):
    """Test replays whose own p90 of a car row exceeds the largest value among the development replays of the
    same game size (a replay that no development replay of its size resembles), with the path."""
    found = []
    seen = set()
    for variant, report in zip(("default", "aligned"), reports):
        for replay in report["replays"]:
            size = size_of(replay["path"])
            rows = rows_of(replay, variant)
            for row in rows:
                p90 = value_of(rows, row, "p90")
                # The one-step row is in both reports: list a (replay, row) once.
                if (row in limits and size in limits[row] and p90 is not None and p90 > limits[row][size]
                        and (replay["path"], row) not in seen):
                    seen.add((replay["path"], row))
                    found.append((replay["path"], row, round(p90, 2), round(limits[row][size], 2)))
    return found


def build_bands(dev, fast_draws=None):
    bands = {}
    for row in all_rows(dev):
        for q in QUANTILES:
            if degenerate(dev, row, q):
                continue
            bands[f"{row} | {q} | all"] = bootstrap(dev, row, q, fast_draws=fast_draws)
            if row.startswith(("car position, one-step", "masked car position h1")):
                for s in ("1v1", "2v2", "3v3"):
                    bands[f"{row} | {q} | {s}"] = bootstrap(dev, row, q, s, fast_draws=fast_draws)
    return bands


def pooled_order(report, horizon):
    """(sim, lin, hold) pooled quantiles of a report for a horizon, by object and quantile."""
    out = {}
    mp = report["masked_position_uu_by_horizon_frames"][horizon]
    for obj in ("car", "ball"):
        for q in ("p50", "p90"):
            # A null quantile (an empty sample) is NaN, which is in no order.
            out[(obj, q)] = tuple(
                float("nan") if mp[obj][b][q] is None else mp[obj][b][q] for b in ("simulated", "linear", "hold")
            )
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


def count_outside(bands, replays):
    """Rows of `replays` outside their bands: (outside, compared)."""
    outside = total = 0
    for key, (lo, hi) in bands.items():
        row, q, scope = key.rsplit(" | ", 2)
        value = statistic(replays, row, q, None if scope == "all" else scope)
        if value != value:
            continue
        total += 1
        outside += not lo <= value <= hi
    return outside, total


def pseudo_split(replays, rng, per_size=20):
    """A split drawn from `replays`: `per_size` replays per game size, with replacement."""
    pools = {s: [r for r in replays if r[0] == s] for s in ("1v1", "2v2", "3v3")}
    drawn = []
    for pool in pools.values():
        if pool:
            drawn.extend(pool[i] for i in rng.integers(0, len(pool), size=per_size))
    return drawn


def partition(dev, rng, a_per_size):
    """A random stratified split of the development replays into A (`a_per_size` per game size) and B (the rest)."""
    a, b = [], []
    for size in ("1v1", "2v2", "3v3"):
        pool = [r for r in dev if r[0] == size]
        order = rng.permutation(len(pool))
        a.extend(pool[i] for i in order[:a_per_size])
        b.extend(pool[i] for i in order[a_per_size:])
    return a, b


def empirical_null(dev, a_per_size, partitions=NULL_PARTITIONS, pseudo_splits=NULL_PSEUDO_SPLITS):
    """The count of rows outside their bands for a new split of the same population, with the noise of the
    development estimate: bands built on A (a_per_size replays per game size), counted on pseudo-splits of the
    disjoint B, over `partitions` random partitions of the development replays."""
    rng = np.random.default_rng(SEED + 1)
    counts = []
    direct = []
    rows = 0
    for _ in range(partitions):
        a, b = partition(dev, rng, a_per_size)
        bands = build_bands(a, fast_draws=NULL_BOOTSTRAP_DRAWS)
        rows = len(bands)
        counts.extend(count_outside(bands, pseudo_split(b, rng))[0] for _ in range(pseudo_splits))
        if len(b) == 60:
            direct.append(count_outside(bands, b)[0])  # B itself is a 60-replay split disjoint from A
    return {
        "mean_b_itself": float(np.mean(direct)) if direct else None,
        "p95_b_itself": float(np.percentile(direct, 95)) if direct else None,
        "p99_b_itself": float(np.percentile(direct, 99)) if direct else None,
        "max_b_itself": int(max(direct)) if direct else None,
        "a_per_size": a_per_size,
        "b_per_size": len(dev) // 3 - a_per_size,
        "partitions": partitions,
        "pseudo_splits": partitions * pseudo_splits,
        "development_replays": len(dev),
        "rows": rows,
        "mean": float(np.mean(counts)),
        "p95": float(np.percentile(counts, 95)),
        "p99": float(np.percentile(counts, 99)),
        "max": int(max(counts)),
    }


def nulls(dev):
    """The null at two A sizes: half and a quarter of the development replays per game size."""
    per_size = len(dev) // 3
    return [empirical_null(dev, per_size // 2), empirical_null(dev, per_size // 4)]


def print_null(null):
    """`null`: a list of nulls at decreasing A size (the first is the one the verdict uses)."""
    for n in null:
        extra = (
            f"; B itself (no resampling) mean {n['mean_b_itself']:.1f}, p95 {n['p95_b_itself']:.0f}, "
            f"p99 {n['p99_b_itself']:.0f}, max {n['max_b_itself']}"
            if n.get("mean_b_itself") is not None
            else ""
        )
        print(
            f"  empirical null for the count outside ({n['pseudo_splits']} pseudo-splits of B from {n['partitions']} "
            f"partitions of {n['development_replays']} development replays; bands on A = {n['a_per_size']} per game size, "
            f"{n['rows']} rows; B = {n['b_per_size']} per size, resampled to 20): mean {n['mean']:.1f}, "
            f"p95 {n['p95']:.0f}, p99 {n['p99']:.0f}, max {n['max']}{extra}"
        )
    if len(null) > 1:
        bias = (
            "the count rises as A shrinks, so the real bands (built on all development replays, more than either A) "
            "give fewer outside than the first null: the first null is conservative"
            if null[1]["mean"] > null[0]["mean"]
            else "the count did not rise as A shrank, so the first null is not clearly conservative or optimistic"
        )
        print(f"  ({bias})")


def report_check(label, bands, test, rules, test_reports, null=None, verbose=True):
    outside = 0
    total = 0
    skipped = []
    print(f"== {label}")
    for key, (lo, hi) in bands.items():
        row, q, scope = key.rsplit(" | ", 2)
        size = None if scope == "all" else scope
        value = statistic(test, row, q, size)
        n = replay_count(test, row, q, size)
        if value != value:
            skipped.append(key)
            print(f"  SKIPPED {row} {q} {scope}: no replay of this split has a value ({n} replays)")
            continue
        total += 1
        flag = "ok" if lo <= value <= hi else ("LOW" if value < lo else "HIGH")
        if flag != "ok":
            outside += 1
            print(f"  {flag:4s} {row} {q} {scope}: {value:.4g} outside [{lo:.4g}, {hi:.4g}] ({n} replays)")
        elif verbose:
            print(f"  ok   {row} {q} {scope}: {value:.4g} in [{lo:.4g}, {hi:.4g}] ({n} replays)")
    print(f"  {total} comparisons ({len(skipped)} skipped), {outside} outside their band; the split has {len(test)} replays")
    if null is not None:
        print_null(null)
        first = null[0]
        if first.get("p95_b_itself") is not None:
            # The primary verdict: B itself is a fresh 60-replay split disjoint from the bands, like the test split.
            verdict = "within" if outside <= first["p95_b_itself"] else (
                "above the p95 of" if outside <= first["p99_b_itself"] else "above the p99 of")
            print(f"  the count outside, {outside}, is {verdict} the null of B itself (p95 {first['p95_b_itself']:.0f}, p99 {first['p99_b_itself']:.0f})")
        wide = "within" if outside <= first["p95"] else ("above the p95 of" if outside <= first["p99"] else "above the p99 of")
        print(f"  secondary: {wide} the pseudo-split null (resampling B doubles the variance of a split's statistics, so it is the wider one)")
    else:
        print(f"  (the binomial expectation {0.05 * total:.1f} understates the spread: rows are correlated)")
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
        # The null from disjoint partitions of all the development replays.
        null = nulls(dev)
        json.dump({"bands": bands, "ordering": rules, "dev_replays": len(dev), "outlier_limits": outlier_limits(dev),
                   "null_outside": null},
                  open(out, "w"), indent=1)
        print(f"{len(bands)} bands from {len(dev)} development replays, {len(rules)} ordering rules -> {out}")
        print_null(null)
    elif mode == "check":
        spec = json.load(open(sys.argv[2]))
        null_spec = spec.get("null_outside")
        if not (isinstance(null_spec, list) and null_spec and null_spec[0].get("p99_b_itself") is not None):
            print("note: the bands file predates the null of disjoint partitions (no 'B itself' null stored); "
                  "rebuild it with `bands` to get the empirical null. Checking the bands only.")
            null_spec = None
        test, d, a = load_split(sys.argv[3], sys.argv[4])
        report_check(f"{sys.argv[3]} against {sys.argv[2]}", {k: tuple(v) for k, v in spec["bands"].items()},
                     test, [tuple(r) for r in spec["ordering"]], (d, a), null_spec)
        outliers = outlier_replays(spec["outlier_limits"], (d, a))
        print(f"  replays above every development replay of their game size (car p90 rows): {len(outliers)}")
        for o in outliers:
            print("   ", o)
    elif mode == "selfcheck":
        t_def, t_al, v_def, v_al = sys.argv[2:6]
        train, td, ta = load_split(t_def, t_al)
        val, vd, va = load_split(v_def, v_al)
        # The same null as `check`'s, from partitions of the two development splits together.
        pool_null = nulls(train + val)
        for name, dev, other, devrep, otherrep in (("train bands, validation checked", train, val, (td, ta), (vd, va)),
                                                    ("validation bands, train checked", val, train, (vd, va), (td, ta))):
            bands = build_bands(dev)
            rules = ordering_rules([devrep])
            report_check(name, bands, other, rules, otherrep, pool_null, verbose=False)
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
