"""Reproducible train-replay size and Python-read benchmark for columnar exports."""

from __future__ import annotations

import argparse
import gzip
import json
import shutil
import statistics
import time
from itertools import zip_longest
from pathlib import Path

import numpy as np

from replay_columnar import (
    iter_columnar_frames,
    load_columnar_numpy,
    write_columnar,
)
from replay_to_rocketsim import iter_frames, load_numpy, read_header


def timed(function, repeats: int) -> tuple[float, object]:
    times = []
    result = None
    for _ in range(repeats):
        start = time.perf_counter()
        result = function()
        times.append(time.perf_counter() - start)
    return statistics.median(times), result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("jsonl", type=Path)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be positive")
    outputs = {
        "arrow": args.jsonl.with_suffix(".arrow"),
        "parquet": args.jsonl.with_suffix(".parquet"),
    }
    compressed = Path(str(args.jsonl) + ".gz")
    report = {
        "source": str(args.jsonl),
        "source_sha256": read_header(args.jsonl).get("source_sha256"),
        "repeats": args.repeats,
        "formats": {"jsonl": {"bytes": args.jsonl.stat().st_size}},
    }
    for kind, path in outputs.items():
        start = time.perf_counter()
        count = write_columnar(args.jsonl, path)
        report["formats"][kind] = {
            "bytes": path.stat().st_size,
            "frames": count,
            "write_seconds": time.perf_counter() - start,
        }

    start = time.perf_counter()
    with args.jsonl.open("rb") as source, gzip.open(compressed, "wb", compresslevel=6) as target:
        shutil.copyfileobj(source, target)
    report["formats"]["gzip_jsonl"] = {
        "bytes": compressed.stat().st_size,
        "write_seconds": time.perf_counter() - start,
    }

    baseline_time, baseline = timed(lambda: load_numpy(args.jsonl), args.repeats)
    report["formats"]["jsonl"]["numpy_read_seconds_median"] = baseline_time
    gzip_time, gzip_arrays = timed(lambda: load_numpy(compressed), args.repeats)
    report["formats"]["gzip_jsonl"]["numpy_read_seconds_median"] = gzip_time
    for key, expected in baseline.items():
        if isinstance(expected, np.ndarray):
            np.testing.assert_equal(gzip_arrays[key], expected, err_msg=f"gzip_jsonl: {key}")
        elif gzip_arrays[key] != expected:
            raise AssertionError(f"gzip_jsonl: {key} differs")
    report["formats"]["gzip_jsonl"]["dense_arrays_match"] = True
    rich_sources = {
        "jsonl": args.jsonl,
        "gzip_jsonl": compressed,
    }
    for kind, path in rich_sources.items():
        duration, count = timed(lambda: sum(1 for _ in iter_frames(path)), 1)
        report["formats"][kind]["rich_read_seconds_single"] = duration
        report["formats"][kind]["frames"] = count
    for kind, path in outputs.items():
        duration, arrays = timed(lambda: load_columnar_numpy(path), args.repeats)
        report["formats"][kind]["numpy_read_seconds_median"] = duration
        if arrays.keys() != baseline.keys():
            raise AssertionError(f"{kind}: dense array keys differ")
        for key, expected in baseline.items():
            if isinstance(expected, np.ndarray):
                np.testing.assert_equal(arrays[key], expected, err_msg=f"{kind}: {key}")
            elif arrays[key] != expected:
                raise AssertionError(f"{kind}: {key} differs")
        sentinel = object()
        for index, (rich, recovered) in enumerate(zip_longest(
            iter_frames(args.jsonl), iter_columnar_frames(path), fillvalue=sentinel
        )):
            if rich != recovered:
                raise AssertionError(f"{kind}: rich frame {index} differs")
        report["formats"][kind]["full_round_trip_matches"] = True
        report["formats"][kind]["dense_arrays_match"] = True
        duration, count = timed(lambda: sum(1 for _ in iter_columnar_frames(path)), 1)
        report["formats"][kind]["rich_read_seconds_single"] = duration
        if count != report["formats"][kind]["frames"]:
            raise AssertionError(f"{kind}: rich frame count differs")
    if args.report:
        args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
