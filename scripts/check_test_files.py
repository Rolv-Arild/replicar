"""Check a folder of replicar files converted from a folder of replays (TEST_PROTOCOL.md, section 7): every replay
has a file; each file resimulates to its states (`replicar resimulate` checks the state checksum); per player and
counted statistic the stat events add up to `final_stats`; every goal report has a scorer; consecutive tick rows are
one tick apart within a segment.

usage: python scripts/check_test_files.py <files dir> <replays dir> [replicar binary]   (PYTHONPATH=python/v2/src)
"""

import collections
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

import replicar


def main() -> None:
    files, replays = Path(sys.argv[1]), Path(sys.argv[2])
    binary = str(Path(sys.argv[3] if len(sys.argv) > 3 else "target/release/replicar").resolve())
    counts = collections.Counter()
    problems = []
    for replay in sorted(replays.rglob("*.replay")):
        counts["replays"] += 1
        file = files / replay.relative_to(replays).with_suffix(".parquet")
        if not file.exists():
            problems.append((str(replay), "no file"))
            continue
        counts["files"] += 1
        with tempfile.TemporaryDirectory() as directory:
            again = Path(directory) / "re.parquet"
            done = subprocess.run([binary, "resimulate", str(file), "--replay", str(replay), "-o", str(again)],
                                  capture_output=True, text=True)
            if done.returncode == 0:
                counts["resimulated"] += 1
            else:
                problems.append((str(replay), "resimulate: " + done.stderr.strip()))
        f = replicar.read(file)
        counted = f.header.get("counted_stats", [])
        totals = collections.Counter()
        for e in f.records("stat_events").to_pylist():
            totals[(e["player"], e["kind"])] += 1
        mismatched = [(p["name"], k) for p in f.header["players"] for k in counted
                      if totals[(p["index"], k)] != p["final_stats"].get(k)]
        counts["stat mismatches"] += len(mismatched)
        if mismatched:
            problems.append((str(replay), f"stat events and final_stats differ: {mismatched[:3]}"))
        for e in f.records("events").to_pylist():
            if e["kind"] == "goal":
                counts["goals"] += 1
                counts["goals with a scorer"] += e["scorer"] is not None
                counts["goals with an assister"] += e["assister"] is not None
        a = f.arrays()
        segment, tick = a["segment"], a["sim_tick"].astype(np.int64)
        same = (segment[1:] == segment[:-1]) & (segment[1:] >= 0)
        gaps = np.diff(tick)[same]
        counts["rows"] += len(tick)
        counts["tick gaps not 0 or 1"] += int(np.sum((gaps != 1) & (gaps != 0)))
    for key, value in counts.items():
        print(f"{key:24} {value}")
    print(f"problems: {len(problems)}")
    for replay, problem in problems:
        print(f"  {os.path.relpath(replay)}: {problem}")


if __name__ == "__main__":
    main()
