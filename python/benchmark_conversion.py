"""Measure end-to-end JSONL/Parquet conversion time and peak process memory.

Build ``convert_replay`` in release mode before running. Requires psutil.
Generated exports stay under target/ and are removed unless --keep-files is set.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import re
import statistics
import subprocess
import time
from pathlib import Path

import psutil


ROOT = Path(__file__).resolve().parents[1]
EXECUTABLE = ROOT / "target" / "release" / (
    "convert_replay.exe" if platform.system() == "Windows" else "convert_replay"
)


def digest(path: Path) -> str:
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(block)
    return checksum.hexdigest()


def run_one(replay: Path, output: Path, warmup: bool) -> dict:
    flags = subprocess.CREATE_NO_WINDOW if platform.system() == "Windows" else 0
    started = time.perf_counter()
    child = subprocess.Popen(
        [str(EXECUTABLE), str(replay), str(output)],
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        creationflags=flags,
    )
    process = psutil.Process(child.pid)
    peak = 0
    peak_kind = "peak_wset" if platform.system() == "Windows" else "rss"
    while child.poll() is None:
        try:
            memory = process.memory_info()
            peak = max(peak, getattr(memory, peak_kind))
        except psutil.NoSuchProcess:
            break
        time.sleep(0.02)
    stdout, stderr = child.communicate()
    elapsed = time.perf_counter() - started
    if child.returncode != 0 or not output.is_file():
        raise RuntimeError(f"conversion failed ({child.returncode}): {replay}\n{stdout}\n{stderr}")
    # The converter prints one `<n> rows -> <table>` line per record table before the `<n> frames -> <file>` line.
    match = re.search(r"^(\d+) frames -> ", stdout, flags=re.MULTILINE)
    if match is None:
        raise RuntimeError(f"no '<n> frames -> <file>' line in the converter output of {replay}:\n{stdout}")
    frames = int(match.group(1))
    result = {
        "replay": str(replay.relative_to(ROOT)),
        "format": output.suffix[1:],
        "warmup": warmup,
        "frames": frames,
        "wall_seconds": elapsed,
        "peak_memory_bytes": peak,
        "peak_memory_kind": peak_kind,
        "output_bytes": output.stat().st_size,
    }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replays", nargs="+", type=Path)
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--report", type=Path, default=Path("target/conversion-benchmark.json"))
    parser.add_argument("--keep-files", action="store_true")
    args = parser.parse_args()
    if args.warmups < 0 or args.repeats < 1:
        parser.error("warmups must be nonnegative and repeats must be positive")
    if not EXECUTABLE.is_file():
        parser.error(f"release converter is missing: {EXECUTABLE}")
    replays = [path.resolve() for path in args.replays]
    for replay in replays:
        if not replay.is_file() or not replay.is_relative_to(ROOT / "replays"):
            parser.error(f"expected a local replay under replays/: {replay}")
        if "test" in replay.relative_to(ROOT / "replays").parts:
            parser.error("replays/test is sealed")
    report_path = (ROOT / args.report).resolve()
    output_dir = ROOT / "target" / "conversion-benchmark"
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    manifest = [
        {"replay": str(path.relative_to(ROOT)), "sha256": digest(path), "bytes": path.stat().st_size}
        for path in replays
    ]
    for replay_index, replay in enumerate(replays):
        rel = replay.relative_to(ROOT / "replays")
        label = "-".join((rel.parts[0], rel.parts[1], replay.stem))
        for repetition in range(args.warmups + args.repeats):
            formats = ("jsonl", "parquet") if (replay_index + repetition) % 2 == 0 else ("parquet", "jsonl")
            for kind in formats:
                output = output_dir / f"{label}-{repetition}.{kind}"
                try:
                    row = run_one(replay, output, repetition < args.warmups)
                    row["repetition"] = repetition
                    rows.append(row)
                    print(
                        f"{row['replay']} {kind} rep={repetition} "
                        f"{row['wall_seconds']:.2f}s peak={row['peak_memory_bytes'] / 1e6:.1f} MB",
                        flush=True,
                    )
                finally:
                    if not args.keep_files:
                        output.unlink(missing_ok=True)
    summary = []
    for replay in replays:
        name = str(replay.relative_to(ROOT))
        for kind in ("jsonl", "parquet"):
            subset = [row for row in rows if row["replay"] == name and row["format"] == kind and not row["warmup"]]
            summary.append({
                "replay": name,
                "format": kind,
                "frames": subset[0]["frames"],
                "median_wall_seconds": statistics.median(row["wall_seconds"] for row in subset),
                "median_peak_memory_bytes": statistics.median(row["peak_memory_bytes"] for row in subset),
                "median_output_bytes": statistics.median(row["output_bytes"] for row in subset),
            })
    report = {
        "protocol": "release executable, one warmup per format and replay by default, alternating format order, 20 ms polling of process peak working set on Windows",
        "platform": platform.platform(),
        "psutil_version": psutil.__version__,
        "executable": str(EXECUTABLE.relative_to(ROOT)),
        "executable_sha256": digest(EXECUTABLE),
        "warmups": args.warmups,
        "repeats": args.repeats,
        "manifest": manifest,
        "runs": rows,
        "summary": summary,
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"report -> {report_path}")


if __name__ == "__main__":
    main()
