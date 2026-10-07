"""Time `replicar convert <folder>` at several job counts, with the process's peak memory, and project the time
for a corpus (story: release planning).

usage: python scripts/time_corpus.py <replicar binary> <replay folder> <scratch dir> <jobs>... [-- <convert options>]   (needs psutil)
"""

import os
import shutil
import subprocess
import sys
import time

import psutil


def main():
    args = sys.argv[1:]
    extra = args[args.index("--") + 1 :] if "--" in args else []
    args = args[: args.index("--")] if "--" in args else args
    binary, folder, scratch, *jobs = args
    replays = [os.path.join(r, f) for r, _, fs in os.walk(folder) for f in fs if f.endswith(".replay")]
    size = sum(os.path.getsize(p) for p in replays)
    print(f"{len(replays)} replays, {size / 1e6:.1f} MB")
    for j in jobs:
        out = os.path.join(scratch, f"out-{j}")
        shutil.rmtree(out, ignore_errors=True)
        start = time.perf_counter()
        process = subprocess.Popen([binary, "convert", folder, "-o", out, "--jobs", j, *extra],
                                   stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        handle = psutil.Process(process.pid)
        peak = 0
        while process.poll() is None:
            try:
                peak = max(peak, handle.memory_info().rss)
            except psutil.Error:
                pass
            time.sleep(0.2)
        elapsed = time.perf_counter() - start
        print(f"jobs {j:>2}: {elapsed:7.1f} s, {elapsed / len(replays):.3f} s per replay, "
              f"{len(replays) / elapsed * 3600:7.0f} replays per hour, peak memory {peak / 1e9:.2f} GB, "
              f"exit {process.returncode}", flush=True)


if __name__ == "__main__":
    main()
