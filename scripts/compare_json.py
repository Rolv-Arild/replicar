"""Compare two JSON files value for value (story 9.1: the v2 evaluator's reports against v1's). Prints the count
of differing leaves and the first few paths; floats compare exactly.

usage: python scripts/compare_json.py <a.json> <b.json> [--ignore KEY]...
"""

import json
import sys


def walk(a, b, path, out, ignore):
    if isinstance(a, dict) and isinstance(b, dict):
        for key in sorted(set(a) | set(b)):
            if key in ignore:
                continue
            walk(a.get(key), b.get(key), f"{path}.{key}", out, ignore)
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            out.append((path, f"length {len(a)}", f"length {len(b)}"))
        for i, (x, y) in enumerate(zip(a, b)):
            walk(x, y, f"{path}[{i}]", out, ignore)
    elif a != b:
        out.append((path, a, b))


def main():
    args = sys.argv[1:]
    ignore = {args[i + 1] for i, a in enumerate(args) if a == "--ignore"}
    paths = [a for i, a in enumerate(args) if a != "--ignore" and (i == 0 or args[i - 1] != "--ignore")]
    with open(paths[0]) as f:
        a = json.load(f)
    with open(paths[1]) as f:
        b = json.load(f)
    out = []
    walk(a, b, "", out, ignore)
    print(f"{len(out)} differing values")
    for path, x, y in out[:10]:
        print(f"  {path}: {x!r} vs {y!r}")
    sys.exit(1 if out else 0)


if __name__ == "__main__":
    main()
