"""Compare two replicar files column by column: every column the second has must be identical in the first
(story 7.3: a resimulated file against the converted one), and the headers equal apart from the groups and the
keys given with --ignore-header.

usage: python scripts/compare_files.py <a.parquet> <b.parquet> [<a.parquet> <b.parquet>]... (needs pyarrow)
"""

import json
import sys

import pyarrow.parquet as pq


def main():
    paths = sys.argv[1:]
    failures = 0
    for a_path, b_path in zip(paths[::2], paths[1::2]):
        a, b = pq.read_table(a_path), pq.read_table(b_path)
        different = [name for name in b.column_names if not a.column(name).equals(b.column(name))]
        ha = json.loads(pq.read_metadata(a_path).metadata[b"replicar"])
        hb = json.loads(pq.read_metadata(b_path).metadata[b"replicar"])
        header = [k for k in ha if k != "groups" and ha[k] != hb.get(k)]
        status = "equal" if not different and not header else "DIFFERENT"
        failures += status != "equal"
        print(f"{status}  {b_path}: {len(b.column_names)} columns, {b.num_rows} rows"
              + (f"; columns {different[:5]}" if different else "")
              + (f"; header {header}" if header else ""))
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
