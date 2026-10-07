"""Check that every column of a replicar file is listed in docs/v2-file-format.md (story 9.2).

A column matches when its name, with the player index written `<i>` and the pad index `<k>`, is in the document
after its brace lists are expanded (`car_<i>_position_{x,y,z}` lists three columns; `(and _frame)` or
`(and their _frame)` after a list adds each name with `_frame`). Record fields are not checked.

usage: python scripts/check_file_format_doc.py <file.parquet>...   (needs pyarrow; write the file with every group)
"""

import itertools
import re
import sys
from pathlib import Path

import pyarrow.parquet as pq

DOC = Path(__file__).resolve().parent.parent / "docs" / "v2-file-format.md"


def expand(pattern):
    parts = re.split(r"(\{[^}]*\})", pattern)
    choices = [p[1:-1].split(",") if p.startswith("{") else [p] for p in parts]
    return {"".join(c) for c in itertools.product(*choices)}


def documented():
    names = set()
    for line in DOC.read_text(encoding="utf-8").splitlines():
        if not line.startswith("|"):
            continue
        cell = line.split("|")[1]
        frames = "_frame" in cell
        for pattern in re.findall(r"`([^`]+)`", cell):
            for name in expand(pattern):
                names.add(name)
                if frames:
                    names.add(name + "_frame")
    return names


def main():
    names = documented()
    missing = set()
    for path in sys.argv[1:]:
        for field in pq.read_schema(path):
            name = re.sub(r"^(car|player|network_car|network_player)_\d+_", r"\1_<i>_", field.name)
            name = re.sub(r"^pad_\d+_", "pad_<k>_", name)
            if name not in names:
                missing.add(name)
    for name in sorted(missing):
        print("NOT DOCUMENTED", name)
    print(f"{len(names)} documented names; {len(missing)} columns missing")
    sys.exit(1 if missing else 0)


if __name__ == "__main__":
    main()
