"""Compare a Rust-written Parquet export with the schema-v1 JSONL reference."""

from __future__ import annotations

import argparse
from itertools import zip_longest

import numpy as np
import pyarrow.parquet as pq

from replay_columnar import iter_columnar_frames, load_columnar_numpy, read_columnar_header
from replay_to_rocketsim import iter_frames, load_numpy, read_header


def verify(jsonl: str, parquet: str) -> int:
    file = pq.ParquetFile(parquet)
    if any(file.metadata.row_group(i).num_rows > 512 for i in range(file.metadata.num_row_groups)):
        raise AssertionError("Parquet row group exceeded the 512-frame batch bound")
    expected_header = read_header(jsonl)
    actual_header = read_columnar_header(parquet)
    if actual_header != expected_header:
        raise AssertionError("Parquet header differs from JSONL header")
    expected = load_numpy(jsonl)
    actual = load_columnar_numpy(parquet)
    if actual.keys() != expected.keys():
        raise AssertionError(f"array keys differ: {actual.keys() ^ expected.keys()}")
    for name, value in expected.items():
        if isinstance(value, np.ndarray):
            np.testing.assert_equal(actual[name], value, err_msg=name)
        elif actual[name] != value:
            raise AssertionError(f"{name} differs")
    count = 0
    for count, (left, right) in enumerate(
        zip_longest(iter_frames(jsonl), iter_columnar_frames(parquet)), start=1
    ):
        if left != right:
            raise AssertionError(f"rich frame {count - 1} differs")
    return count


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("jsonl")
    parser.add_argument("parquet")
    args = parser.parse_args()
    print(f"Verified {verify(args.jsonl, args.parquet)} frames, header, and all NumPy arrays")


if __name__ == "__main__":
    main()
