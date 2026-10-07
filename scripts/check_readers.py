"""Open replicar files in pyarrow, polars and DuckDB and check that they agree (docs/v2-plan.md, story 7.1).

usage: python scripts/check_readers.py <file.parquet>...   (needs pyarrow, polars and duckdb)
"""

import json
import sys

import duckdb
import polars as pl
import pyarrow.parquet as pq


def main():
    for path in sys.argv[1:]:
        table = pq.read_table(path)
        frame = pl.read_parquet(path)
        assert frame.shape == (table.num_rows, table.num_columns), (frame.shape, table.shape)
        with duckdb.connect() as con:
            rows, = con.execute(f"SELECT count(*) FROM read_parquet('{path}')").fetchone()
            assert rows == table.num_rows
            # Typed values, nulls, names and records read the same in all three.
            columns = ["car_0_position_x", "car_0_update_tick", "clock_phase", "future_segment_end"]
            duck = con.execute(f"SELECT {', '.join(columns)} FROM read_parquet('{path}')").fetchall()
            events = con.execute(
                f"SELECT count(*) FROM (SELECT unnest(events) AS e FROM read_parquet('{path}')) "
                "WHERE e.kind = 'goal'").fetchone()[0]
            metadata = con.execute(
                f"SELECT value FROM parquet_kv_metadata('{path}') WHERE key = 'replicar'").fetchone()[0]
        for i, name in enumerate(columns):
            expected = table.column(name).to_pylist()
            assert [r[i] for r in duck] == expected, name
            assert frame[name].to_list() == expected, name
        goals = sum(1 for row in table.column("events").to_pylist() for e in row if e["kind"] == "goal")
        assert events == goals == frame.explode("events")["events"].struct.field("kind").eq("goal").sum()
        header = json.loads(metadata)
        assert header == json.loads(pq.read_metadata(path).metadata[b"replicar"])
        print(f"ok  {path}: {table.num_rows} rows, {table.num_columns} columns, {goals} goal events, "
              f"{len(header['players'])} players")


if __name__ == "__main__":
    main()
