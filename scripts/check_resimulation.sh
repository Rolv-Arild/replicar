#!/bin/sh
# Story 7.3 check (RESULTS.md, "v2: the resimulation group") on train and validation: convert with the resimulation group, resimulate in another process,
# compare the files.
set -e
rm -rf target/v2all target/v2all-out
mkdir -p target/v2all target/v2all-out
./target/release/write_file target/v2all replays/train replays/validation --resimulation > target/v2all.txt
args=""
for f in target/v2all/*.parquet; do
  b=$(basename "$f" .parquet)
  r=$(ls replays/train/*/"$b".replay replays/validation/*/"$b".replay 2>/dev/null | head -1)
  args="$args $f $r"
done
./target/release/resimulate_file target/v2all-out $args > target/v2all-out.txt
args=""
for f in target/v2all/*.parquet; do args="$args $f target/v2all-out/$(basename $f)"; done
python scripts/compare_files.py $args > target/v2all-compare.txt
tail -3 target/v2all-compare.txt
grep -c "^equal" target/v2all-compare.txt
