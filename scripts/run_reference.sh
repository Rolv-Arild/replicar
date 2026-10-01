#!/bin/bash
# Reference evaluation of the current build on the train and validation splits (never test), as in
# TEST_PROTOCOL.md section 4: evaluate_corpus default and --aligned-targets per split.
# The binaries are copied first so that rebuilding while it runs does not hit a locked exe.
# usage: scripts/run_reference.sh [output_dir]   (default target/ref-new; progress in progress.txt)
cd "$(dirname "$0")/.." || exit 1
out=${1:-target/ref-new}
mkdir -p "$out/bin"
cp target/release/evaluate_corpus.exe "$out/bin/"
: > "$out/progress.txt"
git rev-parse HEAD > "$out/commit.txt"
for split in train validation; do
  for variant in default aligned; do
    flag=""
    [ "$variant" = aligned ] && flag="--aligned-targets"
    start=$(date +%s)
    "$out/bin/evaluate_corpus.exe" "replays/$split" "$out/$split-$variant.json" $flag > "$out/$split-$variant.txt" 2>&1
    status=$?
    echo "$split $variant done in $(( $(date +%s) - start ))s exit $status" >> "$out/progress.txt"
  done
done
echo ALL-DONE >> "$out/progress.txt"
