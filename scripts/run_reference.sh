#!/bin/bash
# Reference evaluation of the current build on the train and validation splits (never test), as in
# TEST_PROTOCOL.md section 4: evaluate_corpus default and --aligned-targets per split.
# The binary is built first (release) and then copied, so that rebuilding while it runs does not hit a locked
# exe and the numbers always belong to the current source. Any failure (build or evaluation) makes the script
# exit non-zero and end progress.txt with FAILED instead of ALL-DONE.
# usage: scripts/run_reference.sh [output_dir]   (default target/ref-new; progress in progress.txt)
cd "$(dirname "$0")/.." || exit 1
out=${1:-target/ref-new}
mkdir -p "$out/bin"
: > "$out/progress.txt"
if ! cargo build --release --bin evaluate_corpus; then
  echo "BUILD FAILED" >> "$out/progress.txt"
  exit 1
fi
cp target/release/evaluate_corpus.exe "$out/bin/" || { echo "COPY FAILED" >> "$out/progress.txt"; exit 1; }
git rev-parse HEAD > "$out/commit.txt"
if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
  echo "note: the working tree has uncommitted changes (commit.txt names HEAD only)" >> "$out/progress.txt"
fi
failed=0
for split in train validation; do
  for variant in default aligned; do
    flag=""
    [ "$variant" = aligned ] && flag="--aligned-targets"
    start=$(date +%s)
    "$out/bin/evaluate_corpus.exe" "replays/$split" "$out/$split-$variant.json" $flag > "$out/$split-$variant.txt" 2>&1
    status=$?
    [ "$status" -ne 0 ] && failed=1
    echo "$split $variant done in $(( $(date +%s) - start ))s exit $status" >> "$out/progress.txt"
  done
done
if [ "$failed" -ne 0 ]; then
  echo "FAILED" >> "$out/progress.txt"
  exit 1
fi
echo ALL-DONE >> "$out/progress.txt"
