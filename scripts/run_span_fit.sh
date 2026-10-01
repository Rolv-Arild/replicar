#!/bin/bash
# Run the car-trajectory/contact fit experiment (`rlbot_span_fit`) on the two LAN recordings.
# usage: scripts/run_span_fit.sh <oracle|inferred> [client|host] [extra rlbot_span_fit args]
# The executable is this checkout's; the replays and collision meshes are read from the main checkout
# (the directory that holds ./collision_meshes and ./replays). Output: target/span/<game>_<which>_<mode>.jsonl
here="$(cd "$(dirname "$0")/.." && pwd)"
main="${SPAN_FIT_MAIN:-$(cd "$here" && cd "$(git rev-parse --git-common-dir)/.." && pwd)}"
mode="${1:-oracle}"
which="${2:-client}"
shift 2
extra=()
[ "$mode" = inferred ] && extra+=(--inferred)
mkdir -p "$here/target/span"
cd "$main" || exit 1
for g in game1 game2; do
  d=$(ls -d replays/*lan_remote_4bots_$g)
  "$here/target/release/rlbot_span_fit.exe" "$d/lan_remote_4bots_${g}_$which.replay" "$d/states.jsonl" \
    "$here/target/span/${g}_${which}_${mode}.jsonl" "${extra[@]}" "$@" 2>&1 | tail -4
done
