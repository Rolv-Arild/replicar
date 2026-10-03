#!/bin/bash
# Interior-frame error of the converter against server truth with and without the air boundary-value
# solve, on the remote-client games (host replays thinned to every third frame, client replays to every
# second). usage: scripts/run_interior_eval.sh
cd "$(dirname "$0")/.." || exit 1
B=./target/release/rlbot_reconstruction.exe
for g in game1 game2; do
  d=$(ls -d replays/*lan_remote_4bots_$g)
  for w in host client; do
    if [ "$w" = host ]; then thin="--thin 3 --zero-lag"; else thin="--thin 2"; fi
    for v in base bvp; do
      if [ "$v" = base ]; then export NO_AIR_BVP=1 NO_FIT_NEXT=1; else unset NO_AIR_BVP NO_FIT_NEXT; fi
      echo "== $g $w $v"
      $B $d/lan_remote_4bots_${g}_$w.replay $d/states.jsonl $thin 2>&1 | awk '/^all fits/{f=1} /^no dodge first/{f=0} f' | grep -E "^frames without a fresh|^air, no fresh packet  |^flip, no fresh packet  |^ground, no fresh packet  |^jump, no fresh packet  |^all  " | cut -c1-140
    done
  done
done
