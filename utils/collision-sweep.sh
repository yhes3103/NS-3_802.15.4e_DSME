#!/usr/bin/env bash
set -euo pipefail

# Sweep numNodes from START to END (inclusive) and record collision probability
# Usage:
#   bash utils/collision-sweep.sh [START] [END] [OUTFILE]
# Defaults: START=15 END=35 OUTFILE=collision_sweep.txt

START=${1:-15}
END=${2:-35}
OUT=${3:-collision_sweep.txt}

# Optional environment overrides
SEED=${SEED:-4}
SIMTIME=${SIMTIME:-29}
APP=${APP:-scratch/dsme-beacon-slot-selection-random-pick-backbone}

echo "# numNodes probability" > "$OUT"

for N in $(seq "$START" "$END"); do
  echo "[sweep] numNodes=$N" >&2
  # Run and capture stdout
  LOG=$(./ns3 run "$APP --numNodes=$N --seed=$SEED --simTime=$SIMTIME" 2>&1 || true)
  # Extract the probability value from the summary line
  LINE=$(printf "%s\n" "$LOG" | grep -F "Collision summary (geometric):" | tail -n1 || true)
  if [[ -n "$LINE" ]]; then
    PROB=$(printf "%s\n" "$LINE" | sed -n 's/.*probability=\([0-9.eE+-]\+\).*/\1/p')
  else
    PROB="NA"
  fi
  printf "%d %s\n" "$N" "$PROB" >> "$OUT"
done

echo "[sweep] done. Results in $OUT" >&2

