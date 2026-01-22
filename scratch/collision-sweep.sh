#!/usr/bin/env bash
set -euo pipefail

# Sweep numNodes from START to END (inclusive) and record average collision probability.
# For each numNodes, run REPEATS times and average the reported probability.
# Usage:
#   bash scratch/collision-sweep.sh [START] [END] [OUTFILE] [REPEATS]
# Defaults: START=15 END=40 OUTFILE=collision_sweep.txt REPEATS=10

START=${1:-15}
END=${2:-40}
OUT=${3:-collision_sweep.txt}
REPEATS=${4:-10}

# Optional environment overrides
SEED=${SEED:-4}
SIMTIME=${SIMTIME:-29}
APP=${APP:-scratch/dsme-beacon-slot-selection-random-pick-backbone}

echo "# numNodes avg_probability repeats=$REPEATS" > "$OUT"

for N in $(seq "$START" "$END"); do
  echo "[sweep] numNodes=$N repeats=$REPEATS" >&2
  vals=""
  cnt=0
  for r in $(seq 1 "$REPEATS"); do
    # Derive a varying seed per repeat and numNodes to decorrelate runs
    RUN_SEED=$(( SEED + N*100 + r ))
    LOG=$(./ns3 run "$APP --numNodes=$N --seed=$RUN_SEED --simTime=$SIMTIME" 2>&1 || true)
    LINE=$(printf "%s\n" "$LOG" | grep -F "Collision summary (geometric):" | tail -n1 || true)
    if [[ -n "$LINE" ]]; then
      PROB=$(printf "%s\n" "$LINE" | sed -n 's/.*probability=\([0-9.eE+-]\+\).*/\1/p')
      if [[ -n "$PROB" ]]; then
        vals+="$PROB\n"
        cnt=$((cnt+1))
      fi
    fi
  done
  if [[ $cnt -gt 0 ]]; then
    AVG=$(printf "%b" "$vals" | awk 'BEGIN{s=0;n=0} NF{ s+=$1; n++ } END{ if(n>0) printf("%.10f", s/n); }')
  else
    AVG="NA"
  fi
  printf "%d %s\n" "$N" "$AVG" >> "$OUT"
done

echo "[sweep] done. Results in $OUT" >&2
