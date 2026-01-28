#!/usr/bin/env bash
set -euo pipefail

# Sweep numNodes from START to END (inclusive) and record the averages of
# three collision metrics emitted by the app summary:
#   - [Method-1] Ever-collided receivers (rate)
#   - [Method-2a] Collision probability (receiver-slot)
#   - [Method-2b] Collision severity avg (k-1 per receiver-slot)
# For each numNodes, run REPEATS times and average each metric.
# Usage:
#   bash scratch/collision-sweep.sh [START] [END] [OUTFILE] [REPEATS] [STEP]
# Defaults: START=15 END=40 OUTFILE=collision_sweep.txt REPEATS=10 STEP=1

START=${1:-15}
END=${2:-40}
OUT=${3:-collision_sweep.txt}
REPEATS=${4:-10}
STEP=${5:-1}

# Optional environment overrides
SEED=${SEED:-4}
SIMTIME=${SIMTIME:-29}
APP=${APP:-scratch/dsme-beacon-slot-selection-random-pick-backbone}

echo "# numNodes avg_method1_rate avg_method2a_prob avg_method2b_severity repeats=$REPEATS step=$STEP" > "$OUT"

for N in $(seq "$START" "$STEP" "$END"); do
  echo "[sweep] numNodes=$N repeats=$REPEATS" >&2
  vals1=""  # Method-1 rates
  vals2a="" # Method-2a probabilities
  vals2b="" # Method-2b severities
  cnt1=0
  cnt2a=0
  cnt2b=0
  for r in $(seq 1 "$REPEATS"); do
    # Derive a varying seed per repeat and numNodes to decorrelate runs
    RUN_SEED=$(( SEED + N*100 + r ))
    LOG=$(./ns3 run "$APP --numNodes=$N --seed=$RUN_SEED --simTime=$SIMTIME" 2>&1 || true)
    # Extract Method-1 rate
    M1=$(printf "%s\n" "$LOG" \
      | grep -F "[Method-1] Ever-collided receivers:" \
      | tail -n1 \
      | sed -n 's/.*rate=\([0-9.eE+-]\+\).*/\1/p')
    if [[ -n "$M1" ]]; then
      vals1+="$M1\n"; cnt1=$((cnt1+1));
    fi
    # Extract Method-2a probability
    M2A=$(printf "%s\n" "$LOG" \
      | grep -F "[Method-2a] Collision probability (receiver-slot):" \
      | tail -n1 \
      | sed -n 's/.*receiver-slot): \([0-9.eE+-]\+\).*/\1/p')
    if [[ -n "$M2A" ]]; then
      vals2a+="$M2A\n"; cnt2a=$((cnt2a+1));
    fi
    # Extract Method-2b severity
    M2B=$(printf "%s\n" "$LOG" \
      | grep -F "[Method-2b] Collision severity avg (k-1 per receiver-slot):" \
      | tail -n1 \
      | sed -n 's/.*receiver-slot): \([0-9.eE+-]\+\).*/\1/p')
    if [[ -n "$M2B" ]]; then
      vals2b+="$M2B\n"; cnt2b=$((cnt2b+1));
    fi
  done
  if [[ $cnt1 -gt 0 ]]; then
    AVG1=$(printf "%b" "$vals1" | awk 'BEGIN{s=0;n=0} NF{ s+=$1; n++ } END{ if(n>0) printf("%.10f", s/n); }')
  else
    AVG1="NA"
  fi
  if [[ $cnt2a -gt 0 ]]; then
    AVG2A=$(printf "%b" "$vals2a" | awk 'BEGIN{s=0;n=0} NF{ s+=$1; n++ } END{ if(n>0) printf("%.10f", s/n); }')
  else
    AVG2A="NA"
  fi
  if [[ $cnt2b -gt 0 ]]; then
    AVG2B=$(printf "%b" "$vals2b" | awk 'BEGIN{s=0;n=0} NF{ s+=$1; n++ } END{ if(n>0) printf("%.10f", s/n); }')
  else
    AVG2B="NA"
  fi
  printf "%d %s %s %s\n" "$N" "$AVG1" "$AVG2A" "$AVG2B" >> "$OUT"
done

echo "[sweep] done. Results in $OUT" >&2
