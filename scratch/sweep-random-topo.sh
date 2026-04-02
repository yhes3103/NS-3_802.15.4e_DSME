#!/usr/bin/env bash
set -euo pipefail

# Sweep joiner counts for dsme-beacon-slot-selection-random-topo and average metrics.
# 參照 scratch/collision-sweep.sh 之格式：
#   使用方式：
#     bash scratch/sweep-random-topo.sh [START] [END] [OUTFILE] [REPEATS] [STEP]
#   預設：START=5 END=50 OUTFILE=random_topo_sweep.txt REPEATS=20 STEP=5
# 可用環境變數覆寫：
#   SEED(4) SIMTIME(15) APP(scratch/dsme-beacon-slot-selection-random-topo)
#   RXSENS(-95) PLEXP(3) REFDIST(1.0) REFLOSS(40.05)
#   MINEB(1) BASE_OFFSET(2.0) BASE_SLOPE(0.20) RETRY(0.25) TIMEOUT(6.0)

START=${1:-5}
END=${2:-50}
OUT=${3:-random_topo_sweep.txt}
REPEATS=${4:-20}
STEP=${5:-5}

SEED=${SEED:-4}
SIMTIME=${SIMTIME:-15}
APP=${APP:-scratch/dsme-beacon-slot-selection-random-topo}

RXSENS=${RXSENS:--95}
PLEXP=${PLEXP:-3}
REFDIST=${REFDIST:-1.0}
REFLOSS=${REFLOSS:-40.05}

MINEB=${MINEB:-1}
BASE_OFFSET=${BASE_OFFSET:-2.0}
BASE_SLOPE=${BASE_SLOPE:-0.20}
RETRY=${RETRY:-0.25}
TIMEOUT=${TIMEOUT:-6.0}

echo "# joiners   avg_method1_rate   avg_method2a_prob   avg_method2b_severity   avg_not_joined_rate   repeats=$REPEATS step=$STEP" > "$OUT"

for J in $(seq "$START" "$STEP" "$END"); do
  echo "[sweep] joiners=$J repeats=$REPEATS" >&2
  vals1=""  # Method-1 rates
  vals2a="" # Method-2a probabilities
  vals2b="" # Method-2b severities
  cnt1=0; cnt2a=0; cnt2b=0
  valsNJ="" # Not-joined rate
  cntNJ=0

  for r in $(seq 1 "$REPEATS"); do
    RUN_SEED=$(( SEED + J*100 + r ))
    LOG=$(./ns3 run "$APP --joiners=$J --simTime=$SIMTIME --seed=$RUN_SEED --rxSensDbm=$RXSENS --plExp=$PLEXP --refDist=$REFDIST --refLossDb=$REFLOSS --minEbBeforePick=$MINEB --joinBaseOffset=$BASE_OFFSET --joinBaseSlope=$BASE_SLOPE --joinRetryInterval=$RETRY --joinTimeout=$TIMEOUT" 2>&1 || true)

    # Extract Method-1 rate
    M1=$(printf "%s\n" "$LOG" \
      | grep -F "[Method-1] Ever-collided receivers:" \
      | tail -n1 \
      | sed -n 's/.*rate=\([0-9.eE+-]\+\).*/\1/p' || true)
    if [[ -n "$M1" ]]; then vals1+="$M1\n"; cnt1=$((cnt1+1)); fi

    # Extract Method-2a probability
    M2A=$(printf "%s\n" "$LOG" \
      | grep -F "[Method-2a] Collision probability (receiver-slot):" \
      | tail -n1 \
      | sed -n 's/.*receiver-slot): \([0-9.eE+-]\+\).*/\1/p' || true)
    if [[ -n "$M2A" ]]; then vals2a+="$M2A\n"; cnt2a=$((cnt2a+1)); fi

    # Extract Method-2b severity
    M2B=$(printf "%s\n" "$LOG" \
      | grep -F "[Method-2b] Collision severity avg (k-1 per receiver-slot):" \
      | tail -n1 \
      | sed -n 's/.*receiver-slot): \([0-9.eE+-]\+\).*/\1/p' || true)
    if [[ -n "$M2B" ]]; then vals2b+="$M2B\n"; cnt2b=$((cnt2b+1)); fi

    # Extract Not-joined rate (prefer explicit rate=..., else compute from Unassigned nodes)
    NJ=$(printf "%s\n" "$LOG" \
      | grep -F "Not-joined count / joiners" \
      | tail -n1 \
      | sed -n 's/.*rate=\([0-9.eE+-]\+\).*/\1/p' || true)
    if [[ -z "$NJ" ]]; then
      UN=$(printf "%s\n" "$LOG" \
        | grep -F "Unassigned nodes:" \
        | tail -n1 \
        | sed -n 's/.*Unassigned nodes: \([0-9]\+\).*/\1/p' || true)
      if [[ -n "$UN" ]]; then
        NJ=$(awk -v u="$UN" -v j="$J" 'BEGIN{ if (j>0) printf("%.10f", u/j); }')
      fi
    fi
    if [[ -n "$NJ" ]]; then valsNJ+="$NJ\n"; cntNJ=$((cntNJ+1)); fi
  done

  # Averages
  if [[ $cnt1 -gt 0 ]]; then AVG1=$(printf "%b" "$vals1" | awk 'BEGIN{s=0;n=0} NF{ s+=$1; n++ } END{ if(n>0) printf("%.10f", s/n); }'); else AVG1="NA"; fi
  if [[ $cnt2a -gt 0 ]]; then AVG2A=$(printf "%b" "$vals2a" | awk 'BEGIN{s=0;n=0} NF{ s+=$1; n++ } END{ if(n>0) printf("%.10f", s/n); }'); else AVG2A="NA"; fi
  if [[ $cnt2b -gt 0 ]]; then AVG2B=$(printf "%b" "$vals2b" | awk 'BEGIN{s=0;n=0} NF{ s+=$1; n++ } END{ if(n>0) printf("%.10f", s/n); }'); else AVG2B="NA"; fi
  if [[ $cntNJ -gt 0 ]]; then AVGNJ=$(printf "%b" "$valsNJ" | awk 'BEGIN{s=0;n=0} NF{ s+=$1; n++ } END{ if(n>0) printf("%.10f", s/n); }'); else AVGNJ="NA"; fi

  printf "%d %s %s %s %s\n" "$J" "$AVG1" "$AVG2A" "$AVG2B" "$AVGNJ" >> "$OUT"
done

echo "[sweep] done. Results in $OUT" >&2
