#!/usr/bin/env bash
set -euo pipefail

# Sweep numNodes = 15,18,...,45 and record the averages of three collision
# metrics emitted by the powerControl app summary:
#   - [Method-1] Ever-collided receivers (rate)
#   - [Method-2a] Collision probability (receiver-slot)
#   - [Method-2b] Collision severity avg (k-1 per receiver-slot)
# For each numNodes, run REPEATS times and average each metric.
#
# Usage:
#   bash scratch/sweep-collision-pc.sh [START] [END] [OUTFILE] [REPEATS] [STEP]
# Defaults: START=15 END=45 OUTFILE=collision-sweep-pc.txt REPEATS=3 STEP=3

START=${1:-15}
END=${2:-38}
OUT=${3:-collision-sweep-pc.txt}
REPEATS=${4:-100}
STEP=${5:-3}

# Optional environment overrides
SEED=${SEED:-4}
SIMTIME=${SIMTIME:-15}
APP=${APP:-scratch/dsme-beacon-slot-selection-random-pick-backbone-powerControl}
APP_ARGS=${APP_ARGS:---pcOnlyReduce=1 --pcTargetPrDbm=-95 --pcMarginDb=3 --txMinDbm=-20 --txMaxDbm=3 --statsUseActualTx=1 --plExp=3}

echo "# numNodes avg_method1_rate avg_method2a_prob avg_method2b_severity avg_not_joined_rate avg_avgTx_mW avg_avgTx_dBm repeats=$REPEATS step=$STEP" > "$OUT"

for N in $(seq "$START" "$STEP" "$END"); do
  echo "[sweep-pc] numNodes=$N repeats=$REPEATS" >&2
  vals1=""  # Method-1 rates
  vals2a="" # Method-2a probabilities
  vals2b="" # Method-2b severities
  cnt1=0; cnt2a=0; cnt2b=0
  valsNJ=""  # Not-joined rate
  cntNJ=0
  valsMW=""  # Avg TX power (mW)
  cntMW=0
  valsDBM="" # Avg TX power (dBm)
  cntDBM=0

  for r in $(seq 1 "$REPEATS"); do
    # Vary seed per (N, r) to decorrelate runs
    RUN_SEED=$(( SEED + N*100 + r ))
    LOG=$(./ns3 run "$APP --numNodes=$N --simTime=15 --seed=$RUN_SEED --pcOnlyReduce=1  --pcTargetPrDbm=-95 --pcMarginDb=3 --txMinDbm=-20 --txMaxDbm=3  --statsUseActualTx=1 --plExp=3" 2>&1 || true)

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

    # Extract Not-joined rate (prefer explicit rate=..., else fallback compute from 'Unassigned nodes: X')
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
        if (( N > 9 )); then
          NJ=$(awk -v u="$UN" -v n="$N" 'BEGIN{b=9;j=n-b; if (j>0) printf("%.10f", u/j); }')
        fi
      fi
    fi
    if [[ -n "$NJ" ]]; then valsNJ+="$NJ\n"; cntNJ=$((cntNJ+1)); fi

    # Extract Avg TX power (mW and dBm); if not present, derive from per-node table
    ALINE=$(printf "%s\n" "$LOG" | grep -F "Avg TX power of joined joiners:" | tail -n1 || true)
    AMW=$(printf "%s\n" "$ALINE" | sed -n 's/.*: \([0-9.eE+-]\+\) mW.*/\1/p')
    ADBM=$(printf "%s\n" "$ALINE" | sed -n 's/.*(\([0-9.eE+-]\+\) dBm).*/\1/p')
    if [[ -z "$AMW" || -z "$ADBM" ]]; then
      FALL=$(printf "%s\n" "$LOG" | awk 'BEGIN{sum=0; cnt=0; ln10=log(10);} /^[[:space:]]*[0-9]+[[:space:]]+/ { node=$1; pick=$2; tx=$3; if (node+0>=9 && pick ~ /^[0-9]+$/ && pick+0!=65535 && tx ~ /^-?[0-9.]+([eE][+-]?[0-9]+)?$/) { mw=exp((tx/10.0)*ln10); sum+=mw; cnt++; } } END{ if (cnt>0) { avgmw=sum/cnt; avgdbm=10*log(avgmw)/ln10; printf("%.10f %.10f", avgmw, avgdbm); } }')
      if [[ -n "$FALL" ]]; then
        AMW=$(printf "%s" "$FALL" | awk '{print $1}')
        ADBM=$(printf "%s" "$FALL" | awk '{print $2}')
      fi
    fi
    if [[ -n "$AMW" ]]; then valsMW+="$AMW\n"; cntMW=$((cntMW+1)); fi
    if [[ -n "$ADBM" ]]; then valsDBM+="$ADBM\n"; cntDBM=$((cntDBM+1)); fi
    # After fallback, append if they were populated
    if [[ -z "$AMW" || -z "$ADBM" ]]; then
      if [[ -n "$AMW" ]]; then valsMW+="$AMW\n"; cntMW=$((cntMW+1)); fi
      if [[ -n "$ADBM" ]]; then valsDBM+="$ADBM\n"; cntDBM=$((cntDBM+1)); fi
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

  if [[ $cntNJ -gt 0 ]]; then
    AVGNJ=$(printf "%b" "$valsNJ" | awk 'BEGIN{s=0;n=0} NF{ s+=$1; n++ } END{ if(n>0) printf("%.10f", s/n); }')
  else
    AVGNJ="NA"
  fi
  if [[ $cntMW -gt 0 ]]; then
    AVGMW=$(printf "%b" "$valsMW" | awk 'BEGIN{s=0;n=0} NF{ s+=$1; n++ } END{ if(n>0) printf("%.10f", s/n); }')
  else
    AVGMW="NA"
  fi
  if [[ $cntDBM -gt 0 ]]; then
    AVGDBM=$(printf "%b" "$valsDBM" | awk 'BEGIN{s=0;n=0} NF{ s+=$1; n++ } END{ if(n>0) printf("%.10f", s/n); }')
  else
    AVGDBM="NA"
  fi
  printf "%d %s %s %s %s %s %s\n" "$N" "$AVG1" "$AVG2A" "$AVG2B" "$AVGNJ" "$AVGMW" "$AVGDBM" >> "$OUT"
done

echo "[sweep-pc] done. Results in $OUT" >&2
