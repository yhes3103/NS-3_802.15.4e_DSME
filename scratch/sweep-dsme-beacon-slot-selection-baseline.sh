#!/usr/bin/env bash
set -euo pipefail

# Sweep joiner counts for dsme-beacon-slot-selection-baseline and average metrics.
#   使用方式：
#     bash scratch/sweep-random-topo.sh [START] [END] [OUTFILE] [REPEATS] [STEP]
#   預設：START=5 END=50 OUTFILE=random_topo_sweep.txt REPEATS=20 STEP=5
# 可用環境變數覆寫：
#   SEED(4) SIMTIME(15) APP(scratch/dsme-beacon-slot-selection-baseline)
#   RXSENS(-95) PLEXP(2.7) REFDIST(1.0) REFLOSS(40.05)
#   MINEB(1) BASE_OFFSET(2.0) BASE_SLOPE(0.20) RETRY(0.25) TIMEOUT(6.0)

START=${1:-10}
END=${2:-50}
OUT=${3:-dsme-beacon-slot-selection-baseline.txt}
REPEATS=${4:-100}
STEP=${5:-5}

SEED=${SEED:-4}
SIMTIME=${SIMTIME:-15}
APP=${APP:-scratch/dsme-beacon-slot-selection-baseline}

RXSENS=${RXSENS:--95}
PLEXP=${PLEXP:-2.7}
REFDIST=${REFDIST:-1.0}
REFLOSS=${REFLOSS:-40.05}

MINEB=${MINEB:-1}
BASE_OFFSET=${BASE_OFFSET:-2.0}
BASE_SLOPE=${BASE_SLOPE:-0.20}
RETRY=${RETRY:-0.25}
TIMEOUT=${TIMEOUT:-6.0}

echo "# joiners   avg_p_coll   avg_s_coll   avg_eta   avg_P_tx_avg_dBm   repeats=$REPEATS step=$STEP" > "$OUT"

for J in $(seq "$START" "$STEP" "$END"); do
  echo "[sweep] joiners=$J repeats=$REPEATS" >&2
  vals_pcoll=""
  vals_scoll=""
  vals_eta=""
  vals_ptx=""
  cnt_pcoll=0; cnt_scoll=0; cnt_eta=0; cnt_ptx=0

  for r in $(seq 1 "$REPEATS"); do
    RUN_SEED=$(( SEED + J*100 + r ))
    LOG=$(./ns3 run "$APP --joiners=$J --simTime=$SIMTIME --seed=$RUN_SEED --rxSensDbm=$RXSENS --plExp=$PLEXP --refDist=$REFDIST --refLossDb=$REFLOSS --minEbBeforePick=$MINEB --joinBaseOffset=$BASE_OFFSET --joinBaseSlope=$BASE_SLOPE --joinRetryInterval=$RETRY --joinTimeout=$TIMEOUT --verbose=false" 2>&1 || true)

    # Extract p_coll
    V=$(printf "%s\n" "$LOG" \
      | grep -F "[p_coll]" \
      | tail -n1 \
      | sed -n 's/.*probability: *\([0-9.eE+-]\+\).*/\1/p' || true)
    if [[ -n "$V" ]]; then vals_pcoll+="$V\n"; cnt_pcoll=$((cnt_pcoll+1)); fi

    # Extract s_coll
    V=$(printf "%s\n" "$LOG" \
      | grep -F "[s_coll]" \
      | tail -n1 \
      | sed -n 's/.*severity: *\([0-9.eE+-]\+\).*/\1/p' || true)
    if [[ -n "$V" ]]; then vals_scoll+="$V\n"; cnt_scoll=$((cnt_scoll+1)); fi

    # Extract eta
    V=$(printf "%s\n" "$LOG" \
      | grep -F "[eta]" \
      | tail -n1 \
      | sed -n 's/.*rate: *\([0-9.eE+-]\+\).*/\1/p' || true)
    if [[ -n "$V" ]]; then vals_eta+="$V\n"; cnt_eta=$((cnt_eta+1)); fi

    # Extract P_tx_avg (dBm)
    V=$(printf "%s\n" "$LOG" \
      | grep -F "[P_tx_avg]" \
      | tail -n1 \
      | sed -n 's/.*power: *\([0-9.eE+-]\+\) dBm.*/\1/p' || true)
    if [[ -n "$V" ]]; then vals_ptx+="$V\n"; cnt_ptx=$((cnt_ptx+1)); fi
  done

  # Averages
  avg() { local vals="$1" cnt="$2"; if [[ $cnt -gt 0 ]]; then printf "%b" "$vals" | awk 'BEGIN{s=0;n=0} NF{s+=$1;n++} END{if(n>0) printf("%.10f",s/n)}'; else echo "NA"; fi; }
  AVG_PCOLL=$(avg "$vals_pcoll" "$cnt_pcoll")
  AVG_SCOLL=$(avg "$vals_scoll" "$cnt_scoll")
  AVG_ETA=$(avg "$vals_eta" "$cnt_eta")
  AVG_PTX=$(avg "$vals_ptx" "$cnt_ptx")

  printf "%d %s %s %s %s\n" "$J" "$AVG_PCOLL" "$AVG_SCOLL" "$AVG_ETA" "$AVG_PTX" >> "$OUT"
done

echo "[sweep] done. Results in $OUT" >&2
