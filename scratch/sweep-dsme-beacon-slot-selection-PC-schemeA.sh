#!/usr/bin/env bash
set -euo pipefail

# Sweep joiner counts for dsme-beacon-slot-selection-PC-schemeA and average metrics.
#   使用方式：
#     bash scratch/sweep-dsme-beacon-slot-selection-PC-schemeA.sh [START] [END] [OUTFILE] [REPEATS] [STEP]
#   預設：START=10 END=50 OUTFILE=dsme-beacon-slot-selection-PC-schemeA.txt REPEATS=100 STEP=5
# 可用環境變數覆寫：
#   SEED(4) SIMTIME(15) APP(scratch/dsme-beacon-slot-selection-PC-schemeA)
#   RXSENS(-95) PLEXP(2.7) REFDIST(1.0) REFLOSS(40.05)
#   MINEB(1) BASE_OFFSET(2.0) BASE_SLOPE(0.20) RETRY(0.25) TIMEOUT(6.0)
#   PC_MARGIN(3.0) TX_MIN(-32.0) TX_MAX(0.0) PANC_TX(0.0)

START=${1:-10}
END=${2:-50}
OUT=${3:-dsme-beacon-slot-selection-PC-schemeA.txt}
REPEATS=${4:-100}
STEP=${5:-5}

SEED=${SEED:-4}
SIMTIME=${SIMTIME:-15}
APP=${APP:-scratch/dsme-beacon-slot-selection-PC-schemeA}

RXSENS=${RXSENS:--95}
PLEXP=${PLEXP:-2.7}
REFDIST=${REFDIST:-1.0}
REFLOSS=${REFLOSS:-40.05}

MINEB=${MINEB:-1}
BASE_OFFSET=${BASE_OFFSET:-2.0}
BASE_SLOPE=${BASE_SLOPE:-0.20}
RETRY=${RETRY:-0.25}
TIMEOUT=${TIMEOUT:-6.0}

# PC-specific knobs
PC_MARGIN=${PC_MARGIN:-3.0}
TX_MIN=${TX_MIN:--32.0}
TX_MAX=${TX_MAX:-0.0}
PANC_TX=${PANC_TX:-0.0}

echo "# joiners  p_coll_mean p_coll_std  s_coll_mean s_coll_std  eta_mean eta_std  Ptx_mean_dBm Ptx_std_dBm   repeats=$REPEATS step=$STEP pcMargin=$PC_MARGIN txMin=$TX_MIN txMax=$TX_MAX panCoordTx=$PANC_TX" > "$OUT"

for J in $(seq "$START" "$STEP" "$END"); do
  echo "[sweep] joiners=$J repeats=$REPEATS" >&2
  vals_pcoll=""
  vals_scoll=""
  vals_eta=""
  vals_ptx=""
  cnt_pcoll=0; cnt_scoll=0; cnt_eta=0; cnt_ptx=0

  for r in $(seq 1 "$REPEATS"); do
    RUN_SEED=$(( SEED + J*100 + r ))
    LOG=$(./ns3 run "$APP --joiners=$J --simTime=$SIMTIME --seed=$RUN_SEED --rxSensDbm=$RXSENS --plExp=$PLEXP --refDist=$REFDIST --refLossDb=$REFLOSS --minEbBeforePick=$MINEB --joinBaseOffset=$BASE_OFFSET --joinBaseSlope=$BASE_SLOPE --joinRetryInterval=$RETRY --joinTimeout=$TIMEOUT --pcMarginDb=$PC_MARGIN --txMinDbm=$TX_MIN --txMaxDbm=$TX_MAX --panCoordTxDbm=$PANC_TX --verbose=false" 2>&1 || true)

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

  # Mean + sample std (n-1 denominator). Each metric -> "mean std" pair.
  stat() {
    local vals="$1" cnt="$2"
    if [[ $cnt -gt 0 ]]; then
      printf "%b" "$vals" | awk 'BEGIN{s=0;s2=0;n=0} NF{s+=$1;s2+=$1*$1;n++} END{
        if(n>0){
          m=s/n;
          v=(n>1)?(s2 - n*m*m)/(n-1):0;
          if(v<0) v=0;
          printf("%.10f %.10f", m, sqrt(v));
        }
      }'
    else
      printf "NA NA"
    fi
  }
  STAT_PCOLL=$(stat "$vals_pcoll" "$cnt_pcoll")
  STAT_SCOLL=$(stat "$vals_scoll" "$cnt_scoll")
  STAT_ETA=$(stat "$vals_eta" "$cnt_eta")
  STAT_PTX=$(stat "$vals_ptx" "$cnt_ptx")

  printf "%d %s %s %s %s\n" "$J" "$STAT_PCOLL" "$STAT_SCOLL" "$STAT_ETA" "$STAT_PTX" >> "$OUT"
done

echo "[sweep] done. Results in $OUT" >&2
