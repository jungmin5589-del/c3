#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-/home/peter-msi/isaacsim-5.1.0}"
ISAAC_SH="$ISAAC_SIM_DIR/isaac-sim.sh"

STAGE="$ROOT/project4/project4_hospital_bed_amr_v1_15_ocr.usd"
MAP_DIR="$ROOT/ros2_ws/src/hospital_nav2/maps"

if [[ ! -x "$ISAAC_SH" ]]; then
    echo "[ERROR] isaac-sim.sh not found:"
    echo "$ISAAC_SH"
    exit 1
fi

if [[ ! -f "$STAGE" ]]; then
    echo "[ERROR] Hospital USD not found:"
    echo "$STAGE"
    exit 1
fi

mkdir -p "$MAP_DIR"

echo "============================================================"
echo "[MAP] Isaac Sim:"
echo "$ISAAC_SH"
echo
echo "[MAP] Opening USD:"
echo "$STAGE"
echo
echo "[MAP] Save generated map into:"
echo "$MAP_DIR"
echo "============================================================"

exec "$ISAAC_SH" \
    --enable isaacsim.asset.gen.omap \
    --enable isaacsim.asset.gen.omap.ui \
    --/app/content/emptyStageOnStart=1 \
    --exec "open_stage.py $STAGE"
