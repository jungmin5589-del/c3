#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_DIR="${ISAAC_SIM_DIR:-/mnt/isaac45/isaacsim_5.1}"
PYTHON_SH="$ISAAC_DIR/python.sh"
OUTPUT_DIR="$PROJECT_DIR/output"
FINAL_STAGE="$OUTPUT_DIR/hospital_bed_amr_multi_map_ready.usd"

if [[ ! -x "$PYTHON_SH" ]]; then
  echo "[오류] Isaac Sim python.sh를 찾지 못했습니다: $PYTHON_SH" >&2
  exit 1
fi
mkdir -p "$OUTPUT_DIR"
rm -f "$FINAL_STAGE"

ARGS=(
  "$PROJECT_DIR/scripts/create_demo_stage.py"
  --config "$PROJECT_DIR/config/amr_config.json"
  --bed-glb "$PROJECT_DIR/assets/bed_amr_clear.glb"
  --visual-usd "$OUTPUT_DIR/bed_amr_clear_visual.usd"
  --output "$FINAL_STAGE"
)

if [[ -n "${HOSPITAL_MAP_USD:-}" ]]; then
  ARGS+=(--map-usd "$HOSPITAL_MAP_USD")
fi

if ! "$PYTHON_SH" "${ARGS[@]}"; then
  echo "[오류] 멀티 AMR USD 생성 실패" >&2
  rm -f "$FINAL_STAGE"
  exit 1
fi

[[ -s "$FINAL_STAGE" ]] || { echo "[오류] 생성된 USD가 없습니다: $FINAL_STAGE" >&2; exit 1; }
echo "[완료] 멀티 AMR + 침대 3대 Stage: $FINAL_STAGE"
