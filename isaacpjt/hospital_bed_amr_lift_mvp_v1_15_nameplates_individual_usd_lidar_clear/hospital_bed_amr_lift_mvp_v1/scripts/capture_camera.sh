#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_DIR="${ISAAC_SIM_DIR:-/mnt/isaac45/isaacsim_5.1}"
PYTHON_SH="$ISAAC_DIR/python.sh"
STAGE="$PROJECT_DIR/output/hospital_bed_amr_multi_map_ready.usd"
CONFIG="$PROJECT_DIR/config/amr_config.json"

if [[ ! -x "$PYTHON_SH" ]]; then
  echo "[오류] Isaac Sim python.sh를 찾지 못했습니다: $PYTHON_SH" >&2
  exit 1
fi
if [[ ! -f "$STAGE" ]]; then
  echo "[안내] USD가 없어 먼저 생성합니다."
  "$PROJECT_DIR/scripts/generate_all.sh"
fi

OUTPUT_DIR="$PROJECT_DIR/output/sensor_capture"
"$PYTHON_SH" "$PROJECT_DIR/scripts/capture_camera.py" \
  --config "$CONFIG" \
  --stage "$STAGE" \
  --output-dir "$OUTPUT_DIR"

echo "[완료] 센서 캡처 폴더: $OUTPUT_DIR"
