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
  "$PROJECT_DIR/scripts/generate_all.sh"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

"$PYTHON_SH" "$PROJECT_DIR/scripts/run_multi_amr_system.py" \
  --config "$CONFIG" \
  --stage "$STAGE"
