#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-/home/peter-msi/isaacsim-5.1.0}"
PYTHON_SH="$ISAAC_SIM_DIR/python.sh"
[[ -x "$PYTHON_SH" ]] || { echo "[ERROR] Isaac Sim python.sh not found: $PYTHON_SH" >&2; exit 1; }
source "$ROOT/scripts/clean_isaac_env.sh"
export ROS_DISTRO=humble
export ROS_DOMAIN_ID=117
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0
INTERNAL_ROS_LIB="$ISAAC_SIM_DIR/exts/isaacsim.ros2.bridge/humble/lib"
if [[ -d "$INTERNAL_ROS_LIB" ]]; then
  export LD_LIBRARY_PATH="$INTERNAL_ROS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
fi

echo "[ISAAC] Python-only standalone app"
echo "[ISAAC] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[ISAAC] Do NOT source /opt/ros/humble in this terminal"
exec "$PYTHON_SH" "$ROOT/scripts/isaac_amr_ros.py" \
  --project-root "$ROOT" \
  --config "$ROOT/config/isaac_config.json" \
  "$@"
