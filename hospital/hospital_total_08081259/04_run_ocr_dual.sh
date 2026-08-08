#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="${OCR_ROS_VENV:-$HOME/.venvs/hospital_ocr_ros310}"
source "$ROOT/scripts/clean_ros_env.sh"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
[[ -x "$VENV/bin/python" ]] || { echo "[ERROR] Run ./01_install_ocr_ros.sh first" >&2; exit 1; }
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] || { echo "[ERROR] Run ./02_build_ros_ws.sh first" >&2; exit 1; }
# shellcheck disable=SC1090
source "$VENV/bin/activate"
VENV_SITE="$($VENV/bin/python -c 'import site; print(site.getsitepackages()[0])')"
export PYTHONPATH="$VENV_SITE${PYTHONPATH:+:$PYTHONPATH}"
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=117
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0
export PADDLE_PDX_MODEL_SOURCE=BOS
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
fi
mkdir -p "$ROOT/output/ocr"
cd "$ROOT"
echo "[OCR LAUNCH] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[OCR DUAL] AMR1 + AMR2 nodes; wait for both 'OCR model ready' logs before pressing O/P"
exec ros2 launch hospital_ocr_bridge dual_ocr.launch.py \
  output_root:="$ROOT/output/ocr"
