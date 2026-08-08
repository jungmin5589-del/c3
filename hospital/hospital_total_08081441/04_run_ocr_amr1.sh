#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="${OCR_ROS_VENV:-$HOME/.venvs/hospital_ocr_ros310}"
source "$ROOT/scripts/clean_ros_env.sh"
source /opt/ros/humble/setup.bash
[[ -x "$VENV/bin/python" ]] || { echo "[ERROR] Run ./01_install_ocr_ros.sh first" >&2; exit 1; }
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] || { echo "[ERROR] Run ./02_build_ros_ws.sh first" >&2; exit 1; }
source "$VENV/bin/activate"
VENV_SITE="$($VENV/bin/python -c 'import site; print(site.getsitepackages()[0])')"
export PYTHONPATH="$VENV_SITE${PYTHONPATH:+:$PYTHONPATH}"
source "$ROOT/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0
export PADDLE_PDX_MODEL_SOURCE=BOS
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
fi
mkdir -p "$ROOT/output/ocr"
# Prevent old OCR nodes from accumulating across repeated demos.
pkill -f hospital_ocr_node 2>/dev/null || true
pkill -f "ros2 launch hospital_ocr_bridge amr1_ocr.launch.py" 2>/dev/null || true
sleep 1
cd "$ROOT"
echo "[OCR AMR1] Wait for '[amr1] OCR model ready'. Do not press O; the mission manager starts OCR."
exec ros2 launch hospital_ocr_bridge amr1_ocr.launch.py output_root:="$ROOT/output/ocr"
