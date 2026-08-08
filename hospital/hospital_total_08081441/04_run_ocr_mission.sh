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
# One OCR node and one mission manager only.
pkill -f hospital_ocr_node 2>/dev/null || true
pkill -f patient_transport_manager.py 2>/dev/null || true
pkill -f "ros2 launch hospital_ocr_bridge amr1_ocr_mission.launch.py" 2>/dev/null || true
sleep 1

PATIENT="${1:-}"
if [[ -z "$PATIENT" ]]; then
  echo
  echo "================ 환자 선택 ================"
  echo "1 : 김서울"
  echo "2 : 박인천"
  echo "3 : 서수원"
  read -r -p "입력 > " PATIENT
fi
case "$PATIENT" in
  1|김서울) PATIENT=1 ;;
  2|박인천) PATIENT=2 ;;
  3|서수원) PATIENT=3 ;;
  *) echo "[ERROR] 1.김서울 / 2.박인천 / 3.서수원 중 하나를 선택하세요." >&2; exit 2 ;;
esac

cd "$ROOT"
echo "[OCR+MISSION] OCR node + 선택 환자 이송 manager를 시작합니다. patient=$PATIENT"
exec ros2 launch hospital_ocr_bridge amr1_ocr_mission.launch.py \
  project_root:="$ROOT" patient:="$PATIENT"
