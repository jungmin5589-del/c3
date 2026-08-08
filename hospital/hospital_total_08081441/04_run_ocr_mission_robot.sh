#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AMR="${1:-}"
ARG_PATIENT="${2:-}"
[[ "$AMR" == "amr1" || "$AMR" == "amr2" ]] || { echo "Usage: $0 amr1|amr2 [1|2|3]" >&2; exit 2; }
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
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export PADDLE_PDX_MODEL_SOURCE=BOS
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"; fi
mkdir -p "$ROOT/output/ocr"

name_of(){ case "$1" in 1) echo 김서울;; 2) echo 박인천;; 3) echo 서수원;; esac; }

# 환자 중복 선택은 Nav2/별도 ROS 서버와 무관하게 동작해야 한다.
# 같은 PC에서 실행되는 AMR1/AMR2 mission이 동일 lock file을 사용한다.
# 프로세스가 정상/비정상 종료되면 커널이 FD를 닫으므로 예약도 자동 해제된다.
LOCK_DIR="/tmp/hospital_patient_claims_domain115"
mkdir -p "$LOCK_DIR"
PATIENT_LOCK_FD=""
PATIENT_LOCK_FILE=""
ARUCO_PID=""

claim_patient_lock(){
  local patient="$1"
  local lock_file="$LOCK_DIR/patient_${patient}.lock"
  local fd
  exec {fd}>"$lock_file"
  if ! flock -n "$fd"; then
    eval "exec ${fd}>&-"
    return 1
  fi
  PATIENT_LOCK_FD="$fd"
  PATIENT_LOCK_FILE="$lock_file"
  printf '%s %s pid=%s\n' "$AMR" "$(name_of "$patient")" "$$" >&"$PATIENT_LOCK_FD" || true
  return 0
}

release_patient_lock(){
  if [[ -n "${PATIENT_LOCK_FD:-}" ]]; then
    flock -u "$PATIENT_LOCK_FD" 2>/dev/null || true
    eval "exec ${PATIENT_LOCK_FD}>&-" 2>/dev/null || true
    PATIENT_LOCK_FD=""
  fi
}

cleanup(){
  if [[ -n "${ARUCO_PID:-}" ]]; then
    kill "$ARUCO_PID" 2>/dev/null || true
    wait "$ARUCO_PID" 2>/dev/null || true
    ARUCO_PID=""
  fi
  release_patient_lock
}
trap cleanup EXIT INT TERM

PATIENT="$ARG_PATIENT"
while true; do
  if [[ -z "$PATIENT" ]]; then
    echo
    echo "================ ${AMR^^} 환자 선택 ================"
    echo "1 : 김서울"
    echo "2 : 박인천"
    echo "3 : 서수원"
    read -r -p "입력 > " PATIENT
  fi
  case "$PATIENT" in
    1|김서울) PATIENT=1;;
    2|박인천) PATIENT=2;;
    3|서수원) PATIENT=3;;
    *) echo "[선택 무효] 1/2/3 중 선택하세요."; PATIENT=""; continue;;
  esac

  if claim_patient_lock "$PATIENT"; then
    echo "[환자 선택] ${AMR^^} -> $(name_of "$PATIENT")"
    break
  fi

  echo "[중복 선택 차단] $(name_of "$PATIENT") 환자는 다른 AMR이 이미 수행 중입니다. 다른 환자를 선택하세요."
  PATIENT=""
done

cd "$ROOT"

if ! "$VENV/bin/python" -c 'import cv2, sys; sys.exit(0 if hasattr(cv2, "aruco") else 1)' >/dev/null 2>&1; then
  echo "[ERROR] cv2.aruco 없음: $VENV/bin/python -m pip install --force-reinstall opencv-contrib-python==4.10.0.84" >&2
  exit 1
fi

# ArUco is an independent companion detector only.  The proven OCR mission launch,
# Nav2, Local Costmap and patient_transport_manager are left untouched.
ARUCO_IMAGE_TOPIC="/${AMR}/camera/front/color/image_raw"
ARUCO_RESULT_TOPIC="/${AMR}/aruco/result"
ARUCO_DEBUG_TOPIC="/${AMR}/aruco/debug_image"
"$VENV/bin/python" "$ROOT/scripts/aruco_pair_node.py" --ros-args \
  -r "__node:=${AMR}_aruco_pair" \
  -p "amr_id:=${AMR}" \
  -p "image_topic:=${ARUCO_IMAGE_TOPIC}" \
  -p "result_topic:=${ARUCO_RESULT_TOPIC}" \
  -p "debug_image_topic:=${ARUCO_DEBUG_TOPIC}" \
  -p "dictionary:=DICT_4X4_50" \
  -p "publish_hz:=15.0" &
ARUCO_PID=$!
echo "[ARUCO] ${AMR^^} detector pid=$ARUCO_PID image=$ARUCO_IMAGE_TOPIC result=$ARUCO_RESULT_TOPIC"

LAUNCH="${AMR}_ocr_mission.launch.py"
echo "[OCR+MISSION] ${AMR^^} / patient=$(name_of "$PATIENT") / ROS_DOMAIN_ID=115"
ros2 launch hospital_ocr_bridge "$LAUNCH" project_root:="$ROOT" patient:="$PATIENT"
