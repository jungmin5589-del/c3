#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="${OCR_ROS_VENV:-$HOME/.venvs/hospital_ocr_ros310}"
VENV_PYTHON="$VENV/bin/python"

source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
set +u

cd "$ROOT/ros2_ws"
rm -rf build install log
colcon build --symlink-install

NAV_POSE="$ROOT/ros2_ws/install/hospital_nav2/lib/hospital_nav2/pose_lock_localizer"
NAV_CENTER="$ROOT/ros2_ws/install/hospital_nav2/lib/hospital_nav2/centerline_navigator"
NAV_WORLD_POSE="$ROOT/ros2_ws/install/hospital_nav2/lib/hospital_nav2/world_pose_initializer"
NAV_CORRIDOR="$ROOT/ros2_ws/install/hospital_nav2/lib/hospital_nav2/corridor_priority_manager"
NAV_PATH_CONFLICT="$ROOT/ros2_ws/install/hospital_nav2/lib/hospital_nav2/path_conflict_manager"
NAV_RVIZ_ICON="$ROOT/ros2_ws/install/hospital_nav2/lib/hospital_nav2/rviz_robot_icon"
[[ -x "$NAV_POSE" ]] || { echo "[ERROR] pose_lock_localizer build failed" >&2; exit 1; }
[[ -x "$NAV_CENTER" ]] || { echo "[ERROR] centerline_navigator build failed" >&2; exit 1; }
[[ -x "$NAV_WORLD_POSE" ]] || { echo "[ERROR] world_pose_initializer build failed" >&2; exit 1; }
[[ -x "$NAV_CORRIDOR" ]] || { echo "[ERROR] corridor_priority_manager build failed" >&2; exit 1; }
[[ -x "$NAV_PATH_CONFLICT" ]] || { echo "[ERROR] path_conflict_manager build failed" >&2; exit 1; }
[[ -x "$NAV_RVIZ_ICON" ]] || { echo "[ERROR] rviz_robot_icon build failed" >&2; exit 1; }

# OCR remains optional for Nav2. Repair its shebang only when the OCR venv exists.
OCR_NODE="$ROOT/ros2_ws/install/hospital_ocr_bridge/lib/hospital_ocr_bridge/hospital_ocr_node"
if [[ -x "$VENV_PYTHON" && -f "$OCR_NODE" ]]; then
  "$VENV_PYTHON" - "$OCR_NODE" "$VENV_PYTHON" <<'PY'
from pathlib import Path
import sys
path = Path(sys.argv[1])
python_exe = sys.argv[2]
lines = path.read_text(encoding="utf-8").splitlines()
if lines:
    lines[0] = f"#!{python_exe}"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    path.chmod(path.stat().st_mode | 0o111)
    print(f"[OCR] shebang fixed: {lines[0]}")
PY
else
  echo "[INFO] OCR venv 없음/미사용. Nav2 빌드는 정상 완료되었습니다."
fi

echo "[PASS] pose_lock_localizer"
echo "[PASS] centerline_navigator"
echo "[PASS] world_pose_initializer"
echo "[PASS] corridor_priority_manager"
echo "[PASS] path_conflict_manager"
echo "[PASS] rviz_robot_icon"
echo "[DONE] ROS 2 workspace built"
