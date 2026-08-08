#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

python3 -m py_compile \
  "$ROOT/patient_transport_manager.py" \
  "$ROOT/scripts/isaac_amr_ros.py" \
  "$ROOT/scripts/aruco_nameplate_markers.py" \
  "$ROOT/scripts/nav2_bridge.py" \
  "$ROOT/ros2_ws/src/hospital_ocr_bridge/hospital_ocr_bridge/nameplate_vision.py" \
  "$ROOT/ros2_ws/src/hospital_ocr_bridge/hospital_ocr_bridge/ocr_node.py" \
  "$ROOT/ros2_ws/src/hospital_ocr_bridge/hospital_ocr_bridge/aruco_pair_node.py" \
  "$ROOT/ros2_ws/src/hospital_ocr_bridge/launch/amr1_ocr.launch.py" \
  "$ROOT/ros2_ws/src/hospital_ocr_bridge/launch/amr2_ocr.launch.py" \
  "$ROOT/ros2_ws/src/hospital_ocr_bridge/launch/amr1_ocr_mission.launch.py" \
  "$ROOT/ros2_ws/src/hospital_ocr_bridge/launch/amr2_ocr_mission.launch.py" \
  "$ROOT/ros2_ws/src/hospital_ocr_bridge/launch/dual_ocr.launch.py"

find "$ROOT" -type d -name __pycache__ -prune -exec rm -rf {} +
find "$ROOT" -type f -name '*.pyc' -delete

python3 - "$ROOT" <<'PY'
from pathlib import Path
import json
import sys
root = Path(sys.argv[1])
cfg = json.loads((root / "config/isaac_config.json").read_text(encoding="utf-8"))
aruco = cfg["aruco_markers"]
assert aruco["enabled"] is True
assert [(b["left_id"], b["right_id"]) for b in aruco["beds"]] == [(10, 11), (20, 21), (30, 31)]
for bed in aruco["beds"]:
    assert (root / bed["left_texture"]).is_file()
    assert (root / bed["right_texture"]).is_file()

manager = (root / "patient_transport_manager.py").read_text(encoding="utf-8")
assert 'ARUCO_PAIRS = {"김서울": (10, 11)' in manager
assert "bbox_center_x" not in manager
assert "self.aruco_result_topic" in manager
assert "cmd.linear.y = raw_y" in manager
assert '"approach_distance_m": 2.87' in manager

isaac_source = (root / "scripts/isaac_amr_ros.py").read_text(encoding="utf-8")
assert "install_aruco_markers" in isaac_source
assert "[ARUCO READY]" in isaac_source
bridge_source = (root / "scripts/nav2_bridge.py").read_text(encoding="utf-8")
assert "float(msg.linear.y)" in bridge_source
setup_source = (root / "ros2_ws/src/hospital_ocr_bridge/setup.py").read_text(encoding="utf-8")
assert "aruco_pair_node = hospital_ocr_bridge.aruco_pair_node:main" in setup_source
launch_source = "\n".join(
    path.read_text(encoding="utf-8")
    for path in (root / "ros2_ws/src/hospital_ocr_bridge/launch").glob("*.launch.py")
)
assert launch_source.count('executable="aruco_pair_node"') >= 6

ocr_source = (
    root / "ros2_ws/src/hospital_ocr_bridge/hospital_ocr_bridge/ocr_node.py"
).read_text(encoding="utf-8")
assert 'if ranked and float(ranked[0]["score"]) > 0.0:' in ocr_source
assert '"name": "",' in ocr_source
assert "stale/cancelled OCR result discarded" in ocr_source
print("[OK] OCR is identity-only in mission manager; bbox motion control absent")
print("[OK] Paired ArUco IDs/assets: Kim 10/11, Park 20/21, Seo 30/31")
print("[OK] Isaac marker install + AMR1/AMR2 detector launches")
print("[OK] Nav2 bridge forwards lateral linear.y for ArUco alignment")
print("[OK] OCR request lifecycle clears rejected/cancelled requests")
PY

if find "$ROOT" -type d -name third_party | grep -q .; then
  echo "[FAIL] third_party directory exists" >&2
  exit 2
fi
if find "$ROOT" -type d -name __pycache__ | grep -q .; then
  echo "[FAIL] __pycache__ directory exists" >&2
  exit 3
fi

[[ -f "$ROOT/project4/project4_hospital_bed_amr_v1_15_ocr.usd" ]]
[[ -f "$ROOT/config/isaac_config.json" ]]

echo "[OK] Python syntax"
echo "[OK] No third_party / cache files"
echo "[OK] V4 runtime stage exists"
echo "[PASS] Project source check complete"
