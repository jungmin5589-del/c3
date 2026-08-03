#!/usr/bin/env python3
"""Local validation that does not require Isaac Sim runtime."""
from __future__ import annotations

import json
from pathlib import Path
import py_compile
import subprocess

root = Path(__file__).resolve().parents[1]
required = [
    root / "assets/bed_amr_clear.glb",
    root / "assets/bed_amr_clear_front_rear_open.glb",
    root / "config/amr_config.json",
    root / "scripts/create_demo_stage.py",
    root / "scripts/run_multi_amr_system.py",
    root / "scripts/fleet_magnetic_docking.py",
    root / "scripts/generate_with_map.sh",
    root / "scripts/navigation_math.py",
    root / "scripts/check_nav_stage_1_to_6.sh",
    root / "scripts/test_cmd_vel.sh",
    root / "docs/NAVIGATION_STAGE_1_TO_6.md",
    root / "docs/V1_12_MAGNETIC_WHEEL_TOW.md",
    root / "docs/V1_13_HARD_SNAP_MAGNETIC_LOCK.md",
    root / "docs/V1_14_360_LIDAR_DUAL_CAMERA_DUAL_END_OPENING.md",
    root / "docs/V1_15_NAMEPLATES_INDIVIDUAL_USD_LIDAR_CLEAR.md",
    root / "assets/nameplates/nameplate_seo_suwon.png",
    root / "assets/nameplates/nameplate_kim_seoul.png",
    root / "assets/nameplates/nameplate_park_incheon.png",
    root / "individual_usd_assets/AMR1.usd",
    root / "individual_usd_assets/AMR2.usd",
    root / "individual_usd_assets/HospitalBed_SeoSuwon.usd",
    root / "individual_usd_assets/HospitalBed_KimSeoul.usd",
    root / "individual_usd_assets/HospitalBed_ParkIncheon.usd",
    root / "README.md",
]
missing = [str(path) for path in required if not path.exists()]
if missing:
    raise SystemExit("Missing files:\n- " + "\n- ".join(missing))

cfg = json.loads((root / "config/amr_config.json").read_text(encoding="utf-8"))
assert len(cfg["fleet"]) == 2
assert len(cfg["beds"]) == 3
assert len({unit["root_path"] for unit in cfg["fleet"]}) == 2
assert len({unit["namespace"] for unit in cfg["fleet"]}) == 2
assert len({bed["root_path"] for bed in cfg["beds"]}) == 3
assert cfg["amr"]["lift_test_target_m"] <= cfg["amr"]["lift_upper_limit_m"]
assert cfg["control"]["unloaded_linear_speed_mps"] == 1.70
assert cfg["control"]["unloaded_lateral_speed_mps"] == 1.70
assert cfg["control"]["unloaded_angular_speed_rad_s"] == 2.80
assert cfg["control"]["loaded_linear_speed_mps"] == 0.90
assert cfg["control"]["loaded_lateral_speed_mps"] == 0.76
assert cfg["control"]["loaded_angular_speed_rad_s"] == 1.30
assert cfg["control"]["speed_profile"] == "v1.15_nameplates_individual_usd_lidar_clear"
beam_half_y = cfg["bed"]["adapter_connection_beam_size"][1] * 0.5
side_frame_inner_y = cfg["bed"]["side_frame_y"] - cfg["bed"]["side_frame_size"][1] * 0.5
assert beam_half_y >= side_frame_inner_y, "adapter connection beam does not reach bed side frame"
beam_z_min = cfg["bed"]["adapter_connection_beam_center_z"] - cfg["bed"]["adapter_connection_beam_size"][2] * 0.5
beam_z_max = cfg["bed"]["adapter_connection_beam_center_z"] + cfg["bed"]["adapter_connection_beam_size"][2] * 0.5
side_z_min = cfg["bed"]["side_frame_center_z"] - cfg["bed"]["side_frame_size"][2] * 0.5
side_z_max = cfg["bed"]["side_frame_center_z"] + cfg["bed"]["side_frame_size"][2] * 0.5
assert beam_z_max >= side_z_min and beam_z_min <= side_z_max, "adapter beam has no vertical overlap with bed frame"
assert cfg["control"]["keymap"]["amr2"]["lift_up"] == "LEFT_BRACKET"
assert cfg["control"]["keymap"]["amr2"]["lift_down"] == "RIGHT_BRACKET"
assert cfg["control"]["keymap"]["amr2"]["strafe_modifier"] == "SLASH"
assert cfg["control"]["keymap"]["amr2"]["strafe_left"] == "COMMA"
assert cfg["control"]["keymap"]["amr2"]["strafe_right"] == "PERIOD"
assert cfg["magnetic_dock"]["coupling_mode"] == "wheel_tow"
assert 0.0 <= cfg["magnetic_dock"]["wheel_tow_contact_lift_m"] <= cfg["magnetic_dock"]["wheel_tow_max_lift_m"] < cfg["amr"]["lift_upper_limit_m"]
assert 10.0 <= cfg["magnetic_dock"]["strength_percent"] <= 100.0
assert cfg["magnetic_dock"]["unbreakable_joint"] is True
assert cfg["magnetic_dock"]["manual_lock_required"] is True
assert cfg["magnetic_dock"]["auto_lock_enabled"] is False
assert cfg["materials"]["bed_caster_dynamic_friction"] < 0.2
assert cfg["sensors"]["magnet_strength_topic"] == "magnet_strength"
assert cfg["sensors"]["magnet_lock_topic"] == "magnet_lock"
assert cfg["sensors"]["magnet_release_topic"] == "magnet_release"
assert cfg["navigation"]["enabled"] is True
assert cfg["navigation"]["clock_topic"] == "/clock"
assert len(cfg["sensors"]["cameras"]) == 2
assert {camera["name"] for camera in cfg["sensors"]["cameras"]} == {"front", "rear"}
assert all(unit["camera_enabled"] is True for unit in cfg["fleet"])
assert all(unit.get("camera_count") == 2 for unit in cfg["fleet"])
assert cfg["amr"]["lidar_position"][0:2] == [0.0, 0.0]
assert cfg["amr"]["lidar_position"][2] > cfg["amr"]["lift_plate_center_z"]
assert cfg["bed"]["front_rear_open_edit"]["enabled"] is True
assert cfg["bed"]["end_frame_open_center_enabled"] is True
assert cfg["bed"]["nameplate_enabled"] is True
assert cfg["bed"]["nameplate_center_y_m"] == 0.0
assert cfg["bed"]["nameplate_center_x_m"] == 1.015
assert cfg["bed"]["nameplate_center_z_m"] == 0.67
assert cfg["amr"]["lidar_config"] == "Example_Rotary_2D"
assert cfg["amr"]["lidar_position"] == [0.0, 0.0, 0.285]
housing_top = cfg["amr"]["lidar_housing_center_offset_z_m"] + cfg["amr"]["lidar_housing_height_m"] * 0.5
assert housing_top <= -cfg["amr"]["lidar_scan_plane_clearance_m"]
for bed_instance in cfg["beds"]:
    texture = root / bed_instance["nameplate_texture"]
    assert texture.exists(), texture
    assert bed_instance["patient_name"]
    assert bed_instance["patient_birth_date"]

for script in (root / "scripts").glob("*.py"):
    py_compile.compile(str(script), doraise=True)
for script in (root / "scripts").glob("*.sh"):
    result = subprocess.run(["bash", "-n", str(script)], capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(f"Shell syntax error: {script}\n{result.stderr}")
dock_source = (root / "scripts/fleet_magnetic_docking.py").read_text(encoding="utf-8")
run_source = (root / "scripts/run_multi_amr_system.py").read_text(encoding="utf-8")
assert "CreateBreakForceAttr" in dock_source
assert "CreateBreakTorqueAttr" in dock_source
assert "CreateExcludeFromArticulationAttr(True)" in dock_source
assert "request_lock" in dock_source
assert "KeyboardInput.C" in run_source
assert "KeyboardInput.X" in run_source
assert "KeyboardInput.BACKSLASH" in run_source
assert "KeyboardInput.BACKSPACE" in run_source
assert "KeyboardInput.SLASH" in run_source
assert "KeyboardInput.COMMA" in run_source
assert "KeyboardInput.PERIOD" in run_source
stage_source = (root / "scripts/create_demo_stage.py").read_text(encoding="utf-8")
assert "LidarPedestal" in stage_source
assert "{name}_camera_link" in stage_source
assert "front_camera_path" in stage_source
assert "rear_camera_path" in stage_source
assert "HeadFramePostLeft" not in stage_source  # names are generated dynamically
assert "HeadFramePost{side}" in stage_source
assert "make_omnipbr_texture_material" in stage_source
assert "add_nameplate_plane" in stage_source
assert "lidar_housing_center_offset_z_m" in stage_source
assert "camera_definitions" in run_source
assert 'f"{ns}/{camera_name}_camera_link"' in run_source
print("[ok] v1.15 nameplates, individual USD, clear 2D 360 LiDAR validation passed")
