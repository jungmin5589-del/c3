#!/usr/bin/env python3
"""Open the packaged v4 stage in Isaac Sim and verify required content."""
from __future__ import annotations

import json
from pathlib import Path
import sys

try:
    from isaacsim.simulation_app import SimulationApp
except ImportError:
    from isaacsim import SimulationApp

APP = SimulationApp({"headless": True})

import omni.usd


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    cfg = json.loads((root / "config/isaac_config.json").read_text(encoding="utf-8"))
    stage_path = root / cfg["project"]["stage"]
    context = omni.usd.get_context()
    if not context.open_stage(str(stage_path)):
        print(f"[FAIL] Could not open stage: {stage_path}")
        return 2
    for _ in range(120):
        APP.update()
    stage = context.get_stage()
    required = [
        "/World/HospitalMap",
        "/World/AMR1",
        "/World/AMR2",
        "/World/AMR1/base_link/front_camera_link/Camera",
        "/World/AMR2/base_link/front_camera_link/Camera",
    ]
    missing = [path for path in required if not stage.GetPrimAtPath(path).IsValid()]
    if missing:
        print("[FAIL] Missing prims:")
        for path in missing:
            print(f"  - {path}")
        return 3
    hospital_map = stage.GetPrimAtPath("/World/HospitalMap")
    map_count = sum(1 for prim in stage.Traverse() if str(prim.GetPath()).startswith("/World/HospitalMap"))
    total_count = sum(1 for _ in stage.Traverse())
    if map_count < 10:
        print(f"[FAIL] Hospital map looks empty: prim_count={map_count}")
        return 4
    print(f"[OK] v4 stage: {stage_path}")
    print(f"[OK] HospitalMap prims: {map_count}")
    print(f"[OK] Total prims: {total_count}")
    print("[RESULT] STAGE PASS")
    return 0


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except Exception as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
    finally:
        APP.close()
    raise SystemExit(code)
