#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict

parser = argparse.ArgumentParser()
parser.add_argument("--old", required=True)
parser.add_argument("--new", required=True)
args = parser.parse_args()
old_path = Path(args.old)
new_path = Path(args.new)
if not old_path.is_file() or not new_path.is_file():
    raise SystemExit(0)

try:
    old: Dict[str, Any] = json.loads(old_path.read_text(encoding="utf-8"))
    new: Dict[str, Any] = json.loads(new_path.read_text(encoding="utf-8"))
except Exception:
    raise SystemExit(0)

new_patients = new.get("patients", [])
if isinstance(old.get("patients"), list):
    old_by_id = {str(item.get("id", "")).lower(): item for item in old["patients"] if isinstance(item, dict)}
    for patient in new_patients:
        previous = old_by_id.get(str(patient.get("id", "")).lower())
        if not previous:
            continue
        for key in (
            "source_bed_prim", "mri_bed_prim", "source_mount", "mri_mount",
            "patient_visual", "transfer", "auto_cycle", "ros"
        ):
            if key in previous:
                patient[key] = previous[key]
else:
    # Migrate the original single-patient v1 config into patient1.
    if new_patients:
        patient1 = new_patients[0]
        for key in ("source_bed_prim", "mri_bed_prim", "source_mount", "mri_mount", "patient_visual", "transfer"):
            if key in old:
                patient1[key] = old[key]
        old_auto = old.get("auto_transfer")
        if isinstance(old_auto, dict):
            patient1["auto_cycle"].update({
                "enabled": old_auto.get("enabled", True),
                "enter_radius_m": old_auto.get("trigger_radius_m", 1.25),
                "hold_seconds": old_auto.get("hold_seconds", 1.0),
                "minimum_play_time_sec": old_auto.get("minimum_play_time_sec", 3.0),
                "only_when_timeline_playing": old_auto.get("only_when_timeline_playing", True)
            })

new_path.write_text(json.dumps(new, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print("[INFO] 기존 Prim 경로/배치 설정을 새 3인 설정으로 마이그레이션했습니다.")
