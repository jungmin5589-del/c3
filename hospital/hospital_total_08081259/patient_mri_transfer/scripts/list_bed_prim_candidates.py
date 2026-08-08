#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

try:
    from isaacsim.simulation_app import SimulationApp
except ImportError:
    from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="List transport-bed and MRI-table Prim candidates")
parser.add_argument("--stage", required=True, help="Existing absolute USD stage path")
args = parser.parse_args()
stage_file = Path(args.stage).expanduser().resolve()
if not stage_file.is_file():
    raise SystemExit(f"[ERROR] USD Stage file does not exist: {stage_file}")

app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom

context = omni.usd.get_context()
if not context.open_stage(str(stage_file)):
    app.close()
    raise SystemExit(f"[ERROR] Could not open USD Stage: {stage_file}")
for _ in range(120):
    app.update()
stage = context.get_stage()
if stage is None:
    app.close()
    raise SystemExit(f"[ERROR] Opened context has no Stage: {stage_file}")

print(f"=== STAGE: {stage_file} ===")
print("=== BED / MRI / SCANNER / TABLE XFORM CANDIDATES ===")
count = 0
for prim in stage.Traverse():
    if not prim.IsValid() or prim.IsInstanceProxy() or prim.GetTypeName() != "Xform":
        continue
    path = str(prim.GetPath())
    text = (path + " " + prim.GetName()).lower()
    if any(word in text for word in ("bed", "mri", "scanner", "gantry", "magnetom", "table")):
        matrix = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
        p = matrix.ExtractTranslation()
        print(f"{path:110s} world=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
        count += 1
print(f"=== {count} candidate(s) ===")
app.close()
