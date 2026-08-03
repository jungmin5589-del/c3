#!/usr/bin/env python3
"""Generate individual importable USD files for AMR1, AMR2 and three beds.

Run with Isaac Sim's python.sh after generate_all.sh.  It copies each root prim,
its bound materials, and required referenced visual/texture assets into output/individual_usd.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import shutil

try:
    from isaacsim.simulation_app import SimulationApp
except ImportError:
    from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--stage", required=True)
parser.add_argument("--output-dir", required=True)
args = parser.parse_args()

app = SimulationApp({"headless": True, "sync_loads": True})
from pxr import Sdf, Usd, UsdGeom

source_path = Path(args.stage).expanduser().resolve()
out_dir = Path(args.output_dir).expanduser().resolve()
out_dir.mkdir(parents=True, exist_ok=True)
source = Usd.Stage.Open(str(source_path))
if source is None:
    raise RuntimeError(f"Unable to open source stage: {source_path}")

assets = [
    ("AMR1", "/World/AMR1", "AMR1.usd"),
    ("AMR2", "/World/AMR2", "AMR2.usd"),
    ("HospitalBed_SeoSuwon", "/World/HospitalBed1", "HospitalBed_SeoSuwon.usd"),
    ("HospitalBed_KimSeoul", "/World/HospitalBed2", "HospitalBed_KimSeoul.usd"),
    ("HospitalBed_ParkIncheon", "/World/HospitalBed3", "HospitalBed_ParkIncheon.usd"),
]

for default_name, source_prim_path, filename in assets:
    prim = source.GetPrimAtPath(source_prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Missing source prim: {source_prim_path}")
    out_path = out_dir / filename
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    dst = stage.DefinePrim(f"/{default_name}", "Xform")
    Sdf.CopySpec(source.GetRootLayer(), prim.GetPath(), stage.GetRootLayer(), dst.GetPath())
    stage.SetDefaultPrim(stage.GetPrimAtPath(f"/{default_name}"))
    stage.GetRootLayer().Save()
    print(f"[asset] {source_prim_path} -> {out_path}")

app.close()
