#!/usr/bin/env python3
"""Reference all five individual assets into the open Isaac Sim stage."""
from pathlib import Path
import omni.usd
from pxr import Gf, UsdGeom

BASE = Path(__file__).resolve().parents[1]
stage = omni.usd.get_context().get_stage()
items = [
    ("/World/AMR1", BASE/"AMR1.usd", (-2.0, -1.25, 0.0)),
    ("/World/AMR2", BASE/"AMR2.usd", (-2.0,  1.25, 0.0)),
    ("/World/HospitalBed1", BASE/"HospitalBed_SeoSuwon.usd", (1.5, -2.0, 0.0)),
    ("/World/HospitalBed2", BASE/"HospitalBed_KimSeoul.usd", (1.5,  0.0, 0.0)),
    ("/World/HospitalBed3", BASE/"HospitalBed_ParkIncheon.usd", (1.5,  2.0, 0.0)),
]
for prim_path, asset, pos in items:
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(str(asset))
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    print("referenced", asset, "->", prim_path)
