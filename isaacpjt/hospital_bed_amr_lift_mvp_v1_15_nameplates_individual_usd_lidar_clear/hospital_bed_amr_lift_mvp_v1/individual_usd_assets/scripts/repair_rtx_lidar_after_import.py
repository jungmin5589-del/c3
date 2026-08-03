#!/usr/bin/env python3
"""Replace/import the RTX Lidar prims with official Example_Rotary_2D sensors.
Run inside Isaac Sim Script Editor or with python.sh while a stage is open.
"""
import omni
import omni.usd
from pxr import Gf

stage = omni.usd.get_context().get_stage()
for root in ("/World/AMR1", "/World/AMR2"):
    parent = f"{root}/base_link/lidar_link"
    sensor_path = f"{parent}/RtxLidar"
    if stage.GetPrimAtPath(sensor_path).IsValid():
        stage.RemovePrim(sensor_path)
    ok, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path="RtxLidar",
        parent=parent,
        config="Example_Rotary_2D",
        translation=Gf.Vec3d(0, 0, 0),
        orientation=Gf.Quatd(1, 0, 0, 0),
        visibility=False,
        force_camera_prim=False,
    )
    print(root, ok, sensor)
