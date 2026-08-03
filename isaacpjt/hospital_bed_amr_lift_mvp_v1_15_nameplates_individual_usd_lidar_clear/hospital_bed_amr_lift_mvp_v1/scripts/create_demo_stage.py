#!/usr/bin/env python3
"""Generate a low-profile hospital-bed lift AMR demo stage for Isaac Sim 5.1.

What this script creates
------------------------
- Four-wheel low-profile AMR with revolute wheel joints.
- Z-axis prismatic lift plate with two support pads.
- Real Isaac Sim Camera prim.
- 2D RTX LiDAR using the IsaacSensorCreateRtxLidar command when available.
- IMU mount link (sensor can be added later in the GUI).
- Optional dynamic hospital-bed proxy with the supplied GLB as its visual.
- Ground plane, physics materials, lighting, and a saved USD stage.

Run only with Isaac Sim's bundled Python 3.11 environment.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
from pathlib import Path
import shutil
import sys
from typing import Any, Dict, Iterable, Sequence

try:
    from isaacsim.simulation_app import SimulationApp
except ImportError:
    from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="JSON configuration file")
    parser.add_argument("--output", required=True, help="Output USD/USDA path")
    parser.add_argument("--bed-glb", default="", help="Optional hospital bed GLB")
    parser.add_argument("--visual-usd", default="", help="Converted bed visual USD path")
    parser.add_argument("--no-bed", action="store_true", help="Create AMR-only stage")
    parser.add_argument("--show-window", action="store_true", help="Show GUI while generating")
    parser.add_argument("--map-usd", default="", help="Optional hospital map USD to reference")
    return parser.parse_args()


ARGS = parse_args()
APP = SimulationApp({"headless": not ARGS.show_window, "sync_loads": True})

# Kit/USD imports must happen after SimulationApp startup.
import carb
import omni
import omni.kit.app
import omni.kit.asset_converter
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade
from omni.physx.scripts import physicsUtils


def read_config(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def add_xform_ops(
    xformable: UsdGeom.Xformable,
    translate: Sequence[float] = (0.0, 0.0, 0.0),
    orient: Gf.Quatf | None = None,
    scale: Sequence[float] | None = None,
) -> None:
    """Add transforms only to newly defined prims.

    This avoids the duplicate xformOp problem that can occur on referenced prims.
    """
    if orient is None:
        orient = Gf.Quatf(1.0)
    xformable.AddTranslateOp().Set(Gf.Vec3f(*map(float, translate)))
    xformable.AddOrientOp().Set(orient)
    if scale is not None:
        xformable.AddScaleOp().Set(Gf.Vec3f(*map(float, scale)))


def quat_from_euler_deg(rx: float, ry: float, rz: float) -> Gf.Quatf:
    rotation = Gf.Rotation(Gf.Vec3d(1, 0, 0), rx)
    rotation *= Gf.Rotation(Gf.Vec3d(0, 1, 0), ry)
    rotation *= Gf.Rotation(Gf.Vec3d(0, 0, 1), rz)
    q = rotation.GetQuat()
    return Gf.Quatf(float(q.GetReal()), Gf.Vec3f(*map(float, q.GetImaginary())))


def make_preview_material(stage: Usd.Stage, path: str, color: Sequence[float], metallic=0.0, roughness=0.5):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*map(float, color)))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material



def make_omnipbr_texture_material(stage: Usd.Stage, path: str, texture_path: Path):
    """Create an OmniPBR MDL material using the supplied image as diffuse texture."""
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture_path)))
    shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 1.0, 1.0))
    shader.CreateInput("roughness_constant", Sdf.ValueTypeNames.Float).Set(0.35)
    shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(False)
    material.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    return material


def add_nameplate_plane(
    stage: Usd.Stage,
    path: str,
    center: Sequence[float],
    width: float,
    height: float,
    material: UsdShade.Material,
) -> UsdGeom.Mesh:
    """Create a true Mesh plane in the YZ plane, facing +X, with centered UVs."""
    half_w = float(width) * 0.5
    half_h = float(height) * 0.5
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([
        Gf.Vec3f(0.0, -half_w, -half_h),
        Gf.Vec3f(0.0,  half_w, -half_h),
        Gf.Vec3f(0.0,  half_w,  half_h),
        Gf.Vec3f(0.0, -half_w,  half_h),
    ])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(1.0, 0.0, 0.0)])
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.constant)
    mesh.CreateDoubleSidedAttr(True)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    st.Set([
        Gf.Vec2f(0.0, 0.0), Gf.Vec2f(1.0, 0.0),
        Gf.Vec2f(1.0, 1.0), Gf.Vec2f(0.0, 1.0),
    ])
    add_xform_ops(mesh, translate=center)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    return mesh

def make_physics_material(stage: Usd.Stage, path: str, static_friction: float, dynamic_friction: float, restitution: float):
    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(float(static_friction))
    api.CreateDynamicFrictionAttr(float(dynamic_friction))
    api.CreateRestitutionAttr(float(restitution))
    return material


def bind_preview(prim: Usd.Prim, material: UsdShade.Material) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def bind_physics(stage: Usd.Stage, prim: Usd.Prim, material: UsdShade.Material) -> None:
    physicsUtils.add_physics_material_to_prim(stage, prim, material.GetPath())


def add_rigid_body(prim: Usd.Prim, mass_kg: float) -> None:
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateRigidBodyEnabledAttr(True)
    rb.CreateKinematicEnabledAttr(False)
    mass = UsdPhysics.MassAPI.Apply(prim)
    mass.CreateMassAttr(float(mass_kg))


def add_cube(
    stage: Usd.Stage,
    path: str,
    parent_body_path: str | None,
    position: Sequence[float],
    size: Sequence[float],
    preview_material: UsdShade.Material,
    physics_material: UsdShade.Material | None,
    collision: bool = True,
    visible: bool = True,
) -> UsdGeom.Cube:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    add_xform_ops(cube, translate=position, scale=size)
    bind_preview(cube.GetPrim(), preview_material)
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        if physics_material is not None:
            bind_physics(stage, cube.GetPrim(), physics_material)
    if not visible:
        cube.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    return cube


def add_cylinder(
    stage: Usd.Stage,
    path: str,
    position: Sequence[float],
    radius: float,
    height: float,
    axis: str,
    preview_material: UsdShade.Material,
    physics_material: UsdShade.Material | None,
    collision: bool = True,
) -> UsdGeom.Cylinder:
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateRadiusAttr(float(radius))
    cylinder.CreateHeightAttr(float(height))
    cylinder.CreateAxisAttr(axis)
    add_xform_ops(cylinder, translate=position)
    bind_preview(cylinder.GetPrim(), preview_material)
    if collision:
        UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
        if physics_material is not None:
            bind_physics(stage, cylinder.GetPrim(), physics_material)
    return cylinder


def create_wheel_body(
    stage: Usd.Stage,
    body_path: str,
    position: Sequence[float],
    radius: float,
    width: float,
    mass_kg: float,
    preview_material: UsdShade.Material,
    physics_material: UsdShade.Material,
) -> UsdGeom.Xform:
    body = UsdGeom.Xform.Define(stage, body_path)
    add_xform_ops(body, translate=position)
    add_rigid_body(body.GetPrim(), mass_kg)
    wheel = UsdGeom.Cylinder.Define(stage, f"{body_path}/CollisionAndVisual")
    wheel.CreateRadiusAttr(float(radius))
    wheel.CreateHeightAttr(float(width))
    wheel.CreateAxisAttr(UsdGeom.Tokens.y)
    bind_preview(wheel.GetPrim(), preview_material)
    UsdPhysics.CollisionAPI.Apply(wheel.GetPrim())
    bind_physics(stage, wheel.GetPrim(), physics_material)

    # Visual diagonal bands indicate intended mecanum/omni-wheel appearance.
    # They are visual only; the stable MVP collision remains a simple cylinder.
    for index, offset in enumerate((-0.015, 0.0, 0.015)):
        roller = UsdGeom.Cube.Define(stage, f"{body_path}/OmniBand_{index}")
        roller.CreateSizeAttr(1.0)
        add_xform_ops(
            roller,
            translate=(0.0, offset, 0.0),
            orient=quat_from_euler_deg(0.0, 45.0 if position[1] > 0 else -45.0, 0.0),
            scale=(radius * 1.35, 0.008, 0.014),
        )
        roller.CreateDisplayColorAttr([Gf.Vec3f(0.65, 0.68, 0.72)])
    return body


def add_revolute_joint(
    stage: Usd.Stage,
    path: str,
    body0: str,
    body1: str,
    local_pos0: Sequence[float],
    local_pos1: Sequence[float],
    damping: float,
    max_force: float,
    radius: float,
    mecanum_angle_rad: float,
) -> UsdPhysics.RevoluteJoint:
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float, local_pos0)))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*map(float, local_pos1)))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateAxisAttr(UsdGeom.Tokens.y)
    joint.CreateCollisionEnabledAttr(False)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(float(damping))
    drive.CreateMaxForceAttr(float(max_force))
    drive.CreateTargetVelocityAttr(0.0)

    # These attributes are consumed by Isaac Sim's Holonomic Controller setup.
    joint.GetPrim().CreateAttribute("isaacmecanumwheel:radius", Sdf.ValueTypeNames.Float).Set(float(radius))
    joint.GetPrim().CreateAttribute("isaacmecanumwheel:angle", Sdf.ValueTypeNames.Float).Set(float(mecanum_angle_rad))
    return joint


def add_prismatic_lift_joint(
    stage: Usd.Stage,
    path: str,
    body0: str,
    body1: str,
    local_pos0: Sequence[float],
    local_pos1: Sequence[float],
    lower: float,
    upper: float,
    stiffness: float,
    damping: float,
    max_force: float,
) -> UsdPhysics.PrismaticJoint:
    joint = UsdPhysics.PrismaticJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float, local_pos0)))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*map(float, local_pos1)))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateAxisAttr(UsdGeom.Tokens.z)
    joint.CreateLowerLimitAttr(float(lower))
    joint.CreateUpperLimitAttr(float(upper))
    joint.CreateCollisionEnabledAttr(False)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(stiffness))
    drive.CreateDampingAttr(float(damping))
    drive.CreateMaxForceAttr(float(max_force))
    drive.CreateTargetPositionAttr(float(lower))
    drive.CreateTargetVelocityAttr(0.0)
    return joint


async def convert_glb_to_usd(input_path: Path, output_path: Path) -> None:
    manager = omni.kit.asset_converter.get_instance()
    context = omni.kit.asset_converter.AssetConverterContext()
    settings = {
        "ignore_materials": False,
        "ignore_animations": True,
        "ignore_camera": True,
        "ignore_light": True,
        "single_mesh": False,
        "smooth_normals": True,
        "export_preview_surface": True,
        "use_meter_as_world_unit": True,
        "create_world_as_default_root_prim": True,
        "embed_textures": True,
        "disabling_instancing": True,
    }
    for name, value in settings.items():
        if hasattr(context, name):
            setattr(context, name, value)

    def progress(current: int, total: int) -> None:
        print(f"[bed convert] {current}/{total}")

    task = manager.create_converter_task(str(input_path), str(output_path), progress, context)
    success = await task.wait_until_finished()
    if not success:
        raise RuntimeError(f"GLB conversion failed: {task.get_status()} {task.get_error_message()}")



def measure_bed_visual(stage: Usd.Stage, bed_visual_usd: Path) -> tuple[Gf.Vec3f, Gf.Vec3f]:
    """Measure the Y-up source once at the origin and return local offset and size."""
    measure_root_path = "/World/__BedMeasure"
    measure_root = UsdGeom.Xform.Define(stage, measure_root_path)
    orientation = UsdGeom.Xform.Define(stage, f"{measure_root_path}/VisualOrientation")
    add_xform_ops(orientation, orient=quat_from_euler_deg(90.0, 0.0, 0.0))
    visual_ref = stage.DefinePrim(f"{measure_root_path}/VisualOrientation/VisualReference", "Xform")
    visual_ref.GetReferences().AddReference(str(bed_visual_usd))

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
    )
    aligned = bbox_cache.ComputeWorldBound(orientation.GetPrim()).ComputeAlignedRange()
    minimum = aligned.GetMin()
    maximum = aligned.GetMax()
    center = (minimum + maximum) * 0.5
    offset = Gf.Vec3f(float(-center[0]), float(-center[1]), float(-minimum[2]))
    size = Gf.Vec3f(
        float(maximum[0] - minimum[0]),
        float(maximum[1] - minimum[1]),
        float(maximum[2] - minimum[2]),
    )
    stage.RemovePrim(measure_root_path)
    print(f"[bed visual local offset] {offset}")
    print(f"[bed visual local size] {size}")
    return offset, size


def build_map_reference(stage: Usd.Stage, cfg: Dict[str, Any], map_usd: Path | None) -> bool:
    if map_usd is None:
        return False
    if not map_usd.exists():
        raise FileNotFoundError(f"Hospital map USD not found: {map_usd}")
    map_cfg = cfg.get("map", {})
    prim_path = str(map_cfg.get("prim_path", "/World/HospitalMap"))
    root = UsdGeom.Xform.Define(stage, prim_path)
    add_xform_ops(
        root,
        translate=map_cfg.get("position", (0.0, 0.0, 0.0)),
        orient=quat_from_euler_deg(*map(float, map_cfg.get("rotation_deg", (0.0, 0.0, 0.0)))),
        scale=map_cfg.get("scale", (1.0, 1.0, 1.0)),
    )
    root.GetPrim().GetReferences().AddReference(str(map_usd))
    root.GetPrim().CreateAttribute("hospitalMap:source", Sdf.ValueTypeNames.Asset, custom=True).Set(str(map_usd))
    print(f"[map] Referenced map: {map_usd} -> {prim_path}")
    return True


def build_amr(
    stage: Usd.Stage,
    shared: Dict[str, Any],
    unit: Dict[str, Any],
    materials: Dict[str, Any],
) -> tuple[str, list[str]]:
    root_path = str(unit["root_path"])
    position = unit.get("start_position", (0.0, 0.0, 0.0))
    yaw_deg = float(unit.get("start_yaw_deg", 0.0))
    namespace = str(unit.get("namespace", unit.get("name", "amr").lower()))

    chassis_mat = make_preview_material(
        stage,
        f"/World/Materials/{unit['name']}Chassis",
        unit.get("color", (0.08, 0.30, 0.62)),
        metallic=0.15,
        roughness=0.45,
    )

    amr_root = UsdGeom.Xform.Define(stage, root_path)
    add_xform_ops(amr_root, translate=position, orient=quat_from_euler_deg(0.0, 0.0, yaw_deg))
    UsdPhysics.ArticulationRootAPI.Apply(amr_root.GetPrim())
    articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(amr_root.GetPrim())
    articulation_api.CreateEnabledSelfCollisionsAttr(False)
    articulation_api.CreateSolverPositionIterationCountAttr(32)
    articulation_api.CreateSolverVelocityIterationCountAttr(4)

    base_path = f"{root_path}/base_link"
    base = UsdGeom.Xform.Define(stage, base_path)
    add_xform_ops(base, translate=(0.0, 0.0, shared["base_center_z"]))
    add_rigid_body(base.GetPrim(), shared["base_mass_kg"])
    PhysxSchema.PhysxRigidBodyAPI.Apply(base.GetPrim()).CreateEnableCCDAttr(True)
    add_cube(
        stage, f"{base_path}/Chassis", base_path, (0.0, 0.0, 0.0),
        shared["base_size"], chassis_mat, materials["ground_phys"], collision=True,
    )

    UsdGeom.Scope.Define(stage, f"{root_path}/Joints")
    wheel_positions = {
        "FL": (shared["wheel_x"], shared["wheel_y"], shared["wheel_center_z"]),
        "FR": (shared["wheel_x"], -shared["wheel_y"], shared["wheel_center_z"]),
        "RL": (-shared["wheel_x"], shared["wheel_y"], shared["wheel_center_z"]),
        "RR": (-shared["wheel_x"], -shared["wheel_y"], shared["wheel_center_z"]),
    }
    mecanum_sign = {"FL": 1.0, "FR": -1.0, "RL": -1.0, "RR": 1.0}
    for name, pos in wheel_positions.items():
        body_path = f"{root_path}/wheel_{name}"
        create_wheel_body(
            stage, body_path, pos, shared["wheel_radius"], shared["wheel_width"],
            shared["wheel_mass_kg"], materials["dark"], materials["wheel_phys"],
        )
        local0 = (pos[0], pos[1], pos[2] - shared["base_center_z"])
        add_revolute_joint(
            stage, f"{root_path}/Joints/wheel_joint_{name}", base_path, body_path,
            local0, (0.0, 0.0, 0.0), shared["wheel_drive_damping"],
            shared["wheel_drive_max_force"], shared["wheel_radius"],
            mecanum_sign[name] * math.radians(45.0),
        )

    lift_path = f"{root_path}/lift_plate"
    lift = UsdGeom.Xform.Define(stage, lift_path)
    add_xform_ops(lift, translate=(0.0, 0.0, shared["lift_plate_center_z"]))
    add_rigid_body(lift.GetPrim(), shared["lift_plate_mass_kg"])
    PhysxSchema.PhysxRigidBodyAPI.Apply(lift.GetPrim()).CreateEnableCCDAttr(True)
    add_cube(
        stage, f"{lift_path}/Plate", lift_path, (0.0, 0.0, 0.0),
        shared["lift_plate_size"], materials["lift_mat"], materials["lift_phys"], collision=True,
    )
    for side, y in (("Left", shared["support_pad_y"]), ("Right", -shared["support_pad_y"])):
        add_cube(
            stage, f"{lift_path}/SupportPad{side}", lift_path,
            (0.0, y, shared["support_pad_center_z"]), shared["support_pad_size"],
            materials["dark"], materials["lift_phys"], collision=True,
        )
    add_prismatic_lift_joint(
        stage, f"{root_path}/Joints/lift_joint", base_path, lift_path,
        (0.0, 0.0, shared["lift_plate_center_z"] - shared["base_center_z"]),
        (0.0, 0.0, 0.0), shared["lift_lower_limit_m"], shared["lift_upper_limit_m"],
        shared["lift_stiffness"], shared["lift_damping"], shared["lift_max_force_n"],
    )

    # One centered rotary RTX LiDAR per AMR.  The raised cyan pedestal keeps the
    # sensor easy to see in the viewport and places the scan plane above the lift
    # rails while remaining below the hospital-bed mattress/frame.
    lidar_mount_size = shared.get("lidar_mount_size", (0.065, 0.065, 0.08))
    add_cube(
        stage,
        f"{base_path}/LidarPedestal",
        base_path,
        (0.0, 0.0, float(shared.get("lidar_mount_center_z", 0.165)) - shared["base_center_z"]),
        lidar_mount_size,
        materials["sensor_mat"],
        None,
        collision=False,
        visible=True,
    )
    lidar_link = UsdGeom.Xform.Define(stage, f"{base_path}/lidar_link")
    add_xform_ops(
        lidar_link,
        translate=(
            shared["lidar_position"][0], shared["lidar_position"][1],
            shared["lidar_position"][2] - shared["base_center_z"],
        ),
    )
    add_cylinder(
        stage,
        f"{base_path}/lidar_link/LidarHousing",
        (0.0, 0.0, float(shared.get("lidar_housing_center_offset_z_m", -0.0425))),
        float(shared.get("lidar_housing_radius_m", 0.055)),
        float(shared.get("lidar_housing_height_m", 0.025)),
        UsdGeom.Tokens.z,
        materials["sensor_mat"],
        None,
        collision=False,
    )

    def add_rgbd_camera(name: str, position_key: str, pitch_key: str, yaw_key: str) -> str:
        link_path = f"{base_path}/{name}_camera_link"
        link = UsdGeom.Xform.Define(stage, link_path)
        position = shared[position_key]
        add_xform_ops(
            link,
            translate=(
                position[0], position[1], position[2] - shared["base_center_z"],
            ),
            orient=quat_from_euler_deg(
                0.0,
                -float(shared.get(pitch_key, 5.0)),
                float(shared.get(yaw_key, 0.0)),
            ),
        )
        add_cube(
            stage,
            f"{link_path}/RealSenseVisual",
            None,
            (0, 0, 0),
            (0.09, 0.025, 0.025),
            materials["dark"],
            None,
            collision=False,
        )
        camera = UsdGeom.Camera.Define(stage, f"{link_path}/Camera")
        camera.CreateFocalLengthAttr(18.0)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 20.0))
        add_xform_ops(
            camera,
            translate=(0.046, 0.0, 0.0),
            orient=quat_from_euler_deg(0.0, -90.0, 0.0),
        )
        return f"{link_path}/Camera"

    front_camera_path = add_rgbd_camera(
        "front",
        "front_camera_position",
        "front_camera_pitch_deg",
        "front_camera_yaw_deg",
    )
    rear_camera_path = add_rgbd_camera(
        "rear",
        "rear_camera_position",
        "rear_camera_pitch_deg",
        "rear_camera_yaw_deg",
    )

    imu_link = UsdGeom.Xform.Define(stage, f"{base_path}/imu_link")
    add_xform_ops(
        imu_link,
        translate=(
            shared["imu_position"][0], shared["imu_position"][1],
            shared["imu_position"][2] - shared["base_center_z"],
        ),
    )
    add_cube(
        stage, f"{base_path}/imu_link/ImuVisual", None, (0, 0, 0),
        (0.04, 0.03, 0.015), materials["sensor_mat"], None, collision=False,
    )

    amr_root.GetPrim().CreateAttribute("hospitalBedAMR:version", Sdf.ValueTypeNames.String, custom=True).Set("mvp_v1.15")
    amr_root.GetPrim().CreateAttribute("hospitalBedAMR:namespace", Sdf.ValueTypeNames.String, custom=True).Set(namespace)
    amr_root.GetPrim().CreateAttribute("hospitalBedAMR:liftTargetMeters", Sdf.ValueTypeNames.Float, custom=True).Set(float(shared["lift_test_target_m"]))
    wheel_rel = amr_root.GetPrim().CreateRelationship("hospitalBedAMR:wheelJoints", custom=True)
    wheel_rel.SetTargets([Sdf.Path(f"{root_path}/Joints/wheel_joint_{n}") for n in ("FL", "FR", "RL", "RR")])
    lift_rel = amr_root.GetPrim().CreateRelationship("hospitalBedAMR:liftJoint", custom=True)
    lift_rel.SetTargets([Sdf.Path(f"{root_path}/Joints/lift_joint")])

    lidar_path = f"{base_path}/lidar_link/RtxLidar"
    required = [
        base_path,
        lift_path,
        f"{root_path}/Joints/lift_joint",
        front_camera_path,
        rear_camera_path,
    ] + [f"{root_path}/Joints/wheel_joint_{n}" for n in ("FL", "FR", "RL", "RR")]
    print(f"[amr] {unit['name']} root={root_path} namespace={namespace}")
    return lidar_path, required


def build_bed(
    stage: Usd.Stage,
    bed_cfg: Dict[str, Any],
    instance: Dict[str, Any],
    bed_visual_usd: Path,
    visual_offset_value: Gf.Vec3f,
    materials: Dict[str, Any],
) -> list[str]:
    root_path = str(instance["root_path"])
    root = UsdGeom.Xform.Define(stage, root_path)
    add_xform_ops(
        root,
        translate=instance.get("position", (0.0, 0.0, 0.0)),
        orient=quat_from_euler_deg(0.0, 0.0, float(instance.get("yaw_deg", 0.0))),
    )
    add_rigid_body(root.GetPrim(), bed_cfg["mass_kg"])
    PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim()).CreateEnableCCDAttr(True)

    visual_offset = UsdGeom.Xform.Define(stage, f"{root_path}/VisualOffset")
    add_xform_ops(visual_offset, translate=visual_offset_value)
    visual_orientation = UsdGeom.Xform.Define(stage, f"{root_path}/VisualOffset/VisualOrientation")
    add_xform_ops(visual_orientation, orient=quat_from_euler_deg(90.0, 0.0, 0.0))
    visual_ref = stage.DefinePrim(f"{root_path}/VisualOffset/VisualOrientation/VisualReference", "Xform")
    visual_ref.GetReferences().AddReference(str(bed_visual_usd))

    add_cube(stage, f"{root_path}/Colliders/Mattress", root_path, (0, 0, bed_cfg["mattress_center_z"]), bed_cfg["mattress_size"], materials["bed_proxy_mat"], materials["bed_phys"], collision=True, visible=False)
    for side, y in (("Left", bed_cfg["side_frame_y"]), ("Right", -bed_cfg["side_frame_y"])):
        add_cube(stage, f"{root_path}/Colliders/SideFrame{side}", root_path, (0, y, bed_cfg["side_frame_center_z"]), bed_cfg["side_frame_size"], materials["bed_proxy_mat"], materials["bed_phys"], collision=True, visible=False)
    if bool(bed_cfg.get("end_frame_open_center_enabled", False)):
        for side, y in (("Left", bed_cfg["end_frame_post_y"]), ("Right", -bed_cfg["end_frame_post_y"])):
            add_cube(
                stage,
                f"{root_path}/Colliders/HeadFramePost{side}",
                root_path,
                (bed_cfg["end_frame_x"], y, bed_cfg["end_frame_center_z"]),
                bed_cfg["end_frame_post_size"],
                materials["bed_proxy_mat"],
                materials["bed_phys"],
                collision=True,
                visible=False,
            )
        add_cube(
            stage,
            f"{root_path}/Colliders/HeadFrameTop",
            root_path,
            (bed_cfg["end_frame_x"], 0.0, bed_cfg["end_frame_top_center_z"]),
            bed_cfg["end_frame_top_size"],
            materials["bed_proxy_mat"],
            materials["bed_phys"],
            collision=True,
            visible=False,
        )
    else:
        add_cube(stage, f"{root_path}/Colliders/HeadFrame", root_path, (bed_cfg["end_frame_x"], 0.0, bed_cfg["end_frame_center_z"]), bed_cfg["end_frame_size"], materials["bed_proxy_mat"], materials["bed_phys"], collision=True, visible=False)

    corner_labels = {(-1, -1): "XN_YN", (-1, 1): "XN_YP", (1, -1): "XP_YN", (1, 1): "XP_YP"}
    for sx in (-1, 1):
        for sy in (-1, 1):
            corner = corner_labels[(sx, sy)]
            add_cube(stage, f"{root_path}/Colliders/Leg_{corner}", root_path, (sx * bed_cfg["leg_x"], sy * bed_cfg["leg_y"], bed_cfg["leg_center_z"]), bed_cfg["leg_size"], materials["bed_proxy_mat"], materials["bed_phys"], collision=True, visible=False)
            add_cylinder(stage, f"{root_path}/Colliders/WheelProxy_{corner}", (sx * bed_cfg["leg_x"], sy * bed_cfg["leg_y"], bed_cfg["wheel_proxy_center_z"]), bed_cfg["wheel_proxy_radius"], bed_cfg["wheel_proxy_width"], UsdGeom.Tokens.y, materials["dark"], materials["caster_phys"], collision=True)

    adapter_path = f"{root_path}/LiftAdapter"
    UsdGeom.Xform.Define(stage, adapter_path)
    for side, y in (("Left", bed_cfg["support_rail_y"]), ("Right", -bed_cfg["support_rail_y"])):
        add_cube(stage, f"{adapter_path}/LiftRail{side}", root_path, (0.0, y, bed_cfg["support_rail_height_m"]), bed_cfg["support_rail_size"], materials["adapter_mat"], materials["bed_phys"], collision=True, visible=True)

    hanger_x = float(bed_cfg["adapter_hanger_x"])
    hanger_y = float(bed_cfg["support_rail_y"])
    for x_label, x in (("Front", hanger_x), ("Rear", -hanger_x)):
        for side, y in (("Left", hanger_y), ("Right", -hanger_y)):
            add_cube(stage, f"{adapter_path}/Hanger{x_label}{side}", root_path, (x, y, bed_cfg["adapter_hanger_center_z"]), bed_cfg["adapter_hanger_size"], materials["adapter_mat"], None, collision=False, visible=True)
        add_cube(stage, f"{adapter_path}/UpperBridge{x_label}", root_path, (x, 0.0, bed_cfg["adapter_bridge_center_z"]), bed_cfg["adapter_bridge_size"], materials["adapter_mat"], None, collision=False, visible=True)

        # Visible structural member: spans from the central adapter bridge to both bed side frames.
        # It is intentionally non-colliding so the AMR entry space and docking physics stay unchanged.
        add_cube(
            stage,
            f"{adapter_path}/ConnectionBeam{x_label}",
            root_path,
            (x, 0.0, bed_cfg["adapter_connection_beam_center_z"]),
            bed_cfg["adapter_connection_beam_size"],
            materials["adapter_mat"],
            None,
            collision=False,
            visible=True,
        )

        # Four end plates overlap the configured side-frame region, making the adapter look
        # welded/bolted to the bed rather than floating below it.
        for side, y in (("Left", bed_cfg["adapter_mount_plate_y"]), ("Right", -bed_cfg["adapter_mount_plate_y"])):
            add_cube(
                stage,
                f"{adapter_path}/MountPlate{x_label}{side}",
                root_path,
                (x, y, bed_cfg["adapter_mount_plate_center_z"]),
                bed_cfg["adapter_mount_plate_size"],
                materials["adapter_accent_mat"],
                None,
                collision=False,
                visible=True,
            )

    marker_size = bed_cfg["adapter_contact_marker_size"]
    marker_z = float(bed_cfg["support_rail_height_m"]) - float(bed_cfg["support_rail_size"][2]) * 0.5 - float(marker_size[2]) * 0.5
    for side, y in (("Left", hanger_y), ("Right", -hanger_y)):
        add_cube(stage, f"{adapter_path}/ContactMarker{side}", root_path, (0.0, y, marker_z), marker_size, materials["adapter_accent_mat"], None, collision=False, visible=True)

    if bool(bed_cfg.get("nameplate_enabled", False)) and instance.get("nameplate_texture"):
        texture_path = Path(str(instance["nameplate_texture"])).expanduser().resolve()
        if not texture_path.exists():
            raise FileNotFoundError(f"Nameplate texture not found: {texture_path}")
        safe_name = str(instance.get("name", "Bed")).replace(" ", "_")
        nameplate_material = make_omnipbr_texture_material(
            stage,
            f"/World/Materials/{safe_name}NameplateOmniPBR",
            texture_path,
        )
        center_x = float(bed_cfg.get("nameplate_center_x_m", 1.015))
        if str(bed_cfg.get("nameplate_end", "positive_x")) == "negative_x":
            center_x = -abs(center_x)
        else:
            center_x = abs(center_x)
        add_nameplate_plane(
            stage,
            f"{root_path}/Nameplate/Plane",
            (
                center_x,
                float(bed_cfg.get("nameplate_center_y_m", 0.0)),
                float(bed_cfg.get("nameplate_center_z_m", 0.67)),
            ),
            float(bed_cfg.get("nameplate_width_m", 0.62)),
            float(bed_cfg.get("nameplate_height_m", 0.43)),
            nameplate_material,
        )
        root.GetPrim().CreateAttribute("hospitalBed:patientName", Sdf.ValueTypeNames.String, custom=True).Set(str(instance.get("patient_name", "")))
        root.GetPrim().CreateAttribute("hospitalBed:patientBirthDate", Sdf.ValueTypeNames.String, custom=True).Set(str(instance.get("patient_birth_date", "")))
        root.GetPrim().CreateAttribute("hospitalBed:nameplateTexture", Sdf.ValueTypeNames.Asset, custom=True).Set(Sdf.AssetPath(str(texture_path)))

    root.GetPrim().CreateAttribute("hospitalBed:name", Sdf.ValueTypeNames.String, custom=True).Set(str(instance.get("name", root_path.rsplit('/', 1)[-1])))
    print(f"[bed] {instance.get('name')} at {instance.get('position')}")
    return [root_path, f"{adapter_path}/LiftRailLeft", f"{adapter_path}/LiftRailRight"]


def build_stage(cfg: Dict[str, Any], output: Path, bed_visual_usd: Path | None, map_usd: Path | None) -> None:
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, float(cfg["stage"]["meters_per_unit"]))
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(abs(float(cfg["stage"]["gravity"])))
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
    physx_scene.CreateTimeStepsPerSecondAttr(int(cfg["stage"]["physics_steps_per_second"]))
    physx_scene.CreateEnableCCDAttr(True)
    physx_scene.CreateEnableEnhancedDeterminismAttr(True)

    UsdGeom.Scope.Define(stage, "/World/Materials")
    mat_cfg = cfg["materials"]
    materials = {
        "ground_phys": make_physics_material(stage, "/World/Materials/GroundPhysics", mat_cfg["ground_static_friction"], mat_cfg["ground_dynamic_friction"], mat_cfg["restitution"]),
        "wheel_phys": make_physics_material(stage, "/World/Materials/WheelPhysics", mat_cfg["wheel_static_friction"], mat_cfg["wheel_dynamic_friction"], mat_cfg["restitution"]),
        "lift_phys": make_physics_material(stage, "/World/Materials/LiftPhysics", mat_cfg["lift_static_friction"], mat_cfg["lift_dynamic_friction"], mat_cfg["restitution"]),
        "bed_phys": make_physics_material(stage, "/World/Materials/BedPhysics", 0.9, 0.75, 0.0),
        # Caster proxies are not articulated wheels.  A low-friction contact material
        # approximates rolling resistance so wheel-tow mode does not drag the full
        # bed friction across the floor.
        "caster_phys": make_physics_material(
            stage,
            "/World/Materials/BedCasterPhysics",
            float(mat_cfg.get("bed_caster_static_friction", 0.12)),
            float(mat_cfg.get("bed_caster_dynamic_friction", 0.08)),
            mat_cfg["restitution"],
        ),
        "dark": make_preview_material(stage, "/World/Materials/Dark", (0.06, 0.07, 0.09), metallic=0.35, roughness=0.4),
        "lift_mat": make_preview_material(stage, "/World/Materials/Lift", (0.95, 0.56, 0.08), metallic=0.05, roughness=0.55),
        "adapter_mat": make_preview_material(stage, "/World/Materials/BedLiftAdapter", (0.18, 0.22, 0.28), metallic=0.72, roughness=0.28),
        "adapter_accent_mat": make_preview_material(stage, "/World/Materials/BedLiftAdapterAccent", (0.12, 0.55, 0.72), metallic=0.28, roughness=0.38),
        "sensor_mat": make_preview_material(stage, "/World/Materials/Sensor", (0.05, 0.65, 0.72), metallic=0.1, roughness=0.4),
        "bed_proxy_mat": make_preview_material(stage, "/World/Materials/BedProxy", (0.70, 0.80, 0.90), metallic=0.0, roughness=0.75),
    }

    map_present = build_map_reference(stage, cfg, map_usd)
    map_cfg = cfg.get("map", {})
    add_ground = bool(map_cfg.get("use_ground_plane_when_map_present", False) if map_present else map_cfg.get("use_ground_plane_when_empty", True))
    if add_ground:
        physicsUtils.add_ground_plane(stage, "/World/GroundPlane", "Z", 30.0, Gf.Vec3f(0.0), Gf.Vec3f(0.18, 0.18, 0.20))
        ground_prim = stage.GetPrimAtPath("/World/GroundPlane/CollisionPlane")
        if not ground_prim.IsValid():
            ground_prim = stage.GetPrimAtPath("/World/GroundPlane")
        if ground_prim.IsValid():
            bind_physics(stage, ground_prim, materials["ground_phys"])

    light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    light.CreateIntensityAttr(850.0)
    light.CreateColorAttr(Gf.Vec3f(0.95, 0.97, 1.0))

    lidar_paths: list[str] = []
    required: list[str] = []
    for unit in cfg["fleet"]:
        lidar_path, unit_required = build_amr(stage, cfg["amr"], unit, materials)
        if bool(unit.get("lidar_enabled", True)):
            lidar_paths.append(lidar_path)
            print(f"[sensor] RTX LiDAR deferred: {lidar_path}")
        required.extend(unit_required)

    if bed_visual_usd is not None and cfg.get("bed", {}).get("enabled", True):
        visual_offset_value, _ = measure_bed_visual(stage, bed_visual_usd)
        for bed_instance in cfg.get("beds", []):
            required.extend(build_bed(stage, cfg["bed"], bed_instance, bed_visual_usd, visual_offset_value, materials))

    missing = [path for path in required if not stage.GetPrimAtPath(path).IsValid()]
    if missing:
        raise RuntimeError(f"Required prims missing: {missing}")
    stage.GetRootLayer().Save()

    context = omni.usd.get_context()
    if not context.open_stage(str(output)):
        raise RuntimeError(f"Could not reopen generated stage: {output}")
    for _ in range(60):
        APP.update()
    active_stage = context.get_stage()

    for lidar_path in lidar_paths:
        existing = active_stage.GetPrimAtPath(lidar_path)
        if existing and existing.IsValid():
            sensor_prim = existing
        else:
            parent = str(Sdf.Path(lidar_path).GetParentPath())
            name = Sdf.Path(lidar_path).name
            success, sensor_prim = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                translation=Gf.Vec3d(0.0, 0.0, 0.0),
                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
                path=name,
                parent=parent,
                config=str(cfg["amr"].get("lidar_config", "Example_Rotary_2D")),
                visiblity=False,
                variant=None,
                force_camera_prim=False,
                **{"omni:sensor:Core:scanRateBaseHz": 10},
            )
            if not success or sensor_prim is None:
                raise RuntimeError(f"Failed to create RTX LiDAR: {lidar_path}")
        actual = str(sensor_prim.GetPath())
        if actual != lidar_path or not active_stage.GetPrimAtPath(actual).IsValid():
            raise RuntimeError(f"RTX LiDAR path invalid: actual={actual}, expected={lidar_path}")
        print(f"[sensor] RTX LiDAR verified: {actual}")

    active_stage.GetRootLayer().Save()
    print(f"[done] Saved multi-AMR map-ready stage: {output}")


def main() -> int:
    config_path = Path(ARGS.config).expanduser().resolve()
    output_path = Path(ARGS.output).expanduser().resolve()
    bed_glb = Path(ARGS.bed_glb).expanduser().resolve() if ARGS.bed_glb else None
    visual_usd = Path(ARGS.visual_usd).expanduser().resolve() if ARGS.visual_usd else output_path.with_name(output_path.stem + "_bed_visual.usd")
    ensure_parent(output_path)
    cfg = read_config(config_path)
    project_dir = config_path.parent.parent
    for bed_instance in cfg.get("beds", []):
        texture = bed_instance.get("nameplate_texture")
        if texture:
            texture_path = Path(str(texture)).expanduser()
            if not texture_path.is_absolute():
                texture_path = project_dir / texture_path
            bed_instance["nameplate_texture"] = str(texture_path.resolve())

    map_value = ARGS.map_usd or str(cfg.get("map", {}).get("usd_path", ""))
    map_input = Path(map_value).expanduser().resolve() if map_value else None
    map_usd: Path | None = None
    if map_input is not None:
        if not map_input.exists():
            raise FileNotFoundError(f"Hospital map file not found: {map_input}")
        if map_input.suffix.lower() in {".glb", ".gltf"}:
            map_usd = output_path.with_name(output_path.stem + "_map_converted.usd")
            print(f"[map convert] {map_input} -> {map_usd}")
            asyncio.get_event_loop().run_until_complete(convert_glb_to_usd(map_input, map_usd))
        else:
            map_usd = map_input

    bed_visual: Path | None = None
    if not ARGS.no_bed and cfg.get("bed", {}).get("enabled", True):
        if bed_glb is None or not bed_glb.exists():
            raise FileNotFoundError(f"Bed GLB not found: {bed_glb}")
        ensure_parent(visual_usd)
        print(f"[input bed] {bed_glb}")
        print(f"[visual usd] {visual_usd}")
        asyncio.get_event_loop().run_until_complete(convert_glb_to_usd(bed_glb, visual_usd))
        bed_visual = visual_usd

    print(f"[output] {output_path}")
    if map_input:
        print(f"[input map] {map_input}")
    build_stage(cfg, output_path, bed_visual, map_usd)
    return 0


if __name__ == "__main__":
    exit_code = 1
    try:
        exit_code = main()
    except Exception as exc:
        try:
            partial_output = Path(ARGS.output).expanduser().resolve()
            if partial_output.exists():
                partial_output.unlink()
                print(f"[cleanup] Removed partial output: {partial_output}", file=sys.stderr)
        except Exception as cleanup_exc:
            print(f"[cleanup warning] {cleanup_exc}", file=sys.stderr)
        carb.log_error(str(exc))
        print(f"[error] {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        APP.close()
    raise SystemExit(exit_code)
