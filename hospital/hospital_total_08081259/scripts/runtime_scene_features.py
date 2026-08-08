#!/usr/bin/env python3
"""Selected runtime scene features for the hospital AMR project.

Nav2, cmd_vel, AMR drive parameters, costmaps and the elevator motion logic are
left untouched.  The only floor edit in this module is the explicitly requested
1F-friction -> 2F HospitalMap/Cube match plus removal of the duplicate
/World/GroundPlane/CollisionMesh collider.
"""
from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any, Callable

import omni.kit.app
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


def _stage_units_per_meter(stage: Usd.Stage) -> float:
    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    return 1.0 / meters_per_unit if meters_per_unit > 1.0e-9 else 1.0



def _numeric_attr(prim: Usd.Prim | None, name: str) -> float | None:
    if prim is None or not prim.IsValid():
        return None
    try:
        attr = prim.GetAttribute(name)
        if attr and attr.IsValid():
            value = attr.Get()
            if value is not None:
                return float(value)
    except Exception:
        pass
    return None


def _bound_material_prim(stage: Usd.Stage, prim: Usd.Prim) -> Usd.Prim | None:
    """Resolve the nearest direct physics/all-purpose material binding."""
    current = prim
    while current and current.IsValid():
        for rel_name in ("material:binding:physics", "material:binding"):
            try:
                rel = current.GetRelationship(rel_name)
                if rel and rel.IsValid():
                    targets = rel.GetTargets()
                    if targets:
                        material = stage.GetPrimAtPath(targets[0])
                        if material and material.IsValid():
                            return material
            except Exception:
                pass
        if str(current.GetPath()) == "/World":
            break
        current = current.GetParent()
    try:
        result = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        material = result[0] if isinstance(result, tuple) else result
        if material and material.GetPrim().IsValid():
            return material.GetPrim()
    except Exception:
        pass
    return None


def _friction_from_prim(stage: Usd.Stage, prim: Usd.Prim) -> tuple[float | None, float | None, str]:
    material = _bound_material_prim(stage, prim)
    for candidate, label in ((material, "material"), (prim, "prim")):
        static = _numeric_attr(candidate, "physics:staticFriction")
        dynamic = _numeric_attr(candidate, "physics:dynamicFriction")
        if static is not None or dynamic is not None:
            return static, dynamic, (str(candidate.GetPath()) if candidate else label)
    return None, None, ""


def _bbox_world(cache: UsdGeom.BBoxCache, prim: Usd.Prim):
    try:
        box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
        return box.GetMin(), box.GetMax()
    except Exception:
        return None


def apply_requested_floor_fix(
    stage: Usd.Stage,
    cfg: dict[str, Any],
    elevator_cfg: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Apply only the requested floor changes.

    - Detect a static, horizontal 1F floor collider at one lift-height below the
      known 2F /World/HospitalMap/Cube surface.
    - Copy its static/dynamic friction onto a dedicated physics material bound
      only to /World/HospitalMap/Cube.
    - Deactivate /World/GroundPlane/CollisionMesh in the runtime session layer.

    No drive/Nav2/local-costmap/elevator motion parameters are modified here.
    """
    cfg = dict(cfg or {})
    if not bool(cfg.get("enabled", False)):
        return {"enabled": False}

    floor2_path = str(cfg.get("floor2_path", "/World/HospitalMap/Cube"))
    remove_path = str(cfg.get("remove_collision_path", "/World/GroundPlane/CollisionMesh"))
    floor2 = stage.GetPrimAtPath(floor2_path)
    if not floor2 or not floor2.IsValid():
        print(f"[REQUESTED FLOOR WARNING] 2F floor not found: {floor2_path}")
        return {"enabled": True, "applied": False}

    collision_attr = floor2.GetAttribute("physics:collisionEnabled")
    collision_enabled = True
    if collision_attr and collision_attr.IsValid():
        value = collision_attr.Get()
        if value is not None:
            collision_enabled = bool(value)
    has_collision_api = bool(floor2.HasAPI(UsdPhysics.CollisionAPI))
    has_rigid_body = bool(floor2.HasAPI(UsdPhysics.RigidBodyAPI))
    print(
        f"[2F FLOOR CHECK] path={floor2_path} CollisionAPI={has_collision_api} "
        f"collisionEnabled={collision_enabled} RigidBodyAPI={has_rigid_body}"
    )

    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    floor2_bounds = _bbox_world(cache, floor2)
    if floor2_bounds is None:
        print(f"[REQUESTED FLOOR WARNING] cannot read 2F floor bounds: {floor2_path}")
        return {"enabled": True, "applied": False}

    m_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    if m_per_unit <= 1.0e-9:
        m_per_unit = 1.0
    units_per_m = 1.0 / m_per_unit
    _, floor2_max = floor2_bounds
    floor2_top = float(floor2_max[2])
    lift_m = float((elevator_cfg or {}).get("lift_distance_m", 11.325))
    floor1_expected_top = floor2_top - lift_m * units_per_m
    tolerance = float(cfg.get("level_tolerance_m", 0.25)) * units_per_m
    min_span = float(cfg.get("min_floor_span_m", 1.0)) * units_per_m
    min_area = float(cfg.get("min_floor_area_m2", 4.0)) * units_per_m * units_per_m
    max_thickness = float(cfg.get("max_floor_thickness_m", 0.65)) * units_per_m

    excluded = (
        "/World/AMR", "/World/Bed", "/World/HospitalBed",
        "/World/ParkIncheon", "/World/SeoSuwon",
        "/World/RuntimeElevator", "/World/HospitalRuntime",
    )
    candidates: list[tuple[float, Usd.Prim, float, float, float, float]] = []
    for prim in stage.Traverse():
        if not prim.IsActive() or not prim.IsLoaded() or prim == floor2:
            continue
        path = str(prim.GetPath())
        if path == remove_path or any(path.startswith(prefix) for prefix in excluded):
            continue
        try:
            collision = prim.HasAPI(UsdPhysics.CollisionAPI)
            attr = prim.GetAttribute("physics:collisionEnabled")
            if not collision and not (attr and attr.IsValid()):
                continue
            if attr and attr.IsValid() and attr.Get() is False:
                continue
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
        except Exception:
            continue
        bounds = _bbox_world(cache, prim)
        if bounds is None:
            continue
        minimum, maximum = bounds
        sx = max(0.0, float(maximum[0] - minimum[0]))
        sy = max(0.0, float(maximum[1] - minimum[1]))
        sz = max(0.0, float(maximum[2] - minimum[2]))
        top = float(maximum[2])
        area = sx * sy
        if abs(top - floor1_expected_top) > tolerance:
            continue
        if min(sx, sy) < min_span or area < min_area or sz > max_thickness:
            continue
        candidates.append((area, prim, top, sx, sy, sz))

    candidates.sort(key=lambda item: item[0], reverse=True)
    floor1 = candidates[0][1] if candidates else None
    static_friction = dynamic_friction = None
    friction_source = ""
    if floor1 is not None:
        static_friction, dynamic_friction, friction_source = _friction_from_prim(stage, floor1)
        print(
            f"[1F FLOOR REFERENCE] path={floor1.GetPath()} "
            f"topZ={candidates[0][2]:.4f} area={candidates[0][0] * m_per_unit * m_per_unit:.2f}m2"
        )
    else:
        print(
            f"[REQUESTED FLOOR WARNING] no static 1F floor candidate near "
            f"topZ={floor1_expected_top:.4f}"
        )

    # Fallbacks are used only if the stage does not author friction values on the
    # detected 1F reference/material. They match the stable-floor values from the
    # supplied upgrade project and are isolated to the 2F Cube only.
    if static_friction is None:
        static_friction = float(cfg.get("fallback_static_friction", 0.95))
        friction_source = friction_source or "configured fallback"
    if dynamic_friction is None:
        dynamic_friction = float(cfg.get("fallback_dynamic_friction", 0.85))
        friction_source = friction_source or "configured fallback"

    previous_target = stage.GetEditTarget()
    try:
        stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))

        material_path = Sdf.Path("/World/HospitalRuntimeMaterials/Floor2Match1FPhysics")
        UsdGeom.Xform.Define(stage, material_path.GetParentPath())
        material = UsdShade.Material.Define(stage, material_path)
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr(float(static_friction)).Set(float(static_friction))
        physics_material.CreateDynamicFrictionAttr(float(dynamic_friction)).Set(float(dynamic_friction))
        binding = UsdShade.MaterialBindingAPI.Apply(floor2)
        try:
            binding.Bind(material, materialPurpose="physics")
        except Exception:
            try:
                binding.Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
            except Exception:
                # Never fall back to an all-purpose binding: that could replace
                # the visible 2F material. Author only the physics-purpose link.
                rel = floor2.CreateRelationship("material:binding:physics", custom=False)
                rel.SetTargets([material.GetPath()])
        print(
            f"[2F FRICTION MATCHED] {floor2_path} <- 1F "
            f"static={float(static_friction):.4f} dynamic={float(dynamic_friction):.4f} "
            f"source={friction_source}"
        )

        # 2F wheel anti-sink contact skin.  Do not move, replace, resize, or
        # recreate any floor geometry.  Only the PhysX contact/rest offsets of
        # existing M2 floor colliders are normalized.  A small positive rest
        # offset keeps the wheel contact surface a few millimetres above the
        # triangle mesh, which prevents the visual wheel from repeatedly
        # entering/leaving the 2F floor while preserving the elevator geometry.
        rest_offset = 0.004 * units_per_m     # 4 mm
        contact_offset = 0.020 * units_per_m  # 20 mm contact generation skin
        contact_paths: list[str] = []
        contact_failures: list[str] = []
        for prim in stage.Traverse():
            if not prim.IsActive() or not prim.IsLoaded():
                continue
            path = str(prim.GetPath())
            is_m2_floor = path.startswith("/World/HospitalMap/_floor_hospital/") and "Geo_M2_Floor" in path
            is_floor2_cube = path == floor2_path
            if not (is_m2_floor or is_floor2_cube):
                continue
            try:
                collision_enabled_attr = prim.GetAttribute("physics:collisionEnabled")
                has_collision = prim.HasAPI(UsdPhysics.CollisionAPI)
                if not has_collision and not (collision_enabled_attr and collision_enabled_attr.IsValid()):
                    continue
                if collision_enabled_attr and collision_enabled_attr.IsValid() and collision_enabled_attr.Get() is False:
                    continue
                api = PhysxSchema.PhysxCollisionAPI.Get(stage, prim.GetPath())
                if not api.GetPrim().IsValid():
                    api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                api.CreateRestOffsetAttr().Set(float(rest_offset))
                api.CreateContactOffsetAttr().Set(float(contact_offset))
                contact_paths.append(path)
            except Exception as exc:
                contact_failures.append(f"{path}: {exc}")

        print(
            f"[2F WHEEL CONTACT SKIN] colliders={len(contact_paths)} "
            f"restOffset={0.004:.3f}m contactOffset={0.020:.3f}m; "
            "floor transforms/meshes/materials/elevator unchanged"
        )
        if contact_failures:
            print(
                f"[2F WHEEL CONTACT WARNING] failed={len(contact_failures)}; "
                + " | ".join(contact_failures[:4])
            )

        duplicate = stage.GetPrimAtPath(remove_path)
        if duplicate and duplicate.IsValid():
            duplicate.SetActive(False)
            print(f"[2F DUPLICATE COLLIDER REMOVED] {remove_path} active=False")
        else:
            print(f"[2F DUPLICATE COLLIDER] already absent: {remove_path}")
    finally:
        stage.SetEditTarget(previous_target)

    return {
        "enabled": True,
        "applied": True,
        "floor1_reference": str(floor1.GetPath()) if floor1 else "",
        "floor2_path": floor2_path,
        "static_friction": float(static_friction),
        "dynamic_friction": float(dynamic_friction),
        "removed_path": remove_path,
        "floor2_collision_api": has_collision_api,
        "floor2_collision_enabled": collision_enabled,
        "floor2_rigid_body_api": has_rigid_body,
    }



def disable_visual_only_physics(stage: Usd.Stage, prim_paths: list[str] | tuple[str, ...]) -> dict[str, int]:
    """Keep the requested prim trees visible but remove their runtime collision/body response.

    Only physics enable flags/APIs under the explicitly supplied roots are touched.
    Transform, visibility, material, mesh and child placement are not modified.
    """
    previous_target = stage.GetEditTarget()
    roots_found = 0
    collisions_disabled = 0
    rigid_bodies_disabled = 0
    try:
        # Session-layer override keeps the source USD geometry/placement untouched.
        stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
        for root_path in prim_paths:
            root = stage.GetPrimAtPath(str(root_path))
            if not root or not root.IsValid():
                print(f"[VISUAL ONLY WARNING] prim not found: {root_path}")
                continue
            roots_found += 1
            for prim in Usd.PrimRange(root):
                try:
                    collision_attr = prim.GetAttribute("physics:collisionEnabled")
                    if prim.HasAPI(UsdPhysics.CollisionAPI) or (collision_attr and collision_attr.IsValid()):
                        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(False).Set(False)
                        collisions_disabled += 1
                except Exception as exc:
                    print(f"[VISUAL ONLY WARNING] collision disable failed {prim.GetPath()}: {exc}")
                try:
                    rigid_attr = prim.GetAttribute("physics:rigidBodyEnabled")
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI) or (rigid_attr and rigid_attr.IsValid()):
                        UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(False).Set(False)
                        rigid_bodies_disabled += 1
                except Exception as exc:
                    print(f"[VISUAL ONLY WARNING] rigid-body disable failed {prim.GetPath()}: {exc}")
            print(f"[VISUAL ONLY] {root_path}: visible geometry kept, collision/rigid response disabled")
    finally:
        stage.SetEditTarget(previous_target)
    return {
        "roots_found": roots_found,
        "collisions_disabled": collisions_disabled,
        "rigid_bodies_disabled": rigid_bodies_disabled,
    }

def apply_mri_scene_position(stage: Usd.Stage, cfg: dict[str, Any]) -> str:
    """Set only the requested MRI scene X/Y while preserving its existing Z/rotation/scale."""
    if not bool(cfg.get("enabled", False)):
        return ""

    prim_path = str(cfg.get("prim_path", "/World/HospitalMap/scene"))
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"MRI scene prim not found: {prim_path}")

    units = _stage_units_per_meter(stage)
    xformable = UsdGeom.Xformable(prim)
    local_result = xformable.GetLocalTransformation()
    local = Gf.Matrix4d(local_result[0] if isinstance(local_result, tuple) else local_result)
    current_t = local.ExtractTranslation()
    local.SetTranslateOnly(
        Gf.Vec3d(
            float(cfg.get("x_m", 0.0)) * units,
            float(cfg.get("y_m", 0.0)) * units,
            float(current_t[2]),
        )
    )
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp(
        UsdGeom.XformOp.PrecisionDouble,
        "requestedSceneXY",
    ).Set(local)
    print(
        f"[MRI SCENE READY] {prim_path} x={float(cfg.get('x_m', 0.0)):.5f}, "
        f"y={float(cfg.get('y_m', 0.0)):.5f}, z-preserved"
    )
    return prim_path

def ensure_fixed_mri_target(stage: Usd.Stage, cfg: dict[str, Any]) -> str:
    """Create the non-physical MRI patient mount at the measured world XYZ."""
    if not bool(cfg.get("enabled", False)):
        return ""

    target_path = str(cfg.get("target_prim", "/World/HospitalRuntimeTargets/MRIPatientTarget"))
    source_path = str(cfg.get("source_table_prim", ""))
    xyz_m = cfg.get("world_xyz_m", [8.5246, 5.8035, 12.2733])
    if not isinstance(xyz_m, (list, tuple)) or len(xyz_m) != 3:
        raise ValueError("fixed_mri_target.world_xyz_m must contain x,y,z")

    units = _stage_units_per_meter(stage)
    parent_path = str(Sdf.Path(target_path).GetParentPath())
    UsdGeom.Xform.Define(stage, Sdf.Path(parent_path))
    target = UsdGeom.Xform.Define(stage, Sdf.Path(target_path))
    target_xform = UsdGeom.Xformable(target.GetPrim())
    target_xform.ClearXformOpOrder()

    # Deliberately use world-Z yaw only. The MRI table mesh can have pitch/roll,
    # but those rotations would tilt the patient into the floor/table.
    rotation = Gf.Rotation(
        Gf.Vec3d(0.0, 0.0, 1.0),
        float(cfg.get("fallback_yaw_deg", 0.0)),
    )
    if bool(cfg.get("preserve_source_rotation", False)) and source_path:
        source_prim = stage.GetPrimAtPath(source_path)
        if source_prim.IsValid():
            try:
                source_world = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(source_prim)
                source_rotation = Gf.Transform(source_world).GetRotation()
                # Preserve only the table's horizontal yaw, never pitch/roll.
                forward = source_rotation.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
                yaw_deg = math.degrees(math.atan2(float(forward[1]), float(forward[0])))
                rotation = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), yaw_deg)
            except Exception as exc:
                print(f"[MRI TARGET WARNING] source yaw read failed: {exc}")

    transform = Gf.Transform()
    transform.SetTranslation(
        Gf.Vec3d(
            float(xyz_m[0]) * units,
            float(xyz_m[1]) * units,
            float(xyz_m[2]) * units,
        )
    )
    transform.SetRotation(rotation)
    transform.SetScale(Gf.Vec3d(1.0, 1.0, 1.0))
    target_xform.AddTransformOp(
        UsdGeom.XformOp.PrecisionDouble,
        "mriMeasuredWorld",
    ).Set(transform.GetMatrix())
    target.GetPrim().SetCustomDataByKey("hospitalMRITarget", True)
    target.GetPrim().SetCustomDataByKey("hospitalMRISourceTablePrim", source_path)
    print(
        f"[MRI TARGET READY] {target_path} x={float(xyz_m[0]):.4f}, "
        f"y={float(xyz_m[1]):.4f}, z={float(xyz_m[2]):.4f}"
    )
    return target_path


def create_amr_follow_camera(stage: Usd.Stage, cfg: dict[str, Any]) -> str:
    """Create an AMR1-local, non-physical follow camera and make it active."""
    if not bool(cfg.get("enabled", False)):
        return ""

    body_path = str(cfg.get("body_path", "/World/AMR1/base_link"))
    body_prim = stage.GetPrimAtPath(body_path)
    if not body_prim.IsValid():
        print(f"[FOLLOW CAMERA WARNING] AMR body not found: {body_path}")
        return ""

    rig_path = str(cfg.get("rig_path", body_path + "/FollowCameraRig"))
    camera_path = str(cfg.get("camera_path", rig_path + "/FollowCamera"))
    UsdGeom.Xform.Define(stage, Sdf.Path(rig_path))
    camera = UsdGeom.Camera.Define(stage, Sdf.Path(camera_path))

    units = _stage_units_per_meter(stage)
    pos = cfg.get("position_local_m", [0.0, 0.0, 6.0])
    target = cfg.get("target_local_m", [0.0, 0.0, 0.35])
    up_cfg = cfg.get("up_local", [1.0, 0.0, 0.0])
    camera_position = Gf.Vec3d(float(pos[0]) * units, float(pos[1]) * units, float(pos[2]) * units)
    camera_target = Gf.Vec3d(float(target[0]) * units, float(target[1]) * units, float(target[2]) * units)
    camera_up = Gf.Vec3d(float(up_cfg[0]), float(up_cfg[1]), float(up_cfg[2]))

    view = camera_target - camera_position
    view_len = math.sqrt(sum(float(view[i]) ** 2 for i in range(3)))
    up_len = math.sqrt(sum(float(camera_up[i]) ** 2 for i in range(3)))
    if view_len < 1.0e-9:
        raise RuntimeError("Follow camera position and target are identical")
    if up_len < 1.0e-9:
        camera_up = Gf.Vec3d(1.0, 0.0, 0.0)
        up_len = 1.0
    alignment = abs(
        sum(float(view[i]) * float(camera_up[i]) for i in range(3))
        / (view_len * up_len)
    )
    if alignment > 0.999:
        camera_up = Gf.Vec3d(1.0, 0.0, 0.0)
        if abs(float(view[0])) / view_len > 0.999:
            camera_up = Gf.Vec3d(0.0, 1.0, 0.0)

    camera_matrix = Gf.Matrix4d(1.0)
    camera_matrix.SetLookAt(camera_position, camera_target, camera_up)
    xformable = UsdGeom.Xformable(camera.GetPrim())
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp(
        UsdGeom.XformOp.PrecisionDouble,
        "followCameraLocal",
    ).Set(camera_matrix.GetInverse())
    camera.CreateFocalLengthAttr(float(cfg.get("focal_length_mm", 24.0)))
    clipping = cfg.get("clipping_range_m", [0.05, 10000.0])
    camera.CreateClippingRangeAttr(
        Gf.Vec2f(float(clipping[0]) * units, float(clipping[1]) * units)
    )
    camera.GetPrim().SetCustomDataByKey("hospitalAMRFollowCamera", True)

    if bool(cfg.get("set_active_viewport", True)):
        try:
            from omni.kit.viewport.utility import get_active_viewport

            viewport = get_active_viewport()
            if viewport is not None:
                viewport.camera_path = camera_path
        except Exception as exc:
            print(f"[FOLLOW CAMERA WARNING] viewport switch failed: {exc}")

    print(f"[FOLLOW CAMERA READY] {camera_path} follows {body_path}")
    return camera_path


def enable_patient_transfer_extension(
    project_root: Path,
    cfg: dict[str, Any],
    enable_extension: Callable[[str], None],
    app_update: Callable[[], None],
) -> None:
    """Enable the copied three-patient/MRI extension without touching drive code."""
    if not bool(cfg.get("enabled", False)):
        return

    ext_folder = (project_root / str(cfg.get("extension_folder", "patient_mri_transfer/exts"))).resolve()
    config_path = (project_root / str(cfg.get("config", "patient_mri_transfer/config/patient_transfer.json"))).resolve()
    extension_id = str(cfg.get("extension_id", "hospital.patient_transfer"))
    if not ext_folder.is_dir():
        raise FileNotFoundError(f"Patient extension folder is missing: {ext_folder}")
    if not config_path.is_file():
        raise FileNotFoundError(f"Patient transfer config is missing: {config_path}")

    os.environ["HOSPITAL_PATIENT_TRANSFER_CONFIG"] = str(config_path)
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.add_path(str(ext_folder))
    for _ in range(3):
        app_update()
    enable_extension(extension_id)
    print(f"[PATIENT MRI READY] config={config_path}")
