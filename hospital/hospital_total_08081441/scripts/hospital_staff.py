#!/usr/bin/env python3
"""Place doctor/nurse visuals upright behind the blue chairs at the desk opposite the elevator.

The supplied GLB files are Y-up, but Isaac's asset converter may already bake the
axis conversion into the generated USD.  A hard-coded +90 degree X rotation can
therefore rotate an already corrected character back onto the floor.  This module
waits for composition, tests a small set of safe rotations, and keeps the pose
with the largest vertical extent.

Furniture selection is also deterministic:
* find the 1F elevator center from the configured lift prim names;
* find desk/table candidates, preferring Office_desk and TableMid;
* require/strongly prefer a cluster with two Chair_01a/Chair objects;
* select the qualifying cluster nearest the elevator;
* place each staff member behind one chair, on the side away from the elevator.
"""
from __future__ import annotations

import asyncio
import math
from pathlib import Path
from typing import Any, Iterable

import carb
import omni.kit.app
import omni.kit.asset_converter
from pxr import Gf, Sdf, Usd, UsdGeom

_TASKS: list[asyncio.Task] = []


def _world_bounds(cache: UsdGeom.BBoxCache, prim: Usd.Prim):
    try:
        box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
        minimum, maximum = box.GetMin(), box.GetMax()
        values = [float(v) for v in (*minimum, *maximum)]
        if not all(math.isfinite(v) for v in values):
            return None
        return minimum, maximum
    except Exception:
        return None


def _center(bounds) -> Gf.Vec3d:
    minimum, maximum = bounds
    return Gf.Vec3d(
        (minimum[0] + maximum[0]) * 0.5,
        (minimum[1] + maximum[1]) * 0.5,
        (minimum[2] + maximum[2]) * 0.5,
    )


def _span_xy(bounds) -> float:
    minimum, maximum = bounds
    return max(float(maximum[0] - minimum[0]), float(maximum[1] - minimum[1]))


def _signature(prim: Usd.Prim) -> str:
    return f"{prim.GetName()} {prim.GetPath()}".lower().replace("-", "_")


def _meters_to_stage(stage: Usd.Stage, meters: float) -> float:
    mpu = float(UsdGeom.GetStageMetersPerUnit(stage))
    return float(meters) / mpu if mpu > 1.0e-9 else float(meters)


def _placement_units(stage: Usd.Stage, cfg: dict[str, Any], value: float) -> float:
    """Convert only when the map really follows USD metersPerUnit metadata.

    This hospital map, AMR poses, and Script Editor coordinates are authored in
    practical map stage units (1 unit is treated as about 1 m), while imported
    GLB/USD metadata may report centimeters.  Converting placement distances by
    that metadata made 0.65 become 65 and also caused giant character sizing.
    """
    mode = str(cfg.get("placement_unit_mode", "stage_units")).strip().lower()
    if mode in {"meter", "meters", "usd_meters"}:
        return _meters_to_stage(stage, float(value))
    return float(value)


def _named_candidates(
    stage: Usd.Stage,
    keywords: Iterable[str],
    max_world_z: float,
    excluded: Iterable[str] = (),
) -> list[tuple[str, Gf.Vec3d, float, Usd.Prim]]:
    tokens = [str(v).lower().replace("-", "_") for v in keywords]
    excluded_tokens = [str(v).lower() for v in excluded]
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    result: list[tuple[str, Gf.Vec3d, float, Usd.Prim]] = []
    for prim in stage.Traverse():
        try:
            if not prim.IsValid() or prim.IsInstanceProxy():
                continue
            signature = _signature(prim)
            name_signature = prim.GetName().lower().replace("-", "_")
            if any(token in signature for token in excluded_tokens):
                continue
            # Match the actual prim name, not every descendant whose ancestor path
            # happens to contain Office_desk/Chair. This keeps the anchor stable.
            if not any(token in name_signature for token in tokens):
                continue
            bounds = _world_bounds(cache, prim)
            if bounds is None:
                continue
            center = _center(bounds)
            if float(center[2]) > max_world_z:
                continue
            result.append((str(prim.GetPath()), center, _span_xy(bounds), prim))
        except Exception:
            continue
    return result


def _find_elevator_center(stage: Usd.Stage, cfg: dict[str, Any]) -> tuple[str, Gf.Vec3d] | None:
    names = cfg.get(
        "elevator_anchor_names",
        ["Side_Lift_Anim_29", "Side_Lift_Anim_28", "Dummy002", "elevator", "lift"],
    )
    max_z = float(cfg.get("first_floor_max_world_z", 6.0))
    candidates = _named_candidates(stage, names, max_z)
    if not candidates:
        return None
    # The physical lift floor is typically the broadest low-Z matching object.
    candidates.sort(key=lambda item: (float(item[1][2]), -item[2], item[0]))
    path, center, _span, _prim = candidates[0]
    return path, center


def _distance_xy(a: Gf.Vec3d, b: Gf.Vec3d) -> float:
    return math.hypot(float(a[0] - b[0]), float(a[1] - b[1]))


def _find_station_cluster(
    stage: Usd.Stage,
    cfg: dict[str, Any],
) -> tuple[str, Gf.Vec3d, list[tuple[str, Gf.Vec3d]], str, Gf.Vec3d] | None:
    max_z = float(cfg.get("first_floor_max_world_z", 6.0))
    excluded = cfg.get(
        "anchor_exclude_keywords",
        ["mri", "patient", "bed", "operating", "dining", "material"],
    )
    anchor_names = cfg.get("station_anchor_names", ["office_desk", "tablemid", "desk"])
    chair_names = cfg.get("station_chair_names", ["chair_01a", "chair"])
    anchors = _named_candidates(stage, anchor_names, max_z, excluded)
    chairs_raw = _named_candidates(stage, chair_names, max_z, excluded)
    if not anchors:
        return None

    elevator = _find_elevator_center(stage, cfg)
    if elevator is None:
        fallback_elevator = cfg.get("fallback_elevator_world", [-35.3208, 12.4256, 0.0])
        elevator_path = "FALLBACK_ELEVATOR_CONFIG"
        elevator_center = Gf.Vec3d(*[float(v) for v in fallback_elevator])
    else:
        elevator_path, elevator_center = elevator

    radius = _placement_units(stage, cfg, float(cfg.get("chair_search_radius_m", 6.0)))
    name_priority = [str(v).lower().replace("-", "_") for v in anchor_names]
    scored = []
    for path, center, span, prim in anchors:
        nearby = [
            (chair_path, chair_center)
            for chair_path, chair_center, _chair_span, _chair_prim in chairs_raw
            if _distance_xy(center, chair_center) <= radius
        ]
        nearby.sort(key=lambda item: _distance_xy(center, item[1]))
        signature = _signature(prim)
        priority = min((i for i, token in enumerate(name_priority) if token in signature), default=999)
        # Two nearby blue-chair objects is the strongest signal for the reception desk
        # shown opposite the elevator. Distance to the lift breaks remaining ties.
        missing_two_chairs = 0 if len(nearby) >= 2 else 1
        score = (
            missing_two_chairs,
            _distance_xy(center, elevator_center),
            priority,
            -span,
            path,
        )
        scored.append((score, path, center, nearby[:4]))

    scored.sort(key=lambda item: item[0])
    _score, anchor_path, anchor_center, nearby_chairs = scored[0]
    return anchor_path, anchor_center, nearby_chairs, elevator_path, elevator_center


def _find_floor_top(stage: Usd.Stage, x: float, y: float, fallback: float = 0.0) -> float:
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    contained: list[float] = []
    all_ground: list[tuple[float, float]] = []
    for prim in stage.Traverse():
        path = str(prim.GetPath()).lower()
        if "groundplane" not in path or "collisionmesh" not in path:
            continue
        bounds = _world_bounds(cache, prim)
        if bounds is None:
            continue
        minimum, maximum = bounds
        z = float(maximum[2])
        cx = (float(minimum[0]) + float(maximum[0])) * 0.5
        cy = (float(minimum[1]) + float(maximum[1])) * 0.5
        all_ground.append((math.hypot(cx - x, cy - y), z))
        if (
            float(minimum[0]) - 0.2 <= x <= float(maximum[0]) + 0.2
            and float(minimum[1]) - 0.2 <= y <= float(maximum[1]) + 0.2
        ):
            contained.append(z)
    if contained:
        return min(contained, key=lambda value: abs(value - fallback))
    if all_ground:
        all_ground.sort(key=lambda item: item[0])
        return all_ground[0][1]
    return float(fallback)


async def _convert_asset(source: Path, output: Path) -> Path:
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.is_file() and output.stat().st_size > 0:
        return output
    if not source.is_file():
        raise FileNotFoundError(source)
    converter = omni.kit.asset_converter.get_instance()
    context = omni.kit.asset_converter.AssetConverterContext()
    for name, value in {
        "keep_all_materials": True,
        "merge_all_meshes": False,
        "ignore_materials": False,
        "ignore_animations": False,
        "embed_textures": False,
        "use_meter_as_world_unit": True,
    }.items():
        if hasattr(context, name):
            setattr(context, name, value)
    carb.log_info(f"[HOSPITAL STAFF] converting {source.name} -> {output.name}")
    task = converter.create_converter_task(str(source), str(output), None, context)
    success = await task.wait_until_finished()
    if not success or not output.is_file():
        raise RuntimeError(f"staff asset conversion failed: {source}")
    return output


def _set_root_pose(root: Usd.Prim, x: float, y: float, z: float, yaw_deg: float) -> None:
    api = UsdGeom.XformCommonAPI(root)
    api.SetTranslate(Gf.Vec3d(x, y, z))
    api.SetRotate(Gf.Vec3f(0.0, 0.0, yaw_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    api.SetScale(Gf.Vec3f(1.0, 1.0, 1.0))


def _set_visual_rotation(visual: Usd.Prim, rotation_xyz: tuple[float, float, float], scale: float) -> None:
    xformable = UsdGeom.Xformable(visual)
    xformable.ClearXformOpOrder()
    api = UsdGeom.XformCommonAPI(visual)
    api.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
    api.SetRotate(Gf.Vec3f(*rotation_xyz), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    api.SetScale(Gf.Vec3f(scale, scale, scale))


async def _choose_upright_rotation(
    stage: Usd.Stage,
    visual: Usd.Prim,
    entry: dict[str, Any],
) -> tuple[tuple[float, float, float], float]:
    """Choose an upright pose and normalize the visual to a human map height.

    The converted character can arrive roughly 100x too large when the GLB/USD
    unit metadata differs from this hospital map's practical stage coordinates.
    Therefore final height is fitted from the composed world bbox directly in
    map stage units instead of trusting metersPerUnit.
    """
    probe_scale = float(entry.get("uniform_scale", 0.01))
    yaw_offset = float(entry.get("model_yaw_offset_deg", 0.0))
    raw_candidates = entry.get(
        "upright_candidate_rotations_xyz_deg",
        [[0.0, 0.0, 0.0], [90.0, 0.0, 0.0], [-90.0, 0.0, 0.0], [0.0, 90.0, 0.0], [0.0, -90.0, 0.0]],
    )
    candidates = [
        (float(v[0]), float(v[1]), float(v[2]) + yaw_offset)
        for v in raw_candidates
        if isinstance(v, (list, tuple)) and len(v) >= 3
    ]
    if not candidates:
        candidates = [(0.0, 0.0, yaw_offset), (90.0, 0.0, yaw_offset)]

    best = candidates[0]
    best_score = -math.inf
    best_extents = None
    compose_frames = max(2, int(entry.get("upright_compose_frames", 4)))
    purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]

    for candidate in candidates:
        _set_visual_rotation(visual, candidate, probe_scale)
        for _ in range(compose_frames):
            await omni.kit.app.get_app().next_update_async()
        bounds = _world_bounds(UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes, useExtentsHint=True), visual)
        if bounds is None:
            continue
        minimum, maximum = bounds
        extents = (
            float(maximum[0] - minimum[0]),
            float(maximum[1] - minimum[1]),
            float(maximum[2] - minimum[2]),
        )
        if not all(math.isfinite(v) and v > 1.0e-6 for v in extents):
            continue
        horizontal = max(extents[0], extents[1])
        score = extents[2] / max(horizontal, 1.0e-9)
        if score > best_score:
            best_score = score
            best = candidate
            best_extents = extents

    final_scale = probe_scale
    target_height = float(entry.get("target_height_stage_units", 1.68))
    if best_extents is not None and target_height > 0.0:
        measured_height = float(best_extents[2])
        if math.isfinite(measured_height) and measured_height > 1.0e-9:
            final_scale = probe_scale * target_height / measured_height
            min_scale = float(entry.get("min_auto_scale", 1.0e-5))
            max_scale = float(entry.get("max_auto_scale", 100.0))
            final_scale = max(min_scale, min(max_scale, final_scale))

    _set_visual_rotation(visual, best, final_scale)
    for _ in range(compose_frames):
        await omni.kit.app.get_app().next_update_async()

    final_bounds = _world_bounds(
        UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes, useExtentsHint=True), visual
    )
    final_extents = None
    if final_bounds is not None:
        minimum, maximum = final_bounds
        final_extents = (
            float(maximum[0] - minimum[0]),
            float(maximum[1] - minimum[1]),
            float(maximum[2] - minimum[2]),
        )

    carb.log_info(
        f"[HOSPITAL STAFF HUMAN SCALE] role={entry.get('role','Staff')} "
        f"target_height={target_height:.3f} stage_units scale={final_scale:.6f} "
        f"rotation={best} final_bbox={final_extents}"
    )
    return best, final_scale


def _unit_xy(dx: float, dy: float) -> tuple[float, float]:
    length = math.hypot(dx, dy)
    if length < 1.0e-9:
        return 1.0, 0.0
    return dx / length, dy / length


async def _place_staff(stage: Usd.Stage, project_root: Path, cfg: dict[str, Any]) -> None:
    if not bool(cfg.get("enabled", False)):
        return
    try:
        cluster = _find_station_cluster(stage, cfg)
        if cluster is None:
            fallback_anchor = cfg.get("fallback_anchor_world", [-34.5, 12.0, 0.0])
            fallback_elevator = cfg.get("fallback_elevator_world", [-35.3208, 12.4256, 0.0])
            anchor_path = "FALLBACK_STAFF_STATION_CONFIG"
            anchor = Gf.Vec3d(*[float(v) for v in fallback_anchor])
            chairs: list[tuple[str, Gf.Vec3d]] = []
            elevator_path = "FALLBACK_ELEVATOR_CONFIG"
            elevator = Gf.Vec3d(*[float(v) for v in fallback_elevator])
        else:
            anchor_path, anchor, chairs, elevator_path, elevator = cluster

        behind_distance = _placement_units(stage, cfg, float(cfg.get("behind_chair_offset_m", 0.65)))
        fallback_lateral = _placement_units(stage, cfg, float(cfg.get("fallback_lateral_spacing_m", 0.75)))
        people = list(cfg.get("people", []))
        UsdGeom.Xform.Define(stage, Sdf.Path(str(cfg.get("root_prim", "/World/HospitalStaff"))))

        carb.log_info(
            f"[HOSPITAL STAFF STATION] anchor={anchor_path} elevator={elevator_path} "
            f"chairs={[p for p, _c in chairs[:2]]}"
        )

        for index, entry in enumerate(people):
            role = str(entry.get("role", "Staff"))
            source = (project_root / str(entry["source_asset"])).resolve()
            converted = (project_root / str(entry["converted_usd"])).resolve()
            usd_path = await _convert_asset(source, converted)

            root_path = str(entry.get("root_prim", f"/World/HospitalStaff/{role}"))
            visual_path = root_path + "/Visual"
            root = UsdGeom.Xform.Define(stage, Sdf.Path(root_path)).GetPrim()
            visual = UsdGeom.Xform.Define(stage, Sdf.Path(visual_path)).GetPrim()
            visual.GetReferences().ClearReferences()
            visual.GetReferences().AddReference(str(usd_path))

            if index < len(chairs):
                chair_path, chair = chairs[index]
                # "Behind the chair" means farther from the elevator than the chair.
                away_x, away_y = _unit_xy(float(chair[0] - elevator[0]), float(chair[1] - elevator[1]))
                x = float(chair[0]) + away_x * behind_distance
                y = float(chair[1]) + away_y * behind_distance
                placement_anchor = chair_path
            else:
                away_x, away_y = _unit_xy(float(anchor[0] - elevator[0]), float(anchor[1] - elevator[1]))
                side_x, side_y = -away_y, away_x
                centered_index = index - (len(people) - 1) * 0.5
                x = float(anchor[0]) + away_x * behind_distance + side_x * fallback_lateral * centered_index
                y = float(anchor[1]) + away_y * behind_distance + side_y * fallback_lateral * centered_index
                placement_anchor = anchor_path

            floor_z = _find_floor_top(stage, x, y, float(anchor[2]))
            face_x, face_y = _unit_xy(float(elevator[0] - x), float(elevator[1] - y))
            yaw_deg = math.degrees(math.atan2(face_y, face_x))
            _set_root_pose(root, x, y, floor_z, yaw_deg)

            # Wait for the referenced USD to compose, then choose 0/+90/-90/etc.
            # This prevents the double Y-up->Z-up correction that laid both people down.
            for _ in range(6):
                await omni.kit.app.get_app().next_update_async()
            chosen_rotation, chosen_scale = await _choose_upright_rotation(stage, visual, entry)

            # Floor-fit using the final upright bbox.
            cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
                useExtentsHint=True,
            )
            bounds = _world_bounds(cache, visual)
            final_root_z = floor_z
            if bounds is not None:
                minimum, _maximum = bounds
                correction = floor_z - float(minimum[2])
                max_correction = _placement_units(stage, cfg, float(entry.get("max_floor_fit_m", 3.0)))
                if abs(correction) <= max_correction:
                    final_root_z = floor_z + correction
                    _set_root_pose(root, x, y, final_root_z, yaw_deg)

            root.SetCustomDataByKey("hospitalStaffRole", role)
            root.SetCustomDataByKey("hospitalNonPhysicalVisual", True)
            root.SetCustomDataByKey("hospitalFurnitureAnchor", placement_anchor)
            root.SetCustomDataByKey("hospitalStationDesk", anchor_path)
            root.SetCustomDataByKey("hospitalElevatorOpposite", elevator_path)
            carb.log_info(
                f"[HOSPITAL STAFF] {role} standing behind {placement_anchor}: "
                f"world=({x:.3f},{y:.3f},{final_root_z:.3f}) yaw={yaw_deg:.1f} "
                f"visualR={chosen_rotation} visualS={chosen_scale:.6f}"
            )

        carb.log_info(
            "[HOSPITAL STAFF READY] doctor + nurse upright behind the blue chairs at the desk opposite the elevator"
        )
    except Exception as exc:
        carb.log_warn(f"[HOSPITAL STAFF WARNING] placement skipped without stopping Isaac: {exc}")


def schedule_hospital_staff(stage: Usd.Stage, project_root: Path, cfg: dict[str, Any]) -> None:
    task = asyncio.ensure_future(_place_staff(stage, project_root, dict(cfg or {})))
    _TASKS.append(task)
