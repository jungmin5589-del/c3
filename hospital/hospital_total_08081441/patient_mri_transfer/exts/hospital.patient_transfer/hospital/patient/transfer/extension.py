from __future__ import annotations

import asyncio
import json
import math
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import carb
import carb.input
import omni.appwindow
import omni.ext
import omni.kit.app
import omni.kit.asset_converter
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom

Vec3 = Tuple[float, float, float]


def _vec3(values: Sequence[float] | None, default: Vec3) -> Vec3:
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        return default
    return float(values[0]), float(values[1]), float(values[2])


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _safe_id(value: str, fallback: str) -> str:
    text = "".join(ch if ch.isalnum() or ch in "_-" else "_" for ch in value.strip())
    return text or fallback


def _normalize_vec(value: Sequence[float], fallback: Vec3) -> Gf.Vec3d:
    try:
        vec = Gf.Vec3d(float(value[0]), float(value[1]), float(value[2]))
    except Exception:
        vec = Gf.Vec3d(*fallback)
    length = math.sqrt(float(vec[0]) ** 2 + float(vec[1]) ** 2 + float(vec[2]) ** 2)
    if length <= 1e-9:
        vec = Gf.Vec3d(*fallback)
        length = math.sqrt(float(vec[0]) ** 2 + float(vec[1]) ** 2 + float(vec[2]) ** 2) or 1.0
    return Gf.Vec3d(float(vec[0]) / length, float(vec[1]) / length, float(vec[2]) / length)


def _dot_vec(a: Gf.Vec3d, b: Gf.Vec3d) -> float:
    return float(a[0] * b[0] + a[1] * b[1] + a[2] * b[2])


def _cross_vec(a: Gf.Vec3d, b: Gf.Vec3d) -> Gf.Vec3d:
    return Gf.Vec3d(
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _axis_alignment_rotation(axis_cfg: Dict[str, Any]) -> Gf.Rotation:
    """Map a converted standing character's head/front axes to bed head/up axes.

    The two possible Gf rotation multiplication orders are scored at runtime,
    so this remains robust to USD/Gf composition convention differences.
    """
    source_head = _normalize_vec(axis_cfg.get("source_head_axis", (0.0, 0.0, 1.0)), (0.0, 0.0, 1.0))
    source_front = _normalize_vec(axis_cfg.get("source_front_axis", (0.0, -1.0, 0.0)), (0.0, -1.0, 0.0))
    target_head = _normalize_vec(axis_cfg.get("target_head_axis", (-1.0, 0.0, 0.0)), (-1.0, 0.0, 0.0))
    target_front = _normalize_vec(axis_cfg.get("target_front_axis", (0.0, 0.0, 1.0)), (0.0, 0.0, 1.0))

    align_head = Gf.Rotation(source_head, target_head)
    front_after = _normalize_vec(align_head.TransformDir(source_front), (0.0, 0.0, 1.0))
    cross = _cross_vec(front_after, target_front)
    signed = _dot_vec(target_head, cross)
    cosine = _clamp(_dot_vec(front_after, target_front), -1.0, 1.0)
    roll_deg = math.degrees(math.atan2(signed, cosine))
    align_front = Gf.Rotation(target_head, roll_deg)

    candidates = (align_front * align_head, align_head * align_front)
    best = candidates[0]
    best_score = -1e9
    for candidate in candidates:
        actual_head = _normalize_vec(candidate.TransformDir(source_head), (-1.0, 0.0, 0.0))
        actual_front = _normalize_vec(candidate.TransformDir(source_front), (0.0, 0.0, 1.0))
        score = _dot_vec(actual_head, target_head) + _dot_vec(actual_front, target_front)
        if score > best_score:
            best_score = score
            best = candidate
    return best


def _matrix_from_trs(translate: Vec3, rotate_xyz_deg: Vec3, scale: Vec3) -> Gf.Matrix4d:
    xf = Gf.Transform()
    xf.SetTranslation(Gf.Vec3d(*translate))
    rotation = (
        Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), rotate_xyz_deg[0])
        * Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), rotate_xyz_deg[1])
        * Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), rotate_xyz_deg[2])
    )
    xf.SetRotation(rotation)
    xf.SetScale(Gf.Vec3d(*scale))
    return xf.GetMatrix()


def _quaternion_slerp(q0: Gf.Quatd, q1: Gf.Quatd, t: float) -> Gf.Quatd:
    t = _clamp(t, 0.0, 1.0)
    a = [q0.GetReal(), *q0.GetImaginary()]
    b = [q1.GetReal(), *q1.GetImaginary()]
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:
        b = [-x for x in b]
        dot = -dot
    dot = _clamp(dot, -1.0, 1.0)
    if dot > 0.9995:
        out = [x + t * (y - x) for x, y in zip(a, b)]
    else:
        theta_0 = math.acos(dot)
        sin_theta_0 = max(1e-9, math.sin(theta_0))
        theta = theta_0 * t
        s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta_0
        s1 = math.sin(theta) / sin_theta_0
        out = [s0 * x + s1 * y for x, y in zip(a, b)]
    norm = math.sqrt(sum(x * x for x in out)) or 1.0
    out = [x / norm for x in out]
    return Gf.Quatd(out[0], Gf.Vec3d(out[1], out[2], out[3]))


def _interpolate_matrix(a: Gf.Matrix4d, b: Gf.Matrix4d, t: float, arc_height_stage: float) -> Gf.Matrix4d:
    ta = Gf.Transform(a)
    tb = Gf.Transform(b)
    pa = ta.GetTranslation()
    pb = tb.GetTranslation()
    p = Gf.Vec3d(
        pa[0] + (pb[0] - pa[0]) * t,
        pa[1] + (pb[1] - pa[1]) * t,
        pa[2] + (pb[2] - pa[2]) * t + math.sin(math.pi * t) * arc_height_stage,
    )
    q = _quaternion_slerp(ta.GetRotation().GetQuat(), tb.GetRotation().GetQuat(), t)
    sa = ta.GetScale()
    sb = tb.GetScale()
    s = Gf.Vec3d(
        sa[0] + (sb[0] - sa[0]) * t,
        sa[1] + (sb[1] - sa[1]) * t,
        sa[2] + (sb[2] - sa[2]) * t,
    )
    xf = Gf.Transform()
    xf.SetTranslation(p)
    xf.SetRotation(Gf.Rotation(q))
    xf.SetScale(s)
    return xf.GetMatrix()


def _world_matrix(prim: Usd.Prim) -> Gf.Matrix4d:
    return UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)


def _world_position(prim: Usd.Prim) -> Gf.Vec3d:
    return _world_matrix(prim).ExtractTranslation()


class PatientRuntime:
    LOCATION_SOURCE = "TRANSPORT_BED"
    LOCATION_MRI = "MRI_BED"

    def __init__(self, owner: "HospitalPatientTransferExtension", cfg: Dict[str, Any], index: int) -> None:
        self.owner = owner
        self.cfg = cfg
        self.index = index
        self.patient_id = _safe_id(str(cfg.get("id", f"patient{index + 1}")), f"patient{index + 1}")
        self.patient_name = str(cfg.get("name", self.patient_id))
        self.birth_date = str(cfg.get("birth_date", ""))
        self.root_path = str(cfg.get("patient_root_prim", f"/World/PatientTransfers/{self.patient_id}"))
        self.reference_path = str(cfg.get("patient_reference_prim", self.root_path + "/Visual"))
        self.source_bed_path = ""
        self.mri_bed_path = ""
        self.root_xform: Optional[UsdGeom.Xformable] = None
        self.visual_prim_path = self.reference_path
        self.visual_transform_op: Optional[UsdGeom.XformOp] = None
        self.asset_root_override_path: str = ""
        self.location = self.LOCATION_SOURCE
        self.transfer_active = False
        self.transfer_target = self.LOCATION_SOURCE
        self.transfer_started_wall = 0.0
        self.transfer_from = Gf.Matrix4d(1.0)
        self.transfer_to = Gf.Matrix4d(1.0)
        self.inside_since: Optional[float] = None
        self.inside_region = False
        self.armed = True
        self.cycle_count = 0
        self.last_status = "STARTING"
        self.ready = False
        self.waiting_reason = ""
        self.last_busy_retry = 0.0

    async def setup(self, stage: Usd.Stage, used_source_paths: set[str]) -> bool:
        self.source_bed_path = self.owner.resolve_source_bed(stage, self.cfg, used_source_paths)
        if not self.source_bed_path:
            carb.log_error(
                f"[PatientTransfer:{self.patient_id}] source bed not found. "
                "Check source_bed_prim/source_bed_pose in patient_transfer.json."
            )
            return False
        used_source_paths.add(self.source_bed_path)
        self.mri_bed_path = self.owner.resolve_mri_bed(stage, self.cfg)

        patient_usd = await self.owner.ensure_patient_usd(self.cfg, self.patient_id)
        if patient_usd is None:
            return False

        root = UsdGeom.Xform.Define(stage, Sdf.Path(self.root_path))
        self.root_xform = UsdGeom.Xformable(root.GetPrim())
        self.owner.ensure_matrix_op(self.root_xform)

        visual = UsdGeom.Xform.Define(stage, Sdf.Path(self.reference_path))
        refs = visual.GetPrim().GetReferences()
        refs.ClearReferences()
        refs.AddReference(str(patient_usd))
        visual_cfg = self.cfg.get("patient_visual", {})
        visual_xform = UsdGeom.Xformable(visual.GetPrim())
        visual_xform.ClearXformOpOrder()
        axis_cfg = visual_cfg.get("axis_alignment")
        if isinstance(axis_cfg, dict):
            try:
                visual_transform = Gf.Transform()
                visual_transform.SetTranslation(
                    Gf.Vec3d(*self.owner.meters_vec_to_stage(
                        stage, _vec3(visual_cfg.get("translate_m"), (0.0, 0.0, 0.0))
                    ))
                )
                visual_transform.SetRotation(_axis_alignment_rotation(axis_cfg))
                visual_transform.SetScale(Gf.Vec3d(*_vec3(visual_cfg.get("scale"), (1.0, 1.0, 1.0))))
                self.visual_transform_op = visual_xform.AddTransformOp(
                    UsdGeom.XformOp.PrecisionDouble, "patientVisualAligned"
                )
                self.visual_transform_op.Set(visual_transform.GetMatrix())
                carb.log_info(
                    f"[PatientTransfer:{self.patient_id}] standing asset axis-aligned: "
                    "face-up, head toward configured bed-head axis"
                )
            except Exception as exc:
                carb.log_warn(
                    f"[PatientTransfer:{self.patient_id}] axis alignment failed ({exc}); "
                    "using rotate_xyz_deg fallback"
                )
                visual_api = UsdGeom.XformCommonAPI(visual.GetPrim())
                visual_api.SetTranslate(
                    Gf.Vec3d(*self.owner.meters_vec_to_stage(
                        stage, _vec3(visual_cfg.get("translate_m"), (0.0, 0.0, 0.0))
                    ))
                )
                visual_api.SetRotate(
                    Gf.Vec3f(*_vec3(visual_cfg.get("rotate_xyz_deg"), (-90.0, 0.0, -90.0))),
                    UsdGeom.XformCommonAPI.RotationOrderXYZ,
                )
                visual_api.SetScale(Gf.Vec3f(*_vec3(visual_cfg.get("scale"), (1.0, 1.0, 1.0))))
        else:
            visual_api = UsdGeom.XformCommonAPI(visual.GetPrim())
            visual_api.SetTranslate(
                Gf.Vec3d(*self.owner.meters_vec_to_stage(
                    stage, _vec3(visual_cfg.get("translate_m"), (0.0, 0.0, 0.0))
                ))
            )
            visual_api.SetRotate(
                Gf.Vec3f(*_vec3(visual_cfg.get("rotate_xyz_deg"), (0.0, 0.0, 0.0))),
                UsdGeom.XformCommonAPI.RotationOrderXYZ,
            )
            visual_api.SetScale(Gf.Vec3f(*_vec3(visual_cfg.get("scale"), (1.0, 1.0, 1.0))))

        await self.apply_asset_root_override(stage)

        root_prim = root.GetPrim()
        root_prim.SetCustomDataByKey("hospitalPatientId", self.patient_id)
        root_prim.SetCustomDataByKey("hospitalPatientName", self.patient_name)
        root_prim.SetCustomDataByKey("hospitalPatientBirthDate", self.birth_date)
        root_prim.SetCustomDataByKey("hospitalPatientLocation", self.LOCATION_SOURCE)
        root_prim.SetCustomDataByKey("hospitalNonPhysicalVisual", True)
        root_prim.SetCustomDataByKey("hospitalTransferCycleCount", 0)

        self.location = self.LOCATION_SOURCE
        self.transfer_active = False
        self.inside_since = None
        self.inside_region = False
        self.armed = bool(self.cfg.get("auto_cycle", {}).get("trigger_initial_inside", True))
        self.set_patient_matrix(self.mount_world(stage, self.source_bed_path, "source_mount"))
        # Park Incheon may compose upright depending on the asset converter/root axes.
        # Evaluate several orientations from the fully composed bbox and keep the
        # flattest one before centering the visual on the mattress.
        await self.force_laying_visual(stage)
        # A converted standing character often keeps its skeleton origin around the hips.
        # After rotating it into a lying pose, half of the body can therefore intersect the
        # mattress or floor.  Fit the referenced visual to the mount plane using its actual
        # composed world bounding box instead of relying on a model-specific hard-coded Z.
        await self.fit_visual_to_mount(stage)
        self.ready = True
        self.owner.log_source_pose_validation(stage, self)
        carb.log_info(
            f"[PatientTransfer:{self.patient_id}] name={self.patient_name}; "
            f"source={self.source_bed_path}; mri={self.mri_bed_path or 'UNASSIGNED'}; asset={patient_usd}"
        )
        if not self.mri_bed_path:
            carb.log_warn(
                f"[PatientTransfer:{self.patient_id}] MRI table is not assigned. "
                "Select the MRI patient table in Stage and press K once."
            )
        self.publish_status(force=True)
        return True

    async def apply_asset_root_override(self, stage: Usd.Stage) -> bool:
        """Author a stronger local TRS on a referenced asset root.

        Values are raw stage/local units because they are copied from the Isaac Sim
        Property panel for the referenced child prim. This is intentionally separate
        from the meter-based transport-bed mount.
        """
        visual_cfg = self.cfg.get("patient_visual", {})
        override_cfg = visual_cfg.get("asset_root_override", {})
        if not isinstance(override_cfg, dict) or not bool(override_cfg.get("enabled", False)):
            return False
        suffix = str(override_cfg.get("path_suffix", "")).strip().strip("/")
        if not suffix:
            carb.log_warn(f"[PatientTransfer:{self.patient_id}] asset root override missing path_suffix")
            return False
        for _ in range(max(1, int(override_cfg.get("compose_frames", 3)))):
            await omni.kit.app.get_app().next_update_async()
        path = f"{self.reference_path}/{suffix}"
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            carb.log_warn(f"[PatientTransfer:{self.patient_id}] asset root override prim not found: {path}")
            return False
        try:
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                xformable = UsdGeom.Xformable(prim)
                xformable.ClearXformOpOrder()
                api = UsdGeom.XformCommonAPI(prim)
                api.SetTranslate(Gf.Vec3d(*_vec3(override_cfg.get("translate_stage_units"), (0.0, 0.0, 0.0))))
                api.SetRotate(
                    Gf.Vec3f(*_vec3(override_cfg.get("rotate_xyz_deg"), (0.0, 0.0, 0.0))),
                    UsdGeom.XformCommonAPI.RotationOrderXYZ,
                )
                api.SetScale(Gf.Vec3f(*_vec3(override_cfg.get("scale"), (1.0, 1.0, 1.0))))
            self.asset_root_override_path = path
            carb.log_info(
                f"[PatientTransfer:{self.patient_id}] fixed referenced asset pose applied: {path}; "
                f"T={override_cfg.get('translate_stage_units')} R={override_cfg.get('rotate_xyz_deg')} "
                f"S={override_cfg.get('scale')}"
            )
            return True
        except Exception as exc:
            carb.log_warn(f"[PatientTransfer:{self.patient_id}] asset root override failed: {exc}")
            return False

    async def force_laying_visual(self, stage: Usd.Stage) -> bool:
        """Select the flattest configured visual orientation from composed bboxes.

        This is intentionally enabled only per patient in JSON. It solves the
        converter-axis ambiguity that previously left Park Incheon standing on
        top of the transport bed. The chosen transform is then passed to the
        existing mattress bbox-fit logic.
        """
        visual_cfg = self.cfg.get("patient_visual", {})
        laying_cfg = visual_cfg.get("force_laying_pose", {})
        if not isinstance(laying_cfg, dict) or not bool(laying_cfg.get("enabled", False)):
            return False
        if self.visual_transform_op is None:
            carb.log_warn(
                f"[PatientTransfer:{self.patient_id}] FORCE-LAYING skipped: visual transform op unavailable"
            )
            return False
        visual_prim = stage.GetPrimAtPath(self.visual_prim_path)
        if not visual_prim.IsValid():
            return False

        candidates = laying_cfg.get("candidate_rotations_xyz_deg", [])
        if not isinstance(candidates, list) or not candidates:
            candidates = [[-90.0, 0.0, -90.0], [90.0, 0.0, 90.0], [0.0, 90.0, 0.0]]
        compose_frames = max(1, int(laying_cfg.get("compose_frames", 3)))
        minimum_ratio = max(1.0, float(laying_cfg.get("minimum_flatness_ratio", 1.35)))

        current_matrix = self.visual_transform_op.Get(Usd.TimeCode.Default())
        current = Gf.Transform(current_matrix)
        translation = Gf.Vec3d(current.GetTranslation())
        scale = Gf.Vec3d(current.GetScale())
        purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]
        best_matrix = current_matrix
        best_rotation = None
        best_score = -math.inf
        best_extents = None

        for raw_rotation in candidates:
            rotation_xyz = _vec3(raw_rotation, (0.0, 0.0, 0.0))
            trial = Gf.Transform()
            trial.SetTranslation(translation)
            rotation = (
                Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), rotation_xyz[0])
                * Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), rotation_xyz[1])
                * Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), rotation_xyz[2])
            )
            trial.SetRotation(rotation)
            trial.SetScale(scale)
            self.visual_transform_op.Set(trial.GetMatrix())
            for _ in range(compose_frames):
                await omni.kit.app.get_app().next_update_async()
            try:
                bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes, useExtentsHint=True)
                box = bbox_cache.ComputeWorldBound(visual_prim).ComputeAlignedBox()
                minimum = box.GetMin()
                maximum = box.GetMax()
                extents = (
                    float(maximum[0] - minimum[0]),
                    float(maximum[1] - minimum[1]),
                    float(maximum[2] - minimum[2]),
                )
                if not all(math.isfinite(value) and value > 1e-6 for value in extents):
                    continue
                horizontal = max(extents[0], extents[1])
                vertical = extents[2]
                footprint = max(1e-9, extents[0] * extents[1])
                score = horizontal / max(vertical, 1e-9) + 0.05 * math.sqrt(footprint) / max(vertical, 1e-9)
                if score > best_score:
                    best_score = score
                    best_matrix = trial.GetMatrix()
                    best_rotation = rotation_xyz
                    best_extents = extents
            except Exception as exc:
                carb.log_warn(
                    f"[PatientTransfer:{self.patient_id}] FORCE-LAYING candidate {rotation_xyz} failed: {exc}"
                )

        self.visual_transform_op.Set(best_matrix)
        for _ in range(compose_frames):
            await omni.kit.app.get_app().next_update_async()
        if best_rotation is None or best_extents is None:
            carb.log_warn(
                f"[PatientTransfer:{self.patient_id}] FORCE-LAYING failed; keeping configured orientation"
            )
            return False

        flatness = max(best_extents[0], best_extents[1]) / max(best_extents[2], 1e-9)
        meters = self.owner.meters_per_unit(stage)
        level = carb.log_info if flatness >= minimum_ratio else carb.log_warn
        level(
            f"[PatientTransfer:{self.patient_id}] FORCE-LAYING selected R={best_rotation}; "
            f"bbox=({best_extents[0]*meters:.3f},{best_extents[1]*meters:.3f},"
            f"{best_extents[2]*meters:.3f}) m; flatness={flatness:.2f}"
        )
        return True

    async def fit_visual_to_mount(self, stage: Usd.Stage) -> bool:
        visual_cfg = self.cfg.get("patient_visual", {})
        fit_cfg = visual_cfg.get("auto_fit_to_mount", {})
        if not isinstance(fit_cfg, dict) or not bool(fit_cfg.get("enabled", False)):
            return False
        if self.visual_transform_op is None:
            carb.log_warn(
                f"[PatientTransfer:{self.patient_id}] auto fit skipped: visual transform op unavailable"
            )
            return False

        visual_prim = stage.GetPrimAtPath(self.visual_prim_path)
        root_prim = stage.GetPrimAtPath(self.root_path)
        if not visual_prim.IsValid() or not root_prim.IsValid():
            return False

        # Give the referenced USD and skeleton/mesh hierarchy time to compose.
        for _ in range(max(2, int(fit_cfg.get("compose_frames", 5)))):
            await omni.kit.app.get_app().next_update_async()

        purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes, useExtentsHint=True)
        try:
            aligned = bbox_cache.ComputeWorldBound(visual_prim).ComputeAlignedBox()
            minimum = aligned.GetMin()
            maximum = aligned.GetMax()
        except Exception as exc:
            carb.log_warn(f"[PatientTransfer:{self.patient_id}] auto fit bbox failed: {exc}")
            return False

        values = [float(minimum[i]) for i in range(3)] + [float(maximum[i]) for i in range(3)]
        if not all(math.isfinite(value) for value in values):
            carb.log_warn(f"[PatientTransfer:{self.patient_id}] auto fit bbox is not finite")
            return False
        extents = Gf.Vec3d(
            float(maximum[0] - minimum[0]),
            float(maximum[1] - minimum[1]),
            float(maximum[2] - minimum[2]),
        )
        if max(float(extents[0]), float(extents[1]), float(extents[2])) <= 1e-6:
            carb.log_warn(f"[PatientTransfer:{self.patient_id}] auto fit bbox is empty")
            return False

        root_world = _world_matrix(root_prim)
        root_origin = root_world.ExtractTranslation()
        center = Gf.Vec3d(
            0.5 * float(minimum[0] + maximum[0]),
            0.5 * float(minimum[1] + maximum[1]),
            0.5 * float(minimum[2] + maximum[2]),
        )

        center_offset_m = _vec3(fit_cfg.get("center_offset_m"), (0.0, 0.0, 0.0))
        center_offset_stage = self.owner.meters_vec_to_stage(stage, center_offset_m)
        target_center = root_world.Transform(
            Gf.Vec3d(center_offset_stage[0], center_offset_stage[1], 0.0)
        )
        clearance_stage = max(0.0, float(fit_cfg.get("rest_clearance_m", 0.025))) / self.owner.meters_per_unit(stage)
        target_bottom_z = float(root_origin[2]) + clearance_stage

        world_delta = Gf.Vec3d(
            float(target_center[0] - center[0]) if bool(fit_cfg.get("center_x", True)) else 0.0,
            float(target_center[1] - center[1]) if bool(fit_cfg.get("center_y", True)) else 0.0,
            float(target_bottom_z - minimum[2]),
        )
        max_correction_stage = max(0.1, float(fit_cfg.get("max_correction_m", 3.0))) / self.owner.meters_per_unit(stage)
        length = math.sqrt(sum(float(world_delta[i]) ** 2 for i in range(3)))
        if length > max_correction_stage:
            carb.log_warn(
                f"[PatientTransfer:{self.patient_id}] auto fit correction {length * self.owner.meters_per_unit(stage):.3f} m "
                f"exceeds limit {max_correction_stage * self.owner.meters_per_unit(stage):.3f} m"
            )
            return False

        local_delta = root_world.GetInverse().TransformDir(world_delta)
        current_matrix = self.visual_transform_op.Get(Usd.TimeCode.Default())
        current = Gf.Transform(current_matrix)
        current_translation = current.GetTranslation()
        current.SetTranslation(
            Gf.Vec3d(
                float(current_translation[0] + local_delta[0]),
                float(current_translation[1] + local_delta[1]),
                float(current_translation[2] + local_delta[2]),
            )
        )
        self.visual_transform_op.Set(current.GetMatrix())

        await omni.kit.app.get_app().next_update_async()
        bbox_cache.Clear()
        after = bbox_cache.ComputeWorldBound(visual_prim).ComputeAlignedBox()
        after_min = after.GetMin()
        after_max = after.GetMax()
        carb.log_info(
            f"[PatientTransfer:{self.patient_id}] visual auto-fit complete; "
            f"delta=({world_delta[0] * self.owner.meters_per_unit(stage):.3f},"
            f"{world_delta[1] * self.owner.meters_per_unit(stage):.3f},"
            f"{world_delta[2] * self.owner.meters_per_unit(stage):.3f}) m; "
            f"bbox=({(after_max[0]-after_min[0]) * self.owner.meters_per_unit(stage):.3f},"
            f"{(after_max[1]-after_min[1]) * self.owner.meters_per_unit(stage):.3f},"
            f"{(after_max[2]-after_min[2]) * self.owner.meters_per_unit(stage):.3f}) m"
        )
        return True

    async def refit_visual(self, stage: Usd.Stage) -> bool:
        return await self.fit_visual_to_mount(stage)

    def assign_mri_bed(self, path: str) -> None:
        self.mri_bed_path = path
        self.cfg["mri_bed_prim"] = path
        self.waiting_reason = ""
        self.inside_since = None
        self.inside_region = False
        self.armed = True
        self.publish_status(force=True)

    def publish_status(self, force: bool = False) -> None:
        state = "TRANSFERRING" if self.transfer_active else self.location
        if self.waiting_reason:
            state = self.waiting_reason
        if not force and state == self.last_status:
            return
        self.last_status = state
        carb.log_info(
            f"[PatientTransfer:{self.patient_id}] {self.patient_name} status={state}; "
            f"cycle={self.cycle_count}; armed={self.armed}; mri={self.mri_bed_path or 'UNASSIGNED'}"
        )
        self.owner.publish_patient_status(self, state)

    def command(self, command: str) -> None:
        cmd = command.strip().upper()
        if cmd in {"TO_MRI", "MRI", "TRANSFER"}:
            self.start_transfer(self.LOCATION_MRI)
        elif cmd in {"TO_TRANSPORT", "TO_SOURCE", "TRANSPORT", "SOURCE", "RETURN"}:
            self.start_transfer(self.LOCATION_SOURCE)
        elif cmd in {"TOGGLE", "CYCLE"}:
            target = self.LOCATION_MRI if self.location == self.LOCATION_SOURCE else self.LOCATION_SOURCE
            self.start_transfer(target)
        elif cmd in {"RESET_ARM", "ARM"}:
            self.armed = True
            self.inside_since = None
            self.waiting_reason = ""
            self.publish_status(force=True)
        elif cmd in {"STATUS", "PING"}:
            self.publish_status(force=True)
        else:
            carb.log_warn(f"[PatientTransfer:{self.patient_id}] unknown command: {command!r}")

    def mount_world(self, stage: Usd.Stage, bed_path: str, config_key: str) -> Gf.Matrix4d:
        bed_prim = stage.GetPrimAtPath(bed_path)
        if not bed_prim or not bed_prim.IsValid():
            return Gf.Matrix4d(1.0)
        world = _world_matrix(bed_prim)
        mount_cfg = self.cfg.get(config_key, {})
        local = _matrix_from_trs(
            self.owner.meters_vec_to_stage(stage, _vec3(mount_cfg.get("translate_m"), (0.0, 0.0, 0.0))),
            _vec3(mount_cfg.get("rotate_xyz_deg"), (0.0, 0.0, 0.0)),
            _vec3(mount_cfg.get("scale"), (1.0, 1.0, 1.0)),
        )
        return local * world

    def set_patient_matrix(self, matrix: Gf.Matrix4d) -> None:
        if self.root_xform is None:
            return
        self.owner.ensure_matrix_op(self.root_xform).Set(matrix)

    def current_patient_matrix(self, stage: Usd.Stage) -> Gf.Matrix4d:
        prim = stage.GetPrimAtPath(self.root_path)
        if not prim or not prim.IsValid():
            return Gf.Matrix4d(1.0)
        return _world_matrix(prim)

    def start_transfer(self, target: str) -> bool:
        stage = self.owner.stage
        if stage is None or self.root_xform is None or not self.ready or self.transfer_active:
            return False
        if target == self.location:
            self.waiting_reason = ""
            self.publish_status(force=True)
            return False
        if target == self.LOCATION_MRI:
            if not self.mri_bed_path or not stage.GetPrimAtPath(self.mri_bed_path).IsValid():
                self.waiting_reason = "WAITING_MRI_TABLE"
                self.publish_status(force=True)
                return False
            if not self.owner.request_mri_slot(self.patient_id):
                self.waiting_reason = "WAITING_MRI_BUSY"
                self.publish_status(force=True)
                return False
        target_bed = self.mri_bed_path if target == self.LOCATION_MRI else self.source_bed_path
        target_key = "mri_mount" if target == self.LOCATION_MRI else "source_mount"
        if not target_bed or not stage.GetPrimAtPath(target_bed).IsValid():
            if target == self.LOCATION_MRI:
                self.owner.release_mri_slot(self.patient_id)
            return False
        self.waiting_reason = ""
        self.transfer_from = self.current_patient_matrix(stage)
        self.transfer_to = self.mount_world(stage, target_bed, target_key)
        self.transfer_target = target
        self.transfer_started_wall = time.monotonic()
        self.transfer_active = True
        self.publish_status(force=True)
        return True

    def distance_to_mri_m(self, stage: Usd.Stage) -> float:
        if not self.mri_bed_path:
            return math.inf
        source_prim = stage.GetPrimAtPath(self.source_bed_path)
        mri_prim = stage.GetPrimAtPath(self.mri_bed_path)
        if not source_prim.IsValid() or not mri_prim.IsValid():
            return math.inf
        source = _world_position(source_prim)
        target = _world_position(mri_prim)
        dx = float(target[0] - source[0])
        dy = float(target[1] - source[1])
        return math.hypot(dx, dy) * self.owner.meters_per_unit(stage)

    def update(self, stage: Usd.Stage, is_playing: bool, play_elapsed: Optional[float]) -> None:
        if not self.ready:
            return
        now = time.monotonic()

        if self.transfer_active:
            transfer_cfg = self.cfg.get("transfer", {})
            duration = max(0.01, float(transfer_cfg.get("duration_sec", 2.5)))
            mode = str(transfer_cfg.get("mode", "slide")).lower()
            t = _clamp((now - self.transfer_started_wall) / duration, 0.0, 1.0)
            if mode == "teleport":
                t = 1.0
            arc_stage = max(0.0, float(transfer_cfg.get("arc_height_m", 0.0))) / self.owner.meters_per_unit(stage)
            self.set_patient_matrix(_interpolate_matrix(self.transfer_from, self.transfer_to, t, arc_stage))
            if t >= 1.0:
                previous = self.location
                self.location = self.transfer_target
                self.transfer_active = False
                self.cycle_count += 1
                if previous == self.LOCATION_MRI and self.location == self.LOCATION_SOURCE:
                    self.owner.release_mri_slot(self.patient_id)
                root = stage.GetPrimAtPath(self.root_path)
                if root.IsValid():
                    root.SetCustomDataByKey("hospitalPatientLocation", self.location)
                    root.SetCustomDataByKey("hospitalTransferCycleCount", self.cycle_count)
                self.publish_status(force=True)
            return

        if self.location == self.LOCATION_MRI and self.mri_bed_path:
            self.set_patient_matrix(self.mount_world(stage, self.mri_bed_path, "mri_mount"))
        else:
            self.set_patient_matrix(self.mount_world(stage, self.source_bed_path, "source_mount"))

        auto_cfg = self.cfg.get("auto_cycle", {})
        if not bool(auto_cfg.get("enabled", True)):
            return
        if bool(auto_cfg.get("only_when_timeline_playing", True)) and not is_playing:
            self.inside_since = None
            return
        minimum_play = max(0.0, float(auto_cfg.get("minimum_play_time_sec", 3.0)))
        if play_elapsed is None or play_elapsed < minimum_play:
            return
        if not self.mri_bed_path:
            if self.waiting_reason != "WAITING_MRI_TABLE":
                self.waiting_reason = "WAITING_MRI_TABLE"
                self.publish_status(force=True)
            return

        distance_m = self.distance_to_mri_m(stage)
        enter_radius = max(0.05, float(auto_cfg.get("enter_radius_m", 1.25)))
        exit_radius = max(enter_radius + 0.05, float(auto_cfg.get("exit_radius_m", 1.75)))
        hold_seconds = max(0.0, float(auto_cfg.get("hold_seconds", 1.0)))
        retry_busy = max(0.1, float(auto_cfg.get("retry_busy_sec", 0.5)))

        if distance_m >= exit_radius:
            self.armed = True
            self.inside_region = False
            self.inside_since = None
            if self.waiting_reason in {"WAITING_MRI_BUSY", "WAITING_MRI_TABLE"}:
                self.waiting_reason = ""
                self.publish_status(force=True)
            return

        if distance_m <= enter_radius:
            if not self.inside_region:
                self.inside_region = True
                self.inside_since = now
                carb.log_info(
                    f"[PatientTransfer:{self.patient_id}] source bed entered MRI zone at {distance_m:.2f} m"
                )
            if self.armed and self.inside_since is not None and now - self.inside_since >= hold_seconds:
                target = self.LOCATION_MRI if self.location == self.LOCATION_SOURCE else self.LOCATION_SOURCE
                if self.waiting_reason == "WAITING_MRI_BUSY" and now - self.last_busy_retry < retry_busy:
                    return
                self.last_busy_retry = now
                if self.start_transfer(target):
                    self.armed = False
                    self.inside_since = None
            return

        self.inside_since = None


class HospitalPatientTransferExtension(omni.ext.IExt):
    """Three non-physical patients with repeated transport-bed/MRI-table transfer logic."""

    def on_startup(self, ext_id: str) -> None:
        self.ext_id = ext_id
        self.ext_path = Path(omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)).resolve()
        self.bundle_root = self.ext_path.parent.parent
        self.config_path = Path(
            os.environ.get(
                "HOSPITAL_PATIENT_TRANSFER_CONFIG",
                str(self.bundle_root / "config" / "patient_transfer.json"),
            )
        ).expanduser().resolve()
        self.config: Dict[str, Any] = {}
        self.stage: Optional[Usd.Stage] = None
        self.stage_identity: Optional[int] = None
        self.setup_task: Optional[asyncio.Task] = None
        self.runtimes: List[PatientRuntime] = []
        self.runtime_by_id: Dict[str, PatientRuntime] = {}
        self.play_started_wall: Optional[float] = None
        self.mri_occupant: Optional[str] = None
        self.update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._on_update, name="hospital_three_patient_transfer_update"
        )
        self.ros_node = None
        self.ros_initialized_here = False
        self.ros_subscriptions: List[Any] = []
        self.ros_status_publishers: Dict[str, Any] = {}
        self.ros_global_status_pub = None
        self.input_interface = None
        self.keyboard_subscription = None
        self._setup_keyboard()
        carb.log_info(
            f"[PatientTransfer] extension started; config={self.config_path}; "
            "K=assign selected MRI table, L=print patient status, J=refit configured patient visuals"
        )

    def on_shutdown(self) -> None:
        self.update_sub = None
        self.keyboard_subscription = None
        if self.setup_task and not self.setup_task.done():
            self.setup_task.cancel()
        if self.ros_node is not None:
            try:
                self.ros_node.destroy_node()
            except Exception:
                pass
        # The main Isaac bridge owns the process-wide rclpy lifecycle in the integrated project.
        carb.log_info("[PatientTransfer] extension stopped")

    def _setup_keyboard(self) -> None:
        try:
            app_window = omni.appwindow.get_default_app_window()
            if app_window is None:
                return
            keyboard = app_window.get_keyboard()
            self.input_interface = carb.input.acquire_input_interface()
            self.keyboard_subscription = self.input_interface.subscribe_to_keyboard_events(
                keyboard, self._on_keyboard
            )
        except Exception as exc:
            carb.log_warn(f"[PatientTransfer] keyboard setup unavailable: {exc}")

    def _on_keyboard(self, event: Any, *_args: Any) -> bool:
        if event.type != carb.input.KeyboardEventType.KEY_PRESS:
            return True
        if event.input == carb.input.KeyboardInput.K:
            self.assign_selected_mri_table()
        elif event.input == carb.input.KeyboardInput.L:
            self.print_status()
        elif event.input == carb.input.KeyboardInput.J:
            asyncio.ensure_future(self.refit_patient_visuals())
        return True

    def _load_config(self) -> Dict[str, Any]:
        with self.config_path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
        patients = data.get("patients")
        if not isinstance(patients, list) or not patients:
            raise ValueError("config must contain at least one patient entry")
        return data

    def _on_update(self, _event: Any) -> None:
        self.spin_ros_once()
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        identity = id(stage)
        if identity != self.stage_identity:
            self.stage_identity = identity
            self.stage = stage
            self.runtimes = []
            self.runtime_by_id = {}
            self.mri_occupant = None
            if self.setup_task and not self.setup_task.done():
                self.setup_task.cancel()
            self.setup_task = asyncio.ensure_future(self._setup_stage(stage))
            return

        timeline = omni.timeline.get_timeline_interface()
        is_playing = bool(timeline.is_playing())
        if is_playing and self.play_started_wall is None:
            self.play_started_wall = time.monotonic()
        elif not is_playing:
            self.play_started_wall = None
            for runtime in self.runtimes:
                runtime.inside_since = None
        play_elapsed = None if self.play_started_wall is None else time.monotonic() - self.play_started_wall
        for runtime in self.runtimes:
            runtime.update(stage, is_playing, play_elapsed)

    async def _setup_stage(self, stage: Usd.Stage) -> None:
        # Let referenced USD assets and the hospital stage finish composing first.
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        try:
            self.config = self._load_config()
        except Exception as exc:
            carb.log_error(f"[PatientTransfer] failed to load config: {exc}")
            return
        self.init_ros_from_config()
        used_source_paths: set[str] = set()
        runtimes: List[PatientRuntime] = []
        for index, cfg in enumerate(self.config.get("patients", [])):
            runtime = PatientRuntime(self, cfg, index)
            if await runtime.setup(stage, used_source_paths):
                runtimes.append(runtime)
        self.runtimes = runtimes
        self.runtime_by_id = {runtime.patient_id.lower(): runtime for runtime in runtimes}
        carb.log_info(
            f"[PatientTransfer] ready patients={[r.patient_name for r in runtimes]}; "
            f"MRI={self.shared_mri_path() or 'UNASSIGNED'}"
        )

    async def refit_patient_visuals(self) -> None:
        stage = self.stage or omni.usd.get_context().get_stage()
        if stage is None or not self.runtimes:
            carb.log_warn("[PatientTransfer] J refit unavailable: patients are not ready")
            return
        fitted = 0
        for runtime in self.runtimes:
            if await runtime.refit_visual(stage):
                fitted += 1
        carb.log_info(f"[PatientTransfer] J refit complete: {fitted}/{len(self.runtimes)} visuals adjusted")

    def shared_mri_path(self) -> str:
        shared = self.config.get("shared_mri", {})
        path = str(shared.get("mri_bed_prim", "")).strip()
        if path and self.stage is not None and self.stage.GetPrimAtPath(path).IsValid():
            return path
        for runtime in self.runtimes:
            if runtime.mri_bed_path:
                return runtime.mri_bed_path
        return ""

    def resolve_source_bed(self, stage: Usd.Stage, cfg: Dict[str, Any], excluded_paths: set[str]) -> str:
        exact = str(cfg.get("source_bed_prim", "")).strip()
        if exact and stage.GetPrimAtPath(exact).IsValid() and exact not in excluded_paths:
            return exact

        expected = cfg.get("source_bed_pose", {})
        try:
            ex = float(expected["x"])
            ey = float(expected["y"])
        except Exception:
            ex = ey = math.nan
        aliases = [str(v).lower().replace("_", "") for v in cfg.get("source_bed_aliases", [])]
        candidates: List[Tuple[float, str]] = []
        for prim in stage.Traverse():
            if not prim.IsValid() or prim.IsInstanceProxy():
                continue
            path = str(prim.GetPath())
            if path in excluded_paths or path.startswith("/World/PatientTransfers"):
                continue
            text = (path + " " + prim.GetName()).lower().replace("_", "")
            if not ("bed" in text or any(alias in text for alias in aliases)):
                continue
            if any(token in text for token in ("wheel", "caster", "nameplate", "mattress", "blanket", "amr")):
                continue
            score = 0.0
            if prim.GetTypeName() == "Xform":
                score += 10.0
            if any(alias in text for alias in aliases):
                score += 50.0
            if not math.isnan(ex):
                p = _world_position(prim)
                distance = math.hypot(float(p[0]) - ex, float(p[1]) - ey)
                score += max(0.0, 30.0 - 10.0 * distance)
            score -= path.count("/") * 0.2
            candidates.append((score, path))
        if not candidates:
            return ""
        candidates.sort(reverse=True)
        selected = candidates[0][1]
        carb.log_warn(f"[PatientTransfer] source bed fallback selected: {selected}")
        return selected

    def resolve_mri_bed(self, stage: Usd.Stage, cfg: Dict[str, Any]) -> str:
        shared = self.config.get("shared_mri", {})
        for exact in (
            str(cfg.get("mri_bed_prim", "")).strip(),
            str(shared.get("mri_bed_prim", "")).strip(),
        ):
            if exact and stage.GetPrimAtPath(exact).IsValid():
                return exact

        keywords = [
            str(v).lower()
            for v in shared.get("auto_discovery_keywords", ["mri", "scanner", "gantry", "magnetom"])
        ]
        reject_tokens = [str(v).lower() for v in shared.get("auto_discovery_reject_tokens", [])]
        prefer_second = bool(shared.get("prefer_second_floor", True))
        prefer_highest = bool(shared.get("prefer_highest_world_z", True))
        require_table_like = bool(shared.get("auto_discovery_require_table_like", False))

        candidates: List[Tuple[float, float, float, int, str]] = []
        for prim in stage.Traverse():
            if not prim.IsValid() or prim.IsInstanceProxy() or not UsdGeom.Xformable(prim):
                continue
            path = str(prim.GetPath())
            if path.startswith("/World/PatientTransfers"):
                continue
            text = (path + " " + prim.GetName()).lower()
            if not any(keyword in text for keyword in keywords):
                continue
            if any(token in text for token in reject_tokens):
                continue

            table_like = any(token in text for token in ("table", "bed", "patient", "couch", "stretcher"))
            if require_table_like and not table_like:
                continue
            floor2_hint = any(token in text for token in ("floor2", "floor_2", "2f", "secondfloor", "second_floor"))
            try:
                world_z = float(_world_position(prim)[2]) * self.meters_per_unit(stage)
            except Exception:
                world_z = -1e9
            type_score = 0
            if prim.GetTypeName() == "Xform":
                type_score += 10
            if prim.IsModel():
                type_score += 10
            if table_like:
                type_score += 40
            if any(token in text for token in ("scanner", "gantry", "magnetom")) and not table_like:
                type_score -= 10
            type_score -= path.count("/")
            candidates.append(
                (
                    1.0 if table_like else 0.0,
                    1.0 if (prefer_second and floor2_hint) else 0.0,
                    world_z if prefer_highest else 0.0,
                    type_score,
                    path,
                )
            )

        if not candidates:
            return ""
        table_candidates = [item for item in candidates if item[0] > 0.5]
        pool = table_candidates or candidates
        pool.sort(reverse=True)
        selected = pool[0][4]
        carb.log_info(
            f"[PatientTransfer] auto-discovered 2F-preferred MRI patient table: {selected}; "
            f"worldZ={pool[0][2]:.3f}m tableLike={bool(pool[0][0])}"
        )
        return selected

    def assign_selected_mri_table(self) -> None:
        stage = self.stage or omni.usd.get_context().get_stage()
        if stage is None:
            carb.log_error("[PatientTransfer] no open stage")
            return
        selected = list(omni.usd.get_context().get_selection().get_selected_prim_paths())
        if not selected:
            carb.log_error(
                "[PatientTransfer] K failed: select the MRI patient table Prim in the Stage panel first."
            )
            return
        path = str(selected[-1])
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            carb.log_error(f"[PatientTransfer] selected Prim is invalid: {path}")
            return
        self.config.setdefault("shared_mri", {})["mri_bed_prim"] = path
        for cfg in self.config.get("patients", []):
            cfg["mri_bed_prim"] = path
        for runtime in self.runtimes:
            runtime.assign_mri_bed(path)
        self.mri_occupant = None
        if bool(self.config.get("shared_mri", {}).get("persist_selected_prim", True)):
            try:
                self.config_path.write_text(
                    json.dumps(self.config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
                )
                carb.log_info(f"[PatientTransfer] MRI Prim saved to config: {path}")
            except Exception as exc:
                carb.log_warn(f"[PatientTransfer] could not persist MRI Prim: {exc}")
        p = _world_position(prim)
        carb.log_info(
            f"[PatientTransfer] MRI table assigned to all configured patients: {path}; "
            f"world=({float(p[0]):.3f}, {float(p[1]):.3f}, {float(p[2]):.3f})"
        )

    def request_mri_slot(self, patient_id: str) -> bool:
        if not bool(self.config.get("shared_mri", {}).get("single_occupancy", True)):
            return True
        if self.mri_occupant in (None, patient_id):
            self.mri_occupant = patient_id
            return True
        return False

    def release_mri_slot(self, patient_id: str) -> None:
        if self.mri_occupant == patient_id:
            self.mri_occupant = None
            carb.log_info(f"[PatientTransfer] MRI slot released by {patient_id}")

    def print_status(self) -> None:
        if not self.runtimes:
            carb.log_warn("[PatientTransfer] patients are not ready yet")
            return
        carb.log_info(
            f"[PatientTransfer] MRI={self.shared_mri_path() or 'UNASSIGNED'}; occupant={self.mri_occupant or 'NONE'}"
        )
        for runtime in self.runtimes:
            runtime.publish_status(force=True)

    def log_source_pose_validation(self, stage: Usd.Stage, runtime: PatientRuntime) -> None:
        expected = runtime.cfg.get("source_bed_pose", {})
        try:
            ex = float(expected["x"])
            ey = float(expected["y"])
        except Exception:
            return
        prim = stage.GetPrimAtPath(runtime.source_bed_path)
        if not prim.IsValid():
            return
        p = _world_position(prim)
        distance = math.hypot(float(p[0]) - ex, float(p[1]) - ey)
        level = carb.log_info if distance <= 1.5 else carb.log_warn
        level(
            f"[PatientTransfer:{runtime.patient_id}] source bed coordinate check: "
            f"configured=({ex:.4f},{ey:.4f}), prim=({float(p[0]):.4f},{float(p[1]):.4f}), "
            f"delta={distance:.3f} m"
        )

    async def ensure_patient_usd(self, cfg: Dict[str, Any], patient_id: str) -> Optional[Path]:
        asset_cfg = cfg.get("asset", {})
        source_rel = str(asset_cfg.get("source_asset", asset_cfg.get("source_fbx", ""))).strip()
        output_rel = str(asset_cfg.get("converted_usd", "")).strip()
        if not source_rel or not output_rel:
            carb.log_error(f"[PatientTransfer:{patient_id}] asset paths are missing")
            return None
        source_asset = (self.bundle_root / source_rel).resolve()
        output_usd = (self.bundle_root / output_rel).resolve()
        output_usd.parent.mkdir(parents=True, exist_ok=True)
        if output_usd.is_file() and output_usd.stat().st_size > 0:
            return output_usd
        if not source_asset.is_file():
            carb.log_error(f"[PatientTransfer:{patient_id}] missing patient asset: {source_asset}")
            return None
        carb.log_info(f"[PatientTransfer:{patient_id}] first-run asset conversion: {source_asset}")
        try:
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
            task = converter.create_converter_task(str(source_asset), str(output_usd), None, context)
            success = await task.wait_until_finished()
            if not success or not output_usd.is_file():
                carb.log_error(f"[PatientTransfer:{patient_id}] patient asset conversion failed")
                return None
            carb.log_info(f"[PatientTransfer:{patient_id}] patient asset conversion complete: {output_usd}")
            return output_usd
        except Exception as exc:
            carb.log_error(f"[PatientTransfer:{patient_id}] patient asset conversion exception: {exc}")
            return None

    def init_ros_from_config(self) -> None:
        if self.ros_node is not None:
            return
        if not bool(self.config.get("ros", {}).get("enabled", True)):
            return
        try:
            import rclpy
            from std_msgs.msg import String

            if not rclpy.ok():
                rclpy.init(args=None)
                self.ros_initialized_here = True
            self.ros_node = rclpy.create_node("hospital_three_patient_transfer_isaac")
            global_command = str(self.config.get("ros", {}).get("global_command_topic", "/patient_transfer/command"))
            global_status = str(self.config.get("ros", {}).get("global_status_topic", "/patient_transfer/status"))
            self.ros_subscriptions.append(
                self.ros_node.create_subscription(String, global_command, self._on_global_ros_command, 10)
            )
            self.ros_global_status_pub = self.ros_node.create_publisher(String, global_status, 10)
            for cfg in self.config.get("patients", []):
                patient_id = _safe_id(str(cfg.get("id", "patient")), "patient").lower()
                patient_ros = cfg.get("ros", {})
                command_topic = str(patient_ros.get("command_topic", f"/patient_transfer/{patient_id}/command"))
                status_topic = str(patient_ros.get("status_topic", f"/patient_transfer/{patient_id}/status"))
                self.ros_subscriptions.append(
                    self.ros_node.create_subscription(
                        String,
                        command_topic,
                        lambda msg, pid=patient_id: self._on_patient_ros_command(pid, msg),
                        10,
                    )
                )
                self.ros_status_publishers[patient_id] = self.ros_node.create_publisher(String, status_topic, 10)
            carb.log_info("[PatientTransfer] ROS 2 command/status topics enabled")
        except Exception as exc:
            self.ros_node = None
            self.ros_status_publishers = {}
            self.ros_global_status_pub = None
            carb.log_warn(f"[PatientTransfer] ROS 2 unavailable ({exc}); automatic logic remains active")

    def spin_ros_once(self) -> None:
        if self.ros_node is None:
            return
        try:
            import rclpy
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)
        except Exception as exc:
            carb.log_warn(f"[PatientTransfer] ROS spin error: {exc}")
            self.ros_node = None

    def _on_patient_ros_command(self, patient_id: str, msg: Any) -> None:
        runtime = self.runtime_by_id.get(patient_id.lower())
        if runtime is not None:
            runtime.command(str(getattr(msg, "data", "")))

    def _on_global_ros_command(self, msg: Any) -> None:
        text = str(getattr(msg, "data", "")).strip()
        if not text:
            return
        parts = text.replace(":", " ").replace("/", " ").split()
        target_id, command = ("all", parts[0]) if len(parts) == 1 else (parts[0].lower(), " ".join(parts[1:]))
        if target_id in {"all", "*"}:
            for runtime in self.runtimes:
                runtime.command(command)
            return
        runtime = self.runtime_by_id.get(target_id)
        if runtime is not None:
            runtime.command(command)

    def publish_patient_status(self, runtime: PatientRuntime, state: str) -> None:
        if self.ros_node is None:
            return
        try:
            from std_msgs.msg import String
            detail = {
                "id": runtime.patient_id,
                "name": runtime.patient_name,
                "birth_date": runtime.birth_date,
                "state": state,
                "cycle": runtime.cycle_count,
                "armed": runtime.armed,
                "source_bed": runtime.source_bed_path,
                "mri_bed": runtime.mri_bed_path,
                "mri_occupant": self.mri_occupant or "",
            }
            patient_msg = String()
            patient_msg.data = json.dumps(detail, ensure_ascii=False, separators=(",", ":"))
            pub = self.ros_status_publishers.get(runtime.patient_id.lower())
            if pub is not None:
                pub.publish(patient_msg)
            if self.ros_global_status_pub is not None:
                self.ros_global_status_pub.publish(patient_msg)
        except Exception:
            pass

    @staticmethod
    def ensure_matrix_op(xformable: UsdGeom.Xformable) -> UsdGeom.XformOp:
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTransform:
                return op
        return xformable.AddTransformOp(UsdGeom.XformOp.PrecisionDouble, "patientWorld")

    @staticmethod
    def meters_per_unit(stage: Usd.Stage) -> float:
        value = float(UsdGeom.GetStageMetersPerUnit(stage))
        return value if value > 1e-9 else 1.0

    def meters_vec_to_stage(self, stage: Usd.Stage, value_m: Vec3) -> Vec3:
        scale = 1.0 / self.meters_per_unit(stage)
        return value_m[0] * scale, value_m[1] * scale, value_m[2] * scale
