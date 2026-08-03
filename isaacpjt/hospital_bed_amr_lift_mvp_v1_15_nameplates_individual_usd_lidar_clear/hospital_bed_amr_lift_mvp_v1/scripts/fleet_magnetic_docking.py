#!/usr/bin/env python3
"""Manual snap docking with an unbreakable magnetic FixedJoint for a multi-AMR fleet."""
from __future__ import annotations

import math
from typing import Any, MutableSet

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def move_toward(current: float, target: float, maximum_delta: float) -> float:
    if target > current:
        return min(target, current + maximum_delta)
    if target < current:
        return max(target, current - maximum_delta)
    return current


def normalize_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def world_matrix(prim: Usd.Prim) -> Gf.Matrix4d:
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def planar_yaw(matrix: Gf.Matrix4d) -> float:
    forward = matrix.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
    return math.atan2(float(forward[1]), float(forward[0]))


def quatd_to_quatf(value: Gf.Quatd) -> Gf.Quatf:
    imaginary = value.GetImaginary()
    return Gf.Quatf(
        float(value.GetReal()),
        Gf.Vec3f(float(imaginary[0]), float(imaginary[1]), float(imaginary[2])),
    )


def yaw_quatf(yaw_rad: float) -> Gf.Quatf:
    half = 0.5 * float(yaw_rad)
    return Gf.Quatf(math.cos(half), Gf.Vec3f(0.0, 0.0, math.sin(half)))


class FleetMagneticDockController:
    """Arm, attract, snap and hard-lock one AMR to the nearest available bed.

    This is a deterministic docking approximation rather than a Maxwell field solver.
    The pre-lock attraction remains adjustable, while the final v1.13 lock is an
    unbreakable FixedJoint that is removed only by an explicit release action.
    """

    def __init__(
        self,
        stage: Usd.Stage,
        config: dict[str, Any],
        amr_root_path: str,
        candidate_bed_paths: list[str],
        claimed_beds: MutableSet[str],
    ) -> None:
        self.stage = stage
        self.cfg = config.get("magnetic_dock", {})
        self.root_path = amr_root_path
        self.base_path = f"{amr_root_path}/base_link"
        self.lift_path = f"{amr_root_path}/lift_plate"
        joint_name = str(self.cfg.get("joint_name", "magnetic_bed_joint"))
        self.joint_path = f"{amr_root_path}/Joints/{joint_name}"
        self.candidate_bed_paths = list(candidate_bed_paths)
        self.claimed_beds = claimed_beds

        self.available = bool(self.cfg.get("enabled", True))
        self.enabled = bool(self.cfg.get("auto_enabled_on_start", True))
        self.locked = False
        self.attached_bed_path: str | None = None
        self.target_bed_path: str | None = None
        self.last_state = "SEARCHING" if self.enabled else "DISABLED"
        self.coupling_mode = str(self.cfg.get("coupling_mode", "wheel_tow"))
        if self.coupling_mode not in {"wheel_tow", "lift_carry"}:
            print(f"[{self.root_path} magnet] unknown coupling_mode={self.coupling_mode}; using wheel_tow")
            self.coupling_mode = "wheel_tow"
        self.strength_percent = clamp(
            float(self.cfg.get("strength_percent", 100.0)),
            float(self.cfg.get("minimum_strength_percent", 10.0)),
            float(self.cfg.get("maximum_strength_percent", 100.0)),
        )

        self.base_prim = stage.GetPrimAtPath(self.base_path)
        self.lift_prim = stage.GetPrimAtPath(self.lift_path)
        if not self.base_prim.IsValid() or not self.lift_prim.IsValid():
            self.available = False
            self.enabled = False
            self.last_state = "UNAVAILABLE"

    @property
    def state(self) -> str:
        return self.last_state

    @property
    def attached_bed(self) -> str:
        return self.attached_bed_path or ""

    @property
    def is_wheel_tow_mode(self) -> bool:
        return self.coupling_mode == "wheel_tow"

    @property
    def wheel_tow_contact_lift_m(self) -> float:
        return float(self.cfg.get("wheel_tow_contact_lift_m", 0.0025))

    @property
    def wheel_tow_max_lift_m(self) -> float:
        return float(self.cfg.get("wheel_tow_max_lift_m", 0.0035))

    @property
    def unbreakable(self) -> bool:
        return bool(self.cfg.get("unbreakable_joint", True))

    def _strength_ratio(self) -> float:
        return clamp(self.strength_percent / 100.0, 0.0, 1.0)

    def _break_force_n(self) -> float:
        if self.unbreakable:
            return math.inf
        low = float(self.cfg.get("break_force_min_n", 600.0))
        high = float(self.cfg.get("break_force_max_n", 6000.0))
        return low + (high - low) * self._strength_ratio()

    def _break_torque_nm(self) -> float:
        if self.unbreakable:
            return math.inf
        low = float(self.cfg.get("break_torque_min_nm", 120.0))
        high = float(self.cfg.get("break_torque_max_nm", 1800.0))
        return low + (high - low) * self._strength_ratio()

    def _author_break_thresholds(self) -> None:
        joint = UsdPhysics.FixedJoint.Get(self.stage, self.joint_path)
        if not joint or not joint.GetPrim().IsValid():
            return
        joint.CreateBreakForceAttr().Set(float(self._break_force_n()))
        joint.CreateBreakTorqueAttr().Set(float(self._break_torque_nm()))

    def set_strength(self, percent: float) -> None:
        lower = float(self.cfg.get("minimum_strength_percent", 10.0))
        upper = float(self.cfg.get("maximum_strength_percent", 100.0))
        self.strength_percent = clamp(float(percent), lower, upper)
        self._author_break_thresholds()
        lock_text = "UNBREAKABLE" if self.unbreakable else f"breakForce={self._break_force_n():.0f}N"
        print(
            f"[{self.root_path} magnet] attraction strength={self.strength_percent:.0f}% "
            f"finalLock={lock_text}"
        )

    def toggle(self) -> None:
        if not self.available:
            print(f"[{self.root_path} magnet] unavailable")
            return
        self.enabled = not self.enabled
        if not self.enabled:
            self.release("magnet disabled")
            self.last_state = "DISABLED"
        else:
            self.last_state = "SEARCHING"
        print(
            f"[{self.root_path} magnet] {'ARMED' if self.enabled else 'OFF'} "
            f"mode={self.coupling_mode} attraction={self.strength_percent:.0f}%"
        )

    def release(self, reason: str = "manual release") -> None:
        joint_prim = self.stage.GetPrimAtPath(self.joint_path)
        if joint_prim and joint_prim.IsValid():
            self.stage.RemovePrim(self.joint_path)
        if self.attached_bed_path:
            self.claimed_beds.discard(self.attached_bed_path)
            print(f"[{self.root_path} magnet] UNLOCKED {self.attached_bed_path}: {reason}")
        self.locked = False
        self.attached_bed_path = None
        self.target_bed_path = None
        self.last_state = "SEARCHING" if self.enabled else "DISABLED"

    def _nearest_available_bed(self) -> tuple[str | None, Usd.Prim | None, float]:
        base_position = world_matrix(self.base_prim).ExtractTranslation()
        nearest_path: str | None = None
        nearest_prim: Usd.Prim | None = None
        nearest_distance = math.inf
        for bed_path in self.candidate_bed_paths:
            if bed_path in self.claimed_beds and bed_path != self.attached_bed_path:
                continue
            bed_prim = self.stage.GetPrimAtPath(bed_path)
            if not bed_prim or not bed_prim.IsValid():
                continue
            bed_position = world_matrix(bed_prim).ExtractTranslation()
            distance = math.hypot(
                float(bed_position[0] - base_position[0]),
                float(bed_position[1] - base_position[1]),
            )
            if distance < nearest_distance:
                nearest_path = bed_path
                nearest_prim = bed_prim
                nearest_distance = distance
        return nearest_path, nearest_prim, nearest_distance

    @staticmethod
    def _zero_rigid_velocity(prim: Usd.Prim) -> None:
        body = UsdPhysics.RigidBodyAPI(prim)
        velocity = body.GetVelocityAttr()
        angular = body.GetAngularVelocityAttr()
        if velocity:
            velocity.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        if angular:
            angular.Set(Gf.Vec3f(0.0, 0.0, 0.0))

    def _snap_bed_to_amr(self, bed_prim: Usd.Prim) -> None:
        """Instantly align the bed root's XY and yaw with the AMR while preserving bed Z."""
        base_world = world_matrix(self.base_prim)
        bed_world = world_matrix(bed_prim)
        base_position = base_world.ExtractTranslation()
        bed_position = bed_world.ExtractTranslation()
        target_yaw = planar_yaw(base_world)

        xformable = UsdGeom.Xformable(bed_prim)
        translate_op = None
        orient_op = None
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate and translate_op is None:
                translate_op = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeOrient and orient_op is None:
                orient_op = op
        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
        if orient_op is None:
            orient_op = xformable.AddOrientOp()

        # Bed roots are direct children of /World in the generated stage.
        translate_op.Set(
            Gf.Vec3f(float(base_position[0]), float(base_position[1]), float(bed_position[2]))
        )
        orient_op.Set(yaw_quatf(target_yaw))

        if bool(self.cfg.get("snap_zero_velocities", True)):
            self._zero_rigid_velocity(bed_prim)
            self._zero_rigid_velocity(self.base_prim)

    def _create_fixed_joint(self, bed_path: str, bed_prim: Usd.Prim) -> None:
        existing = self.stage.GetPrimAtPath(self.joint_path)
        if existing and existing.IsValid():
            self.locked = True
            self._author_break_thresholds()
            return

        lift_world = world_matrix(self.lift_prim)
        bed_world = world_matrix(bed_prim)
        anchor_world = lift_world.ExtractTranslation()
        local_pos1_d = bed_world.GetInverse().Transform(anchor_world)
        local_pos1 = Gf.Vec3f(
            float(local_pos1_d[0]), float(local_pos1_d[1]), float(local_pos1_d[2])
        )
        local_rot1 = bed_world.ExtractRotationQuat().GetInverse() * lift_world.ExtractRotationQuat()

        joint = UsdPhysics.FixedJoint.Define(self.stage, self.joint_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(self.lift_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(bed_path)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(local_pos1)
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        joint.CreateLocalRot1Attr().Set(quatd_to_quatf(local_rot1))
        joint.CreateCollisionEnabledAttr(
            not bool(self.cfg.get("disable_collision_between_attached_bodies", True))
        )
        # This is a runtime attachment to an articulation link; keep it outside the
        # reduced-coordinate articulation so it behaves as a maximal-coordinate lock.
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateJointEnabledAttr(True)
        joint.CreateBreakForceAttr().Set(float(self._break_force_n()))
        joint.CreateBreakTorqueAttr().Set(float(self._break_torque_nm()))

        self.claimed_beds.add(bed_path)
        self.locked = True
        self.attached_bed_path = bed_path
        self.target_bed_path = bed_path
        self.last_state = "HARD_LOCKED"
        feedback = str(self.cfg.get("lock_feedback_text", "CLACK! MAGNETIC HARD LOCK"))
        print(
            f"\a[{self.root_path} magnet] {feedback}: {bed_path} "
            f"mode={self.coupling_mode} breakForce=INF breakTorque=INF"
        )

    def request_lock(self) -> bool:
        """Perform the dedicated manual 'clack' lock when a bed is near enough."""
        if not self.available:
            print(f"[{self.root_path} magnet] LOCK FAILED: controller unavailable")
            return False
        if not self.enabled:
            print(f"[{self.root_path} magnet] LOCK FAILED: magnet is OFF; arm it first")
            self.last_state = "DISABLED"
            return False
        if self.locked:
            print(f"[{self.root_path} magnet] already HARD_LOCKED to {self.attached_bed_path}")
            return True

        bed_path, bed_prim, distance = self._nearest_available_bed()
        capture = float(self.cfg.get("manual_lock_capture_distance_m", 0.30))
        if bed_path is None or bed_prim is None:
            print(f"[{self.root_path} magnet] LOCK FAILED: no available bed")
            self.last_state = "NO_TARGET"
            return False
        if distance > capture:
            print(
                f"[{self.root_path} magnet] LOCK FAILED: nearest bed is {distance:.3f}m away "
                f"(must be <= {capture:.3f}m)"
            )
            self.target_bed_path = bed_path
            self.last_state = f"TOO_FAR:{distance:.2f}m"
            return False

        base_world = world_matrix(self.base_prim)
        bed_world = world_matrix(bed_prim)
        yaw_error = normalize_angle(planar_yaw(bed_world) - planar_yaw(base_world))
        yaw_limit = math.radians(float(self.cfg.get("manual_lock_max_yaw_error_deg", 20.0)))
        if abs(yaw_error) > yaw_limit:
            print(
                f"[{self.root_path} magnet] LOCK FAILED: yaw error={math.degrees(yaw_error):.1f}deg "
                f"(must be <= {math.degrees(yaw_limit):.1f}deg)"
            )
            self.target_bed_path = bed_path
            self.last_state = f"BAD_ANGLE:{math.degrees(yaw_error):.0f}deg"
            return False

        self.target_bed_path = bed_path
        if bool(self.cfg.get("snap_on_manual_lock", True)):
            self._snap_bed_to_amr(bed_prim)
        self._create_fixed_joint(bed_path, bed_prim)
        return True

    def request_release(self) -> None:
        self.release("dedicated release command")

    def update(
        self,
        lift_target_m: float,
        emergency_stop: bool = False,
    ) -> tuple[float, float, float]:
        """Return local-frame pre-lock magnetic alignment-assist velocity (vx, vy, wz)."""
        if not self.available or not self.enabled:
            return 0.0, 0.0, 0.0

        if self.locked:
            return 0.0, 0.0, 0.0

        if emergency_stop:
            self.last_state = "STOPPED"
            return 0.0, 0.0, 0.0

        bed_path, bed_prim, distance = self._nearest_available_bed()
        capture_distance = float(self.cfg.get("capture_distance_m", 0.45))
        if bed_path is None or bed_prim is None or distance > capture_distance:
            self.target_bed_path = None
            self.last_state = "SEARCHING"
            return 0.0, 0.0, 0.0

        self.target_bed_path = bed_path
        base_world = world_matrix(self.base_prim)
        bed_world = world_matrix(bed_prim)
        base_position = base_world.ExtractTranslation()
        bed_position = bed_world.ExtractTranslation()
        world_error = Gf.Vec3d(
            float(bed_position[0] - base_position[0]),
            float(bed_position[1] - base_position[1]),
            0.0,
        )
        local_error = base_world.GetInverse().TransformDir(world_error)
        yaw_error = normalize_angle(planar_yaw(bed_world) - planar_yaw(base_world))

        assist_scale = self._strength_ratio() if bool(
            self.cfg.get("scale_alignment_assist_with_strength", True)
        ) else 1.0
        assist_vx = clamp(
            float(local_error[0]) * float(self.cfg.get("position_gain", 4.0)) * assist_scale,
            -float(self.cfg.get("max_assist_linear_speed_mps", 0.22)) * assist_scale,
            float(self.cfg.get("max_assist_linear_speed_mps", 0.22)) * assist_scale,
        )
        assist_vy = clamp(
            float(local_error[1]) * float(self.cfg.get("position_gain", 4.0)) * assist_scale,
            -float(self.cfg.get("max_assist_lateral_speed_mps", 0.22)) * assist_scale,
            float(self.cfg.get("max_assist_lateral_speed_mps", 0.22)) * assist_scale,
        )
        assist_wz = clamp(
            yaw_error * float(self.cfg.get("yaw_gain", 4.0)) * assist_scale,
            -float(self.cfg.get("max_assist_angular_speed_rad_s", 0.75)) * assist_scale,
            float(self.cfg.get("max_assist_angular_speed_rad_s", 0.75)) * assist_scale,
        )
        self.last_state = f"READY_TO_LOCK:{bed_path.rsplit('/', 1)[-1]}:{distance:.2f}m"

        # v1.13 intentionally does not auto-create the joint. A dedicated lock command
        # is required so the user gets a deterministic, visible 'clack' moment.
        if not bool(self.cfg.get("manual_lock_required", True)) and bool(
            self.cfg.get("auto_lock_enabled", False)
        ):
            minimum_lift = 0.0 if not bool(
                self.cfg.get("require_lift_height_for_manual_lock", False)
            ) else float(self.cfg.get("wheel_tow_minimum_lift_for_lock_m", 0.0))
            if (
                distance <= float(self.cfg.get("lock_distance_m", 0.12))
                and abs(yaw_error) <= math.radians(float(self.cfg.get("lock_angle_deg", 15.0)))
                and lift_target_m >= minimum_lift
            ):
                if bool(self.cfg.get("snap_on_manual_lock", True)):
                    self._snap_bed_to_amr(bed_prim)
                self._create_fixed_joint(bed_path, bed_prim)
                return 0.0, 0.0, 0.0

        return assist_vx, assist_vy, assist_wz
