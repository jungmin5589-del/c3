#!/usr/bin/env python3
"""Isaac Sim standalone app for the KimSeoul AMR1 mission, elevator, ROS images, and OCR-guided approach.

Important separation of responsibilities:
- This Isaac Sim process never imports PaddleOCR.
- It publishes front-camera images and OCR requests.
- A separate ROS 2 launch process performs OCR and publishes verification/tracking results.
- After a verified result, this process laterally centers the nameplate and moves forward the configured mission distance.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import copy
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Callable

try:
    from isaacsim.simulation_app import SimulationApp
except ImportError:
    from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--stage", default="")
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


ARGS = parse_args()
APP = SimulationApp({"headless": ARGS.headless, "renderer": "RayTracedLighting"})

import carb
import numpy as np
import omni.appwindow
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.sensors.camera import Camera
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

from magnetic_docking import MagneticDockController, discover_bed_roots
from footprint_measure import report_combined_footprint
from elevator_map_only import DisabledElevator, MapOnlyElevator
from hospital_staff import schedule_hospital_staff
from aruco_nameplate_markers import install_aruco_markers
from runtime_scene_features import (
    disable_visual_only_physics,
    apply_mri_scene_position,
    apply_requested_floor_fix,
    create_amr_follow_camera,
    enable_patient_transfer_extension,
    ensure_fixed_mri_target,
)

PROJECT_ROOT = Path(ARGS.project_root).expanduser().resolve()
CONFIG_PATH = Path(ARGS.config).expanduser().resolve()
CFG: dict[str, Any] = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))



def _remap_internal_relationships(
    root: Usd.Prim,
    old_prefixes: tuple[str, ...],
    new_prefix: str,
) -> None:
    """Remap copied relationship targets so AMR2 never points back into AMR1."""
    for prim in Usd.PrimRange(root):
        for relationship in prim.GetRelationships():
            targets = relationship.GetTargets()
            if not targets:
                continue
            changed = False
            mapped = []
            for target in targets:
                text = str(target)
                replacement = text
                for old_prefix in old_prefixes:
                    if text == old_prefix or text.startswith(old_prefix + "/"):
                        replacement = new_prefix + text[len(old_prefix):]
                        changed = True
                        break
                mapped.append(Sdf.Path(replacement))
            if changed:
                relationship.SetTargets(mapped)


def replace_amr2_with_amr1_clone(stage: Usd.Stage, cfg: dict[str, Any]) -> None:
    """Replace the composed AMR2 prim with an in-memory clone of AMR1.

    The previous AMR2 world transform is preserved. A temporary backup is used,
    so a failed copy/move restores the original AMR2 instead of leaving the stage
    without a second robot.
    """
    clone_cfg = cfg.get("amr2_clone", {})
    if not bool(clone_cfg.get("enabled", True)):
        return

    source = str(clone_cfg.get("source_path", "/World/AMR1"))
    target = str(clone_cfg.get("target_path", "/World/AMR2"))
    temp = target + "__AMR1_CLONE_TMP"
    backup = target + "__OLD_BACKUP_TMP"

    source_prim = stage.GetPrimAtPath(source)
    old_target = stage.GetPrimAtPath(target)
    if not source_prim or not source_prim.IsValid():
        raise RuntimeError(f"AMR2 clone source is missing: {source}")
    if not old_target or not old_target.IsValid():
        raise RuntimeError(f"Existing AMR2 is missing and cannot provide its placement: {target}")

    old_world = UsdGeom.Xformable(old_target).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )

    for path in (temp, backup):
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            stage.RemovePrim(path)

    copied, _ = omni.kit.commands.execute(
        "CopyPrim",
        path_from=source,
        path_to=temp,
        duplicate_layers=True,
        combine_layers=False,
        flatten_references=False,
    )
    temp_prim = stage.GetPrimAtPath(temp)
    required_suffixes = (
        "/base_link",
        "/Joints/lift_joint",
        "/Joints/wheel_joint_FL",
        "/Joints/wheel_joint_FR",
        "/Joints/wheel_joint_RL",
        "/Joints/wheel_joint_RR",
    )
    if not copied or not temp_prim or not temp_prim.IsValid():
        raise RuntimeError("CopyPrim failed while creating the AMR1 clone for AMR2")
    missing = [
        temp + suffix
        for suffix in required_suffixes
        if not stage.GetPrimAtPath(temp + suffix).IsValid()
    ]
    if missing:
        stage.RemovePrim(temp)
        raise RuntimeError(f"AMR1 clone is incomplete: {missing}")

    backed_up = False
    try:
        moved_old, _ = omni.kit.commands.execute(
            "MovePrim",
            path_from=target,
            path_to=backup,
            keep_world_transform=True,
        )
        if not moved_old:
            raise RuntimeError("Could not move existing AMR2 to temporary backup")
        backed_up = True

        moved_clone, _ = omni.kit.commands.execute(
            "MovePrim",
            path_from=temp,
            path_to=target,
            keep_world_transform=False,
        )
        if not moved_clone or not stage.GetPrimAtPath(target).IsValid():
            raise RuntimeError("Could not move AMR1 clone into /World/AMR2")

        new_root = stage.GetPrimAtPath(target)
        xform = UsdGeom.Xformable(new_root)
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(old_world)
        _remap_internal_relationships(new_root, (source, temp), target)

        for _ in range(5):
            APP.update()

        missing_after = [
            target + suffix
            for suffix in required_suffixes
            if not stage.GetPrimAtPath(target + suffix).IsValid()
        ]
        if missing_after:
            raise RuntimeError(f"Replacement AMR2 validation failed: {missing_after}")

        backup_prim = stage.GetPrimAtPath(backup)
        if backup_prim and backup_prim.IsValid():
            stage.RemovePrim(backup)
        print(
            f"[AMR2 CLONE] replaced old {target} with a full clone of {source}; "
            "old AMR2 world placement preserved"
        )
    except Exception:
        bad_target = stage.GetPrimAtPath(target)
        if bad_target and bad_target.IsValid():
            stage.RemovePrim(target)
        if backed_up and stage.GetPrimAtPath(backup).IsValid():
            omni.kit.commands.execute(
                "MovePrim",
                path_from=backup,
                path_to=target,
                keep_world_transform=True,
            )
        if stage.GetPrimAtPath(temp).IsValid():
            stage.RemovePrim(temp)
        raise

def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def move_toward(current: float, target: float, max_delta: float) -> float:
    if current < target:
        return min(current + max_delta, target)
    return max(current - max_delta, target)


def get_drive(stage: Usd.Stage, path: str, drive_name: str) -> UsdPhysics.DriveAPI:
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Missing joint prim: {path}")
    drive = UsdPhysics.DriveAPI.Get(prim, drive_name)
    if not drive:
        raise RuntimeError(f"Missing {drive_name} drive: {path}")
    return drive


def world_position(prim: Usd.Prim) -> Gf.Vec3d:
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()


def world_yaw(prim: Usd.Prim) -> float:
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    forward = matrix.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
    return math.atan2(float(forward[1]), float(forward[0]))


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def enable_extension(name: str) -> None:
    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.is_extension_enabled(name):
        manager.set_extension_enabled_immediate(name, True)
    for _ in range(5):
        APP.update()
    if not manager.is_extension_enabled(name):
        raise RuntimeError(f"Failed to enable Isaac Sim extension: {name}")
    print(f"[extension] enabled: {name}")


class DirectCamera:
    def __init__(self, root_path: str, namespace: str, camera_cfg: dict[str, Any]) -> None:
        suffix = str(camera_cfg["front_prim_suffix"])
        self.prim_path = f"{root_path}/{suffix}"
        self.namespace = namespace
        self.camera = Camera(
            prim_path=self.prim_path,
            name=f"{namespace}_front_camera",
            frequency=float(camera_cfg["frequency_hz"]),
            resolution=tuple(map(int, camera_cfg["resolution"])),
        )
        self.camera.initialize()
        for _ in range(max(2, int(camera_cfg.get("warmup_frames", 8)))):
            APP.update()
        self.camera.initialize()
        APP.update()
        self.render_product_path = self.camera._render_product_path
        if not self.render_product_path:
            raise RuntimeError(f"Camera render product was not created: {self.prim_path}")
        print(f"[camera] /{namespace}/camera/front/color/image_raw <- {self.prim_path}")


def attach_ros2_rgb_writer(camera: DirectCamera, ros_cfg: dict[str, Any], camera_cfg: dict[str, Any]) -> Any:
    import omni.graph.core as og
    import omni.replicator.core as rep
    import omni.syntheticdata._syntheticdata as sd

    topic = f"/{camera.namespace}/{ros_cfg['image_topic_suffix']}"
    frame_id = f"{camera.namespace}/front_camera_link"
    render_var = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    writer = rep.writers.get(render_var + "ROS2PublishImage")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace="",
        queueSize=max(1, int(ros_cfg.get("queue_size", 3))),
        topicName=topic.lstrip("/"),
    )
    writer.attach([camera.render_product_path])
    for _ in range(3):
        APP.update()

    frequency = max(1.0, float(camera_cfg.get("frequency_hz", 15.0)))
    step_size = max(1, int(round(60.0 / frequency)))
    try:
        gate_name = render_var + "IsaacSimulationGate"
        gate = omni.syntheticdata.SyntheticData._get_node_path(gate_name, camera.render_product_path)
        og.Controller.attribute(gate + ".inputs:step").set(step_size)
    except Exception as exc:
        print(f"[camera warning] publish-rate gate: {exc}")
    print(f"[ROS2 image PUB] {topic} at about {frequency:.1f} Hz")
    return writer


class AMRController:
    """Direct holonomic velocity controller used by both teleop and automatic approach."""

    def __init__(
        self,
        stage: Usd.Stage,
        unit: dict[str, Any],
        cfg: dict[str, Any],
        candidate_bed_paths: list[str],
        claimed_beds: set[str],
    ) -> None:
        self.stage = stage
        self.unit = unit
        self.name = str(unit["name"])
        self.namespace = str(unit["namespace"])
        self.root = str(unit["root_path"])
        self.base_path = f"{self.root}/base_link"
        self.base_prim = stage.GetPrimAtPath(self.base_path)
        if not self.base_prim or not self.base_prim.IsValid():
            raise RuntimeError(f"Missing AMR base: {self.base_path}")
        body = UsdPhysics.RigidBodyAPI(self.base_prim)
        self.velocity_attr = body.GetVelocityAttr()
        self.angular_velocity_attr = body.GetAngularVelocityAttr()
        self.lift_drive = get_drive(stage, f"{self.root}/Joints/lift_joint", "linear")
        self.wheel_drives = {
            name: get_drive(stage, f"{self.root}/Joints/wheel_joint_{name}", "angular")
            for name in ("FL", "FR", "RL", "RR")
        }
        self.teleop_cfg = cfg["teleop"]
        self.geometry_cfg = cfg["amr_geometry"]
        self.loaded_drive_cfg = cfg.get("loaded_drive_profile", {})
        self._wheel_original_max_force: dict[str, float | None] = {}
        for name, drive in self.wheel_drives.items():
            try:
                value = drive.GetMaxForceAttr().Get()
                self._wheel_original_max_force[name] = None if value is None else float(value)
            except Exception:
                self._wheel_original_max_force[name] = None
        self._loaded_profile_active = False
        self.lift_target = float(self.lift_drive.GetTargetPositionAttr().Get() or 0.0)
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.command_hold_until = 0.0
        # The physical elevator owns base_link motion only while the FixedJoint is active.
        self.external_physics_mode = False
        self.magnet = MagneticDockController(
            stage, cfg, self.root, candidate_bed_paths, claimed_beds
        )

    def hold_motion(self, seconds: float = 0.6) -> None:
        self.command_hold_until = max(self.command_hold_until, time.monotonic() + seconds)
        self.halt()

    def set_external_physics_mode(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled == self.external_physics_mode:
            return
        if enabled:
            self.halt()
            self.external_physics_mode = True
            print(f"[{self.name}] ELEVATOR PHYSICS PASSIVE=ON")
        else:
            self.external_physics_mode = False
            self.halt()
            print(f"[{self.name}] ELEVATOR PHYSICS PASSIVE=OFF")

    @property
    def motion_hold_active(self) -> bool:
        return time.monotonic() < self.command_hold_until

    def _set_loaded_drive_profile(self, attached: bool) -> None:
        if not bool(self.loaded_drive_cfg.get("enabled", True)):
            return
        if attached == self._loaded_profile_active:
            return

        if attached:
            # AMR1/AMR2 use the same gentle loaded wheel force.  Transport speed is
            # controlled by the common doubled velocity limits, not by extra torque.
            max_force = float(self.loaded_drive_cfg.get("wheel_max_force_nm", 60.0))
            for drive in self.wheel_drives.values():
                drive.GetMaxForceAttr().Set(max_force)
            self._loaded_profile_active = True
            print(
                f"[{self.name} LOADED DRIVE] bed attached: "
                f"wheel Max Force={max_force:.1f} N·m"
            )
            return

        for name, drive in self.wheel_drives.items():
            original = self._wheel_original_max_force.get(name)
            attr = drive.GetMaxForceAttr()
            if original is None:
                attr.Clear()
            else:
                attr.Set(float(original))
        self._loaded_profile_active = False
        print(f"[{self.name} LOADED DRIVE] bed released: wheel Max Force restored")

    def _set_lift_raised(self, raised: bool, settle_sec: float = 0.0) -> None:
        lower = float(self.geometry_cfg["lift_lower_limit_m"])
        upper = float(self.geometry_cfg["lift_upper_limit_m"])
        self.lift_target = upper if raised else lower
        self.lift_drive.GetTargetPositionAttr().Set(float(self.lift_target))
        if settle_sec > 0.0:
            self.hold_motion(settle_sec)
        state = "RAISED" if raised else "LOWERED"
        print(
            f"[{self.name} AUTO LIFT] {state}: target={self.lift_target:.3f}m, "
            f"motion hold={max(0.0, settle_sec):.1f}s"
        )

    def request_magnet_lock(self) -> bool:
        self.hold_motion()
        success = self.magnet.request_lock()
        if success:
            self._set_loaded_drive_profile(True)
            if bool(self.magnet.cfg.get("auto_raise_lift_after_lock", True)):
                self._set_lift_raised(
                    True,
                    float(self.magnet.cfg.get("lift_settle_sec", 3.2)),
                )
        return success

    def ensure_magnet_lock(self) -> bool:
        """Apply the same bed-lock action as C, only if its FixedJoint is missing."""
        joint = self.stage.GetPrimAtPath(self.magnet.joint_path)
        joint_valid = bool(joint and joint.IsValid())
        if self.magnet.locked and joint_valid:
            print(f"[{self.name} MAGNET] 2F auto-C: existing bed joint is secure")
            return True
        if self.magnet.locked and not joint_valid:
            self.magnet.locked = False
            print(f"[{self.name} MAGNET] 2F auto-C: recreating missing bed joint")
        return self.request_magnet_lock()

    def request_magnet_release(self) -> None:
        self.hold_motion()
        self.magnet.release("manual X release")
        self._set_loaded_drive_profile(False)
        if bool(self.magnet.cfg.get("auto_lower_lift_after_release", True)):
            self._set_lift_raised(False, 0.8)

    def halt(self) -> None:
        self.current_vx = self.current_vy = self.current_wz = 0.0
        for drive in self.wheel_drives.values():
            drive.GetTargetVelocityAttr().Set(0.0)
        # Do not overwrite the vertical velocity generated by the elevator carrier.
        if self.external_physics_mode:
            return
        self.velocity_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        self.angular_velocity_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))

    def update(
        self,
        dt: float,
        command: tuple[float, float, float],
        lift_direction: float = 0.0,
        estop: bool = False,
    ) -> None:
        lower = float(self.geometry_cfg["lift_lower_limit_m"])
        upper = float(self.geometry_cfg["lift_upper_limit_m"])
        if lift_direction:
            self.lift_target = clamp(
                self.lift_target
                + lift_direction * float(self.teleop_cfg["lift_command_speed_mps"]) * dt,
                lower,
                upper,
            )
        self.lift_drive.GetTargetPositionAttr().Set(float(self.lift_target))

        if self.external_physics_mode:
            self.current_vx = self.current_vy = self.current_wz = 0.0
            for drive in self.wheel_drives.values():
                drive.GetTargetVelocityAttr().Set(0.0)
            return

        target_vx, target_vy, target_wz = command
        if estop:
            target_vx = target_vy = target_wz = 0.0
            self.current_vx = self.current_vy = self.current_wz = 0.0
        else:
            self.current_vx = move_toward(
                self.current_vx,
                target_vx,
                float(self.teleop_cfg["linear_accel_mps2"]) * dt,
            )
            self.current_vy = move_toward(
                self.current_vy,
                target_vy,
                float(self.teleop_cfg["lateral_accel_mps2"]) * dt,
            )
            self.current_wz = move_toward(
                self.current_wz,
                target_wz,
                float(self.teleop_cfg["angular_accel_rad_s2"]) * dt,
            )

        matrix = UsdGeom.Xformable(self.base_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        world_velocity = matrix.TransformDir(Gf.Vec3d(self.current_vx, self.current_vy, 0.0))
        # Both robots keep the proven v1-style linear rigid-body drive while loaded.
        # This is what keeps coupled straight motion smooth at the new 2x speed.
        self.velocity_attr.Set(
            Gf.Vec3f(float(world_velocity[0]), float(world_velocity[1]), 0.0)
        )

        # Straight loaded motion: give the attached bed the exact same horizontal
        # velocity as its AMR.  Do not assist during turns so forced-yaw behavior
        # remains unchanged.
        if (
            self.magnet.locked
            and abs(self.current_wz) < 0.03
            and abs(self.current_vy) < 0.03
        ):
            bed_body_path = self.magnet.attached_body
            if bed_body_path:
                bed_body = self.stage.GetPrimAtPath(bed_body_path)
                if bed_body and bed_body.IsValid() and bed_body.HasAPI(UsdPhysics.RigidBodyAPI):
                    bed_rb = UsdPhysics.RigidBodyAPI(bed_body)
                    bed_rb.GetVelocityAttr().Set(
                        Gf.Vec3f(float(world_velocity[0]), float(world_velocity[1]), 0.0)
                    )

        self.angular_velocity_attr.Set(Gf.Vec3f(0.0, 0.0, float(math.degrees(self.current_wz))))

        radius = float(self.geometry_cfg["wheel_radius"])
        lever = float(self.geometry_cfg["wheel_x"]) + float(self.geometry_cfg["wheel_y"])
        wheel_rad_s = {
            "FL": (self.current_vx - self.current_vy - lever * self.current_wz) / radius,
            "FR": (self.current_vx + self.current_vy + lever * self.current_wz) / radius,
            "RL": (self.current_vx + self.current_vy - lever * self.current_wz) / radius,
            "RR": (self.current_vx - self.current_vy + lever * self.current_wz) / radius,
        }
        max_deg_s = float(self.teleop_cfg["wheel_visual_velocity_limit_deg_s"])
        for name, rad_s in wheel_rad_s.items():
            self.wheel_drives[name].GetTargetVelocityAttr().Set(
                float(clamp(math.degrees(rad_s), -max_deg_s, max_deg_s))
            )


@dataclass
class TrackingObservation:
    """Latest rectangular nameplate centre reported by the external OCR node.

    Only the horizontal image coordinate is used.  The mission pose fixes the
    camera height and viewing direction, so AMR1/AMR2 correct lateral error with
    the same motion as the manual Q/E keys and never gate motion on image Y.
    """

    request_id: str
    center_x: float
    center_y: float
    image_width: float
    image_height: float
    received_at: float

    @property
    def screen_center_x(self) -> float:
        return self.image_width * 0.5

    @property
    def screen_center_y(self) -> float:
        return self.image_height * 0.5

    @property
    def error_x_pixels(self) -> float:
        return self.center_x - self.screen_center_x

    @property
    def error_x_normalized(self) -> float:
        return self.error_x_pixels / max(1.0, self.screen_center_x)

    @property
    def error_y_pixels(self) -> float:
        return self.center_y - self.screen_center_y


class AutoApproach:
    """State machine: request OCR -> lateral visual servo -> forward configured distance."""

    def __init__(
        self,
        controller: AMRController,
        unit: dict[str, Any],
        cfg: dict[str, Any],
        publish_request: Callable[[dict[str, Any]], None],
        publish_control: Callable[[dict[str, Any]], None],
        publish_status: Callable[[dict[str, Any]], None],
    ) -> None:
        self.controller = controller
        self.unit = unit
        self.cfg = cfg
        self.auto_cfg = dict(cfg["auto_approach"])
        self.publish_request = publish_request
        self.publish_control = publish_control
        self.publish_status = publish_status
        self.state = "IDLE"
        self.request_id = ""
        self.observation: TrackingObservation | None = None
        self.aruco_pair: dict[str, Any] | None = None
        self.aruco_received_at = -1.0
        self.aruco_last_processed_at = -1.0
        self.stable_count = 0
        self.forward_start: Gf.Vec3d | None = None
        self.last_state_print = ""
        self.last_processed_observation_at = -1.0
        self.last_alignment_direction = ""

    @property
    def active(self) -> bool:
        return self.state not in {"IDLE", "COMPLETE", "FAILED"}

    def _set_state(self, state: str, reason: str = "") -> None:
        self.state = state
        if state != self.last_state_print:
            suffix = f" — {reason}" if reason else ""
            print(f"[{self.controller.name}] AUTO={state}{suffix}")
            self.last_state_print = state
        self.publish_status(
            {
                "amr": self.controller.namespace,
                "request_id": self.request_id,
                "state": state,
                "reason": reason,
                "timestamp": time.time(),
            }
        )

    def start(self) -> None:
        self.controller.halt()
        self.request_id = f"{self.controller.namespace}_{time.strftime('%Y%m%d_%H%M%S')}_{int(time.time_ns() % 1000000):06d}"
        self.observation = None
        self.aruco_pair = None
        self.aruco_received_at = -1.0
        self.aruco_last_processed_at = -1.0
        self.stable_count = 0
        self.forward_start = None
        self.last_processed_observation_at = -1.0
        self.last_alignment_direction = ""
        self._set_state("WAITING_OCR", "OCR request published")
        self.publish_request(
            {
                "protocol_version": 1,
                "command": "VERIFY_AND_TRACK",
                "request_id": self.request_id,
                "amr": self.controller.namespace,
                "expected_name": str(self.unit["target_name"]),
                "expected_birth_date": str(self.unit["target_birth_date"]),
                # Seo Suwon only: use one frame for a much faster demo OCR request.
                # Kim Seoul / Park Incheon keep the existing fast 2-frame setting.
                "frames_to_check": int(self.auto_cfg["frames_to_check"]),
                "candidates": self.cfg["patient_candidates"],
                "timestamp": time.time(),
            }
        )
        print(
            f"[{self.controller.name}] OCR request: expected="
            f"{self.unit['target_name']} {self.unit['target_birth_date']} request_id={self.request_id}"
        )

    def cancel(self, reason: str) -> None:
        if self.request_id:
            self.publish_control(
                {
                    "protocol_version": 1,
                    "action": "STOP_TRACKING",
                    "request_id": self.request_id,
                    "amr": self.controller.namespace,
                    "reason": reason,
                }
            )
        self.controller.halt()
        self._set_state("FAILED", reason)

    def handle_result(self, payload: dict[str, Any]) -> None:
        """Use OCR only for identity verification; docking alignment is ArUco-only."""
        if str(payload.get("request_id", "")) != self.request_id:
            return
        state = str(payload.get("state", ""))
        if state in {"REJECTED", "ERROR"}:
            selected = f"{payload.get('selected_name', '')} {payload.get('selected_birth_date', '')}".strip()
            self.cancel(f"OCR {state}: selected={selected or 'NONE'}")
            return
        if state not in {"VERIFIED", "TRACKING"}:
            return
        if not bool(payload.get("verified", False)):
            return
        if str(payload.get("selected_name", "")) != str(self.unit["target_name"]):
            self.cancel("OCR name does not match mission target")
            return
        if str(payload.get("selected_birth_date", "")) != str(self.unit["target_birth_date"]):
            self.cancel("OCR birth date does not match mission target")
            return
        if self.state != "WAITING_OCR":
            return

        print(
            f"[{self.controller.name}] VERIFIED: {payload.get('selected_name')} "
            f"{payload.get('selected_birth_date')} score={payload.get('score')}"
        )
        # Stop OCR tracking immediately. From this point only the fixed left/right
        # ArUco pair is allowed to steer the AMR.
        self.publish_control(
            {
                "protocol_version": 1,
                "action": "STOP_TRACKING",
                "request_id": self.request_id,
                "amr": self.controller.namespace,
                "reason": "identity verified; switch to paired ArUco docking",
            }
        )
        self.stable_count = 0
        self.last_alignment_direction = ""
        # ArUco detector runs continuously in parallel with OCR. If a fresh pair is
        # already available, switch immediately in this same callback instead of
        # adding an extra WAITING_ARUCO cycle.
        if self.aruco_pair is not None and (time.monotonic() - self.aruco_received_at) <= 0.60:
            self.aruco_last_processed_at = -1.0
            self._set_state("ARUCO_ALIGN", "OCR verified -> fresh paired ArUco already available")
        else:
            self._set_state("WAITING_ARUCO", "OCR verified -> waiting only for next paired ArUco frame")

    def handle_aruco_result(self, payload: dict[str, Any]) -> None:
        if str(payload.get("amr", "")).lower() != self.controller.namespace.lower():
            return
        patient = str(self.unit.get("target_name", ""))
        pairs = payload.get("pairs", {})
        if not isinstance(pairs, dict):
            return
        pair = pairs.get(patient)
        if not isinstance(pair, dict):
            return

        expected = {"김서울": (10, 11), "박인천": (20, 21), "서수원": (30, 31)}.get(patient)
        if expected is None:
            return
        try:
            left_id = int(pair.get("left_id", -1))
            right_id = int(pair.get("right_id", -1))
        except (TypeError, ValueError):
            return
        if (left_id, right_id) != expected:
            return
        if not bool(pair.get("pair_order_ok", False)):
            return
        try:
            if float(pair.get("min_marker_side_px", 0.0)) < 5.0:
                return
            # Validate all control inputs before storing the observation.
            float(pair["center_error_px"])
            float(pair["yaw_error_ratio"])
            float(payload["image_width"])
        except (KeyError, TypeError, ValueError):
            return

        self.aruco_pair = dict(pair)
        self.aruco_pair["image_width"] = float(payload["image_width"])
        self.aruco_received_at = time.monotonic()

    def update(self, dt: float, estop: bool) -> tuple[float, float, float] | None:
        if estop and self.active:
            self.cancel("emergency stop")
            return (0.0, 0.0, 0.0)
        if self.state == "WAITING_OCR":
            return (0.0, 0.0, 0.0)
        if self.state == "WAITING_ARUCO":
            if self.aruco_pair is None:
                return (0.0, 0.0, 0.0)
            if time.monotonic() - self.aruco_received_at > 0.60:
                return (0.0, 0.0, 0.0)
            self.stable_count = 0
            self.aruco_last_processed_at = -1.0
            self.last_alignment_direction = ""
            self._set_state("ARUCO_ALIGN", "paired markers acquired; yaw + lateral correction together")
            return (0.0, 0.0, 0.0)

        if self.state == "ARUCO_ALIGN":
            if self.aruco_pair is None:
                return (0.0, 0.0, 0.0)
            age = time.monotonic() - self.aruco_received_at
            if age > 0.60:
                self.stable_count = 0
                return (0.0, 0.0, 0.0)

            pair = self.aruco_pair
            yaw_error = float(pair["yaw_error_ratio"])
            center_error_px = float(pair["center_error_px"])
            image_width = max(1.0, float(pair["image_width"]))
            center_error_norm = center_error_px / (0.5 * image_width)
            is_new = self.aruco_received_at > self.aruco_last_processed_at

            yaw_tol = 0.090
            center_tol_px = 10.0
            yaw_ok = abs(yaw_error) <= yaw_tol
            center_ok = abs(center_error_px) <= center_tol_px

            if yaw_ok and center_ok:
                if is_new:
                    self.aruco_last_processed_at = self.aruco_received_at
                    self.stable_count += 1
                if self.stable_count >= 2:
                    self.forward_start = world_position(self.controller.base_prim)
                    self.last_alignment_direction = "CENTER"
                    self._set_state("FORWARD_TARGET", "ArUco yaw + bed centre stable")
                return (0.0, 0.0, 0.0)

            if is_new:
                self.aruco_last_processed_at = self.aruco_received_at
                self.stable_count = 0

            # Do NOT block lateral correction behind a separate yaw phase. The AMR is
            # holonomic, so fix image-centre error with Q/E while yaw is corrected at
            # the same time. This is both faster and makes lateral motion observable.
            lateral = 0.0
            if not center_ok:
                lateral_sign = float(self.unit.get("image_error_to_lateral_sign", -1.0))
                lateral = lateral_sign * float(self.auto_cfg.get("lateral_kp", 1.6)) * center_error_norm
                lateral_limit = abs(float(self.auto_cfg.get("max_lateral_speed_mps", 0.20)))
                lateral = clamp(lateral, -lateral_limit, lateral_limit)
                if 0.0 < abs(lateral) < 0.035:
                    lateral = math.copysign(0.035, lateral)

            angular = 0.0
            if not yaw_ok:
                yaw_limit = abs(float(self.auto_cfg.get("max_yaw_speed_rad_s", 0.36)))
                yaw_min = min(yaw_limit, abs(float(self.auto_cfg.get("min_yaw_speed_rad_s", 0.10))))
                requested = float(self.auto_cfg.get("yaw_kp", 0.9)) * abs(yaw_error)
                speed = clamp(requested, yaw_min, yaw_limit)
                # Positive yaw_error = right marker appears larger/closer -> rotate right.
                angular = -math.copysign(speed, yaw_error)

            direction = (
                f"lat={'Q' if lateral > 0.0 else 'E' if lateral < 0.0 else '-'} "
                f"yaw={'L' if angular > 0.0 else 'R' if angular < 0.0 else '-'}"
            )
            if direction != self.last_alignment_direction:
                print(
                    f"[{self.controller.name}] ARUCO_ALIGN {direction}: "
                    f"center={center_error_px:+.1f}px yaw={yaw_error:+.3f} "
                    f"vy={lateral:+.3f} wz={angular:+.3f}"
                )
                self.last_alignment_direction = direction
            return (0.0, lateral, angular)

        if self.state == "FORWARD_TARGET":
            if self.forward_start is None:
                self.cancel("forward start position missing")
                return (0.0, 0.0, 0.0)
            current = world_position(self.controller.base_prim)
            distance = math.hypot(
                float(current[0] - self.forward_start[0]),
                float(current[1] - self.forward_start[1]),
            )
            target_distance = float(self.auto_cfg["forward_distance_m"])
            if distance >= target_distance:
                self.controller.halt()
                self.publish_control(
                    {
                        "protocol_version": 1,
                        "action": "STOP_TRACKING",
                        "request_id": self.request_id,
                        "amr": self.controller.namespace,
                        "reason": "forward distance complete",
                    }
                )
                self._set_state("COMPLETE", f"moved {distance:.3f} m")
                print(f"[{self.controller.name}] AUTO APPROACH COMPLETE: {distance:.3f} m")
                return (0.0, 0.0, 0.0)
            return (float(self.auto_cfg["forward_speed_mps"]), 0.0, 0.0)

        return None


class TrafficPassingManeuver:
    """Agreed local-frame pass: E 1m -> forward 1m -> Q 1m.

    Nav2 is paused by the traffic coordinator before this state machine starts.
    """

    def __init__(self, controller: AMRController, teleop_cfg: dict[str, Any], publish_status: Callable[[dict[str, Any]], None]) -> None:
        self.controller = controller
        self.teleop_cfg = teleop_cfg
        self.publish_status = publish_status
        self.active = False
        self.phase = "IDLE"
        self.request_id = ""
        self.segment_start: Gf.Vec3d | None = None
        self.lateral_distance_m = 1.0
        self.forward_distance_m = 1.0

    def _status(self, state: str, **extra: Any) -> None:
        payload = {
            "request_id": self.request_id,
            "robot": self.controller.namespace,
            "state": state,
            "phase": self.phase,
            "timestamp": time.time(),
        }
        payload.update(extra)
        self.publish_status(payload)

    def _start_segment(self, phase: str) -> None:
        self.phase = phase
        self.segment_start = world_position(self.controller.base_prim)
        self._status(phase)

    def _segment_distance(self) -> float:
        if self.segment_start is None:
            return 0.0
        current = world_position(self.controller.base_prim)
        return math.hypot(
            float(current[0] - self.segment_start[0]),
            float(current[1] - self.segment_start[1]),
        )

    def start(self, payload: dict[str, Any]) -> None:
        request_id = str(payload.get("request_id", "")).strip()
        if not request_id:
            return
        if self.active:
            self._status("REJECTED_BUSY", reason="passing maneuver already active")
            return
        self.request_id = request_id
        self.lateral_distance_m = max(0.1, float(payload.get("lateral_distance_m", 1.0)))
        self.forward_distance_m = max(0.1, float(payload.get("forward_distance_m", 1.0)))
        self.controller.halt()
        self.active = True
        self._start_segment("E_RIGHT")
        print(
            f"[{self.controller.name} TRAFFIC PASS] START: E {self.lateral_distance_m:.2f}m "
            f"-> FWD {self.forward_distance_m:.2f}m -> Q {self.lateral_distance_m:.2f}m"
        )

    def cancel(self, reason: str = "cancelled") -> None:
        if not self.active:
            return
        self.controller.halt()
        self.active = False
        self.phase = "FAILED"
        self._status("FAILED", reason=reason)
        print(f"[{self.controller.name} TRAFFIC PASS] FAILED: {reason}")

    def update(self, estop: bool = False) -> tuple[float, float, float] | None:
        if not self.active:
            return None
        if estop:
            self.cancel("emergency stop")
            return (0.0, 0.0, 0.0)

        lateral_speed = abs(float(self.teleop_cfg.get("lateral_speed_mps", 0.35)))
        forward_speed = abs(float(self.teleop_cfg.get("linear_speed_mps", 0.45)))

        if self.phase == "E_RIGHT":
            if self._segment_distance() >= self.lateral_distance_m:
                self.controller.halt()
                self._start_segment("FORWARD")
                return (0.0, 0.0, 0.0)
            return (0.0, -lateral_speed, 0.0)  # manual E

        if self.phase == "FORWARD":
            if self._segment_distance() >= self.forward_distance_m:
                self.controller.halt()
                self._start_segment("Q_LEFT")
                return (0.0, 0.0, 0.0)
            return (forward_speed, 0.0, 0.0)

        if self.phase == "Q_LEFT":
            if self._segment_distance() >= self.lateral_distance_m:
                self.controller.halt()
                self.active = False
                self.phase = "COMPLETE"
                self._status("COMPLETE")
                print(f"[{self.controller.name} TRAFFIC PASS] COMPLETE -> waiting for Nav2 resume")
                return (0.0, 0.0, 0.0)
            return (0.0, lateral_speed, 0.0)  # manual Q

        self.cancel(f"unknown phase {self.phase}")
        return (0.0, 0.0, 0.0)


class IsaacRosNode:
    """Default-message ROS node inside Isaac Sim's Python 3.11 runtime."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import Bool, String
            from std_srvs.srv import Trigger
        except Exception as exc:
            raise RuntimeError(
                "Isaac internal rclpy import failed. Run with ./03_run_isaac.sh and do not source /opt/ros/humble in that terminal."
            ) from exc

        self.rclpy = rclpy
        self.Bool = Bool
        self.String = String
        self.Trigger = Trigger
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node: Node = Node("hospital_amr_isaac_bridge")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publishers: dict[tuple[str, str], Any] = {}
        self.bed_state_publishers: dict[str, Any] = {}
        self.magnet_status_publishers: dict[str, Any] = {}
        self.result_handlers: dict[str, Callable[[dict[str, Any]], None]] = {}
        self.aruco_result_handlers: dict[str, Callable[[dict[str, Any]], None]] = {}
        self.auto_command_queues: dict[str, list[dict[str, Any]]] = {}
        self.magnet_command_queues: dict[str, list[dict[str, Any]]] = {}
        self.traffic_maneuver_queues: dict[str, list[dict[str, Any]]] = {}
        self.traffic_maneuver_status_publishers: dict[str, Any] = {}
        self.elevator_arrival_commands: list[dict[str, Any]] = []
        self.elevator_map_acks: list[dict[str, Any]] = []
        self.world_pose_publishers: dict[str, Any] = {}
        self.elevator_arrived_publishers: dict[str, Any] = {}
        self.elevator_services: list[Any] = []
        self.elevator_service_reserved = False
        self.elevator_service_owner = ""
        self.elevator_busy_checker: Callable[[], bool] = lambda: False
        ros_cfg = cfg["ros2"]
        elevator_cfg = cfg.get("elevator", {})
        self.elevator_cfg = dict(elevator_cfg)

        bed_qos = QoSProfile(depth=1)
        bed_qos.reliability = ReliabilityPolicy.RELIABLE
        bed_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        for unit in cfg["fleet"]:
            ns = str(unit["namespace"])
            request_topic = f"/{ns}/{ros_cfg['request_topic_suffix']}"
            control_topic = f"/{ns}/{ros_cfg['control_topic_suffix']}"
            status_topic = f"/{ns}/{ros_cfg['status_topic_suffix']}"
            result_topic = f"/{ns}/{ros_cfg['result_topic_suffix']}"
            aruco_result_topic = f"/{ns}/aruco/result"
            auto_command_topic = f"/{ns}/auto_approach/command"
            magnet_command_topic = f"/{ns}/magnet/command"
            magnet_status_topic = f"/{ns}/magnet/status"
            traffic_maneuver_command_topic = f"/{ns}/traffic_maneuver/command"
            traffic_maneuver_status_topic = f"/{ns}/traffic_maneuver/status"
            self.auto_command_queues[ns] = []
            self.magnet_command_queues[ns] = []
            self.traffic_maneuver_queues[ns] = []
            self.publishers[(ns, "request")] = self.node.create_publisher(String, request_topic, qos)
            self.publishers[(ns, "control")] = self.node.create_publisher(String, control_topic, qos)
            self.publishers[(ns, "status")] = self.node.create_publisher(String, status_topic, qos)
            self.magnet_status_publishers[ns] = self.node.create_publisher(String, magnet_status_topic, qos)
            self.traffic_maneuver_status_publishers[ns] = self.node.create_publisher(
                String, traffic_maneuver_status_topic, qos
            )
            bed_state_topic = f"/{ns}/bed_attached"
            self.bed_state_publishers[ns] = self.node.create_publisher(Bool, bed_state_topic, bed_qos)
            self.world_pose_publishers[ns] = self.node.create_publisher(String, f"/{ns}/world_pose", qos)
            self.elevator_arrived_publishers[ns] = self.node.create_publisher(
                Bool, f"/{ns}/elevator/arrived", bed_qos
            )
            up_suffix = str(elevator_cfg.get("request_up_service_suffix", "elevator/request_up")).lstrip("/")
            down_suffix = str(elevator_cfg.get("request_down_service_suffix", "elevator/request_down")).lstrip("/")
            up_service = f"/{ns}/{up_suffix}"
            down_service = f"/{ns}/{down_suffix}"
            self.elevator_services.append(
                self.node.create_service(
                    Trigger, up_service,
                    lambda request, response, namespace=ns: self._on_elevator_service(
                        namespace, "up", request, response
                    ),
                )
            )
            self.elevator_services.append(
                self.node.create_service(
                    Trigger, down_service,
                    lambda request, response, namespace=ns: self._on_elevator_service(
                        namespace, "down", request, response
                    ),
                )
            )
            self.node.create_subscription(
                String,
                result_topic,
                lambda msg, namespace=ns: self._on_result(namespace, msg.data),
                qos,
            )
            self.node.create_subscription(
                String,
                aruco_result_topic,
                lambda msg, namespace=ns: self._on_aruco_result(namespace, msg.data),
                qos,
            )
            self.node.create_subscription(
                String,
                auto_command_topic,
                lambda msg, namespace=ns: self._queue_json(self.auto_command_queues[namespace], msg.data, "AUTO"),
                qos,
            )
            self.node.create_subscription(
                String,
                magnet_command_topic,
                lambda msg, namespace=ns: self._queue_json(self.magnet_command_queues[namespace], msg.data, "MAGNET"),
                qos,
            )
            self.node.create_subscription(
                String,
                traffic_maneuver_command_topic,
                lambda msg, namespace=ns: self._queue_json(
                    self.traffic_maneuver_queues[namespace], msg.data, "TRAFFIC MANEUVER"
                ),
                qos,
            )
            print(f"[ROS2 PUB] {request_topic}")
            print(f"[ROS2 SUB] {result_topic}")
            print(f"[ROS2 SUB] {aruco_result_topic}")
            print(f"[ROS2 PUB] {control_topic}")
            print(f"[ROS2 PUB] {status_topic}")
            print(f"[ROS2 SUB] {auto_command_topic}")
            print(f"[ROS2 SUB] {magnet_command_topic}")
            print(f"[ROS2 PUB] {magnet_status_topic}")
            print(f"[ROS2 SUB] {traffic_maneuver_command_topic}")
            print(f"[ROS2 PUB] {traffic_maneuver_status_topic}")
            print(f"[ROS2 PUB] /{ns}/bed_attached (transient local)")
            print(f"[ROS2 PUB] /{ns}/world_pose")
            print(f"[ROS2 PUB] /{ns}/elevator/arrived (transient local)")
            print(f"[ROS2 SERVICE] /{ns}/{up_suffix}")
            print(f"[ROS2 SERVICE] /{ns}/{down_suffix}")

        arrival_topic = str(elevator_cfg.get("arrival_command_topic", "/elevator/amr_arrived"))
        status_topic = str(elevator_cfg.get("status_topic", "/elevator/status"))
        ack_topic = str(elevator_cfg.get("map_switch_ack_topic", "/elevator/map_switch_ack"))
        self.elevator_status_publisher = self.node.create_publisher(String, status_topic, qos)
        self.node.create_subscription(
            String,
            arrival_topic,
            lambda msg: self._queue_json(self.elevator_arrival_commands, msg.data, "ELEVATOR ARRIVAL"),
            qos,
        )
        self.node.create_subscription(
            String,
            ack_topic,
            lambda msg: self._queue_json(self.elevator_map_acks, msg.data, "ELEVATOR MAP ACK"),
            qos,
        )
        print(f"[ROS2 SUB] {arrival_topic}")
        print(f"[ROS2 PUB] {status_topic}")
        print(f"[ROS2 SUB] {ack_topic}")

    def set_elevator_busy_checker(self, checker: Callable[[], bool]) -> None:
        self.elevator_busy_checker = checker

    def release_elevator_service_reservation(self) -> None:
        if self.elevator_service_reserved:
            print(f"[ELEVATOR SERVICE] RELEASE owner={self.elevator_service_owner or 'unknown'}")
        self.elevator_service_reserved = False
        self.elevator_service_owner = ""

    def _on_elevator_service(self, namespace: str, direction: str, _request: Any, response: Any) -> Any:
        namespace = str(namespace).strip().lower()
        direction = str(direction).strip().lower()
        busy = bool(self.elevator_service_reserved or self.elevator_busy_checker())
        if busy:
            response.success = False
            response.message = json.dumps({
                "accepted": False,
                "state": "BUSY",
                "owner": self.elevator_service_owner,
                "robot": namespace,
                "direction": direction,
            }, ensure_ascii=False, separators=(",", ":"))
            print(f"[ELEVATOR SERVICE] BUSY reject robot={namespace} direction={direction}")
            return response

        request_id = f"elevator-{namespace}-{direction}-{int(time.time() * 1000)}"
        command = "START_DOWN" if direction == "down" else "START_UP"
        payload: dict[str, Any] = {
            "command": command,
            "request_id": request_id,
            "robot": namespace,
        }
        if direction == "down":
            payload.update({
                "return_entry_distance_m": float(self.elevator_cfg.get("return_entry_distance_m", 10.0)),
                "return_entry_speed_mps": float(self.elevator_cfg.get("return_entry_speed_mps", 0.50)),
                "return_exit_distance_m": float(self.elevator_cfg.get("return_exit_distance_m", 5.0)),
                "return_exit_speed_mps": float(self.elevator_cfg.get("return_exit_speed_mps", 0.50)),
            })
        self.elevator_service_reserved = True
        self.elevator_service_owner = namespace
        self.elevator_arrival_commands.append(payload)
        response.success = True
        response.message = json.dumps({
            "accepted": True,
            "request_id": request_id,
            "robot": namespace,
            "direction": direction,
        }, ensure_ascii=False, separators=(",", ":"))
        print(f"[ELEVATOR SERVICE] ACCEPT robot={namespace} direction={direction} request_id={request_id}")
        return response

    @staticmethod
    def _queue_json(queue: list[dict[str, Any]], data: str, label: str) -> None:
        try:
            payload = json.loads(data)
            if not isinstance(payload, dict):
                raise ValueError("payload is not an object")
        except Exception as exc:
            print(f"[ROS2 {label}] invalid command: {exc}: {data[:160]}", file=sys.stderr)
            return
        queue.append(payload)

    def pop_auto_command(self, namespace: str) -> dict[str, Any] | None:
        queue = self.auto_command_queues.get(namespace, [])
        return queue.pop(0) if queue else None

    def pop_magnet_command(self, namespace: str) -> dict[str, Any] | None:
        queue = self.magnet_command_queues.get(namespace, [])
        return queue.pop(0) if queue else None

    def pop_traffic_maneuver_command(self, namespace: str) -> dict[str, Any] | None:
        queue = self.traffic_maneuver_queues.get(namespace, [])
        return queue.pop(0) if queue else None

    def publish_traffic_maneuver_status(self, namespace: str, payload: dict[str, Any]) -> None:
        publisher = self.traffic_maneuver_status_publishers.get(namespace)
        if publisher is None:
            return
        msg = self.String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        publisher.publish(msg)

    def pop_elevator_arrival(self) -> dict[str, Any] | None:
        return self.elevator_arrival_commands.pop(0) if self.elevator_arrival_commands else None

    def pop_elevator_map_ack(self) -> dict[str, Any] | None:
        return self.elevator_map_acks.pop(0) if self.elevator_map_acks else None

    def set_result_handler(self, namespace: str, handler: Callable[[dict[str, Any]], None]) -> None:
        self.result_handlers[namespace] = handler

    def set_aruco_result_handler(self, namespace: str, handler: Callable[[dict[str, Any]], None]) -> None:
        self.aruco_result_handlers[namespace] = handler

    def _on_result(self, namespace: str, data: str) -> None:
        try:
            payload = json.loads(data)
        except json.JSONDecodeError:
            print(f"[{namespace}] invalid JSON on OCR result topic: {data[:160]}", file=sys.stderr)
            return
        handler = self.result_handlers.get(namespace)
        if handler:
            handler(payload)

    def _on_aruco_result(self, namespace: str, data: str) -> None:
        try:
            payload = json.loads(data)
        except json.JSONDecodeError:
            print(f"[{namespace}] invalid JSON on ArUco result topic: {data[:160]}", file=sys.stderr)
            return
        handler = self.aruco_result_handlers.get(namespace)
        if handler:
            handler(payload)

    def publish(self, namespace: str, kind: str, payload: dict[str, Any]) -> None:
        msg = self.String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.publishers[(namespace, kind)].publish(msg)

    def publish_magnet_status(self, namespace: str, payload: dict[str, Any]) -> None:
        msg = self.String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.magnet_status_publishers[namespace].publish(msg)

    def publish_elevator_status(self, payload: dict[str, Any]) -> None:
        msg = self.String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.elevator_status_publisher.publish(msg)
        print(f"[ELEVATOR STATUS] {msg.data}")

    def publish_elevator_arrived(self, namespace: str, arrived: bool) -> None:
        publisher = self.elevator_arrived_publishers.get(namespace)
        if publisher is None:
            return
        msg = self.Bool()
        msg.data = bool(arrived)
        publisher.publish(msg)
        print(f"[ROS2 ELEVATOR] /{namespace}/elevator/arrived={bool(arrived)}")

    def publish_world_pose(self, namespace: str, prim: Usd.Prim) -> None:
        publisher = self.world_pose_publishers.get(namespace)
        if publisher is None:
            return
        position = world_position(prim)
        msg = self.String()
        msg.data = json.dumps(
            {
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2]),
                "yaw": float(world_yaw(prim)),
                "timestamp": time.time(),
            },
            separators=(",", ":"),
        )
        publisher.publish(msg)

    def publish_bed_attached(self, namespace: str, attached: bool) -> None:
        msg = self.Bool()
        msg.data = bool(attached)
        self.bed_state_publishers[namespace].publish(msg)
        print(f"[ROS2 MAGNET] /{namespace}/bed_attached={bool(attached)}")

    def spin_once(self) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self) -> None:
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()

class AutomaticElevatorSequence:
    """ROS-arrival-triggered bidirectional 1F<->2F elevator sequence."""

    def __init__(
        self,
        elevator: MapOnlyElevator,
        controller: AMRController,
        ros_node: IsaacRosNode,
        config: dict[str, Any],
    ) -> None:
        self.elevator = elevator
        self.controller = controller
        self.ros_node = ros_node
        self.config = dict(config)
        self.active = False
        self.phase = "IDLE"
        self.request_id = ""
        self.direction = "UP"
        self.command = (0.0, 0.0, 0.0)
        self.phase_started = 0.0
        self.start_xy = (0.0, 0.0)
        self.previous_yaw = 0.0
        self.rotation_progress = 0.0
        # 2F elevator cabin is reached by driving toward world +Y.
        # Use an absolute yaw target so Nav2 arrival heading cannot make entry random.
        self.rotation_target_yaw = math.pi * 0.5
        self.map_ack_received = False
        self.relock_attempted_2f = False
        self.relock_attempted_1f = False
        self.return_entry_distance_m = 10.0
        self.return_entry_speed_mps = 0.25
        # Elevator boarding completion is coordinate-based on both floors.
        # Keep the old distance fields only for backward-compatible payload/config parsing.
        self.boarding_target_y = float(self.config.get("boarding_target_y", 25.91))
        self.return_boarding_target_y = float(self.config.get("return_boarding_target_y", self.boarding_target_y))
        self.return_exit_distance_m = 4.0
        self.return_exit_speed_mps = 0.25
        self.return_exit_target_y = float(self.config.get("return_exit_target_y", 21.487224578857422))

    def owns(self, controller: AMRController) -> bool:
        return bool(self.active and controller is self.controller)

    def _status(self, state: str, **extra: Any) -> None:
        payload = {
            "request_id": self.request_id,
            "robot": self.controller.namespace,
            "state": state,
            "elevator_state": getattr(self.elevator, "state", "UNKNOWN"),
            "direction": self.direction.lower(),
            "timestamp": time.time(),
        }
        payload.update(extra)
        self.ros_node.publish_elevator_status(payload)

    def _set_phase(self, phase: str, **status_extra: Any) -> None:
        self.phase = phase
        self.phase_started = time.monotonic()
        self._status(phase, **status_extra)

    def _current_xy(self) -> tuple[float, float]:
        position = world_position(self.controller.base_prim)
        return float(position[0]), float(position[1])

    def _distance(self) -> float:
        x, y = self._current_xy()
        return math.hypot(x - self.start_xy[0], y - self.start_xy[1])

    def _boarding_y_reached(self, target_y: float) -> tuple[bool, float]:
        """Return True once the AMR crosses the requested world-Y boarding coordinate.

        The driving direction itself is never changed here.  The start-side test makes the
        same rule work whether the AMR begins below or above the target Y.
        """
        _x, current_y = self._current_xy()
        start_y = float(self.start_xy[1])
        target_y = float(target_y)
        if start_y <= target_y:
            return current_y >= target_y, current_y
        return current_y <= target_y, current_y

    def _stabilize_amr_and_bed(self) -> None:
        """Remove residual motion before the elevator transport joint is created."""
        self.command = (0.0, 0.0, 0.0)
        self.controller.halt()
        MagneticDockController._zero_velocity(self.controller.base_prim)

        bed_body_path = self.controller.magnet.attached_body
        if not bed_body_path:
            return
        bed_body = self.controller.stage.GetPrimAtPath(bed_body_path)
        if not bed_body or not bed_body.IsValid():
            return

        MagneticDockController._zero_velocity(bed_body)
        try:
            with Usd.EditContext(self.controller.stage, self.controller.stage.GetSessionLayer()):
                PhysxSchema.PhysxRigidBodyAPI.Apply(bed_body).CreateEnableCCDAttr(True)
            print(f"[ELEVATOR STABILIZE] AMR/bed velocity=0, bed CCD=ON: {bed_body_path}")
        except Exception as exc:
            print(f"[ELEVATOR STABILIZE WARNING] bed CCD setup failed: {exc}")

    def _begin_down_entry_rotation(self) -> None:
        """Run the existing proven 2F absolute-yaw entry sequence."""
        state = str(getattr(self.elevator, "state", ""))
        if state not in {"IDLE_2_CLOSED", "IDLE_2_OPEN"}:
            self.fail(f"elevator is not ready on 2F after empty call: {state}")
            return
        self.controller.set_external_physics_mode(False)
        self.controller.halt()
        self.previous_yaw = world_yaw(self.controller.base_prim)
        self.rotation_progress = 0.0
        self.rotation_target_yaw = math.pi * 0.5
        yaw_error = normalize_angle(self.rotation_target_yaw - self.previous_yaw)
        rotation_speed = abs(float(self.config.get("rotation_speed_rad_s", 0.55)))
        self.command = (0.0, 0.0, math.copysign(rotation_speed, yaw_error))
        print(
            f"[2F ELEVATOR YAW] start={self.previous_yaw:.6f} rad "
            f"target={self.rotation_target_yaw:.6f} rad error={yaw_error:.6f} rad"
        )
        self._set_phase(
            "ROTATING_RIGHT_2F_ENTRY",
            target_yaw_rad=self.rotation_target_yaw,
            target_yaw_deg=90.0,
            direction="face_world_plus_y",
            door_state=state,
        )

    def start(self, payload: dict[str, Any]) -> None:
        if self.active:
            self._status("BUSY", reason=f"already active: {self.phase}")
            return
        robot = str(payload.get("robot", "amr1")).strip().lower()
        if robot not in {self.controller.namespace.lower(), self.controller.name.lower()}:
            self.request_id = str(payload.get("request_id", ""))
            self._status("FAILED", reason=f"unsupported elevator robot: {robot}")
            return

        command = str(payload.get("command", "START_UP")).strip().upper()
        if command not in {"START_UP", "START_DOWN"}:
            self.request_id = str(payload.get("request_id", ""))
            self._status("FAILED", reason=f"unsupported elevator command: {command}")
            return

        self.direction = "DOWN" if command == "START_DOWN" else "UP"
        self.request_id = str(payload.get("request_id", f"elevator-{int(time.time())}"))
        self.active = True
        self.command = (0.0, 0.0, 0.0)
        self.map_ack_received = False
        self.rotation_progress = 0.0
        self.relock_attempted_2f = False
        self.relock_attempted_1f = False
        self.return_entry_distance_m = max(0.1, float(payload.get("return_entry_distance_m", 10.0)))
        self.return_entry_speed_mps = abs(float(payload.get("return_entry_speed_mps", 0.25)))
        self.boarding_target_y = float(self.config.get("boarding_target_y", 25.91))
        self.return_boarding_target_y = float(self.config.get("return_boarding_target_y", self.boarding_target_y))
        self.return_exit_distance_m = max(0.1, float(payload.get("return_exit_distance_m", 4.0)))
        self.return_exit_speed_mps = abs(float(payload.get("return_exit_speed_mps", 0.25)))
        self.return_exit_target_y = float(self.config.get("return_exit_target_y", 21.487224578857422))
        self.ros_node.publish_elevator_arrived(self.controller.namespace, False)

        state = str(getattr(self.elevator, "state", ""))
        if self.direction == "UP":
            if state == "IDLE_1_CLOSED":
                self.elevator.trigger()
                self._set_phase("WAITING_1F_DOOR_OPEN")
            elif state in {"BOARDING_1", "IDLE_1_OPEN"}:
                self.start_xy = self._current_xy()
                self.command = (float(self.config.get("entry_speed_mps", 1.0)), 0.0, 0.0)
                self._set_phase(
                    "DRIVING_IN",
                    target_y=self.boarding_target_y,
                    completion_rule="world_y_coordinate",
                )
            else:
                # No AMR pose reset: wait outside while the EMPTY elevator returns to 1F.
                if self.elevator.call_empty_to_floor(1, open_on_arrival=True):
                    self.controller.halt()
                    self._set_phase("WAITING_EMPTY_TO_1F", elevator_state=state)
                else:
                    self.fail(f"cannot call empty elevator to 1F from state: {state}")
            return

        # DOWN: if the car is already on 2F, keep the existing rotation/entry logic.
        if state in {"IDLE_2_CLOSED", "IDLE_2_OPEN"}:
            self._begin_down_entry_rotation()
            return

        # Otherwise wait outside while the EMPTY car comes from 1F to 2F. Keep the
        # 2F door closed because the existing sequence opens it only after +90deg yaw.
        if self.elevator.call_empty_to_floor(2, open_on_arrival=False):
            self.controller.halt()
            self._set_phase("WAITING_EMPTY_TO_2F", elevator_state=state)
            return
        self.fail(f"cannot call empty elevator to 2F from state: {state}")

    def fail(self, reason: str) -> None:
        self.command = (0.0, 0.0, 0.0)
        self.controller.halt()
        self._status("FAILED", reason=reason, phase=self.phase)
        self.active = False
        self.phase = "FAILED"

    def _check_timeout(self) -> bool:
        timeout = float(self.config.get("phase_timeout_s", 90.0))
        if time.monotonic() - self.phase_started <= timeout:
            return False
        self.fail(f"phase timeout: {self.phase}")
        return True

    def update(self, dt: float) -> None:
        del dt
        if not self.active or self._check_timeout():
            return
        elevator_state = str(getattr(self.elevator, "state", ""))
        if elevator_state == "ERROR":
            self.fail("physical elevator entered ERROR")
            return

        if self.phase == "WAITING_EMPTY_TO_1F":
            self.command = (0.0, 0.0, 0.0)
            self.controller.halt()
            if elevator_state in {"BOARDING_1", "IDLE_1_OPEN"}:
                self.start_xy = self._current_xy()
                self.command = (float(self.config.get("entry_speed_mps", 1.0)), 0.0, 0.0)
                self._set_phase(
                    "DRIVING_IN",
                    target_y=self.boarding_target_y,
                    completion_rule="world_y_coordinate",
                    empty_call_complete=True,
                )
            return

        if self.phase == "WAITING_EMPTY_TO_2F":
            self.command = (0.0, 0.0, 0.0)
            self.controller.halt()
            if elevator_state in {"IDLE_2_CLOSED", "IDLE_2_OPEN"}:
                self._begin_down_entry_rotation()
            return

        # Existing 1F -> 2F sequence is intentionally kept unchanged.
        if self.phase == "WAITING_1F_DOOR_OPEN":
            if elevator_state in {"BOARDING_1", "IDLE_1_OPEN"}:
                self.start_xy = self._current_xy()
                self.command = (float(self.config.get("entry_speed_mps", 1.0)), 0.0, 0.0)
                self._set_phase(
                    "DRIVING_IN",
                    target_y=self.boarding_target_y,
                    completion_rule="world_y_coordinate",
                )

        elif self.phase == "DRIVING_IN":
            reached, current_y = self._boarding_y_reached(self.boarding_target_y)
            if reached:
                moved = self._distance()
                self._stabilize_amr_and_bed()
                self._set_phase(
                    "SETTLING_INSIDE",
                    target_y=self.boarding_target_y,
                    actual_y=current_y,
                    actual_entry_distance_m=moved,
                    settle_s=float(self.config.get("boarding_settle_s", 2.0)),
                )

        elif self.phase == "SETTLING_INSIDE":
            self.command = (0.0, 0.0, 0.0)
            self.controller.halt()
            settle_s = float(self.config.get("boarding_settle_s", 2.0))
            if time.monotonic() - self.phase_started >= settle_s:
                self._stabilize_amr_and_bed()
                self.elevator.trigger()
                new_state = str(getattr(self.elevator, "state", ""))
                if new_state in {"BOARDING_1", "IDLE_1_OPEN"}:
                    self.fail(f"AMR did not enter the elevator capture area at target Y={self.boarding_target_y:.2f}")
                    return
                self._set_phase("RIDING_TO_2F")

        elif self.phase == "RIDING_TO_2F":
            self.command = (0.0, 0.0, 0.0)
            if elevator_state == "IDLE_2_OPEN" and not self.relock_attempted_2f:
                self.relock_attempted_2f = True
                locked = self.controller.ensure_magnet_lock()
                self.ros_node.publish_bed_attached(self.controller.namespace, bool(locked))
                if not locked:
                    self.fail("2F automatic C bed lock failed")
                    return
                self._set_phase("RELOCKING_2F", action="AUTO_C")

        elif self.phase == "RELOCKING_2F":
            self.command = (0.0, 0.0, 0.0)
            if not self.controller.motion_hold_active:
                self._set_phase("MAP_SWITCH_REQUIRED", floor="2f")

        # 2F return path: this wait happens ONLY AFTER the right 90-degree rotation.
        elif self.phase == "WAITING_2F_DOOR_OPEN_AFTER_ROTATION":
            self.command = (0.0, 0.0, 0.0)
            self.controller.halt()
            if elevator_state == "IDLE_2_OPEN":
                self.start_xy = self._current_xy()
                self.command = (self.return_entry_speed_mps, 0.0, 0.0)
                self._set_phase(
                    "DRIVING_IN_2F",
                    target_y=self.return_boarding_target_y,
                    completion_rule="world_y_coordinate",
                    direction="forward_after_right_90_and_door_open",
                )

        # 2F return path: ignore the Nav2 arrival heading and face the cabin at
        # absolute world yaw +90 deg (+Y).  This removes the old accumulated
        # relative-turn dependency that could leave the entry angle inconsistent.
        elif self.phase == "ROTATING_RIGHT_2F_ENTRY":
            self.controller.set_external_physics_mode(False)
            current_yaw = world_yaw(self.controller.base_prim)
            yaw_error = normalize_angle(self.rotation_target_yaw - current_yaw)
            yaw_tolerance = math.radians(2.0)

            if abs(yaw_error) <= yaw_tolerance:
                self.command = (0.0, 0.0, 0.0)
                self.controller.halt()
                print(
                    f"[2F ELEVATOR YAW] aligned current={current_yaw:.6f} rad "
                    f"target={self.rotation_target_yaw:.6f} rad"
                )
                # Rotation is complete.  The 2F door is opened only now.
                if elevator_state == "IDLE_2_CLOSED":
                    self.elevator.trigger()
                    self._set_phase(
                        "WAITING_2F_DOOR_OPEN_AFTER_ROTATION",
                        target_y=self.return_boarding_target_y,
                        sequence="absolute_yaw90_complete_then_open",
                    )
                elif elevator_state == "IDLE_2_OPEN":
                    self.start_xy = self._current_xy()
                    self.command = (self.return_entry_speed_mps, 0.0, 0.0)
                    self._set_phase(
                        "DRIVING_IN_2F",
                        target_y=self.return_boarding_target_y,
                        completion_rule="world_y_coordinate",
                        direction="forward_after_absolute_yaw90_and_door_open",
                    )
                else:
                    self.fail(f"unexpected 2F door state after yaw alignment: {elevator_state}")
            else:
                max_speed = abs(float(self.config.get("rotation_speed_rad_s", 0.55)))
                min_speed = min(0.18, max_speed)
                requested_speed = min(max_speed, max(min_speed, abs(yaw_error) * 1.2))
                # Sign comes from the absolute yaw error.  From the normal MRI-return
                # arrival heading this is the requested RIGHT turn, but it still
                # recovers correctly if Nav2 arrives a few degrees off.
                self.command = (0.0, 0.0, math.copysign(requested_speed, yaw_error))
                self.previous_yaw = current_yaw

        elif self.phase == "DRIVING_IN_2F":
            reached, current_y = self._boarding_y_reached(self.return_boarding_target_y)
            if reached:
                moved = self._distance()
                self._stabilize_amr_and_bed()
                self._set_phase(
                    "SETTLING_INSIDE_2F",
                    target_y=self.return_boarding_target_y,
                    actual_y=current_y,
                    actual_entry_distance_m=moved,
                    settle_s=float(self.config.get("boarding_settle_s", 2.0)),
                )

        elif self.phase == "SETTLING_INSIDE_2F":
            self.command = (0.0, 0.0, 0.0)
            self.controller.halt()
            settle_s = float(self.config.get("boarding_settle_s", 2.0))
            if time.monotonic() - self.phase_started >= settle_s:
                self._stabilize_amr_and_bed()
                self.elevator.trigger()
                new_state = str(getattr(self.elevator, "state", ""))
                if new_state == "IDLE_2_OPEN":
                    self.fail(
                        f"AMR did not enter the elevator capture area at "
                        f"target Y={self.return_boarding_target_y:.2f} after right 90deg"
                    )
                    return
                self._set_phase("RIDING_TO_1F")

        elif self.phase == "RIDING_TO_1F":
            self.command = (0.0, 0.0, 0.0)
            if elevator_state == "IDLE_1_OPEN" and not self.relock_attempted_1f:
                self.relock_attempted_1f = True
                locked = self.controller.ensure_magnet_lock()
                self.ros_node.publish_bed_attached(self.controller.namespace, bool(locked))
                if not locked:
                    self.fail("1F automatic C bed lock failed")
                    return
                self._set_phase("RELOCKING_1F", action="AUTO_C")

        elif self.phase == "RELOCKING_1F":
            self.command = (0.0, 0.0, 0.0)
            if not self.controller.motion_hold_active:
                self._set_phase("MAP_SWITCH_REQUIRED", floor="1f")

        elif self.phase == "MAP_SWITCH_REQUIRED":
            ack = self.ros_node.pop_elevator_map_ack()
            if ack is None:
                return
            ack_request = str(ack.get("request_id", ""))
            if ack_request and ack_request != self.request_id:
                return
            expected_floor = "2f" if self.direction == "UP" else "1f"
            ack_floor = str(ack.get("floor", expected_floor)).strip().lower() or expected_floor
            if ack_floor != expected_floor:
                self.fail(f"unexpected map ack floor: {ack_floor}, expected {expected_floor}")
                return
            if not bool(ack.get("success", False)):
                self.fail(str(ack.get("reason", f"{expected_floor} map switch failed")))
                return

            self.start_xy = self._current_xy()
            if self.direction == "UP":
                self.command = (-abs(float(self.config.get("exit_reverse_speed_mps", 0.5))), 0.0, 0.0)
                self._set_phase(
                    "REVERSING_OUT",
                    target_distance_m=float(self.config.get("exit_reverse_distance_m", 3.0)),
                )
            else:
                # 1F door is open.  Stay in direct elevator control and reverse out
                # until the original 1F elevator external Y coordinate is reached.
                self.command = (-abs(self.return_exit_speed_mps), 0.0, 0.0)
                self._set_phase(
                    "DRIVING_OUT_1F",
                    target_y=self.return_exit_target_y,
                    completion_rule="world_y_coordinate",
                    direction="reverse_to_1f_elevator_coordinate",
                )

        elif self.phase == "REVERSING_OUT":
            moved = self._distance()
            if moved >= float(self.config.get("exit_reverse_distance_m", 3.0)):
                self.command = (0.0, 0.0, 0.0)
                self.controller.halt()
                # 1F->2F 하차 완료 후 문이 열린 채 남지 않게 닫는다.
                # DOWN 복귀 SERVICE가 나중에 들어오면 그때 다시 2F 문을 연다.
                door_close_started = bool(self.elevator.close_idle_door(2))
                self.ros_node.publish_elevator_arrived(self.controller.namespace, True)
                self._status(
                    "COMPLETE",
                    floor="2f",
                    actual_reverse_distance_m=moved,
                    door_close_started=door_close_started,
                )
                self.active = False
                self.phase = "COMPLETE"

        elif self.phase == "DRIVING_OUT_1F":
            reached, current_y = self._boarding_y_reached(self.return_exit_target_y)
            if reached:
                moved = self._distance()
                self.command = (0.0, 0.0, 0.0)
                self.controller.halt()
                door_close_started = bool(self.elevator.close_idle_door(1))
                self.ros_node.publish_elevator_arrived(self.controller.namespace, True)
                self._status(
                    "COMPLETE",
                    floor="1f",
                    target_y=self.return_exit_target_y,
                    actual_y=current_y,
                    actual_reverse_distance_m=moved,
                    door_close_started=door_close_started,
                    nav2_resume_point="1f_elevator_coordinate",
                )
                self.active = False
                self.phase = "COMPLETE"


def print_help() -> None:
    print("\n================ KIMSEOUL AMR1 MISSION ================")
    print("AMR1 teleop : W/S forward, A/D turn, Q/E lateral, R/V lift, SPACE stop")
    print("AMR1 magnet : C = nearby bed LOCK, X = RELEASE, M = measure")
    print("Mission     : patient_transport_manager.py selects patient -> OCR/X align -> patient-specific forward")
    print("Elevator    : SERVICE; UP existing Y-board/5m exit+2F door close, DOWN 2F door open->right90->Y-board->1F/5m exit")
    print("O           : disabled for elevator (ROS elevator services replace O)")
    print("H           : show help")
    if bool(CFG.get("nav2", {}).get("enabled", False)):
        print("Nav2 AMR1   : /cmd_vel, /odom, /scan, frames odom/base_link/base_scan")
        print("Nav2 AMR2   : /amr2/cmd_vel, /amr2/odom, /amr2/scan, amr2/* frames")
        print("Nav2 is forward-only; room reverse is always half of the selected patient forward distance.")
    print("Only image X is aligned; image Y is ignored.")
    print("PaddleOCR always runs in the separate ROS 2 launch terminal.")
    print("========================================================\n")

def main() -> int:
    enable_extension("isaacsim.ros2.bridge")
    if bool(CFG.get("nav2", {}).get("enabled", False)):
        enable_extension("isaacsim.sensors.physx")

    stage_path = (
        Path(ARGS.stage).expanduser().resolve()
        if ARGS.stage
        else PROJECT_ROOT / str(CFG["project"]["stage"])
    )
    if not stage_path.exists():
        raise FileNotFoundError(stage_path)
    context = omni.usd.get_context()
    if not context.open_stage(str(stage_path)):
        raise RuntimeError(f"Could not open stage: {stage_path}")
    for _ in range(120):
        APP.update()
    stage = context.get_stage()
    try:
        replace_amr2_with_amr1_clone(stage, CFG)
    except Exception as exc:
        fallback = stage.GetPrimAtPath("/World/AMR2")
        required_fallback = (
            "/World/AMR2/base_link",
            "/World/AMR2/Joints/lift_joint",
            "/World/AMR2/Joints/wheel_joint_FL",
            "/World/AMR2/Joints/wheel_joint_FR",
            "/World/AMR2/Joints/wheel_joint_RL",
            "/World/AMR2/Joints/wheel_joint_RR",
        )
        if (
            not fallback
            or not fallback.IsValid()
            or any(not stage.GetPrimAtPath(path).IsValid() for path in required_fallback)
        ):
            raise
        print(
            f"[AMR2 CLONE WARNING] runtime clone failed: {exc}. "
            "Using the existing /World/AMR2 with the same AMR1 software/Nav2 configuration."
        )
    required = ["/World/HospitalMap"] + [str(unit["root_path"]) for unit in CFG["fleet"]]
    missing = [path for path in required if not stage.GetPrimAtPath(path).IsValid()]
    if missing:
        raise RuntimeError(f"Stage is missing required prims: {missing}")

    # Requested floor-only fix. Nav2/local costmaps/drive/elevator motion code remains untouched.
    apply_requested_floor_fix(
        stage,
        CFG.get("requested_floor_fix", {}),
        CFG.get("elevator", {}),
    )

    # Requested docking-station visual-only override.  Geometry/transform stays exactly as authored;
    # only collision/rigid-body response is disabled for these two prim trees.
    disable_visual_only_physics(
        stage,
        (
            "/World/AMR_DockingStation_v1_0_1__01",
            "/World/AMR_DockingStation_v1_0_1_",
        ),
    )

    # Selected upgrade features only. These create/non-driving-adjust the requested MRI scene items.
    apply_mri_scene_position(stage, CFG.get("mri_scene_position", {}))
    ensure_fixed_mri_target(stage, CFG.get("fixed_mri_target", {}))
    create_amr_follow_camera(stage, CFG.get("follow_camera", {}))
    schedule_hospital_staff(stage, PROJECT_ROOT, CFG.get("hospital_staff", {}))
    enable_patient_transfer_extension(
        PROJECT_ROOT,
        CFG.get("patient_mri", {}),
        enable_extension,
        APP.update,
    )

    # Use only the door/lift implementation copied from the supplied elevator ZIP.
    elevator_cfg = dict(CFG.get("elevator", {}))
    try:
        map_elevator = MapOnlyElevator(stage, elevator_cfg)
    except Exception as exc:
        map_elevator = DisabledElevator(exc)

    configured_beds = list(CFG.get("magnetic_dock", {}).get("candidate_bed_paths", []))
    bed_keywords = list(CFG.get("magnetic_dock", {}).get("candidate_name_keywords", []))
    candidate_bed_paths = discover_bed_roots(stage, configured_beds, bed_keywords)
    # Render-only ArUco cards. Bed poses, original nameplates, physics and docking are untouched.
    aruco_cards = install_aruco_markers(stage, PROJECT_ROOT, CFG, candidate_bed_paths)
    print(f"[ARUCO READY] separate_cards={len(aruco_cards)} expected=6 original_nameplates=UNCHANGED")
    claimed_beds: set[str] = set()
    print(f"[magnet] discovered bed roots ({len(candidate_bed_paths)}): {candidate_bed_paths or 'NONE'}")
    controllers = [
        AMRController(stage, unit, CFG, candidate_bed_paths, claimed_beds)
        for unit in CFG["fleet"]
    ]
    map_elevator.bind_controllers(controllers)
    cameras = [DirectCamera(str(unit["root_path"]), str(unit["namespace"]), CFG["camera"]) for unit in CFG["fleet"]]
    ros_writers = [attach_ros2_rgb_writer(camera, CFG["ros2"], CFG["camera"]) for camera in cameras]

    ros_node = IsaacRosNode(CFG)
    for controller in controllers:
        ros_node.publish_bed_attached(controller.namespace, False)
        ros_node.publish_elevator_arrived(controller.namespace, False)
    elevator_sequences: dict[str, AutomaticElevatorSequence] = {}
    if hasattr(map_elevator, "state") and controllers:
        for controller in controllers:
            elevator_sequences[controller.namespace.lower()] = AutomaticElevatorSequence(
                map_elevator, controller, ros_node, elevator_cfg
            )
    ros_node.set_elevator_busy_checker(
        lambda: any(sequence.active for sequence in elevator_sequences.values())
    )

    nav2_bridges: list[Any] = []
    bridge_by_controller: dict[int, Any] = {}
    if bool(CFG.get("nav2", {}).get("enabled", False)):
        from nav2_bridge import Nav2Bridge

        nav_common = {
            key: copy.deepcopy(value)
            for key, value in CFG["nav2"].items()
            if key not in {"robots", "enabled"}
        }
        robot_entries = list(CFG["nav2"].get("robots", []))
        if not robot_entries:
            robot_entries = [{"amr_name": "AMR1"}]
        for robot_entry in robot_entries:
            bridge_cfg = copy.deepcopy(nav_common)
            bridge_cfg.update(copy.deepcopy(robot_entry))
            if "lidar" in nav_common:
                merged_lidar = copy.deepcopy(nav_common["lidar"])
                merged_lidar.update(copy.deepcopy(robot_entry.get("lidar", {})))
                bridge_cfg["lidar"] = merged_lidar

            nav_unit_name = str(bridge_cfg.get("amr_name", "AMR1"))
            nav_controller = next(
                (controller for controller in controllers if controller.name == nav_unit_name),
                None,
            )
            if nav_controller is None:
                raise RuntimeError(f"Nav2 AMR controller not found: {nav_unit_name}")
            bridge = Nav2Bridge(stage, nav_controller, ros_node.node, bridge_cfg)
            nav2_bridges.append(bridge)
            bridge_by_controller[id(nav_controller)] = bridge

    approaches: list[AutoApproach] = []
    for controller, unit in zip(controllers, CFG["fleet"]):
        ns = controller.namespace
        approach = AutoApproach(
            controller,
            unit,
            CFG,
            publish_request=lambda payload, namespace=ns: ros_node.publish(namespace, "request", payload),
            publish_control=lambda payload, namespace=ns: ros_node.publish(namespace, "control", payload),
            publish_status=lambda payload, namespace=ns: ros_node.publish(namespace, "status", payload),
        )
        ros_node.set_result_handler(ns, approach.handle_result)
        ros_node.set_aruco_result_handler(ns, approach.handle_aruco_result)
        approaches.append(approach)

    traffic_maneuvers: list[TrafficPassingManeuver] = []
    for controller in controllers:
        ns = controller.namespace
        traffic_maneuvers.append(
            TrafficPassingManeuver(
                controller,
                CFG["teleop"],
                publish_status=lambda payload, namespace=ns: ros_node.publish_traffic_maneuver_status(namespace, payload),
            )
        )

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    app_window = omni.appwindow.get_default_app_window()
    keyboard = app_window.get_keyboard()
    input_interface = carb.input.acquire_input_interface()
    pressed: set[Any] = set()
    request_flags = [False for _ in controllers]
    help_flag = [False]
    magnet_lock_flags = [False for _ in controllers]
    magnet_release_flags = [False for _ in controllers]
    speed_cycle_flag = [False]
    footprint_measure_flags = [False for _ in controllers]
    speed_mode_index = [0]
    speed_multipliers = [float(v) for v in CFG["teleop"].get("speed_multipliers", [1.0, 2.0, 3.0, 0.7])]
    if not speed_multipliers:
        speed_multipliers = [1.0]

    def on_keyboard(event, *_args):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            fresh = event.input not in pressed
            pressed.add(event.input)
            if fresh and event.input == carb.input.KeyboardInput.O:
                print("[ELEVATOR] O key disabled. /elevator/amr_arrived starts the automatic sequence.")
            elif fresh and event.input == carb.input.KeyboardInput.P and len(request_flags) > 1:
                request_flags[1] = True
            elif fresh and event.input == carb.input.KeyboardInput.H:
                help_flag[0] = True
            elif fresh and event.input == carb.input.KeyboardInput.C:
                target = 1 if len(controllers) > 1 and carb.input.KeyboardInput.LEFT_SHIFT in pressed else 0
                magnet_lock_flags[target] = True
            elif fresh and event.input == carb.input.KeyboardInput.X:
                target = 1 if len(controllers) > 1 and carb.input.KeyboardInput.LEFT_SHIFT in pressed else 0
                magnet_release_flags[target] = True
            elif fresh and event.input == carb.input.KeyboardInput.Z:
                speed_cycle_flag[0] = True
            elif fresh and event.input == carb.input.KeyboardInput.M:
                target = 1 if len(controllers) > 1 and carb.input.KeyboardInput.LEFT_SHIFT in pressed else 0
                footprint_measure_flags[target] = True
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            pressed.discard(event.input)
        return True

    keyboard_subscription = input_interface.subscribe_to_keyboard_events(keyboard, on_keyboard)
    print(f"[stage] {stage_path}")
    print(f"[ROS_DOMAIN_ID] {os.environ.get('ROS_DOMAIN_ID', CFG['ros2']['domain_id'])}")
    print_help()

    last = time.monotonic()
    try:
        while APP.is_running():
            now = time.monotonic()
            dt = clamp(now - last, 0.0, 0.05)
            last = now
            ros_node.spin_once()
            map_elevator.update(dt)
            for controller in controllers:
                ros_node.publish_world_pose(controller.namespace, controller.base_prim)
            while True:
                elevator_command = ros_node.pop_elevator_arrival()
                if elevator_command is None:
                    break
                robot = str(elevator_command.get("robot", "amr1")).strip().lower()
                sequence = elevator_sequences.get(robot)
                if sequence is None:
                    ros_node.publish_elevator_status({
                        "request_id": str(elevator_command.get("request_id", "")),
                        "robot": robot,
                        "state": "FAILED",
                        "reason": "physical elevator unavailable for requested robot",
                        "timestamp": time.time(),
                    })
                    ros_node.release_elevator_service_reservation()
                    continue
                other_active = next(
                    (item for item in elevator_sequences.values() if item.active and item is not sequence),
                    None,
                )
                if other_active is not None:
                    ros_node.publish_elevator_status({
                        "request_id": str(elevator_command.get("request_id", "")),
                        "robot": robot,
                        "state": "BUSY",
                        "reason": f"elevator owned by {other_active.controller.namespace}",
                        "timestamp": time.time(),
                    })
                    ros_node.release_elevator_service_reservation()
                    continue
                if not map_elevator.select_controller(sequence.controller):
                    ros_node.publish_elevator_status({
                        "request_id": str(elevator_command.get("request_id", "")),
                        "robot": robot,
                        "state": "BUSY",
                        "reason": "physical elevator owner switch rejected",
                        "timestamp": time.time(),
                    })
                    ros_node.release_elevator_service_reservation()
                    continue
                sequence.start(elevator_command)
            for sequence in elevator_sequences.values():
                sequence.update(dt)
            if (
                ros_node.elevator_service_reserved
                and not ros_node.elevator_arrival_commands
                and not any(sequence.active for sequence in elevator_sequences.values())
            ):
                ros_node.release_elevator_service_reservation()

            if help_flag[0]:
                print_help()
                help_flag[0] = False

            # Mission commands from patient_transport_manager.py.
            for index, (controller, approach) in enumerate(zip(controllers, approaches)):
                auto_command = ros_node.pop_auto_command(controller.namespace)
                if auto_command is not None:
                    command = str(auto_command.get("command", "START")).upper()
                    if command == "START":
                        if approach.active:
                            approach.cancel("restarted by mission manager")
                        patient_name = str(auto_command.get("patient", "")).strip()
                        patient_birth = str(auto_command.get("birth_date", "")).strip()
                        if patient_name:
                            approach.unit["target_name"] = patient_name
                        if patient_birth:
                            approach.unit["target_birth_date"] = patient_birth
                        if "forward_distance_m" in auto_command:
                            approach.auto_cfg["forward_distance_m"] = float(auto_command["forward_distance_m"])
                        print(
                            f"[{controller.name} MISSION TARGET] "
                            f"{approach.unit.get('target_name', '')} / "
                            f"{approach.unit.get('target_birth_date', '')} / "
                            f"forward={float(approach.auto_cfg.get('forward_distance_m', 0.0)):.4f}m"
                        )
                        approach.start()
                    elif command == "CANCEL":
                        approach.cancel("cancelled by mission manager")

                magnet_command = ros_node.pop_magnet_command(controller.namespace)
                if magnet_command is not None:
                    request_id = str(magnet_command.get("request_id", ""))
                    command = str(magnet_command.get("command", "LOCK")).upper()
                    success = False
                    if command in {"LOCK", "C"}:
                        success = controller.request_magnet_lock()
                        if success:
                            ros_node.publish_bed_attached(controller.namespace, True)
                    elif command in {"RELEASE", "X"}:
                        controller.request_magnet_release()
                        ros_node.publish_bed_attached(controller.namespace, False)
                        success = True
                    ros_node.publish_magnet_status(
                        controller.namespace,
                        {
                            "request_id": request_id,
                            "command": command,
                            "success": bool(success),
                            "state": controller.magnet.last_state,
                            "attached_bed": controller.magnet.attached_bed_path or "",
                            "timestamp": time.time(),
                        },
                    )

                traffic_command = ros_node.pop_traffic_maneuver_command(controller.namespace)
                if traffic_command is not None:
                    traffic = traffic_maneuvers[index]
                    action = str(traffic_command.get("command", "START")).upper()
                    if action == "START":
                        if approach.active:
                            ros_node.publish_traffic_maneuver_status(controller.namespace, {
                                "request_id": str(traffic_command.get("request_id", "")),
                                "robot": controller.namespace,
                                "state": "REJECTED_BUSY",
                                "reason": "auto approach active",
                                "timestamp": time.time(),
                            })
                        else:
                            traffic.start(traffic_command)
                    elif action == "CANCEL":
                        traffic.cancel("cancelled by traffic coordinator")

            for index, flag in enumerate(request_flags):
                if flag:
                    approaches[index].start()
                    request_flags[index] = False

            if speed_cycle_flag[0]:
                speed_mode_index[0] = (speed_mode_index[0] + 1) % len(speed_multipliers)
                multiplier = speed_multipliers[speed_mode_index[0]]
                base_linear = float(CFG["teleop"]["linear_speed_mps"])
                print(
                    f"[TELEOP] Z speed mode: x{multiplier:.1f} "
                    f"(forward/reverse max {base_linear * multiplier:.2f} m/s)"
                )
                speed_cycle_flag[0] = False
            for index, controller in enumerate(controllers):
                if magnet_lock_flags[index]:
                    locked = controller.request_magnet_lock()
                    if locked:
                        ros_node.publish_bed_attached(controller.namespace, True)
                    magnet_lock_flags[index] = False
                if footprint_measure_flags[index]:
                    report_combined_footprint(stage, controller, padding_m=0.08)
                    footprint_measure_flags[index] = False
                if magnet_release_flags[index]:
                    controller.request_magnet_release()
                    ros_node.publish_bed_attached(controller.namespace, False)
                    magnet_release_flags[index] = False

            amr1_keys = (
                float(carb.input.KeyboardInput.W in pressed) - float(carb.input.KeyboardInput.S in pressed),
                float(carb.input.KeyboardInput.Q in pressed) - float(carb.input.KeyboardInput.E in pressed),
                float(carb.input.KeyboardInput.A in pressed) - float(carb.input.KeyboardInput.D in pressed),
                float(carb.input.KeyboardInput.R in pressed) - float(carb.input.KeyboardInput.V in pressed),
                carb.input.KeyboardInput.SPACE in pressed,
            )
            amr2_keys = (
                float(carb.input.KeyboardInput.UP in pressed) - float(carb.input.KeyboardInput.DOWN in pressed),
                0.0,
                float(carb.input.KeyboardInput.LEFT in pressed) - float(carb.input.KeyboardInput.RIGHT in pressed),
                float(carb.input.KeyboardInput.LEFT_BRACKET in pressed)
                - float(carb.input.KeyboardInput.RIGHT_BRACKET in pressed),
                carb.input.KeyboardInput.ENTER in pressed,
            )
            key_sets = (amr1_keys, amr2_keys)[: len(controllers)]
            for controller, approach, traffic, keys in zip(controllers, approaches, traffic_maneuvers, key_sets):
                forward_key, lateral_key, yaw_key, lift_key, estop = keys
                elevator_sequence = elevator_sequences.get(controller.namespace.lower())
                if elevator_sequence is not None and elevator_sequence.owns(controller):
                    controller.update(dt, elevator_sequence.command, 0.0, False)
                    continue
                elevator_locked = bool(map_elevator.is_controller_locked(controller))
                if elevator_locked:
                    controller.update(dt, (0.0, 0.0, 0.0), 0.0, True)
                    continue

                traffic_command = traffic.update(bool(estop))
                if traffic_command is not None:
                    # Agreed passing motion has priority only after Nav2 was traffic-paused.
                    # The COMPLETE frame also returns an explicit zero command.
                    command = traffic_command
                    lift = 0.0
                else:
                    auto_command = approach.update(dt, bool(estop))
                    if auto_command is not None and approach.active:
                        command = auto_command
                        lift = 0.0
                    else:
                        manual_active = controller.motion_hold_active or bool(estop) or any(
                            abs(value) > 0.0
                            for value in (forward_key, lateral_key, yaw_key, lift_key)
                        )
                        nav_command = None
                        controller_bridge = bridge_by_controller.get(id(controller))
                        if controller_bridge is not None and not manual_active:
                            nav_command = controller_bridge.get_fresh_command()

                        if controller.motion_hold_active:
                            command = (0.0, 0.0, 0.0)
                            lift = 0.0
                        elif nav_command is not None:
                            # Nav2 limits are already exactly 2x for both robots.
                            # Never multiply again when a bed is attached, or loaded
                            # motion would become 4x and reintroduce judder.
                            command = nav_command
                            lift = 0.0
                        else:
                            speed_multiplier = speed_multipliers[speed_mode_index[0]]
                            command = (
                                forward_key * float(CFG["teleop"]["linear_speed_mps"]) * speed_multiplier,
                                lateral_key * float(CFG["teleop"]["lateral_speed_mps"]) * speed_multiplier,
                                yaw_key * float(CFG["teleop"]["angular_speed_rad_s"]) * speed_multiplier,
                            )
                            lift = lift_key
                controller.update(dt, command, lift, bool(estop))

            APP.update()
            for bridge in nav2_bridges:
                bridge.publish()
    finally:
        for controller in controllers:
            try:
                controller.magnet.release("application shutdown")
                ros_node.publish_bed_attached(controller.namespace, False)
            except Exception as exc:
                print(f"[{controller.name}] magnet shutdown warning: {exc}")
            controller.halt()
        timeline.stop()
        for writer in ros_writers:
            try:
                writer.detach()
            except Exception:
                pass
        for bridge in nav2_bridges:
            bridge.close()
        ros_node.close()
        _ = keyboard_subscription
    return 0


if __name__ == "__main__":
    exit_code = 1
    try:
        exit_code = main()
    except Exception as exc:
        carb.log_error(str(exc))
        print(f"[ERROR] {exc}", file=sys.stderr)
    finally:
        APP.close()
    raise SystemExit(exit_code)
