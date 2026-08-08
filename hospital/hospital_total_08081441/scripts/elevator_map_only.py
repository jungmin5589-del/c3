#!/usr/bin/env python3
"""ROKEY 실물리 엘리베이터 + 안전한 목적층 바닥 인계.

핵심 원칙
- AMR/침대 텔레포트 금지.
- 승강 중 AMR Xform 직접 쓰기 금지.
- 1층 탑승 뒤, 상승 *전에* 숨겨진 물리 바닥과 AMR base_link를 FixedJoint로 결합.
- 보이는 승강부와 숨겨진 Kinematic 물리 바닥을 동일한 Z 궤적으로 이동.
- 목적층에서 물리 바닥이 실제 목표 높이에 도착한 것을 확인한 뒤 문을 열고,
  FixedJoint를 충분히 유지한 다음 해제.
- 중력/충돌을 껐다 켜지 않는다. 목적층 바닥 Collider는 계속 남아 AMR을 받친다.
- 새 경사로를 만들지 않는다. 물리 바닥 윗면은 Side_Lift_Anim_29 윗면보다
  아주 조금 낮게 두고 XY를 안쪽으로 줄여 1층 입구 턱을 만들지 않는다.

O 키 동작
1) 1층 닫힌 상태: O -> 1층 문 열기, 무기한 탑승 대기
2) AMR가 내부에 들어간 상태: O -> Joint 결합 -> 문 닫기 -> 실제 동시 상승
3) 2층 문 개방 완료 후 Joint 해제, 목적층 물리 바닥 위에서 안정화
4) 2층에서 다시 O -> Joint 결합 -> 문 닫기 -> 실제 동시 하강
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, Iterable, Optional

import numpy as np
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics


def _smooth(value: float) -> float:
    value = max(0.0, min(1.0, float(value)))
    return value * value * (3.0 - 2.0 * value)


def _finite_vec(value) -> bool:
    return all(math.isfinite(float(value[index])) for index in range(3))


def _bbox(stage: Usd.Stage, prim: Usd.Prim):
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
    minimum = Gf.Vec3d(box.GetMin())
    maximum = Gf.Vec3d(box.GetMax())
    if not (_finite_vec(minimum) and _finite_vec(maximum)):
        raise RuntimeError(f"invalid bbox: {prim.GetPath()}")
    if any(float(maximum[i]) <= float(minimum[i]) for i in range(3)):
        raise RuntimeError(f"empty bbox: {prim.GetPath()}")
    return minimum, maximum


def _center(minimum, maximum):
    return (Gf.Vec3d(minimum) + Gf.Vec3d(maximum)) * 0.5


def _size(minimum, maximum):
    return Gf.Vec3d(maximum) - Gf.Vec3d(minimum)


def _world_matrix(prim: Usd.Prim) -> Gf.Matrix4d:
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def _world_position(prim: Usd.Prim) -> Gf.Vec3d:
    return Gf.Vec3d(_world_matrix(prim).ExtractTranslation())


def _parent_world_matrix(prim: Usd.Prim) -> Gf.Matrix4d:
    parent = prim.GetParent()
    if parent and parent.IsValid() and UsdGeom.Xformable(parent):
        return _world_matrix(parent)
    return Gf.Matrix4d(1.0)


def _quatd_to_quatf(value: Gf.Quatd) -> Gf.Quatf:
    imag = value.GetImaginary()
    return Gf.Quatf(
        float(value.GetReal()),
        Gf.Vec3f(float(imag[0]), float(imag[1]), float(imag[2])),
    )


def _iter_hospital_prims(stage: Usd.Stage) -> Iterable[Usd.Prim]:
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith("/World/HospitalMap"):
            yield prim


def _collapse_nested_prims(prims: list[Usd.Prim]) -> list[Usd.Prim]:
    selected_paths = {str(prim.GetPath()) for prim in prims}
    roots: list[Usd.Prim] = []
    for prim in prims:
        parent = prim.GetParent()
        nested = False
        while parent and parent.IsValid():
            if str(parent.GetPath()) in selected_paths:
                nested = True
                print(f"[ELEVATOR] child follows selected parent: {prim.GetPath()}")
                break
            parent = parent.GetParent()
        if not nested:
            roots.append(prim)
    return roots


class ExistingPrimMover:
    """현재 World transform 기준 Session Layer offset. 시각 승강부/문 전용."""

    def __init__(self, stage: Usd.Stage, prim: Usd.Prim, suffix: str = "rokeyPhysical"):
        self.stage = stage
        self.prim = prim
        self.base_world = Gf.Matrix4d(_world_matrix(prim))
        self.parent_world = Gf.Matrix4d(_parent_world_matrix(prim))
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            xformable = UsdGeom.Xformable(prim)
            xformable.ClearXformOpOrder()
            self.op = xformable.AddTransformOp(
                UsdGeom.XformOp.PrecisionDouble,
                suffix,
            )
            self.set_offset(Gf.Vec3d(0.0, 0.0, 0.0))

    def set_offset(self, offset: Gf.Vec3d) -> None:
        moved = Gf.Matrix4d(self.base_world)
        translation = Gf.Vec3d(moved.ExtractTranslation()) + Gf.Vec3d(offset)
        moved.SetTranslateOnly(translation)
        local = moved * self.parent_world.GetInverse()
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            self.op.Set(local)


class KinematicFloorMover:
    """숨겨진 RigidBody 바닥을 PhysX Kinematic Target으로 이동한다."""

    def __init__(self, stage: Usd.Stage, prim: Usd.Prim):
        self.stage = stage
        self.prim = prim
        self.path = str(prim.GetPath())
        self.base_world = Gf.Matrix4d(_world_matrix(prim))
        self.desired_offset_z = 0.0
        self.sim_view = None
        self.body_view = None
        self.base_transform: Optional[np.ndarray] = None
        self.indices = np.array([0], dtype=np.int32)
        self.init_attempts = 0
        self.last_error = ""
        self.ready_logged = False

    def ensure_ready(self) -> bool:
        if self.body_view is not None and self.base_transform is not None:
            return True
        self.init_attempts += 1
        try:
            import omni.physics.tensors as tensors

            sim_view = tensors.create_simulation_view("numpy")
            body_view = sim_view.create_rigid_body_view(self.path)
            if hasattr(body_view, "check") and not body_view.check():
                raise RuntimeError("RigidBodyView check failed")
            if int(body_view.count) != 1:
                raise RuntimeError(f"RigidBodyView count={body_view.count}")
            transforms = np.asarray(body_view.get_transforms(), dtype=np.float32).reshape(-1, 7)
            if transforms.shape != (1, 7):
                raise RuntimeError(f"unexpected transform shape={transforms.shape}")
            self.sim_view = sim_view
            self.body_view = body_view
            # View가 승강 중/도착 후 재생성돼도 현재 offset을 다시 더하지 않도록
            # 현재 실제 Pose에서 desired_offset을 빼 1층 기준 Pose를 복원한다.
            self.base_transform = transforms.copy()
            self.base_transform[0, 2] -= np.float32(self.desired_offset_z)
            self.last_error = ""
            if not self.ready_logged:
                print(
                    f"[ELEVATOR FLOOR29] PhysX kinematic target ready: {self.path} "
                    f"baseZ={float(self.base_transform[0, 2]):.4f}"
                )
                self.ready_logged = True
            return True
        except Exception as exc:
            self.sim_view = None
            self.body_view = None
            self.base_transform = None
            message = f"{type(exc).__name__}: {exc}"
            if self.init_attempts <= 3 or self.init_attempts % 60 == 0 or message != self.last_error:
                print(
                    f"[ELEVATOR FLOOR29 WARNING] PhysX target init retry "
                    f"#{self.init_attempts}: {message}"
                )
            self.last_error = message
            return False

    def set_offset(self, offset: Gf.Vec3d) -> None:
        self.desired_offset_z = float(offset[2])
        if abs(self.desired_offset_z) < 1e-9 and self.body_view is None:
            return
        if not self.ensure_ready():
            return
        target = self.base_transform.copy()
        target[0, 2] += np.float32(self.desired_offset_z)
        try:
            self.body_view.set_kinematic_targets(target, self.indices)
            if hasattr(self.sim_view, "flush"):
                self.sim_view.flush()
        except Exception as exc:
            print(f"[ELEVATOR FLOOR29 WARNING] set_kinematic_targets failed: {exc}")
            self.sim_view = None
            self.body_view = None
            self.base_transform = None

    def logical_world_matrix(self) -> Gf.Matrix4d:
        result = Gf.Matrix4d(self.base_world)
        translation = Gf.Vec3d(result.ExtractTranslation())
        translation[2] += float(self.desired_offset_z)
        result.SetTranslateOnly(translation)
        return result

    def physics_z(self) -> float:
        if self.body_view is None and not self.ensure_ready():
            return float("nan")
        try:
            transforms = np.asarray(self.body_view.get_transforms(), dtype=np.float32).reshape(-1, 7)
            return float(transforms[0, 2])
        except Exception as exc:
            print(f"[ELEVATOR FLOOR29 WARNING] get_transforms failed: {exc}")
            return float("nan")

    def target_z(self) -> float:
        if self.base_transform is not None:
            return float(self.base_transform[0, 2]) + float(self.desired_offset_z)
        return float(self.base_world.ExtractTranslation()[2]) + float(self.desired_offset_z)

    def at_target(self, tolerance_m: float = 0.025) -> bool:
        current = self.physics_z()
        target = self.target_z()
        return math.isfinite(current) and abs(current - target) <= float(tolerance_m)


@dataclass
class DoorPair:
    floor: int
    left: ExistingPrimMover
    right: ExistingPrimMover
    axis: int
    distance: float
    ratio: float = 0.0
    target: float = 0.0

    def set_open(self, opened: bool) -> None:
        self.target = 1.0 if opened else 0.0
        print(f"[ELEVATOR MAP] {self.floor}F door {'OPEN' if opened else 'CLOSE'}")

    def update(self, dt: float, duration: float) -> None:
        step = dt / max(duration, 0.05)
        if self.ratio < self.target:
            self.ratio = min(self.target, self.ratio + step)
        elif self.ratio > self.target:
            self.ratio = max(self.target, self.ratio - step)
        ratio = _smooth(self.ratio)
        left_offset = [0.0, 0.0, 0.0]
        right_offset = [0.0, 0.0, 0.0]
        left_offset[self.axis] = -self.distance * ratio
        right_offset[self.axis] = self.distance * ratio
        self.left.set_offset(Gf.Vec3d(*left_offset))
        self.right.set_offset(Gf.Vec3d(*right_offset))

    @property
    def settled(self) -> bool:
        return abs(self.ratio - self.target) < 1e-4


class LiftMover:
    def __init__(
        self,
        visual_movers: list[ExistingPrimMover],
        floor_mover: KinematicFloorMover,
        distance_m: float,
    ):
        if not visual_movers:
            raise RuntimeError("엘리베이터 시각 이동 Prim이 없습니다.")
        self.visual_movers = list(visual_movers)
        self.floor_mover = floor_mover
        self.distance_m = float(distance_m)
        self.ratio = 0.0
        self.target = 0.0
        self.current_z = 0.0

    def set_floor(self, floor: int) -> None:
        if floor not in (1, 2):
            raise ValueError(f"invalid floor: {floor}")
        self.target = 0.0 if floor == 1 else 1.0
        print(f"[ELEVATOR MAP] lift group GO_{floor} exact={self.distance_m:.3f}m")

    def update(self, dt: float, duration: float) -> None:
        step = dt / max(duration, 0.05)
        if self.ratio < self.target:
            self.ratio = min(self.target, self.ratio + step)
        elif self.ratio > self.target:
            self.ratio = max(self.target, self.ratio - step)
        self.current_z = self.distance_m * _smooth(self.ratio)
        offset = Gf.Vec3d(0.0, 0.0, self.current_z)
        # 다음 PhysX step 전에 실제 물리 바닥 목표를 먼저 전달한다.
        self.floor_mover.set_offset(offset)
        for mover in self.visual_movers:
            mover.set_offset(offset)

    @property
    def settled(self) -> bool:
        return abs(self.ratio - self.target) < 1e-4

    @property
    def floor(self) -> int:
        return 1 if self.ratio < 0.5 else 2


class MapOnlyElevator:
    DEFAULT_LIFT_NAMES = (
        "Side_Lift_Anim_29",
        "Side_Lift_Anim_28",
        "Dummy002",
    )
    DEFAULT_FLOOR_NAME = "Side_Lift_Anim_29"
    FLOOR_PROXY_PATH = "/World/RuntimeElevatorFloor29"
    TRANSPORT_JOINT_PATH = "/World/RuntimeElevatorAMR1Joint"
    PLANAR_2F_JOINT_PATH = "/World/RuntimeAMR1Planar2FLock"
    SHAFT_CUBE_PATH = "/World/HospitalMap/Cube"

    def __init__(self, stage: Usd.Stage, config: Optional[dict[str, Any]] = None):
        self.stage = stage
        self.config = dict(config or {})
        self.amr: Optional[Any] = None
        self.controllers: list[Any] = []
        self.state = "IDLE_1_CLOSED"
        self._requested = False
        self._locked = False
        self.wait = 0.0
        self.transport_joint_active = False
        self.planar_2f_lock_active = False
        self._last_diag = 0.0
        self._arrival_floor = 1
        self._arrival_wait_started = 0.0
        self._shaft_cube_collision_paths: list[str] = []
        self._shaft_cube_collision_enabled = True
        # Empty-car call state. No AMR pose reset/teleport is ever used.
        self._empty_target_floor = 0
        self._empty_open_on_arrival = False

        self.door_time = float(self.config.get("door_time_s", 2.0))
        self.lift_time = float(self.config.get("lift_time_s", 9.0))
        self.lift_distance = float(self.config.get("lift_distance_m", 11.325))
        self.arrival_floor_sync_timeout = float(self.config.get("arrival_floor_sync_timeout_s", 3.0))
        self.arrival_joint_hold = float(self.config.get("arrival_joint_hold_s", 0.9))
        self.post_release_settle = float(self.config.get("post_release_settle_s", 0.8))
        self.floor_target_tolerance = float(self.config.get("floor_target_tolerance_m", 0.025))
        self.capture_margin = float(self.config.get("capture_margin_m", 0.35))
        self.floor_proxy_thickness = float(self.config.get("floor_proxy_thickness_m", 0.10))
        self.floor_proxy_top_bias = float(self.config.get("floor_proxy_top_bias_m", -0.001))
        self.floor_proxy_inset = float(self.config.get("floor_proxy_inset_m", 0.03))
        self.lift_names = tuple(self.config.get("lift_prim_names", self.DEFAULT_LIFT_NAMES))
        self.floor_name = str(self.config.get("floor_prim_name", self.DEFAULT_FLOOR_NAME))

        lift_prims = self._discover_lifts()
        move_roots = _collapse_nested_prims(lift_prims)
        floor_prim = next((prim for prim in lift_prims if prim.GetName() == self.floor_name), None)
        if floor_prim is None:
            raise RuntimeError(f"실제 승강 바닥 Prim을 찾지 못했습니다: {self.floor_name}")

        floor_min, floor_max = _bbox(stage, floor_prim)
        self.floor_source_min = floor_min
        self.floor_source_max = floor_max
        self.floor_center_xy = (
            float((floor_min[0] + floor_max[0]) * 0.5),
            float((floor_min[1] + floor_max[1]) * 0.5),
        )
        self.floor_half_x = max(0.25, float(floor_max[0] - floor_min[0]) * 0.5)
        self.floor_half_y = max(0.25, float(floor_max[1] - floor_min[1]) * 0.5)

        door_specs = self._discover_door_pairs()
        door_specs.sort(key=lambda item: item[0])
        if len(door_specs) != 2:
            raise RuntimeError("1층/2층 문 쌍을 정확히 찾지 못했습니다.")
        _, left1, right1, axis1, shift1 = door_specs[0]
        _, left2, right2, axis2, shift2 = door_specs[1]

        self.door1 = DoorPair(
            1,
            ExistingPrimMover(stage, left1, "rokeyDoor1Left"),
            ExistingPrimMover(stage, right1, "rokeyDoor1Right"),
            axis1,
            shift1,
        )
        self.door2 = DoorPair(
            2,
            ExistingPrimMover(stage, left2, "rokeyDoor2Left"),
            ExistingPrimMover(stage, right2, "rokeyDoor2Right"),
            axis2,
            shift2,
        )

        floor_proxy = self._create_floor_proxy(floor_prim)
        self.floor_proxy = floor_proxy
        self._enable_floor_proxy_physics(floor_proxy)
        self.floor_mover = KinematicFloorMover(stage, floor_proxy)
        self.lift = LiftMover(
            [ExistingPrimMover(stage, prim, f"rokeyLift{index}") for index, prim in enumerate(move_roots)],
            self.floor_mover,
            self.lift_distance,
        )

        self.door1.update(0.0, self.door_time)
        self.door2.update(0.0, self.door_time)
        self.lift.update(0.0, self.lift_time)

        self._discover_shaft_cube_colliders()
        # 이전 테스트에서 USD에 collisionEnabled=False가 저장되어 있어도
        # 실행 시작 시 목적층 지지 Collider를 반드시 복구한다.
        self._set_shaft_cube_collision(True, "startup support restore")

        print("[ELEVATOR PHYSICAL READY] ROKEY pre-lift FixedJoint / no teleport / no ramp")
        print("[ELEVATOR PHYSICAL READY] selected lift prims:")
        for prim in lift_prims:
            print(f"  - {prim.GetPath()}")
        print(f"[ELEVATOR PHYSICAL READY] floor source={floor_prim.GetPath()}")
        print(f"[ELEVATOR PHYSICAL READY] exact lift distance={self.lift_distance:.3f} m")
        print(f"[ELEVATOR PHYSICAL READY] 1F doors={left1.GetPath()} | {right1.GetPath()}")
        print(f"[ELEVATOR PHYSICAL READY] 2F doors={left2.GetPath()} | {right2.GetPath()}")
        print("[ELEVATOR PHYSICAL READY] O1=open 1F / O2=joint lock, close, physical lift")

    def _discover_shaft_cube_colliders(self) -> None:
        """Collect only Collider prims under HospitalMap/Cube."""
        cube = self.stage.GetPrimAtPath(self.SHAFT_CUBE_PATH)
        if not cube or not cube.IsValid():
            print(f"[ELEVATOR CUBE WARNING] missing: {self.SHAFT_CUBE_PATH}")
            self._shaft_cube_collision_paths = []
            return

        paths: list[str] = []
        for prim in Usd.PrimRange(cube):
            if prim.IsInstanceProxy():
                continue
            attr = prim.GetAttribute("physics:collisionEnabled")
            if prim.HasAPI(UsdPhysics.CollisionAPI) or (attr and attr.IsValid()):
                paths.append(str(prim.GetPath()))

        if not paths:
            paths.append(self.SHAFT_CUBE_PATH)
        self._shaft_cube_collision_paths = paths
        print(f"[ELEVATOR CUBE] collider targets={paths}")

    def _set_shaft_cube_collision(self, enabled: bool, reason: str) -> None:
        """Toggle HospitalMap/Cube only in the runtime Session Layer."""
        enabled = bool(enabled)
        if not self._shaft_cube_collision_paths:
            return
        if enabled == self._shaft_cube_collision_enabled:
            if not enabled or reason != "startup support restore":
                return

        changed: list[str] = []
        try:
            with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                for path in self._shaft_cube_collision_paths:
                    prim = self.stage.GetPrimAtPath(path)
                    if not prim or not prim.IsValid() or prim.IsInstanceProxy():
                        continue
                    collision = UsdPhysics.CollisionAPI.Apply(prim)
                    collision.CreateCollisionEnabledAttr(enabled).Set(enabled)
                    changed.append(path)
            self._shaft_cube_collision_enabled = enabled
            state = "ON" if enabled else "OFF"
            print(f"[ELEVATOR CUBE] collision={state}: {reason}; prims={changed}")
        except Exception as exc:
            print(f"[ELEVATOR CUBE WARNING] toggle failed ({reason}): {exc}")

    def bind_controllers(self, controllers: Iterable[Any]) -> None:
        items = list(controllers)
        if not items:
            raise RuntimeError("AMR controller list is empty")
        self.controllers = items
        for controller in items:
            try:
                with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                    PhysxSchema.PhysxRigidBodyAPI.Apply(controller.base_prim).CreateEnableCCDAttr(True)
                print(f"[ELEVATOR PHYSICAL] AMR CCD enabled: {controller.base_prim.GetPath()}")
            except Exception as exc:
                print(f"[ELEVATOR PHYSICAL WARNING] AMR CCD setup failed ({controller.name}): {exc}")
        self.amr = items[0]
        print(f"[ELEVATOR PHYSICAL] default AMR bound: {self.amr.name}")

    def select_controller(self, controller: Any) -> bool:
        # The physical elevator has a single cabin, so its owner can only change
        # while no transport sequence is holding the cabin.  This is the second
        # guard after the ROS service mutex.
        if controller not in self.controllers:
            print(f"[ELEVATOR OWNER] rejected unknown controller: {getattr(controller, 'name', controller)}")
            return False
        if self._locked or self.transport_joint_active:
            return controller is self.amr
        self.amr = controller
        print(f"[ELEVATOR OWNER] selected: {controller.name}")
        return True

    def _planar_2f_joint_path(self) -> str:
        namespace = str(getattr(self.amr, "namespace", "amr1")) if self.amr is not None else "amr1"
        safe = "".join(ch for ch in namespace if ch.isalnum()) or "amr1"
        return f"/World/Runtime{safe.upper()}Planar2FLock"

    def is_controller_locked(self, controller: Any) -> bool:
        return bool(self._locked and controller is self.amr)

    def _find_exact_named(self, name: str) -> list[Usd.Prim]:
        matches = [
            prim
            for prim in _iter_hospital_prims(self.stage)
            if prim.GetName() == name and UsdGeom.Xformable(prim)
        ]
        if not matches:
            raise RuntimeError(f"현재 맵에서 Prim을 찾지 못했습니다: {name}")
        matches.sort(key=lambda prim: (len(str(prim.GetPath()).split("/")), str(prim.GetPath())))
        return matches

    def _discover_lifts(self) -> list[Usd.Prim]:
        result: list[Usd.Prim] = []
        for name in self.lift_names:
            matches = self._find_exact_named(str(name))
            preferred = [
                prim
                for prim in matches
                if "Side_Lift" in str(prim.GetPath()) or "SideLift" in str(prim.GetPath())
            ]
            result.append((preferred or matches)[0])
        return result

    @staticmethod
    def _ancestor_names(prim: Usd.Prim) -> list[str]:
        names: list[str] = []
        parent = prim.GetParent()
        while parent and parent.IsValid():
            names.append(parent.GetName())
            parent = parent.GetParent()
        return names

    def _discover_door_pairs(self):
        lefts = self._find_exact_named("LeftDoor")
        rights = self._find_exact_named("RightDoor")
        pairs = []
        for left in lefts:
            parent = left.GetParent()
            if not parent or not parent.IsValid():
                continue
            right = next((candidate for candidate in rights if candidate.GetParent() == parent), None)
            if right is None:
                continue
            try:
                left_min, left_max = _bbox(self.stage, left)
                right_min, right_max = _bbox(self.stage, right)
            except Exception:
                continue
            left_center = _center(left_min, left_max)
            right_center = _center(right_min, right_max)
            left_size = _size(left_min, left_max)
            right_size = _size(right_min, right_max)
            dx = abs(float(left_center[0] - right_center[0]))
            dy = abs(float(left_center[1] - right_center[1]))
            axis = 0 if dx >= dy else 1
            separation = dx if axis == 0 else dy
            panel_span = max(float(left_size[axis]), float(right_size[axis]), separation * 0.5)
            shift = max(panel_span * 0.9, separation * 0.55)
            z_value = float((left_center[2] + right_center[2]) * 0.5)
            pairs.append(
                {
                    "left": left,
                    "right": right,
                    "axis": axis,
                    "shift": shift,
                    "z": z_value,
                    "parent": parent,
                    "ancestors": self._ancestor_names(left),
                }
            )

        floor1 = [pair for pair in pairs if "Floor1" in pair["ancestors"] or pair["parent"].GetName() == "Floor1"]
        floor2 = [pair for pair in pairs if "Floor2" in pair["ancestors"] or pair["parent"].GetName() == "Floor2"]
        selected = []
        if floor1:
            floor1.sort(key=lambda pair: pair["z"])
            selected.append(floor1[0])
        if floor2:
            floor2.sort(key=lambda pair: pair["z"])
            selected.append(floor2[-1])
        elif len(floor1) >= 2:
            selected = [floor1[0], floor1[-1]]
        elif len(pairs) >= 2:
            pairs.sort(key=lambda pair: pair["z"])
            selected = [pairs[0], pairs[-1]]
        if len(selected) != 2:
            details = " / ".join(f"{pair['parent'].GetPath()} z={pair['z']:.3f}" for pair in pairs)
            raise RuntimeError("문 쌍 검색 실패: " + details)
        selected.sort(key=lambda pair: pair["z"])
        return [
            (pair["z"], pair["left"], pair["right"], pair["axis"], pair["shift"])
            for pair in selected
        ]

    def _create_floor_proxy(self, floor_prim: Usd.Prim) -> Usd.Prim:
        minimum, maximum = _bbox(self.stage, floor_prim)
        size = _size(minimum, maximum)
        center = _center(minimum, maximum)
        inset = max(0.0, self.floor_proxy_inset)
        sx = max(0.80, float(size[0]) - inset * 2.0)
        sy = max(0.80, float(size[1]) - inset * 2.0)
        sz = max(0.02, self.floor_proxy_thickness)
        top_z = float(maximum[2]) + self.floor_proxy_top_bias
        proxy_center = Gf.Vec3d(float(center[0]), float(center[1]), top_z - sz * 0.5)

        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            old = self.stage.GetPrimAtPath(self.FLOOR_PROXY_PATH)
            if old and old.IsValid():
                self.stage.RemovePrim(self.FLOOR_PROXY_PATH)
            cube = UsdGeom.Cube.Define(self.stage, self.FLOOR_PROXY_PATH)
            cube.CreateSizeAttr(1.0)
            xformable = UsdGeom.Xformable(cube.GetPrim())
            xformable.ClearXformOpOrder()
            op = xformable.AddTransformOp(
                UsdGeom.XformOp.PrecisionDouble,
                "floor29Base",
            )
            matrix = Gf.Matrix4d(1.0)
            matrix.SetScale(Gf.Vec3d(sx, sy, sz))
            matrix.SetTranslateOnly(proxy_center)
            op.Set(matrix)
            UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()

        print(
            f"[ELEVATOR FLOOR29] source={floor_prim.GetPath()} proxy={self.FLOOR_PROXY_PATH} "
            f"size=({sx:.3f},{sy:.3f},{sz:.3f}) sourceTop={float(maximum[2]):.4f} "
            f"proxyTop={top_z:.4f} inset={inset:.3f}"
        )
        return self.stage.GetPrimAtPath(self.FLOOR_PROXY_PATH)

    def _enable_floor_proxy_physics(self, proxy: Usd.Prim) -> None:
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            collision = UsdPhysics.CollisionAPI.Apply(proxy)
            collision.CreateCollisionEnabledAttr(True)
            body = UsdPhysics.RigidBodyAPI.Apply(proxy)
            body.CreateRigidBodyEnabledAttr(True)
            body.CreateKinematicEnabledAttr(True)
            physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(proxy)
            physx_body.CreateEnableCCDAttr(True)
            scene_count = 0
            for prim in self.stage.Traverse():
                if prim.IsA(UsdPhysics.Scene):
                    PhysxSchema.PhysxSceneAPI.Apply(prim).CreateEnableCCDAttr(True)
                    scene_count += 1
        print(
            f"[ELEVATOR FLOOR29] hidden kinematic collider enabled: {proxy.GetPath()} "
            f"CCD scenes={scene_count}"
        )

    def _amr_inside_cabin(self) -> bool:
        if self.amr is None:
            return False
        pos = _world_position(self.amr.base_prim)
        dx = abs(float(pos[0]) - self.floor_center_xy[0])
        dy = abs(float(pos[1]) - self.floor_center_xy[1])
        inside = (
            dx <= self.floor_half_x + self.capture_margin
            and dy <= self.floor_half_y + self.capture_margin
        )
        if not inside:
            print(
                f"[ELEVATOR BOARDING BLOCKED] AMR base is not inside cabin: "
                f"dx={dx:.3f}/{self.floor_half_x + self.capture_margin:.3f} "
                f"dy={dy:.3f}/{self.floor_half_y + self.capture_margin:.3f}"
            )
        return inside

    def _remove_transport_joint(self, reason: str) -> None:
        # 목적층 지지 Collider가 살아 있는 상태에서만 Joint를 해제한다.
        self._set_shaft_cube_collision(True, f"before joint release: {reason}")
        prim = self.stage.GetPrimAtPath(self.TRANSPORT_JOINT_PATH)
        if prim and prim.IsValid():
            try:
                with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                    self.stage.RemovePrim(self.TRANSPORT_JOINT_PATH)
                print(f"[ELEVATOR JOINT] RELEASED: {reason}")
            except Exception as exc:
                print(f"[ELEVATOR JOINT WARNING] release failed: {exc}")
        self.transport_joint_active = False

    def _remove_2f_planar_lock(self, reason: str) -> None:
        prim = self.stage.GetPrimAtPath(self._planar_2f_joint_path())
        if prim and prim.IsValid():
            try:
                with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                    self.stage.RemovePrim(self._planar_2f_joint_path())
                print(f"[2F HEIGHT LOCK] RELEASED: {reason}")
            except Exception as exc:
                print(f"[2F HEIGHT LOCK WARNING] release failed: {exc}")
        self.planar_2f_lock_active = False

    def _create_2f_planar_lock(self) -> bool:
        """Lock the selected AMR to its current 2F height while leaving X/Y/yaw free.

        This is a world-anchored D6-style joint made from the generic USD Physics
        Joint plus per-axis limits.  Only transZ, rotX and rotY are locked.
        The current base_link pose becomes the reference, so no hard-coded Z or
        pose teleport is needed.
        """
        if self.amr is None:
            return False
        base = self.amr.base_prim
        if not base or not base.IsValid():
            return False
        try:
            self._remove_2f_planar_lock("recreate")
            base_world = _world_matrix(base)
            anchor = Gf.Vec3d(base_world.ExtractTranslation())
            base_rot = base_world.ExtractRotationQuat()
            local_rot1 = _quatd_to_quatf(base_rot.GetInverse())

            with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                joint = UsdPhysics.Joint.Define(self.stage, self._planar_2f_joint_path())
                # body0 intentionally has no target: it is anchored to the world.
                joint.CreateBody1Rel().SetTargets([base.GetPath()])
                joint.CreateLocalPos0Attr().Set(
                    Gf.Vec3f(float(anchor[0]), float(anchor[1]), float(anchor[2]))
                )
                joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
                joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
                joint.CreateLocalRot1Attr().Set(local_rot1)
                joint.CreateCollisionEnabledAttr(False)
                joint.CreateExcludeFromArticulationAttr(True)
                joint.CreateJointEnabledAttr(True)
                joint.CreateBreakForceAttr().Set(1.0e12)
                joint.CreateBreakTorqueAttr().Set(1.0e12)

                # USD Physics: low > high means the selected D6 axis is locked.
                for axis in ("transZ", "rotX", "rotY"):
                    limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
                    limit.CreateLowAttr().Set(1.0)
                    limit.CreateHighAttr().Set(-1.0)

            self.planar_2f_lock_active = True
            print(
                f"[2F HEIGHT LOCK] ENABLED amr={base.GetPath()} "
                f"lockedZ={float(anchor[2]):.4f}; free=X/Y/yaw"
            )
            return True
        except Exception as exc:
            self.planar_2f_lock_active = False
            print(f"[2F HEIGHT LOCK WARNING] create failed: {type(exc).__name__}: {exc}")
            return False

    def _create_transport_joint(self) -> bool:
        if self.amr is None:
            print("[ELEVATOR JOINT ERROR] AMR is not bound")
            return False
        if not self.floor_mover.ensure_ready():
            print("[ELEVATOR JOINT ERROR] physical floor is not ready; departure cancelled")
            return False
        floor = self.floor_proxy
        base = self.amr.base_prim
        if not floor or not floor.IsValid() or not base or not base.IsValid():
            print("[ELEVATOR JOINT ERROR] invalid floor/base prim")
            return False
        try:
            self._remove_transport_joint("recreate")
            floor_world = self.floor_mover.logical_world_matrix()
            base_world = _world_matrix(base)
            anchor_world = Gf.Vec3d(base_world.ExtractTranslation())
            local_floor_d = floor_world.GetInverse().Transform(anchor_world)
            local_base_d = base_world.GetInverse().Transform(anchor_world)
            local_floor = Gf.Vec3f(float(local_floor_d[0]), float(local_floor_d[1]), float(local_floor_d[2]))
            local_base = Gf.Vec3f(float(local_base_d[0]), float(local_base_d[1]), float(local_base_d[2]))
            floor_rot = floor_world.ExtractRotationQuat()
            base_rot = base_world.ExtractRotationQuat()
            relative_rot = floor_rot.GetInverse() * base_rot

            with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                joint = UsdPhysics.FixedJoint.Define(self.stage, self.TRANSPORT_JOINT_PATH)
                joint.CreateBody0Rel().SetTargets([floor.GetPath()])
                joint.CreateBody1Rel().SetTargets([base.GetPath()])
                joint.CreateLocalPos0Attr().Set(local_floor)
                joint.CreateLocalPos1Attr().Set(local_base)
                joint.CreateLocalRot0Attr().Set(_quatd_to_quatf(relative_rot))
                joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
                joint.CreateCollisionEnabledAttr(False)
                joint.CreateExcludeFromArticulationAttr(True)
                joint.CreateJointEnabledAttr(True)
                joint.CreateBreakForceAttr().Set(1.0e12)
                joint.CreateBreakTorqueAttr().Set(1.0e12)
            self.transport_joint_active = True
            print(
                f"[ELEVATOR JOINT] LOCKED BEFORE LIFT floor={floor.GetPath()} "
                f"amr={base.GetPath()} anchorZ={float(anchor_world[2]):.4f}"
            )
            return True
        except Exception as exc:
            self.transport_joint_active = False
            print(f"[ELEVATOR JOINT ERROR] create failed: {type(exc).__name__}: {exc}")
            return False

    def _begin_trip(self, destination_floor: int) -> bool:
        if self.amr is None:
            print("[ELEVATOR PHYSICAL] AMR1 controller is not bound")
            return False
        if not self._amr_inside_cabin():
            return False
        if destination_floor == 1:
            # The 2F planar constraint must be gone before vertical travel starts.
            self._remove_2f_planar_lock("prepare DOWN trip")
        self.amr.hold_motion(self.lift_time + self.door_time * 2.0 + 5.0)
        self.amr.set_external_physics_mode(True)
        if not self._create_transport_joint():
            self.amr.set_external_physics_mode(False)
            return False
        # FixedJoint로 승강 바닥에 잠긴 동안에만 Cube를 통과한다.
        self._set_shaft_cube_collision(False, "passenger joint locked for vertical travel")
        self._locked = True
        self._arrival_floor = int(destination_floor)
        if destination_floor == 2:
            self.door1.set_open(False)
            self.state = "CLOSING_1_UP"
        else:
            self.door2.set_open(False)
            self.state = "CLOSING_2_DOWN"
        return True

    def call_empty_to_floor(self, floor: int, open_on_arrival: bool = False) -> bool:
        """Move the empty elevator car to a requested floor without touching any AMR pose.

        This is used only when the requesting AMR is waiting outside and the car is
        idle on the opposite floor. The opposite-floor door is closed before travel.
        """
        floor = int(floor)
        if floor not in (1, 2):
            return False
        if self.transport_joint_active or self._locked:
            print(f"[ELEVATOR EMPTY CALL] reject: passenger transport active state={self.state}")
            return False

        self._empty_target_floor = floor
        self._empty_open_on_arrival = bool(open_on_arrival)

        if floor == 1:
            if self.state in {"BOARDING_1", "IDLE_1_OPEN"}:
                return True
            if self.state == "IDLE_1_CLOSED":
                if self._empty_open_on_arrival:
                    self.door1.set_open(True)
                    self.state = "EMPTY_OPENING_1"
                return True
            if self.state in {"IDLE_2_OPEN", "CLOSING_2_IDLE"}:
                self.door2.set_open(False)
                self.state = "EMPTY_CLOSING_2_TO_1"
                print("[ELEVATOR EMPTY CALL] 1F requested while car is on 2F -> close 2F and return empty")
                return True
            if self.state == "IDLE_2_CLOSED":
                self.lift.set_floor(1)
                self.state = "EMPTY_MOVING_DOWN"
                print("[ELEVATOR EMPTY CALL] empty car GO_1")
                return True
            return False

        if self.state in {"IDLE_2_OPEN"}:
            return True
        if self.state == "IDLE_2_CLOSED":
            if self._empty_open_on_arrival:
                self.door2.set_open(True)
                self.state = "EMPTY_OPENING_2"
            return True
        if self.state in {"BOARDING_1", "IDLE_1_OPEN", "CLOSING_1_IDLE"}:
            self.door1.set_open(False)
            self.state = "EMPTY_CLOSING_1_TO_2"
            print("[ELEVATOR EMPTY CALL] 2F requested while car is on 1F -> close 1F and send empty")
            return True
        if self.state == "IDLE_1_CLOSED":
            self.lift.set_floor(2)
            self.state = "EMPTY_MOVING_UP"
            print("[ELEVATOR EMPTY CALL] empty car GO_2")
            return True
        return False

    def close_idle_door(self, floor: int) -> bool:
        """Close an idle floor door without starting a lift trip."""
        floor = int(floor)
        if floor == 2 and self.state == "IDLE_2_OPEN":
            self.door2.set_open(False)
            self.state = "CLOSING_2_IDLE"
            print("[ELEVATOR MAP] 2F AMR exit complete -> closing 2F door")
            return True
        if floor == 1 and self.state == "IDLE_1_OPEN":
            self.door1.set_open(False)
            self.state = "CLOSING_1_IDLE"
            print("[ELEVATOR MAP] 1F AMR exit complete -> closing 1F door")
            return True
        return False

    def trigger(self) -> None:
        if self.state == "IDLE_1_CLOSED":
            self.door1.set_open(True)
            self.state = "OPENING_1"
            print("[ELEVATOR MAP] O trigger accepted state=IDLE_1_CLOSED")
            return
        if self.state == "IDLE_2_CLOSED":
            self.door2.set_open(True)
            self.state = "OPENING_2_IDLE"
            print("[ELEVATOR MAP] DOWN request at 2F -> opening 2F door")
            return
        if self.state in {"BOARDING_1", "IDLE_1_OPEN"}:
            print(f"[ELEVATOR MAP] O trigger accepted state={self.state}; preparing physical UP trip")
            self._begin_trip(2)
            return
        if self.state == "IDLE_2_OPEN":
            print("[ELEVATOR MAP] O trigger accepted state=IDLE_2_OPEN; preparing physical DOWN trip")
            self._begin_trip(1)
            return
        print(f"[ELEVATOR MAP] busy: {self.state}")

    def _begin_arrival(self, floor: int) -> None:
        self._arrival_floor = floor
        self._arrival_wait_started = time.monotonic()
        self.wait = self.arrival_joint_hold
        self.state = f"ARRIVAL_SYNC_{floor}"
        print(
            f"[ELEVATOR ARRIVED] floor={floor}; joint stays locked while physical floor reaches target"
        )

    def _diagnostic(self) -> None:
        if self.amr is None:
            return
        now = time.monotonic()
        if now - self._last_diag < 0.5:
            return
        self._last_diag = now
        try:
            amr_z = float(_world_position(self.amr.base_prim)[2])
            floor_z = self.floor_mover.physics_z()
            print(
                f"[ELEVATOR PHYSICS] state={self.state} lift={self.lift.current_z:.3f} "
                f"floorZ={floor_z:.3f} targetFloorZ={self.floor_mover.target_z():.3f} "
                f"amrZ={amr_z:.3f} joint={self.transport_joint_active}"
            )
        except Exception as exc:
            print(f"[ELEVATOR PHYSICS WARNING] diagnostic failed: {exc}")

    def update(self, dt: float) -> None:
        self.door1.update(dt, self.door_time)
        self.door2.update(dt, self.door_time)
        self.lift.update(dt, self.lift_time)

        if self._locked:
            self._diagnostic()

        # Empty-car positioning: doors stay closed while the lift is between floors.
        if self.state == "EMPTY_CLOSING_2_TO_1" and self.door2.settled:
            self.lift.set_floor(1)
            self.state = "EMPTY_MOVING_DOWN"
            print("[ELEVATOR EMPTY CALL] 2F door closed -> empty DOWN")

        elif self.state == "EMPTY_CLOSING_1_TO_2" and self.door1.settled:
            self.lift.set_floor(2)
            self.state = "EMPTY_MOVING_UP"
            print("[ELEVATOR EMPTY CALL] 1F door closed -> empty UP")

        elif self.state == "EMPTY_MOVING_DOWN" and self.lift.settled and self.floor_mover.at_target(self.floor_target_tolerance):
            if self._empty_open_on_arrival:
                self.door1.set_open(True)
                self.state = "EMPTY_OPENING_1"
            else:
                self.state = "IDLE_1_CLOSED"
            print("[ELEVATOR EMPTY CALL] empty car arrived 1F")

        elif self.state == "EMPTY_MOVING_UP" and self.lift.settled and self.floor_mover.at_target(self.floor_target_tolerance):
            if self._empty_open_on_arrival:
                self.door2.set_open(True)
                self.state = "EMPTY_OPENING_2"
            else:
                self.state = "IDLE_2_CLOSED"
            print("[ELEVATOR EMPTY CALL] empty car arrived 2F")

        elif self.state == "EMPTY_OPENING_1" and self.door1.settled:
            self.state = "BOARDING_1"
            print("[ELEVATOR EMPTY CALL] 1F door open -> waiting AMR boarding")

        elif self.state == "EMPTY_OPENING_2" and self.door2.settled:
            self.state = "IDLE_2_OPEN"
            print("[ELEVATOR EMPTY CALL] 2F door open")

        elif self.state == "OPENING_1" and self.door1.settled:
            self.state = "BOARDING_1"
            print("[ELEVATOR BOARDING] 1F door open. Drive AMR/bed inside, then press O again.")

        elif self.state == "OPENING_2_IDLE" and self.door2.settled:
            self.state = "IDLE_2_OPEN"
            print("[ELEVATOR BOARDING] 2F door open. AMR may rotate right 90deg and enter.")

        elif self.state == "CLOSING_2_IDLE" and self.door2.settled:
            self.state = "IDLE_2_CLOSED"
            print("[ELEVATOR MAP] 2F door closed after AMR exit")

        elif self.state == "CLOSING_1_IDLE" and self.door1.settled:
            self.state = "IDLE_1_CLOSED"
            print("[ELEVATOR MAP] 1F idle door closed")

        elif self.state == "CLOSING_1_UP" and self.door1.settled:
            if not self.transport_joint_active:
                self._set_shaft_cube_collision(True, "joint missing before lift")
                self.state = "ERROR"
                print("[ELEVATOR ERROR] joint disappeared before lift start")
                return
            self.lift.set_floor(2)
            self.state = "MOVING_UP"
            print("[ELEVATOR PHYSICAL] UP started with FixedJoint already locked")

        elif self.state == "MOVING_UP" and self.lift.settled:
            # Joint가 잠긴 상태에서 2층 지지 Collider부터 복구한다.
            self._set_shaft_cube_collision(True, "2F reached before handoff")
            self._begin_arrival(2)

        elif self.state == "CLOSING_2_DOWN" and self.door2.settled:
            if not self.transport_joint_active:
                self._set_shaft_cube_collision(True, "joint missing before descent")
                self.state = "ERROR"
                print("[ELEVATOR ERROR] joint disappeared before descent start")
                return
            self.lift.set_floor(1)
            self.state = "MOVING_DOWN"
            print("[ELEVATOR PHYSICAL] DOWN started with FixedJoint already locked")

        elif self.state == "MOVING_DOWN" and self.lift.settled:
            self._set_shaft_cube_collision(True, "1F reached before handoff")
            self._begin_arrival(1)

        elif self.state in {"ARRIVAL_SYNC_1", "ARRIVAL_SYNC_2"}:
            floor = self._arrival_floor
            at_target = self.floor_mover.at_target(self.floor_target_tolerance)
            elapsed = time.monotonic() - self._arrival_wait_started
            if not at_target:
                # 실제 물리 바닥이 목적층에 도달하기 전에는 문도 열지 않고 Joint도 유지한다.
                # timeout 값은 경고 주기 기준일 뿐, 안전 조건을 우회하지 않는다.
                if elapsed >= self.arrival_floor_sync_timeout and int(elapsed * 2.0) % 2 == 0:
                    print(
                        f"[ELEVATOR SUPPORT WAIT] physical floor not at target after {elapsed:.2f}s; "
                        "keeping door closed and FixedJoint locked"
                    )
                return
            self.wait -= dt
            if self.wait <= 0.0:
                if floor == 2:
                    self.door2.set_open(True)
                    self.state = "OPENING_2_LOCKED"
                else:
                    self.door1.set_open(True)
                    self.state = "OPENING_1_LOCKED"
                print(
                    f"[ELEVATOR SUPPORT] floor={floor} physical carrier fixed at destination; "
                    "opening door while FixedJoint remains locked"
                )

        elif self.state == "OPENING_2_LOCKED" and self.door2.settled:
            self.wait = self.arrival_joint_hold
            self.state = "HANDOFF_2"
            print("[ELEVATOR SUPPORT] 2F door fully open; holding joint for floor-contact handoff")

        elif self.state == "OPENING_1_LOCKED" and self.door1.settled:
            self.wait = self.arrival_joint_hold
            self.state = "HANDOFF_1"
            print("[ELEVATOR SUPPORT] 1F door fully open; holding joint for floor-contact handoff")

        elif self.state in {"HANDOFF_1", "HANDOFF_2"}:
            floor = 1 if self.state.endswith("1") else 2
            # 물리 바닥이 목표 위치가 아니면 Joint를 절대 해제하지 않는다.
            if not self.floor_mover.at_target(self.floor_target_tolerance):
                return
            self.wait -= dt
            if self.wait <= 0.0:
                self._remove_transport_joint(f"{floor}F physical floor contact established")
                if self.amr is not None:
                    self.amr.set_external_physics_mode(False)
                    self.amr.hold_motion(self.post_release_settle)
                self.wait = self.post_release_settle
                self.state = f"RELEASE_SETTLE_{floor}"
                print(
                    f"[ELEVATOR SUPPORT HANDOFF] floor={floor}; joint released, "
                    f"physical carrier remains, settle={self.post_release_settle:.2f}s"
                )

        elif self.state in {"RELEASE_SETTLE_1", "RELEASE_SETTLE_2"}:
            floor = 1 if self.state.endswith("1") else 2
            self.wait -= dt
            if self.wait <= 0.0:
                if floor == 2:
                    # Capture the actual arrived base_link height and constrain only
                    # vertical/tilt DOFs.  X/Y/yaw remain fully available to Nav2.
                    self._create_2f_planar_lock()
                else:
                    self._remove_2f_planar_lock("1F active")
                self._locked = False
                self.state = "IDLE_1_OPEN" if floor == 1 else "IDLE_2_OPEN"
                print(f"[ELEVATOR COMPLETE] floor={floor}; AMR released on real physical support")


class DisabledElevator:
    """초기화 오류가 있어도 나머지 프로젝트를 실행한다."""

    def __init__(self, error: Exception):
        self.error = error
        print(f"[ELEVATOR DISABLED] {type(error).__name__}: {error}")

    def bind_controllers(self, _controllers: Iterable[Any]) -> None:
        return None

    def select_controller(self, _controller: Any) -> bool:
        return False

    def is_controller_locked(self, _controller: Any) -> bool:
        return False

    def trigger(self) -> None:
        print(f"[ELEVATOR DISABLED] O unavailable: {self.error}")

    def call_empty_to_floor(self, _floor: int, open_on_arrival: bool = False) -> bool:
        _ = open_on_arrival
        print(f"[ELEVATOR DISABLED] empty call unavailable: {self.error}")
        return False

    def update(self, _dt: float) -> None:
        return None
