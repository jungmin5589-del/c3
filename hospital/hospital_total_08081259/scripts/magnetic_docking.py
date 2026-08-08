#!/usr/bin/env python3
"""Reliable nearby-bed magnetic coupling for Isaac Sim.

C (or /amr1/magnet_lock) searches all configured and discovered hospital beds,
checks the actual world-space bed bound, and creates a FixedJoint without
teleporting the bed. X releases the joint.
"""
from __future__ import annotations

import math
from typing import Any, MutableSet

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


def world_matrix(prim: Usd.Prim) -> Gf.Matrix4d:
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def quatd_to_quatf(value: Gf.Quatd) -> Gf.Quatf:
    imag = value.GetImaginary()
    return Gf.Quatf(
        float(value.GetReal()),
        Gf.Vec3f(float(imag[0]), float(imag[1]), float(imag[2])),
    )


def _has_rigid_body(root: Usd.Prim) -> bool:
    if root.HasAPI(UsdPhysics.RigidBodyAPI):
        return True
    return any(prim.HasAPI(UsdPhysics.RigidBodyAPI) for prim in Usd.PrimRange(root))


def _model_root_for_match(prim: Usd.Prim) -> Usd.Prim:
    """Keep the bed model root, but never climb all the way to HospitalMap/Map."""
    current = prim
    while current and current.IsValid():
        parent = current.GetParent()
        if not parent or not parent.IsValid():
            break
        parent_path = str(parent.GetPath())
        parent_name = parent.GetName().lower()
        if parent_path == "/World" or parent_name in {"map", "hospitalmap", "environment"}:
            break
        # A referenced/model prim or a prim containing rigid bodies is a useful bed root.
        if current.IsModel() or _has_rigid_body(current):
            break
        current = parent
    return current


def discover_bed_roots(
    stage: Usd.Stage,
    configured: list[str],
    name_keywords: list[str] | None = None,
) -> list[str]:
    """Discover bed roots recursively, including patient-named bed prims."""
    found: list[str] = []
    keywords = [
        item.lower().replace(" ", "").replace("_", "")
        for item in (name_keywords or ["bed", "kimseoul", "parkincheon", "seosuwon"])
    ]

    def add_prim(prim: Usd.Prim) -> None:
        if not prim or not prim.IsValid():
            return
        root = _model_root_for_match(prim)
        path = str(root.GetPath())
        if path in {"/", "/World", "/World/Map", "/World/HospitalMap"}:
            return
        if "amr" in root.GetName().lower():
            return
        if path not in found and _has_rigid_body(root):
            found.append(path)

    for path in configured:
        prim = stage.GetPrimAtPath(str(path))
        if prim and prim.IsValid():
            add_prim(prim)

    for prim in stage.Traverse():
        raw_name = prim.GetName().lower()
        normal_name = raw_name.replace(" ", "").replace("_", "")
        if "amr" in normal_name:
            continue
        if any(keyword in normal_name for keyword in keywords):
            add_prim(prim)

    return found


class MagneticDockController:
    """C locks the nearest bed; X releases it."""

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
        self.joint_path = f"{amr_root_path}/Joints/magnetic_bed_joint"
        self.candidate_bed_paths = list(candidate_bed_paths)
        self.claimed_beds = claimed_beds

        self.base_prim = stage.GetPrimAtPath(self.base_path)
        self.lift_prim = stage.GetPrimAtPath(self.lift_path)
        self.available = bool(self.cfg.get("enabled", True))
        self.enabled = bool(self.cfg.get("auto_enabled_on_start", True))
        self.locked = False
        self.attached_bed_path: str | None = None
        self.attached_body_path: str | None = None
        # Runtime-only material overrides for bed caster/wheel colliders.
        # They are applied only while this AMR is magnetically attached and
        # restored on release, so bed body/wall collision remains untouched.
        self._caster_material_restore: dict[str, tuple[bool, list[Sdf.Path]]] = {}
        self.last_state = "READY" if self.enabled else "OFF"

        if not self.base_prim or not self.base_prim.IsValid():
            self.available = False
            self.last_state = f"UNAVAILABLE: missing {self.base_path}"
        if not self.lift_prim or not self.lift_prim.IsValid():
            # Fall back to base_link so magnetic towing still works on stages
            # whose lift plate is visual-only or named differently.
            self.lift_prim = self.base_prim
            self.lift_path = self.base_path
            print(f"[{self.root_path} magnet] lift_plate missing; using base_link as joint body")

        print(
            f"[{self.root_path} magnet] available={self.available} enabled={self.enabled} "
            f"beds={self.candidate_bed_paths or 'NONE'}"
        )

    @property
    def state(self) -> str:
        return self.last_state

    @property
    def attached_bed(self) -> str:
        return self.attached_bed_path or ""

    @property
    def attached_body(self) -> str:
        return self.attached_body_path or ""

    @staticmethod
    def _zero_velocity(prim: Usd.Prim) -> None:
        if not prim or not prim.IsValid() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return
        body = UsdPhysics.RigidBodyAPI(prim)
        velocity = body.GetVelocityAttr()
        angular = body.GetAngularVelocityAttr()
        if velocity:
            velocity.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        if angular:
            angular.Set(Gf.Vec3f(0.0, 0.0, 0.0))

    @staticmethod
    def _rigid_bodies(root: Usd.Prim) -> list[Usd.Prim]:
        bodies: list[Usd.Prim] = []
        if root.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies.append(root)
        for prim in Usd.PrimRange(root):
            if prim == root:
                continue
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                bodies.append(prim)
        return bodies

    @classmethod
    def _best_rigid_body(cls, root: Usd.Prim) -> Usd.Prim | None:
        bodies = cls._rigid_bodies(root)
        if not bodies:
            return None

        def score(prim: Usd.Prim) -> tuple[int, int]:
            name = prim.GetName().lower()
            preferred = any(token in name for token in ("frame", "proxy", "body", "bed"))
            wheel = any(token in name for token in ("wheel", "caster"))
            return (2 if preferred else 0) - (3 if wheel else 0), -len(str(prim.GetPath()))

        return max(bodies, key=score)

    @staticmethod
    def _world_center(root: Usd.Prim) -> Gf.Vec3d:
        try:
            cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
                useExtentsHint=True,
            )
            box = cache.ComputeWorldBound(root).ComputeAlignedBox()
            if not box.IsEmpty():
                center = box.GetCenter()
                return Gf.Vec3d(float(center[0]), float(center[1]), float(center[2]))
        except Exception:
            pass
        pos = world_matrix(root).ExtractTranslation()
        return Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2]))

    @staticmethod
    def _has_enabled_collision(prim: Usd.Prim) -> bool:
        if not prim or not prim.IsValid():
            return False
        try:
            attr = prim.GetAttribute("physics:collisionEnabled")
            if attr and attr.IsValid() and attr.Get() is False:
                return False
            return bool(prim.HasAPI(UsdPhysics.CollisionAPI) or (attr and attr.IsValid()))
        except Exception:
            return False

    @staticmethod
    def _is_caster_collision(prim: Usd.Prim, bed_root: Usd.Prim) -> bool:
        """Match only collision geometry below wheel/caster-named bed branches."""
        if not MagneticDockController._has_enabled_collision(prim):
            return False
        root_path = str(bed_root.GetPath())
        current = prim
        while current and current.IsValid():
            name = current.GetName().lower().replace("_", "").replace(" ", "")
            if any(token in name for token in ("wheel", "caster", "roller")):
                return True
            if str(current.GetPath()) == root_path:
                break
            current = current.GetParent()
        return False

    def _set_attached_caster_low_friction(self, enabled: bool) -> None:
        """Reduce only attached-bed caster friction; restore it on release.

        This deliberately does NOT disable collision, does NOT touch the bed frame,
        and does NOT modify AMR wheel materials.  It only prevents bed casters from
        repeatedly sticking to the floor while a FixedJoint is towing the bed.
        """
        cfg = self.cfg.get("attached_caster_low_friction", {})
        if not bool(cfg.get("enabled", True)):
            return

        previous_target = self.stage.GetEditTarget()
        try:
            self.stage.SetEditTarget(Usd.EditTarget(self.stage.GetSessionLayer()))

            if not enabled:
                restored = 0
                for path, (had_direct_binding, targets) in list(self._caster_material_restore.items()):
                    prim = self.stage.GetPrimAtPath(path)
                    if not prim or not prim.IsValid():
                        continue
                    rel = prim.GetRelationship("material:binding:physics")
                    if had_direct_binding:
                        if not rel or not rel.IsValid():
                            rel = prim.CreateRelationship("material:binding:physics", custom=False)
                        rel.SetTargets(targets)
                    else:
                        try:
                            prim.RemoveProperty("material:binding:physics")
                        except Exception:
                            if rel and rel.IsValid():
                                rel.ClearTargets(True)
                    restored += 1
                if self._caster_material_restore:
                    print(f"[{self.root_path} BED CASTER] friction restored on {restored} collider(s)")
                self._caster_material_restore.clear()
                return

            if not self.attached_bed_path:
                return
            bed_root = self.stage.GetPrimAtPath(self.attached_bed_path)
            if not bed_root or not bed_root.IsValid():
                print(f"[{self.root_path} BED CASTER WARNING] attached bed root missing: {self.attached_bed_path}")
                return

            material_path = Sdf.Path("/World/HospitalRuntimeMaterials/AttachedBedCasterLowFriction")
            UsdGeom.Xform.Define(self.stage, material_path.GetParentPath())
            material = UsdShade.Material.Define(self.stage, material_path)
            physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
            static_friction = float(cfg.get("static_friction", 0.02))
            dynamic_friction = float(cfg.get("dynamic_friction", 0.01))
            physics_material.CreateStaticFrictionAttr(static_friction).Set(static_friction)
            physics_material.CreateDynamicFrictionAttr(dynamic_friction).Set(dynamic_friction)
            try:
                physics_material.CreateRestitutionAttr(0.0).Set(0.0)
            except Exception:
                pass

            matched = 0
            for prim in Usd.PrimRange(bed_root):
                if not self._is_caster_collision(prim, bed_root):
                    continue
                path = str(prim.GetPath())
                if path in self._caster_material_restore:
                    continue
                rel = prim.GetRelationship("material:binding:physics")
                had_direct = bool(rel and rel.IsValid())
                targets = list(rel.GetTargets()) if had_direct else []
                self._caster_material_restore[path] = (had_direct, targets)

                binding = UsdShade.MaterialBindingAPI.Apply(prim)
                try:
                    binding.Bind(material, materialPurpose="physics")
                except Exception:
                    try:
                        binding.Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
                    except Exception:
                        rel = prim.CreateRelationship("material:binding:physics", custom=False)
                        rel.SetTargets([material.GetPath()])
                matched += 1

            if matched:
                print(
                    f"[{self.root_path} BED CASTER] low friction ON: "
                    f"colliders={matched} static={static_friction:.3f} dynamic={dynamic_friction:.3f}"
                )
            else:
                print(
                    f"[{self.root_path} BED CASTER] no wheel/caster collision prim found; "
                    "bed collision left unchanged"
                )
        except Exception as exc:
            print(f"[{self.root_path} BED CASTER WARNING] friction override failed safely: {exc}")
        finally:
            self.stage.SetEditTarget(previous_target)

    def toggle(self) -> None:
        if not self.available:
            print(f"[{self.root_path} magnet] {self.last_state}")
            return
        self.enabled = not self.enabled
        if not self.enabled:
            self.release("magnet toggled OFF")
            self.last_state = "OFF"
        else:
            self.last_state = "READY"
        print(f"[{self.root_path} magnet] {'ON' if self.enabled else 'OFF'}")

    def _nearest_bed(self) -> tuple[str | None, Usd.Prim | None, float]:
        base_pos = self._world_center(self.base_prim)
        nearest_path: str | None = None
        nearest_prim: Usd.Prim | None = None
        nearest_distance = math.inf
        for path in self.candidate_bed_paths:
            if path in self.claimed_beds and path != self.attached_bed_path:
                continue
            prim = self.stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                continue
            pos = self._world_center(prim)
            distance = math.hypot(float(pos[0] - base_pos[0]), float(pos[1] - base_pos[1]))
            if distance < nearest_distance:
                nearest_path = path
                nearest_prim = prim
                nearest_distance = distance
        return nearest_path, nearest_prim, nearest_distance

    def _create_joint(self, bed_root_path: str, bed_body: Usd.Prim) -> None:
        old = self.stage.GetPrimAtPath(self.joint_path)
        if old and old.IsValid():
            self.stage.RemovePrim(self.joint_path)

        joints_scope = f"{self.root_path}/Joints"
        if not self.stage.GetPrimAtPath(joints_scope).IsValid():
            self.stage.DefinePrim(joints_scope, "Scope")

        amr_body = self._best_rigid_body(self.lift_prim) or self._best_rigid_body(self.base_prim)
        if amr_body is None:
            raise RuntimeError(f"No AMR RigidBody found below {self.lift_path} or {self.base_path}")

        amr_world = world_matrix(amr_body)
        bed_world = world_matrix(bed_body)
        anchor_world = amr_world.ExtractTranslation()
        local_pos_bed_d = bed_world.GetInverse().Transform(anchor_world)
        local_pos_bed = Gf.Vec3f(
            float(local_pos_bed_d[0]),
            float(local_pos_bed_d[1]),
            float(local_pos_bed_d[2]),
        )
        local_rot_bed = bed_world.ExtractRotationQuat().GetInverse() * amr_world.ExtractRotationQuat()

        joint = UsdPhysics.FixedJoint.Define(self.stage, self.joint_path)
        joint.CreateBody0Rel().SetTargets([amr_body.GetPath()])
        joint.CreateBody1Rel().SetTargets([bed_body.GetPath()])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(local_pos_bed)
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        joint.CreateLocalRot1Attr().Set(quatd_to_quatf(local_rot_bed))
        joint.CreateCollisionEnabledAttr(False)
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateJointEnabledAttr(True)
        joint.CreateBreakForceAttr().Set(float(self.cfg.get("break_force_n", 1.0e9)))
        joint.CreateBreakTorqueAttr().Set(float(self.cfg.get("break_torque_nm", 1.0e9)))

        self._zero_velocity(amr_body)
        self._zero_velocity(bed_body)
        self.locked = True
        self.attached_bed_path = bed_root_path
        self.attached_body_path = str(bed_body.GetPath())
        self.claimed_beds.add(bed_root_path)
        self.last_state = f"LOCKED:{bed_root_path}"
        self._set_attached_caster_low_friction(True)
        print(
            f"\a[{self.root_path} magnet] 철컥! CLACK! LOCKED {bed_root_path} "
            f"amrBody={amr_body.GetPath()} bedBody={bed_body.GetPath()}"
        )

    def request_lock(self) -> bool:
        """Attach the nearest bed while preserving its current pose."""
        print(f"[{self.root_path} magnet] C/LOCK command received")
        if not self.available:
            print(f"[{self.root_path} magnet] LOCK FAILED: {self.last_state}")
            return False
        if self.locked:
            print(f"[{self.root_path} magnet] already locked to {self.attached_bed_path}")
            return True
        if not self.enabled:
            self.enabled = True
            print(f"[{self.root_path} magnet] auto-armed")

        bed_path, bed_root, distance = self._nearest_bed()
        capture = float(self.cfg.get("capture_distance_m", 1.20))
        if bed_path is None or bed_root is None:
            self.last_state = "NO_BED_FOUND"
            print(
                f"[{self.root_path} magnet] LOCK FAILED: no bed candidate. "
                f"discovered={self.candidate_bed_paths or 'NONE'}"
            )
            return False

        print(f"[{self.root_path} magnet] nearest={bed_path} distance={distance:.3f}m limit={capture:.3f}m")
        if distance > capture:
            self.last_state = f"TOO_FAR:{distance:.3f}m"
            print(f"[{self.root_path} magnet] LOCK FAILED: move AMR closer to the bed centre")
            return False

        body = self._best_rigid_body(bed_root)
        if body is None:
            self.last_state = f"NO_RIGID_BODY:{bed_path}"
            print(f"[{self.root_path} magnet] LOCK FAILED: no RigidBody below {bed_path}")
            return False

        try:
            self._create_joint(bed_path, body)
        except Exception as exc:
            self.last_state = f"JOINT_ERROR:{type(exc).__name__}"
            print(f"[{self.root_path} magnet] LOCK FAILED: joint creation error: {exc}")
            return False
        return True

    def release(self, reason: str = "manual release") -> None:
        self._set_attached_caster_low_friction(False)
        joint = self.stage.GetPrimAtPath(self.joint_path)
        if joint and joint.IsValid():
            self.stage.RemovePrim(self.joint_path)
        if self.attached_bed_path:
            self.claimed_beds.discard(self.attached_bed_path)
            print(f"[{self.root_path} magnet] RELEASED {self.attached_bed_path}: {reason}")
        else:
            print(f"[{self.root_path} magnet] release requested: no bed attached")
        self.locked = False
        self.attached_bed_path = None
        self.attached_body_path = None
        self.last_state = "READY" if self.enabled else "OFF"
