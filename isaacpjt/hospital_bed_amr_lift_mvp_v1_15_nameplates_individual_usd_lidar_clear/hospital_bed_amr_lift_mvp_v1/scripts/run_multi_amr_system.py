#!/usr/bin/env python3
"""Run two hospital-bed AMRs with keyboard and namespaced ROS 2 control."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

try:
    from isaacsim.simulation_app import SimulationApp
except ImportError:
    from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--stage", required=True)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


ARGS = parse_args()
APP = SimulationApp({"headless": ARGS.headless})

import carb
import omni
import omni.appwindow
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from fleet_magnetic_docking import FleetMagneticDockController, clamp, move_toward
from navigation_math import quaternion_from_yaw, relative_planar_pose, split_sim_time


def enable_extension(extension_id: str) -> None:
    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.is_extension_enabled(extension_id):
        manager.set_extension_enabled_immediate(extension_id, True)
    for _ in range(20):
        APP.update()


def get_drive(stage: Usd.Stage, path: str, drive_name: str) -> UsdPhysics.DriveAPI:
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Missing joint prim: {path}")
    drive = UsdPhysics.DriveAPI.Get(prim, drive_name)
    if not drive:
        raise RuntimeError(f"Missing {drive_name} drive: {path}")
    return drive


def planar_yaw_from_matrix(matrix: Gf.Matrix4d) -> float:
    """Extract the AMR's planar heading from its local +X axis."""
    forward = matrix.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
    return math.atan2(float(forward[1]), float(forward[0]))


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Return an x, y, z, w quaternion for ROS TransformStamped."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def ensure_rtx_lidar(stage: Usd.Stage, lidar_path: str, lidar_config: str) -> Usd.Prim:
    import omni.kit.commands

    prim = stage.GetPrimAtPath(lidar_path)
    if prim and prim.IsValid():
        return prim
    parent, child = lidar_path.rsplit("/", 1)
    success, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=child,
        parent=parent,
        config=lidar_config,
        translation=Gf.Vec3d(0.0, 0.0, 0.0),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        visiblity=False,
        variant=None,
        force_camera_prim=False,
        **{"omni:sensor:Core:scanRateBaseHz": 10},
    )
    if not success or sensor is None:
        raise RuntimeError(f"Failed to create RTX LiDAR: {lidar_path}")
    actual = stage.GetPrimAtPath(str(sensor.GetPath()))
    if not actual or not actual.IsValid():
        raise RuntimeError(f"Invalid RTX LiDAR: {lidar_path}")
    return actual


def setup_sensor_writers(cfg: dict[str, Any], unit: dict[str, Any]) -> tuple[list[Any], list[Any]]:
    """Create namespaced camera/LiDAR ROS writers for one AMR."""
    import omni.graph.core as og
    import omni.replicator.core as rep
    import omni.syntheticdata._syntheticdata as sd
    from isaacsim.ros2.bridge import read_camera_info
    from isaacsim.sensors.camera import Camera

    sensors = cfg["sensors"]
    shared = cfg["amr"]
    root = str(unit["root_path"])
    namespace = str(unit["namespace"]).strip("/")
    writers: list[Any] = []
    products: list[Any] = []

    if bool(unit.get("camera_enabled", True)):
        resolution = tuple(map(int, sensors["camera_resolution"]))
        frequency = float(sensors["camera_frequency_hz"])
        camera_definitions = sensors.get("cameras") or [
            {
                "name": "front",
                "prim_suffix": sensors.get("camera_prim_suffix", "base_link/front_camera_link/Camera"),
                "frame_suffix": sensors.get("camera_frame_suffix", "front_camera_link"),
                "rgb_topic": sensors.get("camera_rgb_topic", "camera/front/color/image_raw"),
                "depth_topic": sensors.get("camera_depth_topic", "camera/front/depth/image_raw"),
                "camera_info_topic": sensors.get("camera_info_topic", "camera/front/camera_info"),
            }
        ]

        camera_warmup_frames = max(2, int(sensors.get("camera_warmup_frames", 8)))
        camera_queue_size = max(1, int(sensors.get("camera_queue_size", 2)))

        for camera_cfg in camera_definitions:
            camera_name = str(camera_cfg["name"])
            camera_prim = f"{root}/{camera_cfg['prim_suffix']}"
            camera_prim_obj = omni.usd.get_context().get_stage().GetPrimAtPath(camera_prim)
            if not camera_prim_obj or not camera_prim_obj.IsValid():
                raise RuntimeError(f"Missing camera prim: {camera_prim}")

            camera = Camera(
                prim_path=camera_prim,
                name=f"{namespace}_{camera_name}_camera",
                frequency=frequency,
                resolution=resolution,
            )

            # Isaac Sim 5.1 camera pipelines need at least one rendered update before
            # the ROS Replicator writers are attached.  A second initialize mirrors
            # NVIDIA's standalone camera example and prevents topics being advertised
            # without actual Image messages.
            camera.initialize()
            for _ in range(camera_warmup_frames):
                APP.update()
            camera.initialize()
            APP.update()

            camera_rp = camera._render_product_path
            if not camera_rp:
                raise RuntimeError(f"Camera render product was not created: {camera_prim}")

            frame_id = f"{namespace}/{camera_cfg['frame_suffix']}"
            rgb_topic = f"{namespace}/{camera_cfg['rgb_topic']}"
            depth_topic = f"{namespace}/{camera_cfg['depth_topic']}"
            info_topic = f"{namespace}/{camera_cfg['camera_info_topic']}"
            products.extend([camera, camera_rp])

            rgb_rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
            rgb_writer = rep.writers.get(rgb_rv + "ROS2PublishImage")
            rgb_writer.initialize(
                frameId=frame_id,
                nodeNamespace="",
                queueSize=camera_queue_size,
                topicName=rgb_topic,
            )
            rgb_writer.attach([camera_rp])
            writers.append(rgb_writer)

            depth_rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
            depth_writer = rep.writers.get(depth_rv + "ROS2PublishImage")
            depth_writer.initialize(
                frameId=frame_id,
                nodeNamespace="",
                queueSize=camera_queue_size,
                topicName=depth_topic,
            )
            depth_writer.attach([camera_rp])
            writers.append(depth_writer)

            camera_info, _ = read_camera_info(render_product_path=camera_rp)
            info_writer = rep.writers.get("ROS2PublishCameraInfo")
            info_writer.initialize(
                frameId=frame_id,
                nodeNamespace="",
                queueSize=camera_queue_size,
                topicName=info_topic,
                width=camera_info.width,
                height=camera_info.height,
                projectionType=camera_info.distortion_model,
                k=camera_info.k.reshape([1, 9]),
                r=camera_info.r.reshape([1, 9]),
                p=camera_info.p.reshape([1, 12]),
                physicalDistortionModel=camera_info.distortion_model,
                physicalDistortionCoefficients=camera_info.d,
            )
            info_writer.attach([camera_rp])
            writers.append(info_writer)

            # Allow SDGPipeline writer nodes to be created before changing their rate.
            for _ in range(2):
                APP.update()

            step_size = max(1, int(60.0 / frequency))
            gate_names = (
                rgb_rv + "IsaacSimulationGate",
                depth_rv + "IsaacSimulationGate",
                "PostProcessDispatchIsaacSimulationGate",
            )
            for gate_name in gate_names:
                try:
                    gate = omni.syntheticdata.SyntheticData._get_node_path(gate_name, camera_rp)
                    og.Controller.attribute(gate + ".inputs:step").set(step_size)
                except Exception as exc:
                    print(f"[{namespace} {camera_name} camera warning] {gate_name}: {exc}")

            print(
                f"[camera ready] {namespace}/{camera_name}: prim={camera_prim}, "
                f"render_product={camera_rp}, RGB=/{rgb_topic}, depth=/{depth_topic}, info=/{info_topic}"
            )

    if bool(unit.get("lidar_enabled", True)):
        lidar_path = f"{root}/{sensors.get('lidar_prim_suffix', 'base_link/lidar_link/RtxLidar')}"
        lidar_prim = ensure_rtx_lidar(stage=omni.usd.get_context().get_stage(), lidar_path=lidar_path, lidar_config=str(shared.get("lidar_config", "Example_Rotary_2D")))
        lidar_rp = rep.create.render_product(lidar_prim.GetPath(), [1, 1], name=f"{namespace}_lidar")
        products.append(lidar_rp)
        frame_id = f"{namespace}/{sensors.get('lidar_frame_suffix', 'lidar_link')}"

        scan_writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
        scan_writer.initialize(
            topicName=f"{namespace}/{sensors['lidar_scan_topic']}",
            frameId=frame_id,
        )
        scan_writer.attach([lidar_rp])
        writers.append(scan_writer)

        point_writer = rep.writers.get("RtxLidarROS2PublishPointCloud")
        point_writer.initialize(
            topicName=f"{namespace}/{sensors['lidar_pointcloud_topic']}",
            frameId=frame_id,
        )
        point_writer.attach([lidar_rp])
        writers.append(point_writer)

    return writers, products


class AMRController:
    def __init__(
        self,
        stage: Usd.Stage,
        cfg: dict[str, Any],
        unit: dict[str, Any],
        claimed_beds: set[str],
    ) -> None:
        self.stage = stage
        self.cfg = cfg
        self.unit = unit
        self.shared = cfg["amr"]
        self.control = cfg["control"]
        self.root = str(unit["root_path"])
        self.namespace = str(unit["namespace"]).strip("/")
        self.name = str(unit["name"])

        self.base_path = f"{self.root}/base_link"
        self.base_prim = stage.GetPrimAtPath(self.base_path)
        if not self.base_prim or not self.base_prim.IsValid():
            raise RuntimeError(f"Missing base: {self.base_path}")
        initial_world = UsdGeom.Xformable(self.base_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        initial_position = initial_world.ExtractTranslation()
        self.odom_origin_position = (
            float(initial_position[0]),
            float(initial_position[1]),
            float(initial_position[2]),
        )
        self.odom_origin_yaw = planar_yaw_from_matrix(initial_world)

        rigid_body = UsdPhysics.RigidBodyAPI(self.base_prim)
        self.velocity_attr = rigid_body.GetVelocityAttr()
        self.angular_velocity_attr = rigid_body.GetAngularVelocityAttr()
        self.lift_drive = get_drive(stage, f"{self.root}/Joints/lift_joint", "linear")
        self.wheel_drives = {
            name: get_drive(stage, f"{self.root}/Joints/wheel_joint_{name}", "angular")
            for name in ("FL", "FR", "RL", "RR")
        }
        self.magnet = FleetMagneticDockController(
            stage,
            cfg,
            self.root,
            [str(b["root_path"]) for b in cfg.get("beds", [])],
            claimed_beds,
        )

        self.lower = float(self.shared["lift_lower_limit_m"])
        self.upper = float(self.shared["lift_upper_limit_m"])
        self.lift_target = float(self.lift_drive.GetTargetPositionAttr().Get() or self.lower)
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.ros_vx = 0.0
        self.ros_vy = 0.0
        self.ros_wz = 0.0
        self.last_ros_command_time = -math.inf
        self.ros_estop = False
        self.pending_lift_target: float | None = None

    @property
    def loaded(self) -> bool:
        # Wheel-tow mode keeps the casters on the floor, but the speed profile should
        # still switch to the safer loaded limits while a bed is magnetically coupled.
        return self.magnet.locked or self.lift_target >= float(self.control["lift_loaded_threshold_m"])

    def get_odom_pose(self) -> tuple[float, float, float, float]:
        """Return x, y, z, yaw relative to the AMR startup pose."""
        matrix = UsdGeom.Xformable(self.base_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        position = matrix.ExtractTranslation()
        current_position = (float(position[0]), float(position[1]), float(position[2]))
        current_yaw = planar_yaw_from_matrix(matrix)
        return relative_planar_pose(
            current_position=current_position,
            current_yaw=current_yaw,
            origin_position=self.odom_origin_position,
            origin_yaw=self.odom_origin_yaw,
        )

    def set_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        self.ros_vx = float(vx)
        self.ros_vy = float(vy)
        self.ros_wz = float(wz)
        self.last_ros_command_time = time.monotonic()

    def set_lift_target(self, value: float) -> None:
        self.pending_lift_target = clamp(float(value), self.lower, self.upper)

    def set_magnet_enabled(self, enabled: bool) -> None:
        if enabled and not self.magnet.enabled:
            self.magnet.toggle()
        elif not enabled and self.magnet.enabled:
            self.magnet.toggle()

    def set_magnet_strength(self, percent: float) -> None:
        self.magnet.set_strength(percent)

    def request_magnet_lock(self) -> None:
        self.magnet.request_lock()

    def request_magnet_release(self) -> None:
        self.magnet.request_release()

    def set_estop(self, enabled: bool) -> None:
        self.ros_estop = bool(enabled)

    def update(
        self,
        dt: float,
        keyboard_command: tuple[float, float, float] | None,
        keyboard_lift_direction: float,
        keyboard_estop: bool,
    ) -> None:
        if keyboard_lift_direction:
            self.lift_target = clamp(
                self.lift_target + keyboard_lift_direction * float(self.control["lift_command_speed_mps"]) * dt,
                self.lower,
                self.upper,
            )
            self.pending_lift_target = None
        elif self.pending_lift_target is not None:
            self.lift_target = self.pending_lift_target
            self.pending_lift_target = None

        # In wheel-tow mode the lift only closes the small adapter gap.  It is
        # deliberately prevented from raising the bed far enough to unload its casters.
        if self.magnet.is_wheel_tow_mode and self.magnet.enabled:
            if self.magnet.target_bed_path or self.magnet.locked:
                self.lift_target = self.magnet.wheel_tow_contact_lift_m
            else:
                self.lift_target = min(self.lift_target, self.magnet.wheel_tow_max_lift_m)
        self.lift_drive.GetTargetPositionAttr().Set(float(self.lift_target))

        recent_ros = time.monotonic() - self.last_ros_command_time <= float(self.control.get("ros_command_timeout_s", 0.6))
        if keyboard_command is not None and bool(self.control.get("keyboard_has_priority", True)):
            raw_vx, raw_vy, raw_wz = keyboard_command
        elif recent_ros:
            raw_vx, raw_vy, raw_wz = self.ros_vx, self.ros_vy, self.ros_wz
        elif keyboard_command is not None:
            raw_vx, raw_vy, raw_wz = keyboard_command
        else:
            raw_vx = raw_vy = raw_wz = 0.0

        estop = self.ros_estop or keyboard_estop
        if estop:
            raw_vx = raw_vy = raw_wz = 0.0

        if self.loaded:
            linear_limit = float(self.control["loaded_linear_speed_mps"])
            lateral_limit = float(self.control["loaded_lateral_speed_mps"])
            angular_limit = float(self.control["loaded_angular_speed_rad_s"])
            linear_accel = float(self.control["loaded_linear_accel_mps2"])
            lateral_accel = float(self.control.get("loaded_lateral_accel_mps2", linear_accel))
            angular_accel = float(self.control["loaded_angular_accel_rad_s2"])
        else:
            linear_limit = float(self.control["unloaded_linear_speed_mps"])
            lateral_limit = float(self.control["unloaded_lateral_speed_mps"])
            angular_limit = float(self.control["unloaded_angular_speed_rad_s"])
            linear_accel = float(self.control["unloaded_linear_accel_mps2"])
            lateral_accel = float(self.control.get("unloaded_lateral_accel_mps2", linear_accel))
            angular_accel = float(self.control["unloaded_angular_accel_rad_s2"])

        assist_vx, assist_vy, assist_wz = self.magnet.update(self.lift_target, emergency_stop=estop)
        target_vx = clamp(raw_vx + assist_vx, -linear_limit, linear_limit)
        target_vy = clamp(raw_vy + assist_vy, -lateral_limit, lateral_limit)
        target_wz = clamp(raw_wz + assist_wz, -angular_limit, angular_limit)

        if estop:
            self.current_vx = self.current_vy = self.current_wz = 0.0
        else:
            self.current_vx = move_toward(self.current_vx, target_vx, linear_accel * dt)
            self.current_vy = move_toward(self.current_vy, target_vy, lateral_accel * dt)
            self.current_wz = move_toward(self.current_wz, target_wz, angular_accel * dt)

        world_matrix = UsdGeom.Xformable(self.base_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        world_velocity = world_matrix.TransformDir(Gf.Vec3d(self.current_vx, self.current_vy, 0.0))
        self.velocity_attr.Set(Gf.Vec3f(float(world_velocity[0]), float(world_velocity[1]), 0.0))
        self.angular_velocity_attr.Set(Gf.Vec3f(0.0, 0.0, float(math.degrees(self.current_wz))))

        radius = float(self.shared["wheel_radius"])
        lever = float(self.shared["wheel_x"]) + float(self.shared["wheel_y"])
        wheel_rad_s = {
            "FL": (self.current_vx - self.current_vy - lever * self.current_wz) / radius,
            "FR": (self.current_vx + self.current_vy + lever * self.current_wz) / radius,
            "RL": (self.current_vx + self.current_vy - lever * self.current_wz) / radius,
            "RR": (self.current_vx - self.current_vy + lever * self.current_wz) / radius,
        }
        max_wheel = float(self.control["wheel_visual_velocity_limit_deg_s"])
        for name, wheel_value in wheel_rad_s.items():
            self.wheel_drives[name].GetTargetVelocityAttr().Set(
                float(clamp(math.degrees(wheel_value), -max_wheel, max_wheel))
            )

    def stop(self) -> None:
        self.velocity_attr.Set(Gf.Vec3f(0.0))
        self.angular_velocity_attr.Set(Gf.Vec3f(0.0))
        for drive in self.wheel_drives.values():
            drive.GetTargetVelocityAttr().Set(0.0)
        self.magnet.release("application closing")


class FleetRosBridge:
    """ROS 2 navigation bridge for stages 1-6.

    Provides one shared simulation clock, per-AMR cmd_vel subscribers,
    startup-zeroed ground-truth odometry, dynamic odom->base_link TF, and
    static base_link->sensor transforms.
    """

    def __init__(self, cfg: dict[str, Any], controllers: list[AMRController]) -> None:
        self.available = False
        self.cfg = cfg
        self.controllers = controllers
        self.node = None
        self.odom_publishers: dict[str, Any] = {}
        self.bed_publishers: dict[str, Any] = {}
        self.last_clock_publish = -math.inf
        self.last_state_publish = -math.inf
        self.timeline = omni.timeline.get_timeline_interface()
        self.navigation = cfg.get("navigation", {})
        self.clock_period = 1.0 / max(
            1.0, float(self.navigation.get("clock_publish_rate_hz", 60.0))
        )
        self.state_period = 1.0 / max(
            1.0, float(self.navigation.get("state_publish_rate_hz", 30.0))
        )
        try:
            import rclpy
            from builtin_interfaces.msg import Time as RosTime
            from geometry_msgs.msg import TransformStamped, Twist
            from nav_msgs.msg import Odometry
            from rosgraph_msgs.msg import Clock
            from std_msgs.msg import Bool, Float64, String
            from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
        except Exception as exc:
            print(f"[ROS navigation warning] rclpy interfaces unavailable: {exc}")
            return

        self.rclpy = rclpy
        self.RosTime = RosTime
        self.TransformStamped = TransformStamped
        self.Odometry = Odometry
        self.Clock = Clock
        self.Bool = Bool
        self.String = String
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node("hospital_bed_amr_navigation_bridge")
        self.tf = TransformBroadcaster(self.node)
        self.static_tf = StaticTransformBroadcaster(self.node)
        sensors = cfg["sensors"]

        clock_topic = str(self.navigation.get("clock_topic", "/clock"))
        self.clock_publisher = self.node.create_publisher(Clock, clock_topic, 10)

        for controller in controllers:
            ns = controller.namespace
            self.node.create_subscription(
                Twist,
                f"/{ns}/{sensors.get('cmd_vel_topic', 'cmd_vel')}",
                lambda msg, c=controller: c.set_cmd_vel(
                    msg.linear.x, msg.linear.y, msg.angular.z
                ),
                10,
            )
            self.node.create_subscription(
                Float64,
                f"/{ns}/{sensors.get('lift_cmd_topic', 'lift_cmd')}",
                lambda msg, c=controller: c.set_lift_target(msg.data),
                10,
            )
            self.node.create_subscription(
                Bool,
                f"/{ns}/{sensors.get('magnet_cmd_topic', 'magnet_cmd')}",
                lambda msg, c=controller: c.set_magnet_enabled(msg.data),
                10,
            )
            self.node.create_subscription(
                Float64,
                f"/{ns}/{sensors.get('magnet_strength_topic', 'magnet_strength')}",
                lambda msg, c=controller: c.set_magnet_strength(msg.data),
                10,
            )
            self.node.create_subscription(
                Bool,
                f"/{ns}/{sensors.get('magnet_lock_topic', 'magnet_lock')}",
                lambda msg, c=controller: c.request_magnet_lock() if msg.data else None,
                10,
            )
            self.node.create_subscription(
                Bool,
                f"/{ns}/{sensors.get('magnet_release_topic', 'magnet_release')}",
                lambda msg, c=controller: c.request_magnet_release() if msg.data else None,
                10,
            )
            self.node.create_subscription(
                Bool,
                f"/{ns}/{sensors.get('estop_topic', 'estop')}",
                lambda msg, c=controller: c.set_estop(msg.data),
                10,
            )
            self.odom_publishers[ns] = self.node.create_publisher(
                Odometry, f"/{ns}/{sensors.get('odom_topic', 'odom')}", 10
            )
            self.bed_publishers[ns] = self.node.create_publisher(
                String,
                f"/{ns}/{sensors.get('bed_attached_topic', 'bed_attached')}",
                10,
            )

        self._publish_static_transforms()
        self.available = True
        print("[ROS navigation] Stage 1: shared /clock publisher enabled")
        print("[ROS navigation] Stage 2: namespaced ground-truth /odom enabled")
        print("[ROS navigation] Stage 3: odom -> base_link dynamic TF enabled")
        print("[ROS navigation] Stage 4: base_link -> lidar/camera static TF enabled")
        print("[ROS navigation] Stage 5: namespaced /cmd_vel subscribers enabled")
        print("[ROS navigation] Stage 6: use scripts/check_nav_stage_1_to_6.sh")

    def _zero_stamp(self) -> Any:
        return self.RosTime(sec=0, nanosec=0)

    def _simulation_stamp(self) -> Any:
        seconds = float(self.timeline.get_current_time())
        sec, nanosec = split_sim_time(seconds)
        return self.RosTime(sec=sec, nanosec=nanosec)

    def _make_transform(
        self,
        parent: str,
        child: str,
        translation: tuple[float, float, float],
        quaternion: tuple[float, float, float, float],
        stamp: Any | None = None,
    ) -> Any:
        msg = self.TransformStamped()
        msg.header.stamp = stamp if stamp is not None else self._zero_stamp()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(translation[0])
        msg.transform.translation.y = float(translation[1])
        msg.transform.translation.z = float(translation[2])
        msg.transform.rotation.x = float(quaternion[0])
        msg.transform.rotation.y = float(quaternion[1])
        msg.transform.rotation.z = float(quaternion[2])
        msg.transform.rotation.w = float(quaternion[3])
        return msg

    def _publish_static_transforms(self) -> None:
        shared = self.cfg["amr"]
        messages = []
        publish_world = bool(
            self.navigation.get("publish_world_to_odom_for_rviz", True)
        )
        world_frame = str(self.navigation.get("world_frame", "world"))
        for c in self.controllers:
            ns = c.namespace
            if publish_world:
                messages.append(
                    self._make_transform(
                        world_frame,
                        f"{ns}/odom",
                        c.odom_origin_position,
                        quaternion_from_yaw(c.odom_origin_yaw),
                    )
                )
            messages.append(
                self._make_transform(
                    f"{ns}/base_link",
                    f"{ns}/lidar_link",
                    (
                        float(shared["lidar_position"][0]),
                        float(shared["lidar_position"][1]),
                        float(
                            shared["lidar_position"][2]
                            - shared["base_center_z"]
                        ),
                    ),
                    (0.0, 0.0, 0.0, 1.0),
                )
            )
            for camera_name in ("front", "rear"):
                position = shared[f"{camera_name}_camera_position"]
                pitch = math.radians(-float(shared.get(f"{camera_name}_camera_pitch_deg", 5.0)))
                yaw = math.radians(float(shared.get(f"{camera_name}_camera_yaw_deg", 0.0)))
                messages.append(
                    self._make_transform(
                        f"{ns}/base_link",
                        f"{ns}/{camera_name}_camera_link",
                        (
                            float(position[0]),
                            float(position[1]),
                            float(position[2] - shared["base_center_z"]),
                        ),
                        quaternion_from_rpy(0.0, pitch, yaw),
                    )
                )
        self.static_tf.sendTransform(messages)

    def _publish_clock(self, stamp: Any) -> None:
        msg = self.Clock()
        msg.clock = stamp
        self.clock_publisher.publish(msg)

    def spin_and_publish(self) -> None:
        if not self.available:
            return
        self.rclpy.spin_once(self.node, timeout_sec=0.0)
        monotonic_now = time.monotonic()
        stamp = self._simulation_stamp()

        if monotonic_now - self.last_clock_publish >= self.clock_period:
            self._publish_clock(stamp)
            self.last_clock_publish = monotonic_now

        if monotonic_now - self.last_state_publish < self.state_period:
            return

        for c in self.controllers:
            x, y, z, yaw = c.get_odom_pose()
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            ns = c.namespace

            tf_msg = self._make_transform(
                f"{ns}/odom",
                f"{ns}/base_link",
                (x, y, z),
                (qx, qy, qz, qw),
                stamp=stamp,
            )
            self.tf.sendTransform(tf_msg)

            odom = self.Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = f"{ns}/odom"
            odom.child_frame_id = f"{ns}/base_link"
            odom.pose.pose.position.x = float(x)
            odom.pose.pose.position.y = float(y)
            odom.pose.pose.position.z = float(z)
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            odom.twist.twist.linear.x = c.current_vx
            odom.twist.twist.linear.y = c.current_vy
            odom.twist.twist.angular.z = c.current_wz
            self.odom_publishers[ns].publish(odom)

            attached = self.String()
            attached.data = c.magnet.attached_bed
            self.bed_publishers[ns].publish(attached)

        self.last_state_publish = monotonic_now

    def close(self) -> None:
        if self.available and self.node is not None:
            self.node.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()

def main() -> int:
    cfg_path = Path(ARGS.config).expanduser().resolve()
    stage_path = Path(ARGS.stage).expanduser().resolve()
    if not stage_path.exists():
        raise FileNotFoundError(stage_path)
    cfg = json.loads(cfg_path.read_text(encoding="utf-8"))

    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.sensors.rtx")
    context = omni.usd.get_context()
    if not context.open_stage(str(stage_path)):
        raise RuntimeError(f"Could not open stage: {stage_path}")
    for _ in range(120):
        APP.update()
    stage = context.get_stage()

    claimed_beds: set[str] = set()
    controllers = [AMRController(stage, cfg, unit, claimed_beds) for unit in cfg["fleet"]]
    all_writers: list[Any] = []
    all_products: list[Any] = []
    for unit in cfg["fleet"]:
        writers, products = setup_sensor_writers(cfg, unit)
        all_writers.extend(writers)
        all_products.extend(products)

    ros_bridge = FleetRosBridge(cfg, controllers)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    app_window = omni.appwindow.get_default_app_window()
    keyboard = app_window.get_keyboard()
    input_interface = carb.input.acquire_input_interface()
    if len(controllers) < 2:
        raise RuntimeError("Dual-keyboard mode requires at least two AMR controllers")

    pressed: set[object] = set()
    magnet_toggle_requested = [False, False]
    magnet_lock_requested = [False, False]
    magnet_release_requested = [False, False]

    def on_keyboard(event, *_args):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            fresh_press = event.input not in pressed
            pressed.add(event.input)
            if fresh_press and event.input == carb.input.KeyboardInput.LEFT_SHIFT:
                magnet_toggle_requested[0] = True
            elif fresh_press and event.input == carb.input.KeyboardInput.RIGHT_SHIFT:
                magnet_toggle_requested[1] = True
            elif fresh_press and event.input == carb.input.KeyboardInput.C:
                magnet_lock_requested[0] = True
            elif fresh_press and event.input == carb.input.KeyboardInput.X:
                magnet_release_requested[0] = True
            elif fresh_press and event.input == carb.input.KeyboardInput.BACKSLASH:
                magnet_lock_requested[1] = True
            elif fresh_press and event.input == carb.input.KeyboardInput.BACKSPACE:
                magnet_release_requested[1] = True
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            pressed.discard(event.input)
        return True

    subscription = input_interface.subscribe_to_keyboard_events(keyboard, on_keyboard)

    print("\n========== Multi-AMR System v1.14: 360 LiDAR + dual RGB-D cameras + hard snap lock ==========")
    print("AMR1: W/S forward/reverse, A/D steer, Q/E omni strafe")
    print("AMR1: R/V lift, LEFT SHIFT arm toggle, C CLACK lock, X release, SPACE estop")
    print("AMR2: UP/DOWN forward/reverse, LEFT/RIGHT steer")
    print("AMR2: hold / and press , or . for left/right omni strafe")
    print("AMR2: [ / ] lift, RIGHT SHIFT arm toggle, BACKSLASH CLACK lock, BACKSPACE release, ENTER estop")
    print("Both AMRs can be controlled at the same time.")
    for c in controllers:
        print(
            f"ROS {c.name}: /{c.namespace}/cmd_vel, /{c.namespace}/lift_cmd, "
            f"/{c.namespace}/magnet_cmd, /{c.namespace}/magnet_strength, "
            f"/{c.namespace}/magnet_lock, /{c.namespace}/magnet_release"
        )
        print(
            f"Magnet {c.name}: mode={c.magnet.coupling_mode}, "
            f"strength={c.magnet.strength_percent:.0f}%"
        )
        print(f"Sensors: /{c.namespace}/scan, /{c.namespace}/point_cloud")
    print("==========================================================\n")

    last = time.monotonic()
    last_report = 0.0
    try:
        while APP.is_running():
            now = time.monotonic()
            dt = clamp(now - last, 0.0, 0.05)
            last = now

            for index in range(2):
                if magnet_toggle_requested[index]:
                    controllers[index].magnet.toggle()
                    magnet_toggle_requested[index] = False
                    print(f"[keyboard] {controllers[index].name} magnet arm toggled")
                if magnet_lock_requested[index]:
                    controllers[index].request_magnet_lock()
                    magnet_lock_requested[index] = False
                if magnet_release_requested[index]:
                    controllers[index].request_magnet_release()
                    magnet_release_requested[index] = False

            # AMR1: WASD + Q/E + R/V
            amr1_forward = float(carb.input.KeyboardInput.W in pressed) - float(carb.input.KeyboardInput.S in pressed)
            amr1_lateral = float(carb.input.KeyboardInput.Q in pressed) - float(carb.input.KeyboardInput.E in pressed)
            amr1_yaw = float(carb.input.KeyboardInput.A in pressed) - float(carb.input.KeyboardInput.D in pressed)
            amr1_lift = float(carb.input.KeyboardInput.R in pressed) - float(carb.input.KeyboardInput.V in pressed)
            amr1_estop = carb.input.KeyboardInput.SPACE in pressed

            # AMR2: arrows steer normally. Hold '/' and press ',' or '.' for omni strafe.
            amr2_forward = float(carb.input.KeyboardInput.UP in pressed) - float(carb.input.KeyboardInput.DOWN in pressed)
            amr2_yaw = float(carb.input.KeyboardInput.LEFT in pressed) - float(carb.input.KeyboardInput.RIGHT in pressed)
            amr2_strafe_mode = carb.input.KeyboardInput.SLASH in pressed
            if amr2_strafe_mode:
                amr2_lateral = float(carb.input.KeyboardInput.COMMA in pressed) - float(carb.input.KeyboardInput.PERIOD in pressed)
            else:
                amr2_lateral = 0.0
            amr2_lift = float(carb.input.KeyboardInput.LEFT_BRACKET in pressed) - float(carb.input.KeyboardInput.RIGHT_BRACKET in pressed)
            amr2_estop = carb.input.KeyboardInput.ENTER in pressed

            keyboard_inputs = [
                (amr1_forward, amr1_lateral, amr1_yaw, amr1_lift, amr1_estop),
                (amr2_forward, amr2_lateral, amr2_yaw, amr2_lift, amr2_estop),
            ]

            for index, controller in enumerate(controllers):
                if index < 2:
                    forward, lateral, yaw, lift_direction, keyboard_estop = keyboard_inputs[index]
                    linear_limit = float(cfg["control"]["loaded_linear_speed_mps"] if controller.loaded else cfg["control"]["unloaded_linear_speed_mps"])
                    lateral_limit = float(cfg["control"]["loaded_lateral_speed_mps"] if controller.loaded else cfg["control"]["unloaded_lateral_speed_mps"])
                    angular_limit = float(cfg["control"]["loaded_angular_speed_rad_s"] if controller.loaded else cfg["control"]["unloaded_angular_speed_rad_s"])
                    keyboard_cmd = (
                        forward * linear_limit,
                        lateral * lateral_limit,
                        yaw * angular_limit,
                    )
                    keyboard_moving = any(abs(value) > 0.0 for value in keyboard_cmd)
                    controller.update(
                        dt,
                        keyboard_cmd if keyboard_moving else None,
                        lift_direction,
                        bool(keyboard_estop),
                    )
                else:
                    controller.update(dt, None, 0.0, False)

            ros_bridge.spin_and_publish()
            if now - last_report > 1.0:
                summary = " | ".join(
                    f"{c.name}:v=({c.current_vx:+.2f},{c.current_vy:+.2f},{c.current_wz:+.2f}) "
                    f"lift={c.lift_target:.3f} magnet={c.magnet.state} "
                    f"mode={c.magnet.coupling_mode} strength={c.magnet.strength_percent:.0f}%"
                    for c in controllers
                )
                print(f"[dual-keyboard] {summary}")
                last_report = now
            APP.update()
    finally:
        for controller in controllers:
            controller.stop()
        timeline.stop()
        for writer in all_writers:
            try:
                writer.detach()
            except Exception:
                pass
        for product in all_products:
            try:
                product.destroy()
            except Exception:
                pass
        ros_bridge.close()
        _ = subscription
    return 0


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except Exception as exc:
        carb.log_error(str(exc))
        print(f"[error] {exc}", file=sys.stderr)
    finally:
        APP.close()
    raise SystemExit(code)
