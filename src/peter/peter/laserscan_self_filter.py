#!/usr/bin/env python3
"""Filter hospital-bed self returns from a ROS 2 LaserScan."""

from __future__ import annotations

import copy
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LaserScanSelfFilter(Node):
    """Remove scan endpoints that lie inside the attached bed footprint."""

    def __init__(self) -> None:
        super().__init__('laserscan_self_filter')

        self.declare_parameter('input_topic', '/amr1/scan')
        self.declare_parameter('output_topic', '/amr1/scan_filtered')
        self.declare_parameter('enabled', True)

        # Measured bed size: 2.00 m (X) x 1.30 m (Y), plus 0.03 m margin.
        self.declare_parameter('x_min', -1.03)
        self.declare_parameter('x_max', 1.03)
        self.declare_parameter('y_min', -0.68)
        self.declare_parameter('y_max', 0.68)

        # "nan" marks a self-hit as unknown. This avoids falsely clearing the
        # unseen area behind a bed rail. "inf" is available for comparison.
        self.declare_parameter('replacement_mode', 'nan')
        self.declare_parameter('log_interval_sec', 5.0)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.last_log_time = time.monotonic()

        x_min, x_max, y_min, y_max = self._get_bounds()
        self._validate_bounds(x_min, x_max, y_min, y_max)

        self.get_logger().info(
            'LaserScan self-filter ready: '
            f'{input_topic} -> {output_topic}, '
            f'x=[{x_min:.3f}, {x_max:.3f}], '
            f'y=[{y_min:.3f}, {y_max:.3f}]'
        )

    def _get_bounds(self) -> tuple[float, float, float, float]:
        return (
            float(self.get_parameter('x_min').value),
            float(self.get_parameter('x_max').value),
            float(self.get_parameter('y_min').value),
            float(self.get_parameter('y_max').value),
        )

    @staticmethod
    def _validate_bounds(
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
    ) -> None:
        if x_min >= x_max:
            raise ValueError('x_min must be smaller than x_max')
        if y_min >= y_max:
            raise ValueError('y_min must be smaller than y_max')

    def _replacement_value(self) -> float:
        mode = str(self.get_parameter('replacement_mode').value).lower()
        if mode == 'nan':
            return float('nan')
        if mode == 'inf':
            return float('inf')

        self.get_logger().warning(
            f'Unknown replacement_mode={mode!r}; using nan.'
        )
        return float('nan')

    def scan_callback(self, scan: LaserScan) -> None:
        """Publish a copy of *scan* with bed-intersection samples invalidated."""
        filtered_scan = copy.deepcopy(scan)

        if not bool(self.get_parameter('enabled').value):
            self.publisher.publish(filtered_scan)
            return

        x_min, x_max, y_min, y_max = self._get_bounds()
        try:
            self._validate_bounds(x_min, x_max, y_min, y_max)
        except ValueError as exc:
            self.get_logger().error(f'Invalid filter bounds: {exc}')
            self.publisher.publish(filtered_scan)
            return

        replacement = self._replacement_value()
        filtered_count = 0
        intensities_match_ranges = (
            len(filtered_scan.intensities) == len(filtered_scan.ranges)
        )

        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            if x_min <= x <= x_max and y_min <= y <= y_max:
                filtered_scan.ranges[index] = replacement
                if intensities_match_ranges:
                    filtered_scan.intensities[index] = 0.0
                filtered_count += 1

        self.publisher.publish(filtered_scan)

        now = time.monotonic()
        log_interval = max(
            0.1,
            float(self.get_parameter('log_interval_sec').value),
        )
        if now - self.last_log_time >= log_interval:
            self.get_logger().info(
                f'Filtered {filtered_count}/{len(scan.ranges)} samples '
                f'inside the bed footprint.'
            )
            self.last_log_time = now


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LaserScanSelfFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
