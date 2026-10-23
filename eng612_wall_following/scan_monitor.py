#!/usr/bin/env python3
"""Read-only physical TurtleBot3 laser monitor. No motion commands."""

import math
import time
from statistics import median

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def sector_text(scan, centre_deg, half_width_deg, nearest=False):
    """Return a range summary for a sector in the laser frame."""
    centre = math.radians(centre_deg)
    width = math.radians(half_width_deg)
    values = []
    ray_count = 0

    for index, distance in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        delta = math.atan2(
            math.sin(angle - centre),
            math.cos(angle - centre)
        )

        if abs(delta) <= width + 1e-6:
            ray_count += 1

            if (
                math.isfinite(distance)
                and scan.range_min <= distance <= scan.range_max
            ):
                values.append(float(distance))

    if ray_count == 0:
        return 'OUT_OF_SCAN'

    if not values:
        return f'UNKNOWN [0/{ray_count}]'

    value = min(values) if nearest else median(values)
    return f'{value:.3f} m [{len(values)}/{ray_count}]'


class ScanMonitor(Node):
    def __init__(self):
        super().__init__('scan_monitor')

        if self.get_parameter('use_sim_time').value:
            raise ValueError(
                'Use use_sim_time:=false for this physical test.'
            )

        self.scan = None
        self.received_at = None
        self.shown_metadata = False

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.on_scan,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(1.0, self.show_readings)

        self.get_logger().info(
            'LEFT-WALL SETUP: read-only monitor; '
            'no /cmd_vel publisher.'
        )

    def on_scan(self, message):
        self.scan = message
        self.received_at = time.monotonic()

    def show_readings(self):
        scan = self.scan

        if scan is None or self.received_at is None:
            self.get_logger().warning('Waiting for /scan ...')
            return

        if time.monotonic() - self.received_at > 1.0:
            self.get_logger().warning(
                'No scan received in the last second.'
            )
            return

        metadata = (
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
        )

        if (
            not scan.ranges
            or not all(math.isfinite(x) for x in metadata)
            or scan.angle_increment == 0.0
            or not 0.0 <= scan.range_min < scan.range_max
        ):
            self.get_logger().warning(
                'Empty scan or invalid scan metadata.'
            )
            return

        if not self.shown_metadata:
            self.get_logger().info(
                f'frame={scan.header.frame_id} | '
                f'rays={len(scan.ranges)} | '
                f'angle_step='
                f'{math.degrees(scan.angle_increment):.3f} deg'
            )
            self.shown_metadata = True

        front = sector_text(scan, 0.0, 20.0, nearest=True)
        front_left = sector_text(scan, 45.0, 3.0)
        left = sector_text(scan, 90.0, 3.0)
        right = sector_text(scan, -90.0, 3.0)

        self.get_logger().info(
            f'FRONT_MIN={front} | '
            f'FRONT_LEFT={front_left} | '
            f'LEFT={left} | '
            f'RIGHT={right}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ScanMonitor()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
