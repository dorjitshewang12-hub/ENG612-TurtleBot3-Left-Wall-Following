#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LeftWallFollower(Node):
    def __init__(self):
        super().__init__('left_wall_follower')
        self.declare_parameter('enable_motion', True)
        self.declare_parameter('target_distance', 0.30)
        self.declare_parameter('linear_speed', 0.09)
        self.declare_parameter('kp', 0.60)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('max_follow_angular', 0.22)

        self.enable_motion = bool(self.get_parameter('enable_motion').value)
        self.target = float(self.get_parameter('target_distance').value)
        self.linear_speed = clamp(float(self.get_parameter('linear_speed').value), 0.0, 0.08)
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.max_follow_angular = clamp(float(self.get_parameter('max_follow_angular').value), 0.05, 0.35)

        self.near_left = self.target - 0.08
        self.near_clear = self.target - 0.045  # hysteresis to prevent state chatter
        self.far_left = self.target + 0.35
        self.far_clear = self.target + 0.25
        self.front_trigger = 0.4
        self.front_left_trigger = 0.3
        self.front_slow = 0.5
        self.right_min_clearance = 0.17
        self.recovery_timeout = 4.0
        self.avoid_timeout = 4.0
        self.turn_timeout = 8.0
        self.sensor_timeout = 0.7

        self.yaw = None
        self.last_odom = None
        self.last_scan = None
        self.state = 'FOLLOW'
        self.state_since = time.monotonic()
        self.turn_target = None
        self.left_filtered = None
        self.last_error = None
        self.last_pid_time = None
        self.d_filtered = 0.0
        self.integral = 0.0
        self.prev_angular = 0.0
        self.last_log_time = 0.0

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info(
            f'LEFT WALL FOLLOWER SMOOTH | target={self.target:.2f} m | '
            f'v={self.linear_speed:.3f} | Kp={self.kp:.2f} Ki={self.ki:.2f} Kd={self.kd:.2f} | '
            f'enable_motion={self.enable_motion}')
        if not self.enable_motion:
            self.get_logger().warn('DRY RUN: commands reported, zero velocity published')

    def send(self, linear=0.0, angular=0.0):
        msg = Twist()
        if self.enable_motion and self.state != 'STOPPED':
            msg.linear.x = float(clamp(linear, 0.0, 0.08))
            msg.angular.z = float(clamp(angular, -0.40, 0.40))
        self.pub.publish(msg)

    def reset_pid(self):
        self.last_error = None
        self.last_pid_time = None
        self.integral = 0.0
        self.d_filtered = 0.0
        self.prev_angular = 0.0

    def set_state(self, new_state, reason=''):
        if new_state != self.state:
            self.state = new_state
            self.state_since = time.monotonic()
            self.reset_pid()
            self.get_logger().info(f'STATE -> {new_state}: {reason}')

    def latch_stop(self, reason):
        if self.state != 'STOPPED':
            self.set_state('STOPPED', reason)
            self.get_logger().error('STOPPED: inspect scene and restart controller to reset')
        self.send()

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                              1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.last_odom = time.monotonic()

    def sensors_ready(self, now):
        return (self.last_scan is not None and self.last_odom is not None
                and self.yaw is not None
                and now - self.last_scan <= self.sensor_timeout
                and now - self.last_odom <= self.sensor_timeout)

    def watchdog(self):
        if not self.sensors_ready(time.monotonic()):
            self.send()

    @staticmethod
    def sector(scan, center_deg, width_deg, nearest=False):
        if scan.angle_increment == 0:
            return None
        values = []
        opens = 0
        for index, reading in enumerate(scan.ranges):
            theta = math.degrees(scan.angle_min + index * scan.angle_increment)
            diff = (theta - center_deg + 180.0) % 360.0 - 180.0
            if abs(diff) > width_deg:
                continue
            if math.isfinite(reading) and scan.range_min <= reading <= scan.range_max:
                values.append(reading)
            elif math.isinf(reading) and reading > 0:
                opens += 1
        if values:
            values.sort()
            if nearest:
                return values[0]
            # Median rejects spurious reflections better than the mean.
            return values[len(values)//2]
        return math.inf if opens else None

    def follow_pid(self, error, now):
        # Small deadband avoids left/right command toggling caused by LiDAR noise.
        if abs(error) < 0.012:
            error = 0.0
        if self.last_error is not None and self.last_pid_time is not None:
            dt = now - self.last_pid_time
            if 0.01 <= dt <= 0.5:
                self.integral = clamp(self.integral + error * dt, -0.10, 0.10)
                raw_derivative = (error - self.last_error) / dt
                alpha = dt / (0.40 + dt)
                self.d_filtered += alpha * (raw_derivative - self.d_filtered)
            else:
                self.d_filtered = 0.0
        self.last_error = error
        self.last_pid_time = now
        # Too far from LEFT wall => positive yaw (turn left).
        # Too close => negative yaw (turn right).
        desired = -(self.kp*error + self.ki*self.integral + self.kd*self.d_filtered)
        desired = clamp(desired, -self.max_follow_angular, self.max_follow_angular)
        # Cap change in steering command per scan to prevent zig-zag corrections.
        angular = clamp(desired, self.prev_angular - 0.025, self.prev_angular + 0.025)
        self.prev_angular = angular
        return angular

    def report(self, now, left, front, angular, speed):
        if now - self.last_log_time < 1.0:
            return
        self.last_log_time = now
        def fmt(x):
            return 'UNKNOWN' if x is None else ('OPEN' if math.isinf(x) else f'{x:.3f}')
        self.get_logger().info(
            f'{self.state} | left={fmt(left)} m | front={fmt(front)} m | '
            f'cmd_v={speed:.3f} | cmd_w={angular:+.3f} | motion={self.enable_motion}')

    def scan_callback(self, scan):
        now = time.monotonic()
        self.last_scan = now
        if self.state == 'STOPPED' or not self.sensors_ready(now):
            self.send()
            return

        front = self.sector(scan, 0, 12, nearest=True)
        front_left = self.sector(scan, 45, 10, nearest=True)
        left = self.sector(scan, 90, 5)
        right = self.sector(scan, -90, 8, nearest=True)

        if front is None or front_left is None:
            self.send()
            self.report(now, left, front, 0.0, 0.0)
            return

        if self.state == 'TURN_RIGHT':
            if now - self.state_since > self.turn_timeout:
                self.latch_stop('turn timeout')
                return
            if right is None or right < self.right_min_clearance:
                self.latch_stop('unknown or insufficient right-side clearance')
                return
            err = wrap_angle(self.turn_target - self.yaw)
            if abs(err) <= math.radians(3.0):
                self.turn_target = None
                self.left_filtered = None
                self.set_state('FOLLOW', 'turn completed')
                self.send()
                return
            w = clamp(1.2*err, -0.30, 0.30)
            if 0 < abs(w) < 0.10:
                w = math.copysign(0.10, w)
            self.send(0.0, w)
            self.report(now, left, front, w, 0.0)
            return

        if front <= self.front_trigger or front_left <= self.front_left_trigger:
            if right is None or right < self.right_min_clearance:
                self.latch_stop('front obstructed and right turn is not verified clear')
                return
            self.turn_target = wrap_angle(self.yaw - math.pi/2.0)
            self.set_state('TURN_RIGHT', 'front obstacle')
            self.send()
            return

        if self.state == 'RECOVER_LEFT':
            if now - self.state_since > self.recovery_timeout:
                self.latch_stop('left wall recovery timeout')
                return
            if left is not None and math.isfinite(left) and left < self.far_clear:
                self.left_filtered = left
                self.set_state('FOLLOW', 'wall reacquired')
            else:
                if front < self.front_slow:
                    self.send()
                    return
                self.send(0.025, 0.12)
                self.report(now, left, front, 0.12, 0.025)
                return

        if self.state == 'AVOID_LEFT':
            if now - self.state_since > self.avoid_timeout:
                self.latch_stop('left clearance recovery timeout')
                return
            if left is not None and math.isfinite(left) and left > self.near_clear:
                self.left_filtered = left
                self.set_state('FOLLOW', 'left clearance restored')
            else:
                if right is None or right < self.right_min_clearance:
                    self.latch_stop('insufficient clearance to avoid left wall')
                    return
                self.send(0.02, -0.12)
                self.report(now, left, front, -0.12, 0.02)
                return

        if left is None or math.isinf(left) or left > self.far_left:
            self.left_filtered = None
            self.set_state('RECOVER_LEFT', 'wall out of range')
            self.send()
            return
        if left < self.near_left:
            self.left_filtered = left
            self.set_state('AVOID_LEFT', 'too close to left wall')
            self.send()
            return

        # Low-pass filter LEFT distance; do not filter front collision detection.
        if self.left_filtered is None:
            self.left_filtered = left
        else:
            self.left_filtered = 0.25*left + 0.75*self.left_filtered
        error = self.target - self.left_filtered
        angular = self.follow_pid(error, now)
        speed = self.linear_speed
        if front < self.front_slow:
            speed = min(speed, 0.05)
        self.send(speed, angular)
        self.report(now, self.left_filtered, front, angular, speed)

    def shutdown_stop(self):
        for _ in range(5):
            try:
                self.pub.publish(Twist())
            except Exception:
                break


def main(args=None):
    rclpy.init(args=args)
    node = LeftWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

