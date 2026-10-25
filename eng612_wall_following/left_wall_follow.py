#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


def clamp(x, lower, upper):
    return max(lower, min(upper, x))


def wrap_angle(x):
    return math.atan2(math.sin(x), math.cos(x))


class LeftWallFollower(Node):
    def __init__(self):
        super().__init__('left_wall_follower')

        self.declare_parameter('enable_motion', True)
        self.declare_parameter('target_distance', 0.30)
        self.declare_parameter('linear_speed', 0.08)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.20)

        self.enable_motion = bool(self.get_parameter('enable_motion').value)
        self.target = float(self.get_parameter('target_distance').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)

        self.near_left = self.target - 0.05
        self.far_left = self.target + 0.15
        self.front_trigger = 0.35
        self.front_clear = 0.48
        self.front_left_trigger = 0.23
        self.right_min_clearance = 0.17
        self.max_angular = 0.65
        self.recovery_angular = 0.25
        self.recovery_linear = 0.03
        self.recovery_timeout = 5.0
        self.turn_timeout = 8.0
        self.sensor_timeout = 1.0

        self.state = 'FOLLOW'
        self.state_since = time.monotonic()
        self.yaw = None
        self.last_odom = None
        self.last_scan = None
        self.turn_target = None
        self.integral = 0.0
        self.last_error = None
        self.last_pid_time = None
        self.derivative_filtered = 0.0
        self.last_log_time = 0.0
        self.last_reason = ''

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            f'LEFT WALL FOLLOWER | target={self.target:.2f} m | '
            f'enable_motion={self.enable_motion}')
        if not self.enable_motion:
            self.get_logger().warn('DRY RUN: reporting commands; publishing zero velocity.')

    def send(self, linear=0.0, angular=0.0):
        cmd = Twist()
        if self.enable_motion and self.state != 'STOPPED':
            cmd.linear.x = float(clamp(linear, 0.0, self.linear_speed))
            cmd.angular.z = float(clamp(angular, -self.max_angular, self.max_angular))
        self.pub.publish(cmd)

    def reset_pid(self):
        self.integral = 0.0
        self.last_error = None
        self.last_pid_time = None
        self.derivative_filtered = 0.0

    def set_state(self, state, reason=''):
        if state != self.state:
            self.state = state
            self.state_since = time.monotonic()
            self.reset_pid()
            self.get_logger().info(f'STATE -> {state}: {reason}')

    def stop_latched(self, reason):
        if self.state != 'STOPPED':
            self.set_state('STOPPED', reason)
            self.get_logger().error(
                'Controller stopped; inspect surroundings and restart node to recover.')
        self.send()

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.last_odom = time.monotonic()

    def ready(self, now):
        return (self.last_scan is not None
                and self.last_odom is not None
                and self.yaw is not None
                and now - self.last_scan <= self.sensor_timeout
                and now - self.last_odom <= self.sensor_timeout)

    def watchdog(self):
        if not self.ready(time.monotonic()):
            self.send()  # Stop if scan or odometry stops arriving.

    @staticmethod
    def sector(scan, center_deg, half_width_deg, use_min=False):
        """Return distance, +inf for valid all-infinite (open), None for no usable beams."""
        values = []
        open_beams = 0
        beams_in_sector = 0
        if scan.angle_increment == 0:
            return None
        for index, reading in enumerate(scan.ranges):
            angle_deg = math.degrees(scan.angle_min + index * scan.angle_increment)
            difference = (angle_deg - center_deg + 180.0) % 360.0 - 180.0
            if abs(difference) > half_width_deg:
                continue
            beams_in_sector += 1
            if math.isinf(reading) and reading > 0:
                open_beams += 1
            elif math.isfinite(reading) and scan.range_min <= reading <= scan.range_max:
                values.append(reading)
        if values:
            return min(values) if use_min else sum(values) / len(values)
        if beams_in_sector > 0 and open_beams > 0:
            return math.inf
        return None

    def pid(self, error, now):
        if self.last_error is not None and self.last_pid_time is not None:
            dt = now - self.last_pid_time
            if 0.01 <= dt <= 0.5:
                self.integral = clamp(
                    self.integral + (error + self.last_error) * 0.5 * dt,
                    -0.30, 0.30)
                derivative = (error - self.last_error) / dt
                alpha = dt / (0.25 + dt)
                self.derivative_filtered += alpha * (derivative - self.derivative_filtered)
            else:
                self.derivative_filtered = 0.0
        self.last_error = error
        self.last_pid_time = now
        # Left wall: positive error means too close => steer RIGHT (negative z).
        return clamp(-(
            self.kp * error + self.ki * self.integral
            + self.kd * self.derivative_filtered),
            -self.max_angular, self.max_angular)

    def report(self, now, left, front, front_left, linear, angular):
        if now - self.last_log_time < 1.0:
            return
        self.last_log_time = now
        def fmt(x):
            return 'UNKNOWN' if x is None else ('OPEN' if math.isinf(x) else f'{x:.3f}')
        self.get_logger().info(
            f'{self.state} | left={fmt(left)} m | front={fmt(front)} m | '
            f'front_left={fmt(front_left)} m | cmd_v={linear:.3f} | '
            f'cmd_w={angular:+.3f} | motion={self.enable_motion}')

    def scan_callback(self, scan):
        now = time.monotonic()
        self.last_scan = now
        if self.state == 'STOPPED':
            self.send()
            return
        if not self.ready(now):
            self.send()
            return

        front = self.sector(scan, 0, 12, use_min=True)
        front_left = self.sector(scan, 45, 10, use_min=True)
        left = self.sector(scan, 90, 5)
        right = self.sector(scan, -90, 8, use_min=True)

        if front is None or front_left is None:
            self.send()  # Unknown forward safety distance: do not move.
            self.report(now, left, front, front_left, 0.0, 0.0)
            return

        if self.state == 'TURN_RIGHT':
            if now - self.state_since > self.turn_timeout:
                self.stop_latched('right turn timed out')
                return
            if right is not None and right < self.right_min_clearance:
                self.stop_latched('insufficient right-side clearance to rotate')
                return
            error = wrap_angle(self.turn_target - self.yaw)
            if abs(error) <= math.radians(3.0):
                self.turn_target = None
                self.set_state('FOLLOW', 'right turn completed')
                self.send()
                return
            angular = clamp(1.5 * error, -0.40, 0.40)
            # Ensure last few degrees are not lost due to command deadband.
            if 0 < abs(angular) < 0.12:
                angular = math.copysign(0.12, angular)
            self.send(angular=angular)
            self.report(now, left, front, front_left, 0.0, angular)
            return

        # Facing a barrier: make a controlled in-place right turn.
        if front <= self.front_trigger or front_left <= self.front_left_trigger:
            if right is not None and right < self.right_min_clearance:
                self.stop_latched('front blocked and right side too close')
                return
            self.turn_target = wrap_angle(self.yaw - math.pi / 2.0)
            self.set_state('TURN_RIGHT', 'front or front-left obstacle')
            self.send()
            return

        # Prevent forward motion when nearing a front barrier.
        if front < self.front_clear and self.state == 'RECOVER_LEFT':
            self.send()
            return

        if self.state == 'RECOVER_LEFT':
            if now - self.state_since > self.recovery_timeout:
                self.stop_latched('left wall not rediscovered within recovery timeout')
                return
            if left is not None and math.isfinite(left) and left <= self.far_left:
                self.set_state('FOLLOW', 'left wall reacquired')
            else:
                v = self.recovery_linear
                w = self.recovery_angular
                self.send(v, w)
                self.report(now, left, front, front_left, v, w)
                return

        if self.state == 'AVOID_LEFT':
            if left is not None and math.isfinite(left) and left >= self.near_left:
                self.set_state('FOLLOW', 'left clearance restored')
            else:
                if right is not None and right < self.right_min_clearance:
                    self.stop_latched('left and right side clearance insufficient')
                    return
                v = self.recovery_linear
                w = -self.recovery_angular
                self.send(v, w)
                self.report(now, left, front, front_left, v, w)
                return

        if left is None or math.isinf(left) or left > self.far_left:
            self.set_state('RECOVER_LEFT', 'left wall out of range')
            self.send()
            return
        if left < self.near_left:
            self.set_state('AVOID_LEFT', 'too close to left wall')
            self.send()
            return

        error = self.target - left
        angular = self.pid(error, now)
        # Slow down for a remaining obstacle/large heading correction.
        v = self.linear_speed
        if front < self.front_clear or abs(angular) > 0.40:
            v = min(v, 0.04)
        self.send(v, angular)
        self.report(now, left, front, front_left, v, angular)

    def shutdown_stop(self):
        for _ in range(3):
            self.pub.publish(Twist())


def main(args=None):
    # Disable rclpy's SIGINT handler so the context stays valid for the stop command.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
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
