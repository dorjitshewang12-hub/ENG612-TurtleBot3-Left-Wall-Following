#!/usr/bin/env python3
"""Left-wall follower, extended approach: two PID controllers.

PID 1 (angular): error = target - left distance  -> angular.z
PID 2 (linear):  error = front - front_trigger   -> linear.x
Wall distance uses two beams (45 and 90 deg) to correct for heading.
A state machine handles corners (odometry-based right turn) and recovery.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class DualPidWallFollower(Node):
    def __init__(self):
        super().__init__('dual_pid_wall_follower')
        self.declare_parameter('enable_motion', True)
        self.declare_parameter('target_distance', 0.30)
        self.declare_parameter('linear_speed', 0.08)
        # PID 1: angular velocity
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('max_follow_angular', 0.35)
        # PID 2: linear velocity
        self.declare_parameter('kp_v', 0.16)
        self.declare_parameter('ki_v', 0.0)
        self.declare_parameter('kd_v', 0.0)
        self.declare_parameter('v_min', 0.03)

        self.enable_motion = bool(self.get_parameter('enable_motion').value)
        self.target = float(self.get_parameter('target_distance').value)
        self.linear_speed = clamp(float(self.get_parameter('linear_speed').value), 0.0, 0.08)
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.max_follow_angular = clamp(
            float(self.get_parameter('max_follow_angular').value), 0.05, 0.35)
        self.kp_v = float(self.get_parameter('kp_v').value)
        self.ki_v = float(self.get_parameter('ki_v').value)
        self.kd_v = float(self.get_parameter('kd_v').value)
        self.v_min = clamp(float(self.get_parameter('v_min').value), 0.0, self.linear_speed)

        self.near_left = self.target - 0.08
        self.near_clear = self.target - 0.045  # hysteresis to prevent state chatter
        self.far_left = self.target + 0.35
        self.far_clear = self.target + 0.25
        self.front_trigger = 0.35  # turn starts ~0.35 m out, so robot exits a corner near 0.30 m
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
        # PID 1 state
        self.last_error = None
        self.last_pid_time = None
        self.d_filtered = 0.0
        self.integral = 0.0
        self.prev_angular = 0.0
        # PID 2 state
        self.v_last_error = None
        self.v_last_time = None
        self.v_integral = 0.0
        self.v_d = 0.0
        self.last_log_time = 0.0

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Distance error (target - measured) for plotting with rqt_plot / ros2 bag.
        self.err_pub = self.create_publisher(Float32, '/wall_error', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info(
            f'DUAL PID WALL FOLLOWER | target={self.target:.2f} m | '
            f'v_max={self.linear_speed:.3f} v_min={self.v_min:.3f} | '
            f'ang Kp={self.kp:.2f} Ki={self.ki:.2f} Kd={self.kd:.2f} | '
            f'lin Kp={self.kp_v:.2f} Ki={self.ki_v:.2f} Kd={self.kd_v:.2f} | '
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
        self.v_last_error = None
        self.v_last_time = None
        self.v_integral = 0.0
        self.v_d = 0.0

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
        """PID 1: lateral distance error -> angular velocity."""
        # Small deadband avoids left/right command toggling caused by LiDAR noise.
        if abs(error) < 0.008:
            error = 0.0
        if self.last_error is not None and self.last_pid_time is not None:
            dt = now - self.last_pid_time
            if 0.01 <= dt <= 0.5:
                self.integral = clamp(self.integral + error * dt, -0.10, 0.10)
                raw_derivative = (error - self.last_error) / dt
                alpha = dt / (0.15 + dt)
                self.d_filtered += alpha * (raw_derivative - self.d_filtered)
            else:
                self.d_filtered = 0.0
        self.last_error = error
        self.last_pid_time = now
        # Too far from LEFT wall => positive yaw (turn left).
        # Too close => negative yaw (turn right).
        desired = -(self.kp*error + self.ki*self.integral + self.kd*self.d_filtered)
        desired = clamp(desired, -self.max_follow_angular, self.max_follow_angular)
        # Limit change in steering command per scan.
        angular = clamp(desired, self.prev_angular - 0.10, self.prev_angular + 0.10)
        self.prev_angular = angular
        return angular

    def speed_pid(self, front, now):
        """PID 2: free space ahead of the turn trigger -> linear velocity."""
        front = min(front, 2.0)  # 'OPEN' (inf) would give a NaN derivative
        error = front - self.front_trigger
        if self.v_last_error is not None and self.v_last_time is not None:
            dt = now - self.v_last_time
            if 0.01 <= dt <= 0.5:
                self.v_integral = clamp(self.v_integral + error * dt, -0.2, 0.2)
                self.v_d = (error - self.v_last_error) / dt
        self.v_last_error = error
        self.v_last_time = now
        v = self.kp_v * error + self.ki_v * self.v_integral + self.kd_v * self.v_d
        return clamp(v, self.v_min, self.linear_speed)

    def report(self, now, left, front, angular, speed):
        if now - self.last_log_time < 1.0:
            return
        self.last_log_time = now

        def fmt(x):
            return 'UNKNOWN' if x is None else ('OPEN' if math.isinf(x) else f'{x:.3f}')
        err = 'N/A' if left is None or math.isinf(left) else f'{self.target - left:+.3f}'
        self.get_logger().info(
            f'{self.state} | left={fmt(left)} m | error={err} m | front={fmt(front)} m | '
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

        # Two-beam wall estimate: corrects the side reading when the robot is angled.
        a = self.sector(scan, 45, 3)          # median of a narrow beam at 45 deg
        b = left
        if a is not None and math.isfinite(a):
            th = math.radians(45.0)
            alpha = math.atan2(a*math.cos(th) - b, a*math.sin(th))  # wall angle
            dist = b*math.cos(alpha) + 0.20*math.sin(alpha)         # perpendicular + 0.20 m lookahead
        else:
            dist = b

        # Light low-pass filter on the wall distance (heavier filtering caused lag).
        if self.left_filtered is None:
            self.left_filtered = dist
        else:
            self.left_filtered = 0.6*dist + 0.4*self.left_filtered
        error = self.target - self.left_filtered
        self.err_pub.publish(Float32(data=float(error)))

        angular = self.follow_pid(error, now)   # PID 1: steering
        speed = self.speed_pid(front, now)      # PID 2: speed from front clearance
        # Also slow down when the lateral error is large.
        speed *= max(0.5, 1.0 - abs(error) / 0.10)
        speed = max(speed, self.v_min)

        self.send(speed, angular)
        self.report(now, self.left_filtered, front, angular, speed)

    def shutdown_stop(self):
        for _ in range(5):
            self.pub.publish(Twist())


def main(args=None):
    # Disable rclpy's SIGINT handler so the context stays valid for the stop command.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DualPidWallFollower()
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
