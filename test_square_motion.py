#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_diff(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


class SquareMotionTest(Node):
    def __init__(self) -> None:
        super().__init__('square_motion_test')
        self._odom: Odometry | None = None
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 20)

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def wait_for_odom(self) -> None:
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and self._odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._odom is None:
            raise RuntimeError('No /odom messages received')

    def stop(self, seconds: float = 0.5) -> None:
        msg = Twist()
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self._pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive_forward(self, distance_m: float, speed_mps: float = 0.20) -> None:
        assert self._odom is not None
        start = self._odom.pose.pose.position
        start_x = float(start.x)
        start_y = float(start.y)
        msg = Twist()
        msg.linear.x = abs(speed_mps)
        deadline = time.monotonic() + max(8.0, distance_m / speed_mps + 5.0)

        while rclpy.ok() and time.monotonic() < deadline:
            assert self._odom is not None
            pos = self._odom.pose.pose.position
            travelled = math.hypot(float(pos.x) - start_x, float(pos.y) - start_y)
            if travelled >= distance_m:
                self.stop()
                print(f'forward ok: {travelled:.2f} m')
                return
            self._pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop()
        raise RuntimeError('Forward motion timed out; robot did not travel far enough')

    def turn_left(self, radians: float = math.pi / 2.0, speed_radps: float = 0.45) -> None:
        assert self._odom is not None
        start_yaw = yaw_from_odom(self._odom)
        target_yaw = start_yaw + radians
        msg = Twist()
        msg.angular.z = abs(speed_radps)
        deadline = time.monotonic() + max(8.0, radians / speed_radps + 5.0)

        while rclpy.ok() and time.monotonic() < deadline:
            assert self._odom is not None
            current_yaw = yaw_from_odom(self._odom)
            remaining = angle_diff(target_yaw, current_yaw)
            if abs(remaining) <= math.radians(4.0):
                self.stop()
                print(f'turn ok: yaw error {math.degrees(remaining):.1f} deg')
                return
            self._pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop()
        raise RuntimeError('Turn motion timed out; robot did not rotate far enough')


def main() -> None:
    rclpy.init()
    node = SquareMotionTest()
    try:
        node.wait_for_odom()
        for side in range(4):
            print(f'side {side + 1}/4: forward 1.0 m')
            node.drive_forward(1.0)
            print(f'side {side + 1}/4: turn left 90 deg')
            node.turn_left()
        node.stop()
        print('square motion test completed')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
