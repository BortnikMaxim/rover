#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.br = TransformBroadcaster(self)
        self._last_stamp_ns = None
        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg: Odometry):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
            self.get_logger().warn(
                f'ignoring non-monotonic odom timestamp {stamp_ns} <= {self._last_stamp_ns}',
                throttle_duration_sec=2.0,
            )
            return
        self._last_stamp_ns = stamp_ns

        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id or 'odom'
        t.child_frame_id = msg.child_frame_id or 'base_footprint'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
