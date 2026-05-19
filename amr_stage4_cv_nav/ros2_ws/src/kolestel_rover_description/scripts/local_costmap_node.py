#!/usr/bin/env python3
"""
Local costmap: rolling window in `map` frame, refreshed from lidar + depth camera.

Builds a Nav2-style costmap inside a sliding window centered on the robot.
Sensor returns are projected to the map plane, inflated by robot_radius and a
soft inflation gradient, and republished at a configurable rate.

Publishes:
  /local_costmap   nav_msgs/OccupancyGrid    rolling 6x6m window @ ~5 Hz

The same encoding as the global costmap is used: 100 = lethal, 99 = inscribed,
0..98 = inflation gradient, 0 = free.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformException, TransformListener


def _quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


class LocalCostmapNode(Node):
    def __init__(self) -> None:
        super().__init__('local_costmap_node')
        self.declare_parameter('window_size_m', 6.0)
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('robot_radius_m', 0.30)
        self.declare_parameter('inflation_radius_m', 0.60)
        self.declare_parameter('decay_rate', 5.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('lidar_topic', '/lidar_map')
        self.declare_parameter('depth_topic', '/camera/depth/points')
        self.declare_parameter('min_obstacle_height_m', 0.05)
        self.declare_parameter('max_obstacle_height_m', 2.0)
        self.declare_parameter('camera_base_frame', 'base_footprint')

        self.window = float(self.get_parameter('window_size_m').value)
        self.res = float(self.get_parameter('resolution').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.robot_radius = float(self.get_parameter('robot_radius_m').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius_m').value)
        self.decay = float(self.get_parameter('decay_rate').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        lidar_topic = str(self.get_parameter('lidar_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        self.z_min = float(self.get_parameter('min_obstacle_height_m').value)
        self.z_max = float(self.get_parameter('max_obstacle_height_m').value)
        self.cam_base = str(self.get_parameter('camera_base_frame').value)

        self.cells = int(round(self.window / self.res))
        self._lidar_msg: Optional[PointCloud2] = None
        self._depth_msg: Optional[PointCloud2] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(PointCloud2, lidar_topic, self._lidar_cb, 5)
        self.create_subscription(PointCloud2, depth_topic, self._depth_cb, 5)
        self.pub = self.create_publisher(OccupancyGrid, '/local_costmap', 5)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

        self.get_logger().info(
            f'local_costmap_node ready: window={self.window}m res={self.res}m '
            f'({self.cells}x{self.cells} cells) rate={rate}Hz '
            f'lidar={lidar_topic} depth={depth_topic} '
            f'z=[{self.z_min},{self.z_max}]m'
        )

    def _lidar_cb(self, msg: PointCloud2) -> None:
        self._lidar_msg = msg

    def _depth_cb(self, msg: PointCloud2) -> None:
        self._depth_msg = msg

    def _robot_world(self) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None

    def _tick(self) -> None:
        ts = self._robot_world()
        if ts is None:
            return
        rx = float(ts.transform.translation.x)
        ry = float(ts.transform.translation.y)

        # rolling window centered on robot, snapped to grid
        ox = rx - self.window / 2.0
        oy = ry - self.window / 2.0
        # snap origin to res-aligned grid so the local cells align with global
        ox = math.floor(ox / self.res) * self.res
        oy = math.floor(oy / self.res) * self.res

        H = W = self.cells
        occupied = np.zeros((H, W), dtype=np.uint8)

        # ---- lidar layer (cloud is already in map frame at /lidar_map) ----
        if self._lidar_msg is not None and self._lidar_msg.header.frame_id == self.map_frame:
            pts = pc2.read_points_numpy(self._lidar_msg, field_names=('x', 'y', 'z'), skip_nans=True)
            if pts.size:
                pts = pts.astype(float).reshape(-1, 3)
                pts = pts[np.isfinite(pts).all(axis=1)]
                # filter by height (z relative to lidar mount; lidar is at world z ~ 0.76)
                # accept points clearly off the floor
                mask_h = (pts[:, 2] > -0.5) & (pts[:, 2] < self.z_max + 1.0)
                pts = pts[mask_h]
                if pts.size:
                    cols = np.floor((pts[:, 0] - ox) / self.res).astype(int)
                    rows = np.floor((pts[:, 1] - oy) / self.res).astype(int)
                    keep = (cols >= 0) & (cols < W) & (rows >= 0) & (rows < H)
                    occupied[rows[keep], cols[keep]] = 1

        # ---- depth camera layer (cloud is in camera optical frame raw, or base if republished) ----
        if self._depth_msg is not None:
            # transform points into map frame on the fly
            try:
                tcam = self._tf_buffer.lookup_transform(
                    self.map_frame, self._depth_msg.header.frame_id, Time(),
                    timeout=Duration(seconds=0.05),
                )
                q = tcam.transform.rotation
                from math import cos, sin
                Rcam = self._quat_mat(q.x, q.y, q.z, q.w)
                tvec = np.array([tcam.transform.translation.x,
                                 tcam.transform.translation.y,
                                 tcam.transform.translation.z], dtype=np.float32)
                pts = pc2.read_points_numpy(self._depth_msg, field_names=('x', 'y', 'z'), skip_nans=True)
                if pts.size:
                    pts = pts.astype(np.float32).reshape(-1, 3)
                    pts = pts[np.isfinite(pts).all(axis=1)]
                    if pts.size:
                        # downsample heavy depth cloud
                        if pts.shape[0] > 10000:
                            pts = pts[::5]
                        world = pts @ Rcam.T + tvec
                        # filter by world height
                        mask_h = (world[:, 2] > self.z_min) & (world[:, 2] < self.z_max)
                        world = world[mask_h]
                        if world.size:
                            cols = np.floor((world[:, 0] - ox) / self.res).astype(int)
                            rows = np.floor((world[:, 1] - oy) / self.res).astype(int)
                            keep = (cols >= 0) & (cols < W) & (rows >= 0) & (rows < H)
                            occupied[rows[keep], cols[keep]] = 1
            except TransformException:
                pass

        # ---- inflate ----
        from scipy.ndimage import distance_transform_edt
        dist_cells = distance_transform_edt(occupied == 0)
        dist_m = dist_cells * self.res
        cost = np.zeros_like(dist_m, dtype=np.int16)
        cost[occupied == 1] = 100
        mask_i = (occupied == 0) & (dist_m <= self.robot_radius)
        cost[mask_i] = 99
        mask_f = (occupied == 0) & (dist_m > self.robot_radius) & (dist_m <= self.inflation_radius)
        d2 = dist_m[mask_f] - self.robot_radius
        v = 98 * np.exp(-self.decay * d2 / max(self.inflation_radius - self.robot_radius, 1e-3))
        cost[mask_f] = v.astype(np.int16)
        cost = np.clip(cost, 0, 100).astype(np.int8)

        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.res
        msg.info.width = W
        msg.info.height = H
        msg.info.origin.position.x = ox
        msg.info.origin.position.y = oy
        msg.info.origin.orientation.w = 1.0
        msg.data = cost.flatten().tolist()
        self.pub.publish(msg)

    @staticmethod
    def _quat_mat(qx, qy, qz, qw):
        n = qx * qx + qy * qy + qz * qz + qw * qw
        if n < 1e-12:
            return np.eye(3, dtype=np.float32)
        s = 2.0 / n
        return np.array([
            [1 - s * (qy * qy + qz * qz),  s * (qx * qy - qz * qw),      s * (qx * qz + qy * qw)],
            [s * (qx * qy + qz * qw),      1 - s * (qx * qx + qz * qz),  s * (qy * qz - qx * qw)],
            [s * (qx * qz - qy * qw),      s * (qy * qz + qx * qw),      1 - s * (qx * qx + qy * qy)],
        ], dtype=np.float32)


def main() -> None:
    rclpy.init()
    node = LocalCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
