#!/usr/bin/env python3
"""
Global costmap with inflation gradient.

Loads a static occupancy map (.pgm + .yaml in map_server format) and produces
a Nav2-style costmap where:
  - lethal obstacles (original walls / shelves) = 100
  - cells inside `robot_radius` of an obstacle      = 99 (inscribed)
  - cells inside `inflation_radius` of an obstacle  = exponentially decaying
  - free cells                                       = 0

Publish:
  /global_costmap     nav_msgs/OccupancyGrid    latched, single message at startup
"""
from __future__ import annotations

import math
import os

import numpy as np
import yaml
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid


class GlobalCostmapNode(Node):
    def __init__(self) -> None:
        super().__init__('global_costmap_node')

        self.declare_parameter('map_pgm_path', '')
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_radius_m', 0.30)
        self.declare_parameter('inflation_radius_m', 0.80)
        self.declare_parameter('cost_decay_rate', 5.0)
        self.declare_parameter('occupancy_threshold', 200)

        map_pgm = str(self.get_parameter('map_pgm_path').value)
        map_yaml = str(self.get_parameter('map_yaml_path').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.robot_radius = float(self.get_parameter('robot_radius_m').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius_m').value)
        self.decay_rate = float(self.get_parameter('cost_decay_rate').value)
        self.occ_thresh = int(self.get_parameter('occupancy_threshold').value)

        if not map_pgm or not map_yaml:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('kolestel_rover_description')
            map_pgm = map_pgm or os.path.join(pkg, 'maps', 'warehouse_nav2_map.pgm')
            map_yaml = map_yaml or os.path.join(pkg, 'maps', 'warehouse_nav2_map.yaml')

        meta = yaml.safe_load(open(map_yaml, encoding='utf-8'))
        self.res = float(meta['resolution'])
        origin = meta.get('origin', [0.0, 0.0, 0.0])
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])

        img = np.flipud(np.array(Image.open(map_pgm))).copy()
        h, w = img.shape
        occupied = (img < self.occ_thresh).astype(np.uint8)

        # distance from each free cell to nearest obstacle (in metres)
        from scipy.ndimage import distance_transform_edt
        dist_cells = distance_transform_edt(occupied == 0)
        dist_m = dist_cells * self.res

        cost = np.zeros_like(dist_m, dtype=np.int16)
        cost[occupied == 1] = 100                                   # lethal
        mask_inscribed = (occupied == 0) & (dist_m <= self.robot_radius)
        cost[mask_inscribed] = 99                                   # inscribed
        mask_inflate = (
            (occupied == 0)
            & (dist_m > self.robot_radius)
            & (dist_m <= self.inflation_radius)
        )
        # exponential decay from 99 -> 0 over (inflation_radius - robot_radius)
        d_inflate = dist_m[mask_inflate] - self.robot_radius
        v = 98 * np.exp(-self.decay_rate * d_inflate / max(self.inflation_radius - self.robot_radius, 1e-3))
        cost[mask_inflate] = v.astype(np.int16)
        cost = np.clip(cost, 0, 100).astype(np.int8)

        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.res
        msg.info.width = w
        msg.info.height = h
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = cost.flatten().tolist()

        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(OccupancyGrid, '/global_costmap', latched)
        self.pub.publish(msg)

        self.get_logger().info(
            f'global_costmap_node ready: {w}x{h} cells res={self.res}m '
            f'robot_r={self.robot_radius}m inflation_r={self.inflation_radius}m '
            f'lethal={int(occupied.sum())} inscribed={int(mask_inscribed.sum())} '
            f'inflated={int(mask_inflate.sum())}'
        )

        # also republish periodically — protects against subscribers that miss the
        # TRANSIENT_LOCAL latched message
        self._msg = msg
        self.create_timer(2.0, self._republish)

    def _republish(self) -> None:
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self._msg)


def main() -> None:
    rclpy.init()
    node = GlobalCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
