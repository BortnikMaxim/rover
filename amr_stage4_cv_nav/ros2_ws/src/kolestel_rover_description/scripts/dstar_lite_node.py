#!/usr/bin/env python3
"""
D* Lite incremental replanner (Koenig & Likhachev, 2005).

A graph-search planner that re-uses previous search effort across calls.
When the cost of a few edges changes (e.g., a new obstacle appears in the
local costmap), D* Lite updates only the affected vertices instead of doing
a full A* re-plan. This is the canonical algorithm for dynamic environments.

Architecture in our node:
  - load the static (global) costmap once
  - listen to /local_costmap and overlay its lethal/inscribed cells
  - on /goal_pose: initialise; on subsequent local-cost changes: incremental update
  - publish the same /grid_planner/path topic the pure-pursuit follower expects

This sits alongside the other algorithms — pick one stack per session via launch
arg (algorithm=d_star_lite uses this node instead of grid_planner_node).
"""
from __future__ import annotations

import heapq
import json
import math
import os
import time
from collections import defaultdict
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
import yaml
from PIL import Image

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


SQRT2 = math.sqrt(2.0)
INF = math.inf
NEIGH = [
    (-1, -1, SQRT2), (-1, 0, 1.0), (-1, 1, SQRT2),
    (0, -1, 1.0),                  (0, 1, 1.0),
    (1, -1, SQRT2),  (1, 0, 1.0),  (1, 1, SQRT2),
]


def octile(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    dy = abs(a[0] - b[0])
    dx = abs(a[1] - b[1])
    return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)


class DStarLite:
    """Pure D* Lite — operates on a binary occupancy grid (1=blocked, 0=free)."""

    def __init__(self, grid: np.ndarray):
        self.grid = grid
        self.h, self.w = grid.shape
        self.start: Optional[Tuple[int, int]] = None
        self.last_start: Optional[Tuple[int, int]] = None
        self.goal: Optional[Tuple[int, int]] = None
        self.k_m: float = 0.0
        self.U: List[Tuple[Tuple[float, float], Tuple[int, int]]] = []
        self.g: Dict[Tuple[int, int], float] = defaultdict(lambda: INF)
        self.rhs: Dict[Tuple[int, int], float] = defaultdict(lambda: INF)
        # set of (s, key) currently logically in U — used to skip stale heap entries
        self.in_queue: Set[Tuple[Tuple[int, int], Tuple[float, float]]] = set()

    # ---------- bookkeeping ----------

    def in_bounds(self, s: Tuple[int, int]) -> bool:
        return 0 <= s[0] < self.h and 0 <= s[1] < self.w

    def is_blocked(self, s: Tuple[int, int]) -> bool:
        return (not self.in_bounds(s)) or self.grid[s] != 0

    def cost(self, u: Tuple[int, int], v: Tuple[int, int]) -> float:
        if self.is_blocked(u) or self.is_blocked(v):
            return INF
        dr = abs(u[0] - v[0])
        dc = abs(u[1] - v[1])
        return SQRT2 if (dr == 1 and dc == 1) else (1.0 if (dr + dc == 1) else INF)

    def neighbors(self, s: Tuple[int, int]):
        for dr, dc, _ in NEIGH:
            n = (s[0] + dr, s[1] + dc)
            if self.in_bounds(n):
                yield n

    def calculate_key(self, s: Tuple[int, int]) -> Tuple[float, float]:
        m = min(self.g[s], self.rhs[s])
        if m == INF:
            return (INF, INF)
        return (m + octile(self.start, s) + self.k_m, m)

    def _push(self, s: Tuple[int, int]) -> None:
        key = self.calculate_key(s)
        heapq.heappush(self.U, (key, s))
        self.in_queue.add((s, key))

    def _top(self) -> Optional[Tuple[Tuple[float, float], Tuple[int, int]]]:
        while self.U:
            key, s = self.U[0]
            if (s, key) in self.in_queue:
                return key, s
            heapq.heappop(self.U)
        return None

    def _pop(self) -> Tuple[Tuple[float, float], Tuple[int, int]]:
        while self.U:
            key, s = heapq.heappop(self.U)
            if (s, key) in self.in_queue:
                self.in_queue.discard((s, key))
                return key, s
        return ((INF, INF), (-1, -1))

    # ---------- core D* Lite ----------

    def initialize(self, start: Tuple[int, int], goal: Tuple[int, int]) -> None:
        self.start = start
        self.last_start = start
        self.goal = goal
        self.k_m = 0.0
        self.U.clear()
        self.in_queue.clear()
        self.g.clear()
        self.rhs.clear()
        self.rhs[goal] = 0.0
        self._push(goal)

    def update_vertex(self, u: Tuple[int, int]) -> None:
        if u != self.goal:
            best = INF
            for s in self.neighbors(u):
                c = self.cost(u, s) + self.g[s]
                if c < best:
                    best = c
            self.rhs[u] = best
        # remove any existing queue entry for u; re-insert if g != rhs
        self.in_queue = {(s, k) for (s, k) in self.in_queue if s != u}
        if self.g[u] != self.rhs[u]:
            self._push(u)

    def compute_shortest_path(self, max_iters: int = 200000) -> int:
        iters = 0
        while True:
            top = self._top()
            if top is None:
                break
            kold, u = top
            knew_start = self.calculate_key(self.start)
            if not (kold < knew_start or self.rhs[self.start] > self.g[self.start]):
                break

            knew = self.calculate_key(u)
            if kold < knew:
                # re-prioritise
                self.in_queue.discard((u, kold))
                self._push(u)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                self.in_queue.discard((u, kold))
                # pop the entry
                heapq.heappop(self.U)
                for s in self.neighbors(u):
                    if s != self.goal:
                        self.rhs[s] = min(self.rhs[s], self.cost(s, u) + self.g[u])
                    self.update_vertex(s)
            else:
                g_old = self.g[u]
                self.g[u] = INF
                self.in_queue.discard((u, kold))
                heapq.heappop(self.U)
                for s in list(self.neighbors(u)) + [u]:
                    if self.rhs[s] == self.cost(s, u) + g_old and s != self.goal:
                        best = INF
                        for sp in self.neighbors(s):
                            c = self.cost(s, sp) + self.g[sp]
                            if c < best:
                                best = c
                        self.rhs[s] = best
                    self.update_vertex(s)

            iters += 1
            if iters > max_iters:
                break
        return iters

    def extract_path(self, max_len: int = 5000) -> List[Tuple[int, int]]:
        if self.g[self.start] == INF:
            return []
        path = [self.start]
        cur = self.start
        while cur != self.goal and len(path) < max_len:
            best = None
            best_cost = INF
            for s in self.neighbors(cur):
                c = self.cost(cur, s) + self.g[s]
                if c < best_cost:
                    best_cost = c
                    best = s
            if best is None or best_cost == INF:
                return []
            cur = best
            path.append(cur)
        return path

    def update_edges(self, changed_cells: List[Tuple[int, int]]) -> None:
        """Called when grid cells (passed in) have changed cost. Reflect on the search state."""
        if not changed_cells:
            return
        for c in changed_cells:
            self.update_vertex(c)
            for n in self.neighbors(c):
                self.update_vertex(n)

    def shift_start(self, new_start: Tuple[int, int]) -> None:
        """Robot moved — bump k_m by the heuristic distance and update start."""
        self.k_m += octile(self.last_start, new_start)
        self.last_start = new_start
        self.start = new_start


# ---------------- ROS2 node ----------------


class DStarLiteNode(Node):
    def __init__(self) -> None:
        super().__init__('dstar_lite_node')
        self.declare_parameter('map_pgm_path', '')
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_radius_m', 0.30)
        self.declare_parameter('occupancy_threshold', 200)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.robot_radius = float(self.get_parameter('robot_radius_m').value)
        self.occ_thresh = int(self.get_parameter('occupancy_threshold').value)

        map_pgm = str(self.get_parameter('map_pgm_path').value)
        map_yaml = str(self.get_parameter('map_yaml_path').value)
        if not map_pgm or not map_yaml:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('kolestel_rover_description')
            map_pgm = map_pgm or os.path.join(pkg, 'maps', 'warehouse_nav2_map.pgm')
            map_yaml = map_yaml or os.path.join(pkg, 'maps', 'warehouse_nav2_map.yaml')

        self._load_map(map_pgm, map_yaml)
        self.planner = DStarLite(self.grid)
        self.active_goal: Optional[Tuple[int, int]] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(Path, '/grid_planner/path', 5)
        self.visited_pub = self.create_publisher(PointCloud2, '/grid_planner/visited', 5)
        self.stats_pub = self.create_publisher(String, '/grid_planner/stats', 5)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 5)
        self.create_subscription(OccupancyGrid, '/local_costmap', self.local_costmap_cb, 5)

        self.get_logger().info(
            f'dstar_lite_node ready: grid={self.grid.shape[1]}x{self.grid.shape[0]} '
            f'res={self.res:.3f}m origin=({self.origin_x:.2f},{self.origin_y:.2f})'
        )

    def _load_map(self, pgm: str, yaml_path: str) -> None:
        meta = yaml.safe_load(open(yaml_path, encoding='utf-8'))
        self.res = float(meta['resolution'])
        origin = meta.get('origin', [0.0, 0.0, 0.0])
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        img = np.flipud(np.array(Image.open(pgm))).copy()
        occupied = (img < self.occ_thresh).astype(np.uint8)
        from scipy.ndimage import binary_dilation
        r = max(1, int(math.ceil(self.robot_radius / self.res)))
        Y, X = np.ogrid[-r:r + 1, -r:r + 1]
        kernel = (X * X + Y * Y) <= r * r
        self.grid = binary_dilation(occupied, structure=kernel).astype(np.uint8)
        self.h, self.w = self.grid.shape

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor((y - self.origin_y) / self.res)),
                int(math.floor((x - self.origin_x) / self.res)))

    def grid_to_world(self, r: int, c: int) -> Tuple[float, float]:
        return self.origin_x + (c + 0.5) * self.res, self.origin_y + (r + 0.5) * self.res

    def _robot_cell(self) -> Optional[Tuple[int, int]]:
        try:
            ts = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException:
            return None
        return self.world_to_grid(ts.transform.translation.x, ts.transform.translation.y)

    def goal_cb(self, msg: PoseStamped) -> None:
        start = self._robot_cell()
        if start is None:
            self.get_logger().error('no TF for robot pose')
            return
        goal = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)
        self.active_goal = goal

        t0 = time.perf_counter()
        self.planner.initialize(start, goal)
        iters = self.planner.compute_shortest_path()
        elapsed = (time.perf_counter() - t0) * 1000.0
        path = self.planner.extract_path()
        self._publish_path(path, iters, elapsed, fresh=True)

    def local_costmap_cb(self, msg: OccupancyGrid) -> None:
        if self.active_goal is None:
            return
        start = self._robot_cell()
        if start is None:
            return
        # determine which global-grid cells fall inside the local costmap window
        # and have become lethal/inscribed. Update only those.
        cm = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        lethal_local = np.where(cm >= 90)
        if lethal_local[0].size == 0:
            return
        # convert local indices to world, then to our grid indices
        lr, lc = lethal_local
        wx = ox + (lc + 0.5) * msg.info.resolution
        wy = oy + (lr + 0.5) * msg.info.resolution
        gc = np.floor((wx - self.origin_x) / self.res).astype(int)
        gr = np.floor((wy - self.origin_y) / self.res).astype(int)
        keep = (gc >= 0) & (gc < self.w) & (gr >= 0) & (gr < self.h)
        gr = gr[keep]; gc = gc[keep]
        if gr.size == 0:
            return
        # mark them blocked in our grid (only those newly blocked)
        changed = []
        for r, c in zip(gr.tolist(), gc.tolist()):
            if self.grid[r, c] == 0:
                self.grid[r, c] = 1
                changed.append((r, c))
        if not changed:
            return
        # incremental update
        self.planner.shift_start(start)
        self.planner.update_edges(changed)
        t0 = time.perf_counter()
        iters = self.planner.compute_shortest_path()
        elapsed = (time.perf_counter() - t0) * 1000.0
        path = self.planner.extract_path()
        if path:
            self._publish_path(path, iters, elapsed, fresh=False)

    def _publish_path(self, cells, iters, elapsed_ms, fresh):
        if not cells:
            self.get_logger().error('no path found')
            return
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        path_len = 0.0
        prev = None
        for (r, c) in cells:
            ps = PoseStamped()
            ps.header = path.header
            wx, wy = self.grid_to_world(r, c)
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
            if prev is not None:
                path_len += math.hypot(wx - prev[0], wy - prev[1])
            prev = (wx, wy)
        self.path_pub.publish(path)
        stats = {
            'algorithm': 'd_star_lite',
            'planning_time_ms': round(elapsed_ms, 2),
            'iters': int(iters),
            'path_length_m': round(path_len, 2),
            'path_points': len(cells),
            'incremental': not fresh,
            'success': True,
        }
        self.stats_pub.publish(String(data=json.dumps(stats)))
        self.get_logger().info(f'D* Lite {"replan" if not fresh else "plan"}: {json.dumps(stats)}')


def main() -> None:
    rclpy.init()
    node = DStarLiteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
