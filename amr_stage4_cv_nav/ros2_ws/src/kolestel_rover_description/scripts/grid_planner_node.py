#!/usr/bin/env python3
"""
Grid planner node — five algorithms over a 2D occupancy grid.

Algorithms (selectable via parameter `algorithm`):
  - dijkstra         Uninformed shortest path, expands like a wave.
  - a_star           Heuristic-guided A*, octile distance heuristic.
  - greedy_bfs       Greedy best-first, follows heuristic only. Fast, NOT optimal.
  - jps              Jump Point Search — A* with diagonal-skipping pruning.
                     Massive speedup on uniform 8-connected grids.
  - theta_star       Any-angle planner; uses line-of-sight to bypass intermediate
                     nodes. Produces SHORT, SMOOTH paths not aligned to the grid.

Topics:
  Sub:   /goal_pose                geometry_msgs/PoseStamped   from RViz "2D Goal Pose"
         /global_costmap           nav_msgs/OccupancyGrid      preferred map source
  Pub:   /map                      nav_msgs/OccupancyGrid      fallback latched map
         /grid_planner/path        nav_msgs/Path
         /grid_planner/visited     sensor_msgs/PointCloud2     expanded cells
         /grid_planner/stats       std_msgs/String             JSON {algorithm, time_ms,
                                                                expanded, path_length, points}
"""
from __future__ import annotations

import heapq
import json
import math
import os
import time
from typing import Dict, List, Optional, Tuple

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


# ---------------- algorithm implementations ----------------


def _octile(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    dy = abs(a[0] - b[0])
    dx = abs(a[1] - b[1])
    return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)


_NEIGH8 = [
    (-1, -1, SQRT2), (-1, 0, 1.0), (-1, 1, SQRT2),
    (0, -1, 1.0),                  (0, 1, 1.0),
    (1, -1, SQRT2),  (1, 0, 1.0),  (1, 1, SQRT2),
]


def _reconstruct(came, start, goal):
    cur = goal
    out = [cur]
    while cur != start:
        cur = came.get(cur)
        if cur is None:
            return []
        out.append(cur)
    return list(reversed(out))


def dijkstra(grid: np.ndarray, start, goal):
    h, w = grid.shape
    dist = np.full((h, w), math.inf, dtype=np.float64)
    dist[start] = 0.0
    visited = np.zeros((h, w), dtype=bool)
    came = {}
    pq = [(0.0, start)]
    while pq:
        d, (r, c) = heapq.heappop(pq)
        if visited[r, c]:
            continue
        visited[r, c] = True
        if (r, c) == goal:
            break
        for dr, dc, cost in _NEIGH8:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            if grid[nr, nc]:
                continue
            nd = d + cost
            if nd < dist[nr, nc]:
                dist[nr, nc] = nd
                came[(nr, nc)] = (r, c)
                heapq.heappush(pq, (nd, (nr, nc)))
    if not visited[goal]:
        return [], visited
    return _reconstruct(came, start, goal), visited


def a_star(grid: np.ndarray, start, goal):
    h, w = grid.shape
    gscore = np.full((h, w), math.inf, dtype=np.float64)
    gscore[start] = 0.0
    visited = np.zeros((h, w), dtype=bool)
    came = {}
    pq = [(_octile(start, goal), 0.0, start)]
    while pq:
        _, g, (r, c) = heapq.heappop(pq)
        if visited[r, c]:
            continue
        visited[r, c] = True
        if (r, c) == goal:
            break
        for dr, dc, cost in _NEIGH8:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            if grid[nr, nc]:
                continue
            ng = g + cost
            if ng < gscore[nr, nc]:
                gscore[nr, nc] = ng
                came[(nr, nc)] = (r, c)
                heapq.heappush(pq, (ng + _octile((nr, nc), goal), ng, (nr, nc)))
    if not visited[goal]:
        return [], visited
    return _reconstruct(came, start, goal), visited


def greedy_bfs(grid: np.ndarray, start, goal):
    h, w = grid.shape
    visited = np.zeros((h, w), dtype=bool)
    came = {}
    pq = [(_octile(start, goal), start)]
    while pq:
        _, (r, c) = heapq.heappop(pq)
        if visited[r, c]:
            continue
        visited[r, c] = True
        if (r, c) == goal:
            break
        for dr, dc, _cost in _NEIGH8:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            if grid[nr, nc] or visited[nr, nc]:
                continue
            if (nr, nc) not in came:
                came[(nr, nc)] = (r, c)
                heapq.heappush(pq, (_octile((nr, nc), goal), (nr, nc)))
    if not visited[goal]:
        return [], visited
    return _reconstruct(came, start, goal), visited


# -- JPS helpers --

def _jps_jump(grid, r, c, dr, dc, goal):
    """Recursive jump along (dr, dc). Returns a jump point cell or None."""
    h, w = grid.shape
    while True:
        nr, nc = r + dr, c + dc
        if not (0 <= nr < h and 0 <= nc < w):
            return None
        if grid[nr, nc]:
            return None
        if (nr, nc) == goal:
            return (nr, nc)
        # forced neighbor checks
        if dr != 0 and dc != 0:
            # diagonal: must be able to go straight along either component
            if (0 <= nr - dr < h and 0 <= nc < w and grid[nr - dr, nc] and
                    0 <= nr < h and 0 <= nc + dc < w and not grid[nr, nc + dc]):
                return (nr, nc)
            if (0 <= nr < h and 0 <= nc - dc < w and grid[nr, nc - dc] and
                    0 <= nr + dr < h and 0 <= nc < w and not grid[nr + dr, nc]):
                return (nr, nc)
            if (_jps_jump(grid, nr, nc, dr, 0, goal) is not None or
                    _jps_jump(grid, nr, nc, 0, dc, goal) is not None):
                return (nr, nc)
        elif dr != 0:
            # vertical
            if ((0 <= nc + 1 < w and grid[nr, nc + 1] and
                 0 <= nr + dr < h and not grid[nr + dr, nc + 1]) or
                (0 <= nc - 1 < w and grid[nr, nc - 1] and
                 0 <= nr + dr < h and not grid[nr + dr, nc - 1])):
                return (nr, nc)
        else:
            # horizontal
            if ((0 <= nr + 1 < h and grid[nr + 1, nc] and
                 0 <= nc + dc < w and not grid[nr + 1, nc + dc]) or
                (0 <= nr - 1 < h and grid[nr - 1, nc] and
                 0 <= nc + dc < w and not grid[nr - 1, nc + dc])):
                return (nr, nc)
        r, c = nr, nc


def jps(grid: np.ndarray, start, goal):
    """Jump Point Search — A* with diagonal skipping. Produces sparse waypoints."""
    h, w = grid.shape
    gscore = {start: 0.0}
    came = {}
    visited = np.zeros((h, w), dtype=bool)
    pq = [(_octile(start, goal), 0.0, start)]
    while pq:
        _, g, (r, c) = heapq.heappop(pq)
        if visited[r, c]:
            continue
        visited[r, c] = True
        if (r, c) == goal:
            break
        for dr, dc, _cost in _NEIGH8:
            jp = _jps_jump(grid, r, c, dr, dc, goal)
            if jp is None:
                continue
            step = math.hypot(jp[0] - r, jp[1] - c)
            ng = g + step
            if ng < gscore.get(jp, math.inf):
                gscore[jp] = ng
                came[jp] = (r, c)
                heapq.heappush(pq, (ng + _octile(jp, goal), ng, jp))
    if not visited[goal]:
        return [], visited
    # densify the JPS waypoints into a per-cell path for the follower
    waypoints = _reconstruct(came, start, goal)
    dense = []
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        dense.extend(_bresenham_line(a, b)[:-1])
    dense.append(waypoints[-1])
    return dense, visited


# -- Theta* helpers --

def _bresenham_line(a, b):
    """Integer cells along the segment a -> b, inclusive."""
    r0, c0 = a
    r1, c1 = b
    cells = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    if dc > dr:
        err = dc / 2.0
        r = r0
        for c in range(c0, c1 + sc, sc):
            cells.append((r, c))
            err -= dr
            if err < 0:
                r += sr
                err += dc
    else:
        err = dr / 2.0
        c = c0
        for r in range(r0, r1 + sr, sr):
            cells.append((r, c))
            err -= dc
            if err < 0:
                c += sc
                err += dr
    return cells


def _line_of_sight(grid, a, b):
    """True if the straight line from cell a to b stays on free cells."""
    h, w = grid.shape
    for r, c in _bresenham_line(a, b):
        if not (0 <= r < h and 0 <= c < w):
            return False
        if grid[r, c]:
            return False
    return True


def theta_star(grid: np.ndarray, start, goal):
    """Any-angle planner. Uses line-of-sight to skip intermediate nodes."""
    h, w = grid.shape
    gscore = np.full((h, w), math.inf, dtype=np.float64)
    gscore[start] = 0.0
    visited = np.zeros((h, w), dtype=bool)
    parent = {start: start}
    pq = [(_octile(start, goal), 0.0, start)]
    while pq:
        _, g, (r, c) = heapq.heappop(pq)
        if visited[r, c]:
            continue
        visited[r, c] = True
        if (r, c) == goal:
            break
        for dr, dc, cost in _NEIGH8:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            if grid[nr, nc]:
                continue
            p = parent[(r, c)]
            if _line_of_sight(grid, p, (nr, nc)):
                ng = gscore[p] + math.hypot(nr - p[0], nc - p[1])
                if ng < gscore[nr, nc]:
                    gscore[nr, nc] = ng
                    parent[(nr, nc)] = p
                    heapq.heappush(pq, (ng + _octile((nr, nc), goal), ng, (nr, nc)))
            else:
                ng = g + cost
                if ng < gscore[nr, nc]:
                    gscore[nr, nc] = ng
                    parent[(nr, nc)] = (r, c)
                    heapq.heappush(pq, (ng + _octile((nr, nc), goal), ng, (nr, nc)))
    if not visited[goal]:
        return [], visited
    # walk parent chain; densify the any-angle segments via Bresenham for the follower
    chain = []
    cur = goal
    while cur != start:
        chain.append(cur)
        cur = parent[cur]
    chain.append(start)
    chain.reverse()
    dense = []
    for a, b in zip(chain[:-1], chain[1:]):
        dense.extend(_bresenham_line(a, b)[:-1])
    dense.append(chain[-1])
    return dense, visited


ALGORITHMS = {
    'dijkstra': dijkstra,
    'a_star': a_star,
    'greedy_bfs': greedy_bfs,
    'jps': jps,
    'theta_star': theta_star,
}


# ---------------- node ----------------


class GridPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('grid_planner_node')

        self.declare_parameter('algorithm', 'a_star')
        self.declare_parameter('map_pgm_path', '')
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_radius_m', 0.30)
        self.declare_parameter('occupancy_threshold', 200)
        self.declare_parameter('use_external_costmap', True)

        self.algorithm = str(self.get_parameter('algorithm').value).lower()
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.robot_radius = float(self.get_parameter('robot_radius_m').value)
        self.occ_thresh = int(self.get_parameter('occupancy_threshold').value)
        self.use_external = bool(self.get_parameter('use_external_costmap').value)

        if self.algorithm not in ALGORITHMS:
            self.get_logger().warn(f'unknown algorithm "{self.algorithm}", defaulting to a_star')
            self.algorithm = 'a_star'

        map_pgm = str(self.get_parameter('map_pgm_path').value)
        map_yaml = str(self.get_parameter('map_yaml_path').value)
        if not map_pgm or not map_yaml:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('kolestel_rover_description')
            map_pgm = map_pgm or os.path.join(pkg, 'maps', 'warehouse_nav2_map.pgm')
            map_yaml = map_yaml or os.path.join(pkg, 'maps', 'warehouse_nav2_map.yaml')

        self._load_map(map_pgm, map_yaml)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.path_pub = self.create_publisher(Path, '/grid_planner/path', 5)
        self.visited_pub = self.create_publisher(PointCloud2, '/grid_planner/visited', 5)
        self.stats_pub = self.create_publisher(String, '/grid_planner/stats', 5)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 5)
        self.create_subscription(OccupancyGrid, '/global_costmap', self._costmap_cb, latched)

        self._publish_map(latched)

        self.get_logger().info(
            f'grid_planner_node ready: algorithm={self.algorithm} '
            f'map={os.path.basename(map_pgm)} grid={self.grid.shape[1]}x{self.grid.shape[0]} '
            f'res={self.res:.3f}m origin=({self.origin_x:.2f},{self.origin_y:.2f}) '
            f'robot_radius={self.robot_radius:.2f}m inflated_cells={self.inflated_radius_cells} '
            f'algos_available={list(ALGORITHMS.keys())}'
        )

    # ---------- map loading ----------

    def _load_map(self, pgm: str, yaml_path: str) -> None:
        meta = yaml.safe_load(open(yaml_path, encoding='utf-8'))
        self.res = float(meta['resolution'])
        origin = meta.get('origin', [0.0, 0.0, 0.0])
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])

        img = np.array(Image.open(pgm))
        img = np.flipud(img).copy()
        self.raw = img
        occupied = (img < self.occ_thresh).astype(np.uint8)
        r = max(1, int(math.ceil(self.robot_radius / self.res)))
        self.inflated_radius_cells = r
        from scipy.ndimage import binary_dilation
        Y, X = np.ogrid[-r:r + 1, -r:r + 1]
        kernel = (X * X + Y * Y) <= r * r
        inflated = binary_dilation(occupied, structure=kernel).astype(np.uint8)
        self.grid = inflated.astype(np.uint8)
        self.h, self.w = self.grid.shape

    def _publish_map(self, qos) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.res
        msg.info.width = self.w
        msg.info.height = self.h
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        data = np.where(self.grid == 1, 100, 0).astype(np.int8)
        msg.data = data.flatten().tolist()
        self.map_pub.publish(msg)

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        """Adopt an externally-published costmap (with proper inflation gradient).

        We treat any cell with value >= 50 (out of 100) as blocked for the
        binary search algorithms. The smooth gradient is preserved for any
        future cost-aware planner extensions.
        """
        if not self.use_external:
            return
        if (msg.info.resolution != self.res or
                msg.info.width != self.w or
                msg.info.height != self.h):
            self.get_logger().warn(
                f'ignoring /global_costmap with size mismatch '
                f'({msg.info.width}x{msg.info.height}@{msg.info.resolution}m)'
            )
            return
        data = np.array(msg.data, dtype=np.int8).reshape(self.h, self.w)
        self.grid = (data >= 50).astype(np.uint8)
        self.get_logger().info(
            f'adopted external /global_costmap: blocked_cells={int(self.grid.sum())}'
        )

    # ---------- helpers ----------

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        col = int(math.floor((x - self.origin_x) / self.res))
        row = int(math.floor((y - self.origin_y) / self.res))
        return row, col

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        return self.origin_x + (col + 0.5) * self.res, self.origin_y + (row + 0.5) * self.res

    def is_free(self, row: int, col: int) -> bool:
        return 0 <= row < self.h and 0 <= col < self.w and self.grid[row, col] == 0

    # ---------- goal callback ----------

    def goal_cb(self, msg: PoseStamped) -> None:
        try:
            ts = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as exc:
            self.get_logger().warn(f'no TF {self.map_frame}->{self.base_frame}: {exc}')
            return

        sx = float(ts.transform.translation.x)
        sy = float(ts.transform.translation.y)
        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)

        s_row, s_col = self.world_to_grid(sx, sy)
        g_row, g_col = self.world_to_grid(gx, gy)
        self.get_logger().info(
            f'plan {self.algorithm}: start=({sx:.2f},{sy:.2f})=({s_row},{s_col}) '
            f'goal=({gx:.2f},{gy:.2f})=({g_row},{g_col})'
        )

        if not self.is_free(s_row, s_col):
            self.get_logger().error(f'start cell ({s_row},{s_col}) blocked')
            return
        if not self.is_free(g_row, g_col):
            self.get_logger().error(f'goal cell ({g_row},{g_col}) blocked')
            return

        algo = ALGORITHMS[self.algorithm]
        t0 = time.perf_counter()
        cells, visited = algo(self.grid, (s_row, s_col), (g_row, g_col))
        elapsed = (time.perf_counter() - t0) * 1000.0

        if not cells:
            self.get_logger().error('no path found')
            self.stats_pub.publish(String(data=json.dumps({
                'algorithm': self.algorithm,
                'planning_time_ms': round(elapsed, 2),
                'expanded_cells': int(visited.sum()),
                'success': False,
            })))
            return

        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for (r, c) in cells:
            ps = PoseStamped()
            ps.header = path.header
            wx, wy = self.grid_to_world(r, c)
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

        self._publish_visited(visited)

        # path length + smoothness metric (total turn angle)
        path_len = 0.0
        total_turn = 0.0
        prev_heading = None
        for (r1, c1), (r2, c2) in zip(cells[:-1], cells[1:]):
            x1, y1 = self.grid_to_world(r1, c1)
            x2, y2 = self.grid_to_world(r2, c2)
            seg_len = math.hypot(x2 - x1, y2 - y1)
            path_len += seg_len
            if seg_len > 1e-6:
                heading = math.atan2(y2 - y1, x2 - x1)
                if prev_heading is not None:
                    dtheta = abs(((heading - prev_heading) + math.pi) % (2 * math.pi) - math.pi)
                    total_turn += dtheta
                prev_heading = heading

        stats = {
            'algorithm': self.algorithm,
            'planning_time_ms': round(elapsed, 2),
            'expanded_cells': int(visited.sum()),
            'path_length_m': round(path_len, 2),
            'path_points': len(cells),
            'total_turn_rad': round(total_turn, 2),
            'success': True,
        }
        self.stats_pub.publish(String(data=json.dumps(stats)))
        self.get_logger().info(f'plan ok: {json.dumps(stats)}')

    # ---------- visited cloud ----------

    def _publish_visited(self, visited: np.ndarray) -> None:
        rows, cols = np.where(visited)
        if rows.size == 0:
            return
        if rows.size > 50000:
            step = rows.size // 50000 + 1
            rows = rows[::step]
            cols = cols[::step]
        xs = self.origin_x + (cols.astype(np.float32) + 0.5) * self.res
        ys = self.origin_y + (rows.astype(np.float32) + 0.5) * self.res
        zs = np.full_like(xs, 0.05, dtype=np.float32)
        pts = np.stack([xs, ys, zs], axis=1).astype(np.float32)

        msg = PointCloud2()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = pts.shape[0]
        msg.is_bigendian = False
        msg.is_dense = True
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = pts.tobytes()
        self.visited_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = GridPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
