#!/usr/bin/env python3
"""
Pure-pursuit path follower + NavigateToPose action server.

Subscribes to /grid_planner/path (nav_msgs/Path) and drives the robot along it
using a lookahead point. Exposes a Nav2-compatible NavigateToPose action so the
web app's task_nav2_bridge can dispatch goals through this node:

  Web app -> task_nav2_bridge --action--> /navigate_to_pose
                                            |
                                            |  publish /goal_pose
                                            v
                                          grid_planner_node
                                            |
                                            |  publish /grid_planner/path
                                            v
                                          path_follower_node  -> /cmd_vel

When a goal is received via the action, we POST the planned route to the
backend (/robot/route) so the web UI can draw the path on the warehouse map.
"""
from __future__ import annotations

import json
import math
import os
import threading
import urllib.error
import urllib.request
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformException, TransformListener


BACKEND_URL = os.environ.get('AMR_BACKEND_URL', 'http://127.0.0.1:8010')
urllib.request.install_opener(urllib.request.build_opener(urllib.request.ProxyHandler({})))


class PathFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('path_follower_node')

        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('lookahead_m', 1.0)
        self.declare_parameter('linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 0.4)
        self.declare_parameter('arrival_tolerance_m', 0.25)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('obstacle_stop_distance_m', 1.0)
        self.declare_parameter('obstacle_lateral_half_width_m', 0.5)
        self.declare_parameter('obstacle_min_forward_m', 0.2)
        self.declare_parameter('plan_wait_s', 5.0)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.lookahead = float(self.get_parameter('lookahead_m').value)
        self.v_lin = float(self.get_parameter('linear_speed').value)
        self.w_max = float(self.get_parameter('max_angular_speed').value)
        self.arrival_tol = float(self.get_parameter('arrival_tolerance_m').value)
        rate = float(self.get_parameter('control_rate_hz').value)
        self.obs_dist = float(self.get_parameter('obstacle_stop_distance_m').value)
        self.obs_lat = float(self.get_parameter('obstacle_lateral_half_width_m').value)
        self.obs_min = float(self.get_parameter('obstacle_min_forward_m').value)
        self.plan_wait_s = float(self.get_parameter('plan_wait_s').value)

        self.path_pts: List[Tuple[float, float]] = []
        self.obstacle_blocked = False
        self._lock = threading.Lock()
        self._active_goal_xy: Optional[Tuple[float, float]] = None
        self._path_event = threading.Event()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        cb_group = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 5)
        self.create_subscription(Path, '/grid_planner/path', self.path_cb, 5, callback_group=cb_group)
        self.create_subscription(PointCloud2, '/lidar', self.lidar_cb, 5, callback_group=cb_group)
        self.create_timer(1.0 / max(rate, 1.0), self.control_tick, callback_group=cb_group)

        self.action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_navigate,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f'path_follower ready: lookahead={self.lookahead:.2f}m v={self.v_lin:.2f}m/s '
            f'arrival={self.arrival_tol:.2f}m obstacle_stop={self.obs_dist:.2f}m'
        )

    # ---------- subscribers ----------

    def path_cb(self, msg: Path) -> None:
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self._lock:
            self.path_pts = pts
        self._path_event.set()
        self.get_logger().info(f'received path: {len(pts)} pts')
        # mirror path to the web UI as the active route
        self._post_route(pts, phase='active')

    def lidar_cb(self, msg: PointCloud2) -> None:
        min_d = float('inf')
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y = float(p[0]), float(p[1])
            if x < self.obs_min:
                continue
            if abs(y) > self.obs_lat:
                continue
            d = math.hypot(x, y)
            if d < min_d:
                min_d = d
        was = self.obstacle_blocked
        self.obstacle_blocked = min_d < self.obs_dist
        if self.obstacle_blocked and not was:
            self.get_logger().warn(f'obstacle at {min_d:.2f}m forward, holding')
        elif was and not self.obstacle_blocked:
            self.get_logger().info('path clear, resuming')

    # ---------- control loop ----------

    def _get_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            ts = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None
        x = float(ts.transform.translation.x)
        y = float(ts.transform.translation.y)
        q = ts.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return x, y, yaw

    def control_tick(self) -> None:
        with self._lock:
            pts = list(self.path_pts)
            goal_xy = self._active_goal_xy
        if not pts:
            return

        pose = self._get_pose()
        if pose is None:
            return
        x, y, yaw = pose

        # arrival on the action goal (precise final point), not the path tail
        target_xy = goal_xy if goal_xy is not None else pts[-1]
        if math.hypot(target_xy[0] - x, target_xy[1] - y) < self.arrival_tol:
            self.cmd_pub.publish(Twist())
            return

        # arrival on the path tail (transient — wait for new plan or stop)
        gx_t, gy_t = pts[-1]
        if math.hypot(gx_t - x, gy_t - y) < self.arrival_tol:
            self.cmd_pub.publish(Twist())
            with self._lock:
                self.path_pts = []
            return

        target = self._lookahead_point(x, y, pts)
        if target is None:
            self.cmd_pub.publish(Twist())
            return
        tx, ty = target

        dx = tx - x
        dy = ty - y
        cos_y = math.cos(-yaw)
        sin_y = math.sin(-yaw)
        local_x = cos_y * dx - sin_y * dy
        local_y = sin_y * dx + cos_y * dy

        if local_x <= 0.05:
            tw = Twist()
            tw.angular.z = self.w_max if local_y > 0 else -self.w_max
            self.cmd_pub.publish(tw)
            return

        if self.obstacle_blocked:
            tw = Twist()
            angle_to_target = math.atan2(local_y, local_x)
            tw.angular.z = max(-self.w_max, min(self.w_max, 1.5 * angle_to_target))
            self.cmd_pub.publish(tw)
            return

        L = max(math.hypot(local_x, local_y), 1e-3)
        kappa = 2.0 * local_y / (L * L)
        v = self.v_lin
        w = max(-self.w_max, min(self.w_max, v * kappa))
        tw = Twist()
        tw.linear.x = v
        tw.angular.z = w
        self.cmd_pub.publish(tw)

    def _lookahead_point(self, x: float, y: float, pts: List[Tuple[float, float]]):
        best_i = 0
        best_d = float('inf')
        for i, (px, py) in enumerate(pts):
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best_d = d
                best_i = i
        for i in range(best_i, len(pts)):
            px, py = pts[i]
            if math.hypot(px - x, py - y) >= self.lookahead:
                return px, py
        return pts[-1]

    # ---------- NavigateToPose action ----------

    def handle_goal(self, goal_request):
        self.get_logger().info('navigate_to_pose goal accepted')
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        self.get_logger().warn('navigate_to_pose cancel requested')
        with self._lock:
            self.path_pts = []
            self._active_goal_xy = None
        self.cmd_pub.publish(Twist())
        return CancelResponse.ACCEPT

    def execute_navigate(self, goal_handle):
        result = NavigateToPose.Result()
        goal_pose = goal_handle.request.pose
        gx = float(goal_pose.pose.position.x)
        gy = float(goal_pose.pose.position.y)
        self.get_logger().info(f'navigate_to_pose executing to ({gx:.2f}, {gy:.2f})')

        with self._lock:
            self._active_goal_xy = (gx, gy)
            self.path_pts = []
        self._path_event.clear()

        # forward to planner
        gp = PoseStamped()
        gp.header.frame_id = self.map_frame
        gp.header.stamp = self.get_clock().now().to_msg()
        gp.pose = goal_pose.pose
        self.goal_pose_pub.publish(gp)

        if not self._path_event.wait(timeout=self.plan_wait_s):
            self.get_logger().error('planner did not produce a path in time')
            with self._lock:
                self._active_goal_xy = None
            self._post_route([], phase='idle')
            goal_handle.abort()
            return result

        rate = self.create_rate(5.0)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('execute_navigate: cancel triggered')
                with self._lock:
                    self.path_pts = []
                    self._active_goal_xy = None
                self.cmd_pub.publish(Twist())
                self._post_route([], phase='idle')
                goal_handle.canceled()
                return result

            pose = self._get_pose()
            if pose is not None:
                x, y, _ = pose
                if math.hypot(gx - x, gy - y) < self.arrival_tol:
                    self.cmd_pub.publish(Twist())
                    with self._lock:
                        self.path_pts = []
                        self._active_goal_xy = None
                    self._post_route([], phase='idle')
                    self.get_logger().info('navigate_to_pose: goal reached')
                    goal_handle.succeed()
                    return result
                fb = NavigateToPose.Feedback()
                fb.current_pose.header.frame_id = self.map_frame
                fb.current_pose.header.stamp = self.get_clock().now().to_msg()
                fb.current_pose.pose.position.x = x
                fb.current_pose.pose.position.y = y
                goal_handle.publish_feedback(fb)
            rate.sleep()
        goal_handle.abort()
        return result

    # ---------- /robot/route POST ----------

    def _post_route(self, pts, phase: str):
        # sample down: backend expects a short list of waypoints, not 5000 cells
        if len(pts) > 30:
            step = max(1, len(pts) // 30)
            sampled = pts[::step] + [pts[-1]]
        else:
            sampled = list(pts)
        payload = {
            'points': [{'x': float(px), 'y': float(py)} for px, py in sampled],
            'phase': phase,
        }

        def _send():
            try:
                req = urllib.request.Request(
                    BACKEND_URL.rstrip('/') + '/robot/route',
                    data=json.dumps(payload).encode('utf-8'),
                    headers={'Content-Type': 'application/json'},
                    method='POST',
                )
                with urllib.request.urlopen(req, timeout=2.0) as resp:
                    resp.read()
            except Exception:
                pass

        threading.Thread(target=_send, daemon=True).start()


def main() -> None:
    rclpy.init()
    node = PathFollowerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
