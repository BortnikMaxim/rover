#!/usr/bin/env python3
"""
Robot controller node for Kolestel robot.
Provides basic motion control and sensor data processing.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
import math


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('obstacle_distance', 0.5)
        self.declare_parameter('enable_obstacle_avoidance', True)
        
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.obstacle_dist = self.get_parameter('obstacle_distance').value
        self.avoid_obstacles = self.get_parameter('enable_obstacle_avoidance').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_safe', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel_raw', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        
        # State variables
        self.current_pose = None
        self.current_velocity = None
        self.min_front_distance = float('inf')
        self.min_left_distance = float('inf')
        self.min_right_distance = float('inf')
        
        # Timer for status updates
        self.create_timer(1.0, self.status_callback)
        
        self.get_logger().info('Robot controller initialized')
        
    def cmd_vel_callback(self, msg):
        """Process incoming velocity commands with safety checks."""
        twist = Twist()
        
        # Clamp velocities
        twist.linear.x = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        twist.angular.z = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        
        # Obstacle avoidance
        if self.avoid_obstacles:
            if self.min_front_distance < self.obstacle_dist and twist.linear.x > 0:
                twist.linear.x = 0.0
                self.get_logger().warn('Obstacle detected! Stopping forward motion.')
        
        self.cmd_vel_pub.publish(twist)
        
    def odom_callback(self, msg):
        """Process odometry data."""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist
        
    def scan_callback(self, msg):
        """Process laser scan data for obstacle detection."""
        if len(msg.ranges) == 0:
            return
            
        num_ranges = len(msg.ranges)
        
        # Front sector (center 60 degrees)
        front_start = int(num_ranges * 0.4)
        front_end = int(num_ranges * 0.6)
        front_ranges = [r for r in msg.ranges[front_start:front_end] 
                       if msg.range_min < r < msg.range_max]
        self.min_front_distance = min(front_ranges) if front_ranges else float('inf')
        
        # Left sector
        left_start = int(num_ranges * 0.6)
        left_end = int(num_ranges * 0.8)
        left_ranges = [r for r in msg.ranges[left_start:left_end]
                      if msg.range_min < r < msg.range_max]
        self.min_left_distance = min(left_ranges) if left_ranges else float('inf')
        
        # Right sector
        right_start = int(num_ranges * 0.2)
        right_end = int(num_ranges * 0.4)
        right_ranges = [r for r in msg.ranges[right_start:right_end]
                       if msg.range_min < r < msg.range_max]
        self.min_right_distance = min(right_ranges) if right_ranges else float('inf')
        
    def imu_callback(self, msg):
        """Process IMU data."""
        # Can be extended for orientation tracking, tilt detection, etc.
        pass
        
    def status_callback(self):
        """Periodic status update."""
        if self.current_velocity:
            self.get_logger().debug(
                f'Velocity - Linear: {self.current_velocity.linear.x:.2f} m/s, '
                f'Angular: {self.current_velocity.angular.z:.2f} rad/s')
        self.get_logger().debug(
            f'Distances - Front: {self.min_front_distance:.2f}m, '
            f'Left: {self.min_left_distance:.2f}m, Right: {self.min_right_distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
