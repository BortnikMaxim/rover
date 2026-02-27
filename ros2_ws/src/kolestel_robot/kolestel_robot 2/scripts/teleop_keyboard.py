#!/usr/bin/env python3
"""
Simple keyboard teleoperation script for the robot.
Alternative to teleop_twist_keyboard package.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

msg = """
Kolestel Robot Teleoperation
-----------------------------
Moving around:
   w
a  s  d

w/s : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space : stop

q/e : increase/decrease max speeds
z/c : increase/decrease acceleration

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
    ' ': (0, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'e': (0.9, 0.9),
    'z': (1.1, 1.0),
    'c': (1.0, 1.1),
}


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.speed = 0.5
        self.turn = 0.5
        self.x = 0.0
        self.th = 0.0
        
        self.get_logger().info(msg)
        
    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def run(self):
        try:
            while True:
                key = self.get_key()
                
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.th = moveBindings[key][1]
                elif key in speedBindings.keys():
                    self.speed *= speedBindings[key][0]
                    self.turn *= speedBindings[key][1]
                    self.get_logger().info(f'Speed: {self.speed:.2f}, Turn: {self.turn:.2f}')
                elif key == '\x03':  # CTRL-C
                    break
                else:
                    self.x = 0.0
                    self.th = 0.0
                
                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.angular.z = self.th * self.turn
                self.publisher.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Stop robot
            twist = Twist()
            self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
