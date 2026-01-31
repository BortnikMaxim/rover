import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class LineFollowNode(Node):
    def __init__(self):
        super().__init__('line_follow_node')

        # Topics
        self.declare_parameter('image_topic', '/line_camera')   # IMPORTANT: у вас Image уже на /line_camera
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Motion
        self.declare_parameter('base_speed', 0.35)
        self.declare_parameter('max_ang', 1.2)

        # PID
        self.declare_parameter('kp', 0.006)
        self.declare_parameter('ki', 0.0000)
        self.declare_parameter('kd', 0.0015)

        # Vision
        self.declare_parameter('invert_for_black_line', True)   # True: ищем черную линию на светлом полу
        self.declare_parameter('roi_y_from_bottom', 180)        # высота ROI снизу (px)
        self.declare_parameter('min_contour_area', 800)         # фильтр мусора
        self.declare_parameter('blur_ksize', 5)
        self.declare_parameter('morph_ksize', 5)

        # Lost line behavior
        self.declare_parameter('lost_turn', 0.35)               # как крутиться при потере линии
        self.declare_parameter('lost_speed', 0.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.base_speed = float(self.get_parameter('base_speed').value)
        self.max_ang = float(self.get_parameter('max_ang').value)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)

        self.inv_black = bool(self.get_parameter('invert_for_black_line').value)
        self.roi_h = int(self.get_parameter('roi_y_from_bottom').value)
        self.min_area = int(self.get_parameter('min_contour_area').value)
        self.blur_ksize = int(self.get_parameter('blur_ksize').value)
        self.morph_ksize = int(self.get_parameter('morph_ksize').value)

        self.lost_turn = float(self.get_parameter('lost_turn').value)
        self.lost_speed = float(self.get_parameter('lost_speed').value)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_dbg = self.create_publisher(Image, '/line_debug/image_raw', 10)

        self.err_i = 0.0
        self.prev_err = 0.0
        self.prev_t = time.time()

        self.get_logger().info(f"LineFollow: image={self.image_topic} -> cmd_vel={self.cmd_vel_topic}")

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def on_image(self, msg: Image):
        # Convert image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        h, w = frame.shape[:2]
        roi_h = min(self.roi_h, h)
        roi = frame[h - roi_h:h, :]

        # Preprocess
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        if self.blur_ksize >= 3 and self.blur_ksize % 2 == 1:
            gray = cv2.GaussianBlur(gray, (self.blur_ksize, self.blur_ksize), 0)

        # Threshold (adaptive-ish): Otsu is robust to lighting
        # For black line: line pixels are dark => we want mask = 1 on line
        _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        if self.inv_black:
            mask = cv2.bitwise_not(th)   # black line becomes white in mask
        else:
            mask = th# Morphology to clean
        k = max(3, self.morph_ksize)
        kernel = np.ones((k, k), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= self.min_area]

        twist = Twist()

        debug = roi.copy()
        cx = None

        if not contours:
            # Lost line
            twist.linear.x = self.lost_speed
            twist.angular.z = self.lost_turn
            self._publish(debug, mask, msg)
            self.pub_cmd.publish(twist)
            return

        # Take the largest contour
        c = max(contours, key=cv2.contourArea)
        cv2.drawContours(debug, [c], -1, (0, 255, 0), 2)

        M = cv2.moments(c)
        if M["m00"] > 1e-6:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(debug, (cx, cy), 6, (0, 0, 255), -1)
        else:
            twist.linear.x = self.lost_speed
            twist.angular.z = self.lost_turn
            self._publish(debug, mask, msg)
            self.pub_cmd.publish(twist)
            return

        # Error: center of image minus line center
        err = (w / 2.0) - float(cx)

        # PID
        now = time.time()
        dt = max(1e-3, now - self.prev_t)
        self.prev_t = now

        # integral with anti-windup
        self.err_i = self._clamp(self.err_i + err * dt, -5000.0, 5000.0)
        derr = (err - self.prev_err) / dt
        self.prev_err = err

        ang = self.kp * err + self.ki * self.err_i + self.kd * derr
        ang = self._clamp(ang, -self.max_ang, self.max_ang)

        # speed scheduling: slow down on big turns
        v = self.base_speed * (1.0 - min(abs(ang) / self.max_ang, 0.7))
        v = max(0.10, v)

        twist.linear.x = v
        twist.angular.z = ang

        # debug overlay
        cv2.line(debug, (int(w/2), 0), (int(w/2), roi_h-1), (255, 0, 0), 2)
        cv2.putText(debug, f"err={err:.1f} ang={ang:.2f} v={v:.2f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        self._publish(debug, mask, msg)
        self.pub_cmd.publish(twist)

    def _publish(self, debug_bgr, mask, msg):
        # Put mask to debug (top-left)
        mh, mw = mask.shape[:2]
        small = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        small = cv2.resize(small, (min(320, mw), min(180, mh)))
        debug_bgr[0:small.shape[0], 0:small.shape[1]] = small

        try:
            dbg = self.bridge.cv2_to_imgmsg(debug_bgr, encoding='bgr8')
            dbg.header = msg.header
            self.pub_dbg.publish(dbg)
        except Exception:
            pass


def main():
    rclpy.init()
    node = LineFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
