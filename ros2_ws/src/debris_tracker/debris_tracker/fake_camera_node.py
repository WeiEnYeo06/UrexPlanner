#!/usr/bin/env python3
"""
fake_camera_node.py
Stand-in for the real/Unity camera, to test the tracker loop without hardware.

Simulates a pan-tilt camera looking at a debris cube that starts off-centre. It publishes
rgb8 images and CameraInfo, and turns the camera by integrating the tracker's
camera/cmd_twist. With the loop closed, the cube should move to the crosshair.

  ros2 launch debris_tracker tracker.launch.py fake_camera:=true
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TwistStamped

from debris_tracker.perception import intrinsics_from_fov


class FakeCamera(Node):
    def __init__(self):
        super().__init__("fake_camera")
        self.declare_parameter("width", 424)
        self.declare_parameter("height", 240)
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("debris_start", [0.12, -0.06, 0.30])  # camera frame at t=0 [m]
        self.declare_parameter("debris_size", 0.1)
        g = lambda n: self.get_parameter(n).value
        self.w, self.h, self.size = g("width"), g("height"), g("debris_size")
        self.K = intrinsics_from_fov(self.w, self.h, 87.0, 58.0)
        self.p_world = np.array(g("debris_start"), float)
        self.R = np.eye(3)      # camera -> world rotation
        self.omega = np.zeros(3)
        self.dt = 1.0 / g("rate")

        self.pub_img = self.create_publisher(Image, "/camera/camera/color/image_raw", 10)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/camera/color/camera_info", 10)
        self.create_subscription(TwistStamped, "/urex/camera/cmd_twist", self.on_twist, 10)
        self.create_timer(self.dt, self.tick)

    def on_twist(self, msg):
        a = msg.twist.angular
        self.omega = np.array([a.x, a.y, a.z])

    def tick(self):
        rot, _ = cv2.Rodrigues(self.omega * self.dt)   # body-frame increment
        self.R = self.R @ rot
        p = self.R.T @ self.p_world                     # debris in the camera frame

        fx, fy, cx, cy = self.K
        img = np.full((self.h, self.w, 3), (20, 20, 30), np.uint8)   # dark background, RGB
        if p[2] > 0.05:
            u, v = fx * p[0] / p[2] + cx, fy * p[1] / p[2] + cy
            half = 0.5 * fx * self.size / p[2]
            cv2.rectangle(img, (int(u - half), int(v - half)), (int(u + half), int(v + half)),
                          (204, 158, 51), -1)                       # Unity debris colour (0.80, 0.62, 0.20)

        stamp = self.get_clock().now().to_msg()
        m = Image(height=self.h, width=self.w, encoding="rgb8", is_bigendian=0, step=self.w * 3)
        m.header.stamp, m.header.frame_id = stamp, "d405_color_optical_frame"
        m.data = img.tobytes()
        self.pub_img.publish(m)

        info = CameraInfo(height=self.h, width=self.w, distortion_model="plumb_bob")
        info.header = m.header
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.d = [0.0] * 5
        self.pub_info.publish(info)


def main():
    rclpy.init()
    node = FakeCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
