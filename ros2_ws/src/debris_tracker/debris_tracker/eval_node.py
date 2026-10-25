#!/usr/bin/env python3
"""
eval_node.py
Checks the tracker against the Unity twin's ground truth, live.

For every /urex/debris/position (camera optical frame), it takes Unity's camera and debris poses with the
same stamp, expresses the true debris position in the camera optical frame, and compares:
  err_centre  estimate - true debris centre
  err_face    estimate - true front-face point (centre moved debris_size/2 towards the camera along the line
              of sight); the tracker measures the visible face, so this is the fair error
Prints a summary every `report_period` s and optionally writes every sample to a CSV.

  ros2 run debris_tracker eval_node
  ros2 run debris_tracker eval_node --ros-args -p csv:=/tmp/eval.csv
"""

import csv
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def key(stamp):
    return (stamp.sec, stamp.nanosec)


class Eval(Node):
    def __init__(self):
        super().__init__("debris_eval")
        self.declare_parameter("debris_size", 0.1)
        self.declare_parameter("report_period", 2.0)
        self.declare_parameter("csv", "")
        self.half = 0.5 * self.get_parameter("debris_size").value
        self.cam, self.deb = {}, {}   # stamp -> pose (Unity publishes both with the image's stamp)
        self.rows = []
        self.unmatched = 0

        path = self.get_parameter("csv").value
        self.csv = None
        if path:
            self.csv = csv.writer(open(path, "w", newline=""))
            self.csv.writerow(["t", "est_x", "est_y", "est_z", "true_x", "true_y", "true_z",
                               "err_centre", "err_face", "range_err_face", "angle_err_deg"])

        self.create_subscription(PoseStamped, "/twin/camera/pose", lambda m: self.store(self.cam, m), 50)
        self.create_subscription(PoseStamped, "/twin/debris/pose", lambda m: self.store(self.deb, m), 50)
        self.create_subscription(PointStamped, "/urex/debris/position", self.on_estimate, 50)
        self.create_timer(self.get_parameter("report_period").value, self.report)
        self.get_logger().info("comparing /urex/debris/position with Unity ground truth (camera frame)")

    def store(self, table, msg):
        table[key(msg.header.stamp)] = msg.pose
        if len(table) > 200:
            del table[next(iter(table))]

    def on_estimate(self, msg):
        k = key(msg.header.stamp)
        if k not in self.cam or k not in self.deb:
            self.unmatched += 1
            return
        cam, deb = self.cam[k], self.deb[k]
        R = quat_to_matrix(cam.orientation)                     # optical frame -> world
        d = np.array([deb.position.x - cam.position.x, deb.position.y - cam.position.y,
                      deb.position.z - cam.position.z])
        true_c = R.T @ d                                         # true centre, optical frame
        r = np.linalg.norm(true_c)
        true_f = true_c * (r - self.half) / r                    # front face along the line of sight
        est = np.array([msg.point.x, msg.point.y, msg.point.z])

        e_c = float(np.linalg.norm(est - true_c))
        e_f = float(np.linalg.norm(est - true_f))
        range_f = float(np.linalg.norm(est) - (r - self.half))
        cosang = np.dot(est, true_c) / (np.linalg.norm(est) * r)
        ang = math.degrees(math.acos(float(np.clip(cosang, -1.0, 1.0))))
        self.rows.append((e_c, e_f, range_f, ang))
        if self.csv:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.csv.writerow([f"{t:.3f}", *(f"{v:.4f}" for v in est), *(f"{v:.4f}" for v in true_c),
                               f"{e_c:.4f}", f"{e_f:.4f}", f"{range_f:.4f}", f"{ang:.3f}"])

    def report(self):
        if not self.rows:
            self.get_logger().info(f"no matched samples yet (unmatched estimates: {self.unmatched}); "
                                   "is Unity playing and the debris detected?")
            return
        a = np.array(self.rows)
        self.get_logger().info(
            f"n={len(a):3d}  error vs face: mean {a[:, 1].mean() * 100:.1f} cm, max {a[:, 1].max() * 100:.1f} cm | "
            f"vs centre: mean {a[:, 0].mean() * 100:.1f} cm | range err {a[:, 2].mean() * 100:+.1f} cm | "
            f"direction err {a[:, 3].mean():.2f} deg")
        self.rows.clear()


def main():
    rclpy.init()
    node = Eval()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
