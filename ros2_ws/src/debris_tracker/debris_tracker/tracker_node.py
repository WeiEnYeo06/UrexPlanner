#!/usr/bin/env python3
"""
tracker_node.py
ROS 2 node: reads camera images, finds the debris, publishes commands that centre the camera
on it, and publishes the debris coordinates for the other team.

Subscribes (topic names are parameters, defaults match the realsense2_camera driver):
  image_topic        sensor_msgs/Image        RGB image (rgb8 / bgr8 / rgba8 / bgra8 / mono8)
  camera_info_topic  sensor_msgs/CameraInfo   intrinsics (falls back to the D405 FOV until received)
  depth_topic        sensor_msgs/Image        optional, aligned depth (16UC1 mm or 32FC1 m)

Publishes (relative names; the launch file puts them under /urex):
  debris/position         geometry_msgs/PointStamped   debris position, camera optical frame [m]
  debris/position_world   geometry_msgs/PointStamped   same point in `world_frame` (only if TF has it)
  debris/detected         std_msgs/Bool                True while the debris is tracked
  camera/centering_error  geometry_msgs/Vector3Stamped x = horizontal, y = vertical angle error [rad],
                                                       z = pixel distance from the image centre
  camera/cmd_twist        geometry_msgs/TwistStamped   angular velocity (optical frame) that centres
                                                       the debris; zero when lost
  camera/cmd_pose         geometry_msgs/PoseStamped    look-at setpoint in the optical frame: on the
                                                       line of sight at `standoff`, facing the debris
  debris/debug_image      sensor_msgs/Image            annotated image (bgr8)
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped, Vector3Stamped
from std_msgs.msg import Bool

from debris_tracker import perception as P


def image_to_numpy(msg):
    """sensor_msgs/Image -> (array, kind) without cv_bridge. kind is 'rgb' or 'depth' (metres)."""
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        a = a.reshape(msg.height, msg.width, 3)
        return (a if enc == "rgb8" else a[..., ::-1]).copy(), "rgb"
    if enc in ("rgba8", "bgra8"):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 4]
        a = a.reshape(msg.height, msg.width, 4)[..., :3]
        return (a if enc == "rgba8" else a[..., ::-1]).copy(), "rgb"
    if enc in ("mono8", "8uc1"):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
        return np.repeat(a[..., None], 3, axis=2), "rgb"
    if enc in ("16uc1", "mono16"):
        dt = np.dtype("<u2" if not msg.is_bigendian else ">u2")
        a = np.frombuffer(msg.data, dt).reshape(msg.height, msg.step // 2)[:, :msg.width]
        return a.astype(np.float32) / 1000.0, "depth"   # RealSense: millimetres
    if enc == "32fc1":
        dt = np.dtype("<f4" if not msg.is_bigendian else ">f4")
        a = np.frombuffer(msg.data, dt).reshape(msg.height, msg.step // 4)[:, :msg.width]
        return a.astype(np.float32), "depth"
    raise ValueError(f"unsupported image encoding '{msg.encoding}'")


class DebrisTracker(Node):
    def __init__(self):
        super().__init__("debris_tracker")
        p = self.declare_parameter
        p("image_topic", "/camera/camera/color/image_raw")
        p("camera_info_topic", "/camera/camera/color/camera_info")
        p("depth_topic", "")                    # e.g. /camera/camera/aligned_depth_to_color/image_raw
        p("detector", "color")                  # color | yolo
        p("hsv_lower", [15, 120, 80])
        p("hsv_upper", [30, 255, 255])
        p("yolo_weights", "")
        p("yolo_conf", 0.4)
        p("debris_size", 0.1)                   # 1U cube edge [m], used for range when there is no depth
        p("min_pixels", 30)
        p("erode_px", 2)
        p("alpha", 0.6)                         # low-pass on the estimate, 1 = off
        p("max_lost", 10)                       # frames to coast on the last estimate
        p("gain", 1.5)                          # centering gain [1/s]
        p("max_rate", 0.5)                      # angular-rate limit [rad/s]
        p("deadband_deg", 0.5)
        p("standoff", 0.2)                      # look-at setpoint distance [m] (hold_distance)
        p("camera_frame", "")                   # override header.frame_id of the image
        p("world_frame", "")                    # e.g. "world": also publish position_world via TF
        p("publish_debug_image", True)
        p("fallback_hfov_deg", 87.0)            # D405
        p("fallback_vfov_deg", 58.0)
        p("depth_max_age", 0.1)                 # [s] max stamp gap between colour and depth
        g = lambda n: self.get_parameter(n).value

        if g("detector") == "yolo":
            if not g("yolo_weights"):
                raise SystemExit("detector:=yolo needs yolo_weights:=path/to/best.pt")
            self.detector = P.YoloSegDetector(g("yolo_weights"), g("yolo_conf"))
        else:
            self.detector = P.ColorDetector(g("hsv_lower"), g("hsv_upper"))

        self.cfg = {n: g(n) for n in ("debris_size", "min_pixels", "erode_px", "alpha", "max_lost", "gain",
                                      "max_rate", "deadband_deg", "standoff", "camera_frame", "world_frame",
                                      "publish_debug_image", "fallback_hfov_deg", "fallback_vfov_deg",
                                      "depth_max_age")}
        self.K = None
        self.depth, self.depth_stamp = None, None
        self.p_filt, self.lost = None, 0

        self.tf_buffer = None
        if self.cfg["world_frame"]:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, g("image_topic"), self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g("camera_info_topic"), self.on_info, qos_profile_sensor_data)
        if g("depth_topic"):
            self.create_subscription(Image, g("depth_topic"), self.on_depth, qos_profile_sensor_data)

        self.pub_pos = self.create_publisher(PointStamped, "debris/position", 10)
        self.pub_pos_w = self.create_publisher(PointStamped, "debris/position_world", 10)
        self.pub_det = self.create_publisher(Bool, "debris/detected", 10)
        self.pub_err = self.create_publisher(Vector3Stamped, "camera/centering_error", 10)
        self.pub_twist = self.create_publisher(TwistStamped, "camera/cmd_twist", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "camera/cmd_pose", 10)
        self.pub_dbg = self.create_publisher(Image, "debris/debug_image", 1)

        self.get_logger().info(f"detector={self.detector.name}, image={g('image_topic')}, "
                               f"depth={g('depth_topic') or 'none (range from cube size)'}")

    # ------------------------------------------------------------------
    def on_info(self, msg):
        if msg.k[0] > 0:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_depth(self, msg):
        try:
            self.depth, _ = image_to_numpy(msg)
            self.depth_stamp = Time.from_msg(msg.header.stamp)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)

    def on_image(self, msg):
        try:
            rgb, kind = image_to_numpy(msg)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return
        if kind != "rgb":
            self.get_logger().warn("image_topic carries depth, expected colour", throttle_duration_sec=5.0)
            return

        cfg = self.cfg
        h, w = rgb.shape[:2]
        K = self.K or P.intrinsics_from_fov(w, h, cfg["fallback_hfov_deg"], cfg["fallback_vfov_deg"])
        header = msg.header
        if cfg["camera_frame"]:
            header.frame_id = cfg["camera_frame"]

        depth = None
        if self.depth is not None and self.depth_stamp is not None:
            gap = abs((Time.from_msg(header.stamp) - self.depth_stamp).nanoseconds) * 1e-9
            if gap <= cfg["depth_max_age"]:
                depth = self.depth

        est = P.estimate_debris(self.detector(rgb), K, depth, cfg["debris_size"],
                                cfg["min_pixels"], cfg["erode_px"])

        # Filter in the camera frame. The camera turns while it centres, so keep alpha high
        # (light filtering), or the filter lag fights the controller.
        coasting = False
        if est is not None:
            p = est["p_cam"]
            a = cfg["alpha"]
            self.p_filt = p if self.p_filt is None else a * p + (1 - a) * self.p_filt
            self.lost = 0
        else:
            self.lost += 1
            coasting = self.p_filt is not None and self.lost <= cfg["max_lost"]
            if not coasting:
                self.p_filt = None

        tracked = self.p_filt is not None
        self.pub_det.publish(Bool(data=tracked))
        if tracked:
            self.publish_estimate(header, self.p_filt, K, est)
        else:
            self.publish_stop(header)

        if cfg["publish_debug_image"] and self.pub_dbg.get_subscription_count() > 0:
            status = "DET" if est else ("COAST" if coasting else "LOST")
            self.pub_dbg.publish(self.bgr_to_msg(P.draw_debug(rgb, est, K, status), header))

    # ------------------------------------------------------------------
    def publish_estimate(self, header, p, K, est):
        cfg = self.cfg
        pt = PointStamped(header=header)
        pt.point.x, pt.point.y, pt.point.z = (float(v) for v in p)
        self.pub_pos.publish(pt)

        if self.tf_buffer is not None:
            try:
                from tf2_geometry_msgs import do_transform_point
                from tf2_ros import ExtrapolationException
                # Never block here: TF arrives through this same single-threaded executor, so waiting would
                # only delay it. Use the image's stamp if TF has it yet, otherwise the latest transform.
                try:
                    tf = self.tf_buffer.lookup_transform(cfg["world_frame"], header.frame_id,
                                                         Time.from_msg(header.stamp))
                except ExtrapolationException:
                    tf = self.tf_buffer.lookup_transform(cfg["world_frame"], header.frame_id, Time())
                self.pub_pos_w.publish(do_transform_point(pt, tf))
            except Exception as e:  # TF not available yet: publish camera frame only
                self.get_logger().warn(f"no TF {cfg['world_frame']} <- {header.frame_id}: {e}",
                                       throttle_duration_sec=5.0)

        eh, ev = P.pointing_errors(p)
        err = Vector3Stamped(header=header)
        err.vector.x, err.vector.y = eh, ev
        if est is not None:
            err.vector.z = float(math.hypot(est["uv"][0] - K[2], est["uv"][1] - K[3]))
        else:
            err.vector.z = float("nan")
        self.pub_err.publish(err)

        wx, wy, wz = P.centering_rates(p, cfg["gain"], cfg["max_rate"], math.radians(cfg["deadband_deg"]))
        tw = TwistStamped(header=header)
        tw.twist.angular.x, tw.twist.angular.y, tw.twist.angular.z = wx, wy, wz
        self.pub_twist.publish(tw)

        ps = PoseStamped(header=header)
        sp = P.standoff_point(p, cfg["standoff"])
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in sp)
        q = P.look_at_quaternion(p)
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
        self.pub_pose.publish(ps)

    def publish_stop(self, header):
        self.pub_twist.publish(TwistStamped(header=header))  # all zeros: hold still

    @staticmethod
    def bgr_to_msg(img, header):
        m = Image(header=header, height=img.shape[0], width=img.shape[1], encoding="bgr8",
                  is_bigendian=0, step=img.shape[1] * 3)
        m.data = np.ascontiguousarray(img).tobytes()
        return m


def main():
    rclpy.init()
    node = DebrisTracker()
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
