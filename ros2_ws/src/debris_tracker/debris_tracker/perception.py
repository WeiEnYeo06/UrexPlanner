"""
perception.py
ROS-free debris perception and camera-centering maths (so it can be tested without ROS).

Pipeline per frame (same as path_planning_server.py):
  1. Detect the debris -> binary mask        (color | yolo)
  2. Mask centroid     -> pixel (u, v)
  3. Range             -> median depth over the eroded mask, or, with no depth image,
                          from the known cube size:  Z ~ f * size / sqrt(mask area)
  4. Back-project      -> debris position in the camera optical frame
                          X = (u - cx) Z / fx,  Y = (v - cy) Z / fy
  5. Centering         -> angular-rate command that turns the optical axis onto the debris,
                          plus a look-at pose setpoint at `standoff` metres

Camera optical frame (REP-103 "_optical_frame"): x right, y down, z forward, metres.
"""

import math

import numpy as np
import cv2


# ----------------------------------------------------------------------------
# Detectors: each returns a boolean mask (H, W) or None. Input image is RGB uint8.
class ColorDetector:
    name = "color"

    def __init__(self, lower, upper):
        self.lower = np.array(lower, np.uint8)  # OpenCV HSV: H 0-179, S 0-255, V 0-255
        self.upper = np.array(upper, np.uint8)
        self.kernel = np.ones((3, 3), np.uint8)

    def __call__(self, rgb):
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        m = cv2.inRange(hsv, self.lower, self.upper)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self.kernel)
        return m > 0


class YoloSegDetector:
    name = "yolo"

    def __init__(self, weights, conf):
        from ultralytics import YOLO  # imported here so the color mode doesn't need it
        self.model = YOLO(weights)
        self.conf = conf

    def __call__(self, rgb):
        res = self.model(rgb[..., ::-1].copy(), conf=self.conf, verbose=False)[0]  # model expects BGR
        if res.masks is None or len(res.masks) == 0:
            return None
        best = int(res.boxes.conf.argmax())
        poly = res.masks.xy[best]
        if len(poly) < 3:
            return None
        m = np.zeros(rgb.shape[:2], np.uint8)
        cv2.fillPoly(m, [poly.astype(np.int32)], 1)
        return m > 0


# ----------------------------------------------------------------------------
def largest_component(mask):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    if n <= 1:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels == k


def estimate_debris(mask, K, depth=None, debris_size=0.1, min_pixels=30, erode_px=2):
    """
    Returns a dict {p_cam [3], uv (u, v), mask, range_source} or None.
    K = (fx, fy, cx, cy). depth (H, W) in metres, or None to range from the cube size.
    """
    if mask is None:
        return None
    mask = largest_component(mask)
    if mask is None or mask.sum() < min_pixels:
        return None

    vs, us = np.nonzero(mask)
    u, v = float(us.mean()), float(vs.mean())
    fx, fy, cx, cy = K

    Z, source = None, "size"
    if depth is not None and depth.shape == mask.shape:
        # Erode before taking depth: edge pixels mix debris and background depth on real sensors.
        core = mask
        if erode_px > 0:
            k = np.ones((2 * erode_px + 1, 2 * erode_px + 1), np.uint8)
            eroded = cv2.erode(mask.astype(np.uint8), k).astype(bool)
            if eroded.sum() >= min_pixels:
                core = eroded
        d = depth[core]
        d = d[np.isfinite(d) & (d > 0)]
        if d.size >= min_pixels:
            Z, source = float(np.median(d)), "depth"
    if Z is None:
        # Pinhole: a face of side s at range Z covers about (f s / Z)^2 pixels.
        # Seen at an angle the silhouette is bigger, so this under-estimates the range a little.
        Z = math.sqrt(fx * fy) * debris_size / math.sqrt(float(mask.sum()))

    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return {"p_cam": np.array([X, Y, Z]), "uv": (u, v), "mask": mask, "range_source": source}


# ----------------------------------------------------------------------------
def pointing_errors(p_cam):
    """Angles (rad) from the optical axis to the debris: (horizontal, vertical). +x right, +y down."""
    X, Y, Z = p_cam
    return math.atan2(X, Z), math.atan2(Y, Z)


def centering_rates(p_cam, gain, max_rate, deadband_rad=0.0):
    """
    Proportional visual-servo law for the camera's angular velocity in the optical frame.
    Rotating about +y (down) turns the view to the right; rotating about +x turns it up.
    So  w_y = +k * err_h  and  w_x = -k * err_v.  Returns (wx, wy, wz).
    """
    eh, ev = pointing_errors(p_cam)
    if math.hypot(eh, ev) < deadband_rad:
        return 0.0, 0.0, 0.0
    wx = float(np.clip(-gain * ev, -max_rate, max_rate))
    wy = float(np.clip(gain * eh, -max_rate, max_rate))
    return wx, wy, 0.0


def look_at_quaternion(p_cam):
    """Smallest rotation (x, y, z, w) taking the optical axis (+z) onto the debris direction."""
    d = np.asarray(p_cam, float)
    n = np.linalg.norm(d)
    if n < 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    d = d / n
    axis = np.cross([0.0, 0.0, 1.0], d)
    s = np.linalg.norm(axis)
    c = float(np.clip(d[2], -1.0, 1.0))
    if s < 1e-9:
        return (0.0, 0.0, 0.0, 1.0) if c > 0 else (1.0, 0.0, 0.0, 0.0)
    axis = axis / s
    half = math.atan2(s, c) / 2.0
    return (*(axis * math.sin(half)).tolist(), math.cos(half))


def standoff_point(p_cam, standoff):
    """Point on the line of sight `standoff` metres in front of the debris (camera frame)."""
    p = np.asarray(p_cam, float)
    r = np.linalg.norm(p)
    if r < 1e-9:
        return p.copy()
    return p - standoff * p / r


def intrinsics_from_fov(width, height, hfov_deg, vfov_deg):
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    fy = (height / 2.0) / math.tan(math.radians(vfov_deg) / 2.0)
    return fx, fy, width / 2.0, height / 2.0


def draw_debug(rgb, est, K, status):
    """Annotated BGR image: crosshair at the principal point, mask overlay, centroid, range."""
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    c = (int(round(K[2])), int(round(K[3])))
    cv2.drawMarker(img, c, (255, 255, 255), cv2.MARKER_CROSS, 14, 1)
    if est is not None:
        overlay = img.copy()
        overlay[est["mask"]] = (0, 255, 0)
        img = cv2.addWeighted(overlay, 0.35, img, 0.65, 0)
        uv = (int(round(est["uv"][0])), int(round(est["uv"][1])))
        cv2.line(img, c, uv, (0, 255, 255), 1)
        cv2.circle(img, uv, 4, (0, 0, 255), -1)
        p = est["p_cam"]
        cv2.putText(img, f"Z={p[2]:.3f} m ({est['range_source']})", (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(img, status, (5, img.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return img
