"""ROS-free tests: pytest test/  (only numpy and opencv needed)."""

import math
import os
import sys

import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from debris_tracker import perception as P  # noqa: E402

W, H = 424, 240
K = P.intrinsics_from_fov(W, H, 87.0, 58.0)
DEBRIS_RGB = (204, 158, 51)   # Unity debris colour (0.80, 0.62, 0.20)


def render(p_cam, size=0.1):
    fx, fy, cx, cy = K
    img = np.full((H, W, 3), (20, 20, 30), np.uint8)
    u, v = fx * p_cam[0] / p_cam[2] + cx, fy * p_cam[1] / p_cam[2] + cy
    half = 0.5 * fx * size / p_cam[2]
    cv2.rectangle(img, (int(u - half), int(v - half)), (int(u + half), int(v + half)), DEBRIS_RGB, -1)
    return img


def test_color_detector_and_size_range():
    p_true = np.array([0.05, -0.03, 0.40])
    est = P.estimate_debris(P.ColorDetector([15, 120, 80], [30, 255, 255])(render(p_true)), K)
    assert est is not None and est["range_source"] == "size"
    assert np.allclose(est["p_cam"], p_true, atol=0.02)


def test_depth_range():
    p_true = np.array([-0.04, 0.02, 0.25])
    depth = np.full((H, W), 2.0, np.float32)
    mask = P.ColorDetector([15, 120, 80], [30, 255, 255])(render(p_true))
    depth[mask] = 0.25
    est = P.estimate_debris(mask, K, depth)
    assert est["range_source"] == "depth"
    assert np.allclose(est["p_cam"], p_true, atol=0.01)


def test_rate_signs():
    wx, wy, _ = P.centering_rates([0.1, 0.0, 1.0], 1.0, 10.0)   # debris to the right -> turn right
    assert wy > 0 and wx == 0
    wx, wy, _ = P.centering_rates([0.0, -0.1, 1.0], 1.0, 10.0)  # debris above -> pitch up (+x)
    assert wx > 0 and wy == 0


def test_look_at_quaternion():
    p = np.array([0.1, -0.05, 0.3])
    x, y, z, w = P.look_at_quaternion(p)
    R, _ = cv2.Rodrigues(np.array([x, y, z]) / math.sin(math.acos(w)) * 2 * math.acos(w))
    assert np.allclose(R @ [0, 0, 1], p / np.linalg.norm(p), atol=1e-6)


def test_closed_loop_centres_debris():
    """Same model as fake_camera_node: pan-tilt camera integrating the tracker's rate command."""
    det = P.ColorDetector([15, 120, 80], [30, 255, 255])
    p_world, R, dt = np.array([0.12, -0.06, 0.30]), np.eye(3), 1 / 30
    for _ in range(150):   # 5 s
        p = R.T @ p_world
        est = P.estimate_debris(det(render(p)), K)
        assert est is not None
        w = np.array(P.centering_rates(est["p_cam"], 1.5, 0.5, math.radians(0.5)))
        R = R @ cv2.Rodrigues(w * dt)[0]
    eh, ev = P.pointing_errors(R.T @ p_world)
    assert math.degrees(math.hypot(eh, ev)) < 1.5
