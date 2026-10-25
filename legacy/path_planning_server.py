#!/usr/bin/env python3
"""
path_planning_server.py
Python side of the Path Planning digital-twin loop (UREX proximity operations).

Receives RGB-D frames from Unity (PathPlanningTwin.cs) and returns a camera setpoint that
keeps the debris in view (line of sight, LOS) at a fixed standoff distance.

Pipeline per frame:
  1. Detect the debris -> binary mask        (gt | color | yolo)
  2. Mask centroid     -> pixel (u, v)
  3. Range             -> median depth over the (slightly eroded) mask
  4. Back-project      -> debris position in the camera optical frame
                          X = (u - cx) Z / fx,  Y = (v - cy) Z / fy
  5. Setpoint          -> move along the line of sight to `standoff` metres from the debris,
                          looking at the debris

Camera optical frame: x right, y down, z forward (OpenCV convention), metres.
The estimate is low-pass filtered in WORLD coordinates (the camera moves, so filtering in
camera coordinates would mix in the camera's own motion).

Note: the estimate is the visible SURFACE point of the debris, not its centre, so the standoff
is measured to the surface.

Usage:
  pip install numpy opencv-python          (add `ultralytics` for --detector yolo)
  python path_planning_server.py --detector gt --standoff 0.30
  then press Play in Unity.

Detectors:
  gt     Unity's ground-truth mask. Use it to validate geometry and control, not as your result.
  color  HSV threshold. Give the debris a distinctive material colour and tune --hsv-lower/upper.
  yolo   Ultralytics YOLO segmentation model (--weights path/to/best.pt), e.g. trained on Unity renders.
"""

import argparse
import csv
import json
import socket
import struct
import time

import numpy as np
import cv2


# ----------------------------------------------------------------------------
# Networking: [4-byte big-endian length][JSON header][rgb][depth float32 LE][mask]
def recv_exact(sock, n):
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        r = sock.recv_into(view[got:], n - got)
        if r == 0:
            raise ConnectionError("Unity closed the connection")
        got += r
    return bytes(buf)


def recv_frame(sock):
    (hlen,) = struct.unpack(">I", recv_exact(sock, 4))
    header = json.loads(recv_exact(sock, hlen).decode("utf-8"))
    w, h = header["width"], header["height"]
    rgb = np.frombuffer(recv_exact(sock, header["rgbBytes"]), np.uint8).reshape(h, w, 3)
    depth = np.frombuffer(recv_exact(sock, header["depthBytes"]), "<f4").reshape(h, w)
    mask = np.frombuffer(recv_exact(sock, header["maskBytes"]), np.uint8).reshape(h, w)
    return header, rgb, depth, mask


def send_json(sock, obj):
    data = json.dumps(obj).encode("utf-8")
    sock.sendall(struct.pack(">I", len(data)) + data)


# ----------------------------------------------------------------------------
# Detectors: each returns a boolean mask (H, W) or None
class GroundTruthDetector:
    name = "gt"

    def __call__(self, rgb, gt_mask):
        return gt_mask.astype(bool)


class ColorDetector:
    name = "color"

    def __init__(self, lower, upper):
        self.lower = np.array(lower, np.uint8)  # OpenCV HSV: H 0-179, S 0-255, V 0-255
        self.upper = np.array(upper, np.uint8)
        self.kernel = np.ones((3, 3), np.uint8)

    def __call__(self, rgb, gt_mask):
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        m = cv2.inRange(hsv, self.lower, self.upper)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self.kernel)
        return m > 0


class YoloSegDetector:
    name = "yolo"

    def __init__(self, weights, conf):
        from ultralytics import YOLO  # imported here so other modes don't need it
        self.model = YOLO(weights)
        self.conf = conf

    def __call__(self, rgb, gt_mask):
        res = self.model(rgb[..., ::-1].copy(), conf=self.conf, verbose=False)[0]  # model expects BGR
        if res.masks is None or len(res.masks) == 0:
            return None
        best = int(res.boxes.conf.argmax())
        poly = res.masks.xy[best]  # polygon in original image pixels
        if len(poly) < 3:
            return None
        m = np.zeros(rgb.shape[:2], np.uint8)
        cv2.fillPoly(m, [poly.astype(np.int32)], 1)
        return m > 0


# ----------------------------------------------------------------------------
# Perception: mask + depth -> 3D debris position in the camera optical frame
def largest_component(mask):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    if n <= 1:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels == k


def estimate_debris(mask, depth, K, min_pixels=30, erode_px=2):
    """Returns (p_cam [3], (u, v), mask_used) or None."""
    if mask is None:
        return None
    mask = largest_component(mask)
    if mask is None or mask.sum() < min_pixels:
        return None

    vs, us = np.nonzero(mask)
    u, v = float(us.mean()), float(vs.mean())  # centroid

    # Erode before taking depth: edge pixels mix debris and background depth on real sensors.
    core = mask
    if erode_px > 0:
        k = np.ones((2 * erode_px + 1, 2 * erode_px + 1), np.uint8)
        eroded = cv2.erode(mask.astype(np.uint8), k).astype(bool)
        if eroded.sum() >= min_pixels:
            core = eroded

    d = depth[core]
    d = d[np.isfinite(d) & (d > 0)]
    if d.size < min_pixels:
        return None
    Z = float(np.median(d))

    fx, fy, cx, cy = K
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return np.array([X, Y, Z]), (u, v), mask


# ----------------------------------------------------------------------------
# Planner: filtered debris estimate -> camera setpoint
class StandoffPlanner:
    """
    Keeps a low-pass-filtered debris position in world coordinates
    (first-order IIR: p_f[k] = a p[k] + (1 - a) p_f[k-1]).
    If detection drops out, it coasts on the last estimate for up to max_lost frames,
    then asks Unity to hold still.
    """

    def __init__(self, standoff, alpha, max_lost):
        self.standoff = standoff
        self.alpha = alpha
        self.max_lost = max_lost
        self.p_world = None
        self.lost = 0

    def update(self, p_cam_meas, T_cam_to_world):
        T_world_to_cam = np.linalg.inv(T_cam_to_world)
        coasting = False
        if p_cam_meas is not None:
            p_w = (T_cam_to_world @ np.append(p_cam_meas, 1.0))[:3]
            self.p_world = p_w if self.p_world is None else self.alpha * p_w + (1 - self.alpha) * self.p_world
            self.lost = 0
        else:
            self.lost += 1
            if self.p_world is None or self.lost > self.max_lost:
                return None, False
            coasting = True

        p = (T_world_to_cam @ np.append(self.p_world, 1.0))[:3]  # debris in current camera frame
        r = np.linalg.norm(p)
        if r < 1e-6:
            return None, coasting
        los = p / r
        position = p - self.standoff * los  # point on the line of sight, `standoff` from the debris
        return {"position": position, "lookAt": p}, coasting


# ----------------------------------------------------------------------------
def angle_deg(a, b):
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return float("nan")
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))))


LOG_FIELDS = [
    "frame", "t", "detected", "coasting", "in_view_gt", "mask_px", "u", "v", "pix_offset",
    "est_x", "est_y", "est_z", "est_range", "true_x", "true_y", "true_z", "true_range",
    "bearing_err_deg", "standoff_err", "prev_clamped", "proc_ms",
]


def run(args):
    if args.detector == "gt":
        detector = GroundTruthDetector()
    elif args.detector == "color":
        detector = ColorDetector(args.hsv_lower, args.hsv_upper)
    else:
        if not args.weights:
            raise SystemExit("--detector yolo needs --weights path/to/best.pt")
        detector = YoloSegDetector(args.weights, args.conf)

    planner = StandoffPlanner(args.standoff, args.alpha, args.max_lost)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    print(f"[planner] detector={detector.name} standoff={args.standoff} m, "
          f"waiting for Unity on {args.host}:{args.port} ...")
    conn, addr = srv.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"[planner] Unity connected from {addr}")

    log_file = open(args.log, "w", newline="")
    log = csv.DictWriter(log_file, fieldnames=LOG_FIELDS)
    log.writeheader()
    rows = []

    try:
        while True:
            try:
                header, rgb, depth, gt_mask = recv_frame(conn)
            except ConnectionError:
                break

            t0 = time.perf_counter()
            K = (header["fx"], header["fy"], header["cx"], header["cy"])
            T = np.array(header["camToWorld"], dtype=float).reshape(4, 4)

            est = estimate_debris(detector(rgb, gt_mask), depth, K, args.min_pixels, args.erode)
            p_meas = est[0] if est else None
            sp, coasting = planner.update(p_meas, T)

            if sp is None:
                reply = {"valid": False, "stop": False}
            else:
                reply = {
                    "valid": True,
                    "stop": False,
                    "position": [float(x) for x in sp["position"]],
                    "lookAt": [float(x) for x in sp["lookAt"]],
                }
            proc_ms = (time.perf_counter() - t0) * 1000.0
            send_json(conn, reply)

            # ---------- metrics ----------
            true_p = np.array(header["debrisCentreCam"], dtype=float)
            row = {k: "" for k in LOG_FIELDS}
            row.update(
                frame=header["frame"], t=round(header["t"], 3),
                detected=int(est is not None), coasting=int(coasting),
                in_view_gt=int(gt_mask.any()), true_x=true_p[0], true_y=true_p[1], true_z=true_p[2],
                true_range=float(np.linalg.norm(true_p)),
                prev_clamped=int(header.get("lastSetpointClamped", False)), proc_ms=round(proc_ms, 3),
            )
            if est is not None:
                p, (u, v), m = est
                r = float(np.linalg.norm(p))
                row.update(
                    mask_px=int(m.sum()), u=round(u, 2), v=round(v, 2),
                    pix_offset=round(float(np.hypot(u - K[2], v - K[3])), 2),
                    est_x=p[0], est_y=p[1], est_z=p[2], est_range=r,
                    bearing_err_deg=angle_deg(p, true_p), standoff_err=r - args.standoff,
                )
            log.writerow(row)
            rows.append(row)

            if args.show:
                show_frame(rgb, est, K)

            if header["frame"] % args.print_every == 0:
                status = "DET" if est else ("COAST" if coasting else "LOST")
                rng = f"{row['est_range']:.3f}" if est else "  -  "
                print(f"[planner] frame {header['frame']:5d} t={header['t']:7.1f}s {status:5s} "
                      f"range={rng} m  true={row['true_range']:.3f} m  {proc_ms:.1f} ms")
    finally:
        log_file.close()
        conn.close()
        srv.close()
        if args.show:
            cv2.destroyAllWindows()

    summarise(rows, args)


def show_frame(rgb, est, K):
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    cv2.drawMarker(img, (int(round(K[2])), int(round(K[3]))), (255, 255, 255), cv2.MARKER_CROSS, 12, 1)
    if est is not None:
        p, (u, v), m = est
        overlay = img.copy()
        overlay[m] = (0, 255, 0)
        img = cv2.addWeighted(overlay, 0.35, img, 0.65, 0)
        cv2.circle(img, (int(round(u)), int(round(v))), 4, (0, 0, 255), -1)
        cv2.putText(img, f"Z={p[2]:.3f} m", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.imshow("planner", cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
    cv2.waitKey(1)


def summarise(rows, args):
    if not rows:
        print("[planner] no frames received")
        return

    def col(name, only_detected=False):
        vals = [r[name] for r in rows if r[name] != "" and (not only_detected or r["detected"])]
        return np.array(vals, dtype=float)

    det, view, clamp, proc = col("detected"), col("in_view_gt"), col("prev_clamped"), col("proc_ms")
    pix, bear, serr = col("pix_offset", True), col("bearing_err_deg", True), col("standoff_err", True)
    print("\n========== run summary ==========")
    print(f"frames                     : {len(rows)}")
    print(f"debris in view (gt LOS)    : {100 * view.mean():.1f} %")
    print(f"detected                   : {100 * det.mean():.1f} %")
    print(f"setpoints clamped by reach : {100 * clamp.mean():.1f} %   (feasibility)")
    print(f"processing time            : mean {proc.mean():.2f} ms, max {proc.max():.2f} ms   (planning time)")
    if pix.size:
        print(f"centroid offset from centre: mean {pix.mean():.1f} px, max {pix.max():.1f} px   (viewpoint quality)")
        print(f"bearing error vs truth     : mean {np.nanmean(bear):.2f} deg")
        print(f"perceived standoff error   : mean |e| {np.abs(serr).mean() * 100:.1f} cm, "
              f"final {serr[-1] * 100:+.1f} cm")
    print(f"log written to {args.log}")


def main():
    ap = argparse.ArgumentParser(description="Path Planning server for the Unity digital twin")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--detector", choices=["gt", "color", "yolo"], default="gt")
    ap.add_argument("--hsv-lower", type=int, nargs=3, default=[5, 120, 80], metavar=("H", "S", "V"))
    ap.add_argument("--hsv-upper", type=int, nargs=3, default=[25, 255, 255], metavar=("H", "S", "V"))
    ap.add_argument("--weights", default="", help="YOLO segmentation weights (.pt)")
    ap.add_argument("--conf", type=float, default=0.4, help="YOLO confidence threshold")
    ap.add_argument("--standoff", type=float, default=0.30, help="desired camera-to-debris distance [m]")
    ap.add_argument("--alpha", type=float, default=0.5, help="low-pass gain, 1 = no filtering")
    ap.add_argument("--max-lost", type=int, default=10, help="frames to coast on the last estimate")
    ap.add_argument("--min-pixels", type=int, default=30)
    ap.add_argument("--erode", type=int, default=2, help="mask erosion (px) before median depth")
    ap.add_argument("--log", default="planner_metrics.csv")
    ap.add_argument("--print-every", type=int, default=50)
    ap.add_argument("--show", action="store_true", help="show the camera image with the mask overlay")
    run(ap.parse_args())


if __name__ == "__main__":
    main()
