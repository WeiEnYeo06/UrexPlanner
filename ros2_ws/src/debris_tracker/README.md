# debris_tracker (ROS 2)

Reads camera images, finds the 1U debris, publishes commands that centre the camera on it, and
publishes the debris coordinates for the other team. Pure-Python (`ament_python`), no `cv_bridge`.

## Build and run (Ubuntu / WSL, ROS 2 Humble or Jazzy)
```bash
cd ros2_ws
colcon build --packages-select debris_tracker && source install/setup.bash

# 1) Closed-loop test with no hardware (a fake pan-tilt camera follows our cmd_twist)
ros2 launch debris_tracker tracker.launch.py fake_camera:=true
ros2 run rqt_image_view rqt_image_view /urex/debris/debug_image

# 2) Real D405 (realsense2_camera driver), with depth
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
ros2 launch debris_tracker tracker.launch.py \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw

# 3) Unity twin, through rosbridge (see "Unity twin bridge" below)
ros2 launch debris_tracker twin.launch.py
```
Without ROS: `pytest test/` (needs only numpy + opencv).

## Unity twin bridge
Unity talks to ROS 2 through **rosbridge** (JSON over WebSocket, port 9090). It uses the same protocol as
[ROSBridgeLib](https://github.com/MathiasCiarlo/ROSBridgeLib). That library itself is ROS 1-only (its `Header` has
`seq` and `secs`/`nsecs`), so the Unity side is a small built-in client instead, with no DLLs:
`Codes/unity/Scripts/RosBridgeClient.cs`, `RosMsgs.cs` and `TwinRosBridge.cs`.

### One-time setup (Windows + WSL2)
```powershell
wsl --install -d Ubuntu-24.04          # admin PowerShell, then reboot and create the Linux user
```
```bash
# in Ubuntu: install ROS 2 Jazzy (docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), then
sudo apt install ros-jazzy-rosbridge-suite ros-jazzy-tf2-geometry-msgs ros-jazzy-rqt-image-view python3-opencv
# build in the Linux filesystem (building on /mnt/c inside OneDrive is slow, and OneDrive would sync build/)
mkdir -p ~/ros2_ws/src
ln -s "/mnt/c/Users/Acer/OneDrive/Documents/NUS YEAR 2 SEM1/UREX/Codes/ros2_ws/src/debris_tracker" ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```
In Unity, add the **TwinRosBridge** component to the `Twin` GameObject (next to TwinPlayback).

### Run
```bash
ros2 launch debris_tracker twin.launch.py      # rosbridge + tracker (twin topics, world_frame:=world, green HSV)
```
Then press Play in Unity. The bottom-left box should say "connected". WSL2 forwards `localhost`, so the default
`ws://localhost:9090` works. Then check:
```bash
ros2 topic hz /twin/camera/image_raw           # ~10 Hz (TwinRosBridge.imageRate)
ros2 run rqt_image_view rqt_image_view /urex/debris/debug_image
ros2 topic echo /urex/debris/position_world    # tracker estimate, compare with the ground truth:
ros2 topic echo /twin/debris/pose
```
Turn the D405 towards the debris (M, then F, or the arrow keys). Once the debris is in view, the tracker's
`cmd_twist` turns the camera to centre it. Camera control (box in the Game view): **twist** turns the camera only.
**pose** also moves it to `standoff` from the debris. **off** only observes. Any command switches the twin to its
manual camera, which pauses the mission.

### Topics published by Unity
| Topic | Type | Notes |
|---|---|---|
| `/twin/camera/image_raw` | `sensor_msgs/Image` | rgb8 424 x 240, `frame_id` `d405_color_optical_frame` |
| `/twin/camera/camera_info` | `sensor_msgs/CameraInfo` | exact pinhole intrinsics of the Unity camera |
| `/tf` | `tf2_msgs/TFMessage` | `world -> d405_color_optical_frame`, `world -> debris`, same stamp as the image |
| `/twin/camera/pose` | `geometry_msgs/PoseStamped` | optical frame pose in `world` |
| `/twin/debris/pose` | `geometry_msgs/PoseStamped` | debris **centre** in `world` (ground truth) |

`world` is the REP-103 version of the Unity world: x = Unity z, y = -Unity x, z = Unity y. This is the table frame
of the Python twin. The tracker measures the **visible face** of the cube, so expect its estimate to sit up to
~5 cm (half the cube) closer to the camera than `/twin/debris/pose`. Stamps use the Windows wall clock. Images over
rosbridge are base64 JSON (~400 kB per frame), which is fine at 10 Hz on localhost.

## Topics

### Inputs (from another node)
| Topic (parameter) | Type | Notes |
|---|---|---|
| `image_topic` | `sensor_msgs/Image` | rgb8 / bgr8 / rgba8 / bgra8 / mono8 |
| `camera_info_topic` | `sensor_msgs/CameraInfo` | Until it arrives, the D405 FOV (87 x 58 deg) is used |
| `depth_topic` (optional) | `sensor_msgs/Image` | 16UC1 (mm) or 32FC1 (m), aligned to colour. Without it, the range comes from the cube size (0.1 m) |

All subscriptions use sensor-data QoS (best effort), so they accept both reliable and best-effort publishers.

### Outputs (ours, under `/urex`)
| Topic | Type | Meaning |
|---|---|---|
| **`/urex/debris/position`** | `geometry_msgs/PointStamped` | **Debris position [m] in the camera optical frame** (x right, y down, z forward). `frame_id` = the image's frame. Published every frame while tracked |
| `/urex/debris/position_world` | `geometry_msgs/PointStamped` | Same point in `world_frame`. Only published when `world_frame:=...` is set and TF has the transform |
| `/urex/debris/detected` | `std_msgs/Bool` | True while tracked (including up to `max_lost` frames of coasting) |
| `/urex/camera/centering_error` | `geometry_msgs/Vector3Stamped` | x = horizontal angle error, y = vertical angle error [rad]; z = pixel distance from the image centre |
| `/urex/camera/cmd_twist` | `geometry_msgs/TwistStamped` | Angular velocity (optical frame, rad/s) that turns the camera onto the debris. All zeros when lost |
| `/urex/camera/cmd_pose` | `geometry_msgs/PoseStamped` | Look-at setpoint in the optical frame: a point on the line of sight `standoff` m from the debris, oriented to face it |
| `/urex/debris/debug_image` | `sensor_msgs/Image` | Annotated image (only rendered when someone subscribes) |

The other team should read **`/urex/debris/position`** (or `position_world`). Check them with:
```bash
ros2 topic echo /urex/debris/position
ros2 topic hz /urex/debris/position
```

## How centering works
1. HSV colour mask (or YOLO segmentation), then the largest blob, then the centroid `(u, v)`.
2. Range `Z` = median depth over the eroded mask, or `Z ~ f * 0.1 / sqrt(area)` with no depth.
3. Back-projection: `X = (u-cx) Z/fx`, `Y = (v-cy) Z/fy`.
4. Proportional law: `w_y = k * atan2(X, Z)`, `w_x = -k * atan2(Y, Z)`, clipped to `max_rate`, with a deadband.

Whatever moves the camera (the Unity manual-camera/IK path, or the real myCobot controller) subscribes to
`cmd_twist` (rate control) or `cmd_pose` (position control) and turns the commands into motion.

## Coordinates
- ROS optical frame: x right, y down, z forward (REP-103 `*_optical_frame`).
- Unity camera: x right, y up, z forward. So `ROS optical = (x, -y, z)` of the Unity camera-local point.
- For a world-frame output, publish TF `world -> ... -> d405_color_optical_frame` and set `world_frame:=world`.

## Debris colour
The twin's debris used to be `(0.80, 0.62, 0.20)`, the same hue (H = 21) as the yellow debris arm, so the colour
detector merged the two. It is now `TwinPlayback.debrisColor` = green `(0.20, 0.80, 0.30)` (HSV ~(65, 191, 204)).
`twin.launch.py` sets `hsv_lower: [50, 100, 50]`, `hsv_upper: [80, 255, 255]` to match. `tracker.yaml` still holds
the orange range for the **real** cube; set it to the real cube's colour (or use `detector:=yolo`).
