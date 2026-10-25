# UrexPlanner

UREX (NUS) space-robotics proximity operations. A chaser arm carries an Intel RealSense D405 on a myCobot 280,
approaches a 1U debris cube held by a second arm, detects it, and keeps the camera centred on it.
This repo is the **Unity digital twin** (Unity 6000.5.8f1, URP) and the **ROS 2 (Jazzy) debris tracker**. They run
as one closed loop through rosbridge.

**Progress, results and full setup: [checkpt1.md](checkpt1.md).**

## Layout
| Path | What |
|---|---|
| `Assets/Scenes/TwinScene.unity` | **The twin scene: open it and press Play** (`SampleScene` is the blank template) |
| `Assets/Scripts/` | `TwinPlayback` (scene, 3 path missions, manual camera), `TwinRosBridge` + `RosBridgeClient` + `RosMsgs` (ROS 2 bridge), `TwinData`, `OrbitCamera`, `UrdfJointDriver` |
| `Assets/StreamingAssets/` | Mission data played back by the twin |
| `ros2_ws/src/debris_tracker/` | ROS 2 package: detection, camera centering, fake camera, accuracy check (`eval_node`). [README](ros2_ws/src/debris_tracker/README.md) |
| `docs/UNITY_TWIN_SUMMARY.md` | How the Unity twin works |
| `legacy/` | Earlier socket-based path-planning loop (not ROS) |
| `trajectory_orbit_lvlh.csv` | Orbit trajectory (LVLH) from the offline Python twin |

## Quick start
1. **ROS 2** (Ubuntu / WSL, Jazzy, `ros-jazzy-rosbridge-suite`):
   ```bash
   ln -s /path/to/UrexPlanner/ros2_ws/src/debris_tracker ~/ros2_ws/src/
   cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
   ros2 launch debris_tracker twin.launch.py
   ```
2. **Unity:** open this folder in Unity 6000.5.8f1, open `Assets/Scenes/TwinScene.unity`, then press Play.
   - The bottom-left box should say "ROS connected".
   - Keys **1 / 2 / 3** choose the mission (table / ceiling / orbiting debris).
   - Press **M**, then **F**, to point the camera at the debris; the tracker then takes over.
3. **Watch it:** `ros2 run rqt_image_view rqt_image_view /urex/debris/debug_image`.
   **Check accuracy:** `ros2 run debris_tracker eval_node`.
