# Unity digital twin: what it does (handover for adding ROS 2)

## Project
- Unity 6 (6000.5.8f1), URP, project at `C:\Users\Acer\UREX_TwinPlayback`, scene `Assets/Scenes/SampleScene.unity`.
- Source copies of the scripts are kept in `UREX/Codes/unity/Scripts/` and copied into `Assets/Scripts/`.
- Input handling is set to "Both" (the legacy `Input` class is used).
- No ROS packages are installed yet (no ROS-TCP-Connector, no URDF Importer in `Packages/manifest.json`).

## What the scene is
A tabletop rig used to test camera approach and inspection for space-robotics proximity operations:
- **Blue arm (chaser B601-RS):** 6-DOF, 7 frames. Its base rides on a **linear rail**.
- **White arm (myCobot 280):** mounted on the blue arm's flange through a bracket. It carries the **D405 camera** (Intel RealSense, 87 x 58 deg FOV, near 0.05 m, far 3 m, 42 x 42 x 23 mm).
- **Yellow arm (debris B601-RS):** held still in an **L pose** (straight up to the elbow, then horizontal towards the blue arm). It holds a **1U debris cube** (0.1 m) on a stand-off at its tip, at about (-0.07, 0.39, 0.40) m.
- **Table:** 2.0 x 0.74 m.

The arms are drawn as "stick models": spheres at the joint frames and cylinders for the links. They are not URDF meshes.

## Scripts
| File | Role |
|---|---|
| `TwinPlayback.cs` | Main component on the `Twin` GameObject. Builds everything at `Start()`, plays back the mission, handles the rail, manual camera, GUI and D405 render. |
| `TwinData.cs` | `SceneInfo` (JSON loader) and `CsvTable` (CSV loader with time interpolation: `Value(t,col)`, `Vec(t,col)`). |
| `OrbitCamera.cs` | Viewer camera: right-drag to orbit, middle-drag to pan, scroll to zoom. |
| `UrdfJointDriver.cs` | Optional. Drives URDF-imported robots (ArticulationBody) from `joint_trajectory.csv`. Not used in the scene yet, and it does **not** know about the rail, the fixed yellow arm or the manual camera. |

## Data (in `Assets/StreamingAssets/`, produced offline by a Python twin)
- `unity_scene.json`: table size, base positions (chaser (0.2,0,-0.58), debris (-0.2,0,0.88)), camera intrinsics, `hold_distance` 0.2 m, frame counts (chaser 7, debris 7, mycobot 8), URDF joint and link names.
- `unity_playback.csv`: 2716 rows, t = 176.9 to 719.9 s. Columns: `t, phase` (0 = APPROACH, 1 = HOLD from t ≈ 600.1), `rail`, `{chaser,debris,mycobot}{i}_{x,y,z}` (frame positions), `cam_pos/fwd/up_*`, `debris_pos/fwd/up_*`, `mycobotbase_*`, `cam_dist, d_cmd, los_deg`.
- `joint_trajectory.csv`: joint angles (deg) per robot over time, plus `rail_shift_m`.

## Coordinates and units
- Metres, Unity left-handed, **y up**. Origin is the centre of the table top.
- Mapping from the Python/table frame: Unity x = -table y, Unity y = up, Unity z = table x.
- **ROS uses right-handed, z up (REP-103)**, so every published pose needs a conversion (ROS-TCP-Connector's `To<FLU>()` / `From<FLU>()` helpers, or an explicit swap).

## Runtime behaviour (what TwinPlayback does each frame)
1. **Mission clock.** `MissionTime` advances by `deltaTime * speed` (default x10; the GUI slider goes from 1 to 60) while `playing`. It loops if `loop` is on. The GUI has Play/Pause, Restart and a time scrub slider.
2. **Rail** (`railEnabled`, `railDoesApproach` = true):
   - The blue arm, myCobot and camera are frozen at their **starting pose** (sampled at t = StartTime).
   - A rail offset moves them in a straight horizontal line from the camera's start position towards the debris (direction about (0.008, 0, 1), i.e. along the table's length).
   - The stroke is 0.467 m, eased with smoothstep over the APPROACH phase, then held. At the end the camera is **0.200 m** from the debris with about 9 deg pointing error, and the bases are 1.07 m apart.
   - Settings: `railStopDistance` (0 means use `hold_distance`) and `railMinClearance` (0.45 m).
   - The public API exposes the rail position as `RailOffset(t)` (private method) and `railTravel`.
3. **Yellow arm** (`debrisArmFixedPose`, `debrisArmBendFrame` = 3): its fixed frame positions are computed once from its real link lengths. The debris pose comes from this fixed pose, not from the CSV.
4. **Manual camera** (key **M** or the GUI toggle `manualCamera`):
   - Turning it on pauses the mission and starts from the current camera pose.
   - Keys: WASD / Q E move, arrows yaw/pitch, Z/C roll, Shift x3, F look at the debris, R reset.
   - The **myCobot follows by FABRIK IK** (base fixed on the blue flange, link lengths kept, last link rigid to the camera). If the target is out of reach, the camera is pulled back.
   - No joint limits and no collision checking.
5. **Outputs computed every frame:**
   - Camera pose: GameObject `D405 view` (child of `D405`).
   - Debris pose: `Debris (1U replica)`.
   - Joint frame positions: `chaser frame i`, `debris frame i`, `mycobot frame i`.
   - Live camera-to-debris distance and pointing error (`liveDist`, `liveLos`, private).
   - Line of sight drawn camera → debris.
6. **D405 camera:** a Unity `Camera` renders into a 424 x 240 `RenderTexture` (`d405Tex`, private), shown picture-in-picture at the top right with a crosshair. There is no depth output yet.

## Path missions
`TwinPlayback.mission` selects the scenario. Switch with keys 1 / 2 / 3, the Mission toolbar in the GUI, or the Inspector.
- **1 TableArm:** the original. Yellow arm on the table in its L pose; the debris is still.
- **2 CeilingArm:** the yellow arm hangs upside down from an overhead gantry. The beam is at `ceilingHeight`
  (0.80 m), and its legs stand on the floor beside the table. The debris is still, at about mission 1's height.
- **3 OrbitingDebris:** the yellow arm is on the table. The debris turns on a spoke in a vertical circle facing the
  chaser (`orbitRadius` 0.15 m, `orbitPeriod` 20 s). The orbit runs on real time, so it keeps going when the mission
  is paused.

Switching missions calls `ApplyMission()`, which rebuilds the debris-arm pose, the base plate, the gantry and the
rail stroke, then restarts the approach.

## ROS 2 bridge (implemented)
`TwinRosBridge.cs` (add it to the `Twin` GameObject), `RosBridgeClient.cs` and `RosMsgs.cs` connect the twin to ROS 2
through rosbridge (`ws://localhost:9090`). Unity publishes the D405 image and camera info, TF, and the camera and
debris poses, and follows `/urex/camera/cmd_twist` or `cmd_pose` from `debris_tracker`. TwinPlayback now has public
accessors and `SetManualPose`, and a `debrisColor` field (green by default, so it doesn't clash with the yellow arm).
How to run it, and the topic list: `Codes/ros2_ws/src/debris_tracker/README.md`, section "Unity twin bridge".
Not done yet: depth, joint states, rail and mission commands.

## Original notes on ROS 2 hooks (written before the bridge)
Likely **publishers** (Unity → ROS):
- `/twin/camera/image_raw` (`sensor_msgs/Image`) and `/twin/camera/camera_info` from the D405 RenderTexture. Depth would need a depth texture or a shader.
- `/twin/camera/pose` and `/twin/debris/pose` (`geometry_msgs/PoseStamped`), or TF frames `world → rail → chaser_base → … → d405_optical`, plus `debris`.
- `/twin/rail/position` (`std_msgs/Float64`, metres), `/twin/mission/phase`, `/twin/metrics` (distance, pointing error).
- `/twin/joint_states` (`sensor_msgs/JointState`). Note: the stick models are positions only, so joint angles exist only in `joint_trajectory.csv` (recorded, not live). The IK and rail do not produce angles.

Likely **subscribers** (ROS → Unity):
- A camera pose setpoint (e.g. `/twin/camera/cmd_pose`, `PoseStamped`) feeding the manual-camera path. Setting `manualPos` / `manualRot` and enabling `manualCamera` gives IK-followed motion. This needs a public setter, because those fields are currently private.
- A rail position command (`Float64`) to replace the scripted smoothstep.
- Play / pause / time scrub.

To do this, private state (`manualPos`, `manualRot`, `liveDist`, `liveLos`, `d405Tex`, `RailOffset`, `railTravel`) needs public accessors. The usual stack is the **Unity ROS-TCP-Connector** package plus `ros_tcp_endpoint` on the ROS 2 side.

The related project file `Codes/PathPlanningTwin.cs` together with `Codes/path_planning_server.py` is a separate path-planning twin. It sends camera images or depth to a Python server and applies the returned camera setpoint with a reach-limited "stand-in arm". Its interface may be the model for which ROS topics are needed.
