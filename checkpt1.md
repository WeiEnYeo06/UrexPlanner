# UREX: checkpoint 1 (updated 25 Oct 2026)

Progress summary for the space-robotics proximity operations project. A chaser arm carries a D405 camera on a
myCobot, approaches a 1U debris cube held by a second arm, and keeps it in view.

**Status: the Unity twin and ROS 2 now run as one closed loop.** Unity renders the D405 view and ROS 2 detects the
debris. ROS 2 then sends camera commands back to Unity, which turns the camera to centre the debris.
Checked against Unity's ground truth, the published debris position is accurate to **0.3 cm** and **0.22°**
(centred view).

> **Paths in this repo.** This document was written in the original working folder, so it uses those paths. In this
> repo they map to:
>
> | In this document | In this repo |
> |---|---|
> | `Codes/unity/Scripts/` | `Assets/Scripts/` |
> | `Codes/unity/StreamingAssets/` | `Assets/StreamingAssets/` |
> | `Codes/ros2_ws/src/debris_tracker/` | `ros2_ws/src/debris_tracker/` |
> | `Codes/unity/UNITY_TWIN_SUMMARY.md` | `docs/UNITY_TWIN_SUMMARY.md` |
> | `Codes/path_planning_server.py`, `Codes/PathPlanningTwin.cs` | `legacy/` |
>
> The twin's scene is `Assets/Scenes/TwinScene.unity`. It already has TwinPlayback and TwinRosBridge on `Twin`, so
> step A7 is done.

---

## 1. What exists

### Offline Python twin (earlier work)
- Mission data: `trajectory_orbit_lvlh.csv`, `Codes/unity/StreamingAssets/` (`unity_playback.csv`,
  `joint_trajectory.csv`, `unity_scene.json`).

### Unity digital twin: `C:\Users\Acer\UREX_TwinPlayback` (Unity 6000.5.8f1, URP)
- Source copies are kept in `Codes/unity/Scripts/` and copied into `Assets/Scripts/`.
- `TwinPlayback.cs`:
  - stick models of the blue chaser arm, the yellow debris arm and the myCobot
  - a linear rail that does the approach (stops 0.2 m from the debris)
  - a manual camera (key M) that the myCobot follows by FABRIK IK
  - a 424 x 240 D405 render
- Overview of the twin: `Codes/unity/UNITY_TWIN_SUMMARY.md`.

### Earlier path-planning loop (not ROS)
- `Codes/path_planning_server.py` + `Codes/PathPlanningTwin.cs`: raw TCP socket. Its detect, range and setpoint
  pipeline was reused in the ROS package.

### ROS 2 package `debris_tracker`: `Codes/ros2_ws/src/debris_tracker`
- `perception.py`, plain maths with no ROS needed. Per frame:
  - HSV colour (or YOLO) mask, then the largest blob's centroid
  - range from depth, or from the cube size when there is no depth
  - back-projection into the camera frame
  - a proportional centering rate command, a look-at pose, and a standoff point
- `tracker_node.py`:
  - **subscribes:** image, camera_info and (optionally) depth
  - **publishes:** `/urex/debris/position` (the output for the other team), `position_world` (through TF),
    `detected`, `centering_error`, `cmd_twist`, `cmd_pose`, `debug_image`
- `fake_camera_node.py`: a simulated pan-tilt camera for testing with no hardware.
- `eval_node.py` (new, see section 2): measures the tracker's error against Unity's ground truth.
- 5 pytest tests, including a closed-loop centering test.
- Full topic list and usage: `Codes/ros2_ws/src/debris_tracker/README.md`.

---

## 2. Done since the last session

### Unity <-> ROS 2 bridge (new)
- **Transport:** rosbridge (JSON over WebSocket, `ws://localhost:9090`), the same protocol as
  [ROSBridgeLib](https://github.com/MathiasCiarlo/ROSBridgeLib). That library is ROS 1 only (its Header uses
  `seq` and `secs`/`nsecs`), so the Unity side is a small client of our own with no DLLs.
- New Unity scripts:
  - `RosBridgeClient.cs`: WebSocket client with auto-reconnect; callbacks run on Unity's main thread.
  - `RosMsgs.cs`: ROS 2 message layouts, plus the Unity <-> ROS coordinate conversions.
  - `TwinRosBridge.cs`: the component on the `Twin` GameObject. Bottom-left GUI shows the connection status and
    the camera control mode (off / twist / pose).
- **Unity -> ROS:**

  | Topic | Type | Content |
  |---|---|---|
  | `/twin/camera/image_raw` | `sensor_msgs/Image` | rgb8 424 x 240 at 10 Hz |
  | `/twin/camera/camera_info` | `sensor_msgs/CameraInfo` | camera intrinsics |
  | `/tf` | `tf2_msgs/TFMessage` | `world -> d405_color_optical_frame`, `world -> debris` |
  | `/twin/camera/pose` | `geometry_msgs/PoseStamped` | camera pose in `world` |
  | `/twin/debris/pose` | `geometry_msgs/PoseStamped` | true debris position, for checking the tracker |

- **ROS -> Unity:** `/urex/camera/cmd_twist` (turn the camera) or `/urex/camera/cmd_pose` (move it to the standoff
  point). Either one switches the twin to its manual camera, which pauses the mission; the myCobot follows by IK.
- **Frames:**
  - `world` is the REP-103 version of the Unity world (x = Unity z, y = -Unity x, z = Unity y), which is the
    table frame.
  - Optical frame = (x, -y, z) of the Unity camera.

### Other changes
- **Debris colour fix.** The twin's debris was the same hue as the yellow arm (H = 21), so the colour detector
  merged them. It is now `TwinPlayback.debrisColor` = green `(0.20, 0.80, 0.30)`. TwinPlayback also gained public
  accessors and `SetManualPose()` for the bridge.
- **New `launch/twin.launch.py`:** starts rosbridge and the tracker with the twin's topics, `world_frame:=world`
  and HSV range `[50,100,50]`-`[80,255,255]`. `rosbridge_server` was added to `package.xml`.
- **Tracker TF bug fix.** The TF lookup used to block for 50 ms inside the image callback. TF arrives through the
  same single-threaded executor, so it could never arrive during that wait. The lookup no longer waits: it uses the
  image's own timestamp if TF has it, otherwise the latest transform.
- **New `eval_node.py` (accuracy check).** For every `/urex/debris/position` it takes Unity's true camera and debris
  poses with the same timestamp, expresses the true debris position in the camera frame, and reports:
  - the error vs the cube's front face, which is what the tracker measures
  - the error vs the cube's centre
  - the range error
  - the direction error

  It can also save every sample to CSV (`-p csv:=...`).
- **Unity "run in background" fix.** The Unity Editor stopped simulating whenever its window lost focus (e.g. while
  using an Ubuntu terminal or rqt), so ROS received nothing. `TwinRosBridge` now sets
  `Application.runInBackground = true`.
- **Twin Rosbridge saved in the scene.** It had been added during Play mode, so it was lost when Play stopped. It is
  now on `Twin` in `SampleScene` and saved.

### Path missions (new, 25 Oct)
Three scenarios in `TwinPlayback`. Switch with keys **1 / 2 / 3**, the **Mission** toolbar at the top of the in-game
GUI, or the `mission` dropdown in the Inspector. Switching restarts the approach towards the new debris position.

| # | Mission | Debris arm (yellow) | Debris |
|---|---|---|---|
| 1 | **table** (original) | on the table, fixed L pose | still, at (-0.07, 0.39, 0.40) |
| 2 | **ceiling** | hangs upside down from an overhead gantry (beam at `ceilingHeight` = 0.80 m; legs on the floor beside the table, nothing on the table) | still, at about the same height (0.41 m) |
| 3 | **orbit** | on the table, fixed L pose | turns on a spoke in a **vertical circle** facing the chaser: `orbitRadius` 0.15 m, `orbitPeriod` 20 s |

- **Mission 3 timing:** the orbit runs on real time, not mission time, so it keeps turning when the tracker takes
  the camera and pauses the mission. The cube turns with its spoke.
- **Rail:** for every mission, the rail recomputes its stroke so the camera stops 0.2 m from the debris (mission 3:
  from the orbit centre).
- **Recompile guard:** `TwinPlayback` stays quiet (instead of spamming errors) if scripts are recompiled during Play;
  stop and press Play again.

### ROS 2 topics (17 while the twin is running)
| Topic | From | Meaning |
|---|---|---|
| `/twin/camera/image_raw`, `/twin/camera/camera_info` | Unity | D405 render and its intrinsics |
| `/twin/camera/pose`, `/twin/debris/pose`, `/tf` | Unity | true camera and debris poses in `world` |
| **`/urex/debris/position`** | tracker | **debris XYZ in the camera frame** (x right, y down, z forward) |
| `/urex/debris/position_world` | tracker | same point in `world` |
| `/urex/debris/detected`, `/urex/camera/centering_error` | tracker | tracking status, pointing error |
| `/urex/camera/cmd_twist`, `/urex/camera/cmd_pose` | tracker | camera commands, applied by Unity |
| `/urex/debris/debug_image` | tracker | annotated image for rqt |
| `/rosout`, `/parameter_events`, `/tf_static`, `/client_count`, `/connected_clients` | ROS / rosbridge | infrastructure, ignore |

**Image frame -> camera frame.** The tracker does this conversion on every frame:
1. Find the pixel centroid `(u, v)` of the debris.
2. Get the range `Z` (from depth, or from the cube's apparent size).
3. Back-project: `X = (u-cx)·Z/fx`, `Y = (v-cy)·Z/fy`.
4. Publish `(X, Y, Z)` on `/urex/debris/position`. **This is the topic for the other team.**

The pixel `(u, v)` itself is not published yet.

### Environment set up
- WSL2 **Ubuntu 24.04** + **ROS 2 Jazzy** (desktop), `rosbridge_suite`, `tf2_geometry_msgs`, `rqt_image_view`,
  OpenCV, colcon.
- Workspace: `~/ros2_ws` in WSL. `src/debris_tracker` is a symlink to the OneDrive copy, built with
  `--symlink-install`. `~/.bashrc` sources both Jazzy and the workspace.

---

## 3. Verified

| Check | Result |
|---|---|
| Coordinate conversions (numerical check, 200 random poses) | all consistent |
| Simulated loop, the Unity way (tracker twist -> Unity rotation) | 27.8 deg -> 0.5 deg (deadband) |
| HSV range vs. scene colours, at 30-100 % shading | only the debris matches |
| Unity compile (headless batchmode) | no errors |
| `pytest test/` in WSL | 5 passed |
| Fake-camera closed loop under ROS | debris centred to 1.4 mm / 0.37 deg |
| `twin.launch.py` | rosbridge listening on 9090, tracker running |
| **Live Unity <-> ROS loop** | Unity connected, image the right way up, `DET` with the mask on the cube only, centroid on the image centre |
| Range estimate in the twin | Z = 0.617 m (from cube size), true distance ~0.66 m to the centre (0.61 m to the front face) |
| Topic rates (live) | image 9.2 Hz, camera pose 9.5 Hz, `/urex/debris/position` 9.7 Hz |
| **Accuracy vs ground truth** (`eval_node`, 82 samples, centred view) | error vs face **0.3 cm**, direction **0.22°**, range **+0.2 cm**; vs centre 4.8 cm (the half-cube offset) |
| Missions 1-3 geometry (Play mode) | 1: debris (-0.070, 0.389, 0.404), unchanged. 2: arm base at 0.80 m on the gantry, debris at 0.41 m. 3: debris stays exactly 0.150 m from the hub while orbiting |
| **Mission 3, tracking a moving target** (`eval_node`, 12 s) | direction error **0.15-0.54°**, camera within ~1-2° of centre; range error swings **-8 cm to 0 cm** as the spinning cube shows its edges (size-based ranging under-estimates at an angle, so depth is needed) |

---

## 4. Setup and run, step by step

**Terminals.** Commands marked **PowerShell** go in Windows PowerShell. Everything else goes in an **Ubuntu**
terminal: Start menu → "Ubuntu 24.04", or Windows Terminal's **˅** menu → Ubuntu 24.04. The Ubuntu prompt looks
like `acer@DESKTOP-...:~$`. `ros2`, `source` and `colcon` do **not** work in PowerShell.

### A. One-time setup (already done on this laptop; skip to B)

**Step 1. Install Ubuntu (PowerShell, as admin)**
```powershell
wsl --install -d Ubuntu-24.04
```
Reboot, open Ubuntu, and create a Linux username and password.

**Step 2. Set a UTF-8 locale**
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

**Step 3. Add the ROS 2 apt repository**
```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

**Step 4. Install ROS 2 Jazzy and the project's packages**
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools \
    ros-jazzy-rosbridge-suite ros-jazzy-tf2-geometry-msgs ros-jazzy-rqt-image-view python3-opencv
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

**Step 5. Build the tracker workspace**
```bash
mkdir -p ~/ros2_ws/src
ln -s "/mnt/c/Users/Acer/OneDrive/Documents/NUS YEAR 2 SEM1/UREX/Codes/ros2_ws/src/debris_tracker" ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```
Thanks to `--symlink-install`, edits to the Python files take effect without rebuilding. Rebuild
(`cd ~/ros2_ws && colcon build --symlink-install`) only after adding new files: launch, config, or a new node in
`setup.py`.

**Step 6. Check the ROS side, without Unity**
```bash
ros2 launch debris_tracker tracker.launch.py fake_camera:=true    # terminal 1
ros2 topic echo /urex/debris/position                             # terminal 2: numbers should appear
```
Stop terminal 1 with **Ctrl+C**.

**Step 7. Prepare Unity (once; already done on this laptop)**
1. Copy the scripts in `Codes/unity/Scripts/` into `C:\Users\Acer\UREX_TwinPlayback\Assets\Scripts\`. Repeat this
   whenever a script in `Codes/unity/Scripts/` changes.
2. Open the project, then scene `SampleScene`.
3. **Make sure Play mode is off** (the Play button isn't blue). Anything you change during Play is lost when
   you stop.
4. Select the `Twin` GameObject → **Add Component** → **Twin Rosbridge**. Save the scene (**Ctrl+S**).

### B. Every session

**Step 1. Start ROS** (Ubuntu terminal 1, leave it running)
```bash
ros2 launch debris_tracker twin.launch.py
```

**Step 2. Start Unity**
Press **Play**. Check that the box at the bottom-left says **"ROS connected to ws://localhost:9090"**.

**Step 2b. Choose the mission**
Press **1** (table), **2** (ceiling) or **3** (orbit), or click it in the **Mission** toolbar (top-left box).

**Step 3. Point the camera at the debris**
In the Game view, press **M** (manual camera), then **F** (look at the debris).

**Step 4. Let the tracker take over**
In the bottom-left box, select **twist** (turn only) or **pose** (also move to 0.2 m from the debris). The
camera centres on the green cube by itself.

**Step 5. Watch the result** (Ubuntu terminal 2)
```bash
ros2 run rqt_image_view rqt_image_view /urex/debris/debug_image
```
Expected: status `DET`, a green mask on the cube only, and the red dot on the image centre.

**Step 6. Check accuracy against the ground truth** (Ubuntu terminal 3)
```bash
ros2 run debris_tracker eval_node                                  # live summary every 2 s
ros2 run debris_tracker eval_node --ros-args -p csv:=/tmp/eval.csv # also save every sample
```
For each estimate, it takes Unity's true camera and debris poses with the same timestamp and prints:
- **error vs face**: the fair error, because the tracker measures the cube's front face
- error vs the cube's centre
- range error
- direction error

Result on 25 Oct (camera centred, range from cube size): **0.3 cm** error vs face, **0.22°** direction error,
**+0.2 cm** range error. The error vs centre (4.8 cm) is just the half-cube offset, as expected.

To see the raw numbers instead: `ros2 topic echo /urex/debris/position_world` next to
`ros2 topic echo /twin/debris/pose`.

**Step 7. Stop**
Stop Play in Unity, then press **Ctrl+C** in terminal 1.

### If something is wrong
| Symptom | Fix |
|---|---|
| `ros2: command not found` / `source not recognized` | You're in PowerShell; use an Ubuntu terminal |
| Topics go silent when Unity isn't the focused window | Fixed: TwinRosBridge sets `Application.runInBackground = true` |
| No ROS box at the bottom-left in Unity | Twin Rosbridge isn't on `Twin`: do A step 7 with Play mode **off** |
| Unity box stays on "connecting..." | Start `twin.launch.py` first (B step 1); check port 9090 is free |
| rqt shows a grey gradient | No frames yet: Unity isn't playing, or isn't connected |
| Image upside down | Untick **Flip Image** on the Twin Rosbridge component |
| Status `LOST` | Debris out of view: press **F** in Unity |

---

## 5. Open items / next steps
1. **Harder test cases** (run `eval_node` during each):
   - knock the camera off target with the arrow keys and watch it recover (expect larger errors while it swings)
   - view the cube at an angle (ranging from the cube size under-estimates the distance there)
   - `pose` mode: the camera moves to 0.2 m standoff
   - run each check on all three missions; mission 3 at a faster `orbitPeriod` to find where tracking breaks
2. **Depth from Unity (highest priority):** publish an aligned depth image so range comes from depth, as with the
   real D405, instead of from the cube size. Mission 3 showed size-based range errors of up to 8 cm.
3. **Logging for evaluation:** `eval_node` CSV covers position error. Still to do: record a rosbag, and measure
   time to centre and tracking rate.
4. **Publish the pixel `(u, v)`** on its own topic (e.g. `/urex/debris/pixel`) if the other team needs the
   image-frame point.
5. **Real hardware:**
   - set `tracker.yaml`'s HSV range to the real cube's colour (it still holds the orange range)
   - run with the real D405 (`realsense2_camera`)
   - write a myCobot controller that turns `cmd_twist` / `cmd_pose` into joint motion
6. **Not bridged yet:** joint states, rail position command, mission play/pause/scrub.
7. **Housekeeping:** delete the stray `C:\Users\Acer\.bashrc`, created by running `echo >> ~/.bashrc` in
   PowerShell. It is still there: `Remove-Item C:\Users\Acer\.bashrc` (PowerShell).
