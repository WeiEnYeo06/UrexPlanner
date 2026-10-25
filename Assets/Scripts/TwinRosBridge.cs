// TwinRosBridge.cs -- connects the digital twin to ROS 2 through rosbridge (RosBridgeClient).
//
// Add it to the Twin GameObject, next to TwinPlayback, and press Play. ROS side (WSL / Ubuntu):
//   ros2 launch debris_tracker twin.launch.py          (rosbridge on port 9090 + the debris tracker)
//
// Publishes (Unity -> ROS). The image, camera_info and TF of one capture share the same stamp:
//   /twin/camera/image_raw     sensor_msgs/Image          D405 render, rgb8, frame d405_color_optical_frame
//   /twin/camera/camera_info   sensor_msgs/CameraInfo     pinhole intrinsics of the Unity camera
//   /tf                        tf2_msgs/TFMessage         world -> d405_color_optical_frame, world -> debris
//   /twin/camera/pose          geometry_msgs/PoseStamped  optical frame pose in world
//   /twin/debris/pose          geometry_msgs/PoseStamped  debris centre in world (ground truth for the tracker)
// Subscribes (ROS -> Unity), from debris_tracker:
//   /urex/camera/cmd_twist     geometry_msgs/TwistStamped velocity in the optical frame (rate control)
//   /urex/camera/cmd_pose      geometry_msgs/PoseStamped  setpoint in the optical frame (position control)
// A non-zero command switches the twin to its manual camera (this pauses the mission); the myCobot follows by IK.
//
// world = REP-103 version of the Unity world (x = Unity z, y = -Unity x, z = Unity y), i.e. the table frame.
using System;
using System.Collections;
using UnityEngine;
using UnityEngine.Rendering;
using RosMsgs;

[RequireComponent(typeof(TwinPlayback))]
public class TwinRosBridge : MonoBehaviour
{
    public enum CameraControl { Off, Twist, Pose }

    [Header("rosbridge")]
    public string url = "ws://localhost:9090";

    [Header("Unity -> ROS")]
    public float imageRate = 10f;         // Hz (one 424 x 240 rgb8 frame is ~400 kB of JSON)
    public bool flipImage = true;         // GPU readback rows run bottom-up; ROS images run top-down
    public string worldFrame = "world";
    public string cameraFrame = "d405_color_optical_frame";
    public string debrisFrame = "debris";

    [Header("ROS -> Unity")]
    public CameraControl control = CameraControl.Twist;
    public string cmdTwistTopic = "/urex/camera/cmd_twist";
    public string cmdPoseTopic = "/urex/camera/cmd_pose";
    public float commandTimeout = 0.5f;   // s without a command -> hold still
    public float poseMoveSpeed = 0.05f;   // m/s towards a cmd_pose setpoint
    public float poseTurnSpeed = 30f;     // deg/s

    TwinPlayback twin;
    RosBridgeClient ros;
    bool readbackPending;
    float nextCapture;
    Vector3 cmdLinear, cmdAngular;        // Unity camera-local
    Vector3 poseTargetPos;                // Unity world
    Quaternion poseTargetRot = Quaternion.identity;
    float twistTime = -1e9f, poseTime = -1e9f;
    int framesSent, commandsReceived;

    void Start()
    {
        twin = GetComponent<TwinPlayback>();
        Application.runInBackground = true;   // keep simulating and publishing while rqt / a terminal has focus
        ros = new RosBridgeClient(url);
        ros.Advertise("/twin/camera/image_raw", "sensor_msgs/msg/Image");
        ros.Advertise("/twin/camera/camera_info", "sensor_msgs/msg/CameraInfo");
        ros.Advertise("/tf", "tf2_msgs/msg/TFMessage");
        ros.Advertise("/twin/camera/pose", "geometry_msgs/msg/PoseStamped");
        ros.Advertise("/twin/debris/pose", "geometry_msgs/msg/PoseStamped");
        ros.Subscribe(cmdTwistTopic, "geometry_msgs/msg/TwistStamped", OnTwist);
        ros.Subscribe(cmdPoseTopic, "geometry_msgs/msg/PoseStamped", OnPose);
        StartCoroutine(CaptureLoop());
    }

    void OnDestroy()
    {
        ros?.Dispose();   // otherwise the socket stays open after leaving Play mode
        ros = null;
    }

    // ------------------------------------------------------------------ //
    // ROS -> Unity
    void OnTwist(string json)
    {
        var m = JsonUtility.FromJson<TwistStampedIn>(json).msg;
        cmdLinear = RosConv.OpticalToUnityLocal(RosConv.V(m.twist.linear));
        cmdAngular = RosConv.OpticalRateToUnityLocal(RosConv.V(m.twist.angular));
        twistTime = Time.time;
        commandsReceived++;
    }

    void OnPose(string json)
    {
        // The setpoint is relative to the camera when the image was taken; the current pose is close enough.
        var m = JsonUtility.FromJson<PoseStampedIn>(json).msg;
        var cam = twin.D405Camera.transform;
        poseTargetPos = cam.position + cam.rotation * RosConv.OpticalToUnityLocal(RosConv.V(m.pose.position));
        poseTargetRot = Quaternion.Normalize(cam.rotation * RosConv.OpticalToUnityLocal(RosConv.Q(m.pose.orientation)));
        poseTime = Time.time;
        commandsReceived++;
    }

    void Update()
    {
        if (ros == null) return;
        ros.Poll();

        bool twist = control == CameraControl.Twist && Time.time - twistTime < commandTimeout
                     && (cmdAngular.sqrMagnitude > 0f || cmdLinear.sqrMagnitude > 0f);
        bool pose = control == CameraControl.Pose && Time.time - poseTime < commandTimeout;
        if (!twist && !pose) return;
        if (!twin.ManualActive) { twin.manualCamera = true; return; }   // TwinPlayback enters manual mode next Update

        float dt = Time.deltaTime;
        Vector3 pos = twin.ManualPosition;
        Quaternion rot = twin.ManualRotation;
        if (twist)
        {
            pos += rot * cmdLinear * dt;
            float rate = cmdAngular.magnitude;   // rad/s, body frame -> right-multiply
            if (rate > 1e-9f) rot *= Quaternion.AngleAxis(rate * dt * Mathf.Rad2Deg, cmdAngular / rate);
        }
        else
        {
            pos = Vector3.MoveTowards(pos, poseTargetPos, poseMoveSpeed * dt);
            rot = Quaternion.RotateTowards(rot, poseTargetRot, poseTurnSpeed * dt);
        }
        twin.SetManualPose(pos, rot);
    }

    // ------------------------------------------------------------------ //
    // Unity -> ROS. Capture at the end of the frame, after the D405 camera has rendered, so the image and
    // the poses describe the same instant.
    IEnumerator CaptureLoop()
    {
        var endOfFrame = new WaitForEndOfFrame();
        while (true)
        {
            yield return endOfFrame;
            if (ros == null || !ros.Connected || readbackPending || ros.Pending > 4) continue;
            if (Time.time < nextCapture || twin.D405Texture == null) continue;
            nextCapture = Time.time + 1f / Mathf.Max(0.1f, imageRate);

            var stamp = TimeMsg.Now();
            PublishPoses(stamp);   // TF before the image, so the tracker can look it up
            var tex = twin.D405Texture;
            int w = tex.width, h = tex.height;
            readbackPending = true;
            AsyncGPUReadback.Request(tex, 0, TextureFormat.RGBA32, req => OnReadback(req, stamp, w, h));
        }
    }

    void PublishPoses(TimeMsg stamp)
    {
        var cam = twin.D405Camera.transform;
        Vector3 camPos = RosConv.ToRos(cam.position), debPos = RosConv.ToRos(twin.Debris.position);
        Quaternion camRot = RosConv.OpticalFrameInRosWorld(cam.rotation), debRot = RosConv.ToRos(twin.Debris.rotation);
        ros.Publish("/tf", new TFMessageMsg
        {
            transforms = new[] { MakeTransform(stamp, cameraFrame, camPos, camRot), MakeTransform(stamp, debrisFrame, debPos, debRot) }
        });
        ros.Publish("/twin/camera/pose", MakePose(stamp, camPos, camRot));
        ros.Publish("/twin/debris/pose", MakePose(stamp, debPos, debRot));
    }

    void OnReadback(AsyncGPUReadbackRequest req, TimeMsg stamp, int w, int h)
    {
        readbackPending = false;
        if (ros == null || req.hasError) return;
        byte[] src = req.GetData<byte>().ToArray();   // RGBA32
        var rgb = new byte[w * h * 3];
        for (int y = 0; y < h; y++)
        {
            int s = (flipImage ? h - 1 - y : y) * w * 4, d = y * w * 3;
            for (int x = 0; x < w; x++, s += 4, d += 3)
            {
                rgb[d] = src[s]; rgb[d + 1] = src[s + 1]; rgb[d + 2] = src[s + 2];
            }
        }
        var header = new HeaderMsg { stamp = stamp, frame_id = cameraFrame };
        ros.Publish("/twin/camera/image_raw", new ImageMsg
        {
            header = header, height = h, width = w, encoding = "rgb8", step = w * 3, data = Convert.ToBase64String(rgb)
        });
        ros.Publish("/twin/camera/camera_info", Intrinsics(header, w, h));
        framesSent++;
    }

    CameraInfoMsg Intrinsics(HeaderMsg header, int w, int h)
    {
        // Unity's fieldOfView is vertical and its pixels are square (the 87 deg horizontal FOV follows from the aspect).
        double fy = 0.5 * h / Math.Tan(0.5 * twin.D405Camera.fieldOfView * Mathf.Deg2Rad), fx = fy;
        double cx = 0.5 * w, cy = 0.5 * h;
        return new CameraInfoMsg
        {
            header = header, height = h, width = w,
            k = new[] { fx, 0, cx, 0, fy, cy, 0, 0, 1 },
            p = new[] { fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0 },
        };
    }

    TransformStampedMsg MakeTransform(TimeMsg stamp, string child, Vector3 p, Quaternion q)
    {
        return new TransformStampedMsg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = worldFrame }, child_frame_id = child,
            transform = new TransformMsg { translation = RosConv.V(p), rotation = RosConv.Q(q) },
        };
    }

    PoseStampedMsg MakePose(TimeMsg stamp, Vector3 p, Quaternion q)
    {
        return new PoseStampedMsg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = worldFrame },
            pose = new PoseMsg { position = RosConv.P(p), orientation = RosConv.Q(q) },
        };
    }

    // ------------------------------------------------------------------ //
    void OnGUI()
    {
        if (ros == null) return;
        GUILayout.BeginArea(new Rect(10, Screen.height - 95, 330, 85), GUI.skin.box);
        GUILayout.Label(ros.Connected ? $"ROS  connected to {url}" : $"ROS  connecting to {url} ...");
        GUILayout.Label($"sent {framesSent} frames, received {commandsReceived} commands");
        GUILayout.BeginHorizontal();
        GUILayout.Label("camera control", GUILayout.Width(95));
        control = (CameraControl)GUILayout.Toolbar((int)control, new[] { "off", "twist", "pose" });
        GUILayout.EndHorizontal();
        GUILayout.EndArea();
    }
}
