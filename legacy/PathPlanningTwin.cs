// PathPlanningTwin.cs
// Unity side of the Path Planning digital-twin loop (UREX proximity operations).
//
// Every simulation step this script:
//   1. Moves the chaser along trajectory_lvlh.csv (debris-centred LVLH frame, debris fixed at the origin).
//   2. Captures an RGB image + metric depth map + ground-truth debris mask from the end-effector camera.
//   3. Sends the frame to the Python planner over TCP and waits for its reply (lockstep, so runs are repeatable).
//   4. Applies the returned camera setpoint with a simple "stand-in arm" (reach limit + speed limits).
//      The Manipulator team later replaces step 4 with real IK.
//
// Requirements: Unity 2022.2 or newer (uses RaycastCommand + QueryParameters). Works with Built-in and URP.
//
// Scene setup:
//   Debris          (needs a Collider, e.g. MeshCollider, on it or a child; its position = LVLH origin)
//   Chaser
//     ArmBase       (empty, where the on-board arm is mounted)
//     EECamera      (Camera component; child of Chaser, scale 1)
//   Put trajectory_lvlh.csv in Assets/StreamingAssets/.
//   The EE camera renders into an off-screen texture, so add a separate camera if you want to watch the Game view.
//
// Frames sent to Python use the OpenCV camera-optical convention: x right, y down, z forward (metres).
// Unity's camera local frame is x right, y up, z forward, so the conversion is just y -> -y.

using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net.Sockets;
using System.Text;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

public class PathPlanningTwin : MonoBehaviour
{
    public enum AxisSource { PlusX, MinusX, PlusY, MinusY, PlusZ, MinusZ }

    [Header("Scene references")]
    [Tooltip("Debris object. Its position is the LVLH origin. It (or a child) needs a Collider for the depth rays.")]
    public Transform debris;
    [Tooltip("Chaser body, driven along the reference trajectory.")]
    public Transform chaser;
    [Tooltip("Mount point of the on-board arm (child of the chaser). The camera stays within Arm Reach of this.")]
    public Transform armBase;
    [Tooltip("End-effector RGB-D camera. Make it a child of the chaser with scale (1,1,1).")]
    public Camera eeCamera;

    [Header("Reference trajectory (debris-centred LVLH)")]
    public string trajectoryFile = "trajectory_lvlh.csv";
    [Tooltip("Simulation step in seconds. The CSV uses 0.1 s.")]
    public float simDt = 0.1f;
    public bool chaserFacesDebris = true;
    [Tooltip("Which LVLH axis feeds each Unity axis. LVLH is right-handed, Unity is left-handed, " +
             "so the mapping must have determinant -1 (checked at start).")]
    public AxisSource unityXFrom = AxisSource.PlusY;
    public AxisSource unityYFrom = AxisSource.PlusZ;
    public AxisSource unityZFrom = AxisSource.MinusX;

    [Header("Simulated RGB-D sensor")]
    public int width = 320;
    public int height = 240;
    public float verticalFovDeg = 42f;
    public float minDepth = 0.05f;
    public float maxDepth = 5f;
    [Tooltip("Layers the depth rays can hit. Exclude the chaser/arm/camera bodies.")]
    public LayerMask depthLayers = ~0;
    [Tooltip("Toggle if the image arrives in Python upside down.")]
    public bool flipImageVertically = true;

    [Header("Stand-in arm (replace with the Manipulator team's IK later)")]
    public float armReach = 0.5f;
    public float maxLinearSpeed = 0.1f;   // m/s
    public float maxAngularSpeed = 30f;   // deg/s
    public bool aimCameraAtStart = true;

    [Header("Python link")]
    public string host = "127.0.0.1";
    public int port = 5555;
    public int timeoutMs = 5000;

    // ---------- runtime state ----------
    readonly List<float> trajT = new List<float>();
    readonly List<Vector3> trajR = new List<Vector3>();
    Vector3 currentLvlh;
    float simTime;
    int frameIndex;
    bool lastClamped;

    RenderTexture rt;
    Texture2D readTex;
    float fx, fy, cx, cy;
    Vector3[] rayDirLocal;   // unit ray per pixel, Unity camera-local
    float[] rayToZ;          // converts distance along the ray into optical-axis depth Z

    TcpClient client;
    NetworkStream stream;

    [Serializable]
    class FrameHeader
    {
        public int frame;
        public float t;
        public int width, height;
        public float fx, fy, cx, cy;
        public float[] camToWorld;       // 4x4 row-major, camera-optical -> Unity world
        public float[] debrisCentreCam;  // ground truth, camera-optical frame (for metrics only)
        public float[] chaserLvlh;       // reference trajectory point at this time
        public bool lastSetpointClamped; // previous setpoint was outside arm reach (feasibility metric)
        public int rgbBytes, depthBytes, maskBytes;
    }

    [Serializable]
    class SetpointMsg
    {
        public bool valid;
        public bool stop;
        public float[] position; // camera-optical frame of the frame just sent
        public float[] lookAt;   // camera-optical frame of the frame just sent
    }

    // =====================================================================
    IEnumerator Start()
    {
        if (!CheckSetup()) yield break;

        LoadTrajectory(Path.Combine(Application.streamingAssetsPath, trajectoryFile));
        if (trajT.Count < 2) yield break;

        SetupSensor();

        try { Connect(); }
        catch (Exception e)
        {
            Debug.LogError($"[PathPlanningTwin] Could not connect to Python at {host}:{port}. " +
                           $"Start path_planning_server.py first. ({e.Message})");
            yield break;
        }

        float tStart = trajT[0];
        float tEnd = trajT[trajT.Count - 1];
        simTime = tStart;
        PlaceChaser(simTime);
        if (aimCameraAtStart)
        {
            Vector3 d = debris.position - eeCamera.transform.position;
            if (d.sqrMagnitude > 1e-8f) eeCamera.transform.rotation = Quaternion.LookRotation(d, chaser.up);
        }

        while (simTime <= tEnd + 1e-4f)
        {
            PlaceChaser(simTime);
            Physics.SyncTransforms();
            yield return new WaitForEndOfFrame(); // camera has now rendered this pose

            bool ok;
            try { ok = StepOnce(); }
            catch (Exception e)
            {
                Debug.LogError($"[PathPlanningTwin] Link error at frame {frameIndex}: {e.Message}");
                ok = false;
            }
            if (!ok) break;

            frameIndex++;
            simTime = tStart + frameIndex * simDt; // avoids float drift from repeated +=
        }

        Debug.Log($"[PathPlanningTwin] Finished after {frameIndex} frames (t = {simTime:F1} s).");
        CloseLink();
    }

    bool StepOnce()
    {
        byte[] rgb = CaptureRgb();
        CaptureDepthAndMask(out float[] depth, out byte[] mask);

        Vector3 c = eeCamera.transform.InverseTransformPoint(debris.position);
        var header = new FrameHeader
        {
            frame = frameIndex,
            t = simTime,
            width = width,
            height = height,
            fx = fx, fy = fy, cx = cx, cy = cy,
            camToWorld = OpticalToWorldRowMajor(),
            debrisCentreCam = new[] { c.x, -c.y, c.z },
            chaserLvlh = new[] { currentLvlh.x, currentLvlh.y, currentLvlh.z },
            lastSetpointClamped = lastClamped,
            rgbBytes = rgb.Length,
            depthBytes = depth.Length * 4,
            maskBytes = mask.Length
        };

        SendFrame(header, rgb, depth, mask);

        int len = ReadInt32BE();
        var reply = JsonUtility.FromJson<SetpointMsg>(Encoding.UTF8.GetString(ReadExact(len)));
        if (reply == null || reply.stop) return false;

        ApplySetpoint(reply);
        return true;
    }

    // =====================================================================
    // Setup and checks
    bool CheckSetup()
    {
        if (debris == null || chaser == null || armBase == null || eeCamera == null)
        {
            Debug.LogError("[PathPlanningTwin] Assign Debris, Chaser, Arm Base and EE Camera in the Inspector.");
            return false;
        }
        if (debris.GetComponentInChildren<Collider>() == null)
        {
            Debug.LogError("[PathPlanningTwin] Debris has no Collider, so depth rays cannot hit it. Add a MeshCollider.");
            return false;
        }
        if (!eeCamera.transform.IsChildOf(chaser))
            Debug.LogWarning("[PathPlanningTwin] EE Camera is not a child of the Chaser; it will not be carried along the trajectory.");
        if ((eeCamera.transform.lossyScale - Vector3.one).sqrMagnitude > 1e-6f)
            Debug.LogWarning("[PathPlanningTwin] EE Camera has non-unit scale; setpoint conversions assume scale (1,1,1).");

        Vector3 a = AxisVec(unityXFrom), b = AxisVec(unityYFrom), cc = AxisVec(unityZFrom);
        float det = Vector3.Dot(a, Vector3.Cross(b, cc));
        if (Mathf.Abs(det) < 0.5f)
        {
            Debug.LogError("[PathPlanningTwin] LVLH->Unity mapping uses the same LVLH axis twice.");
            return false;
        }
        if (det > 0f)
            Debug.LogWarning("[PathPlanningTwin] LVLH->Unity mapping has determinant +1, so the trajectory will be MIRRORED. " +
                             "Flip the sign of one axis (right-handed LVLH needs det = -1 in left-handed Unity).");
        return true;
    }

    void SetupSensor()
    {
        rt = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
        rt.Create();
        readTex = new Texture2D(width, height, TextureFormat.RGB24, false);
        eeCamera.targetTexture = rt;
        eeCamera.fieldOfView = verticalFovDeg;
        eeCamera.nearClipPlane = Mathf.Min(eeCamera.nearClipPlane, 0.01f);

        // Pinhole intrinsics, OpenCV convention (pixel centres at integer coordinates).
        fy = 0.5f * height / Mathf.Tan(0.5f * verticalFovDeg * Mathf.Deg2Rad);
        fx = fy; // square pixels
        cx = 0.5f * width - 0.5f;
        cy = 0.5f * height - 0.5f;

        int n = width * height;
        rayDirLocal = new Vector3[n];
        rayToZ = new float[n];
        for (int v = 0; v < height; v++)
        {
            for (int u = 0; u < width; u++)
            {
                float xn = (u - cx) / fx;
                float yn = (v - cy) / fy;
                var d = new Vector3(xn, -yn, 1f); // optical y-down -> Unity y-up
                float norm = d.magnitude;
                int i = v * width + u;
                rayDirLocal[i] = d / norm;
                rayToZ[i] = 1f / norm;
            }
        }
        Debug.Log($"[PathPlanningTwin] Sensor {width}x{height}, fx = fy = {fx:F1} px, cx = {cx:F1}, cy = {cy:F1}");
    }

    // =====================================================================
    // Trajectory
    void LoadTrajectory(string path)
    {
        if (!File.Exists(path))
        {
            Debug.LogError($"[PathPlanningTwin] Trajectory not found: {path}. Put {trajectoryFile} in Assets/StreamingAssets/.");
            return;
        }
        var inv = CultureInfo.InvariantCulture;
        foreach (string line in File.ReadLines(path))
        {
            string s = line.Trim();
            if (s.Length == 0) continue;
            string[] p = s.Split(',');
            if (p.Length < 4) continue;
            if (!float.TryParse(p[0], NumberStyles.Float, inv, out float t)) continue; // header row
            float x = float.Parse(p[1], NumberStyles.Float, inv);
            float y = float.Parse(p[2], NumberStyles.Float, inv);
            float z = float.Parse(p[3], NumberStyles.Float, inv);
            trajT.Add(t);
            trajR.Add(new Vector3(x, y, z));
        }
        if (trajT.Count < 2)
        {
            Debug.LogError("[PathPlanningTwin] Trajectory has fewer than 2 valid rows.");
            return;
        }
        Vector3 r0 = trajR[0], r1 = trajR[trajR.Count - 1];
        Debug.Log($"[PathPlanningTwin] Loaded {trajT.Count} points, t = {trajT[0]:F1}..{trajT[trajT.Count - 1]:F1} s. " +
                  $"Start LVLH {r0.ToString("F3")} (|r| = {r0.magnitude:F3} m) -> Unity {LvlhToUnity(r0).ToString("F3")}; " +
                  $"end LVLH {r1.ToString("F3")} (|r| = {r1.magnitude:F3} m) -> Unity {LvlhToUnity(r1).ToString("F3")}");
    }

    Vector3 TrajectoryAt(float t)
    {
        int last = trajT.Count - 1;
        if (t <= trajT[0]) return trajR[0];
        if (t >= trajT[last]) return trajR[last];
        int i = trajT.BinarySearch(t);
        if (i >= 0) return trajR[i];
        i = ~i; // index of first element larger than t
        float a = (t - trajT[i - 1]) / (trajT[i] - trajT[i - 1]);
        return Vector3.Lerp(trajR[i - 1], trajR[i], a);
    }

    void PlaceChaser(float t)
    {
        currentLvlh = TrajectoryAt(t);
        chaser.position = debris.position + LvlhToUnity(currentLvlh);
        if (chaserFacesDebris)
        {
            Vector3 d = debris.position - chaser.position;
            if (d.sqrMagnitude > 1e-8f) chaser.rotation = Quaternion.LookRotation(d, Vector3.up);
        }
    }

    static Vector3 AxisVec(AxisSource a)
    {
        switch (a)
        {
            case AxisSource.PlusX: return new Vector3(1, 0, 0);
            case AxisSource.MinusX: return new Vector3(-1, 0, 0);
            case AxisSource.PlusY: return new Vector3(0, 1, 0);
            case AxisSource.MinusY: return new Vector3(0, -1, 0);
            case AxisSource.PlusZ: return new Vector3(0, 0, 1);
            default: return new Vector3(0, 0, -1);
        }
    }

    Vector3 LvlhToUnity(Vector3 r)
    {
        return new Vector3(Vector3.Dot(AxisVec(unityXFrom), r),
                           Vector3.Dot(AxisVec(unityYFrom), r),
                           Vector3.Dot(AxisVec(unityZFrom), r));
    }

    // =====================================================================
    // Sensor capture
    byte[] CaptureRgb()
    {
        var prev = RenderTexture.active;
        RenderTexture.active = rt;
        readTex.ReadPixels(new Rect(0, 0, width, height), 0, 0, false);
        readTex.Apply(false);
        RenderTexture.active = prev;

        NativeArray<byte> raw = readTex.GetRawTextureData<byte>(); // RGB24, bottom row first
        int rowBytes = width * 3;
        var outBytes = new byte[rowBytes * height];
        for (int row = 0; row < height; row++)
        {
            int src = flipImageVertically ? (height - 1 - row) : row;
            NativeArray<byte>.Copy(raw, src * rowBytes, outBytes, row * rowBytes, rowBytes);
        }
        return outBytes; // row 0 = top of image
    }

    // Depth by batched raycasts: exact metric depth with no shaders, so it works in any render pipeline.
    void CaptureDepthAndMask(out float[] depth, out byte[] mask)
    {
        int n = width * height;
        depth = new float[n];
        mask = new byte[n];

        var cmds = new NativeArray<RaycastCommand>(n, Allocator.TempJob);
        var hits = new NativeArray<RaycastHit>(n, Allocator.TempJob);
        try
        {
            Transform cam = eeCamera.transform;
            Vector3 origin = cam.position;
            var qp = new QueryParameters(depthLayers.value, false, QueryTriggerInteraction.Ignore, false);
            for (int i = 0; i < n; i++)
                cmds[i] = new RaycastCommand(origin, cam.TransformDirection(rayDirLocal[i]), qp, maxDepth);

            JobHandle h = RaycastCommand.ScheduleBatch(cmds, hits, 256);
            h.Complete();

            for (int i = 0; i < n; i++)
            {
                RaycastHit hit = hits[i];
                if (hit.collider == null) continue;     // nothing within maxDepth -> depth 0 = invalid
                float z = hit.distance * rayToZ[i];     // ray length -> optical-axis depth
                if (z < minDepth) continue;
                depth[i] = z;
                if (hit.collider.transform.IsChildOf(debris)) mask[i] = 1;
            }
        }
        finally
        {
            cmds.Dispose();
            hits.Dispose();
        }
    }

    float[] OpticalToWorldRowMajor()
    {
        Matrix4x4 m = eeCamera.transform.localToWorldMatrix * Matrix4x4.Scale(new Vector3(1f, -1f, 1f));
        var a = new float[16];
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                a[r * 4 + c] = m[r, c];
        return a;
    }

    // =====================================================================
    // Stand-in arm
    static Vector3 OpticalToUnityLocal(float[] p) => new Vector3(p[0], -p[1], p[2]);

    void ApplySetpoint(SetpointMsg sp)
    {
        lastClamped = false;
        if (!sp.valid || sp.position == null || sp.position.Length < 3) return; // hold pose

        Transform cam = eeCamera.transform;
        Vector3 target = cam.TransformPoint(OpticalToUnityLocal(sp.position));
        bool hasLook = sp.lookAt != null && sp.lookAt.Length >= 3;
        Vector3 lookWorld = hasLook ? cam.TransformPoint(OpticalToUnityLocal(sp.lookAt)) : Vector3.zero;

        // Reach limit (feasibility). Real IK replaces this.
        Vector3 fromBase = target - armBase.position;
        if (fromBase.magnitude > armReach)
        {
            target = armBase.position + fromBase.normalized * armReach;
            lastClamped = true;
        }

        cam.position = Vector3.MoveTowards(cam.position, target, maxLinearSpeed * simDt);

        if (hasLook)
        {
            Vector3 dir = lookWorld - cam.position;
            if (dir.sqrMagnitude > 1e-8f)
            {
                Quaternion want = Quaternion.LookRotation(dir, chaser.up);
                cam.rotation = Quaternion.RotateTowards(cam.rotation, want, maxAngularSpeed * simDt);
            }
        }
    }

    // =====================================================================
    // TCP framing: [4-byte big-endian header length][header JSON][rgb][depth float32 LE][mask]
    void Connect()
    {
        client = new TcpClient { NoDelay = true, ReceiveTimeout = timeoutMs, SendTimeout = timeoutMs };
        client.Connect(host, port);
        stream = client.GetStream();
        Debug.Log($"[PathPlanningTwin] Connected to Python at {host}:{port}");
    }

    void SendFrame(FrameHeader header, byte[] rgb, float[] depth, byte[] mask)
    {
        byte[] h = Encoding.UTF8.GetBytes(JsonUtility.ToJson(header));
        WriteInt32BE(h.Length);
        stream.Write(h, 0, h.Length);
        stream.Write(rgb, 0, rgb.Length);
        var db = new byte[depth.Length * 4];
        Buffer.BlockCopy(depth, 0, db, 0, db.Length); // little-endian on all Unity desktop targets
        stream.Write(db, 0, db.Length);
        stream.Write(mask, 0, mask.Length);
        stream.Flush();
    }

    void WriteInt32BE(int v)
    {
        byte[] b = BitConverter.GetBytes(v);
        if (BitConverter.IsLittleEndian) Array.Reverse(b);
        stream.Write(b, 0, 4);
    }

    int ReadInt32BE()
    {
        byte[] b = ReadExact(4);
        if (BitConverter.IsLittleEndian) Array.Reverse(b);
        return BitConverter.ToInt32(b, 0);
    }

    byte[] ReadExact(int n)
    {
        var buf = new byte[n];
        int off = 0;
        while (off < n)
        {
            int r = stream.Read(buf, off, n - off);
            if (r <= 0) throw new IOException("Python closed the connection.");
            off += r;
        }
        return buf;
    }

    void CloseLink()
    {
        try { stream?.Close(); client?.Close(); } catch { /* ignore */ }
        stream = null;
        client = null;
    }

    void OnDestroy()
    {
        CloseLink();
        if (eeCamera != null && eeCamera.targetTexture == rt) eeCamera.targetTexture = null;
        if (rt != null) rt.Release();
    }
}
