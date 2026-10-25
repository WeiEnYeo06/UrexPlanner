// RosMsgs.cs -- ROS 2 message layouts for the rosbridge JSON protocol, and Unity <-> ROS coordinate conversion.
//
// Field names match the ROS 2 .msg definitions exactly (JsonUtility serialises public fields by name).
// ROS 2 differs from ROS 1 here: std_msgs/Header has no seq, and its stamp is {sec, nanosec}.
// uint8[] fields (Image.data) travel as base64 strings, which rosbridge decodes.
//
// Frames:
//   Unity world      left-handed, x right, y up, z forward (metres; origin = table-top centre)
//   ROS world        REP-103, right-handed, x forward, y left, z up  ->  (x, y, z)_ros = (z, -x, y)_unity
//                    (this is the table frame of the Python twin: table x = Unity z, table y = -Unity x)
//   Optical frame    REP-103 *_optical_frame: x right, y down, z forward  ->  (x, y, z)_opt = (x, -y, z)_unity camera
using System;
using UnityEngine;

namespace RosMsgs
{
    [Serializable] public class TimeMsg
    {
        public int sec;
        public int nanosec;

        static readonly long EpochTicks = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc).Ticks;

        // Wall clock (UTC). ROS 2 nodes use the same clock unless use_sim_time is set.
        public static TimeMsg Now()
        {
            long t = DateTime.UtcNow.Ticks - EpochTicks;   // 100 ns ticks
            return new TimeMsg { sec = (int)(t / 10000000), nanosec = (int)(t % 10000000) * 100 };
        }
    }

    [Serializable] public class HeaderMsg { public TimeMsg stamp; public string frame_id; }
    [Serializable] public class Vector3Msg { public double x, y, z; }
    [Serializable] public class PointMsg { public double x, y, z; }
    [Serializable] public class QuaternionMsg { public double x, y, z, w = 1; }
    [Serializable] public class PoseMsg { public PointMsg position = new PointMsg(); public QuaternionMsg orientation = new QuaternionMsg(); }
    [Serializable] public class PoseStampedMsg { public HeaderMsg header; public PoseMsg pose = new PoseMsg(); }
    [Serializable] public class TwistMsg { public Vector3Msg linear = new Vector3Msg(), angular = new Vector3Msg(); }
    [Serializable] public class TwistStampedMsg { public HeaderMsg header; public TwistMsg twist = new TwistMsg(); }
    [Serializable] public class TransformMsg { public Vector3Msg translation = new Vector3Msg(); public QuaternionMsg rotation = new QuaternionMsg(); }
    [Serializable] public class TransformStampedMsg { public HeaderMsg header; public string child_frame_id; public TransformMsg transform = new TransformMsg(); }
    [Serializable] public class TFMessageMsg { public TransformStampedMsg[] transforms; }

    [Serializable] public class ImageMsg
    {
        public HeaderMsg header;
        public int height, width;
        public string encoding;
        public int is_bigendian;
        public int step;
        public string data;   // base64
    }

    [Serializable] public class RegionOfInterestMsg { public int x_offset, y_offset, height, width; public bool do_rectify; }

    [Serializable] public class CameraInfoMsg
    {
        public HeaderMsg header;
        public int height, width;
        public string distortion_model = "plumb_bob";
        public double[] d = new double[5];
        public double[] k = new double[9];
        public double[] r = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        public double[] p = new double[12];
        public int binning_x, binning_y;
        public RegionOfInterestMsg roi = new RegionOfInterestMsg();
    }

    // Wrappers for parsing incoming {"op":"publish","topic":...,"msg":{...}} (JsonUtility needs concrete types).
    [Serializable] public class TwistStampedIn { public string topic; public TwistStampedMsg msg; }
    [Serializable] public class PoseStampedIn { public string topic; public PoseStampedMsg msg; }

    public static class RosConv
    {
        // ---- Unity world <-> ROS world ----
        public static Vector3 ToRos(Vector3 u) { return new Vector3(u.z, -u.x, u.y); }

        // A change of handedness maps a rotation about axis a to a rotation about -C a (C = the axis mapping).
        public static Quaternion ToRos(Quaternion q) { return new Quaternion(-q.z, q.x, -q.y, q.w); }

        // ---- Unity camera-local <-> optical ----
        public static Vector3 OpticalToUnityLocal(Vector3 p) { return new Vector3(p.x, -p.y, p.z); }

        // angular velocity / rotation vector: -M w with M = diag(1, -1, 1)
        public static Vector3 OpticalRateToUnityLocal(Vector3 w) { return new Vector3(-w.x, w.y, -w.z); }

        public static Quaternion OpticalToUnityLocal(Quaternion q) { return new Quaternion(-q.x, q.y, -q.z, q.w); }

        // Orientation of the camera's optical frame in the ROS world, from the Unity camera rotation.
        public static Quaternion OpticalFrameInRosWorld(Quaternion camRot)
        {
            return FromAxes(ToRos(camRot * Vector3.right), ToRos(camRot * Vector3.down), ToRos(camRot * Vector3.forward));
        }

        // Quaternion of the rotation matrix whose columns are the unit axes ex, ey, ez.
        public static Quaternion FromAxes(Vector3 ex, Vector3 ey, Vector3 ez)
        {
            float m00 = ex.x, m10 = ex.y, m20 = ex.z, m01 = ey.x, m11 = ey.y, m21 = ey.z, m02 = ez.x, m12 = ez.y, m22 = ez.z;
            float tr = m00 + m11 + m22, s;
            if (tr > 0f)
            {
                s = Mathf.Sqrt(tr + 1f) * 2f;
                return new Quaternion((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s);
            }
            if (m00 > m11 && m00 > m22)
            {
                s = Mathf.Sqrt(1f + m00 - m11 - m22) * 2f;
                return new Quaternion(0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s);
            }
            if (m11 > m22)
            {
                s = Mathf.Sqrt(1f + m11 - m00 - m22) * 2f;
                return new Quaternion((m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s);
            }
            s = Mathf.Sqrt(1f + m22 - m00 - m11) * 2f;
            return new Quaternion((m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s);
        }

        // ---- Unity -> message ----
        public static Vector3Msg V(Vector3 v) { return new Vector3Msg { x = v.x, y = v.y, z = v.z }; }
        public static PointMsg P(Vector3 v) { return new PointMsg { x = v.x, y = v.y, z = v.z }; }
        public static QuaternionMsg Q(Quaternion q) { return new QuaternionMsg { x = q.x, y = q.y, z = q.z, w = q.w }; }

        // ---- message -> Unity (values stay in the message's frame) ----
        public static Vector3 V(Vector3Msg v) { return new Vector3((float)v.x, (float)v.y, (float)v.z); }
        public static Vector3 V(PointMsg v) { return new Vector3((float)v.x, (float)v.y, (float)v.z); }
        public static Quaternion Q(QuaternionMsg q) { return new Quaternion((float)q.x, (float)q.y, (float)q.z, (float)q.w); }
    }
}
