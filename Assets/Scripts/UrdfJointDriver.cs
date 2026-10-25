// UrdfJointDriver.cs -- moves a robot imported with the Unity URDF Importer
// (com.unity.robotics.urdf-importer) using the joint angles from joint_trajectory.csv.
//
// Add one UrdfJointDriver per imported robot (3 in total):
//   robot = ChaserB601  -> the B601 that carries the myCobot
//   robot = DebrisB601  -> the B601 that holds the debris
//   robot = MyCobot     -> the myCobot 280 (its base follows the chaser flange)
// Drag the imported robot's root object into "Robot Root" and the TwinPlayback object
// into "Playback". Time stays in sync with TwinPlayback.
//
// ALIGNMENT CHECK: keep TwinPlayback's stick models visible. The imported meshes should
// sit exactly on them. If one joint moves the wrong way, set its Joint Sign to -1.
using UnityEngine;

public class UrdfJointDriver : MonoBehaviour
{
    public enum Robot { ChaserB601, DebrisB601, MyCobot }

    public TwinPlayback playback;
    public Robot robot = Robot.ChaserB601;
    public GameObject robotRoot;
    public string jointFile = "joint_trajectory.csv";
    public bool placeRoot = true;               // move the robot base to where the twin puts it
    public float[] jointSign = { 1, 1, 1, 1, 1, 1 };
    public float[] jointOffsetDeg = { 0, 0, 0, 0, 0, 0 };

    ArticulationBody rootBody;
    ArticulationBody[] joints;
    int[] cols;
    CsvTable data;
    int cRail = -1, cBase = -1;

    void Start()
    {
        if (playback == null) playback = FindObjectOfType<TwinPlayback>();
        data = CsvTable.Load(jointFile);
        var scene = SceneInfo.Load(playback.sceneFile);

        string[] colNames, links;
        if (robot == Robot.MyCobot) { colNames = scene.joint_columns.mycobot; links = scene.urdf_child_links.mycobot; }
        else if (robot == Robot.ChaserB601) { colNames = scene.joint_columns.chaser; links = scene.urdf_child_links.b601; }
        else { colNames = scene.joint_columns.debris; links = scene.urdf_child_links.b601; }

        cols = new int[colNames.Length];
        joints = new ArticulationBody[links.Length];
        for (int i = 0; i < links.Length; i++)
        {
            cols[i] = data.Col(colNames[i]);
            var t = FindDeep(robotRoot.transform, links[i]);
            if (t == null) { Debug.LogError($"[{name}] link '{links[i]}' not found under {robotRoot.name}"); continue; }
            joints[i] = t.GetComponent<ArticulationBody>();
        }
        foreach (var ab in robotRoot.GetComponentsInChildren<ArticulationBody>())
        {
            ab.useGravity = false;                       // pure kinematic playback
            if (ab.isRoot) { rootBody = ab; ab.immovable = true; }
        }
        if (data.Has("rail_shift_m")) cRail = data.Col("rail_shift_m");
        if (robot == Robot.MyCobot) cBase = playback.Data.Col("mycobotbase_x");
    }

    void LateUpdate()
    {
        if (playback == null || playback.Data == null) return;
        float t = playback.MissionTime;

        if (placeRoot) PlaceRoot(t);

        for (int i = 0; i < joints.Length; i++)
        {
            var ab = joints[i];
            if (ab == null) continue;
            float deg = data.Value(t, cols[i]) * jointSign[i] + jointOffsetDeg[i];
            ab.jointPosition = new ArticulationReducedSpace(deg * Mathf.Deg2Rad);   // teleport (exact)
            var drive = ab.xDrive;
            drive.target = deg;                                                     // and hold it there
            ab.xDrive = drive;
        }
    }

    void PlaceRoot(float t)
    {
        var scene = playback.Scene;
        Vector3 pos; Quaternion rot;
        if (robot == Robot.MyCobot)
        {
            pos = playback.Data.Vec(t, cBase);            // myCobot base rides on the chaser flange
            rot = Quaternion.identity;                    // upright, reaching toward the debris
        }
        else
        {
            var b = robot == Robot.ChaserB601 ? scene.chaser_base : scene.debris_base;
            pos = SceneInfo.V(b.position);
            if (robot == Robot.ChaserB601 && cRail >= 0)
                pos += Vector3.forward * data.Value(t, cRail);   // rail runs along table x = Unity z
            rot = Quaternion.Euler(0f, b.yaw_deg, 0f);
        }
        if (rootBody != null) rootBody.TeleportRoot(pos, rot);
        else robotRoot.transform.SetPositionAndRotation(pos, rot);
    }

    static Transform FindDeep(Transform parent, string n)
    {
        if (parent.name == n) return parent;
        foreach (Transform c in parent)
        {
            var r = FindDeep(c, n);
            if (r != null) return r;
        }
        return null;
    }
}
