// TwinPlayback.cs -- plays back the Python digital twin in Unity.
//
// Attach to an empty GameObject in an empty scene and press Play. It builds:
//   * the 2000 x 740 mm table (+ rail if enabled)
//   * both B601-RS arms and the myCobot 280 as joint/link stick models (exact joint
//     positions from the twin -- no CAD import needed)
//   * the 1U debris replica, the bracket, the debris stand-off
//   * the D405 (42 x 42 x 23 mm) with a Unity camera using the D405 field of view,
//     shown as a live picture-in-picture
// and plays the mission with play / pause / speed / time-scrub controls.
//
// To see the real robot meshes as well, import the official URDFs and add
// UrdfJointDriver (see README). Keep the stick models on to check the alignment.
using System.Collections.Generic;
using UnityEngine;

public class TwinPlayback : MonoBehaviour
{
    // Path missions (keys 1 / 2 / 3 or the GUI toolbar):
    //   TableArm        yellow arm on the table in its L pose, debris still (the original mission)
    //   CeilingArm      yellow arm hangs from an overhead gantry (legs on the floor, not the table), debris still
    //   OrbitingDebris  yellow arm on the table, debris turns on a spoke in a vertical circle facing the chaser
    public enum Mission { TableArm, CeilingArm, OrbitingDebris }

    [Header("Mission (1 / 2 / 3)")]
    public Mission mission = Mission.TableArm;
    public float ceilingHeight = 0.80f;   // m above the table top (mission 2); 0.8 puts the debris near mission 1's height
    public float orbitRadius = 0.15f;     // m (mission 3)
    public float orbitPeriod = 20f;       // s per turn, real time: keeps turning while the mission is paused

    [Header("Data (in Assets/StreamingAssets)")]
    public string playbackFile = "unity_playback.csv";
    public string sceneFile = "unity_scene.json";

    [Header("Playback")]
    public bool playing = true;
    public float speed = 10f;          // x real time (the mission is ~10 min)
    public bool loop = true;

    [Header("Display")]
    public bool showStickModels = true;   // turn off once the imported URDF meshes line up
    public bool showLineOfSight = true;
    public bool debrisArmFixedPose = true;   // hold the debris arm in a fixed L pose instead of playing its motion
    public int debrisArmBendFrame = 3;       // frame where the arm bends 90 deg (3 = elbow between the two long links)
    public Vector2Int cameraPreviewSize = new Vector2Int(424, 240);
    // Not yellow/orange: the debris arm is hue ~21 (OpenCV), and the colour detector would merge the two.
    // Green (0.20, 0.80, 0.30) is OpenCV HSV ~(65, 191, 204); debris_tracker's twin.launch.py matches it.
    public Color debrisColor = new Color(0.20f, 0.80f, 0.30f);

    [Header("Manual camera (M to toggle)")]
    public bool manualCamera = false;     // drive the D405 by keyboard; the myCobot follows by IK
    public float moveSpeed = 0.1f;        // m/s (hold Shift for x3)
    public float turnSpeed = 45f;         // deg/s

    [Header("Rail (blue arm base drives towards the yellow arm)")]
    public bool railEnabled = true;
    public float railStopDistance = 0f;    // camera-to-debris distance when the rail stops (0 = hold distance from the scene file)
    public float railMinClearance = 0.45f; // never bring the two arm bases closer than this (m)
    public bool railDoesApproach = true;   // hold the blue arm + myCobot at their starting pose; the rail does the whole approach

    public float MissionTime { get; private set; }
    public CsvTable Data { get; private set; }
    public SceneInfo Scene { get; private set; }

    // ---- for other components (TwinRosBridge) ----
    public Camera D405Camera => d405Cam;
    public RenderTexture D405Texture => d405Tex;
    public Transform Debris => debris;
    public bool ManualActive => manualActive;
    public Vector3 ManualPosition => manualPos;
    public Quaternion ManualRotation => manualRot;
    public float LiveDistance => liveDist;
    public float LivePointingError => liveLos;   // deg

    // Moves the manual camera (only while ManualActive; set manualCamera = true first). The myCobot follows by IK.
    public void SetManualPose(Vector3 pos, Quaternion rot)
    {
        if (!manualActive) return;
        manualPos = pos;
        manualRot = rot;
    }

    // ---- internals ----
    class Chain { public string prefix; public int count; public Transform[] joints; public Transform[] links; }
    readonly List<Chain> chains = new List<Chain>();
    Transform debris, d405Body, bracket, standoff;
    Camera d405Cam;
    RenderTexture d405Tex;
    LineRenderer los;
    int cRail, cCam, cDeb, cBase, cDist, cCmd, cLos, cPhase;
    Material matChaser, matDebrisArm, matMyCobot, matDebris, matCam, matTable, matDark;
    Vector3[] debrisFixed;   // fixed frame positions of the debris arm
    Vector3 debrisFixedPos;  // debris centre at the tip of the fixed arm
    bool manualActive;
    Vector3 manualPos, lastDebPos;
    Quaternion manualRot;
    Vector3 cobotOffWrist, cobotOffFlange;   // myCobot wrist / flange frames in camera coordinates
    Vector3[] cobotPose;                     // last IK solution (warm start)
    float liveDist, liveLos;
    Transform chaserPlate, debrisPlate, ceilingMount, spoke, railObj;
    Vector3 debrisDir;            // horizontal unit vector along the debris arm's last link (towards the chaser)
    float orbitAngle;             // deg
    Mission builtMission;
    float builtCeiling;
    Vector3 railDir;              // horizontal unit vector, blue base -> yellow base
    float railTravel, railArrive; // rail stroke (m) and the mission time it is reached (start of HOLD)

    // ------------------------------------------------------------------ //
    void Start()
    {
        Scene = SceneInfo.Load(sceneFile);
        Data = CsvTable.Load(playbackFile);
        MissionTime = Data.StartTime;

        cRail = Data.Col("rail"); cPhase = Data.Col("phase");
        cCam = Data.Col("cam_pos_x"); cDeb = Data.Col("debris_pos_x"); cBase = Data.Col("mycobotbase_x");
        cDist = Data.Col("cam_dist"); cCmd = Data.Col("d_cmd"); cLos = Data.Col("los_deg");

        matChaser = MakeMat(new Color(0.20f, 0.45f, 0.95f));
        matDebrisArm = MakeMat(new Color(0.85f, 0.65f, 0.15f));
        matMyCobot = MakeMat(new Color(0.85f, 0.85f, 0.88f));
        matDebris = MakeMat(debrisColor);
        matCam = MakeMat(new Color(0.75f, 0.75f, 0.78f));
        matTable = MakeMat(new Color(0.55f, 0.42f, 0.30f));
        matDark = MakeMat(new Color(0.15f, 0.15f, 0.17f));

        BuildTable();
        chains.Add(BuildChain("chaser", Scene.frame_counts.chaser, 0.055f, matChaser));
        chains.Add(BuildChain("debris", Scene.frame_counts.debris, 0.055f, matDebrisArm));
        chains.Add(BuildChain("mycobot", Scene.frame_counts.mycobot, 0.035f, matMyCobot));
        BuildCeilingMount();
        ApplyMission();

        float s = Scene.debris_size;
        debris = Prim(PrimitiveType.Cube, "Debris (1U replica)", matDebris).transform;
        debris.localScale = new Vector3(s, s, s);
        bracket = Prim(PrimitiveType.Cylinder, "Bracket", matDark).transform;
        standoff = Prim(PrimitiveType.Cylinder, "Debris stand-off", matDark).transform;
        spoke = Prim(PrimitiveType.Cylinder, "Debris orbit spoke", matDark).transform;

        var sz = Scene.camera.size_m;
        d405Body = Prim(PrimitiveType.Cube, "D405", matCam).transform;
        d405Body.localScale = new Vector3(sz[0], sz[1], sz[2]);
        var camGo = new GameObject("D405 view");
        camGo.transform.SetParent(d405Body, false);
        camGo.transform.localScale = new Vector3(1f / sz[0], 1f / sz[1], 1f / sz[2]);   // undo body scale
        d405Cam = camGo.AddComponent<Camera>();
        d405Cam.fieldOfView = Scene.camera.vfov_deg;        // Unity FOV is vertical, like the datasheet's 58 deg
        d405Cam.nearClipPlane = Scene.camera.near;
        d405Cam.farClipPlane = Scene.camera.far;
        d405Cam.clearFlags = CameraClearFlags.SolidColor;
        d405Cam.backgroundColor = Color.black;
        d405Tex = new RenderTexture(cameraPreviewSize.x, cameraPreviewSize.y, 24);
        d405Cam.targetTexture = d405Tex;

        los = new GameObject("Line of sight").AddComponent<LineRenderer>();
        los.positionCount = 2;
        los.startWidth = los.endWidth = 0.002f;
        los.material = MakeMat(Color.cyan);

        if (Camera.main != null && Camera.main != d405Cam)
        {
            Camera.main.transform.position = new Vector3(1.3f, 1.1f, -1.2f);
            Camera.main.transform.LookAt(new Vector3(0f, 0.25f, 0.1f));
            Camera.main.nearClipPlane = 0.02f;
            if (Camera.main.GetComponent<OrbitCamera>() == null) Camera.main.gameObject.AddComponent<OrbitCamera>();
        }
        Apply(MissionTime);
    }

    void Update()
    {
        if (Data == null) return;   // scripts were recompiled during Play: stop and press Play again
        if (playing)
        {
            MissionTime += Time.deltaTime * speed;
            if (MissionTime > Data.EndTime) MissionTime = loop ? Data.StartTime : Data.EndTime;
        }
#if ENABLE_LEGACY_INPUT_MANAGER
        if (Input.GetKeyDown(KeyCode.M)) manualCamera = !manualCamera;
        if (Input.GetKeyDown(KeyCode.Alpha1)) mission = Mission.TableArm;
        if (Input.GetKeyDown(KeyCode.Alpha2)) mission = Mission.CeilingArm;
        if (Input.GetKeyDown(KeyCode.Alpha3)) mission = Mission.OrbitingDebris;
#endif
        if (mission != builtMission || ceilingHeight != builtCeiling)
        {
            ApplyMission();
            MissionTime = Data.StartTime;   // replay the approach towards the new debris position
            manualCamera = false;
        }
        if (mission == Mission.OrbitingDebris && orbitPeriod > 0f)
            orbitAngle = Mathf.Repeat(orbitAngle + 360f * Time.deltaTime / orbitPeriod, 360f);
        if (manualCamera && !manualActive) EnterManual(MissionTime);
        manualActive = manualCamera;
        if (manualActive) DriveManual(Time.deltaTime);
        Apply(MissionTime);
    }

    // ------------------------------------------------------------------ //
    // Manual camera: start from the recorded camera pose, pause the mission, then move by keyboard.
    void EnterManual(float t)
    {
        playing = false;
        Vector3 off = RailOffset(t);
        float ta = ArmTime(t);
        Vector3 camPos = Data.Vec(ta, cCam) + off;
        manualPos = camPos;
        manualRot = Quaternion.LookRotation(Data.Vec(ta, cCam + 3), Data.Vec(ta, cCam + 6));
        var ch = chains[2];
        int c0 = Data.Col("mycobot0_x");
        cobotPose = new Vector3[ch.count];
        for (int i = 0; i < ch.count; i++) cobotPose[i] = Data.Vec(ta, c0 + 3 * i) + off;
        var inv = Quaternion.Inverse(manualRot);
        cobotOffWrist = inv * (cobotPose[ch.count - 2] - camPos);
        cobotOffFlange = inv * (cobotPose[ch.count - 1] - camPos);
    }

    void DriveManual(float dt)
    {
#if ENABLE_LEGACY_INPUT_MANAGER
        float k = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift) ? 3f : 1f;
        Vector3 mv = Vector3.zero;   // camera-local: W/S forward-back, A/D left-right, Q/E down-up
        if (Input.GetKey(KeyCode.W)) mv.z += 1; if (Input.GetKey(KeyCode.S)) mv.z -= 1;
        if (Input.GetKey(KeyCode.D)) mv.x += 1; if (Input.GetKey(KeyCode.A)) mv.x -= 1;
        if (Input.GetKey(KeyCode.E)) mv.y += 1; if (Input.GetKey(KeyCode.Q)) mv.y -= 1;
        manualPos += manualRot * mv * moveSpeed * k * dt;

        float a = turnSpeed * k * dt;   // arrows yaw/pitch, Z/C roll
        if (Input.GetKey(KeyCode.LeftArrow)) manualRot = Quaternion.AngleAxis(-a, Vector3.up) * manualRot;
        if (Input.GetKey(KeyCode.RightArrow)) manualRot = Quaternion.AngleAxis(a, Vector3.up) * manualRot;
        if (Input.GetKey(KeyCode.UpArrow)) manualRot *= Quaternion.AngleAxis(-a, Vector3.right);
        if (Input.GetKey(KeyCode.DownArrow)) manualRot *= Quaternion.AngleAxis(a, Vector3.right);
        if (Input.GetKey(KeyCode.Z)) manualRot *= Quaternion.AngleAxis(a, Vector3.forward);
        if (Input.GetKey(KeyCode.C)) manualRot *= Quaternion.AngleAxis(-a, Vector3.forward);

        if (Input.GetKeyDown(KeyCode.F) && (lastDebPos - manualPos).sqrMagnitude > 1e-8f)   // point at the debris
            manualRot = Quaternion.LookRotation(lastDebPos - manualPos, manualRot * Vector3.up);
        if (Input.GetKeyDown(KeyCode.R)) EnterManual(MissionTime);   // back to the recorded pose
#endif
    }

    // myCobot follows the manual camera: FABRIK on the frames up to the wrist (base fixed from the data,
    // link lengths kept); the flange stays rigidly on the camera. Out of reach -> the camera is pulled back.
    Vector3[] SolveCobot(float t)
    {
        var ch = chains[2];
        int n = ch.count, w = n - 2, c0 = Data.Col("mycobot0_x");
        var p = new Vector3[n];
        Vector3 off = RailOffset(t);
        for (int i = 0; i < n; i++) p[i] = Data.Vec(ArmTime(t), c0 + 3 * i) + off;
        var len = new float[w];
        for (int i = 0; i < w; i++) len[i] = Vector3.Distance(p[i], p[i + 1]);

        Vector3 shift = p[0] - cobotPose[0];   // the base moves with the chaser arm / rail as time runs
        for (int i = 0; i < n; i++) cobotPose[i] += shift;

        Vector3 target = manualPos + manualRot * cobotOffWrist, root = p[0];
        for (int iter = 0; iter < 30; iter++)
        {
            cobotPose[w] = target;
            for (int i = w - 1; i >= 0; i--) cobotPose[i] = Toward(cobotPose[i + 1], cobotPose[i], len[i]);
            cobotPose[0] = root;
            for (int i = 0; i < w; i++) cobotPose[i + 1] = Toward(cobotPose[i], cobotPose[i + 1], len[i]);
            if ((cobotPose[w] - target).sqrMagnitude < 1e-8f) break;
        }
        if ((cobotPose[w] - target).sqrMagnitude > 1e-6f) manualPos = cobotPose[w] - manualRot * cobotOffWrist;
        else cobotPose[w] = target;
        cobotPose[n - 1] = manualPos + manualRot * cobotOffFlange;
        return cobotPose;
    }

    static Vector3 Toward(Vector3 from, Vector3 to, float len)
    {
        Vector3 v = to - from;
        float r = v.magnitude;
        return from + (r < 1e-6f ? Vector3.up : v / r) * len;
    }

    // ------------------------------------------------------------------ //
    void Apply(float t)
    {
        Vector3 off = RailOffset(t);
        float ta = ArmTime(t);
        Vector3[] cobotIK = manualActive ? SolveCobot(t) : null;
        foreach (var ch in chains)
        {
            int c0 = Data.Col($"{ch.prefix}0_x");
            var p = new Vector3[ch.count];
            bool onRail = ch.prefix != "debris";   // the rail carries the blue arm and the myCobot
            for (int i = 0; i < ch.count; i++) p[i] = onRail ? Data.Vec(ta, c0 + 3 * i) + off : Data.Vec(t, c0 + 3 * i);
            if (DebrisArmFixed && ch.prefix == "debris") p = debrisFixed;
            if (cobotIK != null && ch.prefix == "mycobot") p = cobotIK;
            for (int i = 0; i < ch.count; i++)
            {
                ch.joints[i].gameObject.SetActive(showStickModels);
                ch.joints[i].position = p[i];
            }
            for (int i = 0; i < ch.count - 1; i++)
            {
                ch.links[i].gameObject.SetActive(showStickModels);
                PlaceBetween(ch.links[i], p[i], p[i + 1], ch.links[i].localScale.x);
            }
        }
        Vector3 camPos = Data.Vec(ta, cCam) + off, camFwd = Data.Vec(ta, cCam + 3), camUp = Data.Vec(ta, cCam + 6);
        Vector3 debPos = Data.Vec(t, cDeb), debFwd = Data.Vec(t, cDeb + 3), debUp = Data.Vec(t, cDeb + 6);
        Vector3 basePos = Data.Vec(ta, cBase) + off;
        Quaternion debRot = Quaternion.LookRotation(debFwd, debUp);
        bool orbit = mission == Mission.OrbitingDebris;
        if (DebrisArmFixed) debPos = debrisFixedPos;
        if (orbit)   // the spoke turns rigidly about the arm's last link: hub = mission 1 debris position
        {
            Quaternion spin = Quaternion.AngleAxis(orbitAngle, debrisDir);
            debPos = debrisFixedPos + spin * (Vector3.up * orbitRadius);
            debRot = spin * debRot;
        }
        if (manualActive) { camPos = manualPos; camFwd = manualRot * Vector3.forward; camUp = manualRot * Vector3.up; }
        lastDebPos = debPos;
        liveDist = Vector3.Distance(camPos, debPos);
        liveLos = Vector3.Angle(camFwd, debPos - camPos);

        d405Body.SetPositionAndRotation(camPos - camFwd.normalized * Scene.camera.size_m[2] * 0.5f,
                                        Quaternion.LookRotation(camFwd, camUp));
        d405Cam.transform.position = camPos;
        debris.SetPositionAndRotation(debPos, debRot);

        // bracket: chaser flange (last chaser frame) -> myCobot base; stand-off: debris flange -> debris centre
        // (mission 3: stand-off -> orbit hub, then the spoke -> debris)
        var chaser = chains[0]; var debArm = chains[1];
        PlaceBetween(bracket, chaser.joints[chaser.count - 1].position, basePos, 0.03f);
        PlaceBetween(standoff, debArm.joints[debArm.count - 1].position, orbit ? debrisFixedPos : debPos, 0.02f);
        PlaceBetween(spoke, debrisFixedPos, debPos, 0.015f);
        bracket.gameObject.SetActive(showStickModels);
        standoff.gameObject.SetActive(showStickModels);
        spoke.gameObject.SetActive(showStickModels && orbit);

        los.enabled = showLineOfSight;
        los.SetPosition(0, camPos);
        los.SetPosition(1, debPos);

        if (chaserPlate != null) chaserPlate.position = SceneInfo.V(Scene.chaser_base.position) + Vector3.up * 0.01f + off;
    }

    // ------------------------------------------------------------------ //
    void OnGUI()
    {
        if (Data == null) return;
        GUILayout.BeginArea(new Rect(10, 10, 330, manualActive ? 355 : 280), GUI.skin.box);
        GUILayout.BeginHorizontal();
        GUILayout.Label("Mission", GUILayout.Width(55));
        mission = (Mission)GUILayout.Toolbar((int)mission, new[] { "1 table", "2 ceiling", "3 orbit" });
        GUILayout.EndHorizontal();
        int phase = Mathf.RoundToInt(Data.Value(MissionTime, cPhase));
        GUILayout.Label($"Mission time  {MissionTime:F1} s   ({(phase == 0 ? "APPROACH" : "HOLD at threshold")})");
        // measured live in the scene (fixed debris arm, rail and manual camera all change it from the recorded values)
        GUILayout.Label($"Camera to debris   {liveDist * 100f:F1} cm{(manualActive ? "  (manual)" : "")}");
        GUILayout.Label($"Pointing error      {liveLos:F2} deg");
        if (railEnabled) GUILayout.Label($"Rail position       {RailOffset(MissionTime).magnitude * 100f:F1} / {railTravel * 100f:F1} cm");
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(playing ? "Pause" : "Play")) playing = !playing;
        if (GUILayout.Button("Restart")) MissionTime = Data.StartTime;
        showStickModels = GUILayout.Toggle(showStickModels, "stick models");
        GUILayout.EndHorizontal();
        GUILayout.Label($"Speed x{speed:F0}");
        speed = GUILayout.HorizontalSlider(speed, 1f, 60f);
        GUILayout.Label("Time");
        MissionTime = GUILayout.HorizontalSlider(MissionTime, Data.StartTime, Data.EndTime);
        manualCamera = GUILayout.Toggle(manualCamera, "manual camera (M)");
        if (manualActive)
            GUILayout.Label("WASD move, Q/E down/up, arrows aim, Z/C roll,\nShift faster, F look at debris, R reset");
        GUILayout.EndArea();

        if (d405Tex != null)
        {
            float w = 424, h = w / ((float)d405Tex.width / d405Tex.height);
            var r = new Rect(Screen.width - w - 10, 10, w, h);
            GUI.DrawTexture(r, d405Tex, ScaleMode.ScaleToFit, false);
            GUI.Label(new Rect(r.x + 6, r.y + 4, w, 20), "D405 view (87 x 58 deg)");
            // image-centre crosshair
            float cx = r.x + w / 2, cy = r.y + h / 2;
            GUI.DrawTexture(new Rect(cx - 10, cy, 20, 1), Texture2D.whiteTexture);
            GUI.DrawTexture(new Rect(cx, cy - 10, 1, 20), Texture2D.whiteTexture);
        }
    }

    // ------------------------------------------------------------------ //
    void BuildTable()
    {
        var T = Scene.table;
        // table length runs along Unity z (= table x), width along Unity x (= -table y)
        var top = Prim(PrimitiveType.Cube, "Table top", matTable).transform;
        top.localScale = new Vector3(T.width, T.thickness, T.length);
        top.position = new Vector3(0, -T.thickness / 2f, 0);
        float legH = 0.72f;
        foreach (var sx in new[] { -1f, 1f })
            foreach (var sz in new[] { -1f, 1f })
            {
                var leg = Prim(PrimitiveType.Cube, "Leg", matDark).transform;
                leg.localScale = new Vector3(0.05f, legH, 0.05f);
                leg.position = new Vector3(sx * (T.width / 2 - 0.05f), -T.thickness - legH / 2, sz * (T.length / 2 - 0.05f));
            }
        foreach (var b in new[] { Scene.chaser_base, Scene.debris_base })
        {
            var plate = Prim(PrimitiveType.Cylinder, "Arm base", matDark).transform;
            plate.localScale = new Vector3(0.14f, 0.01f, 0.14f);
            plate.position = SceneInfo.V(b.position) + Vector3.up * 0.01f;
            if (b == Scene.chaser_base) chaserPlate = plate; else debrisPlate = plate;
        }
    }

    // Overhead gantry for mission 2: a beam across the table at the debris base, on two legs that stand on
    // the floor beside the table (nothing touches the table). Placed by ApplyMission.
    void BuildCeilingMount()
    {
        ceilingMount = new GameObject("Ceiling mount").transform;
        ceilingMount.SetParent(transform, false);
        foreach (var name in new[] { "Gantry beam", "Gantry leg", "Gantry leg" })
            Prim(PrimitiveType.Cube, name, matDark).transform.SetParent(ceilingMount, false);
    }

    void PlaceCeilingMount()
    {
        var T = Scene.table;
        Vector3 b = SceneInfo.V(Scene.debris_base.position);
        float floorY = -T.thickness - 0.72f, top = ceilingHeight + 0.01f + 0.03f;   // beam centre, above the base plate
        float span = T.width + 0.3f, legX = span / 2f - 0.03f, legH = top + 0.03f - floorY;
        var beam = ceilingMount.GetChild(0);
        beam.localScale = new Vector3(span, 0.06f, 0.08f);
        beam.position = new Vector3(0f, top, b.z);
        for (int i = 1; i <= 2; i++)
        {
            var leg = ceilingMount.GetChild(i);
            leg.localScale = new Vector3(0.06f, legH, 0.06f);
            leg.position = new Vector3(i == 1 ? -legX : legX, floorY + legH / 2f, b.z);
        }
    }

    // missions 2 and 3 always hold the debris arm still
    bool DebrisArmFixed => debrisArmFixedPose || mission != Mission.TableArm;

    // (Re)build everything that depends on the mission: debris arm pose, base plate, gantry, rail.
    void ApplyMission()
    {
        bool ceiling = mission == Mission.CeilingArm;
        BuildDebrisFixed();
        PlaceCeilingMount();
        ceilingMount.gameObject.SetActive(ceiling);
        // base plate on the table, or under the gantry beam
        debrisPlate.position = SceneInfo.V(Scene.debris_base.position) + Vector3.up * (ceiling ? ceilingHeight + 0.005f : 0.01f);
        SetupRail();
        orbitAngle = 0f;
        builtMission = mission;
        builtCeiling = ceilingHeight;
    }

    // Fixed L-shaped debris arm: keep the base and the real link lengths (from the first sample),
    // go straight up to the bend frame, then turn 90 deg and run horizontally towards the chaser
    // so its camera can reach the debris held at the tip (same stand-off distance as the data).
    // Mission 2 mirrors it: the base hangs from the ceiling and the arm goes straight down to the bend.
    void BuildDebrisFixed()
    {
        int n = Scene.frame_counts.debris, c0 = Data.Col("debris0_x");
        float t0 = Data.StartTime;
        int bend = Mathf.Clamp(debrisArmBendFrame, 1, n - 1);
        Vector3 toChaser = SceneInfo.V(Scene.chaser_base.position) - SceneInfo.V(Scene.debris_base.position);
        toChaser.y = 0f;
        toChaser = toChaser.sqrMagnitude > 1e-6f ? toChaser.normalized : Vector3.back;
        debrisDir = toChaser;
        bool ceiling = mission == Mission.CeilingArm;

        debrisFixed = new Vector3[n];
        debrisFixed[0] = Data.Vec(t0, c0) + (ceiling ? Vector3.up * ceilingHeight : Vector3.zero);
        for (int i = 1; i < n; i++)
        {
            float len = Vector3.Distance(Data.Vec(t0, c0 + 3 * (i - 1)), Data.Vec(t0, c0 + 3 * i));
            debrisFixed[i] = debrisFixed[i - 1] + (i <= bend ? (ceiling ? Vector3.down : Vector3.up) : toChaser) * len;
        }
        float standoffLen = Vector3.Distance(Data.Vec(t0, c0 + 3 * (n - 1)), Data.Vec(t0, cDeb));
        debrisFixedPos = debrisFixed[n - 1] + toChaser * standoffLen;
    }

    // Rail: a straight horizontal track that carries the camera head-on towards the debris on the yellow arm.
    // The stroke is chosen so that the camera (arm held at its starting pose, or following its recorded approach
    // if railDoesApproach is off) ends the APPROACH phase at the stop distance from the debris, and the bases
    // keep railMinClearance apart. The base eases along it during APPROACH, then holds.
    void SetupRail()
    {
        Vector3 a = SceneInfo.V(Scene.chaser_base.position), b = SceneInfo.V(Scene.debris_base.position);

        railArrive = Data.EndTime;
        for (int i = 0; i < Data.rows.Length; i++)
            if (Data.rows[i][cPhase] > 0.5f) { railArrive = Data.times[i]; break; }

        float stop = railStopDistance > 0f ? railStopDistance : Scene.hold_distance;
        Vector3 cam = Data.Vec(ArmTime(railArrive), cCam);
        Vector3 deb = DebrisArmFixed ? debrisFixedPos : Data.Vec(railArrive, cDeb);   // mission 3: the orbit hub
        railDir = deb - cam; railDir.y = 0f;
        if (railDir.sqrMagnitude < 1e-6f) { railDir = b - a; railDir.y = 0f; }
        railDir = railDir.sqrMagnitude > 1e-6f ? railDir.normalized : Vector3.forward;

        Vector3 d = deb - cam;
        float along = Vector3.Dot(d, railDir), side2 = Mathf.Max(0f, d.sqrMagnitude - along * along);
        railTravel = Mathf.Max(0f, along - Mathf.Sqrt(Mathf.Max(0f, stop * stop - side2)));   // |cam + railDir*s - deb| = stop
        while (railTravel > 0f && Vector3.Distance(Flat(a + railDir * railTravel), Flat(b)) < railMinClearance)
            railTravel = Mathf.Max(0f, railTravel - 0.005f);

        if (railObj == null) railObj = Prim(PrimitiveType.Cube, "Linear rail", matDark).transform;   // track under the moving base
        railObj.gameObject.SetActive(railEnabled);
        railObj.localScale = new Vector3(0.08f, 0.01f, railTravel + 0.2f);
        railObj.rotation = Quaternion.LookRotation(railDir, Vector3.up);
        railObj.position = a + railDir * (railTravel / 2f) + Vector3.up * 0.005f;
    }

    static Vector3 Flat(Vector3 v) { v.y = 0f; return v; }

    // time used to sample the blue arm / myCobot / camera from the recording
    float ArmTime(float t) { return railEnabled && railDoesApproach ? Data.StartTime : t; }

    Vector3 RailOffset(float t)
    {
        if (!railEnabled || railTravel <= 0f) return Vector3.zero;
        float u = Mathf.InverseLerp(Data.StartTime, railArrive, t);
        return railDir * (railTravel * Mathf.SmoothStep(0f, 1f, u));
    }

    Chain BuildChain(string prefix, int count, float thickness, Material mat)
    {
        var root = new GameObject(prefix + " arm (stick model)").transform;
        root.SetParent(transform, false);
        var ch = new Chain { prefix = prefix, count = count, joints = new Transform[count], links = new Transform[count - 1] };
        for (int i = 0; i < count; i++)
        {
            var j = Prim(PrimitiveType.Sphere, $"{prefix} frame {i}", matDark).transform;
            j.SetParent(root, false);
            j.localScale = Vector3.one * thickness * 1.25f;
            ch.joints[i] = j;
        }
        for (int i = 0; i < count - 1; i++)
        {
            var l = Prim(PrimitiveType.Cylinder, $"{prefix} link {i + 1}", mat).transform;
            l.SetParent(root, false);
            l.localScale = new Vector3(thickness, 0.01f, thickness);
            ch.links[i] = l;
        }
        return ch;
    }

    static void PlaceBetween(Transform cyl, Vector3 a, Vector3 b, float thickness)
    {
        Vector3 d = b - a;
        float len = d.magnitude;
        if (len < 1e-5f) { cyl.localScale = Vector3.zero; return; }
        cyl.position = (a + b) / 2f;
        cyl.rotation = Quaternion.FromToRotation(Vector3.up, d / len);
        cyl.localScale = new Vector3(thickness, len / 2f, thickness);   // Unity cylinder is 2 m tall
    }

    GameObject Prim(PrimitiveType type, string name, Material mat)
    {
        var go = GameObject.CreatePrimitive(type);
        go.name = name;
        go.transform.SetParent(transform, false);
        var col = go.GetComponent<Collider>();
        if (col != null) Destroy(col);
        go.GetComponent<Renderer>().sharedMaterial = mat;
        return go;
    }

    static Material MakeMat(Color c)
    {
        // works in the Built-in render pipeline and in URP
        Shader sh = Shader.Find("Universal Render Pipeline/Lit");
        if (sh == null) sh = Shader.Find("Standard");
        if (sh == null) sh = Shader.Find("Unlit/Color");
        var m = new Material(sh) { color = c };
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
        return m;
    }
}
