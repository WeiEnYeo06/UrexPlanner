// TwinData.cs -- loads the files exported by python/twin_sim.py
// Put unity_playback.csv, joint_trajectory.csv and unity_scene.json in Assets/StreamingAssets/
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

[Serializable] public class TableInfo { public float length, width, thickness; }
[Serializable] public class BaseInfo { public float[] position; public float yaw_deg; }
[Serializable] public class RailInfo { public bool enabled; public float[] axis_unity; public float travel; }
[Serializable] public class CameraInfo { public string model; public float vfov_deg, hfov_deg, aspect, near, far; public float[] size_m; }
[Serializable] public class FrameCounts { public int chaser, debris, mycobot; }
[Serializable] public class RobotLists { public string[] b601; public string[] mycobot; }
[Serializable] public class JointColumns { public string[] chaser, debris, mycobot; }

[Serializable]
public class SceneInfo
{
    public string units;
    public TableInfo table;
    public BaseInfo chaser_base, debris_base;
    public RailInfo rail;
    public float debris_size;
    public CameraInfo camera;
    public float hold_distance;
    public JointColumns joint_columns;
    public RobotLists urdf_joint_names;
    public RobotLists urdf_child_links;
    public FrameCounts frame_counts;

    public static SceneInfo Load(string fileName)
    {
        string path = Path.Combine(Application.streamingAssetsPath, fileName);
        return JsonUtility.FromJson<SceneInfo>(File.ReadAllText(path));
    }

    public static Vector3 V(float[] a) { return new Vector3(a[0], a[1], a[2]); }
}

/// A numeric CSV with a header row: column lookup by name, rows sorted by time (column 0),
/// linear interpolation between rows.
public class CsvTable
{
    public string[] header;
    public float[][] rows;
    public float[] times;
    readonly Dictionary<string, int> index = new Dictionary<string, int>();
    static readonly Dictionary<string, CsvTable> cache = new Dictionary<string, CsvTable>();

    public static CsvTable Load(string fileName)
    {
        if (cache.TryGetValue(fileName, out var t)) return t;
        string path = Path.Combine(Application.streamingAssetsPath, fileName);
        string[] lines = File.ReadAllLines(path);
        var table = new CsvTable { header = lines[0].Trim().Split(',') };
        for (int i = 0; i < table.header.Length; i++) table.index[table.header[i]] = i;
        var rows = new List<float[]>();
        for (int i = 1; i < lines.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(lines[i])) continue;
            string[] parts = lines[i].Split(',');
            var r = new float[parts.Length];
            for (int j = 0; j < parts.Length; j++)
                r[j] = float.Parse(parts[j], CultureInfo.InvariantCulture);
            rows.Add(r);
        }
        table.rows = rows.ToArray();
        table.times = new float[table.rows.Length];
        for (int i = 0; i < table.rows.Length; i++) table.times[i] = table.rows[i][0];
        cache[fileName] = table;
        Debug.Log($"[Twin] loaded {fileName}: {table.rows.Length} rows, {table.header.Length} columns");
        return table;
    }

    public int Col(string name)
    {
        if (!index.TryGetValue(name, out int c))
            throw new ArgumentException($"column '{name}' not found");
        return c;
    }

    public bool Has(string name) { return index.ContainsKey(name); }
    public float StartTime { get { return times[0]; } }
    public float EndTime { get { return times[times.Length - 1]; } }

    /// row index i and blend a such that t lies between rows i and i+1
    public void Locate(float t, out int i, out float a)
    {
        if (t <= times[0]) { i = 0; a = 0; return; }
        if (t >= times[times.Length - 1]) { i = times.Length - 2; a = 1; return; }
        int lo = 0, hi = times.Length - 1;
        while (hi - lo > 1) { int mid = (lo + hi) / 2; if (times[mid] <= t) lo = mid; else hi = mid; }
        i = lo;
        a = (t - times[lo]) / Mathf.Max(1e-6f, times[lo + 1] - times[lo]);
    }

    public float Value(float t, int col)
    {
        Locate(t, out int i, out float a);
        return Mathf.Lerp(rows[i][col], rows[i + 1][col], a);
    }

    public Vector3 Vec(float t, int col)
    {
        Locate(t, out int i, out float a);
        var p = new Vector3(rows[i][col], rows[i][col + 1], rows[i][col + 2]);
        var q = new Vector3(rows[i + 1][col], rows[i + 1][col + 1], rows[i + 1][col + 2]);
        return Vector3.Lerp(p, q, a);
    }
}
