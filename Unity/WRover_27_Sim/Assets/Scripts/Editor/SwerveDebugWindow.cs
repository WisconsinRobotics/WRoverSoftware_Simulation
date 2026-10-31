using UnityEditor;
using UnityEngine;

/// <summary>
/// Live debug panel for SwerveDriveController. Open with Tools > Rover > Swerve Debug and dock it
/// anywhere. Independent of the Game view, so its resolution / Scale slider don't affect it.
/// Uses the selected rover if one is selected, otherwise the first one in the scene.
/// Must live in a folder named "Editor".
/// </summary>
public class SwerveDebugWindow : EditorWindow
{
    [MenuItem("Tools/Rover/Swerve Debug")]
    static void Open() => GetWindow<SwerveDebugWindow>("Swerve Debug");

    const float LabelWidth = 78f;
    const float CellWidth = 74f;
    const float SteerWarnDeg = 5f;   // highlight modules whose steering error is above this

    SwerveDriveController ctrl;
    Vector2 scroll;
    GUIStyle num, numWarn, colHeader, section;

    // ~10 Hz refresh while playing
    void OnInspectorUpdate()
    {
        if (Application.isPlaying) Repaint();
    }

    void OnGUI()
    {
        InitStyles();

        if (!Application.isPlaying)
        {
            EditorGUILayout.HelpBox("Enter Play mode to see live data.", MessageType.Info);
            return;
        }

        FindController();
        if (ctrl == null)
        {
            EditorGUILayout.HelpBox("No SwerveDriveController found in the scene.", MessageType.Warning);
            return;
        }
        if (!ctrl.IsReady)
        {
            EditorGUILayout.HelpBox("Controller is not ready. Check the Console for setup errors.", MessageType.Warning);
            return;
        }

        scroll = EditorGUILayout.BeginScrollView(scroll);

        DrawStatus();
        DrawVelocity();
        DrawPose();
        DrawModules();

        EditorGUILayout.Space(8);
        EditorGUILayout.HelpBox(
            "W/S forward/back   A/D strafe   Q/E rotate   Space stop\n" +
            "Click the Game view first: keys only register while it has focus.",
            MessageType.None);

        EditorGUILayout.EndScrollView();
    }

    // ------------------------------------------------------------------ sections

    void DrawStatus()
    {
        EditorGUILayout.BeginHorizontal();
        EditorGUILayout.LabelField("Source", EditorStyles.boldLabel, GUILayout.Width(LabelWidth));
        EditorGUILayout.LabelField(ctrl.ActiveSource);
        if (GUILayout.Button("Select rover", EditorStyles.miniButton, GUILayout.Width(90)))
        {
            Selection.activeGameObject = ctrl.gameObject;
            EditorGUIUtility.PingObject(ctrl.gameObject);
        }
        EditorGUILayout.EndHorizontal();
    }

    void DrawVelocity()
    {
        Section("Velocity (robot frame)");
        HeaderRow("vx  m/s", "vy  m/s", "|v|  m/s", "wz  rad/s", "wz  deg/s");
        VelocityRow("Requested", ctrl.RequestedCommand);
        VelocityRow("Cmd", ctrl.AppliedCommand);
        VelocityRow("Odom", ctrl.OdometryTwist);
    }

    void DrawPose()
    {
        Section("Odometry pose (odom frame)");
        HeaderRow("x  m", "y  m", "yaw  deg");
        Vector3 p = ctrl.OdometryPose;
        Row("Pose", num, p.x.ToString("F2"), p.y.ToString("F2"), (p.z * Mathf.Rad2Deg).ToString("F1"));
    }

    void DrawModules()
    {
        Section("Modules");
        HeaderRow("steer °", "cmd °", "error °", "wheel m/s", "target m/s", "motor RPM", "torque lim Nm");

        foreach (var m in ctrl.modules)
        {
            if (m == null) continue;
            float meas = m.measuredHeading * Mathf.Rad2Deg;
            float cmd = m.commandedHeading * Mathf.Rad2Deg;
            float err = Mathf.DeltaAngle(meas, cmd);

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField(m.id.ToUpperInvariant(), EditorStyles.boldLabel, GUILayout.Width(LabelWidth));
            Cell(meas.ToString("F1"), num);
            Cell(cmd.ToString("F1"), num);
            Cell(err.ToString("F1"), Mathf.Abs(err) > SteerWarnDeg ? numWarn : num);
            Cell(m.measuredSpeed.ToString("F2"), num);
            Cell(m.targetSpeed.ToString("F2"), num);
            Cell(m.currentMotorRPM.ToString("F0"), num);
            Cell(m.currentAxleTorque.ToString("F1"), num);
            EditorGUILayout.EndHorizontal();
        }
    }

    // ------------------------------------------------------------------ helpers

    void VelocityRow(string label, Vector3 v)
    {
        float speed = new Vector2(v.x, v.y).magnitude;
        Row(label, num,
            v.x.ToString("F2"), v.y.ToString("F2"), speed.ToString("F2"),
            v.z.ToString("F2"), (v.z * Mathf.Rad2Deg).ToString("F1"));
    }

    void Section(string title)
    {
        EditorGUILayout.Space(6);
        EditorGUILayout.LabelField(title, section);
        Rect r = GUILayoutUtility.GetRect(1, 1, GUILayout.ExpandWidth(true));
        EditorGUI.DrawRect(r, new Color(0.5f, 0.5f, 0.5f, 0.4f));
    }

    void HeaderRow(params string[] titles)
    {
        EditorGUILayout.BeginHorizontal();
        GUILayout.Space(LabelWidth + 3);
        foreach (var t in titles) Cell(t, colHeader);
        EditorGUILayout.EndHorizontal();
    }

    void Row(string label, GUIStyle style, params string[] cells)
    {
        EditorGUILayout.BeginHorizontal();
        EditorGUILayout.LabelField(label, EditorStyles.boldLabel, GUILayout.Width(LabelWidth));
        foreach (var c in cells) Cell(c, style);
        EditorGUILayout.EndHorizontal();
    }

    void Cell(string text, GUIStyle style) =>
        EditorGUILayout.LabelField(text, style, GUILayout.Width(CellWidth));

    void FindController()
    {
        // Prefer the selected rover (handy with several rovers), else keep / find one.
        var go = Selection.activeGameObject;
        var selected = go != null ? go.GetComponentInParent<SwerveDriveController>() : null;
        if (selected != null) ctrl = selected;
        else if (ctrl == null) ctrl = FindObjectOfType<SwerveDriveController>();
    }

    void InitStyles()
    {
        if (num != null) return;
        num = new GUIStyle(EditorStyles.label) { alignment = TextAnchor.MiddleRight };
        numWarn = new GUIStyle(num) { fontStyle = FontStyle.Bold };
        numWarn.normal.textColor = new Color(1f, 0.7f, 0.2f);
        colHeader = new GUIStyle(EditorStyles.miniLabel) { alignment = TextAnchor.MiddleRight };
        section = new GUIStyle(EditorStyles.boldLabel) { fontSize = 13 };
    }
}
