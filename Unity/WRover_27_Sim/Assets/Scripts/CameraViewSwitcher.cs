using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Switches the Game view between the follow camera (Main Camera) and the rover's cameras.
/// V = next view, Shift+V = previous view.
///
/// Rover cameras are shown by drawing their RenderTexture full screen, so a camera that
/// RosCameraPublisher uses keeps publishing to ROS while you look through it. Cameras that
/// nothing else renders into get their own texture here, and only render while you view them.
/// Put it on the Main Camera (Tools > WRover > Add Rover Cameras does this).
/// </summary>
public class CameraViewSwitcher : MonoBehaviour
{
    [System.Serializable]
    public class View
    {
        public string label;
        public Camera camera;
    }

    [Tooltip("Empty = the Main Camera first, then every Camera under the rover, labelled by its link name.")]
    public List<View> views = new List<View>();
    [Tooltip("Next view. Hold Shift for the previous view.")]
    public KeyCode nextKey = KeyCode.V;
    [Tooltip("Resolution for rover cameras that don't already render into a texture (e.g. for ROS).")]
    public int textureWidth = 1280;
    public int textureHeight = 800;
    public bool showLabel = true;

    private Camera screenCamera;
    private int current;
    private bool texturesReady;
    private readonly List<Camera> ownedCameras = new List<Camera>();
    private readonly List<RenderTexture> ownedTextures = new List<RenderTexture>();
    private GUIStyle labelStyle;

    void Start()
    {
        screenCamera = Camera.main;
        if (views.Count == 0) AutoFill();
        if (views.Count == 0) Debug.LogWarning("CameraViewSwitcher: no cameras found.", this);
    }

    void AutoFill()
    {
        if (screenCamera != null) views.Add(new View { label = "follow", camera = screenCamera });
        var rover = FindObjectOfType<SwerveDriveController>();
        if (rover == null)
        {
            Debug.LogWarning("CameraViewSwitcher: no SwerveDriveController in the scene, so no rover cameras.", this);
            return;
        }
        foreach (Camera cam in rover.GetComponentsInChildren<Camera>(true))
            if (cam != screenCamera) views.Add(new View { label = LabelFor(cam), camera = cam });
    }

    // camera_link -> "front", camera_front_left_link -> "front_left"
    static string LabelFor(Camera cam)
    {
        string link = cam.transform.parent != null ? cam.transform.parent.name : cam.name;
        if (link == "camera_link") return "front";
        if (link.StartsWith("camera_") && link.EndsWith("_link") && link.Length > 12)
            return link.Substring(7, link.Length - 12);
        return link;
    }

    void Update()
    {
        if (views.Count == 0 || !Input.GetKeyDown(nextKey)) return;
        bool back = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
        current = (current + (back ? views.Count - 1 : 1)) % views.Count;
    }

    void LateUpdate()
    {
        // Done here, not in Start, so every RosCameraPublisher has already set its own texture.
        if (!texturesReady)
        {
            texturesReady = true;
            foreach (View v in views)
            {
                if (v.camera == null || v.camera == screenCamera || v.camera.targetTexture != null) continue;
                var rt = new RenderTexture(textureWidth, textureHeight, 24, RenderTextureFormat.ARGB32,
                                           RenderTextureReadWrite.sRGB);
                rt.Create();
                v.camera.targetTexture = rt;
                ownedTextures.Add(rt);
                ownedCameras.Add(v.camera);
            }
        }
        // Cameras only this script uses render only while they're on screen.
        Camera shown = views.Count > 0 ? views[current].camera : null;
        foreach (Camera cam in ownedCameras)
            if (cam != null) cam.enabled = cam == shown;
    }

    void OnGUI()
    {
        if (views.Count == 0) return;
        GUI.depth = 1;   // lower depth draws on top, so CameraFeedOverlay's picture-in-picture stays visible
        View v = views[current];

        if (v.camera != null && v.camera != screenCamera && v.camera.targetTexture != null)
        {
            var full = new Rect(0, 0, Screen.width, Screen.height);
            GUI.color = Color.black;
            GUI.DrawTexture(full, Texture2D.whiteTexture);
            GUI.color = Color.white;
            GUI.DrawTexture(full, v.camera.targetTexture, ScaleMode.ScaleToFit, false);
        }

        if (showLabel)
        {
            if (labelStyle == null)
                labelStyle = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, normal = { textColor = Color.white } };
            GUI.Label(new Rect(10, Screen.height - 30, 700, 24),
                      $"View {current + 1}/{views.Count}: {v.label}   ({nextKey} next, Shift+{nextKey} previous)", labelStyle);
        }
    }

    void OnDestroy()
    {
        foreach (Camera cam in ownedCameras)
            if (cam != null) { cam.targetTexture = null; cam.enabled = true; }
        foreach (RenderTexture rt in ownedTextures)
            if (rt != null) { rt.Release(); Destroy(rt); }
    }
}
