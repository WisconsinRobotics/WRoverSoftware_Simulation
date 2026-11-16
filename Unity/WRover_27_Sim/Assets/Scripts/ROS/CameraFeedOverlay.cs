using UnityEngine;

/// <summary>
/// Shows a camera's image as a picture-in-picture box in a corner of the Game view.
/// Meant for the rover camera used by RosCameraPublisher: it draws the same RenderTexture that
/// is sent to ROS, so it costs no extra rendering and shows exactly what the nodes receive.
/// Put it on the RosConnector GameObject (or anywhere). Press the toggle key to hide/show.
/// </summary>
public class CameraFeedOverlay : MonoBehaviour
{
    public enum Corner { TopLeft, TopRight, BottomLeft, BottomRight }

    [Tooltip("Empty = the Camera used by RosCameraPublisher in the scene.")]
    public Camera sourceCamera;
    public Corner corner = Corner.TopRight;
    [Range(0.1f, 0.6f)] [Tooltip("Width of the box as a fraction of the Game view width.")]
    public float widthFraction = 0.3f;
    public int marginPx = 10;
    public string label = "Rover camera";
    public KeyCode toggleKey = KeyCode.C;
    public bool visible = true;

    private GUIStyle labelStyle;

    void Start()
    {
        if (sourceCamera == null)
        {
            var pub = FindObjectOfType<RosCameraPublisher>();
            if (pub != null) sourceCamera = pub.sensorCamera;
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(toggleKey)) visible = !visible;
    }

    void OnGUI()
    {
        if (!visible || sourceCamera == null) return;
        // RosCameraPublisher sets targetTexture in its Start(); until then there is nothing to show.
        Texture tex = sourceCamera.targetTexture;
        if (tex == null) return;

        float w = Screen.width * widthFraction;
        float h = w * tex.height / tex.width;
        float x = (corner == Corner.TopLeft || corner == Corner.BottomLeft) ? marginPx : Screen.width - w - marginPx;
        float y = (corner == Corner.TopLeft || corner == Corner.TopRight) ? marginPx : Screen.height - h - marginPx;

        // Thin dark frame so the box stands out from the scene.
        GUI.color = new Color(0f, 0f, 0f, 0.8f);
        GUI.DrawTexture(new Rect(x - 2, y - 2, w + 4, h + 4), Texture2D.whiteTexture);
        GUI.color = Color.white;
        GUI.DrawTexture(new Rect(x, y, w, h), tex, ScaleMode.StretchToFill, false);

        if (!string.IsNullOrEmpty(label))
        {
            if (labelStyle == null)
                labelStyle = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, normal = { textColor = Color.white } };
            GUI.Label(new Rect(x + 6, y + 4, w - 12, 22), $"{label}  ({toggleKey} to hide)", labelStyle);
        }
    }
}
