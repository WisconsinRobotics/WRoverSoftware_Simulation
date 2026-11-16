using System;
using System.Reflection;
using RosSharp.RosBridgeClient;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;
using CameraInfo = RosSharp.RosBridgeClient.MessageTypes.Sensor.CameraInfo;
using CompressedImage = RosSharp.RosBridgeClient.MessageTypes.Sensor.CompressedImage;
using RosHeader = RosSharp.RosBridgeClient.MessageTypes.Std.Header;

/// <summary>
/// Simulated rover camera over rosbridge (ROS#).
/// Publishes sensor_msgs/CompressedImage (JPEG) + sensor_msgs/CameraInfo with the same stamp.
/// The Camera renders into its own RenderTexture (so it never touches the Game view) and the
/// pixels are read back asynchronously, so it works on Built-in, URP and HDRP.
/// Put it on the RosConnector GameObject, like the other Ros*Publisher scripts.
///
/// ROS side, if your nodes want raw images:
///   ros2 run image_transport republish compressed raw --ros-args
///       -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_raw
/// </summary>
[RequireComponent(typeof(RosConnector))]
public class RosCameraPublisher : MonoBehaviour
{
    [UnityEngine.Header("Camera")]
    [Tooltip("Empty = first Camera found under the rover (the GameObject with SwerveDriveController).")]
    public Camera sensorCamera;
    [Tooltip("Resolution of the published image. Same aspect as the calibration; the intrinsics are scaled to it.")]
    public int width = 1280;
    public int height = 800;
    [Range(1, 100)] public int jpegQuality = 75;

    [UnityEngine.Header("Intrinsics (real camera calibration)")]
    [Tooltip("On = render and publish with the calibrated K below. Off = use the Camera's FOV.")]
    public bool useCalibratedIntrinsics = true;
    [Tooltip("Resolution the calibration was done at.")]
    public int calibWidth = 1280;
    public int calibHeight = 800;
    public double fx = 569.71661377;
    public double fy = 569.48486328;
    public double cx = 622.4732666;
    public double cy = 407.38534546;
    [Tooltip("Only used when the calibrated intrinsics are off. Horizontal FOV in degrees, 0 = keep the Camera's FOV.")]
    public float horizontalFovDeg = 0f;

    [UnityEngine.Header("Publishing")]
    public float publishRateHz = 15f;
    [Tooltip("Tick only if the image shows upside down in rqt_image_view.")]
    public bool flipVertical = false;

    [UnityEngine.Header("ROS")]
    [Tooltip("Use the same topic names as the real camera driver so the nodes run unchanged.")]
    public string imageTopic = "/camera/image_raw/compressed";
    public string cameraInfoTopic = "/camera/camera_info";
    [Tooltip("Optical frame: z forward, x right, y down (REP-103).")]
    public string frameId = "camera_optical_frame";

    private RosConnector connector;
    private RenderTexture rt;
    private Texture2D cpuTex;          // only used if async readback is not supported
    private CameraInfo info;
    private string imageId, infoId;
    private bool advertised, pending;
    private float nextPublish;

    void Start()
    {
        connector = GetComponent<RosConnector>();
        if (sensorCamera == null)
        {
            var ctrl = FindObjectOfType<SwerveDriveController>();
            if (ctrl != null) sensorCamera = ctrl.GetComponentInChildren<Camera>(true);
        }
        if (sensorCamera == null)
        {
            Debug.LogError("RosCameraPublisher: no camera assigned and none found under the rover.", this);
            enabled = false;
            return;
        }

        rt = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
        rt.Create();
        sensorCamera.targetTexture = rt;

        ComputeIntrinsics(out double kfx, out double kfy, out double kcx, out double kcy);
        if (useCalibratedIntrinsics) ApplyProjection(kfx, kfy, kcx, kcy);
        info = BuildCameraInfo(kfx, kfy, kcx, kcy);
        Debug.Log($"RosCameraPublisher: {width}x{height}, fx={kfx:F2} fy={kfy:F2} cx={kcx:F2} cy={kcy:F2}, " +
                  $"{imageTopic} + {cameraInfoTopic}", this);
    }

    // K for the published resolution: the calibration scaled to width/height, or derived from the FOV.
    void ComputeIntrinsics(out double kfx, out double kfy, out double kcx, out double kcy)
    {
        if (useCalibratedIntrinsics)
        {
            double sx = (double)width / calibWidth, sy = (double)height / calibHeight;
            if (Math.Abs(sx - sy) > 1e-3)
                Debug.LogWarning($"RosCameraPublisher: {width}x{height} has a different aspect than the " +
                                 $"calibration {calibWidth}x{calibHeight}; the image will be stretched.", this);
            kfx = fx * sx; kfy = fy * sy;
            // Scale pixel-centre coordinates (OpenCV: pixel i covers [i-0.5, i+0.5]).
            kcx = (cx + 0.5) * sx - 0.5; kcy = (cy + 0.5) * sy - 0.5;
            return;
        }
        if (horizontalFovDeg > 0f)
            sensorCamera.fieldOfView = Camera.HorizontalToVerticalFieldOfView(horizontalFovDeg, (float)width / height);
        kfy = 0.5 * height / Math.Tan(0.5 * sensorCamera.fieldOfView * Mathf.Deg2Rad);
        kfx = kfy;
        kcx = 0.5 * width - 0.5; kcy = 0.5 * height - 0.5;
    }

    // Makes the Unity camera project exactly like the pinhole K (fx != fy and an off-centre
    // principal point included), which the plain FOV setting can't do.
    void ApplyProjection(double kfx, double kfy, double kcx, double kcy)
    {
        float n = sensorCamera.nearClipPlane, f = sensorCamera.farClipPlane;
        var m = Matrix4x4.zero;
        m[0, 0] = (float)(2.0 * kfx / width);
        m[0, 2] = (float)(1.0 - 2.0 * (kcx + 0.5) / width);
        m[1, 1] = (float)(2.0 * kfy / height);
        m[1, 2] = (float)(2.0 * (kcy + 0.5) / height - 1.0);   // image v grows downward
        m[2, 2] = -(f + n) / (f - n);
        m[2, 3] = -2f * f * n / (f - n);
        m[3, 2] = -1f;
        sensorCamera.projectionMatrix = m;
        // Keep the Inspector's FOV roughly right (display only, the matrix above is what renders).
        sensorCamera.fieldOfView = (float)(2.0 * Math.Atan(0.5 * height / kfy) * Mathf.Rad2Deg);
    }

    void Update()
    {
        if (connector == null || connector.IsConnected == null || !connector.IsConnected.WaitOne(0)) return;
        if (!advertised)
        {
            imageId = connector.RosSocket.Advertise<CompressedImage>(imageTopic);
            infoId = connector.RosSocket.Advertise<CameraInfo>(cameraInfoTopic);
            advertised = true;
        }
        if (pending || Time.time < nextPublish) return;
        nextPublish = Time.time + 1f / Mathf.Max(1f, publishRateHz);

        if (SystemInfo.supportsAsyncGPUReadback)
        {
            pending = true;
            AsyncGPUReadback.Request(rt, 0, TextureFormat.RGBA32, OnReadback);
        }
        else
        {
            if (cpuTex == null) cpuTex = new Texture2D(width, height, TextureFormat.RGBA32, false);
            var prev = RenderTexture.active;
            RenderTexture.active = rt;
            cpuTex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            RenderTexture.active = prev;
            Send(cpuTex.GetRawTextureData<byte>().ToArray());
        }
    }

    void OnReadback(AsyncGPUReadbackRequest req)
    {
        pending = false;
        if (this == null || !advertised || req.hasError) return;
        Send(req.GetData<byte>().ToArray());
    }

    void Send(byte[] rgba)
    {
        if (flipVertical) FlipRows(rgba, width * 4, height);
        byte[] jpg = ImageConversion.EncodeArrayToJPG(rgba, GraphicsFormat.R8G8B8A8_UNorm,
                                                      (uint)width, (uint)height, 0, jpegQuality);

        var header = new RosHeader { frame_id = frameId };
        header.Update();                       // stamp = now (same stamp on both messages)
        info.header = header;
        connector.RosSocket.Publish(infoId, info);
        connector.RosSocket.Publish(imageId, new CompressedImage { header = header, format = "jpeg", data = jpg });
    }

    // Distortion is published as zero: the simulated image is a perfect pinhole, so nodes must not undistort it.
    CameraInfo BuildCameraInfo(double fx, double fy, double cx, double cy)
    {
        var m = new CameraInfo { width = (uint)width, height = (uint)height, distortion_model = "plumb_bob" };
        SetArray(m, "d", new double[5]);
        SetArray(m, "k", new[] { fx, 0, cx, 0, fy, cy, 0, 0, 1 });
        SetArray(m, "r", new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 });
        SetArray(m, "p", new[] { fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0 });
        return m;
    }

    // ROS# names these D/K/R/P (ROS 1 build) or d/k/r/p (ROS 2 build); set whichever exists.
    static void SetArray(object msg, string name, double[] v)
    {
        const BindingFlags flags = BindingFlags.Public | BindingFlags.Instance | BindingFlags.IgnoreCase;
        var prop = msg.GetType().GetProperty(name, flags);
        if (prop != null) { prop.SetValue(msg, v); return; }
        var field = msg.GetType().GetField(name, flags);
        if (field != null) field.SetValue(msg, v);
    }

    static void FlipRows(byte[] data, int rowBytes, int rows)
    {
        var tmp = new byte[rowBytes];
        for (int top = 0, bottom = rows - 1; top < bottom; top++, bottom--)
        {
            Buffer.BlockCopy(data, top * rowBytes, tmp, 0, rowBytes);
            Buffer.BlockCopy(data, bottom * rowBytes, data, top * rowBytes, rowBytes);
            Buffer.BlockCopy(tmp, 0, data, bottom * rowBytes, rowBytes);
        }
    }

    void OnDestroy()
    {
        if (advertised && connector != null && connector.RosSocket != null)
        {
            try { connector.RosSocket.Unadvertise(imageId); connector.RosSocket.Unadvertise(infoId); }
            catch (Exception) { /* socket already closed */ }
        }
        if (sensorCamera != null) { sensorCamera.targetTexture = null; sensorCamera.ResetProjectionMatrix(); }
        if (rt != null) { rt.Release(); Destroy(rt); }
        if (cpuTex != null) Destroy(cpuTex);
    }
}
