using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>
/// Tools > WRover > Add Rover Cameras
/// Copies the existing front camera mount (camera_link with its camera and camera_cube) in
/// wrover_camera.prefab to four more spots on the chassis, and adds a CameraViewSwitcher to the
/// open scene's Main Camera. Safe to run again: mounts that already exist are skipped.
///
/// Positions are in base_link's frame: +X forward, +Y up, +Z left (rocker_left sits at +Z).
/// The chassis is base_link's URDF collision box: centre (0, 0.1016, 0), size 0.4137 x 0.2032 x 0.4137 m,
/// so its faces are at x = +-0.2068, z = +-0.2068, y = 0 (bottom) and y = 0.2032 (top).
/// To move a camera afterwards, move its *_link in the prefab, or change the numbers below,
/// delete the link and run the menu again.
/// </summary>
public static class AddRoverCameras
{
    const string PrefabPath = "Assets/Robots/wrover/wrover_camera.prefab";
    const string TemplateLink = "camera_link";

    const float FaceX = 0.215f;        // same distance out as the existing front camera_link
    const float HalfWidth = 0.2068f;
    const float TopY = 0.2032f;
    const float BottomY = 0f;
    const float MidY = 0.10f;          // height of the existing front camera (link 0.08 + camera 0.02)
    // camera_cube is 0.01 m deep (along the view direction), 0.015 m tall and 0.03 m wide.
    const float CubeHalfDepth = 0.005f, CubeHalfHeight = 0.0075f, CubeHalfWidth = 0.015f;
    // How far a cube turned 45 deg reaches along x and z from its centre.
    const float CornerInset = (CubeHalfWidth + CubeHalfDepth) * 0.70710678f;

    struct Mount
    {
        public string link;
        public Vector3 cameraPos;      // where the camera itself ends up, in base_link
        public Quaternion rotation;    // camera looks along its +Z, image "up" is its +Y
        public Mount(string link, Vector3 cameraPos, Vector3 look, Vector3 up)
        { this.link = link; this.cameraPos = cameraPos; rotation = Quaternion.LookRotation(look, up); }
    }

    static readonly Mount[] Mounts =
    {
        // Sitting on the top face at the front-left / front-right corners, each turned 45 deg
        // outward from forward (left camera toward +Z/left, right camera toward -Z/right).
        // The cube rests on the top and its rotated corner stops at the chassis edges.
        new Mount("camera_front_left_link",
                  new Vector3(HalfWidth - CornerInset, TopY + CubeHalfHeight, HalfWidth - CornerInset), new Vector3(1f, 0f, 1f), Vector3.up),
        new Mount("camera_front_right_link",
                  new Vector3(HalfWidth - CornerInset, TopY + CubeHalfHeight, -(HalfWidth - CornerInset)), new Vector3(1f, 0f, -1f), Vector3.up),
        // Centre of the back face, mirror of the existing front camera, looking backward.
        new Mount("camera_rear_link",
                  new Vector3(-FaceX, MidY, 0f), Vector3.left, Vector3.up),
        // Centre of the bottom, looking straight down; top of the image = rover forward.
        new Mount("camera_bottom_link",
                  new Vector3(0f, BottomY - CubeHalfDepth, 0f), Vector3.down, Vector3.right),
    };

    [MenuItem("Tools/WRover/Add Rover Cameras")]
    static void Run()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            EditorUtility.DisplayDialog("Add Rover Cameras",
                "Stop Play mode first. Anything added to the scene during Play is thrown away when Play stops.", "OK");
            return;
        }
        GameObject root = PrefabUtility.LoadPrefabContents(PrefabPath);
        try
        {
            Transform baseLink = FindDeep(root.transform, "base_link");
            Transform template = baseLink != null ? baseLink.Find(TemplateLink) : null;
            Transform templateCam = template != null ? template.Find("camera") : null;
            if (templateCam == null)
            {
                Debug.LogError($"AddRoverCameras: no base_link/{TemplateLink}/camera in {PrefabPath}.");
                return;
            }

            int added = 0;
            foreach (Mount m in Mounts)
            {
                if (baseLink.Find(m.link) != null)
                {
                    Debug.Log($"AddRoverCameras: {m.link} already exists, skipped.");
                    continue;
                }
                Transform link = Object.Instantiate(template.gameObject, baseLink).transform;
                link.name = m.link;
                link.localRotation = m.rotation;
                // The camera and cube sit at an offset inside the link (0, 0.02, 0); place the link
                // so the camera lands exactly on cameraPos.
                link.localPosition = m.cameraPos - m.rotation * templateCam.localPosition;
                added++;
                Debug.Log($"AddRoverCameras: added {m.link} at {link.localPosition.ToString("F4")} in base_link.");
            }

            if (added > 0)
            {
                PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
                Debug.Log($"AddRoverCameras: saved {added} new camera mount(s) to {PrefabPath}.");
            }
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }

        AddSwitcherToScene();
    }

    static void AddSwitcherToScene()
    {
        Camera main = Camera.main;
        if (main == null)
        {
            Debug.LogWarning("AddRoverCameras: no camera tagged MainCamera in the open scene; " +
                             "add CameraViewSwitcher to a GameObject yourself.");
            return;
        }
        if (main.GetComponent<CameraViewSwitcher>() != null) return;
        Undo.AddComponent<CameraViewSwitcher>(main.gameObject);
        EditorSceneManager.MarkSceneDirty(main.gameObject.scene);
        Debug.Log($"AddRoverCameras: added CameraViewSwitcher to '{main.name}'. Save the scene to keep it.", main);
    }

    static Transform FindDeep(Transform t, string name)
    {
        if (t.name == name) return t;
        foreach (Transform child in t)
        {
            Transform hit = FindDeep(child, name);
            if (hit != null) return hit;
        }
        return null;
    }
}
