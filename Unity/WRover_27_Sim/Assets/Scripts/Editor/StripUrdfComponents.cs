using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Editor tool: removes every leftover "Urdf..." helper script (UrdfRobot, UrdfLink,
/// UrdfJoint*, UrdfInertial, UrdfVisual(s), UrdfCollision(s), UrdfPlugin(s)) from the
/// selected robot, plus any "Missing script" entries.
///
/// Keeps every GameObject, mesh, collider, material and ArticulationBody, and any other
/// script (e.g. RockerDifferential). Supports Undo (Ctrl+Z).
///
/// Usage: select the robot root (best: open the prefab in Prefab Mode and select its
/// root), then Tools > WRover > Strip URDF Helper Scripts.
/// Must live in a folder named "Editor" (e.g. Assets/Scripts/Editor/).
/// </summary>
public static class StripUrdfComponents
{
    // Dependents first, so nothing is blocked by a [RequireComponent] on another Urdf script.
    static int Order(Component c)
    {
        string n = c.GetType().Name;
        if (n.StartsWith("UrdfJoint")) return 0;
        if (n.StartsWith("UrdfInertial")) return 1;
        if (n.StartsWith("UrdfVisual") || n.StartsWith("UrdfCollision") || n.StartsWith("UrdfPlugin")) return 2;
        if (n.StartsWith("UrdfLink")) return 3;
        if (n.StartsWith("UrdfRobot")) return 4;
        return 5;
    }

    [MenuItem("Tools/WRover/Strip URDF Helper Scripts")]
    static void Strip()
    {
        GameObject root = Selection.activeGameObject;
        if (root == null)
        {
            EditorUtility.DisplayDialog("Strip URDF scripts", "Select the robot's root object first.", "OK");
            return;
        }

        List<MonoBehaviour> targets = root.GetComponentsInChildren<MonoBehaviour>(true)
            .Where(c => c != null && c.GetType().Name.StartsWith("Urdf"))
            .OrderBy(Order)
            .ToList();

        int missing = root.GetComponentsInChildren<Transform>(true)
            .Sum(t => GameObjectUtility.GetMonoBehavioursWithMissingScriptCount(t.gameObject));

        if (targets.Count == 0 && missing == 0)
        {
            EditorUtility.DisplayDialog("Strip URDF scripts", $"No Urdf scripts found under '{root.name}'.", "OK");
            return;
        }

        if (!EditorUtility.DisplayDialog("Strip URDF scripts",
                $"Remove {targets.Count} Urdf script(s) and {missing} missing-script entr(ies) under '{root.name}'?\n\n" +
                "GameObjects, meshes, colliders, ArticulationBodies and your own scripts are kept.",
                "Remove", "Cancel"))
            return;

        Undo.SetCurrentGroupName("Strip URDF helper scripts");
        int group = Undo.GetCurrentGroup();

        // Unlock objects the importer marked NotEditable (the greyed-out Visuals/Collisions).
        foreach (Transform t in root.GetComponentsInChildren<Transform>(true))
        {
            if ((t.gameObject.hideFlags & HideFlags.NotEditable) != 0)
            {
                Undo.RecordObject(t.gameObject, "Unlock");
                t.gameObject.hideFlags &= ~HideFlags.NotEditable;
            }
        }

        int removed = 0;
        // A few passes in case one script still requires another.
        for (int pass = 0; pass < 4 && targets.Count > 0; pass++)
        {
            foreach (MonoBehaviour c in targets)
            {
                if (c == null) continue;
                Undo.DestroyObjectImmediate(c);
                if (c == null) removed++;
            }
            targets = targets.Where(c => c != null).ToList();
        }

        int missingRemoved = 0;
        foreach (Transform t in root.GetComponentsInChildren<Transform>(true))
            missingRemoved += GameObjectUtility.RemoveMonoBehavioursWithMissingScript(t.gameObject);

        Undo.CollapseUndoOperations(group);

        string msg = $"Removed {removed} Urdf script(s) and {missingRemoved} missing-script entr(ies) from '{root.name}'.";
        if (targets.Count > 0)
            msg += $" {targets.Count} could not be removed: " + string.Join(", ", targets.Select(c => $"{c.gameObject.name}/{c.GetType().Name}"));
        Debug.Log(msg, root);
        EditorUtility.DisplayDialog("Strip URDF scripts", msg, "OK");
    }
}
