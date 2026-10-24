using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Editor tool: adds a collider fitted to the real shape of every selected visual mesh.
///
/// For each MeshFilter under the selection it finds the mesh's principal axes (PCA of its
/// vertices) and adds, on a child object named "FittedCollider":
///   - a CapsuleCollider for long round parts (tubes, shafts),
///   - a BoxCollider for everything else (motors, housings, brackets).
/// The collider sits under the link's ArticulationBody, so it moves with that link.
/// Running it again replaces the previous FittedCollider. Supports Undo.
///
/// Usage: open the rover prefab, select the meshes (e.g. search "rocker_tube" in the Hierarchy
/// and select the results), then Tools > WRover > Add Fitted Colliders To Selection.
/// Must live in a folder named "Editor".
/// </summary>
public static class FitCollidersFromMeshes
{
    const string ChildName = "FittedCollider";

    [MenuItem("Tools/WRover/Add Fitted Colliders To Selection")]
    static void AddToSelection()
    {
        var filters = new HashSet<MeshFilter>();
        foreach (GameObject go in Selection.gameObjects)
            foreach (MeshFilter mf in go.GetComponentsInChildren<MeshFilter>(true))
                if (mf.sharedMesh != null && mf.gameObject.name != ChildName) filters.Add(mf);

        if (filters.Count == 0)
        {
            EditorUtility.DisplayDialog("Fitted colliders", "Select one or more visual meshes (or their parents) first.", "OK");
            return;
        }

        Undo.SetCurrentGroupName("Add fitted colliders");
        int group = Undo.GetCurrentGroup();
        int capsules = 0, boxes = 0, skipped = 0;
        var report = new System.Text.StringBuilder();

        foreach (MeshFilter mf in filters)
        {
            Vector3[] v = mf.sharedMesh.vertices;
            if (v == null || v.Length < 4) { skipped++; continue; }

            // Replace an earlier fitted collider on this mesh.
            Transform old = mf.transform.Find(ChildName);
            if (old != null) Undo.DestroyObjectImmediate(old.gameObject);

            FitObb(v, out Vector3 center, out Vector3 ax, out Vector3 ay, out Vector3 az, out Vector3 size);

            var child = new GameObject(ChildName);
            Undo.RegisterCreatedObjectUndo(child, "Add fitted collider");
            child.transform.SetParent(mf.transform, false);
            child.transform.localPosition = center;
            child.transform.localRotation = Quaternion.LookRotation(az, ay);
            child.transform.localScale = Vector3.one;

            // Sort the three extents to decide capsule vs box.
            float[] len = { size.x, size.y, size.z };
            int longest = 0;
            for (int i = 1; i < 3; i++) if (len[i] > len[longest]) longest = i;
            float a = len[(longest + 1) % 3], b = len[(longest + 2) % 3];
            float thick = Mathf.Max(a, b), thin = Mathf.Min(a, b);
            bool tube = len[longest] > 2.5f * thick && thick < 1.6f * Mathf.Max(thin, 1e-4f);

            if (tube)
            {
                var c = child.AddComponent<CapsuleCollider>();
                c.direction = longest;
                c.radius = thick / 2f;
                c.height = len[longest];
                capsules++;
                report.AppendLine($"{mf.name}: capsule  length {len[longest]:F3} m, radius {thick / 2f:F3} m");
            }
            else
            {
                var c = child.AddComponent<BoxCollider>();
                c.size = size;
                boxes++;
                report.AppendLine($"{mf.name}: box  {size.x:F3} x {size.y:F3} x {size.z:F3} m");
            }
        }

        Undo.CollapseUndoOperations(group);
        string summary = $"Added {capsules} capsule(s) and {boxes} box(es)" + (skipped > 0 ? $", skipped {skipped}" : "") + ".";
        Debug.Log(summary + "\n" + report);
        EditorUtility.DisplayDialog("Fitted colliders", summary + " Details are in the Console.", "OK");
    }

    /// <summary>Oriented bounding box from the principal axes of the vertices (mesh-local space).</summary>
    static void FitObb(Vector3[] v, out Vector3 center, out Vector3 ax, out Vector3 ay, out Vector3 az, out Vector3 size)
    {
        // Mean
        double mx = 0, my = 0, mz = 0;
        foreach (var p in v) { mx += p.x; my += p.y; mz += p.z; }
        mx /= v.Length; my /= v.Length; mz /= v.Length;

        // Covariance
        double[,] c = new double[3, 3];
        foreach (var p in v)
        {
            double dx = p.x - mx, dy = p.y - my, dz = p.z - mz;
            c[0, 0] += dx * dx; c[0, 1] += dx * dy; c[0, 2] += dx * dz;
            c[1, 1] += dy * dy; c[1, 2] += dy * dz; c[2, 2] += dz * dz;
        }
        c[1, 0] = c[0, 1]; c[2, 0] = c[0, 2]; c[2, 1] = c[1, 2];

        double[,] e = Jacobi(c);
        Vector3 e0 = new Vector3((float)e[0, 0], (float)e[1, 0], (float)e[2, 0]).normalized;
        Vector3 e1 = new Vector3((float)e[0, 1], (float)e[1, 1], (float)e[2, 1]).normalized;
        Vector3 e2 = Vector3.Cross(e0, e1).normalized;   // orthonormal frame
        e1 = Vector3.Cross(e2, e0).normalized;
        // Unity frame: x = Cross(y, z)
        ay = e1; az = e2; ax = Vector3.Cross(ay, az).normalized;

        // PCA axes are ambiguous for symmetric cross-sections (a square motor body), so refine:
        // spin the box about each axis in 1-degree steps and keep the smallest volume.
        for (int pass = 0; pass < 2; pass++)
            for (int k = 0; k < 3; k++)
            {
                Vector3 axis = k == 0 ? ax : (k == 1 ? ay : az);
                Vector3 bx = ax, by = ay, bz = az;
                float best = Volume(v, ax, ay, az);
                for (int deg = 1; deg < 90; deg++)
                {
                    Quaternion r = Quaternion.AngleAxis(deg, axis);
                    Vector3 tx = r * ax, ty = r * ay, tz = r * az;
                    float vol = Volume(v, tx, ty, tz);
                    if (vol < best - 1e-12f) { best = vol; bx = tx; by = ty; bz = tz; }
                }
                ax = bx.normalized; ay = by.normalized; az = bz.normalized;
            }

        Extents(v, ax, ay, az, out Vector3 lo, out Vector3 hi);
        Vector3 mid = (lo + hi) * 0.5f;
        center = ax * mid.x + ay * mid.y + az * mid.z;
        size = hi - lo;
    }

    static void Extents(Vector3[] v, Vector3 ax, Vector3 ay, Vector3 az, out Vector3 lo, out Vector3 hi)
    {
        lo = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        hi = -lo;
        foreach (var p in v)
        {
            Vector3 q = new Vector3(Vector3.Dot(p, ax), Vector3.Dot(p, ay), Vector3.Dot(p, az));
            lo = Vector3.Min(lo, q);
            hi = Vector3.Max(hi, q);
        }
    }

    static float Volume(Vector3[] v, Vector3 ax, Vector3 ay, Vector3 az)
    {
        Extents(v, ax, ay, az, out Vector3 lo, out Vector3 hi);
        Vector3 s = hi - lo;
        return s.x * s.y * s.z;
    }

    /// <summary>Eigenvectors (columns) of a symmetric 3x3 matrix, cyclic Jacobi rotations.</summary>
    static double[,] Jacobi(double[,] a)
    {
        double[,] m = (double[,])a.Clone();
        double[,] vec = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
        for (int sweep = 0; sweep < 50; sweep++)
        {
            double off = m[0, 1] * m[0, 1] + m[0, 2] * m[0, 2] + m[1, 2] * m[1, 2];
            if (off < 1e-18) break;
            for (int p = 0; p < 2; p++)
                for (int q = p + 1; q < 3; q++)
                {
                    if (System.Math.Abs(m[p, q]) < 1e-15) continue;
                    double theta = (m[q, q] - m[p, p]) / (2 * m[p, q]);
                    double t = System.Math.Sign(theta) / (System.Math.Abs(theta) + System.Math.Sqrt(theta * theta + 1));
                    if (theta == 0) t = 1;
                    double cs = 1 / System.Math.Sqrt(t * t + 1), sn = t * cs;
                    for (int k = 0; k < 3; k++)
                    {
                        double mkp = m[k, p], mkq = m[k, q];
                        m[k, p] = cs * mkp - sn * mkq;
                        m[k, q] = sn * mkp + cs * mkq;
                    }
                    for (int k = 0; k < 3; k++)
                    {
                        double mpk = m[p, k], mqk = m[q, k];
                        m[p, k] = cs * mpk - sn * mqk;
                        m[q, k] = sn * mpk + cs * mqk;
                    }
                    for (int k = 0; k < 3; k++)
                    {
                        double vkp = vec[k, p], vkq = vec[k, q];
                        vec[k, p] = cs * vkp - sn * vkq;
                        vec[k, q] = sn * vkp + cs * vkq;
                    }
                }
        }
        return vec;
    }
}
