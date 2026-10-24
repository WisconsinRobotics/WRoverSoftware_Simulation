using UnityEngine;

/// <summary>
/// Turns off collisions between the rover's own colliders (e.g. a rocker tube touching the
/// arm column or a wheel), so parts that sit close together don't push on each other.
/// The rover still collides with the ground, rocks and every other object.
/// Put it on the robot root. Physics.IgnoreCollision is not saved in the scene, so this
/// runs every time Play starts.
/// </summary>
[DisallowMultipleComponent]
public class RobotSelfCollisionFilter : MonoBehaviour
{
    [Tooltip("Off = the arm can still hit the rover body (useful later for arm work).")]
    public bool includeArm = true;

    static readonly string[] ArmLinks = { "arm_column", "arm_carriage", "arm_supinator", "wrist_differential", "manipulator", "claw" };

    void Start()
    {
        Collider[] cols = GetComponentsInChildren<Collider>(true);
        int pairs = 0;
        for (int i = 0; i < cols.Length; i++)
            for (int j = i + 1; j < cols.Length; j++)
            {
                if (!includeArm && (OnArm(cols[i]) != OnArm(cols[j]))) continue;
                Physics.IgnoreCollision(cols[i], cols[j], true);
                pairs++;
            }
        Debug.Log($"RobotSelfCollisionFilter: ignoring {pairs} collider pairs inside {name}.", this);
    }

    static bool OnArm(Collider c)
    {
        var body = c.GetComponentInParent<ArticulationBody>();
        if (body == null) return false;
        foreach (string n in ArmLinks) if (body.name == n) return true;
        return false;
    }
}
