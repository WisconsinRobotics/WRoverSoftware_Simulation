using UnityEngine;

/// <summary>
/// Replaces the rover's differential bar (and its rods / ball joints), which a URDF
/// cannot represent because it closes a loop.
///
/// It enforces   theta_left = coupling * theta_right   (1:1 ratio) on the two rocker
/// joints through their ArticulationBody drives. Only the shared ("symmetric") part
/// of the rocker motion is corrected, which is the part that would pitch the chassis.
/// The rockers stay free to move against each other over terrain, exactly like the
/// real bar. Drive torques act between the chassis and each rocker, so the chassis
/// receives the reaction load just as it would through the bar.
///
/// Setup: add this component to the imported robot's root (base_link). The rocker
/// bodies are found by name if left empty. Unity 2022.3, ArticulationBody.
/// </summary>
[DisallowMultipleComponent]
public class RockerDifferential : MonoBehaviour
{
    [Tooltip("ArticulationBody of link 'rocker_left' (its joint is rocker_left_joint). Found by name if empty.")]
    public ArticulationBody leftRocker;

    [Tooltip("ArticulationBody of link 'rocker_right' (its joint is rocker_right_joint). Found by name if empty.")]
    public ArticulationBody rightRocker;

    [Tooltip("+1 or -1. In the exported URDF the two rocker joint axes point in opposite " +
             "directions, so physically opposite rotation means equal joint angles: +1. " +
             "If lifting one wheel makes the other rocker move the wrong way, use -1.")]
    public float coupling = 1f;

    [Tooltip("Drive stiffness (N*m per degree). Higher = stiffer differential. " +
             "Lower it if you see shaking; raise it if the chassis sags under the arm.")]
    public float stiffness = 2000f;

    [Tooltip("Drive damping (N*m*s per degree). Only damps the symmetric (chassis-pitch) motion.")]
    public float damping = 100f;

    [Tooltip("Maximum torque the virtual bar can apply (N*m).")]
    public float forceLimit = 1000f;

    [Header("Debug (read only)")]
    [SerializeField] private float leftAngleDeg;
    [SerializeField] private float rightAngleDeg;
    [SerializeField] private float constraintErrorDeg;

    void Awake()
    {
        if (leftRocker == null) leftRocker = FindBody("rocker_left");
        if (rightRocker == null) rightRocker = FindBody("rocker_right");
        if (leftRocker == null || rightRocker == null)
        {
            Debug.LogError("RockerDifferential: could not find rocker_left / rocker_right ArticulationBodies.", this);
            enabled = false;
        }
    }

    void FixedUpdate()
    {
        // Revolute joint positions/velocities are in radians and rad/s.
        float thL = leftRocker.jointPosition[0];
        float thR = rightRocker.jointPosition[0];
        float vL = leftRocker.jointVelocity[0];
        float vR = rightRocker.jointVelocity[0];

        // Residual of the constraint thL = c * thR, split evenly between the rockers.
        // Since c = +/-1, targets satisfy tL - c * tR = 0 exactly.
        float c = Mathf.Sign(coupling);
        float r = thL - c * thR;
        float rv = vL - c * vR;

        SetDrive(leftRocker, thL - r / 2f, vL - rv / 2f);
        SetDrive(rightRocker, thR + c * r / 2f, vR + c * rv / 2f);

        leftAngleDeg = thL * Mathf.Rad2Deg;
        rightAngleDeg = thR * Mathf.Rad2Deg;
        constraintErrorDeg = r * Mathf.Rad2Deg;
    }

    void SetDrive(ArticulationBody body, float targetRad, float targetVelRad)
    {
        // Revolute joints are driven by xDrive; drive targets are in degrees.
        ArticulationDrive d = body.xDrive;
        d.stiffness = stiffness;
        d.damping = damping;
        d.forceLimit = forceLimit;
        d.target = targetRad * Mathf.Rad2Deg;
        d.targetVelocity = targetVelRad * Mathf.Rad2Deg;
        body.xDrive = d;
    }

    ArticulationBody FindBody(string linkName)
    {
        foreach (var b in GetComponentsInChildren<ArticulationBody>(true))
            if (b.name == linkName) return b;
        return null;
    }
}
