using UnityEngine;

/// <summary>
/// Replaces the rover's differential bar (and its rods / ball joints), which a URDF
/// cannot represent because it closes a loop.
///
/// It enforces   theta_left = coupling * theta_right   (1:1 ratio) on the two rocker
/// joints through their ArticulationBody drives. Only the shared ("symmetric") part
/// of the rocker motion is corrected, which is the part that would pitch the chassis.
/// The rockers stay free to move against each other over terrain, like the real bar,
/// up to the hard stop set by rockerLimitDeg.
///
/// v2 changes:
///  - Hard stops: the URDF rocker joints are "continuous" (no limits), so nothing stopped
///    the free (differential) motion. They are now limited to +/- rockerLimitDeg.
///  - No one-step lag: the drive target is predicted to the end of the physics step, so the
///    stiff drive no longer drags on the free differential motion.
///  - Wrap-safe residual: the error is wrapped to [-180, 180] deg, so a joint reading that
///    jumps by 360 deg can no longer produce a huge correction torque.
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
             "directions (left -X, right +X in base_link), so physically opposite rotation " +
             "means equal joint angles: +1.")]
    public float coupling = 1f;

    [Tooltip("Rocker travel each way from the zero pose (deg): the real rover's hard stop. " +
             "Measure it on the rover. 0 = no limit (not recommended).")]
    public float rockerLimitDeg = 25f;

    [Tooltip("Drive stiffness. Higher = stiffer differential. " +
             "Lower it if you see shaking; raise it if the chassis sags under the arm.")]
    public float stiffness = 2000f;

    [Tooltip("Drive damping. Only damps the symmetric (chassis-pitch) motion.")]
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
            return;
        }

        if (rockerLimitDeg > 0f)
        {
            ApplyLimits(leftRocker);
            ApplyLimits(rightRocker);
        }
    }

    void ApplyLimits(ArticulationBody body)
    {
        // Revolute joints use the twist DOF and xDrive. SetDrive() below copies xDrive,
        // so these limits are kept every frame.
        body.twistLock = ArticulationDofLock.LimitedMotion;
        ArticulationDrive d = body.xDrive;
        d.lowerLimit = -rockerLimitDeg;
        d.upperLimit = rockerLimitDeg;
        body.xDrive = d;
    }

    void FixedUpdate()
    {
        float c = coupling >= 0f ? 1f : -1f;
        float dt = Time.fixedDeltaTime;

        // Joint positions/velocities come in rad and rad/s; work in degrees (drive units).
        float thL = leftRocker.jointPosition[0] * Mathf.Rad2Deg;
        float thR = rightRocker.jointPosition[0] * Mathf.Rad2Deg;
        float vL = leftRocker.jointVelocity[0] * Mathf.Rad2Deg;
        float vR = rightRocker.jointVelocity[0] * Mathf.Rad2Deg;

        // Residual of the constraint thL = c * thR, wrapped to [-180, 180].
        float r = Mathf.DeltaAngle(c * thR, thL);
        float rv = vL - c * vR;

        // Velocity of the free (differential) mode, in left-rocker coordinates.
        float midVel = vL - rv / 2f;

        // Predict where the free mode will be at the end of this step. Without this the
        // target lags one step behind and the stiff drive brakes the differential motion.
        float pred = midVel * dt;

        // Each target is written relative to that joint's own reading, so a wrapped
        // reading on the other joint can never pull this one by 360 deg.
        float tL = thL - r / 2f + pred;
        float tR = thR + c * (r / 2f + pred);

        if (rockerLimitDeg > 0f)
        {
            tL = Mathf.Clamp(tL, -rockerLimitDeg, rockerLimitDeg);
            tR = Mathf.Clamp(tR, -rockerLimitDeg, rockerLimitDeg);
        }

        SetDrive(leftRocker, tL, midVel);
        SetDrive(rightRocker, tR, c * midVel);

        leftAngleDeg = thL;
        rightAngleDeg = thR;
        constraintErrorDeg = r;
    }

    void SetDrive(ArticulationBody body, float targetDeg, float targetVelDeg)
    {
        // Revolute joints are driven by xDrive; drive targets are in degrees.
        ArticulationDrive d = body.xDrive;
        d.stiffness = stiffness;
        d.damping = damping;
        d.forceLimit = forceLimit;
        d.target = targetDeg;
        d.targetVelocity = targetVelDeg;
        body.xDrive = d;
    }

    ArticulationBody FindBody(string linkName)
    {
        foreach (var b in GetComponentsInChildren<ArticulationBody>(true))
            if (b.name == linkName) return b;
        return null;
    }
}