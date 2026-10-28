using System;
using UnityEngine;

/// <summary>
/// Swerve drive controller for the WRover simulation (Unity 2022.3, ArticulationBody).
///
/// Input: a body velocity command (vx, vy, wz) in the robot frame, same convention as a
/// ROS geometry_msgs/Twist on /cmd_vel:
///   vx = forward (m/s), vy = left (m/s), wz = counter-clockwise yaw rate seen from above (rad/s).
/// Sources: keyboard (W/S forward-back, A/D strafe, Q/E rotate, Space = stop) or any script /
/// ROS bridge calling SetCommand().
///
/// Each physics step it:
///  1. clamps the command to the rover limits and rate-limits it (acceleration),
///  2. computes each module's velocity  v_i = (vx - wz*y_i, vy + wz*x_i)  at the wheel's
///     ground position (x_i, y_i) measured live, so rocker tilt and the 3.8 cm caster offset
///     are included,
///  3. desaturates: if any wheel would exceed maxWheelSurfaceSpeed, all are scaled together,
///  4. steering: heading = atan2(v_i); the change is normalized to [-180, 180] deg so a module
///     never turns more than half a turn (optional: reverse the wheel instead of turning > 90),
///     and slewed at maxSteerRate,
///  5. drives the joints: steering = position drive, wheels = velocity drive,
///  6. computes wheel odometry (least squares over the 4 modules).
///
/// The robot frame (forward / left / up) is worked out from the module names
/// (steer_fl, steer_fr, steer_rl, steer_rr) and the steering axes, so it does not depend on
/// how the CAD base frame is oriented.
/// </summary>
[DisallowMultipleComponent]
[DefaultExecutionOrder(-50)]


public class SwerveDriveController : MonoBehaviour
{
    public enum CommandSource { Auto, KeyboardOnly, ExternalOnly }

    [Serializable]
    public class SwerveModule
    {
        public string id;                 // fl, fr, rl, rr
        public ArticulationBody steer;    // link steer_xx (steering joint)
        public ArticulationBody wheel;    // link wheel_xx (wheel joint)

        [NonSerialized] public Vector2 position;          // wheel ground position, robot frame (m)
        [NonSerialized] public float measuredHeading;     // rad, robot frame
        [NonSerialized] public float commandedHeading;    // rad, slewed
        [NonSerialized] public float holdHeading;         // heading kept while stopped
        [NonSerialized] public float targetSpeed;         // m/s along measured heading
        [NonSerialized] public float measuredSpeed;       // m/s along measured heading
        [NonSerialized] public float currentAxleTorque; // Torque of meter
        [NonSerialized] public float currentMotorRPM; // Rotations per minute of the motor
        [NonSerialized] public bool initialized;
    }

    [Header("Robot (auto-filled by name if empty)")]
    public ArticulationBody baseLink;
    public SwerveModule[] modules = new SwerveModule[0];
    [Tooltip("Wheel radius (m). 0.110 m from the tread geometry.")]
    public float wheelRadius = 0.110f;

    [Header("Rover limits")]
    [Tooltip("Max linear speed of the rover (m/s), magnitude of (vx, vy).")]
    public float maxLinearSpeed = 2.566f;
    [Tooltip("Max yaw rate of the whole rover (rad/s).")]
    public float maxAngularSpeed = 3.5f;
    [Tooltip("Placeholder until real data: linear acceleration limit (m/s^2).")]
    public float maxLinearAccel = 3.0f;
    [Tooltip("Placeholder until real data: angular acceleration limit (rad/s^2).")]
    public float maxAngularAccel = 6.0f;

    [Header("Module limits")]
    [Tooltip("Max ground speed of any single wheel (m/s). 227RPM gives 227 * 2pi / 60 ~ 23 rad/s * 0.110 = 2.615 m/s")]
    public float maxWheelSurfaceSpeed = 2.615f;
    [Tooltip("Max steering rate per module (rad/s). 172 RPM for swerve motor gives 172 * 2pi / 60 ~ 18 rad/s")]
    public float maxSteerRate = 18.01f;
    [Tooltip("Off = only normalize to +-180 deg (your rover). On = also reverse the wheel instead of steering more than 90 deg.")]
    public bool reverseWheelInsteadOfTurningPast90 = false;
    [Tooltip("Scale wheel speed by cos(steering error) so wheels don't drag sideways while a module is still turning.")]
    public bool slowWheelsWhileSteering = true;

    [Header("Steering drive (position)")]
    [Tooltip("Proportional gain (N*m per degree). 80-150 keeps azimuth locked over rocky ruts.")]
    public float steerStiffness = 100f;   // N*m per degree
    [Tooltip("Derivative gain (N*m*s per degree). Prevents caster shimmy on uneven triangles.")]
    public float steerDamping = 5f;    // N*m*s per degree
    [Tooltip("Max steering azimuth torque (N*m) after gearbox reduction. Range is 45-120N")]
    public float steerForceLimit = 55f;  // N*m  (placeholder: steering motor torque) 2.67 * 25 * 0.85 (gear ratio * efficiency * Tstall)

    [Header("REV NEO v1.1 Drive Motor & 64:1 Gearbox")]
    [Tooltip("Stall torque at motor shaft in N*m - Estimated from mechanical's graph")]
    public float motorStallTorque = 2.67f;
    [Tooltip("Free speed in RPM - estimated from Mechanical's graph")]
    public float motorFreeSpeedRPM = 5676f;
    [Tooltip("Planetary reduction ratio G:1")]
    public float gearRatio = 25.0f;
    [Tooltip("Transmission mechanical efficiency (0.0 - 1.0)")]
    public float gearEfficiency = 0.88f;
    [Tooltip("Wheel Damping")]
    public float wheelDamping = 1f;
    
    
    [Header("Soil Interaction")]
    [Tooltip("Soil rolling resistance coefficient on desert sand (0.08 - 0.12)")]
    public float rollingResistanceCoeff = 0.1f;

    [Header("Command input")]
    public CommandSource source = CommandSource.Auto;
    [Tooltip("External (ROS) command is dropped if no new message arrives within this time (s).")]
    public float externalCommandTimeout = 0.5f;
    [Range(0.05f, 1f)] public float keyboardSpeedFraction = 1.0f;
    [Tooltip("Keyboard W/A/S/D relative to the rover's starting heading instead of its current heading.")]
    public bool keyboardFieldCentric = false;

    [Header("Debug")]
    public bool showHud = true;
    public bool drawGizmos = true;
    [SerializeField] private string activeSource = "-";
    [SerializeField] private Vector3 requestedCommand;   // vx, vy, wz before limits
    [SerializeField] private Vector3 appliedCommand;     // after limits and acceleration
    [SerializeField] private Vector3 odometryTwist;      // measured vx, vy, wz
    [SerializeField] private Vector3 odometryPose;       // x, y, yaw (odom frame)

    // ---------------------------------------------------------------- public API
    /// <summary>Set the body velocity command (robot frame, ROS convention). Call every cycle.</summary>
    public void SetCommand(float vx, float vy, float wz)
    {
        externalCommand = new Vector3(vx, vy, wz);
        lastExternalTime = Time.time;
    }

    /// <summary>Wheel odometry pose in the odom frame: x (m), y (m), yaw (rad). Starts at 0.</summary>
    public Vector3 OdometryPose => odometryPose;
    /// <summary>Wheel odometry twist in the robot frame: vx, vy (m/s), wz (rad/s).</summary>
    public Vector3 OdometryTwist => odometryTwist;
    public Vector3 AppliedCommand => appliedCommand;
    public bool IsReady => ready;

    /// <summary>
    /// True pose and twist from the physics engine, relative to the start pose (Unity axes:
    /// +z forward, +y up). twistRobot = (vx, vy, wz) in the robot frame, ROS convention.
    /// </summary>
    public void GetGroundTruth(out Vector3 relativePosition, out Quaternion relativeRotation, out Vector3 twistRobot)
    {
        Quaternion rot = Quaternion.LookRotation(fwd, up);
        relativePosition = Quaternion.Inverse(startRotation) * (center - startCenter);
        relativeRotation = Quaternion.Inverse(startRotation) * rot;
        Vector3 v = baseLink.velocity + Vector3.Cross(baseLink.angularVelocity, center - baseLink.worldCenterOfMass);
        twistRobot = new Vector3(Vector3.Dot(v, fwd), Vector3.Dot(v, left), -Vector3.Dot(baseLink.angularVelocity, up));
    }

    // ---------------------------------------------------------------- state
    private Vector3 externalCommand;
    private float lastExternalTime = -1e9f;
    private Vector3 keyboardCommand;
    private bool keyboardActive, stopHeld;

    private Vector3 upLocal, fwdLocal;      // robot frame axes in base_link local space
    private Vector3 up, fwd, left, center;  // world, updated every step
    private Vector3 startCenter, startFwdFlat, startLeftFlat;
    private Quaternion startRotation;
    private float jointSign = 1f;           // Unity joint sign convention (verified online)
    private float jointSignEvidence;
    private bool ready;
    private Vector2[] moduleVel = new Vector2[0];

    static readonly string[] ModuleIds = { "fl", "fr", "rl", "rr" };

    // ---------------------------------------------------------------- setup
    void Start()
    {
        if (baseLink == null) baseLink = FindBody("base_link");
        if (modules == null || modules.Length != 4 || Array.Exists(modules, m => m == null || m.steer == null || m.wheel == null))
        {
            modules = new SwerveModule[4];
            for (int i = 0; i < 4; i++)
                modules[i] = new SwerveModule { id = ModuleIds[i], steer = FindBody("steer_" + ModuleIds[i]), wheel = FindBody("wheel_" + ModuleIds[i]) };
        }

        if (baseLink == null || Array.Exists(modules, m => m.steer == null || m.wheel == null))
        {
            Debug.LogError("SwerveDriveController: could not find base_link / steer_xx / wheel_xx ArticulationBodies under " + name, this);
            enabled = false;
            return;
        }

        foreach (var m in modules)
        {
            EnsureFreeRevolute(m.steer);
            EnsureFreeRevolute(m.wheel);
        }
        moduleVel = new Vector2[modules.Length];

        // Robot frame from geometry: up = average steering axis, forward = front modules - rear modules.
        Vector3 upW = Vector3.zero;
        foreach (var m in modules)
        {
            Vector3 a = JointAxisWorld(m.steer);
            upW += Vector3.Dot(a, Vector3.up) >= 0 ? a : -a;
        }
        upW.Normalize();
        Vector3 front = Vector3.zero, back = Vector3.zero, leftSide = Vector3.zero, rightSide = Vector3.zero;
        foreach (var m in modules)
        {
            Vector3 p = m.wheel.worldCenterOfMass;
            if (m.id[0] == 'f') front += p; else back += p;
            if (m.id[1] == 'l') leftSide += p; else rightSide += p;
        }
        Vector3 fwdW = Vector3.ProjectOnPlane(front - back, upW).normalized;
        Vector3 leftW = Vector3.Cross(fwdW, upW); // Unity is left-handed: Cross(forward, up) = left
        if (Vector3.Dot(leftSide - rightSide, leftW) < 0)
            Debug.LogWarning("SwerveDriveController: module names left/right don't match geometry. Check steer_xx naming.", this);

        upLocal = baseLink.transform.InverseTransformDirection(upW);
        fwdLocal = baseLink.transform.InverseTransformDirection(fwdW);

        UpdateFrame();
        startCenter = center;
        startRotation = Quaternion.LookRotation(fwd, up);
        startFwdFlat = Vector3.ProjectOnPlane(fwd, Vector3.up).normalized;
        startLeftFlat = Vector3.Cross(startFwdFlat, Vector3.up);
        ready = true;
    }

    void EnsureFreeRevolute(ArticulationBody b)
    {
        if (b.jointType != ArticulationJointType.RevoluteJoint)
        {
            Debug.LogWarning($"SwerveDriveController: {b.name} was {b.jointType}; switching it to a free Revolute joint.", b);
            b.jointType = ArticulationJointType.RevoluteJoint;
        }
        if (b.twistLock != ArticulationDofLock.FreeMotion) b.twistLock = ArticulationDofLock.FreeMotion;
    }

    ArticulationBody FindBody(string linkName)
    {
        foreach (var b in GetComponentsInChildren<ArticulationBody>(true))
            if (b.name == linkName) return b;
        return null;
    }

    // ---------------------------------------------------------------- input
    void Update()
    {
#if ENABLE_LEGACY_INPUT_MANAGER
        Vector3 k = new Vector3(KeyAxis(KeyCode.W, KeyCode.S), KeyAxis(KeyCode.A, KeyCode.D), KeyAxis(KeyCode.Q, KeyCode.E));
        stopHeld = Input.GetKey(KeyCode.Space);
        keyboardActive = k != Vector3.zero || stopHeld;
        keyboardCommand = new Vector3(k.x * maxLinearSpeed, k.y * maxLinearSpeed, k.z * maxAngularSpeed) * keyboardSpeedFraction;
#else
        if (Time.time < 0.1f) Debug.LogWarning("SwerveDriveController: keyboard needs 'Active Input Handling' = Input Manager (Old) or Both (Project Settings > Player).", this);
#endif
    }

#if ENABLE_LEGACY_INPUT_MANAGER
    static float KeyAxis(KeyCode positive, KeyCode negative)
    {
        return (Input.GetKey(positive) ? 1f : 0f) - (Input.GetKey(negative) ? 1f : 0f);
    }
#endif

    Vector3 SelectCommand()
    {
        bool externalFresh = Time.time - lastExternalTime < externalCommandTimeout;
        bool useKeyboard = source == CommandSource.KeyboardOnly || (source == CommandSource.Auto && keyboardActive);

        if (stopHeld && source != CommandSource.ExternalOnly) { activeSource = "Stop (Space)"; return Vector3.zero; }
        if (useKeyboard)
        {
            activeSource = "Keyboard";
            Vector3 c = keyboardCommand;
            if (keyboardFieldCentric)
            {
                Vector3 vWorld = c.x * startFwdFlat + c.y * startLeftFlat;
                c = new Vector3(Vector3.Dot(vWorld, fwd), Vector3.Dot(vWorld, left), c.z);
            }
            return c;
        }
        if (source != CommandSource.KeyboardOnly && externalFresh) { activeSource = "External (ROS)"; return externalCommand; }
        activeSource = source == CommandSource.ExternalOnly ? "External (timed out)" : "Idle";
        return Vector3.zero;
    }

    // ---------------------------------------------------------------- control loop
    void FixedUpdate()
    {
        if (!ready) return;
        float dt = Time.fixedDeltaTime;
        UpdateFrame();

        // 1. Command, limits, acceleration
        requestedCommand = SelectCommand();
        Vector2 lin = new Vector2(requestedCommand.x, requestedCommand.y);
        if (lin.magnitude > maxLinearSpeed) lin = lin.normalized * maxLinearSpeed;
        float wz = Mathf.Clamp(requestedCommand.z, -maxAngularSpeed, maxAngularSpeed);
        Vector2 linApplied = Vector2.MoveTowards(new Vector2(appliedCommand.x, appliedCommand.y), lin, maxLinearAccel * dt);
        float wzApplied = Mathf.MoveTowards(appliedCommand.z, wz, maxAngularAccel * dt);
        appliedCommand = new Vector3(linApplied.x, linApplied.y, wzApplied);

        // 2-3. Inverse kinematics + desaturation
        float fastest = 0f;
        for (int i = 0; i < modules.Length; i++)
        {
            Vector2 p = modules[i].position;
            moduleVel[i] = new Vector2(appliedCommand.x - appliedCommand.z * p.y, appliedCommand.y + appliedCommand.z * p.x);
            fastest = Mathf.Max(fastest, moduleVel[i].magnitude);
        }
        float scale = fastest > maxWheelSurfaceSpeed ? maxWheelSurfaceSpeed / fastest : 1f;

        // 4-5. Steering + wheels
        for (int i = 0; i < modules.Length; i++)
            DriveModule(modules[i], moduleVel[i] * scale, dt);

        // 6. Odometry
        UpdateOdometry(dt);
        VerifyJointSign();
        ApplyTerrainIntreaction();
    }

    void UpdateFrame()
    {
        up = baseLink.transform.TransformDirection(upLocal).normalized;
        fwd = Vector3.ProjectOnPlane(baseLink.transform.TransformDirection(fwdLocal), up).normalized;
        left = Vector3.Cross(fwd, up);

        center = Vector3.zero;
        foreach (var m in modules) center += m.wheel.worldCenterOfMass;
        center /= modules.Length;
        foreach (var m in modules)
        {
            Vector3 d = m.wheel.worldCenterOfMass - center;
            m.position = new Vector2(Vector3.Dot(d, fwd), Vector3.Dot(d, left));
        }
    }

    void DriveModule(SwerveModule m, Vector2 v, float dt)
    {
        // Direction the module moves on the ground for a positive wheel joint velocity.
        Vector3 rollDir = jointSign * Vector3.Cross(JointAxisWorld(m.wheel), up);
        m.measuredHeading = Mathf.Atan2(Vector3.Dot(rollDir, left), Vector3.Dot(rollDir, fwd));
        if (!m.initialized)
        {
            m.commandedHeading = m.holdHeading = m.measuredHeading;
            m.initialized = true;
        }

        // Desired heading; keep the last one while stopped instead of snapping back to 0.
        float speed = v.magnitude;
        if (speed > 1e-3f) m.holdHeading = Mathf.Atan2(v.y, v.x);
        float desired = m.holdHeading;
        float wheelDir = 1f;
        if (reverseWheelInsteadOfTurningPast90 && Mathf.Abs(Wrap(desired - m.measuredHeading)) > Mathf.PI / 2f)
        {
            desired = Wrap(desired + Mathf.PI);
            wheelDir = -1f;
        }

        // Angle normalization (+-180 deg) and steering rate limit.
        float previous = m.commandedHeading;
        float step = Mathf.Clamp(Wrap(desired - m.commandedHeading), -maxSteerRate * dt, maxSteerRate * dt);
        m.commandedHeading = Wrap(m.commandedHeading + step);
        float headingRate = Wrap(m.commandedHeading - previous) / dt;

        // Heading -> joint angle: d(heading)/d(joint) from the actual steering axis (about +-1).
        Vector3 h = Mathf.Cos(m.measuredHeading) * fwd + Mathf.Sin(m.measuredHeading) * left;
        float gain = jointSign * Vector3.Dot(Vector3.Cross(JointAxisWorld(m.steer), h), Vector3.Cross(h, up));
        if (Mathf.Abs(gain) > 0.2f)
        {
            float q = m.steer.jointPosition[0];
            ArticulationDrive sd = m.steer.xDrive;
            sd.stiffness = steerStiffness;
            sd.damping = steerDamping;
            sd.forceLimit = steerForceLimit;
            sd.target = (q + Wrap(m.commandedHeading - m.measuredHeading) / gain) * Mathf.Rad2Deg;
            sd.targetVelocity = headingRate / gain * Mathf.Rad2Deg;
            m.steer.xDrive = sd;
        }

        // Wheel speed along the module's actual heading.
        float s = wheelDir * speed;
        if (slowWheelsWhileSteering) s *= Mathf.Clamp01(Mathf.Cos(Wrap(desired - m.measuredHeading)));
        m.targetSpeed = s;

        // Motor Torque Dynamics
        float wheelRadPerSec = Mathf.Abs(m.wheel.jointVelocity[0]);
        float wheelRPM = wheelRadPerSec * (60.0f / (2.0f * Mathf.PI));
        float motorRPM = wheelRPM * gearRatio;

        // EMF Curve
        float availableMotorTorque = Mathf.Clamp(
            motorStallTorque * (1.0f - (motorRPM / motorFreeSpeedRPM)), 0.0f, motorStallTorque
        );

        // Torque at wheel axle
        float maxAxleTorque = availableMotorTorque * gearRatio * gearEfficiency;

        ArticulationDrive wd = m.wheel.xDrive;
        wd.stiffness = 0f;
        wd.damping = wheelDamping;
        wd.forceLimit = Mathf.Max(maxAxleTorque, 0.5f);
        wd.targetVelocity = s / wheelRadius * Mathf.Rad2Deg;
        m.wheel.xDrive = wd;

        m.measuredSpeed = m.wheel.jointVelocity[0] * wheelRadius;
    }

    void UpdateOdometry(float dt)
    {
        // Least-squares rigid-body fit of the 4 measured wheel velocities (positions are centered).
        Vector2 sum = Vector2.zero;
        float num = 0f, den = 0f;
        foreach (var m in modules)
        {
            Vector2 u = m.measuredSpeed * new Vector2(Mathf.Cos(m.measuredHeading), Mathf.Sin(m.measuredHeading));
            sum += u;
            num += m.position.x * u.y - m.position.y * u.x;
            den += m.position.sqrMagnitude;
        }
        float vx = sum.x / modules.Length, vy = sum.y / modules.Length;
        float wz = den > 1e-6f ? num / den : 0f;
        odometryTwist = new Vector3(vx, vy, wz);

        float yawMid = odometryPose.z + 0.5f * wz * dt;
        odometryPose.x += (vx * Mathf.Cos(yawMid) - vy * Mathf.Sin(yawMid)) * dt;
        odometryPose.y += (vx * Mathf.Sin(yawMid) + vy * Mathf.Cos(yawMid)) * dt;
        odometryPose.z = Wrap(odometryPose.z + wz * dt);
    }

    /// <summary>
    /// The math assumes a positive joint velocity rotates the child about the joint's X axis
    /// following Unity's Quaternion.AngleAxis convention. This checks it on the spinning wheels
    /// and flips the convention once (with a warning) if the physics says otherwise.
    /// </summary>
    void VerifyJointSign()
    {
        foreach (var m in modules)
        {
            float jv = m.wheel.jointVelocity[0];
            if (Mathf.Abs(jv) < 2f) continue;
            float rel = Vector3.Dot(m.wheel.angularVelocity - m.steer.angularVelocity, JointAxisWorld(m.wheel));
            if (Mathf.Abs(rel) < 1f) continue;
            jointSignEvidence += Mathf.Sign(rel * jv) * jointSign;
        }
        if (jointSignEvidence < -50f)
        {
            jointSign = -jointSign;
            jointSignEvidence = 0f;
            foreach (var m in modules) m.initialized = false;
            Debug.LogWarning("SwerveDriveController: joint sign convention was inverted; corrected automatically.", this);
        }
        jointSignEvidence = Mathf.Clamp(jointSignEvidence, -100f, 100f);
    }

    static Vector3 JointAxisWorld(ArticulationBody b) => (b.transform.rotation * b.anchorRotation) * Vector3.right;

    static float Wrap(float a) => Mathf.Repeat(a + Mathf.PI, 2f * Mathf.PI) - Mathf.PI;

    // ---------------------------------------------------------------- debug
    void OnGUI()
    {
        if (!showHud || !ready) return;
        var sb = new System.Text.StringBuilder();
        sb.AppendLine($"Source: {activeSource}");
        sb.AppendLine($"Cmd   vx {appliedCommand.x,6:F2}  vy {appliedCommand.y,6:F2}  wz {appliedCommand.z,6:F2}");
        sb.AppendLine($"Odom  vx {odometryTwist.x,6:F2}  vy {odometryTwist.y,6:F2}  wz {odometryTwist.z,6:F2}");
        sb.AppendLine($"Pose  x {odometryPose.x,6:F2}  y {odometryPose.y,6:F2}  yaw {odometryPose.z * Mathf.Rad2Deg,6:F1} deg");
        foreach (var m in modules)
            sb.AppendLine($"{m.id}: steer {m.measuredHeading * Mathf.Rad2Deg,6:F1} deg (cmd {m.commandedHeading * Mathf.Rad2Deg,6:F1})  wheel {m.measuredSpeed,5:F2} / {m.targetSpeed,5:F2} m/s");
        sb.Append("W/S fwd  A/D strafe  Q/E rotate  Space stop");
        GUI.Box(new Rect(10, 10, 430, 175), "");
        GUI.Label(new Rect(18, 14, 420, 170), sb.ToString());
    }

    void OnDrawGizmos()
    {
        if (!drawGizmos || !ready || !Application.isPlaying) return;
        foreach (var m in modules)
        {
            Vector3 p = m.wheel.worldCenterOfMass;
            Vector3 dc = Mathf.Cos(m.commandedHeading) * fwd + Mathf.Sin(m.commandedHeading) * left;
            Vector3 dm = Mathf.Cos(m.measuredHeading) * fwd + Mathf.Sin(m.measuredHeading) * left;
            Gizmos.color = Color.green;
            Gizmos.DrawLine(p, p + dc * (0.25f + 0.25f * Mathf.Abs(m.targetSpeed)) * Mathf.Sign(m.targetSpeed == 0 ? 1 : m.targetSpeed));
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(p + up * 0.02f, p + up * 0.02f + dm * 0.2f);
        }
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(center, center + fwd * 0.6f);
    }

    void ApplyTerrainIntreaction()
    {
        foreach(var m in modules)
        {
            Vector3 wheelPos = m.wheel.worldCenterOfMass;
            // Init ray to start 20 cm (wheel radius is 11 cm) above wheel center
            Ray ray = new Ray(wheelPos + Vector3.up * 0.2f, Vector3.down); // Raycasting

            // Shoot a ray 50cm down
            if (Physics.Raycast(ray, out RaycastHit hit, 0.5f))
            {
                Terrain terrain = hit.collider.GetComponent<Terrain>();
                float localResistance = rollingResistanceCoeff;

                // TODO: When splatmap implemented sample what splatmap it is to better model soil interactions

                Vector3 wheelLinearVel = m.wheel.velocity;
                Vector3 rollVel = Vector3.ProjectOnPlane(wheelLinearVel, hit.normal);
                
                // Deadzone check actual speed is speed > 3cm /sec. Prevents parked rover from vibrating back and forth
                if (rollVel.sqrMagnitude > 0.001f)
                {
                    float totalRoverMass = 45.5f;
                    float cosTheta = Mathf.Max(Vector3.Dot(hit.normal, Vector3.up), 0.1f);
                    float normalForce = (totalRoverMass / 4.0f) * Physics.gravity.magnitude * cosTheta; // Divide mass equally across load bearing wheels (TODO check with mechanical)
                    Vector3 resistanceForce = -rollVel.normalized * localResistance * normalForce;
                    m.wheel.AddForce(resistanceForce, ForceMode.Force);
                }
            }
        }
    }
}
