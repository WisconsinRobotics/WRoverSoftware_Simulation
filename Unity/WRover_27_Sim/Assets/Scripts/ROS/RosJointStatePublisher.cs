using System.Collections.Generic;
using System.Linq;
using RosSharp.RosBridgeClient;
using UnityEngine;
using JointState = RosSharp.RosBridgeClient.MessageTypes.Sensor.JointState;
using Header = RosSharp.RosBridgeClient.MessageTypes.Std.Header;

/// <summary>
/// Publishes sensor_msgs/JointState (default /joint_states) for every movable ArticulationBody
/// joint of the rover over rosbridge (ROS#). Joint names match wrover_27_unity.urdf.
/// Position: rad (revolute) or m (prismatic). Velocity: rad/s or m/s. Effort: N*m or N.
/// Put it on the RosConnector GameObject.
/// </summary>
public class RosJointStatePublisher : UnityPublisher<JointState>
{
    [Tooltip("Robot root. Empty = the GameObject with the SwerveDriveController.")]
    public Transform robotRoot;
    public string frameId = "";
    public float publishRateHz = 30f;

    // Link name -> URDF joint name, where it isn't simply "<link>_joint".
    static readonly Dictionary<string, string> JointNames = new Dictionary<string, string>
    {
        { "arm_column", "arm_slide_joint" },
        { "arm_carriage", "arm_lift_joint" },
        { "arm_supinator", "arm_roll_joint" },
        { "wrist_differential", "wrist_pitch_joint" },
        { "manipulator", "wrist_roll_joint" },
        { "claw", "claw_joint" },
    };

    private List<ArticulationBody> joints = new List<ArticulationBody>();
    private JointState message;
    private RosConnector connector;
    private bool advertised;
    private float nextPublish;

    void Reset() { Topic = "/joint_states"; }

    protected override void Start()
    {
        if (string.IsNullOrEmpty(Topic)) Topic = "/joint_states";
        if (robotRoot == null)
        {
            var c = FindObjectOfType<SwerveDriveController>();
            if (c != null) robotRoot = c.transform;
        }
        connector = GetComponent<RosConnector>();
        if (robotRoot == null) { Debug.LogError("RosJointStatePublisher: no robot root assigned.", this); enabled = false; return; }

        joints = robotRoot.GetComponentsInChildren<ArticulationBody>(true)
            .Where(b => !b.isRoot && b.jointType != ArticulationJointType.FixedJoint)
            .OrderBy(b => b.name)
            .ToList();

        int n = joints.Count;
        message = new JointState
        {
            header = new Header { frame_id = frameId },
            name = joints.Select(b => JointNames.TryGetValue(b.name, out var j) ? j : b.name + "_joint").ToArray(),
            position = new double[n],
            velocity = new double[n],
            effort = new double[n]
        };
        // base.Start() (advertise) runs once rosbridge is connected, see FixedUpdate.
    }

    void FixedUpdate()
    {
        if (message == null || connector == null || connector.IsConnected == null) return;
        if (!connector.IsConnected.WaitOne(0)) return;
        if (!advertised) { base.Start(); advertised = true; }
        if (Time.time < nextPublish) return;
        nextPublish = Time.time + 1f / Mathf.Max(1f, publishRateHz);

        for (int i = 0; i < joints.Count; i++)
        {
            var b = joints[i];
            if (b.dofCount < 1) continue;
            message.position[i] = b.jointPosition[0];
            message.velocity[i] = b.jointVelocity[0];
            message.effort[i] = b.jointForce[0];
        }
        message.header.Update();
        Publish(message);
    }
}
