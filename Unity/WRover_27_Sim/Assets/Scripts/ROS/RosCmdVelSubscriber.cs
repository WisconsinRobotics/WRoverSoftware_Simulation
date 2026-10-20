using RosSharp.RosBridgeClient;
using UnityEngine;
using Twist = RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist;

/// <summary>
/// Subscribes to geometry_msgs/Twist (default /cmd_vel) over rosbridge (ROS#) and passes it to
/// the SwerveDriveController: linear.x = forward, linear.y = left, angular.z = CCW yaw rate.
/// Put it on the same GameObject as the RosConnector.
/// Messages arrive on a background thread; they are applied in FixedUpdate on the main thread.
/// </summary>
public class RosCmdVelSubscriber : UnitySubscriber<Twist>
{
    public SwerveDriveController controller;

    private readonly object gate = new object();
    private Vector3 pending;
    private bool hasNew;
    private bool subscribed;
    private RosConnector connector;

    void Reset() { Topic = "/cmd_vel"; }

    protected override void Start()
    {
        if (string.IsNullOrEmpty(Topic)) Topic = "/cmd_vel";
        if (controller == null) controller = FindObjectOfType<SwerveDriveController>();
        connector = GetComponent<RosConnector>();
        // base.Start() (the actual subscribe) runs once rosbridge is connected, see FixedUpdate.
    }

    protected override void ReceiveMessage(Twist message)
    {
        lock (gate)
        {
            pending = new Vector3((float)message.linear.x, (float)message.linear.y, (float)message.angular.z);
            hasNew = true;
        }
    }

    void FixedUpdate()
    {
        if (!subscribed && connector != null && connector.IsConnected != null && connector.IsConnected.WaitOne(0))
        {
            base.Start();
            subscribed = true;
        }

        Vector3 cmd;
        bool fresh;
        lock (gate)
        {
            cmd = pending;
            fresh = hasNew;
            hasNew = false;
        }
        if (fresh && controller != null) controller.SetCommand(cmd.x, cmd.y, cmd.z);
    }
}
