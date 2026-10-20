using RosSharp;
using RosSharp.RosBridgeClient;
using UnityEngine;
using Odometry = RosSharp.RosBridgeClient.MessageTypes.Nav.Odometry;

/// <summary>
/// Publishes nav_msgs/Odometry from the SwerveDriveController over rosbridge (ROS#).
///  - WheelOdometry: integrated from the simulated wheel/steering joints (drifts like the real one).
///  - GroundTruth:   the true pose from the physics engine, relative to the start pose.
/// Add one component per topic (e.g. /odom and /ground_truth/odom) on the RosConnector GameObject.
/// Frame convention (REP-103): x forward, y left, z up; child frame = rover center between the 4 wheels.
/// </summary>
public class RosOdometryPublisher : UnityPublisher<Odometry>
{
    public enum Source { WheelOdometry, GroundTruth }

    public SwerveDriveController controller;
    public Source source = Source.WheelOdometry;
    public string frameId = "odom";
    public string childFrameId = "base_link";
    public float publishRateHz = 30f;

    private Odometry message;
    private RosConnector connector;
    private bool advertised;
    private float nextPublish;

    void Reset() { Topic = "/odom"; }

    protected override void Start()
    {
        if (string.IsNullOrEmpty(Topic)) Topic = source == Source.GroundTruth ? "/ground_truth/odom" : "/odom";
        if (controller == null) controller = FindObjectOfType<SwerveDriveController>();
        connector = GetComponent<RosConnector>();
        message = new Odometry();
        message.header.frame_id = frameId;
        message.child_frame_id = childFrameId;
        // base.Start() (advertise) runs once rosbridge is connected, see FixedUpdate.
    }

    void FixedUpdate()
    {
        if (controller == null || !controller.IsReady || connector == null || connector.IsConnected == null) return;
        if (!connector.IsConnected.WaitOne(0)) return;
        if (!advertised) { base.Start(); advertised = true; }
        if (Time.time < nextPublish) return;
        nextPublish = Time.time + 1f / Mathf.Max(1f, publishRateHz);

        Vector3 pos; Quaternion rot; Vector3 twist;
        if (source == Source.WheelOdometry)
        {
            Vector3 p = controller.OdometryPose;      // x, y, yaw (ROS convention already)
            pos = new Vector3(p.x, p.y, 0f);
            rot = new Quaternion(0f, 0f, Mathf.Sin(p.z / 2f), Mathf.Cos(p.z / 2f));
            twist = controller.OdometryTwist;
        }
        else
        {
            controller.GetGroundTruth(out Vector3 relPos, out Quaternion relRot, out twist);
            pos = relPos.Unity2Ros();
            rot = relRot.Unity2Ros();
        }

        var pp = message.pose.pose;
        pp.position.x = pos.x; pp.position.y = pos.y; pp.position.z = pos.z;
        pp.orientation.x = rot.x; pp.orientation.y = rot.y; pp.orientation.z = rot.z; pp.orientation.w = rot.w;
        var tw = message.twist.twist;
        tw.linear.x = twist.x; tw.linear.y = twist.y; tw.linear.z = 0;
        tw.angular.x = 0; tw.angular.y = 0; tw.angular.z = twist.z;

        message.header.Update();
        Publish(message);
    }
}
