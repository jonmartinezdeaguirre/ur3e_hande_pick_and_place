using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.UR3eHande;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class SourceDestinationPublisher : MonoBehaviour
{
    const int k_NumRobotJoints = 6;

    public static readonly string[] LinkNames =
        { "base_link/base_link_inertia/shoulder_link", "/upper_arm_link", "/forearm_link", "/wrist_1_link", "/wrist_2_link", "/wrist_3_link" };

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/ur3e_targets";

    [SerializeField]
    GameObject m_UR3e;
    [SerializeField]
    GameObject m_RedBall1;
    [SerializeField]
    GameObject m_RedBall2;
    [SerializeField]
    GameObject m_BlueBall1;
    [SerializeField]
    GameObject m_BlueBall2;
    [SerializeField]
    GameObject m_RedBallBasket;
    [SerializeField]
    GameObject m_BlueBallBasket;

    readonly Quaternion m_PickOrientation = Quaternion.Euler(-90, 0, 0);

    // Robot Joints
    UrdfJointRevolute[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<UR3eTargetsMsg>(m_TopicName);

        m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = m_UR3e.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
        }
    }

    public void Publish()
    {
        var sourceDestinationMessage = new UR3eTargetsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            sourceDestinationMessage.targets[i] = m_JointArticulationBodies[i].GetPosition();
        }

        // Red Ball 1 Pick Pose
        sourceDestinationMessage.redBall1_pickPose = new PoseMsg
        {
            position = m_RedBall1.transform.position.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        // Red Ball 2 Pick Pose
        sourceDestinationMessage.redBall2_pickPose = new PoseMsg
        {
            position = m_RedBall2.transform.position.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        // Blue Ball 1 Pick Pose
        sourceDestinationMessage.blueBall1_pickPose = new PoseMsg
        {
            position = m_BlueBall1.transform.position.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        // Blue Ball 2 Pick Pose
        sourceDestinationMessage.blueBall2_pickPose = new PoseMsg
        {
            position = m_BlueBall2.transform.position.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        // Blue Balls Place Pose
        sourceDestinationMessage.redBall_placePose = new PoseMsg
        {
            position = m_RedBallBasket.transform.position.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        // Blue Balls Place Pose
        sourceDestinationMessage.blueBall_placePose = new PoseMsg
        {
            position = m_BlueBallBasket.transform.position.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        // Finally send the message to server_endpoint.py running in ROS
        m_Ros.Publish(m_TopicName, sourceDestinationMessage);
    }
}
