using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.UR3eHande;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "ur3e_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_UR3e;
    public GameObject UR3e { get => m_UR3e; set => m_UR3e = value; }
    [SerializeField]
    GameObject m_RedBall1;
    public GameObject RedBall1 { get => m_RedBall1; set => m_RedBall1 = value; }
    [SerializeField]
    GameObject m_RedBall2;
    public GameObject RedBall2 { get => m_RedBall2; set => m_RedBall2 = value; }
    [SerializeField]
    GameObject m_BlueBall1;
    public GameObject BlueBall1 { get => m_BlueBall1; set => m_BlueBall1 = value; }
    [SerializeField]
    GameObject m_BlueBall2;
    public GameObject BlueBall2 { get => m_BlueBall2; set => m_BlueBall2 = value; }
    [SerializeField]
    GameObject m_RedBallBasket;
    public GameObject RedBallBasket { get => m_RedBallBasket; set => m_RedBallBasket = value; }
    [SerializeField]
    GameObject m_BlueBallBasket;
    public GameObject BlueBallBasket { get => m_BlueBallBasket; set => m_BlueBallBasket = value; }

    // Assures that the gripper is always positioned above the target ball before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(0, 90, 180);
    readonly Quaternion m_PlaceOrientationRed = Quaternion.Euler(0, 0, 180);
    readonly Quaternion m_PlaceOrientationBlue = Quaternion.Euler(0, 180, 180);
    
    readonly Vector3 m_TableOffset = new Vector3(0, -0.9145f, 0);
    readonly Vector3 m_BasketOffset = new Vector3(-0.15f, 0.1f, 0.15f);

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<UR3ePickAndPlaceRequest, UR3ePickAndPlaceResponse>(m_RosServiceName);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];
        // m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints + 2];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_UR3e.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // Find left and right fingers
        var rightGripper = linkName + "/flange/tool0/linkage/hand_e_link/hande_right_finger";
        var leftGripper = linkName + "/flange/tool0/linkage/hand_e_link/hande_left_finger";

        m_RightGripper = m_UR3e.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_UR3e.transform.Find(leftGripper).GetComponent<ArticulationBody>();
        // m_JointArticulationBodies[6] = m_UR3e.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        // m_JointArticulationBodies[7]= m_UR3e.transform.Find(leftGripper).GetComponent<ArticulationBody>();
    }

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = 0.0075f;
        rightDrive.target = 0.0075f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -0.0075f;
        rightDrive.target = -0.0075f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>UR3eTargetsMsg</returns>
    UR3eTargetsMsg CurrentJointConfig()
    {
        var targets = new UR3eTargetsMsg();
        
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            targets.targets[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return targets;
    }

    PoseMsg CreateMsg(string poseType, Vector3 position, Quaternion orientation)
    {
        if (poseType == "Pick")
        {
            position += m_TableOffset;
        };

        if (poseType == "Place")
        {
            position += m_TableOffset + m_BasketOffset;
        };

        return new PoseMsg
        {
            position = position.To<FLU>(),
            orientation = orientation.To<FLU>()
        };
    }

    /// <summary>
    ///     Create a new UR3ePickAndPlaceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the UR3eMoveService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        var request = new UR3ePickAndPlaceRequest();
        request.targets_input = CurrentJointConfig();
        
        // Red Ball 1 Pick Pose
        request.redBall1_pickPose = CreateMsg("Pick", m_RedBall1.transform.position, m_PickOrientation);

        // Red Ball 2 Pick Pose
        request.redBall2_pickPose = CreateMsg("Pick", m_RedBall2.transform.position, m_PickOrientation);

        // Blue Ball 1 Pick Pose
        request.blueBall1_pickPose = CreateMsg("Pick", m_BlueBall1.transform.position, m_PickOrientation);

        // Blue Ball 2 Pick Pose
        request.blueBall2_pickPose = CreateMsg("Pick", m_BlueBall2.transform.position, m_PickOrientation);

        // Red Balls Place Pose
        request.redBall_placePose = CreateMsg("Place", m_RedBallBasket.transform.position, m_PlaceOrientationRed);

        // Red Blue Place Pose
        request.blueBall_placePose = CreateMsg("Place", m_BlueBallBasket.transform.position, m_PlaceOrientationBlue);

        m_Ros.SendServiceMessage<UR3ePickAndPlaceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }
    void TrajectoryResponse(UR3ePickAndPlaceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from UR3eMoveService.");
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the UR3eMoveService.
    ///     The expectation is that the UR3eMoveService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> UR3ePickAndPlaceResponse received from niryo_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(UR3ePickAndPlaceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float) r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp poses
                if (poseIndex == 1 || poseIndex == 9 || poseIndex == 17 || poseIndex == 25)
                {
                    CloseGripper();
                }

                // Open the gripper if completed executing the trajectory for the Place poses
                if (poseIndex == 5 || poseIndex == 13 || poseIndex == 21 || poseIndex == 29)
                {
                    OpenGripper();
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }
        }
    }

    enum Poses
    {
        PreGraspR1,
        GraspR1,        // Index: 1 (Close)
        PickUpR1,
        PrePlaceR1_offX,
        PrePlaceR1,
        PlaceR1,        // Index: 5 (Open)
        PostPlaceR1,
        PostPlaceR1_offX,
        PreGraspR2,
        GraspR2,        // Index: 9 (Close)
        PickUpR2,
        PrePlaceR2_offX,
        PrePlaceR2,
        PlaceR2,        // Index: 13 (Open)
        PostPlaceR2,
        PostPlaceR2_offX,
        PreGraspB1,
        GraspB1,        // Index: 17 (Close)
        PickUpB1,
        PrePlaceB1_offX,
        PrePlaceB1,
        PlaceB1,        // Index: 21 (Open)
        PostPlaceB1,
        PostPlaceB1_offX,
        PreGraspB2,
        GraspB2,        // Index: 25 (Close)
        PickUpB2,
        PrePlaceB2_offX,
        PrePlaceB2,
        PlaceB2,        // Index: 29 (Open)
        PostPlaceB2,
        PostPlaceB2_offX,
    }
}
