using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.UR3eHande;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TrajectoryPlannerRTA : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const int k_NumRobotFingers = 2;
    const float k_JointAssignmentWait = 0.01f;
    const float k_FingerAssignmentWait = 0.001f;
    const float k_PoseAssignmentWait = 0.5f;

    const int k_NumBalls = 4;
    int k_NumPlacedBalls = 0;

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
    
    readonly Vector3 m_TableOffset = new Vector3(0, -0.914f, 0);
    readonly Vector3 m_BasketOffset = new Vector3(-0.15f, 0.1f, 0.15f);

    // Articulation Bodies
    ArticulationBody[] m_UR3eArticulationBodies;
    ArticulationBody[] m_HandeArticulationBodies;
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
        m_Ros.RegisterRosService<UR3ePickAndPlaceRTARequest, UR3ePickAndPlaceRTAResponse>(m_RosServiceName);

        m_UR3eArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_UR3eArticulationBodies[i] = m_UR3e.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        m_HandeArticulationBodies = new ArticulationBody[k_NumRobotFingers];

        // Find left and right fingers
        var rightGripper = linkName + "/flange/tool0/linkage/hand_e_link/hande_right_finger";
        var leftGripper = linkName + "/flange/tool0/linkage/hand_e_link/hande_left_finger";

        m_HandeArticulationBodies[0] = m_UR3e.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_HandeArticulationBodies[1] = m_UR3e.transform.Find(leftGripper).GetComponent<ArticulationBody>();
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>UR3eStateMsg</returns>
    UR3eStateMsg CurrentJointState()
    {
        var state = new UR3eStateMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            state.joints[i] = m_UR3eArticulationBodies[i].jointPosition[0];
        }

        state.fingers[0] = m_HandeArticulationBodies[0].jointPosition[0];

        return state;
    }

    UR3eTargetMsg CreateMsg(string poseType, string color, Vector3 position, Quaternion orientation)
    {
        if (poseType == "pick")
        {
            position += m_TableOffset;
        };

        if (poseType == "place")
        {
            position += m_TableOffset + m_BasketOffset;
        };

        return new UR3eTargetMsg
        {
            color = new StringMsg(color),
            pose = new PoseMsg
            {
                position = position.To<FLU>(),
                orientation = orientation.To<FLU>()
            }
        };
    }

    public void StartPickAndPlace()
    {
        PublishJoints(k_NumPlacedBalls);
    }

    /// <summary>
    ///     Create a new UR3ePickAndPlaceRTARequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the UR3eMoveService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints(int ballIndex)
    {
        var request = new UR3ePickAndPlaceRTARequest();
        request.state = CurrentJointState();
        
        switch (ballIndex)
        {
            case 0:
                // Red Ball 1 Pick Pose
                request.ball = CreateMsg("pick", "red", m_RedBall1.transform.position, m_PickOrientation);
                // Red Balls Place Pose
                request.basket = CreateMsg("place", "red", m_RedBallBasket.transform.position, m_PlaceOrientationRed);
                break;

            case 1:
                // Red Ball 2 Pick Pose
                request.ball = CreateMsg("pick", "red", m_RedBall2.transform.position, m_PickOrientation);
                // Red Balls Place Pose
                request.basket = CreateMsg("place", "red", m_RedBallBasket.transform.position, m_PlaceOrientationRed);
                break;

            case 2:
                // Blue Ball 1 Pick Pose
                request.ball = CreateMsg("pick", "blue", m_BlueBall1.transform.position, m_PickOrientation);
                // Blue Balls Place Pose
                request.basket = CreateMsg("place", "blue", m_BlueBallBasket.transform.position, m_PlaceOrientationBlue);
                break;

            case 3:
                // Blue Ball 2 Pick Pose
                request.ball = CreateMsg("pick", "blue", m_BlueBall2.transform.position, m_PickOrientation);
                // Blue Balls Place Pose
                request.basket = CreateMsg("place", "blue", m_BlueBallBasket.transform.position, m_PlaceOrientationBlue);
                break;

            default:
                break;
        }

        m_Ros.SendServiceMessage<UR3ePickAndPlaceRTAResponse>(m_RosServiceName, request, TrajectoryResponse);
    }
    void TrajectoryResponse(UR3ePickAndPlaceRTAResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            PublishJoints(k_NumPlacedBalls);
        }
    }

    public void GoToNextBall()
    {
        if (k_NumPlacedBalls + 1 >= k_NumBalls)
        {
            Debug.Log("All balls are placed correctly.");
        }
        else
        {
            k_NumPlacedBalls += 1;
            PublishJoints(k_NumPlacedBalls);
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
    /// <param name="response"> UR3ePickAndPlaceRTAResponse received from niryo_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(UR3ePickAndPlaceRTAResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                if (response.trajectories[poseIndex].group.data == "arm")
                {
                    // For every robot pose in trajectory plan
                    foreach (var t in response.trajectories[poseIndex].trajectory.joint_trajectory.points)
                    {
                        var jointPositions = t.positions;
                        var result = jointPositions.Select(r => (float) r * Mathf.Rad2Deg).ToArray();

                        // Set the joint values for every joint
                        for (var joint = 0; joint < m_UR3eArticulationBodies.Length; joint++)
                        {
                            var jointXDrive = m_UR3eArticulationBodies[joint].xDrive;
                            jointXDrive.target = result[joint];
                            m_UR3eArticulationBodies[joint].xDrive = jointXDrive;
                        }

                        // Wait for robot to achieve pose for all joint assignments
                        yield return new WaitForSeconds(k_JointAssignmentWait);
                    }
                }

                if (response.trajectories[poseIndex].group.data == "gripper")
                {
                    // For every robot pose in trajectory plan
                    foreach (var t in response.trajectories[poseIndex].trajectory.joint_trajectory.points)
                    {
                        var jointPositions = t.positions;
                        var result = jointPositions.Select(r => (float) r).ToArray();

                        // Set the joint values for every joint
                        for (var joint = 0; joint < m_HandeArticulationBodies.Length; joint++)
                        {
                            var fingerXDrive = m_HandeArticulationBodies[joint].xDrive;
                            fingerXDrive.target = result[0];
                            m_HandeArticulationBodies[joint].xDrive = fingerXDrive;
                        }

                        // Wait for robot to achieve pose for all joint assignments
                        yield return new WaitForSeconds(k_FingerAssignmentWait);
                    }
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);

                if (poseIndex + 1 == response.trajectories.Length || response.trajectories[0].group.data == "none")
                {
                    GoToNextBall();
                }
            }
        }
    }
}
