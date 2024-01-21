//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UR3eHande
{
    [Serializable]
    public class UR3ePickAndPlaceRTAResponse : Message
    {
        public const string k_RosMessageName = "ur3e_hande_controller/UR3ePickAndPlaceRTA";
        public override string RosMessageName => k_RosMessageName;

        public UR3eTrajectoryRTAMsg[] trajectories;

        public UR3ePickAndPlaceRTAResponse()
        {
            this.trajectories = new UR3eTrajectoryRTAMsg[0];
        }

        public UR3ePickAndPlaceRTAResponse(UR3eTrajectoryRTAMsg[] trajectories)
        {
            this.trajectories = trajectories;
        }

        public static UR3ePickAndPlaceRTAResponse Deserialize(MessageDeserializer deserializer) => new UR3ePickAndPlaceRTAResponse(deserializer);

        private UR3ePickAndPlaceRTAResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.trajectories, UR3eTrajectoryRTAMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.trajectories);
            serializer.Write(this.trajectories);
        }

        public override string ToString()
        {
            return "UR3ePickAndPlaceRTAResponse: " +
            "\ntrajectories: " + System.String.Join(", ", trajectories.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
