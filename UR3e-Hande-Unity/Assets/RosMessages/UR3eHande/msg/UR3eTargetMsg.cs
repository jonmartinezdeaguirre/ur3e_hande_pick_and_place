//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UR3eHande
{
    [Serializable]
    public class UR3eTargetMsg : Message
    {
        public const string k_RosMessageName = "ur3e_hande_controller/UR3eTarget";
        public override string RosMessageName => k_RosMessageName;

        public Std.StringMsg color;
        public Geometry.PoseMsg pose;

        public UR3eTargetMsg()
        {
            this.color = new Std.StringMsg();
            this.pose = new Geometry.PoseMsg();
        }

        public UR3eTargetMsg(Std.StringMsg color, Geometry.PoseMsg pose)
        {
            this.color = color;
            this.pose = pose;
        }

        public static UR3eTargetMsg Deserialize(MessageDeserializer deserializer) => new UR3eTargetMsg(deserializer);

        private UR3eTargetMsg(MessageDeserializer deserializer)
        {
            this.color = Std.StringMsg.Deserialize(deserializer);
            this.pose = Geometry.PoseMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.color);
            serializer.Write(this.pose);
        }

        public override string ToString()
        {
            return "UR3eTargetMsg: " +
            "\ncolor: " + color.ToString() +
            "\npose: " + pose.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
