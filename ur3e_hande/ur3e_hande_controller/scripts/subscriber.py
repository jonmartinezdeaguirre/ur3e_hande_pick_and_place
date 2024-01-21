#!/usr/bin/env python

import rospy

from ur3e_hande_controller.msg import UR3eJoints

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard:\n%s", data)

def listener():
    rospy.init_node('trajectory_subscriber', anonymous=True)
    rospy.Subscriber("/ur3e_targets", UR3eJoints, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()