#!/usr/bin/env python3
import time
import math
import rospy
from geometry_msgs.msg import Pose as Pose
from sensor_msgs.msg import JointState as JointMsg
import numpy as np
from std_msgs.msg import Float64 as F64
import roslib


if __name__ == "__main__":

    rospy.init_node('dvrk_pub_py')
    rate = rospy.Rate(1000)

    f_pub = rospy.Publisher("/dvrk/data", F64, queue_size= 1000)
    q_pub = rospy.Publisher("/dvrk/q_state", JointMsg, queue_size= 1000)
    p_pub = rospy.Publisher("/dvrk/pose", Pose, queue_size= 1000)

    q = np.zeros(6)
    q_msg = JointMsg()
    pos = np.zeros(3)
    quat = np.zeros(3)
    w = 0
    p_msg = Pose()

    while not rospy.is_shutdown():

        K = 270.50
        f_pub.publish(K)

        q[0] = 1
        q[1] = 2
        q[2] = 3
        q[3] = 4
        q[4] = 5
        q[5] = 6

        q_msg.position = q
        q_pub.publish(q_msg)

        pos[0] = 1
        pos[1] = 2
        pos[2] = 3
        quat[0] = 4
        quat[1] = 5
        quat[2] = 6
        w = 1

        p_msg.position.x = pos[0]
        p_msg.position.y = pos[1]
        p_msg.position.z = pos[2]

        p_msg.orientation.x = quat[0]
        p_msg.orientation.y = quat[1]
        p_msg.orientation.z = quat[2]
        p_msg.orientation.w = w

        p_pub.publish(p_msg)

        rate.sleep()
        #rospy.spin()
