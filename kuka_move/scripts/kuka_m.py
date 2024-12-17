#!/usr/bin/env python3
import time
import math
import rospy
from geometry_msgs.msg import Pose as Pose
from sensor_msgs.msg import JointState as JointMsg
import numpy as np
from std_msgs.msg import Float64 as F64
import roslib

class KukaPub:

    def __init__(self):
        self.q = np.zeros(7)
        self.q_sub= rospy.Subscriber ("/iiwa/joint_states", JointMsg, self.q_callback, queue_size= 1000)
        self.f_pub_1 = rospy.Publisher("/iiwa/joint1_position_controller/command", F64, queue_size= 1000)
        self.f_pub_2 = rospy.Publisher("/iiwa/joint2_position_controller/command", F64, queue_size= 1000)
        self.f_pub_3 = rospy.Publisher("/iiwa/joint3_position_controller/command", F64, queue_size= 1000)
        self.f_pub_4 = rospy.Publisher("/iiwa/joint4_position_controller/command", F64, queue_size= 1000)
        self.f_pub_5 = rospy.Publisher("/iiwa/joint5_position_controller/command", F64, queue_size= 1000)
        self.f_pub_6 = rospy.Publisher("/iiwa/joint6_position_controller/command", F64, queue_size= 1000)
        self.f_pub_7 = rospy.Publisher("/iiwa/joint7_position_controller/command", F64, queue_size= 1000)

    def q_callback(self, q_msg):
        self.q[0] = q_msg.position[0]
        self.q[1] = q_msg.position[1]
        self.q[2] = q_msg.position[2]
        self.q[3] = q_msg.position[3]
        self.q[4] = q_msg.position[4]
        self.q[5] = q_msg.position[5]

def main():

    rospy.init_node('dvrk_sub_py')
    rate = rospy.Rate(1000)

    while not rospy.is_shutdown():

        print(kuka_pub.q)

        '''q_c_0 = 0.0
        q_c_1 = 0.0
        q_c_2 = 0.0
        q_c_3 = 0.0
        q_c_4 = 0.0
        q_c_5 = 0.0
        q_c_6 = 0.0'''

        q_c_0 = -0.46
        q_c_1 = 0.33
        q_c_2 = -0.51
        q_c_3 = -1.5
        q_c_4 = 0.16
        q_c_5 = 1.33
        q_c_6 = -0.99

        kuka_pub.f_pub_1.publish(q_c_0)
        kuka_pub.f_pub_2.publish(q_c_1)
        kuka_pub.f_pub_3.publish(q_c_2)
        kuka_pub.f_pub_4.publish(q_c_3)
        kuka_pub.f_pub_5.publish(q_c_4)
        kuka_pub.f_pub_6.publish(q_c_5)
        kuka_pub.f_pub_7.publish(q_c_6)

        rate.sleep()

if __name__ == "__main__":
    kuka_pub = KukaPub()
    main()
