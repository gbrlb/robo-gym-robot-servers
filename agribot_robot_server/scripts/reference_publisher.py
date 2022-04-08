#!/usr/bin/env python3
import rospy
import time
import copy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Point
from queue import Queue
from visualization_msgs.msg import Marker
# from time_check import Time_Check
import copy

class ReferenceState:
    def __init__(self):
        # self.time_check = Time_Check()
        rospy.init_node('reference_publisher')
        self.reference_state_pub = rospy.Publisher('joint_reference', JointState, queue_size=1)

        rospy.Subscriber('/env_cmd', JointState, self.callback_env_cmd, queue_size=2)

        self.reference = JointState()
        self.reference.name = [ 'front_left_steering_joint', 
                                'front_right_steering_joint', 
                                'rear_left_steering_joint', 
                                'rear_right_steering_joint']
        self.reference.position = [0] * 4
        self.reference.velocity = [0] * 4
        self.reference.effort = [0] * 4
        self.last_reference = JointState()


        ac_rate = rospy.get_param('/action_cycle_rate', 25)
        rc_rate = rospy.get_param('/robot_actuation_cycle_rate', ac_rate)
        self.rate = rospy.Rate(rc_rate*5)

        rospy.loginfo(f"============Reference Publisher============")
        rospy.sleep(1)

        
    def callback_env_cmd(self, data):
        self.reference.position = data.position
        self.reference.velocity = data.velocity
        self.reference.effort = data.effort

        if self.last_reference != self.reference:
            # print(f"publish:{data.position}")
            self.reference_state_pub.publish(self.reference)
            self.last_reference = copy.deepcopy(self.reference)

    # def reference_state_publisher(self):
    #     while not rospy.is_shutdown():
    #         try:
    #             pass
    #             # self.reference_state_pub.publish(self.reference)
    #         except rospy.ServiceException as e:
    #             print('Error:' + e)
    #         self.rate.sleep()

if __name__ == '__main__':
    try:
        ch = ReferenceState()
        rospy.spin()
        # ch.reference_state_publisher()
    except rospy.ROSInterruptException:
        pass