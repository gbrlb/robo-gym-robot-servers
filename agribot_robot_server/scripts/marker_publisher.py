#!/usr/bin/env python3
from sentry_sdk import last_event_id
import rospy
import time
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Point
from queue import Queue
from visualization_msgs.msg import Marker, MarkerArray
# from time_check import Time_Check
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetJointProperties
import copy

class ObservationState:
    def __init__(self):
        # self.time_check = Time_Check()
        rospy.init_node('marker_publisher')

        # publish market
        # rospy.Subscriber('env_thetaref', Float64, self.callback_env_thetaref, queue_size=1)
        self.marker_target_pub = rospy.Publisher('/marker_target', Marker, queue_size=2)
        self.marker_array_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=2)

        rospy.Subscriber('joint_reference', JointState, self.callback_joint_reference, queue_size=2)

        self.limit_upper = rospy.get_param('/limit_upper', +120)
        self.limit_lower = rospy.get_param('/limit_lower', -120)

        self.links = {'front_left_base_link': 1,
	                  'front_right_base_link': -1,
	                  'rear_left_base_link': 1,
	                  'rear_right_base_link': -1 }
           
        self.marker_array = MarkerArray()
        self.marker_array.markers = [Marker()] * 12
        self.env_thetaref = [0] * 4
        self.last_thetaref = 0
        self.angle = 0


    def callback_joint_reference(self, data):
        # data = JointState()
        self.env_thetaref = list(data.position)
        # print(f"last_thetaref:{self.last_thetaref}")
        # print(f"env_thetaref:{self.env_thetaref}")

        if self.env_thetaref != self.last_thetaref:
            # key_target = self.get_marker_key(self.env_thetaref[0])
            # self.marker_target_pub.publish(key_target)

            for idx, link in enumerate(self.links):
                id = idx
                self.marker_array.markers[id] = self.get_marker_key(theta=self.env_thetaref[id], id=id, rgb=[1,1,1],link=link)

            # self.marker_array.markers[0] = self.get_marker_key(self.env_thetaref,1, rgb= [0,0,1])
            # self.marker_array_pub.publish(self.marker_array)
            self.marker_array_pub.publish(self.marker_array)

            self.last_thetaref = self.env_thetaref

    def pub_markers(self):
        # r = rospy.Rate(2)
        # for _ in range(3):
        for idx, link in enumerate(self.links):
            id = 2*idx+4
            self.marker_array.markers[id] = self.get_marker_key(theta=self.limit_upper,id=id, rgb=[0,1,1],link=link)
            self.marker_array.markers[id+1] = self.get_marker_key(theta=self.limit_lower,id=id+1, rgb=[1,0,1],link=link)
        self.marker_array_pub.publish(self.marker_array)
        # r.sleep()

    def get_marker_key(self, theta=0, id=0, rgb= [1, 1, 1],link=None):
        KEY = Marker()
        KEY.type = 0 #arrow
        KEY.action = 0
        KEY.frame_locked = 1
        KEY.pose.position.x = 0
        KEY.pose.position.y = 0.926 * self.links[link]
        KEY.pose.position.z = 0
        KEY.pose.orientation.x = 0
        KEY.pose.orientation.y = 0
        KEY.pose.orientation.z = 0
        KEY.pose.orientation.w = 1
        KEY.scale.x = 0.035 # dia base 
        KEY.scale.y = 0.035 # dia head
        KEY.scale.z = 1 # arrow head leng
        KEY.id = id
        KEY.header.stamp = rospy.Time.now()
        KEY.header.frame_id = link
        KEY.color.a = .9
        KEY.color.r = rgb[0]
        KEY.color.g = rgb[1]
        KEY.color.b = rgb[2]
        x = np.cos(theta)*.5
        y = np.sin(theta)*.5
        z = 0
        tip = Point( x , y , z)
        KEY.points = [Point(0,0,0), tip]
        return KEY
            

if __name__ == '__main__':
    try:
        ch = ObservationState()
        ch.pub_markers()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    # make a visualization marker array for the occupancy grid
