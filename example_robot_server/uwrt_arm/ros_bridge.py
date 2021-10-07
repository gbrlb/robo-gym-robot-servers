#!/usr/bin/env python2
#uwrt_arm
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist, Pose, Pose2D
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState, GetLinkState
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Float64MultiArray, Header
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import PyKDL
import copy
from threading import Event
import time
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2

class UWRTRosBridge:

    def __init__(self,  real_robot=False, uwrt_model = 'uwrt_arm'):

        # Event is clear while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()

        self.real_robot = real_robot

        # joint_trajectory_command_handler publisher
        self.arm_cmd_pub = rospy.Publisher('env_arm_command', Float64MultiArray, queue_size=1)

        # Target RViz Marker publisher
        self.target_key_pub = rospy.Publisher('target_key', Marker, queue_size=10)

        # TF Listener
        self.tf_listener = tf.TransformListener()

        # Joint States
        self.num_joints = 5
        rospy.Subscriber("joint_states", JointState, self.UWRTJointStateCallback)

        self.observation = {
            'key': {
                # TODO: setting fake key location
                "position": [np.random.uniform(0.625, 0.675),
                             np.random.uniform(-0.30, 0.30),
                             np.random.uniform(0.65, 0.675)],
                "orientation": [0, 0, 0, 1],
                "distance_to_target": [0] * 1,
                "orientation_difference": [0] * 4,
            },
            'uwrt_arm': {
                "position": [0] * self.num_joints,
                "velocity": [0] * self.num_joints,
                "effort": [0] * self.num_joints,
                "joint_limit_switches": [0] * self.num_joints,
            },
        }

        # Robot control rate
        self.sleep_time = (1.0/rospy.get_param("~action_cycle_rate")) - 0.01
        self.control_period = rospy.Duration.from_sec(self.sleep_time)

        self.reference_frame = 'world'
        self.ee_frame = 'arm_wrist_link'

    def UWRTJointStateCallback(self, data):
        if self.get_state_event.is_set():
            self.observation['uwrt_arm']['position'] = data.position
            self.observation['uwrt_arm']['velocity'] = data.velocity
            self.observation['uwrt_arm']['effort'] = data.effort

            # TODO: calculate the difference in distance and orientation
            # self.observation['key']['distance_to_target'] = [0]
            # self.observation['key']['orientation_difference'] = [0] * 4

    def get_state(self):
        self.get_state_event.clear()
        ###
        self.get_state_event.set()

        msg = robot_server_pb2.State()
        # TODO: more efficient method to flatten dictionary
        msg.state.extend(self.observation['key']['position'])
        msg.state.extend(self.observation['key']['orientation'])
        msg.state.extend(self.observation['key']['distance_to_target'])
        msg.state.extend(self.observation['key']['orientation_difference'])
        msg.state.extend(self.observation['uwrt_arm']['position'])
        msg.state.extend(self.observation['uwrt_arm']['velocity'])
        msg.state.extend(self.observation['uwrt_arm']['effort'])
        msg.state.extend(self.observation['uwrt_arm']['joint_limit_switches'])

        msg.success = 1

        return msg

    def set_state(self, state_msg):
        # Set environment state
        state = state_msg.state

        # Clear reset Event
        self.reset.clear()

        # Publish Target Marker
        self.publish_target_key()

        # UR Joints Positions
        reset_steps = int(15.0/self.sleep_time)
        for i in range(reset_steps):
            self.publish_env_arm_cmd(state)

        self.reset.set()

        return 1

    def publish_env_arm_cmd(self, vel_cmd):
        msg = Float64MultiArray()
        msg.data = vel_cmd
        self.arm_cmd_pub.publish(msg)
        rospy.sleep(self.control_period)

    def publish_target_key(self):
        KEY = Marker()
        KEY.type = 1 # CUBE
        KEY.action = 0
        KEY.frame_locked = 1
        KEY.pose.position.x = self.observation['key']['position'][0]
        KEY.pose.position.y = self.observation['key']['position'][1]
        KEY.pose.position.z = self.observation['key']['position'][2]
        KEY.pose.orientation.x = self.observation['key']['orientation'][0]
        KEY.pose.orientation.y = self.observation['key']['orientation'][1]
        KEY.pose.orientation.z = self.observation['key']['orientation'][2]
        KEY.pose.orientation.w = self.observation['key']['orientation'][3]
        KEY.scale.x = 0.1
        KEY.scale.y = 0.1
        KEY.scale.z = 0.1
        KEY.id = 0
        KEY.header.stamp = rospy.Time.now()
        KEY.header.frame_id = self.reference_frame
        KEY.color.a = 0.7
        KEY.color.r = 1.0  # red
        KEY.color.g = 0.0
        KEY.color.b = 0.0
        self.target_key_pub.publish(KEY)
