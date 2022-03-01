#!/usr/bin/env python3
#agribot
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose, Vector3, Point
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState, ModelState
from gazebo_msgs.srv import SetModelState, SetLinkState
import PyKDL
import copy
from threading import Event
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2

class RosBridge:

    def __init__(self, real_robot=False):

        # Event is clear while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()

        self.real_robot = real_robot

        # get parameters:
        self.get_parameters()

        self.links = {'front_left_steering_link': [1, 1],
	                  'front_right_steering_link': [1, -1],
	                  'rear_left_steering_link': [-1, 1],
	                  'rear_right_steering_link': [-1, -1] }

        # Joint states
        self.joint_names = [ 'front_left_steering_joint', 'front_right_steering_joint', 'rear_left_steering_joint', 'rear_right_steering_joint', 'front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']
        self.joint_position = dict.fromkeys(self.joint_names, 0.0)
        self.joint_velocity = dict.fromkeys(self.joint_names, 0.0)
        self.joint_effort = dict.fromkeys(self.joint_names, 0.0)

        rospy.Subscriber('joint_states', JointState, self._on_joint_states, queue_size=1)
        self.joint_steering_pub = rospy.Publisher('joint_steering', JointState, queue_size=1)


        # Publisher to Command Handler
        self.env_cmd_eff = rospy.Publisher('env_cmd_eff', Float64MultiArray, queue_size=1)
        self.env_thetadot = rospy.Publisher('env_thetadot', Float64MultiArray, queue_size=1)
        self.env_thetaref = rospy.Publisher('env_thetaref', Float64MultiArray, queue_size=1)

        self.effor_msg = Float64MultiArray()
        self.effor_msg.data = [0] * 4
        self.effor_msg.layout.data_offset = 0
        self.effor_msg.layout.dim.append(MultiArrayDimension("front_left_steering_joint", 1, 4))
        self.effor_msg.layout.dim.append(MultiArrayDimension("front_right_steering_joint", 1, 1))
        self.effor_msg.layout.dim.append(MultiArrayDimension("rear_left_steering_joint", 1, 1))
        self.effor_msg.layout.dim.append(MultiArrayDimension("rear_right_steering_joint", 1, 1))

        # # Robot State
        # self.agribot_position = [0.0] * 2
        # self.agribot_twist = [0.0] * 2

        # rospy.Subscriber('robot_pose', Pose, self.callbackState, queue_size=1)

        self.rate = rospy.Rate(10)  # 30Hz
        self.reset.set()

    def get_parameters(self):
        self.limit_upper = rospy.get_param('/limit_upper', +120.)
        self.limit_lower = rospy.get_param('/limit_lower', -120.)
        rospy.loginfo(f"============parameters============")
        rospy.loginfo(f"{self.limit_upper}")
        rospy.loginfo(f"{self.limit_lower}")

        action_cicle_rate = rospy.get_param('/action_cycle_rate', 25.)
        robot_cicle_rate = rospy.get_param('/robot_actuation_cycle_rate', action_cicle_rate)
        action_generation_time = rospy.get_param('/action_generation_time', 0.01)
        action_cicle_time = 1/action_cicle_rate
        robot_cicle_time = 1/robot_cicle_rate
        sleep_time = action_cicle_time - action_generation_time
        ratio= int(robot_cicle_rate/action_cicle_rate)

        self.ac_rate = rospy.Rate(action_cicle_rate)
        self.rc_rate = rospy.Rate(robot_cicle_rate)
        self.control_period = rospy.Duration.from_sec(sleep_time)

        # iinit finish msg
        rospy.loginfo(f"============RosBridge============")
        rospy.loginfo(f"ac_rate: {action_cicle_rate}, rc_rate: {robot_cicle_rate}, ratio: {ratio}")
        rospy.loginfo(f"ac time: {action_cicle_time}, rc_time: {robot_cicle_time}")
        rospy.loginfo(f"sleep_time: {sleep_time}, action_generation_time {action_generation_time}")


    def get_state(self):
        state = []
        state_dict = {}
        self.get_state_event.clear()
        # Joint Positions and Joint Velocities
        joint_position = copy.deepcopy(self.joint_position)
        joint_velocity = copy.deepcopy(self.joint_velocity)
        state += self._get_joint_ordered_value_list(joint_position)
        state += self._get_joint_ordered_value_list(joint_velocity)
        state_dict.update(self._get_joint_states_dict(joint_position, joint_velocity))
        print(state_dict)
        self.get_state_event.set()

        # Create and fill State message
        msg = robot_server_pb2.State(state=state, state_dict=state_dict, success= True)
        
        return msg

    def set_state(self, state_msg):

        self.reset.clear()

        # self.env_thetaref.publish(thetaref_msg)

        if not self.real_robot:
            # Set Gazebo Robot Model state
            for _, link in enumerate(state_msg.state_dict):
                if link in self.links:
                    print(link, state_msg.state_dict[link], self.links[link])
                    self.set_link_state(link, state_msg.state_dict[link], self.links[link] )
              
        self.reset.set()

        # Sleep time set manually to allow gazebo to reposition model
        rospy.sleep(0.2)

        return 1

    def publish_env_eff_cmd(self, effort):
        self.effor_msg.data.clear()
        # effort = np.array([])
        # effort.reshape
        self.effor_msg.data = effort
        # print(msg)
        self.env_cmd_eff.publish(self.effor_msg)
        # Sleep time set manually to achieve approximately 10Hz rate
        # self.time_check.print_time("RB: send action")
        rospy.sleep(self.control_period)
        # self.time_check.print_time(f"RB: sleep control period: {self.sleep_time}")
        return effort

    def get_robot_state(self):
        # method to get robot position from real mir
        return self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta, self.robot_twist.linear.x, self.robot_twist.linear.y, self.robot_twist.angular.z

    def set_link_state(self, link_name, state, pos_ref):
        # Set Gazebo Model State
        rospy.wait_for_service('/gazebo/set_link_state')

        start_state = LinkState()
        start_state.link_name = f"agribot::{link_name}"
        start_state.reference_frame = "base_footprint"
        start_state.pose.position.x = 1.5*pos_ref[0]
        start_state.pose.position.y = 1.2*pos_ref[1]
        start_state.pose.position.z = 1.925
        orientation = PyKDL.Rotation.RPY(0, 0, state)
        start_state.pose.orientation.x, start_state.pose.orientation.y, start_state.pose.orientation.z, start_state.pose.orientation.w = orientation.GetQuaternion()

        start_state.twist.linear.x = 0.0
        start_state.twist.linear.y = 0.0
        start_state.twist.linear.z = 0.0
        start_state.twist.angular.x = 0.0
        start_state.twist.angular.y = 0.0
        start_state.twist.angular.z = 0.0

        # print(start_state)
        try:
            set_link_state_client = rospy.ServiceProxy('/gazebo/set_link_state/', SetLinkState)
            set_link_state_client(start_state)
        except rospy.ServiceException as e:
            print("Service call failed:" + e)


    def set_model_state(self, model_name, state):
        # Set Gazebo Model State
        rospy.wait_for_service('/gazebo/set_model_state')

        start_state = ModelState()
        start_state.model_name = model_name
        start_state.pose.position.x = state[0]
        start_state.pose.position.y = state[1]
        orientation = PyKDL.Rotation.RPY(0,0, state[2])
        start_state.pose.orientation.x, start_state.pose.orientation.y, start_state.pose.orientation.z, start_state.pose.orientation.w = orientation.GetQuaternion()

        start_state.twist.linear.x = 0.0
        start_state.twist.linear.y = 0.0
        start_state.twist.linear.z = 0.0
        start_state.twist.angular.x = 0.0
        start_state.twist.angular.y = 0.0
        start_state.twist.angular.z = 0.0

        try:
            set_model_state_client = rospy.ServiceProxy('/gazebo/set_model_state/', SetModelState)
            set_model_state_client(start_state)
        except rospy.ServiceException as e:
            print("Service call failed:" + e)

    def callbackState(self,data):
        # If state is not being reset proceed otherwise skip callback
        if self.reset.isSet():
            x = data.position.x
            y = data.position.y

            # Update internal Position variable
            self.agribot_position = copy.deepcopy([x, y])
        else:
            pass

    def _on_joint_states(self, msg):
        if self.get_state_event.is_set():
            for idx, name in enumerate(msg.name):
                if name in self.joint_names:
                    self.joint_position[name] = msg.position[idx]
                    self.joint_velocity[name] = msg.velocity[idx]
                    self.joint_effort[name] = msg.effort[idx]
        joint_steering_msg = JointState()
        joint_steering_msg.name = self.joint_names
        joint_steering_msg.position = self._get_joint_ordered_value_list(copy.deepcopy(self.joint_position))
        self.joint_steering_pub.publish(joint_steering_msg)

    def _get_joint_ordered_value_list(self, joint_values):    
        return [joint_values[name] for name in self.joint_names]

    def _get_joint_states_dict(self, joint_position, joint_velocity):
        d = {}
        d['front_left_steering_joint_position'] = joint_position['front_left_steering_joint']
        d['front_right_steering_joint_position'] = joint_position['front_right_steering_joint']
        d['rear_left_steering_joint_position'] = joint_position['rear_left_steering_joint']
        d['rear_right_steering_joint_position'] = joint_position['rear_right_steering_joint']
        d['front_left_wheel_joint_position'] = joint_position['front_left_wheel_joint']
        d['front_right_wheel_joint_position'] = joint_position['front_right_wheel_joint']
        d['rear_left_wheel_joint_position'] = joint_position['rear_left_wheel_joint']
        d['rear_right_wheel_joint_position'] = joint_position['rear_right_wheel_joint']
        d['front_left_steering_joint_velocity'] = joint_velocity['front_left_steering_joint']
        d['front_right_steering_joint_velocity'] = joint_velocity['front_right_steering_joint']
        d['rear_left_steering_joint_velocity'] = joint_velocity['rear_left_steering_joint']
        d['rear_right_steering_joint_velocity'] = joint_velocity['rear_right_steering_joint']
        d['front_left_wheel_joint_velocity'] = joint_velocity['front_left_wheel_joint']
        d['front_right_wheel_joint_velocity'] = joint_velocity['front_right_wheel_joint']
        d['rear_left_wheel_joint_velocity'] = joint_velocity['rear_left_wheel_joint']
        d['rear_right_wheel_joint_velocity'] = joint_velocity['rear_right_wheel_joint']
        return d 