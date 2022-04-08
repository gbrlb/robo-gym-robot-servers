#!/usr/bin/env python3
import rospy
import time
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from queue import Queue
# from time_check import Time_Check

class CmdEffCH:
    def __init__(self):
        # # self.time_check = Time_Check()
        rospy.init_node('cmd_eff_command_handler')
        # self.pendulum_info_pub = rospy.Publisher('/pendulum_info', Vector3, queue_size=10)

        self.get_parameters()
        # Publisher effort command
        self.cmd_eff_pub = rospy.Publisher('/steering_effort_controller/command', Float64MultiArray, queue_size=2)
  
        # Subscriber to Eeffort Command coming from Environment
        # the suscriber must send commands at execution rate
        rospy.Subscriber('/env_cmd_eff', Float64MultiArray, self.callback_env_cmd_eff, queue_size=2)
        self.empty_msg = Float64MultiArray()
        self.empty_msg.data = [0] * 4
        self.effort = []
        # Queue with maximum size 1
        self.queue = Queue(maxsize=1)


    def callback_env_cmd_eff(self, data):
        # self.time_call.print_time(name="\ncallback_env_cmd_eff=================")
        try:
            # Add to the Queue the next command to execute
            self.effort = data
            self.queue.put(data)
            # self.time_call.print_time(name="  queue data OK")
        except:
            # self.time_call.print_time(name="  no_data PASS")
            pass
    

    def cmd_eff_publisher(self):
        self.step = 0
        while not rospy.is_shutdown():
            if self.queue.full():
                data = self.queue.get()
                for _ in range(self.ratio):
                    # print(f"publish:\n{data.data}")
                    self.cmd_eff_pub.publish(data)
                    rospy.sleep(self.control_period)
            else:
                self.cmd_eff_pub.publish(self.empty_msg)

    def get_parameters(self):
        self.limit_upper = rospy.get_param('/limit_upper', +120.)
        self.limit_lower = rospy.get_param('/limit_lower', -120.)

        action_cicle_rate = rospy.get_param('/action_cycle_rate', 25.)
        robot_cicle_rate = rospy.get_param('/robot_actuation_cycle_rate', action_cicle_rate)
        action_generation_time = rospy.get_param('/action_generation_time', 0.01)
        action_cicle_time = 1/action_cicle_rate
        robot_cicle_time = 1/robot_cicle_rate
        sleep_time = action_cicle_time - action_generation_time
        
        self.ratio= int(robot_cicle_rate/action_cicle_rate)
        self.ac_rate = rospy.Rate(action_cicle_rate)
        self.rc_rate = rospy.Rate(robot_cicle_rate)
        self.control_period = rospy.Duration.from_sec(sleep_time)

        # iinit finish msg
        rospy.loginfo(f"============Init Command Handller============")
        rospy.loginfo(f"ac_rate: {action_cicle_rate}, rc_rate: {robot_cicle_rate}, ratio: {self.ratio}")
        rospy.loginfo(f"ac time: {action_cicle_time}, rc_time: {robot_cicle_time}")
        rospy.loginfo(f"sleep_time: {sleep_time}, action_generation_time {action_generation_time}")
            

if __name__ == '__main__':
    try:
        ch = CmdEffCH()
        ch.cmd_eff_publisher()
    except rospy.ROSInterruptException:
        pass