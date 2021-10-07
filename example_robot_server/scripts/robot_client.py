#!/usr/bin/env python
#example
import grpc
import rospy
from concurrent import futures
# import the publisher and subscriber node
from ros_bridge import RosBridge
# import the gRPC generated classes
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2, robot_server_pb2_grpc

class Client():

    def __init__(self, server_port):
        self.channel = grpc.insecure_channel('[::]:' + repr(server_port))
        self.robot_server_stub = robot_server_pb2_grpc.RobotServerStub(self.channel)

    def set_state(self, state):
        # Old method, to be gradually replaced in all the stack
        msg = self.robot_server_stub.SetState(robot_server_pb2.State(state = state), timeout = 60)
        return msg.success

    def set_state_msg(self, state_msg):
        msg = self.robot_server_stub.SetState(state_msg, timeout = 60)
        return msg.success

    def get_state(self,):
        # Old method, to be gradually replaced in all the stack
        msg = self.robot_server_stub.GetState(robot_server_pb2.Empty(), timeout = 20)
        if msg.success == 1:
            return msg.state
        else:
            raise Exception('Error while retrieving state')

    def get_state_msg(self,):
        msg = self.robot_server_stub.GetState(robot_server_pb2.Empty(), timeout = 20)
        if msg.success == 1:
            return msg
        else:
            raise Exception('Error while retrieving state')

    def send_action(self, action):
        msg = self.robot_server_stub.SendAction(robot_server_pb2.Action(action = action), timeout = 20 )
        return msg.success

    def send_action_get_state(self, action):
        msg = self.robot_server_stub.SendActionGetState(robot_server_pb2.Action(action = action), timeout = 20 )
        if msg.success == 1:
            return msg
        else:
            raise Exception('Error while retrieving state')

rospy.loginfo('Starting Example Robot Server...')
server_port = 50051
client = Client(server_port)
state = client.get_state()
action = client.send_action([2, -5])
if action == True:
    print("action send")
action_state = client.send_action_get_state([1, 0])
get_state_msg =client.get_state_msg()
print("state: ", state)
print("\n*****State Message:*******\n")
print(get_state_msg)
print("here")
# if __name__ == '__main__':

#     try:
#         wait_time = 5
#         rospy.init_node('robot_server')
#         rospy.loginfo('Waiting {}s before starting initialization of Robot Server'.format(wait_time))
#         rospy.sleep(wait_time)
#         serve()
#     except (KeyboardInterrupt, SystemExit):
#         pass
