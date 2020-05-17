#!/usr/bin/env python

from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import socket
import struct
import numpy as np
import rospy
from sauvc_common.msg import Object 

# udp setup
qtReceiverIP = "127.0.0.5"
qtReceiverPort = 1033
rosSenderIP = "127.0.0.5"
rosSenderPort = 1034
rosReceiverIP = "127.0.0.5"
rosReceiverPort = 1035
bufferSize  = 1024

# Create a datagram socket
RosSenderUDPSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
RosReceiverUDPSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# to Qt
is_exist = False
x_start = 0.0
y_start = 0.0
x_end = 0.0
y_end = 0.0
x_center = 0.0
y_center = 0.0

# from qt
linearPoseX = 1.0
linearPoseY = 0.5
linearPoseZ = 2.0
angularPoseRoll = 1.5708
angularPosePitch = 0.0
angularPoseYaw = 1.5708
linearVelosityX = 0.0
linearVelosityY = 0.0
linearVelosityZ = 0.0
angularVelosityX = 0.0
angularVelosityY = 0.0
angularVelosityZ = 0.0

# to simulator
state_msg = ModelState()
state_msg.model_name = 'ROV_model_URDF'
state_msg.pose.position.x = 1.0
state_msg.pose.position.y = 0.5
state_msg.pose.position.z = 2.0
state_msg.pose.orientation.x = 0.0
state_msg.pose.orientation.y = 0.0
state_msg.pose.orientation.z = 0.0
state_msg.pose.orientation.w = 0.0
state_msg.twist.linear.x = 0.0
state_msg.twist.linear.y = 0.0
state_msg.twist.linear.z = 0.0
state_msg.twist.angular.x = 0.0
state_msg.twist.angular.y = 0.0
state_msg.twist.angular.z = 0.0

def gate_callback(msg):
    rospy.loginfo("Udp callback")
    #udp_receive()
    euler_to_quaternion()
    set_model_service()

    if msg.is_exist:
        is_exist = msg.is_exist
        x_start = msg.x_start
        y_start = msg.y_start
        x_end = msg.x_end
        y_end = msg.y_end
        x_center = msg.x_center
        y_center = msg.y_center
    else:
        is_exist = False
        x_start = 0.0
        y_start = 0.0
        x_end = 0.0
        y_end = 0.0
        x_center = 0.0
        y_center = 0.0
    # make message
    messageToQt = struct.pack("?ffffff", is_exist, x_start, y_start, x_end, y_end, x_center, y_center)
    # send to qt
    RosSenderUDPSocket.sendto(messageToQt, (qtReceiverIP, qtReceiverPort))

def udp_receive():
    rospy.loginfo("udp_receive")
    data, addr = RosReceiverUDPSocket.recvfrom(bufferSize)
    state_msg.twist.linear.x, 
    state_msg.twist.linear.y, 
    state_msg.twist.linear.z, 
    state_msg.twist.angular.x, 
    state_msg.twist.angular.y, 
    state_msg.twist.angular.z = struct.unpack("ffffff", data)

def set_model_service():
    rospy.loginfo("set_model_service")

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def euler_to_quaternion():
    state_msg.pose.orientation.x = np.sin(angularPoseRoll/2) * np.cos(angularPosePitch/2) * np.cos(angularPoseYaw/2) - np.cos(angularPoseRoll/2) * np.sin(angularPosePitch/2) * np.sin(angularPoseYaw/2)
    state_msg.pose.orientation.y = np.cos(angularPoseRoll/2) * np.sin(angularPosePitch/2) * np.cos(angularPoseYaw/2) + np.sin(angularPoseRoll/2) * np.cos(angularPosePitch/2) * np.sin(angularPoseYaw/2)
    state_msg.pose.orientation.z = np.cos(angularPoseRoll/2) * np.cos(angularPosePitch/2) * np.sin(angularPoseYaw/2) - np.sin(angularPoseRoll/2) * np.sin(angularPosePitch/2) * np.cos(angularPoseYaw/2)
    state_msg.pose.orientation.w = np.cos(angularPoseRoll/2) * np.cos(angularPosePitch/2) * np.cos(angularPoseYaw/2) + np.sin(angularPoseRoll/2) * np.sin(angularPosePitch/2) * np.sin(angularPoseYaw/2)

def udp_bridge():
    # init node
    rospy.init_node('udp_bridge')
    rospy.loginfo("udp_bridge node started")
    # subscribers init
    gate_sub = rospy.Subscriber("/object_detector/gate", Object, gate_callback, queue_size=1) 
    # init position


if __name__ == '__main__':
    # Bind to address and ip
    RosSenderUDPSocket.bind((rosSenderIP, rosSenderPort))
    RosReceiverUDPSocket.bind((rosReceiverIP, rosReceiverPort))
    rospy.loginfo("UDP up")

    try:
        udp_bridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down")