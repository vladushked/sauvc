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
bufferSize  = 24

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
Y = 0.5 # X
Roll = 1.5708
Pitch = 0.0
VelosityY = 0.0
VelosityRoll = 0.0
VelosityPitch = 0.0

# to simulator
state_msg = ModelState()
state_msg.model_name = 'ROV_model_URDF'

def set_model_state(Z, X, Yaw, VelosityZ, VelosityX, VelosityYaw):
    qx, qy, qz, qw = euler_to_quaternion(Roll, Pitch, Yaw)
    state_msg.pose.position.x = X
    state_msg.pose.position.y = Y
    state_msg.pose.position.z = Z
    state_msg.pose.orientation.x = qx
    state_msg.pose.orientation.y = qy
    state_msg.pose.orientation.z = qz
    state_msg.pose.orientation.w = qw
    state_msg.twist.linear.x = VelosityX
    state_msg.twist.linear.y = VelosityY
    state_msg.twist.linear.z = VelosityZ
    state_msg.twist.angular.x = VelosityRoll
    state_msg.twist.angular.y = VelosityPitch
    state_msg.twist.angular.z = VelosityYaw
    rospy.loginfo("set_model_state")
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def euler_to_quaternion(Roll, Pitch, Yaw):
    qx = np.sin(Roll/2) * np.cos(Pitch/2) * np.cos(Yaw/2) - np.cos(Roll/2) * np.sin(Pitch/2) * np.sin(Yaw/2)
    qy = np.cos(Roll/2) * np.sin(Pitch/2) * np.cos(Yaw/2) + np.sin(Roll/2) * np.cos(Pitch/2) * np.sin(Yaw/2)
    qz = np.cos(Roll/2) * np.cos(Pitch/2) * np.sin(Yaw/2) - np.sin(Roll/2) * np.sin(Pitch/2) * np.cos(Yaw/2)
    qw = np.cos(Roll/2) * np.cos(Pitch/2) * np.cos(Yaw/2) + np.sin(Roll/2) * np.sin(Pitch/2) * np.sin(Yaw/2)

    return (qx, qy, qz, qw)

def udp_send(is_exist, x_start, y_start, x_end, y_end, x_center, y_center):
    # make message
    messageToQt = struct.pack("?ffffff", is_exist, x_start, y_start, x_end, y_end, x_center, y_center)
    # send to qt
    RosSenderUDPSocket.sendto(messageToQt, (qtReceiverIP, qtReceiverPort))

def udp_receive():
    # rospy.loginfo("udp_receive")
    data, addr = RosReceiverUDPSocket.recvfrom(1024)
    receivedZ, receivedX, receivedYaw, receivedVelosityZ, receivedVelosityX, receivedVelosityYaw = struct.unpack("ffffff", data)
    print(receivedZ, receivedX, receivedYaw, receivedVelosityZ, receivedVelosityX, receivedVelosityYaw)
    #set_model_state(receivedZ, receivedX, receivedYaw, receivedVelosityZ, receivedVelosityX, receivedVelosityYaw)
    # timer = rospy.Timer(rospy.Duration(0.1), udp_receive(), oneshot=True)


def gate_callback(msg):
    # udp_receive()
    udp_send(msg.is_exist, msg.x_start, msg.y_start, msg.x_end, msg.y_end, msg.x_center, msg.y_center)

def init_udp_bridge():
    # init node
    rospy.init_node('udp_bridge')
    rospy.loginfo("udp_bridge node started")
    # Bind to address and ip
    RosSenderUDPSocket.bind((rosSenderIP, rosSenderPort))
    RosReceiverUDPSocket.bind((rosReceiverIP, rosReceiverPort))
    rospy.loginfo("UDP up")
    # subscribers init
    gate_sub = rospy.Subscriber("/object_detector/gate", Object, gate_callback, queue_size=1) 
    
    r = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        udp_receive()
        r.sleep()
    

if __name__ == '__main__':
    try:
        init_udp_bridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down UDP bridge")