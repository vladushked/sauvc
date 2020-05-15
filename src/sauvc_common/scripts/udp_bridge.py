#!/usr/bin/env python

import socket
import struct
import rospy
from sauvc_common.msg import Object 

# udp setup
rosSenderIP = "127.0.0.5"
rosSenderPort = 1004
rosReceiverIP = "127.0.0.5"
rosReceiverPort = 1005
bufferSize  = 1024
# Create a datagram socket
RosSenderUDPSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
RosReceiverUDPSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# to Qt
is_exist = False
x_start = 0
y_start = 0
x_end = 0
y_end = 0
x_center = 0
y_center = 0
# from qt
linearVelosityX = 0
linearVelosityY = 0
linearVelosityZ = 0
angularVelosityX = 0
angularVelosityY = 0
angularVelosityZ = 0


def gate_callback(msg):
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
        x_start = 0
        y_start = 0
        x_end = 0
        y_end = 0
        x_center = 0
        y_center = 0
    # make message
    messageToQt = struct.pack("?ffffff", is_exist, x_start, y_start, x_end, y_end, x_center, y_center)
    # send to qt
    RosSenderUDPSocket.send(messageToQt)

def udp_bridge():
    # init node
    rospy.init_node('udp_bridge')
    rospy.loginfo("udp_bridge node started")
    # subscribers init
    gate_sub = rospy.Subscriber("/object_detector/gate", Object, gate_callback, queue_size=1) #

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

    # Listen for incoming datagrams
    while not rospy.is_shutdown():

        data, addr = RosReceiverUDPSocket.recvfrom(bufferSize)
        linearVelosityX, linearVelosityY, linearVelosityZ, angularVelosityX, angularVelosityY, angularVelosityZ = struct.unpack("ffffff", data)
        rospy.loginfo(linearVelosityX, linearVelosityY, linearVelosityZ, angularVelosityX, angularVelosityY, angularVelosityZ)
        