#!/usr/bin/env python

import socket

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sauvc_common.msg import Object 
from sensor_msgs.msg import Image



def dnn_image_callback():
    pass

def gate_callback():
    pass

def udp_bridge():
    rospy.init_node('udp_bridge')
    rospy.loginfo("udp_bridge node started")

    # subscribers init
    image_sub = rospy.Subscriber("/object_detector/image", Image, dnn_image_callback, queue_size=1) #
    gate_sub = rospy.Subscriber("/object_detector/gate", Object, gate_callback, queue_size=1) #

if __name__ == '__main__':
    try:
        udp_bridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down")
    
    # udp setup
    senderIP     = "127.0.0.5"
    senderPort   = 1003
    receiverIP     = "127.0.0.5"
    receiverPort   = 1002
    bufferSize  = 1024
    rospy.loginfo("UDP sender IP:", senderIP)
    rospy.loginfo("UDP sender port:", senderPort)
    rospy.loginfo("UDP receiver IP:", receiverIP)
    rospy.loginfo("UDP receiver port:", receiverPort)
    # Create a datagram socket
    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    # Bind to address and ip
    UDPServerSocket.bind((receiverIP, receiverPort))
    print("UDP server up and listening")

