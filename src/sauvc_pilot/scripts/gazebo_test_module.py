#!/usr/bin/env python

"""Adapted for SAUVC 2020"""

import rospy
import rospkg
import math
import actionlib

import stingray_movement_msgs.msg

from stingray_msgs.srv import SetLagAndMarch

from src.stingray.src.stingray_pilot.scripts.top_level_actions import AUV


def main():
    rospy.init_node("action_test")
    print("Creating client")
    auv = AUV()

    auv.rotate_and_forward(0, 1000, 0.3)

    client = actionlib.SimpleActionClient('stingray_action_dive', stingray_movement_msgs.msg.DiveAction)
    print("Waiting for server")
    client.wait_for_server()
    goal = stingray_movement_msgs.msg.DiveGoal(depth=95)
    print("Sending goal")
    client.send_goal(goal)
    print("Waiting for result")
    client.wait_for_result()
    print("Result received")


    for i in range(11):
        auv.rotate_and_forward(30, 1000, 0.35)
    auv.rotate_and_forward(30)
    auv.rotate_and_forward(-180, 1000, 0.3)
    auv.rotate_and_forward(180)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
