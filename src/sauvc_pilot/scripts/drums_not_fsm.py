#! /usr/bin/env python

import rospy
from src.sauvc_pilot.scripts.auxillary_scripts.sauvc_top_level_actions import SAUVC_AUV


def main():
    rospy.init_node("drums_mission_standalone_node")  # TODO make the sauvc_master_pilot node
    auv = SAUVC_AUV()

    auv.initial_move()

    while auv.detect_mat() != "MAT_DETECTED":
        auv.lag_control()

    auv.front_nav()

    auv.bot_nav()

    auv.drums_nav()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

