from src.stingray.src.stingray_pilot.scripts.top_level_actions import AUV
import smach
import smach_ros

import src.sauvc_pilot.scripts.auxillary_scripts.drums_navigation as drums_navigation
import src.sauvc_pilot.scripts.auxillary_scripts.mat_front_cam_navigation as mat_front_cam_navigation
from src.sauvc_common.msg import OptionalPoint2D


class SAUVC_AUV(AUV):
    def initial_move(self):
        pass

    def detect_mat(self):
        pass

    def fail(self):
        pass

    def front_nav(self):
        pass

    def bot_nav(self):
        pass

    def drums_nav(self):
        pass

    def lag_control(self):
        pass

    def right(self):
        pass

    def left(self):
        pass

