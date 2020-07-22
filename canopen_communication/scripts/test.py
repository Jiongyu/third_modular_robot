#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import rospy

import sys
from rospkg import RosPack
sys.path.append(RosPack().get_path('canopen_communication') + "/modular/")
from modular_I100 import I100

from time import sleep

def main():
    eds_file = RosPack().get_path('canopen_communication') + "/file/Copley.eds"
    I1 = I100(1, eds_file)
    I1.start()
    # I1.opmode_set('PROFILED POSITION')
    I1.opmode_set('PROFILED VELOCITY')

    position = 100
    velocity = -5

    # I1.sent_position(position, velocity)
    I1.sent_velocity(velocity)
    sleep(10)

    I1.stop()


if __name__ == "__main__":
    main()     

