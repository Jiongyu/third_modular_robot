#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import rospy

import sys
from rospkg import RosPack
sys.path.append(RosPack().get_path('canopen_communication') + "/modular/")
from modular_G100 import G100

from time import sleep

def main():
    eds_file = RosPack().get_path('canopen_communication') + "/file/Copley.eds"
    I1 = G100(7, eds_file)
    I1.start()
    # I1.opmode_set('PROFILED POSITION')
    # I1.opmode_set('PROFILED VELOCITY')

    # position = 50
    # velocity = -5 
    I1.sent_current(300)
    print I1.get_current()
    print I1.get_current()
    print I1.get_current()
    print I1.get_current()
    print I1.get_current()

    # I1.sent_position(position, velocity)
    # I1.sent_velocity(velocity)
    sleep(1)
    I1.sent_current(0)

    I1.stop()
    # I1.quick_stop()


if __name__ == "__main__":
    main()     

