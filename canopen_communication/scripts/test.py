#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import rospy

import sys
from rospkg import RosPack
sys.path.append(RosPack().get_path('canopen_communication') + "/modular/")
from modular_T85 import T85

from time import sleep

def main():
    eds_file = RosPack().get_path('canopen_communication') + "/file/Copley.eds"
    joint = T85(5, eds_file)
    joint.start()
    # joint.opmode_set('PROFILED POSITION')
    joint.opmode_set('PROFILED VELOCITY')

    # position = 50
    velocity = -5 
    # joint.sent_current(300)
    # print joint.get_current()
    # print joint.get_current()
    # print joint.get_current()
    # print joint.get_current()
    # print joint.get_current()

    # joint.sent_position(position, velocity)
    joint.sent_velocity(velocity)
    sleep(2)
    print "pause run"
    joint.pause_run()
    sleep(2)
    print "continue run"
    joint.continue_run()
    sleep(2)
    # sleep(1)
    # joint.sent_current(0)

    joint.stop()
    joint.stop_communication()
    # joint.quick_stop()


if __name__ == "__main__":
    main()     

