#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from PyQt5.QtCore import pyqtSignal,QThread

from birl_module_robot.msg import pole_position

def main():
    rospy.init_node('test_pub_pole_position_msg', anonymous=True)
    pub = rospy.Publisher('/pole_position', pole_position , queue_size=10)
    msg = pole_position()
    p1 = [0, 50, 50]
    p2 = [0, -50, 50]
    msg.p1 = p1
    msg.p2 = p2

    rate = rospy.Rate(1) 
    index = 0
    
    while index < 10:
        msg.timeHeader.stamp = rospy.Time.now()

        pub.publish(msg)

        rate.sleep()
        index += 1

if __name__ == "__main__":
    main()