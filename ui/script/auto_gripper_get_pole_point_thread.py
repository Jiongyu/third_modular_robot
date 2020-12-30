#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: 
@file: auto_gripper_get_pole_point_thread.py

@biref: 接收ros topic (/low_level/joints_point_command) 杆件端点
"""
import rospy

from PyQt5.QtCore import pyqtSignal,QThread

# from birl_module_robot.msg import pole_position
from std_msgs.msg import Float64MultiArray

class Auto_gripper_get_pole_point_thread(QThread):

    # 发送控制命令信号
    sin_pole_point = pyqtSignal(list)

    def __init__(self):
        super(Auto_gripper_get_pole_point_thread, self).__init__()

        # ros初始化
        # rospy.logwarn("Auto_gripper_get_pole_point_thread")
        # rospy.Subscriber('/pole_position', pole_position, self.__callback)
        rospy.Subscriber('/detection/polemodel', Float64MultiArray, self.__callback)

    # ros 回调函数
    def __callback(self, pole_point):
        # self.sin_pole_point.emit([pole_point.p1, pole_point.p2])
        p1 = pole_point.data[0:3]
        p2 = pole_point.data[3:6]
        self.sin_pole_point.emit([p1, p2])

    def run(self):
        rospy.spin()
        pass


    
    


