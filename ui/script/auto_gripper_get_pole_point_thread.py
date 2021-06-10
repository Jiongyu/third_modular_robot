#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: 
@file: auto_gripper_get_pole_point_thread.py

@biref: 接收ros topic (/detection/polemodel) 杆件端点
"""
import rospy

from PyQt5.QtCore import pyqtSignal,QThread

from birl_module_robot.srv import detection_results
from std_msgs.msg import Float64MultiArray

class Auto_gripper_get_pole_point_thread(QThread):

    # 发送控制命令信号
    sin_pole_point = pyqtSignal(list)

    def __init__(self, base):
        self.__base = base
        super(Auto_gripper_get_pole_point_thread, self).__init__()

        # ros初始化
        # rospy.logwarn("Auto_gripper_get_pole_point_thread")
        # rospy.Subscriber('/detection/polemodel', Float64MultiArray, self.__callback)

    # # ros 回调函数
    # def __callback(self, pole_point):
    #     # self.sin_pole_point.emit([pole_point.p1, pole_point.p2])
    #     if len(pole_point.data) >= 6:
    #         p1 = pole_point.data[0:3]
    #         p2 = pole_point.data[3:6]
    #         self.sin_pole_point.emit([p1, p2])
    #     else:
    #         rospy.logwarn("the list length of pole point < 6.")

    def run(self):

        try:
            rospy.wait_for_service("pose_estimation_server", timeout=3)

        except rospy.ROSException:
            rospy.loginfo("pose_estimation_server timeout.")
            return

        try:
            client = rospy.ServiceProxy("pose_estimation_server", detection_results)
            resp = client.call(self.__base)

        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s"%e)
            return

        # 发送第一点
        if len(resp.detection_results.data) >= 6:
            p1 = resp.detection_results.data[0:3]
            p2 = resp.detection_results.data[3:6]
            self.sin_pole_point.emit([p1, p2])
        else:
            rospy.logwarn("the list length of pole point < 6.")
        pass


    
    


