#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: 
@file: receive_ros_command_thread.py

@biref: 接收ros topic (/low_level/joints_point_command) 控制命令
"""
import rospy
from ui.msg import robot_command

from PyQt5.QtCore import pyqtSignal,QThread

class Receive_ros_command_thread(QThread):

    # 发送控制命令信号
    sin_sent_ros_command_thread = pyqtSignal(list)

    def __init__(self):
        super(Receive_ros_command_thread, self).__init__()

        # 存储ros控制命令数据[0:5, 位置; 5:10, 速度]
        self.__ros_pos_vel_command = []
        # ros初始化
        rospy.Subscriber('/low_level/joints_point_command', robot_command, self.__callback)

    # ros 回调函数
    def __callback(self,robot_command):
        self.__ros_pos_vel_command = list(robot_command.CommandPosData)
        self.__ros_pos_vel_command.extend(robot_command.CommandVelData) 
        self.sin_sent_ros_command_thread.emit(self.__ros_pos_vel_command)

    def run(self):
        rospy.spin()
        pass


    
    


