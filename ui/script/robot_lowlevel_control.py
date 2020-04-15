#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:Jony
@contact: 35024339@qq.com
@software: RoboWareStudio
@file: robot_lowlevel_control.py

@biref: 上位机底层控制 canopen
"""

import rospy
from std_msgs.msg import Float64MultiArray

import sys
from rospkg import RosPack
sys.path.append(RosPack().get_path('canopen_communication') + "/modular/")

from modular_G100 import G100
from modular_I100 import I100
from modular_T100 import T100

from PyQt5.QtCore import QThread,pyqtSignal,QMutex

from time import sleep

class Robot_lowlevel_control(QThread):
    
    # 关节状态反馈信号
    sin_feedback = pyqtSignal(list)

    # 机器人 启动 停止 通知信号
    sin_robot_start_compelete = pyqtSignal()
    sin_robot_stop_compelete = pyqtSignal()

    # 反馈关节状态列表
    __pos_joints_feeback = [0, 0, 0, 0, 0]


    def __init__(self,have_gripper):

        # print "Robot_lowlevel_control__init__"
        super(Robot_lowlevel_control, self).__init__()
        self.__eds_file = RosPack().get_path('canopen_communication') + "/file/Copley.eds"
        self.__have_gripper = have_gripper
        self.__mutex = QMutex()
        self.__pos_joints_command = [0, 0, 0, 0, 0]
        self.__new_pos_joints_command = [False ,False ,False ,False ,False]
        self.__path_pos_joints_command = [0 ,0 ,0 ,0 ,0]
        self.__path_point_index = 0
        self.__length_path = 0

        self.__G0_command = 0
        self.__G6_commamd = 0
        self.__new_G0_gripper_command = False  # G0 
        self.__new_G6_gripper_command = False  # G6 

        # 初始化关节角速度（5度）
        self.__joint_velocity = 0.09

        self.__stop = False
        self.__quick_stop = False

        self.__joint_mode = True
        self.__gripper_mode = False

        rospy.init_node("modular_robot_lowlevel_control", anonymous=True)
        self.__publisher = rospy.Publisher("/low_level/joints_point_feedback",Float64MultiArray,queue_size = 10)
        self.__ros_pos_feedback = Float64MultiArray()
        self.__ros_pos_feedback.data = [0 ,0 ,0 ,0 ,0]

    def run(self):

        self.__start_communication(self.__have_gripper)

        while((not self.__stop) and (not self.__quick_stop)):

            self.__mutex.lock()

            if self.__joint_mode:
                self.__execute_joints_command()
            else:
                self.__execute_path_command()
            
            if self.__gripper_mode:
                self.__execute_gripper_command()

            self.__mutex.unlock()

            self.__transimit_feedback_data()
                      

    def __start_communication(self, have_gripper):

        self.__mutex.lock()

        self.__I1 = I100(1,self.__eds_file)
        self.__T2 = T100(2,self.__eds_file)
        self.__T3 = T100(3,self.__eds_file)
        self.__T4 = T100(4,self.__eds_file)
        self.__I5 = I100(5,self.__eds_file)
        self.__joints = [self.__I1 ,self.__T2 ,self.__T3 ,self.__T4 ,self.__I5]

        if have_gripper:
            self.__G0 = G100(6,self.__eds_file)
            self.__G6 = G100(7,self.__eds_file)

        for i in range(len(self.__joints)):
            self.__joints[i].start()

        if have_gripper:
            self.__G0.start()
            self.__G6.start()

        # # set position mode.
        for i in range(len(self.__joints)):
            self.__joints[i].set_mode(1)

        self.__mutex.unlock()

        self.sin_robot_start_compelete.emit()

    def __stop_communication(self, have_gripper):

        self.__mutex.lock()

        for i in range(len(self.__joints)):
            self.__joints[i].stop()

        if have_gripper:
            self.__G0.stop()
            self.__G6.stop()

        self.__mutex.unlock()

        self.sin_robot_stop_compelete.emit()
        # self.__quick_stop = False
        # self.__stop = False

    def G0_command(self, data):
        self.__G0_command = data
        self.__new_G0_gripper_command = True
        self.__gripper_mode = True
        pass
    
    def G6_command(self, data):
        self.__G6_commamd = data
        self.__new_G6_gripper_command = True
        self.__gripper_mode = True
        pass

    def I1_commnad(self, data):
        self.__pos_joints_command[0] = data
        self.__new_pos_joints_command[0] = True
        self.__joint_mode = True
        pass

    def T2_commnad(self, data):
        self.__pos_joints_command[1] = data
        self.__new_pos_joints_command[1] = True
        self.__joint_mode = True
        pass

    def T3_commnad(self, data):
        self.__pos_joints_command[2] = data
        self.__new_pos_joints_command[2] = True
        self.__joint_mode = True
        pass

    def T4_commnad(self, data):
        self.__pos_joints_command[3] = data
        self.__new_pos_joints_command[3] = True
        self.__joint_mode = True
        pass

    def I5_commnad(self, data):
        self.__pos_joints_command[4] = data
        self.__new_pos_joints_command[4] = True
        self.__joint_mode = True
        pass

    def set_jointVelocity(self, data):
        self.__joint_velocity = data

    def path_command(self,data):
        self.__joint_mode = False
        self.__path_pos_joints_command = data
        self.__length_path = len(self.__path_pos_joints_command)
        self.__path_point_index = 0


    def __execute_joints_command(self):
        for i in range(len(self.__joints)):
            if self.__new_pos_joints_command[i]:
                self.__joints[i].sent_position(self.__pos_joints_command[i],self.__joint_velocity)
                self.__new_pos_joints_command[i] = False

    def __execute_path_command(self):

        if (0 == self.__path_point_index):
            for i in range(len(self.__joints)):
                self.__joints[i].sent_position( self.__path_pos_joints_command[self.__path_point_index][i],  \
                                                self.__path_pos_joints_command[self.__path_point_index][i+5])

        if (self.__I1.reached() and self.__T2.reached() and self.__T3.reached() and \
            self.__T4.reached() and self.__I5.reached() ):
            
            if self.__path_point_index < (self.__length_path - 1):
                self.__path_point_index += 1
                for i in range(len(self.__joints)):
                    self.__joints[i].sent_position( self.__path_pos_joints_command[self.__path_point_index][i],  \
                                                    self.__path_pos_joints_command[self.__path_point_index][i+5])
            else:
                self.__joint_mode = True
        pass

    def __execute_gripper_command(self):
        
        if self.__new_G0_gripper_command:
            self.__G0.sent_torque(self.__G0_command)
            self.__new_G0_gripper_command = False
            
        if self.__new_G6_gripper_command:
            self.__G6.sent_torque(self.__G6_command)
            self.__new_G6_gripper_command = False

        self.__gripper_mode = False

    def robot_quick_stop(self):

        self.__quick_stop = True

        self.__mutex.lock()
        if self.__quick_stop:
            for i in range(len(self.__joints)):
                self.__joints[i].quick_stop()
            if self.__have_gripper:
                self.__G0.quick_stop()
                self.__G6.quick_stop()
        self.__mutex.unlock()

        self.__stop_communication()

    def robot_stop(self):

        self.__stop = True
        self.__stop_communication(self.__have_gripper)

    def robot_halt(self,data):
        
        self.__mutex.lock()

        if data:
            for i in range(len(self.__joints)):
                self.__joints[i].pause_run()

        else:
            for i in range(len(self.__joints)):
                self.__joints[i].continue_run()

        self.__mutex.unlock()

    def __transimit_feedback_data(self):
        
        self.__mutex.lock()
        for i in range(len(self.__joints)):

            self.__pos_joints_feeback[i] = self.__joints[i].get_position()
            self.__ros_pos_feedback.data[i] = self.__pos_joints_feeback[i]
        self.__mutex.unlock()

        self.sin_feedback.emit(self.__pos_joints_feeback)
        self.__publisher.publish(self.__ros_pos_feedback)                    
        sleep(0.02) # 20ms  