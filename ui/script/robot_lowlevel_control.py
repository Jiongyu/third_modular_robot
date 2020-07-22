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
from ui.msg import robot_feedback

import sys
from rospkg import RosPack
sys.path.append(RosPack().get_path('canopen_communication') + "/modular/")
import traceback

from modular_G100 import G100
from modular_I100 import I100
from modular_T100 import T100
from modular_I85 import I85

from PyQt5.QtCore import QThread,pyqtSignal,QMutex

from time import sleep

class Robot_lowlevel_control(QThread):
    
    # 关节状态反馈信号
    sin_feedback = pyqtSignal(list)

    # 机器人 启动 停止 通知信号
    sin_robot_start_compelete = pyqtSignal()
    sin_robot_start_error = pyqtSignal()
    sin_robot_stop_compelete = pyqtSignal()

    # 反馈关节状态列表
    __pos_joints_feeback = [0, 0, 0, 0, 0]


    def __init__(self,have_gripper,ros_pos_feed,ros_vel_feed,ros_curr_feed):

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

        self.__ros_pos_feedback = ros_pos_feed
        self.__ros_vel_feedback = ros_vel_feed
        self.__ros_current_feedback = ros_curr_feed 

        # 初始化关节角速度（5度）
        self.__joint_velocity = 5

        self.__stop = False
        self.__quick_stop = False

        self.__joint_mode = True
        self.__gripper_mode = False

        self.__robot_start = False

        if self.__ros_pos_feedback or self.__ros_vel_feedback or self.__ros_current_feedback:
            rospy.init_node("modular_robot_lowlevel_control", anonymous=True)
            self.__publisher = rospy.Publisher("/low_level/joints_point_feedback",robot_feedback,queue_size = 10)
            self.__ros_feedback = robot_feedback()
            self.__ros_feedback.feedbackPosData.data = [0 ,0 ,0 ,0 ,0]
            self.__ros_feedback.feedbackVelData.data = [0 ,0 ,0 ,0 ,0]
            self.__ros_feedback.feedbackCurrData.data = [0 ,0 ,0 ,0 ,0]


    def run(self):
        self.__robot_start = self.__start_communication(self.__have_gripper)
        if(self.__robot_start):
            while((not self.__stop) and (not self.__quick_stop)):

                self.__mutex.lock()
                
                if self.__joint_mode:
                    self.__execute_joints_command()
                elif self.__execute_path_command:
                    self.__execute_path_command()
                elif self.__gripper_mode:
                    self.__execute_gripper_command()

                self.__mutex.unlock()

                self.__transimit_feedback_data()  
               

    def __start_communication(self, have_gripper):

        self.__mutex.lock()
        try:
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
                self.__joints[i].set_mode('PROFILED POSITION')

        # # test code ######################### 
        # self.__I1 = I85(4, self.__eds_file)
        # self.__I1.start()
        # self.__I1.opmode_set('PROFILED POSITION')
        except Exception as e:
            traceback.print_exc()
            self.sin_robot_start_error.emit()
            self.__mutex.unlock()
            return False
        
        # #####################################

        self.__mutex.unlock()
        return True

        self.sin_robot_start_compelete.emit()

    def __stop_communication(self, have_gripper):

        if self.__robot_start:
            for i in range(len(self.__joints)):
                self.__joints[i].stop()

            if have_gripper:
                self.__G0.stop()
                self.__G6.stop()

            # # test code ######################### 
            # self.__I1.stop()
            # #####################################

            self.sin_robot_stop_compelete.emit()
            self.__quick_stop = False
            self.__stop = False

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

        # ###  test code ###############################
        # if self.__new_pos_joints_command[0]:
        #     print "execute joint command"
        #     self.__I1.sent_position(self.__pos_joints_command[0],self.__joint_velocity)
        #     self.__new_pos_joints_command[0] = False
        # ##############################################      

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

        # ## test code #################################
        # if (0 == self.__path_point_index):
        #     self.__I1.sent_position(    self.__path_pos_joints_command[self.__path_point_index][0],  \
        #                                 self.__path_pos_joints_command[self.__path_point_index][5])

        # if (self.__I1.reached()):
            
        #     if self.__path_point_index < (self.__length_path - 1):
        #         self.__path_point_index += 1
        #         self.__I1.sent_position(    self.__path_pos_joints_command[self.__path_point_index][0],  \
        #                                     self.__path_pos_joints_command[self.__path_point_index][5])
        #     else:
        #         self.__joint_mode = True
        # ###############################################

    def __execute_gripper_command(self):
        
        if self.__new_G0_gripper_command:
            self.__G0.sent_torque(self.__G0_command)
            self.__new_G0_gripper_command = False
            
        if self.__new_G6_gripper_command:
            self.__G6.sent_torque(self.__G6_command)
            self.__new_G6_gripper_command = False

        self.__gripper_mode = False

    def robot_quick_stop(self):
        if self.__robot_start:
            self.__mutex.lock()
            self.__quick_stop = True
            if self.__quick_stop:
                for i in range(len(self.__joints)):
                    self.__joints[i].quick_stop()
            self.__stop_communication(self.__have_gripper)

            self.__mutex.unlock()


    def robot_stop(self):
        if self.__robot_start:
            self.__mutex.lock()
            self.__stop = True
            self.__stop_communication(self.__have_gripper)
            self.__mutex.unlock()

    def robot_halt(self,data):
        if self.__robot_start:
            self.__mutex.lock()

            if data:
                for i in range(len(self.__joints)):
                    self.__joints[i].pause_run()

            else:
                for i in range(len(self.__joints)):
                    self.__joints[i].continue_run()

            # #### test code ###########################
            # if data:
            #     self.__I1.pause_run()

            # else:
            #     self.__I1.continue_run()
            # ##########################################
            self.__mutex.unlock()

    def __transimit_feedback_data(self):
        
        self.__mutex.lock()

        for i in range(len(self.__joints)):
            self.__pos_joints_feeback[i] = self.__joints[i].get_position()
            if self.__ros_pos_feedback or self.__ros_vel_feedback or self.__ros_current_feedback:
                if self.__ros_pos_feedback:
                    self.__ros_feedback.feedbackPosData.data[i] = self.__pos_joints_feeback[i]
                if self.__ros_vel_feedback:
                    self.__ros_feedback.feedbackVelData.data[i] = self.__joints[i].get_velocity()
                if self.__ros_current_feedback:
                    self.__ros_feedback.feedbackCurrData.data[i] = self.__joints[i].get_torque()

        # ### test code #################################################
        # self.__pos_joints_feeback[0] = self.__I1.get_position()
        # if self.__ros_pos_feedback or self.__ros_vel_feedback or self.__ros_current_feedback:
        #     self.__ros_feedback.timeHeader.stamp = rospy.Time.now()
        #     if self.__ros_pos_feedback:
        #         self.__ros_feedback.feedbackPosData.data[0] = self.__pos_joints_feeback[0]
        #     if self.__ros_vel_feedback:
        #         self.__ros_feedback.feedbackVelData.data[0] = self.__I1.get_velocity()
        #     if self.__ros_current_feedback:
        #         self.__ros_feedback.feedbackCurrData.data[0] = self.__I1.get_torque()
        # ###############################################################

        self.__mutex.unlock()

        self.sin_feedback.emit(self.__pos_joints_feeback)
        if self.__ros_pos_feedback or self.__ros_vel_feedback or self.__ros_current_feedback:
            self.__ros_feedback.timeHeader.stamp = rospy.Time.now()
            self.__publisher.publish(self.__ros_feedback)                    
        sleep(0.02) # 20ms  