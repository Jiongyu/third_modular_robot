#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: 
@file: robot_lowlevel_control_muti_thread.py

@biref: 上位机机器人底层控制
"""
import sys
from rospkg import RosPack
sys.path.append(RosPack().get_path('canopen_communication') + "/modular/")

import threading
import traceback

from modular_G100 import G100
from modular_I100 import I100
from modular_T100 import T100
from modular_I85 import I85

from PyQt5.QtCore import QThread,pyqtSignal,QMutex

from time import sleep

import threading

class Robot_lowlevel_control_Muti_thread(QThread):
    
    # 关节状态反馈信号
    sin_feedback = pyqtSignal(list)

    # 机器人 启动 停止 通知信号
    sin_robot_start_compelete = pyqtSignal()
    sin_robot_start_error = pyqtSignal()
    sin_robot_stop_compelete = pyqtSignal()

    # 软限位触发
    sin_robot_soft_limit_happen = pyqtSignal() 

    # 反馈夹持器电流
    sin_gripper_feedback = pyqtSignal(list)

    # 反馈关节状态列表
    __pos_joints_feeback = [0, 0, 0, 0, 0]
    __vel_joints_feedback = [0, 0, 0, 0, 0]
    __current_joints_feedback = [0, 0, 0, 0, 0]

    def __init__(self,which_robot):

        # print "Robot_lowlevel_control__init__"
        super(Robot_lowlevel_control_Muti_thread, self).__init__()
        # canopen字典
        self.__eds_file = RosPack().get_path('canopen_communication') + "/file/Copley.eds"
        self.__which_robot = which_robot
        # 线程同步锁
        self.__mutex = QMutex()
        # 机器人关节位置命令
        self.__pos_joints_command = [0, 0, 0, 0, 0]
        self.__old_pos_joints_command = [0, 0, 0, 0, 0]
        # 判断机器人关节是否为新命令
        self.__new_pos_joints_command = [False ,False ,False ,False ,False]
        # 轨迹数据
        self.__path_pos_joints_command = []
        # 轨迹数据index
        self.__path_point_index = 0
        # 轨迹长度
        self.__length_path = 0

        # 夹持器命令
        self.__G0_command = 0
        self.__G6_command = 0
        self.__new_G0_gripper_command = False  # G0
        self.__new_G6_gripper_command = False  # G6 
        # 是否打开夹持器反馈
        self.__open_gripper_feedback = False


        # 初始化关节角速度控制
        self.__joint_velocity = [0, 0, 0, 0, 0]
        self.__old_joint_velocity = [0, 0, 0, 0, 0]

        # 机器人停止标志
        self.__stop = False
        # 机器人急停标志
        self.__quick_stop = False

        # 机器人位置模式
        self.__joint_pos_mode = False
        # 机器人速度模式
        self.__joint_vel_mode = False
        # 夹持器模式
        self.__gripper_mode = False
        # 离线轨迹模式
        self.path_command_mode = False

        # 机器人启动标志
        self.__robot_start = False

        # 离线轨迹模式下，判断是否微调
        self.__if_fine_tune = False

        # 机器人软限位
        self.__joint_i_limit = 360
        self.__joint_t_limit = 120
        
        # 软限位发生标志
        self.__robot_soft_limit_happen = False

    def run(self):
        # 发送命令线程
        self.thread_command = threading.Thread(target = self.thread_command_func, args= (None,))
        # 反馈数据线程
        self.thread_feedback = threading.Thread(target = self.thread_feedback_func, args= (None,))

        # 线程启动
        self.thread_command.start()
        self.thread_feedback.start()

        # 等待线程结束
        self.thread_command.join()
        self.thread_feedback.join()
               
    def thread_command_func(self,data):

        self.__robot_start = self.__start_communication(self.__which_robot)
        try:
            if self.__robot_start:
                while((not self.__stop) and (not self.__quick_stop)):

                    if self.__joint_pos_mode:
                        self.__execute_joints_position_command()
                    elif self.__joint_vel_mode:
                        self.__exeute_joints_velocity_command()
                    elif self.path_command_mode:
                        self.__execute_path_command()
                    elif self.__gripper_mode:
                        self.__execute_gripper_command()
                    sleep(0.1)
        except Exception as e:
            traceback.print_exc()

    def thread_feedback_func(self,data):
        
        while not self.__robot_start:
            sleep(0.5)
        while((not self.__stop) and (not self.__quick_stop)):
            self.__transimit_feedback_data()  


    # 机器人启动
    def __start_communication(self, which_robot):

        try:
            self.__mutex.lock()
            self.__I1 = I100(1,self.__eds_file)
            self.__T2 = T100(2,self.__eds_file)
            self.__T3 = T100(3,self.__eds_file)
            self.__T4 = T100(4,self.__eds_file)
            self.__I5 = I100(5,self.__eds_file)
            self.__joints = [self.__I1 ,self.__T2 ,self.__T3 ,self.__T4 ,self.__I5]

            if which_robot == 0:
                self.__G0 = G100(6,self.__eds_file)
                self.__G6 = G100(7,self.__eds_file)

            for i in range(len(self.__joints)):
                self.__joints[i].start()

            if which_robot == 0:
                self.__G0.start()
                self.__G6.start()

            self.__mutex.unlock()

            self.robot_set_position_mode()

        # # test code ######################### 
        # self.__I1 = I85(4, self.__eds_file)
        # self.__I1.start()
        # self.__I1.opmode_set('PROFILED POSITION')

        except Exception as e:
            # print "__start_communication error"
            self.__mutex.unlock()
            traceback.print_exc()
            self.sin_robot_start_error.emit()
            return False
        
        # #####################################

        self.sin_robot_start_compelete.emit()
        return True

    # 机器人停止
    def __stop_communication(self, which_robot):

        if self.__robot_start:
            for i in range(len(self.__joints)):
                self.__joints[i].stop_communication()

            if which_robot == 0:
                self.__G0.stop_communication()
                self.__G6.stop_communication()

            # # test code ######################### 
            # self.__I1.stop()
            # #####################################

            self.sin_robot_stop_compelete.emit()
            self.__quick_stop = False
            self.__stop = False

    # 夹持器G0命令槽函数
    def G0_command(self, data):
        self.__G0_command = data
        self.__new_G0_gripper_command = True
        self.__gripper_mode = True
        pass

    # 夹持器G6命令槽函数
    def G6_command(self, data):
        self.__G6_command = data
        self.__new_G6_gripper_command = True
        self.__gripper_mode = True
        pass

    # 打开夹持器反馈槽函数
    def open_gripper_feedback(self, data):
        self.__open_gripper_feedback = data

    # 关节控制位置控制 命令槽函数
    def set_jointPosition(self,data):
        self.robot_set_position_mode()
        self.__pos_joints_command = data[0]
        self.__joint_velocity = data[1]
        pass

    # 关节空间速度控制 命令槽函数
    def set_jointVelocity(self, data):
        self.robot_set_velocity_mode()
        self.__joint_velocity = data
    
    # 离线轨迹槽函数
    def path_command(self,data):
        self.robot_path_mode()
        self.__path_pos_joints_command = data
        self.__length_path = len(self.__path_pos_joints_command)
        self.__path_point_index = 0
        self.path_command_mode = True
        self.__if_fine_tune = False


    # 离线轨迹继续运行
    def continue_path_command(self):
        if not self.path_command_mode:
            self.robot_path_mode()
            self.path_command_mode = True
            self.__if_fine_tune = True


    # 执行关节空间位置控制命令
    def __execute_joints_position_command(self):
        if not self.__robot_soft_limit_happen:
            self.__mutex.lock()
            for i in range(len(self.__joints)):
                if i == 0 and i == 4:
                    if abs(self.__pos_joints_command[i]) < self.__joint_i_limit:
                        if self.__old_pos_joints_command[i] !=  self.__pos_joints_command[i]:
                            self.__joints[i].sent_position(self.__pos_joints_command[i],self.__joint_velocity[i])
                            self.__old_pos_joints_command[i] =  self.__pos_joints_command[i]
                    else:
                        self.sin_robot_soft_limit_happen.emit()
                else:
                    if abs(self.__pos_joints_command[i]) < self.__joint_t_limit:
                        if self.__old_pos_joints_command[i] !=  self.__pos_joints_command[i]:
                            self.__joints[i].sent_position(self.__pos_joints_command[i],self.__joint_velocity[i])
                            self.__old_pos_joints_command[i] =  self.__pos_joints_command[i]
                    else:
                        self.sin_robot_soft_limit_happen.emit()

                # print "__execute_joints_position_command"
            self.__joint_pos_mode = False
            self.__mutex.unlock()
            # ###  test code ###############################
            # if self.__new_pos_joints_command[0]:
            #     print "execute joint command"
            #     self.__I1.sent_position(self.__pos_joints_command[0],self.__joint_velocity)
            #     self.__new_pos_joints_command[0] = False
            # ##############################################     

    # 执行关节空间速度控制命令
    def __exeute_joints_velocity_command(self):
        if not self.__robot_soft_limit_happen:
    
            self.__mutex.lock()
            for i in range(len(self.__joints)):
                if self.__joint_velocity[i] != self.__old_joint_velocity[i]:
                    self.__joints[i].sent_velocity(self.__joint_velocity[i])
                    self.__old_joint_velocity[i] = self.__joint_velocity[i]
                # print self.__joint_velocity[i]
            self.__joint_vel_mode = False
            self.__mutex.unlock()

    # 执行离线轨迹
    def __execute_path_command(self):

        if ((0 <= self.__path_point_index) and (self.__length_path > self.__path_point_index) ):
            
            path_time_location = len(self.__path_pos_joints_command[self.__path_point_index]) - 1
            while self.__path_point_index < self.__length_path:
                
                if self.__robot_soft_limit_happen:
                    break

                if(self.__path_pos_joints_command[self.__path_point_index] == "HALT"):
                    self.__path_point_index += 1
                    break

                # 判断机器人暂停后，是否人为调整，从而更新计算相应的延迟时间
                if self.__if_fine_tune:
                    self.__if_add_time_for_path_command()

                self.__mutex.lock()
                for i in range(len(self.__joints)):
                    if i == 0 and i == 4:
                        if abs(self.__path_pos_joints_command[self.__path_point_index][i]) < self.__joint_i_limit:
                            self.__joints[i].sent_position( self.__path_pos_joints_command[self.__path_point_index][i],  \
                                                            self.__path_pos_joints_command[self.__path_point_index][i+5] )
                        else:
                            self.sin_robot_soft_limit_happen.emit()
                            self.robot_halt(True)
                            break
                    else:
                        if abs(self.__path_pos_joints_command[self.__path_point_index][i]) < self.__joint_t_limit:
                            self.__joints[i].sent_position( self.__path_pos_joints_command[self.__path_point_index][i],  \
                                                            self.__path_pos_joints_command[self.__path_point_index][i+5] )
                        else:
                            self.sin_robot_soft_limit_happen.emit()
                            self.robot_halt(True)
                            break

                self.__mutex.unlock()
                sleep(self.__path_pos_joints_command[self.__path_point_index][path_time_location])
                self.__path_point_index += 1
                if ((self.__stop) or (self.__quick_stop)):
                    break
        self.path_command_mode = False

    # 笛卡尔空间调整后，更新轨迹延时参数，防止轨迹中断
    def __if_add_time_for_path_command(self):
        temp_time = []
        self.__mutex.lock()
        # 计算当前关节位置与下一个路径点之间的延迟时间
        max_vel = max(self.__path_pos_joints_command[self.__path_point_index][5:9])
        for i in range(len(self.__joints)):
            temp =  abs(self.__pos_joints_feeback[i] - self.__path_pos_joints_command[self.__path_point_index][i])
            if( max_vel != 0):
                temp_time.append(temp / max_vel)
            else:
                temp_time.append(0)

        # 比较最新延迟时间与预设延迟时间，更新
        path_time_location = len(self.__path_pos_joints_command[self.__path_point_index]) - 1
        if(max(temp_time) != self.__path_pos_joints_command[self.__path_point_index][path_time_location]):
            self.__path_pos_joints_command[self.__path_point_index][path_time_location] = max(temp_time)
            pass
    
        # 更新各关节路径点速度
        for i in range(len(self.__joints)):
            temp =  abs(self.__pos_joints_feeback[i] - self.__path_pos_joints_command[self.__path_point_index][i])
            temp_vel = temp / self.__path_pos_joints_command[self.__path_point_index][path_time_location]
            self.__path_pos_joints_command[self.__path_point_index][i + 5] = temp_vel

        self.__mutex.unlock()
        self.__if_fine_tune = False
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
        #         self.__joint_pos_mode = True
        # ###############################################

    # 执行夹持器命令
    def __execute_gripper_command(self):
        if self.__new_G0_gripper_command:
            self.__mutex.lock()
            try:
                self.__G0.sent_current(self.__G0_command)
            except:
                self.__G0.sent_current(self.__G0_command)
            self.__mutex.unlock()
            self.__new_G0_gripper_command = False
            
        if self.__new_G6_gripper_command:
            self.__mutex.lock()
            try:
                self.__G6.sent_current(self.__G6_command)
            except:
                self.__G6.sent_current(self.__G6_command)
            self.__mutex.unlock()
            self.__new_G6_gripper_command = False

        self.__gripper_mode = False

    # 机器人急停
    def robot_quick_stop(self):
        if self.__robot_start:
            self.__mutex.lock()
            self.__quick_stop = True
            for i in range(len(self.__joints)):
                self.__joints[i].quick_stop()
            if self.__which_robot == 0:
                self.__G0.quick_stop()
                self.__G6.quick_stop();
            # self.__stop_communication(self.__which_robot)
            self.__mutex.unlock()

    # 机器人停止
    def robot_stop(self):
        if self.__robot_start:
            self.__mutex.lock()
            self.__stop = True
            if self.__which_robot == 0:
                self.__G0.stop()
                self.__G6.stop()
            self.__stop_communication(self.__which_robot)
            self.__mutex.unlock()

    # 机器人暂停
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

    # 反馈机器人状态
    def __transimit_feedback_data(self):
            
        self.__mutex.lock()

        try:

            for i in range(len(self.__joints)):
                self.__pos_joints_feeback[i] = self.__joints[i].get_position()
                self.__vel_joints_feedback[i] = self.__joints[i].get_velocity()
                self.__current_joints_feedback[i] = self.__joints[i].get_current()

            if self.__open_gripper_feedback:
                self.sin_gripper_feedback.emit([self.__G0.get_current(), self.__G6.get_current()])        

            self.sin_feedback.emit([self.__pos_joints_feeback, self.__vel_joints_feedback, self.__current_joints_feedback])                   
            
        except Exception as e:
            traceback.print_exc()
        
        self.__mutex.unlock()

        sleep(0.03) # 30ms  

    # 机器人设置位置模式
    def robot_set_position_mode(self):
        # print "robot_set_position_mode"
        if not self.__joint_pos_mode:
            self.__mutex.lock()
            for i in range(len(self.__joints)):
                self.__joints[i].set_normal_mode_acc_dcc()
                self.__joints[i].opmode_set('PROFILED POSITION')
                # print "robot_set_position_mode"
            self.__joint_pos_mode = True
            self.__mutex.unlock()

    # 机器人设置速度模式
    def robot_set_velocity_mode(self):
        if not self.__joint_vel_mode:
            self.__joint_velocity = [0,0,0,0,0]
            self.__old_joint_velocity = [0,0,0,0,0]
            self.__mutex.lock()
            for i in range(len(self.__joints)):
                self.__joints[i].set_normal_mode_acc_dcc()
                self.__joints[i].opmode_set('PROFILED VELOCITY')
                # print "robot set vel mode "
            self.__joint_vel_mode = True
            self.__mutex.unlock()
        # print "robot_set_velocity_mode"
    
    def robot_path_mode(self):
        if not self.path_command_mode:
            self.__mutex.lock()
            for i in range(len(self.__joints)):
                self.__joints[i].set_path_mode_acc_dcc()
                self.__joints[i].opmode_set('PROFILED POSITION')
            self.__mutex.unlock()
        