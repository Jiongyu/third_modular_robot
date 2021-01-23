#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: biped_receive_control_command_func.py

@biref: 爬壁机器人获取控制命令界面
"""


from biped_receive_control_command import Ui_biped_receive_control_command

from get_fifo_relative_position_thread import   Get_fifo_relative_position_thread, \
                                                Translate_goal_to_base_thread, \
                                                Send_now_robot_descarte_goal_position_thread

from get_positive_solution_thread import Get_positive_solution_thread

from PyQt5.QtWidgets import QWidget, QDesktopWidget
from PyQt5.QtGui import QIntValidator
from PyQt5.QtCore import pyqtSignal, QThread, QTimer

class Biped_receive_control_command_func(QWidget,Ui_biped_receive_control_command):

    # 关节位置命令信号
    sin_joint_position = pyqtSignal(list)
    # 更新机器人基座
    sin_update_robot_base_flag = pyqtSignal(bool)
    # 请求更新机器人状态
    sin_request_update_robot_state = pyqtSignal()

    sin_close = pyqtSignal()

    def __init__(self, which_robot, base_flag, pos_joints, direction_joints, zero_pos_joints):

        super(Biped_receive_control_command_func,self).__init__()
        self.setupUi(self)

        self.__which_robot = which_robot
        self.__base_flag = base_flag
        self.__pos_joints = pos_joints
        self.__direction_joints = direction_joints
        self.__zero_pos_joints = zero_pos_joints

        self.goal_to_base_tag_ = ""
        self.init_data_5 =[0.0, 0.0, 0.0, 0.0, 0.0]
        self.init_data_6 =[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.goal_to_base_motor_temp_position_command = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.goal_to_base_motor_temp_velocity_command = [0.0, 0.0, 0.0, 0.0, 0.0]

        # 求解正运动学
        self.__get_actual_tcp_pos_Solu = QThread()

        # 延迟获取机器人状态定时器
        self.__get_robot_state_timer = QTimer()
        self.__get_robot_state_timer.timeout.connect(self.__request_update_robot_state)
        # 定时周期5s更新一次
        self.__get_robot_state_timer.start(5000)

    def receive_data(self):
        # ui界面全部数据初始化为'0'
        self._show_relative__joint_position_data(self.init_data_6)
        self.show_goal_to_base_position_data(self.init_data_6)
        self.show_goal_to_base_motor_pos_command_data(self.init_data_5, self.init_data_5)
        
        # 转换的界面标志位清空 
        self.lineEdit_72.setText(" ")

        # 调用管道接收和数据转换线程
        self.translate_pos_to_base_fun = Translate_goal_to_base_thread(self.__which_robot, self.__base_flag, self.__pos_joints)
        
        # 在界面显示 接收到的相对笛卡尔位姿
        self.translate_pos_to_base_fun.receive_relative__joint_position_data.connect(self._show_relative__joint_position_data)

        # 当直接传输的是关节I1、I5的增量时，显示增量
        self.translate_pos_to_base_fun.direct_angle_to_motor_data.connect(self.show_direct_increase_motor_angle_I1_I5)

        # 在界面显示 目标笛卡尔点到基座标系下的表示(unit : mm; deg)
        self.translate_pos_to_base_fun.goal_to_base_position_.connect(self.show_goal_to_base_position_data)

        # 在界面显示 逆解转换成功与否
        self.translate_pos_to_base_fun.wrong_translation_feedback.connect(self.show_determine_whether_translation_correct_flag)
        
        # 在界面显示 转换后电机关机运动位置+速度指令
        self.translate_pos_to_base_fun.goal_to_base_motor_pos_command.connect(self.show_goal_to_base_motor_pos_command_data)

        # 更新基座
        self.translate_pos_to_base_fun.now_base_flag.connect(self.now_base_flag_fun)
        
        self.translate_pos_to_base_fun.start()   # 线程启动

        # 运行 转换结果(槽函数)
       

        

    # 发送 当前机器人的笛卡尔坐标(槽函数)
    def send_now_robot_deacarte_data(self):                                            
        
        if not self.__get_actual_tcp_pos_Solu.isRunning():
            self.__get_actual_tcp_pos_Solu = Get_positive_solution_thread()
            self.__get_actual_tcp_pos_Solu.update_joint_state(self.__which_robot, self.__base_flag, self.__pos_joints)
            self.__get_actual_tcp_pos_Solu.sin_positive_solution.connect(self.__upadte_tcp_position)
            self.__get_actual_tcp_pos_Solu.start()
        
    # 更新正运动学解
    def __upadte_tcp_position(self, data):
        self.__now_robot_descartes_position = data
        self.__send_now_robot_deacarte_pos_fun = Send_now_robot_descarte_goal_position_thread(self.__now_robot_descartes_position, self.__base_flag)
        self.__send_now_robot_deacarte_pos_fun.start()


    # 新增函数
    # 显示接收到的相对位姿数据
    def _show_relative__joint_position_data(self, data):

        self.lineEdit_38.setText(str(data[0]))
        self.lineEdit_39.setText(str(data[1]))
        self.lineEdit_40.setText(str(data[2]))
        self.lineEdit_41.setText(str(data[3]))
        self.lineEdit_42.setText(str(data[4]))
        self.lineEdit_43.setText(str(data[5]))
        pass

    # 在界面展示目标位置到基坐标系下的位姿数据
    def show_goal_to_base_position_data(self, data):
        self.lineEdit_50.setText(str(data[0]))
        self.lineEdit_51.setText(str(data[1]))
        self.lineEdit_52.setText(str(data[2]))
        self.lineEdit_53.setText(str(data[3]))
        self.lineEdit_54.setText(str(data[4]))
        self.lineEdit_55.setText(str(data[5]))
        pass

    # 在界面显示转换成功后关机电机位置指令
    def show_goal_to_base_motor_pos_command_data(self, data_pos, data_vel):
        print "show_the_data_motor"
        self.lineEdit_59.setText(str(data_pos[0]))
        self.lineEdit_65.setText(str(data_pos[1]))
        self.lineEdit_68.setText(str(data_pos[2]))
        self.lineEdit_69.setText(str(data_pos[3]))
        self.lineEdit_70.setText(str(data_pos[4]))

        self.lineEdit_71.setText(str(data_vel[0]))
        self.lineEdit_73.setText(str(data_vel[1]))
        self.lineEdit_74.setText(str(data_vel[2]))
        self.lineEdit_75.setText(str(data_vel[3]))
        self.lineEdit_76.setText(str(data_vel[4]))

        self.goal_to_base_motor_temp_position_command[0] = data_pos[0]
        self.goal_to_base_motor_temp_position_command[1] = data_pos[1]
        self.goal_to_base_motor_temp_position_command[2] = data_pos[2]
        self.goal_to_base_motor_temp_position_command[3] = data_pos[3]
        self.goal_to_base_motor_temp_position_command[4] = data_pos[4]

        self.goal_to_base_motor_temp_velocity_command[0] = data_vel[0] * self.__direction_joints[0]
        self.goal_to_base_motor_temp_velocity_command[1] = data_vel[1] * self.__direction_joints[1]
        self.goal_to_base_motor_temp_velocity_command[2] = data_vel[2] * self.__direction_joints[2]
        self.goal_to_base_motor_temp_velocity_command[3] = data_vel[3] * self.__direction_joints[3]
        self.goal_to_base_motor_temp_velocity_command[4] = data_vel[4] * self.__direction_joints[4]

        for i in range(len(self.goal_to_base_motor_temp_position_command)):
            self.goal_to_base_motor_temp_position_command[i] = self.__user_data_to_motor(self.goal_to_base_motor_temp_position_command[i],self.__zero_pos_joints[i],self.__direction_joints[i])

        pass

    # 在ui界面显示逆解转换成功与否   
    def show_determine_whether_translation_correct_flag(self, data):
        self.lineEdit_72.setText(str(data))
        self.goal_to_base_tag_ = str(data)
        # 运行 转换结果(槽函数)
        if self.goal_to_base_tag_ == "True": 
            print "run ing..."
            # 发送关节位置和速度指令
            self.sin_joint_position.emit([self.goal_to_base_motor_temp_position_command, self.goal_to_base_motor_temp_velocity_command])
        else:
            print "run false...."
        pass
    
    # 在界面显示增加的I1/I5的关节角
    def show_direct_increase_motor_angle_I1_I5(self,data):
        self.lineEdit_60.setText(str(data[0]))
        self.lineEdit_61.setText(str(data[1]))
        pass

    # 更新基座
    def now_base_flag_fun(self, data):
        self.__base_flag = data
        self.sin_update_robot_base_flag.emit(self.__base_flag)
        pass


    # 根据零点、关节方向， 计算关节命令
    def __user_data_to_motor(self, command_pos, zero_pos, direction):
        """
        brief:  关节方向 * 用户值 + 零点   
        """
        return round((direction * command_pos + zero_pos) , 3)

    # 定时器延时到，请求更新机器人状态
    def __request_update_robot_state(self):
        self.sin_request_update_robot_state.emit()
        self.__get_robot_state_timer.stop()
        self.__get_robot_state_timer.start()
        pass

    # 更新机器人状态
    def update_robot_state(self, data):
        self.__zero_pos_joints = data[0]
        self.__direction_joints = data[1]
        self.__pos_joints = data[2]
        pass

    def closeEvent(self, event):
        self.sin_close.emit()
        event.accept()

    def close_windows(self):
        self.close()