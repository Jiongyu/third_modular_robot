#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:Jony
@contact: 35024339@qq.com
@software: RoboWareStudio
@file: modular_robot_control_func.py

@biref:上位机主窗口，包含关节控制，路径离线控制
"""

from modular_robot_control import Ui_MainWindow_modular_robot
from path_point_recorder_func import Path_point_recorder_func
from gripper_control_func import Gripper_control_func
from zero_point_set_func import Zero_point_set_func
from robot_lowlevel_control import Robot_lowlevel_control
from path_process import Path_process

from PyQt5.QtWidgets import QMainWindow, QDesktopWidget, QMessageBox, QFileDialog
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QDoubleValidator

import traceback

class Modular_robot_control_func(QMainWindow,Ui_MainWindow_modular_robot):

    # 窗口传递信号
    sin_sent_data_to_WindowsPathRecoder = pyqtSignal(list)
    sin_sent_data_to_windowsSetZero     = pyqtSignal(list)

    # 关闭窗口信号
    sin_close_windowsPathRecoder    = pyqtSignal()
    sin_close_windowsSetZero        = pyqtSignal()
    sin_close_windowsGripper        = pyqtSignal()
    
    # 停止、急停、运行信号
    sin_stop_robot_operation            = pyqtSignal()
    sin_quickStop_robot_operation       = pyqtSignal()
    sin_halt_robot_operation            = pyqtSignal(bool)

    # 关节命令信号
    sin_I1_command = pyqtSignal(float)
    sin_T2_command = pyqtSignal(float)
    sin_T3_command = pyqtSignal(float)
    sin_T4_command = pyqtSignal(float)
    sin_I5_command = pyqtSignal(float)
    sin_joint_velocity = pyqtSignal(float)

    # 夹持器信号
    sin_G0_command = pyqtSignal(int)
    sin_G6_command = pyqtSignal(int)

    # 路径信号
    sin_path_command = pyqtSignal(list)

    # 动态关节值
    # position I1, T2, T3, T4, I5
    __pos_joints = [0,0,0,0,0]

    def  __init__(self,have_gripper):
        super(Modular_robot_control_func,self).__init__()
        self.setupUi(self)
        self.__center()
        self.__input_range_limit()
        
        self.__string_robot_Enabled = "机器人使能成功"
        self.__string_robot_NotEnabled = "机器人未使能"
        self.__string_robot_Enable_ing = "机器人使能中"

        self.__ros_pos_feedback = False
        self.__ros_vel_feedback = False
        self.__ros_current_feedback = False 

        # connect action slots function
        self.action.triggered.connect(self.__open_gripper_control)
        self.action_2.triggered.connect(self.__open_path_point_record)
        self.action_3.triggered.connect(self.__open_set_zero_point)
        self.action_4.triggered.connect(self.__about)
        self.action_5.triggered.connect(self.__quit)
        self.action_8.triggered.connect(self.__auto_gripper_pole)
        self.action_6.triggered.connect(self.__ros_pos_feedback_Set)
        self.action_7.triggered.connect(self.__ros_vel_feedback_set)
        self.action_9.triggered.connect(self.__ros_current_feedback_set)
        self.action_10.triggered.connect(self.__about)
        # end

        self.__have_gripper = have_gripper

        self.__if_robot_enabled = False
        self.label.setText(self.__string_robot_NotEnabled)

        # 路径点集合
        self.__pos_joints_path_array = []
        self.__old_path_array = []

        # 零点 I1, T2, T3, T4, I5
        self.__zero_pos_joints = [0,0,0,0,0]
        # 关节是否正方向(1:正;-1:反)I1, T2, T3, T4, I5
        self.__direction_joints = [1,1,1,1,1]
        # 是否设置零点
        self.__if_set_zero_point = True

    def __center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def start_robot(self):

        if not self.__if_robot_enabled:
            # 底层控制
            self.__lowLevelControl = Robot_lowlevel_control(self.__have_gripper,self.__ros_pos_feedback,self.__ros_vel_feedback,self.__ros_current_feedback)                
            self.__lowLevelControl.sin_feedback.connect(self.__dispaly_feeback_data)
            self.__lowLevelControl.sin_robot_start_compelete.connect(self.__robot_start_compelete)
            self.__lowLevelControl.sin_robot_start_error.connect(self.start_robot_error)
            self.__lowLevelControl.sin_robot_stop_compelete.connect(self.__robot_stop_compelete)
            self.sin_quickStop_robot_operation.connect(self.__lowLevelControl.robot_quick_stop)
            self.sin_stop_robot_operation.connect(self.__lowLevelControl.robot_stop)
            self.sin_halt_robot_operation.connect(self.__lowLevelControl.robot_halt)
            self.sin_I1_command.connect(self.__lowLevelControl.I1_commnad)
            self.sin_T2_command.connect(self.__lowLevelControl.T2_commnad)
            self.sin_T3_command.connect(self.__lowLevelControl.T3_commnad)
            self.sin_T4_command.connect(self.__lowLevelControl.T4_commnad)
            self.sin_I5_command.connect(self.__lowLevelControl.I5_commnad)
            self.sin_joint_velocity.connect(self.__lowLevelControl.set_jointVelocity)
            self.sin_path_command.connect(self.__lowLevelControl.path_command)

            self.label.setText(self.__string_robot_Enable_ing)
            self.__lowLevelControl.start()

    def start_robot_error(self):
        self.__if_robot_enabled = False
        del self.__lowLevelControl
        self.label.setText(self.__string_robot_NotEnabled)

    def stop_robot(self):
        if self.__if_robot_enabled:
            self.sin_stop_robot_operation.emit()
            # print "Modular_robot_control_func__stop_robot"
        pass

    def quick_stop(self):
        if self.__if_robot_enabled:
            self.sin_quickStop_robot_operation.emit()
            pass
        pass

    def halt(self,data):
        if self.__if_robot_enabled:
            self.sin_halt_robot_operation.emit(data)
        pass

    def __robot_start_compelete(self):
        print "complete"
        self.__if_robot_enabled = True
        self.label.setText(self.__string_robot_Enabled)

    def __robot_stop_compelete(self):
        self.__if_robot_enabled = False
        del self.__lowLevelControl
        self.label.setText(self.__string_robot_NotEnabled)

    def joint_i1_command(self):
        if self.__if_robot_enabled:  
            print "i1_command"
            self.sin_I1_command.emit(   Modular_robot_control_func.__user_data_to_motor(   \
                                        float(str(self.lineEdit_7.text())), \
                                        self.__zero_pos_joints[0],  \
                                        self.__direction_joints[0]  ))
            self.__if_set_zero_point = False

    def joint_t2_command(self):
        if self.__if_robot_enabled:
            self.sin_T2_command.emit(   Modular_robot_control_func.__user_data_to_motor(   \
                                        float(str(self.lineEdit_8.text())), \
                                        self.__zero_pos_joints[1],  \
                                        self.__direction_joints[1]  ))
            self.__if_set_zero_point = False

    def joint_t3_command(self):
        if self.__if_robot_enabled:
            self.sin_T3_command.emit(   Modular_robot_control_func.__user_data_to_motor(   \
                                        float(str(self.lineEdit_9.text())), \
                                        self.__zero_pos_joints[2],  \
                                        self.__direction_joints[2]  ))
            self.__if_set_zero_point = False

    def joint_t4_command(self):
        if self.__if_robot_enabled:
            self.sin_T4_command.emit(   Modular_robot_control_func.__user_data_to_motor(   \
                                        float(str(self.lineEdit_10.text())), \
                                        self.__zero_pos_joints[3],  \
                                        self.__direction_joints[3]  ))
            self.__if_set_zero_point = False

    def joint_i5_command(self):
        if self.__if_robot_enabled:
            self.sin_I5_command.emit(   Modular_robot_control_func.__user_data_to_motor(   \
                                        float(str(self.lineEdit_11.text())), \
                                        self.__zero_pos_joints[4],  \
                                        self.__direction_joints[4]  ))
            self.__if_set_zero_point = False

    def joint_velocity_set(self):
        if self.__if_robot_enabled:
            self.sin_joint_velocity.emit( round(float(str(self.lineEdit_12.text())), 3) )


    def __get_G0_torque_from_windowsGripperControl(self,data):
        if self.__if_robot_enabled:
            self.sin_G0_command.emit(data)

    def __get_G6_torque_from_windowsGripperControl(self,data):
        if self.__if_robot_enabled:
            self.sin_G6_command.emit(data)

    def __dispaly_feeback_data(self,data):
        
        self.__pos_joints = data
        for i in range(len(self.__pos_joints)):
            self.__pos_joints[i] = (data[i] - self.__zero_pos_joints[i] ) * self.__direction_joints[i] 
        
        self.lineEdit_14.setText(str( self.__pos_joints[0] ))   # I1
        self.lineEdit_15.setText(str( self.__pos_joints[1] ))   # T2
        self.lineEdit_13.setText(str( self.__pos_joints[2] ))   # T3
        self.lineEdit_17.setText(str( self.__pos_joints[3] ))   # T4
        self.lineEdit_16.setText(str( self.__pos_joints[4] ))   # I5

    def load_point_file(self):

        # file_path = QFileDialog.getOpenFileName(self, "载入文件", "~/")
        # if file_path[0]:
        #     f = open(file_path[0], "r")
        #     data = f.read()
        #     f.close()
        #     self.textEdit.setText(data)

        self.__pos_joints_path_array = []
        temp_data = self.textEdit.toPlainText()
        temp_path_process = Path_process(self.__direction_joints)

        try:
            self.__pos_joints_path_array = temp_path_process.get_trajectory(temp_data) 
            QMessageBox.about(self,'通知','\n       路径点载入成功!!        \n')
        except:
            QMessageBox.about(self,'错误','\n       路径点格式错误!!        \n')
            return 
        print self.__pos_joints_path_array


    def operate_point_data(self):

            if self.__if_robot_enabled:
                if self.__pos_joints_path_array != self.__old_path_array:
                    self.__old_path_array = self.__pos_joints_path_array
                    self.sin_path_command.emit(self.__pos_joints_path_array)

    def __open_gripper_control(self):
        if self.__have_gripper:
            self.__windows_gripper_control = Gripper_control_func()
            temp = self.frameGeometry()
            self.__windows_gripper_control.move(temp.right(),temp.top())
            
            self.__windows_gripper_control.sin_open_or_close_gripper0.connect(self.__get_G0_torque_from_windowsGripperControl)
            self.__windows_gripper_control.sin_open_or_close_gripper6.connect(self.__get_G6_torque_from_windowsGripperControl)
            self.sin_close_windowsGripper.connect(self.__windows_gripper_control.close_windows)
            
            if (self.__string_robot_Enabled == self.label.text):
                self.sin_G0_command.connect(self.__lowLevelControl.G0_command)
                self.sin_G6_command.connect(self.__lowLevelControl.G6_command) 

            self.__windows_gripper_control.show()
            pass
        pass

    def __open_path_point_record(self):

        self.__windows_path_point_record = Path_point_recorder_func()
        temp = self.frameGeometry()
        temp_height = self.__windows_path_point_record.height()
        self.__windows_path_point_record.move(temp.right(), temp.top() + temp_height)
        
        self.__windows_path_point_record.sin_request_pos_joints.connect(self.__sent_data_to_windowsPathRecorder)
        self.sin_sent_data_to_WindowsPathRecoder.connect(self.__windows_path_point_record.receive_pos_joints)
        self.sin_close_windowsPathRecoder.connect(self.__windows_path_point_record.close_windows)
        self.__windows_path_point_record.show()

    def __open_set_zero_point(self):

        self.__window_zero_point_set = Zero_point_set_func(self.__zero_pos_joints, self.__direction_joints)
        temp = self.frameGeometry()
        temp_width = self.__window_zero_point_set.width()
        self.__window_zero_point_set.move(temp.left() - temp_width, temp.top())

        self.__window_zero_point_set.sin_set_zero_point.connect(self.__get_data_from_windowsSetZeroPoint)
        self.__window_zero_point_set.sin_get_actual_joints_pos.connect(self.__sent_data_to_windowsSetZero)
        self.sin_sent_data_to_windowsSetZero.connect(self.__window_zero_point_set.receive_actual_joint_point)
        self.sin_close_windowsSetZero.connect(self.__window_zero_point_set.close_windows)
        self.__window_zero_point_set.show()
        pass

    def __auto_gripper_pole(self):

        pass

    def __quit(self):
        try:

            self.__windows_path_point_record.close()
            self.__window_zero_point_set.close()
            self.__windows_gripper_control.close()
        except:
            pass
        self.close()
        pass

    def __about(self):
        QMessageBox.about(self,'关于',"模块化机器人上位机 V1.0.    \
                                     \n\nAuthor: Jony.          \
                                      \n\nMail: tanjony@qq.com")
    
    def __sent_data_to_windowsPathRecorder(self):
        self.sin_sent_data_to_WindowsPathRecoder.emit(self.__pos_joints)

    def __sent_data_to_windowsSetZero(self):
        self.sin_sent_data_to_windowsSetZero.emit(self.__pos_joints)
    
    def __get_data_from_windowsSetZeroPoint(self,data):
        if(not self.__if_set_zero_point):
            for i in range(len(data[0])):
                self.__zero_pos_joints[i] += data[0][i]
                self.__direction_joints[i] = data[1][i]
            self.__if_set_zero_point = True

    @staticmethod
    def __user_data_to_motor(command_pos, zero_pos, direction):
        """
        brief:  （用户值 + 零点）* 关节方向    
        """
        # 0.017453293 = pi/180 角度转弧度
        return round((command_pos + zero_pos) * direction,  3)

    def __input_range_limit(self):

        pDoubleValidator_I = QDoubleValidator(self)
        pDoubleValidator_I.setRange(-360, 360)
        pDoubleValidator_I.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_I.setDecimals(2)
        # I1,I5 位置范围
        self.lineEdit_7.setValidator(pDoubleValidator_I)
        self.lineEdit_11.setValidator(pDoubleValidator_I)

        pDoubleValidator_T = QDoubleValidator(self)
        pDoubleValidator_T.setRange(-120, 120)
        pDoubleValidator_T.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_T.setDecimals(2)
        # T2 T3 T4 位置范围 
        self.lineEdit_8.setValidator(pDoubleValidator_T)
        self.lineEdit_9.setValidator(pDoubleValidator_T)
        self.lineEdit_10.setValidator(pDoubleValidator_T)

        pDoubleValidator_V = QDoubleValidator(self)
        pDoubleValidator_V.setRange(1, 30)
        pDoubleValidator_V.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_V.setDecimals(2)
        # 关节速度范围
        self.lineEdit_12.setValidator(pDoubleValidator_V)


    def closeEvent(self, event):
        result = QMessageBox.question(self, "模块化机器人上位机", "Do you want to exit?", QMessageBox.Yes | QMessageBox.No)
        if(result == QMessageBox.Yes):
            self.sin_close_windowsGripper.emit()
            self.sin_close_windowsPathRecoder.emit()
            self.sin_close_windowsSetZero.emit()
            if self.__string_robot_Enabled:
                self.sin_stop_robot_operation.emit()
            event.accept()
        else:
            event.ignore()


    def __ros_pos_feedback_Set(self):
        if(self.action_6.isChecked()):
            self.__ros_pos_feedback = True
        else:
            self.__ros_pos_feedback = False
        # print self.__ros_pos_feedback

    def __ros_vel_feedback_set(self):
        if(self.action_7.isChecked()):
            self.__ros_vel_feedback = True
        else:
            self.__ros_vel_feedback = False
        # print self.__ros_vel_feedback
    
    def __ros_current_feedback_set(self):
        if(self.action_9.isChecked()):
            self.__ros_current_feedback = True
        else:
            self.__ros_current_feedback = False
        # print self.__ros_current_feedback