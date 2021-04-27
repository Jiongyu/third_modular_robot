#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: auto_gripper_func.py

@biref:自主抓夹窗口
"""
from auto_gripper import Ui_auto_gripper

from PyQt5.QtWidgets import QWidget, QDesktopWidget
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtCore import pyqtSignal, QThread

from auto_gripper_get_pole_point_thread import Auto_gripper_get_pole_point_thread
from auto_gripper_get_grasp_point_thread import Get_grasp_point_thread
from get_inverse_solution_thread import Get_inverse_solution_thread

class Auto_gripper_func(QWidget,Ui_auto_gripper):


    sin_close = pyqtSignal()

    # 发送机器人控制命令
    sin_robot_command = pyqtSignal(list)

    def __init__(self, which_base, descartes_position, current_joint_pos):
        super(Auto_gripper_func,self).__init__()
        self.setupUi(self)
        # 夹持点位置姿态初始化
        self.__grasp_point = [0] * 6
        self.__which_base = which_base
        self.__descartes_position = descartes_position
        self.__current_joint_pos = current_joint_pos

        # 获取杆件端点
        self.__get_pole_point_thread = QThread()
        # 获取夹持点
        self.__get_grasp_point_thread = QThread()
        # 获取逆解
        self.__get_inverse_solution_thread = QThread()

        # 默认笛卡尔空间运动速度
        self.__descartes_vel = 5
        self.__descartes_vel_list = [0] * 6

        self.__joint_commad = [[0,0,0,0,0],[0,0,0,0,0]]

        self.__input_setting()

    def get_grasp_point(self):
        # 启动获取杆件端点
        if not self.__get_pole_point_thread.isRunning():
            # 获取杆件端点
            self.__get_pole_point_thread = Auto_gripper_get_pole_point_thread()
            # 杆件端点回调
            self.__get_pole_point_thread.sin_pole_point.connect(self.__update_pole_point)
            self.__get_pole_point_thread.start()
        pass

    def __update_pole_point(self, data):
        # print str(data) + "  __update_pole_point"
        # 启动获取抓夹点
        if not self.__get_grasp_point_thread.isRunning():
            # 获取夹持点
            self.__get_grasp_point_thread = Get_grasp_point_thread(self.__which_base, self.__descartes_position, data[0], data[1])
            # 夹持点回调
            self.__get_grasp_point_thread.sin_grasp_point.connect(self.__update_grasp_point)
            self.__get_grasp_point_thread.start()
            # 获取杆件点 取消连接
            self.__get_pole_point_thread.sin_pole_point.disconnect()
            del self.__get_pole_point_thread
            self.__get_pole_point_thread = QThread()
        pass
        
    def __update_grasp_point(self, data):
        # print str(data) + "  __update_grasp_point"
        if(data[1]):
            self.__grasp_point = list(data[0])
            for i in range(len(self.__grasp_point)):
                self.__grasp_point[i] = round(self.__grasp_point[i], 2)

            self.show_grasp_point_data(self.__grasp_point)

        pass

    def __calculate_descartes_vel(self, current, later):
        for i in range(len(self.__descartes_vel_list)):
            if(abs(current[i] - later[i]) > 0.001):
                self.__descartes_vel_list[i] = self.__descartes_vel
            else:
                self.__descartes_vel_list[i] = 0

    def adjust_posture(self):      
        if not self.__get_inverse_solution_thread.isRunning():
            try:
                self.__grasp_point[3] = round(float(self.lineEdit_4.text()), 3)
                self.__grasp_point[4] = round(float(self.lineEdit_5.text()), 3)
                self.__grasp_point[5] = round(float(self.lineEdit_6.text()), 3)
            except:
                print "__grasp_point adjust_posture input error"
                return
            temp_pos = self.__descartes_position[0:3] + self.__grasp_point[3:6]
            
            # print temp_pos
            # print self.__descartes_vel_list
            self.__get_inverse_solution_thread = Get_inverse_solution_thread(0, self.__which_base, temp_pos, \
                                                        self.__descartes_vel_list, self.__current_joint_pos)
            self.__get_inverse_solution_thread.sin_inverse_solution.connect(self.__get_inverse_solution)   
            
            self.__get_inverse_solution_thread.start()     
            # print self.__grasp_point                                
        pass

    def adjust_position(self):
        if not self.__get_inverse_solution_thread.isRunning():
            try:
                self.__grasp_point[0] = round(float(self.lineEdit.text()), 3)
                self.__grasp_point[1] = round(float(self.lineEdit_2.text()), 3)
                self.__grasp_point[2] = round(float(self.lineEdit_3.text()), 3)
            except: 
                print "__grasp_point adjust_position input error"
                return
            temp_pos = self.__descartes_position[0:3] + self.__grasp_point[3:6]
            # print temp_pos
            self.__calculate_descartes_vel(temp_pos, self.__grasp_point)
            self.__get_inverse_solution_thread = Get_inverse_solution_thread(0, self.__which_base, self.__grasp_point, \
                                                        self.__descartes_vel_list, self.__current_joint_pos)
            self.__get_inverse_solution_thread.sin_inverse_solution.connect(self.__get_inverse_solution)   
            
            self.__get_inverse_solution_thread.start() 
            # print self.__descartes_position
        pass

    def __get_inverse_solution(self,data):
        # print data
        self.show_joint_command_data(data)
        self.__joint_commad = data

    def sent_joint_command(self):
        try:
            self.__joint_commad[0][0] = round(float(self.lineEdit_16.text()), 3)
            self.__joint_commad[0][1] = round(float(self.lineEdit_13.text()), 3)
            self.__joint_commad[0][2] = round(float(self.lineEdit_18.text()), 3)
            self.__joint_commad[0][3] = round(float(self.lineEdit_17.text()), 3)
            self.__joint_commad[0][4] = round(float(self.lineEdit_14.text()), 3)
        except:
            print "sent_joint_command input error"
            return
        # print self.__joint_commad
        self.sin_robot_command.emit(self.__joint_commad)
        pass

    # 显示抓夹点数据
    def show_grasp_point_data(self,data):
        self.lineEdit.setText(str(data[0]))
        self.lineEdit_2.setText(str(data[1]))
        self.lineEdit_3.setText(str(data[2]))
        self.lineEdit_4.setText(str(data[3]))
        self.lineEdit_5.setText(str(data[4]))
        self.lineEdit_6.setText(str(data[5]))
        pass

    # 显示关节命令
    def show_joint_command_data(self, data):
        self.lineEdit_16.setText(str(data[0][0]))
        self.lineEdit_13.setText(str(data[0][1]))
        self.lineEdit_18.setText(str(data[0][2]))
        self.lineEdit_17.setText(str(data[0][3]))
        self.lineEdit_14.setText(str(data[0][4]))
        pass

    def close_windows(self):
        self.close()

    def closeEvent(self, event):
        self.sin_close.emit()
        event.accept()

    # 输入框输入限制
    def __input_setting(self):
        pDoubleDescartesIncre = QDoubleValidator(self)
        pDoubleDescartesIncre.setNotation(QDoubleValidator.StandardNotation)
        pDoubleDescartesIncre.setDecimals(2)
        self.lineEdit.setValidator(pDoubleDescartesIncre)
        self.lineEdit_2.setValidator(pDoubleDescartesIncre)
        self.lineEdit_3.setValidator(pDoubleDescartesIncre)
        self.lineEdit_4.setValidator(pDoubleDescartesIncre)
        self.lineEdit_5.setValidator(pDoubleDescartesIncre)
        self.lineEdit_6.setValidator(pDoubleDescartesIncre)
        pass