#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: 
@file: robot_feedback_fun.py

@biref: 机器人状态反馈界面
"""

from robot_feedback import Ui_robotFeedback
from PyQt5.QtWidgets import QWidget, QDesktopWidget
from PyQt5.QtCore import pyqtSignal

from get_positive_solution_thread import Get_positive_solution_thread

class Robot_feedback_fun(QWidget, Ui_robotFeedback):

    # 获取正解信号
    sin_get_positive_solution = pyqtSignal(list)

    sin_open_robot_state_feedback = pyqtSignal(bool)

    sin_close = pyqtSignal()

    sin_update_descartes_pos_data = pyqtSignal(list)

    def __init__(self, which_robot):
        super(Robot_feedback_fun,self).__init__()
        self.setupUi(self)
        self.sin_get_positive_solution.connect(self.get_descartes_pos)
        self.__get_positive_soluition = Get_positive_solution_thread()
        self.__get_positive_soluition.sin_positive_solution.connect(self.display_descartes_position)
        self.__descartes_position = [586.4,0,0,0,0,180]
        self.__which_base = True
        self.__which_robot = which_robot

        pass

    def open_robot_state_feedback(self):
        self.sin_open_robot_state_feedback.emit(True)
    
    # 显示关节位置数据
    def display(self,data):
          
        # 关节位置 I1 T2 T3 T4 I5
        self.lineEdit_50.setText(str(data[0][0]))        
        self.lineEdit_52.setText(str(data[0][1]))        
        self.lineEdit_48.setText(str(data[0][2]))        
        self.lineEdit_49.setText(str(data[0][3]))        
        self.lineEdit_51.setText(str(data[0][4]))
        self.sin_get_positive_solution.emit(data[0]) 

        #　关节速度
        self.lineEdit_60.setText(str(data[1][0]))        
        self.lineEdit_62.setText(str(data[1][1]))        
        self.lineEdit_58.setText(str(data[1][2]))        
        self.lineEdit_59.setText(str(data[1][3]))        
        self.lineEdit_61.setText(str(data[1][4]))

        #　关节电流
        self.lineEdit_55.setText(str(data[2][0]))        
        self.lineEdit_57.setText(str(data[2][1]))        
        self.lineEdit_53.setText(str(data[2][2]))        
        self.lineEdit_54.setText(str(data[2][3]))        
        self.lineEdit_56.setText(str(data[2][4]))

    # 更新夹持器基座
    def update_gripper(self,data):
        self.__which_base = data
        pass 
    
    # 获取运动学正解
    def get_descartes_pos(self, data):
        self.__descartes_position = data
        self.__get_positive_soluition.update_joint_state(self.__which_robot, self.__which_base, self.__descartes_position)
        self.__get_positive_soluition.start()

    # 显示运动学正解数据
    def display_descartes_position(self,data):
        # 笛卡尔 X Y Z RX RY RZ
        self.lineEdit_44.setText(str(data[0]))
        self.lineEdit_47.setText(str(data[1]))        
        self.lineEdit_42.setText(str(data[2]))        
        self.lineEdit_43.setText(str(data[3]))        
        self.lineEdit_46.setText(str(data[4]))        
        self.lineEdit_45.setText(str(data[5]))    
        
        self.sin_update_descartes_pos_data.emit(data)

    # 关闭界面
    def close_windows(self):
        self.sin_open_robot_state_feedback.emit(False)
        self.close()
        pass

    def closeEvent(self, event):
        self.sin_close.emit()
        event.accept()