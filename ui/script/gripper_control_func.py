#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: gripper_control_func.py

@biref:夹持器控制窗口
"""
from gripper_control import Ui_Form_gripper

from PyQt5.QtWidgets import QWidget, QDesktopWidget
from PyQt5.QtGui import QIntValidator
from PyQt5.QtCore import pyqtSignal

class Gripper_control_func(QWidget,Ui_Form_gripper):

    # 爪子打开关闭信号
    sin_open_or_close_gripper0 = pyqtSignal(int)
    sin_open_or_close_gripper6 = pyqtSignal(int)

    #　启动夹持器反馈状态
    sin_open_gripper_feedback = pyqtSignal(bool)

    def __init__(self):
        super(Gripper_control_func,self).__init__()
        self.setupUi(self)
        
        # 电流值范围0~1000（int）
        pIntValidator = QIntValidator(self)
        pIntValidator.setRange(0,1000)
        self.lineEdit.setValidator(pIntValidator)

        # 电流默认值 200mＡ
        self.__current = 200 
        self.lineEdit.setText(str(self.__current))
        pass

    def open_gripper_current_feedback(self):
        self.sin_open_gripper_feedback.emit(True)

    def gripper_torque_set(self):
        self.__current = int(str(self.lineEdit.text()))
        # print type(self.__current)
        # print (self.__current)

    def gripper_0_open(self,data):
        if data:
            self.sin_open_or_close_gripper0.emit(self.__current)
        else:
            self.sin_open_or_close_gripper0.emit(0)

    def gripper_0_close(self,data):
        if data:
            self.sin_open_or_close_gripper0.emit(-self.__current)
        else:
            self.sin_open_or_close_gripper0.emit(0)

    def gripper_6_open(self,data):
        if data:
            self.sin_open_or_close_gripper6.emit(self.__current)
        else:
            self.sin_open_or_close_gripper6.emit(0)

    def gripper_6_close(self,data):
        if data:
            self.sin_open_or_close_gripper6.emit(-self.__current)
        else:
            self.sin_open_or_close_gripper6.emit(0)

    # 电流状态显示
    def display(self,data):
        self.lineEdit_2.setText(str(data[0]))
        self.lineEdit_3.setText(str(data[1]))

    def close_windows(self):
        self.sin_open_gripper_feedback.emit(False)
        self.close()