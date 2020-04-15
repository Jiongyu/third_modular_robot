#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:Jony
@contact: 35024339@qq.com
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

    def __init__(self):
        super(Gripper_control_func,self).__init__()
        self.setupUi(self)
        
        # 力矩值范围0~2000（int）
        pIntValidator = QIntValidator(self)
        pIntValidator.setRange(0,2000)
        self.lineEdit.setValidator(pIntValidator)

        # 力矩默认值 1200mN.m
        self.__torque = 1200 
        self.lineEdit.setText(str(self.__torque))
        pass

    def gripper_torque_set(self):
        self.__torque = int(str(self.lineEdit.text()))
        # print type(self.__torque)
        # print (self.__torque)

    def gripper_0_open(self,data):
        if data:
            self.sin_open_or_close_gripper0.emit(self.__torque)
        else:
            self.sin_open_or_close_gripper0.emit(0)

    def gripper_0_close(self,data):
        if data:
            self.sin_open_or_close_gripper0.emit(-self.__torque)
        else:
            self.sin_open_or_close_gripper0.emit(0)

    def gripper_6_open(self,data):
        if data:
            self.sin_open_or_close_gripper6.emit(self.__torque)
        else:
            self.sin_open_or_close_gripper6.emit(0)

    def gripper_6_close(self,data):
        if data:
            self.sin_open_or_close_gripper6.emit(self.__torque)
        else:
            self.sin_open_or_close_gripper6.emit(0)

    def close_windows(self):
        self.close()