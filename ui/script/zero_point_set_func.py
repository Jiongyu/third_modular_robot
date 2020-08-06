#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: zero_point_set_func.py

@biref:零点设置窗口
"""

from zero_point_set import Ui_Form_zero_point

from PyQt5.QtWidgets import QWidget, QDesktopWidget
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtCore import pyqtSignal, QTimer

class Zero_point_set_func(QWidget,Ui_Form_zero_point):

    # 设置零点信号
    sin_set_zero_point = pyqtSignal(list)
    # 获取当前关节值信号
    sin_get_actual_joints_pos = pyqtSignal()

    sin_close = pyqtSignal()

    def __init__(self, pos_joint, direction_joint):
        super(Zero_point_set_func,self).__init__()
        self.setupUi(self)
        
        # 延迟获取关节值使用
        self.__qtimer = QTimer()
        self.__qtimer.timeout.connect(self.__update) # 计时器挂接到槽：update

        # 零点 I1, T2, T3, T4, I5
        self.__zero_pos_joints = pos_joint

        # 是否正方向(1:正;-1:反)I1, T2, T3, T4, I5
        self.__direction_joints = direction_joint

        self.__set_input_range()

        self.__display_joints_direction()
        self.__display_joints_pos()

    # 获取零点，机器人正方向
    def set_zero_point(self):

        self.get_actual_joint_point()
        self.__zero_pos_joints = [  round(float(str(self.lineEdit.text())),3),    \
                                    round(float(str(self.lineEdit_2.text())),3),  \
                                    round(float(str(self.lineEdit_3.text())),3),  \
                                    round(float(str(self.lineEdit_4.text())),3),  \
                                    round(float(str(self.lineEdit_5.text())),3)  ]

        # I1
        if self.checkBox.isChecked():
            self.__direction_joints[0] = 1
        else:
            self.__direction_joints[0] = -1
        # T2
        if self.checkBox_2.isChecked():
            self.__direction_joints[1] = 1
        else:
            self.__direction_joints[1] = -1
        # T3
        if self.checkBox_3.isChecked():
            self.__direction_joints[2] = 1
        else:
            self.__direction_joints[2] = -1
        # T4
        if self.checkBox_4.isChecked():
            self.__direction_joints[3] = 1
        else:
            self.__direction_joints[3] = -1
        # I5
        if self.checkBox_5.isChecked():
            self.__direction_joints[4] = 1
        else:
            self.__direction_joints[4] = -1

        self.sin_set_zero_point.emit([ self.__zero_pos_joints, \
                                        self.__direction_joints ])

    ################# 获取机器人当前关节值 ##################################
    def get_actual_joint_point(self):
        self.sin_get_actual_joints_pos.emit()
        self.__qtimer.start(100)    # 100ms

    def receive_actual_joint_point(self,data):
        self.__zero_pos_joints = data

    def __update(self):
        self.__display_joints_pos()
        self.__qtimer.stop()
    ################# 获取机器人当前关节值 end  #############################

    # 显示机器人关节数据
    def __display_joints_pos(self):
        self.lineEdit.setText(str(self.__zero_pos_joints[0]))
        self.lineEdit_2.setText(str(self.__zero_pos_joints[1]))
        self.lineEdit_3.setText(str(self.__zero_pos_joints[2]))
        self.lineEdit_4.setText(str(self.__zero_pos_joints[3]))
        self.lineEdit_5.setText(str(self.__zero_pos_joints[4]))
        pass

    # 显示机器人关节方向 
    def __display_joints_direction(self):

        # I1
        if self.__direction_joints[0] == 1:
            self.checkBox.setChecked(True)
        else:
            self.checkBox.setChecked(False)
        # T2
        if self.__direction_joints[1] == 1:
            self.checkBox_2.setChecked(True)
        else:
            self.checkBox_2.setChecked(False)
        # T3
        if self.__direction_joints[2] == 1:
            self.checkBox_3.setChecked(True)
        else:
            self.checkBox_3.setChecked(False)
        # T4
        if self.__direction_joints[3] == 1:
            self.checkBox_4.setChecked(True)
        else:
            self.checkBox_4.setChecked(False)
        # I5
        if self.__direction_joints[4] == 1:
            self.checkBox_5.setChecked(True)
        else:
            self.checkBox_5.setChecked(False)

    # 设置输入限制
    def __set_input_range(self):
        pdoubleValidator_I = QDoubleValidator(self)
        pdoubleValidator_I.setRange(-360,360)
        pdoubleValidator_I.setNotation(QDoubleValidator.StandardNotation)
        pdoubleValidator_I.setDecimals(2)
        self.lineEdit.setValidator(pdoubleValidator_I)
        self.lineEdit_5.setValidator(pdoubleValidator_I)

        pdoubleValidator_T = QDoubleValidator(self)
        pdoubleValidator_T.setRange(-120,120)
        pdoubleValidator_T.setNotation(QDoubleValidator.StandardNotation)
        pdoubleValidator_T.setDecimals(2)
        self.lineEdit_2.setValidator(pdoubleValidator_T)
        self.lineEdit_3.setValidator(pdoubleValidator_T)
        self.lineEdit_4.setValidator(pdoubleValidator_T)

    # 关闭界面
    def close_windows(self):
        self.close()

    def closeEvent(self, event):
        self.sin_close.emit()
        event.accept()