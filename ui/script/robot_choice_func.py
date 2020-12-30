#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: robot_choice_func.py

@biref: 机器人选择界面
"""
from rospkg import RosPack

from PyQt5.QtWidgets import QWidget, QDesktopWidget
from PyQt5.QtCore import pyqtSignal, QEvent
from PyQt5.QtGui import QPixmap, QIcon

from robot_choice import Ui_Form_robot_choice
from modular_robot_control_func import Modular_robot_control_func

import json

class Robot_choice_func(QWidget,Ui_Form_robot_choice):

    def __init__(self):
        super(Robot_choice_func,self).__init__()
        self.setupUi(self)
        self.__center()

        self.pushButton.installEventFilter(self)
        self.pushButton_2.installEventFilter(self)

        # 机器人图片需改成300x300大小
        self.__climbot5d_pic = QPixmap(RosPack().get_path('ui') + "/pic/climbot5d_.png")
        self.__biped5d_pic = QPixmap(RosPack().get_path('ui') + "/pic/biped5d_.png")
        self.setWindowIcon(QIcon(RosPack().get_path('ui') + "/pic/robot.ico"))
        # 机器人状态 : 关节位置， 原点位置， 原点方向
        self.__robot_state_file = RosPack().get_path('ui') + "/file/robot_state.json"
        self.__robot_state = {}

        self.label.setPixmap(self.__climbot5d_pic)

        pass
    
    # 双手爪爬杆机器人
    def climbot5d(self):
        # 0 : climbing robot 
        self.__robot = Modular_robot_control_func(0, self.__robot_state)
        self.__robot.setWindowIcon(QIcon(RosPack().get_path('ui') + "/pic/robot.ico"))
        self.hide()
        self.__robot.show()
        pass
    
    # 双手爪爬臂机器人
    def biped5d(self):
        # 1 : biped robot 
        self.__robot = Modular_robot_control_func(1, self.__robot_state)
        self.__robot.setWindowIcon(QIcon(RosPack().get_path('ui') + "/pic/robot.ico"))
        self.hide()
        self.__robot.show()
        pass
    
    # 界面放置在屏幕中心
    def __center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    # 判断机器人是否更新机器人断电状态
    def update_robot_state(self):
        if self.radioButton.isChecked():
            # 更新状态
            try:
                with open(self.__robot_state_file, 'r') as f:
                    self.__robot_state = json.load(f)
            except:
                    self.__robot_state = {}    
        else:
            pass
        pass


    # 机器人选择效果信号处理
    def eventFilter(self, object, event):

        if object == self.pushButton:
            if event.type() == QEvent.HoverMove:
                self.label.setPixmap(self.__climbot5d_pic)
                return True

        if object == self.pushButton_2:
            if event.type() == QEvent.HoverMove:
                self.label.setPixmap(self.__biped5d_pic)

                return True     

        return False
