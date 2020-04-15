#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:Jony
@contact: 35024339@qq.com
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

class Robot_choice_func(QWidget,Ui_Form_robot_choice):

    def __init__(self):
        super(Robot_choice_func,self).__init__()
        self.setupUi(self)
        self.__center()

        self.pushButton.installEventFilter(self)
        self.pushButton_2.installEventFilter(self)

        # 需改成300x300大小
        self.__climbot5d_pic = QPixmap(RosPack().get_path('ui') + "/pic/climbot5d_.png")
        self.__biped5d_pic = QPixmap(RosPack().get_path('ui') + "/pic/biped5d_.png")
        self.setWindowIcon(QIcon(RosPack().get_path('ui') + "/pic/robot.ico"))

        self.label.setPixmap(self.__climbot5d_pic)

        pass
    
    # 双手爪爬杆机器人
    def climbot5d(self):
        # True:have gripper 
        self.__robot = Modular_robot_control_func(True)
        self.__robot.setWindowIcon(QIcon(RosPack().get_path('ui') + "/pic/robot.ico"))
        self.hide()
        self.__robot.show()
        pass
    
    # 双手爪爬臂机器人
    def biped5d(self):
        # False:no gripper 
        self.__robot = Modular_robot_control_func(False)
        self.__robot.setWindowIcon(QIcon(RosPack().get_path('ui') + "/pic/robot.ico"))
        self.hide()
        self.__robot.show()
        pass

    def __center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

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
