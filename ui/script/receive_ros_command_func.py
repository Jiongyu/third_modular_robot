#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: 
@file: receive_ros_command_func.py

@biref: 接收ros topic 控制窗口
"""

from PyQt5.QtWidgets import QWidget, QDesktopWidget
from PyQt5.QtCore import pyqtSignal,QMutex

from receive_ros_command import Ui_receive_ros_command
from receive_ros_command_thread import Receive_ros_command_thread

class Receive_ros_command_func(QWidget, Ui_receive_ros_command):

    # 发送控制命令信号
    sin_sent_ros_command = pyqtSignal(list)

    sin_close = pyqtSignal()

    def __init__(self):
        super(Receive_ros_command_func, self).__init__()
        self.setupUi(self)
        # 存储ros控制命令数据
        self.__pos_vel_commnad_data = []
        self.__ros_command_thread = Receive_ros_command_thread()
        self.__ros_command_thread.sin_sent_ros_command_thread.connect(self.__dislay_ros_command)
        self.__ros_command_thread.start()
        self.__index = 0

    # 显示接收的ros命令 
    def __dislay_ros_command(self, data):
        self.__pos_vel_commnad_data.append(data)

        self.listWidget.addItem(    'P' + str(self.__index) + " " \
                                    + str(data[0]) + ',' \
                                    + str(data[1]) + ',' \
                                    + str(data[2]) + ',' \
                                    + str(data[3]) + ',' \
                                    + str(data[4]) + ',;' )

        self.listWidget.addItem(    'V' + str(self.__index) + " " \
                                    + str(data[5]) + ',' \
                                    + str(data[6]) + ',' \
                                    + str(data[7]) + ',' \
                                    + str(data[8]) + ',' \
                                    + str(data[9]) + ',;' )  
        self.__index += 1         
    
    # 发送控制命令按钮槽函数
    def sent_ros_command(self):
        if self.__pos_vel_commnad_data:
            print self.__pos_vel_commnad_data
            self.__pos_vel_commnad_data = self.add_time_intervel(self.__pos_vel_commnad_data)
            print self.__pos_vel_commnad_data
            self.sin_sent_ros_command.emit(self.__pos_vel_commnad_data)
    
    # 删除路径点按钮槽函数
    def delete_path_point(self):
        if self.listWidget.count() == 0:
            return
        temp = self.listWidget.currentRow()

        del self.__pos_vel_commnad_data[temp / 2]
        
        # 删除位置速度点
        if temp % 2:
            self.listWidget.takeItem(temp - 1)
            self.listWidget.takeItem(temp - 1)

        else:
            self.listWidget.takeItem(temp + 1)
            self.listWidget.takeItem(temp)
    
    # 清空路径点按钮槽函数
    def clear_path_point(self):
        self.listWidget.clear()
        self.__pos_vel_commnad_data = []
        self.__index = 0

    def closeEvent(self, event):
        del self.__ros_command_thread
        self.sin_close.emit()
        event.accept()

    def add_time_intervel(self, data):
        return_data = data
        if return_data:
            for i in range(len(return_data) - 1):
                temp_time = 0
                for j in range(len(return_data[i]) / 2):
                    if ( abs(return_data[i][j + 5] / float(return_data[i][j])) >  temp_time):
                        temp_time = return_data[i][j + 5] / float(return_data[i][j])
                return_data[i + 1].append(temp_time)
            
            del return_data[0]
            return return_data
