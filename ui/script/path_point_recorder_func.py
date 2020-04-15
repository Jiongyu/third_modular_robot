#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@version: python2.7
@author:Jony
@contact: 35024339@qq.com
@software: RoboWareStudio
@file: path_point_recorder_func.py

@biref: 示教记录关节点窗口
"""

from path_point_recorder import Ui_Form_recoder_point

from PyQt5.QtWidgets import QWidget, QDesktopWidget, QFileDialog
from PyQt5.QtCore import pyqtSignal,QTimer


class Path_point_recorder_func(QWidget, Ui_Form_recoder_point):

    # 获取当前关节信号
    sin_request_pos_joints = pyqtSignal()

    def __init__(self):
        super(Path_point_recorder_func, self).__init__()
        self.setupUi(self)

        # position I1, T2, T3, T4, I5
        self.__pos_joints = [0, 0, 0, 0, 0]

        # 延迟获取关节状态使用
        self.__timer = QTimer()
        self.__timer.timeout.connect(self.__update) # 计时器挂接到槽：update

    def insert_point(self):
        self.sin_request_pos_joints.emit()
        self.__timer.start(100) # 100ms

    def __update(self):
        # print self.__pos_joints
        self.listWidget.addItem('P='   + str(self.__pos_joints[0]) + ',' + str(self.__pos_joints[1]) + ',' \
                                        + str(self.__pos_joints[2]) + ',' + str(self.__pos_joints[3]) + ',' \
                                        + str(self.__pos_joints[4]) + ',;' )
        self.__timer.stop()

    def delete_point(self):
        self.listWidget.takeItem(self.listWidget.currentRow())

    def save_point(self):
        filename = QFileDialog.getSaveFileName(self,'保存为文件','./../','txt')
        if filename[0]:
            with open(filename[0],'w') as f:
                for i in range(self.listWidget.count()):
                    f.write(self.listWidget.item(i).text() + '\n')

    def clear_point(self):
        self.listWidget.clear()

    def receive_pos_joints(self,data):
        self.__pos_joints = data

    def close_windows(self):
        self.close()