#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: path_point_recorder_func.py

@biref: 示教记录关节点窗口
"""

from path_point_recorder import Ui_Form_recoder_point

from PyQt5.QtWidgets import QWidget, QDesktopWidget, QFileDialog
from PyQt5.QtCore import pyqtSignal,QTimer

from rospkg import RosPack

from os import path
import sys
reload(sys)
sys.setdefaultencoding('utf8')

class Path_point_recorder_func(QWidget, Ui_Form_recoder_point):

    # 获取当前关节信号
    sin_request_pos_joints = pyqtSignal()

    sin_close = pyqtSignal()

    def __init__(self):
        super(Path_point_recorder_func, self).__init__()
        self.setupUi(self)

        # position I1, T2, T3, T4, I5
        self.__pos_joints = [0, 0, 0, 0, 0]

        # 延迟获取关节状态使用
        self.__timer = QTimer()
        self.__timer.timeout.connect(self.__update) # 计时器挂接到槽：update

        self.__joint_enable = False

    # 插入关节点
    def insert_point(self):
        self.sin_request_pos_joints.emit()
        self.__timer.start(100) # 100ms

    def __update(self):
        # print self.__pos_joints
        self.listWidget.addItem('P='   + str(self.__pos_joints[0]) + ',' + str(self.__pos_joints[1]) + ',' \
                                        + str(self.__pos_joints[2]) + ',' + str(self.__pos_joints[3]) + ',' \
                                        + str(self.__pos_joints[4]) + ',;' )
        self.__timer.stop()

    # 删除点
    def delete_point(self):
        self.listWidget.takeItem(self.listWidget.currentRow())

    # 保存示教点数据
    def save_point(self):

        fileTitle = str(self.lineEdit.text())
        if fileTitle and (fileTitle != "文件已存在"):
            saveFile = RosPack().get_path('ui') + "/path_point_file/" + fileTitle + ".txt"
            if(path.exists(saveFile)):
                self.lineEdit.setText("文件已存在")
            else:
                with open(saveFile,'w') as f:
                    for i in range(self.listWidget.count()):
                        f.write(self.listWidget.item(i).text() + '\n')

    # 清除示教点数据
    def clear_point(self):
        self.listWidget.clear()

    # 更新关节数据
    def receive_pos_joints(self,data):
        self.__pos_joints = data

    # 关闭界面
    def close_windows(self):
        self.close()

    def closeEvent(self, event):
        self.sin_close.emit()
        event.accept()
