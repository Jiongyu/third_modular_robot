#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QFileDialog

class Path_point_recorder_func_thread(QThread):

    def __init__(self, data):
        super(Path_point_recorder_func_thread, self).__init__()
        self.__data = data
        pass

    def run(self):
        filename = QFileDialog.getSaveFileName(self,'保存为文件','./../','txt')
        if filename[0]:
            with open(filename[0],'w') as f:
                for i in range(self.__data.count()):
                    f.write(self.__data.item(i).text() + '\n')