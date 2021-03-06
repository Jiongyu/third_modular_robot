#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: auto_gripper_get_grasp_point_thread.py

@biref:　获取夹持点
"""
from PyQt5.QtCore import QThread,pyqtSignal

import sys
from rospkg import RosPack
sys.path.append(RosPack().get_path('birl_module_robot') + "/script/")

from find_grasp_point_client import Find_grasp_point_client

from time import sleep

class Get_grasp_point_thread(QThread):

    # 逆解信号
    sin_grasp_point = pyqtSignal(list)

    def __init__(self, which_base, current_descartes_position, p1, p2):
        super(Get_grasp_point_thread, self).__init__()
        self.__which_base = which_base
        self.__current_descartes_position = current_descartes_position
        self.__p1 = p1
        self.__p2 = p2

        # print self.__current_descartes_position
        # print self.__p1
        # print self.__p2

    def run(self):
        ifgetSolve = False
        [grasp_point, ifgetSolve] = Find_grasp_point_client( self.__which_base, 
                                                                self.__current_descartes_position, self.__p1, self.__p2)
        temp = [grasp_point, ifgetSolve]
        if ifgetSolve:
            self.sin_grasp_point.emit(temp)
            sleep(1)
