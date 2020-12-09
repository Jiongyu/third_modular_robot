#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: get_positive_solution_thread.py

@biref:　获取运动学正解
"""
from PyQt5.QtCore import QThread,pyqtSignal

import sys
from rospkg import RosPack
sys.path.append(RosPack().get_path('birl_module_robot') + "/script/")

from positive_solution_client import Positive_solution_client

from time import sleep

class Get_positive_solution_thread(QThread):

    # 正解信号    
    sin_positive_solution = pyqtSignal(list)

    def __init__(self):
        super(Get_positive_solution_thread, self).__init__()

        # print " init Get_positive_solution_thread"

    def update_joint_state(self, which_robot, which_base, current_joint_position):
        self.__which_robot = which_robot
        self.__which_base = which_base
        self.__current_joint_position = current_joint_position

    def run(self):
        # print " run Get_positive_solution_thread"
        # print self.__which_base
        # print self.__current_joint_position
        descartes_position = Positive_solution_client(self.__which_robot, self.__which_base, self.__current_joint_position)
        self.sin_positive_solution.emit(descartes_position)
        sleep(1)


