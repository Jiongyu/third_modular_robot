#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: get_inverse_solution_thread.py

@biref:　获取运动学逆解
"""
from PyQt5.QtCore import QThread,pyqtSignal

import sys
from rospkg import RosPack
sys.path.append(RosPack().get_path('birl_module_robot') + "/script/")

from inverse_solution_client import Inverse_solution_client

from time import sleep

class Get_inverse_solution_thread(QThread):

    # 逆解信号    
    sin_inverse_solution = pyqtSignal(list)

    def __init__(self, which_robot, which_base, descartes_position_command, descartes_velocity_command, current_joint_position):
        super(Get_inverse_solution_thread, self).__init__()
        self.__which_robot = which_robot
        self.__which_base = which_base
        self.__descartes_position_command = descartes_position_command
        self.__descartes_velocity_command = descartes_velocity_command
        self.__current_joint_position = current_joint_position
        # print " init Get_inverse_solution_thread"

    def run(self):
        # print " run Get_inverse_solution_thread"
        # print self.__which_base
        # print self.__descartes_position_command
        # print self.__descartes_velocity_command
        # print self.__current_joint_position
        ifgetSolve = False
        [joint_pos_command, joint_vel_command, ifgetSolve] = Inverse_solution_client(   self.__which_robot, self.__which_base, \
                                                                            self.__descartes_position_command, \
                                                                            self.__descartes_velocity_command, \
                                                                            self.__current_joint_position)
        temp = [joint_pos_command, joint_vel_command]
        self.sin_inverse_solution.emit(temp)
        sleep(1)


