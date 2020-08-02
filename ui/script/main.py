#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: mian.py

@biref:　界面入口
"""

import sys
from PyQt5.QtWidgets import QApplication
from robot_choice_func import Robot_choice_func

if __name__ == "__main__":

    app = QApplication(sys.argv)
    windows = Robot_choice_func() 
    windows.show()
    sys.exit(app.exec_())