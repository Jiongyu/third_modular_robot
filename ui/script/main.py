#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
from PyQt5.QtWidgets import QApplication
from robot_choice_func import Robot_choice_func

if __name__ == "__main__":

    app = QApplication(sys.argv)
    windows = Robot_choice_func() 
    windows.show()
    sys.exit(app.exec_())