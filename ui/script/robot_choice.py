# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'robot_choice.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form_robot_choice(object):
    def setupUi(self, Form_robot_choice):
        Form_robot_choice.setObjectName("Form_robot_choice")
        Form_robot_choice.resize(560, 339)
        self.widget = QtWidgets.QWidget(Form_robot_choice)
        self.widget.setGeometry(QtCore.QRect(360, 0, 180, 340))
        self.widget.setObjectName("widget")
        self.gridLayout = QtWidgets.QGridLayout(self.widget)
        self.gridLayout.setObjectName("gridLayout")
        self.pushButton_2 = QtWidgets.QPushButton(self.widget)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton_2.setFont(font)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout.addWidget(self.pushButton_2, 2, 0, 1, 1)
        self.pushButton = QtWidgets.QPushButton(self.widget)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton.setFont(font)
        self.pushButton.setObjectName("pushButton")
        self.gridLayout.addWidget(self.pushButton, 1, 0, 1, 1)
        self.widget1 = QtWidgets.QWidget(Form_robot_choice)
        self.widget1.setGeometry(QtCore.QRect(0, 0, 411, 341))
        self.widget1.setObjectName("widget1")
        self.label = QtWidgets.QLabel(self.widget1)
        self.label.setGeometry(QtCore.QRect(30, 18, 300, 300))
        self.label.setText("")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")

        self.retranslateUi(Form_robot_choice)
        self.pushButton.clicked.connect(Form_robot_choice.climbot5d)
        self.pushButton_2.clicked.connect(Form_robot_choice.biped5d)
        QtCore.QMetaObject.connectSlotsByName(Form_robot_choice)

    def retranslateUi(self, Form_robot_choice):
        _translate = QtCore.QCoreApplication.translate
        Form_robot_choice.setWindowTitle(_translate("Form_robot_choice", "机器人选择"))
        self.pushButton_2.setText(_translate("Form_robot_choice", "双手爪爬壁\n"
"机器人"))
        self.pushButton.setText(_translate("Form_robot_choice", "双手爪爬杆\n"
"机器人"))

