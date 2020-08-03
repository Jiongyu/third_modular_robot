# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'receive_ros_command.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_receive_ros_command(object):
    def setupUi(self, receive_ros_command):
        receive_ros_command.setObjectName("receive_ros_command")
        receive_ros_command.resize(458, 318)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(receive_ros_command.sizePolicy().hasHeightForWidth())
        receive_ros_command.setSizePolicy(sizePolicy)
        receive_ros_command.setMinimumSize(QtCore.QSize(458, 318))
        receive_ros_command.setMaximumSize(QtCore.QSize(458, 318))
        self.listWidget = QtWidgets.QListWidget(receive_ros_command)
        self.listWidget.setGeometry(QtCore.QRect(20, 20, 281, 281))
        self.listWidget.setObjectName("listWidget")
        self.pushButton_6 = QtWidgets.QPushButton(receive_ros_command)
        self.pushButton_6.setGeometry(QtCore.QRect(320, 260, 121, 41))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.pushButton_6.setFont(font)
        self.pushButton_6.setObjectName("pushButton_6")
        self.pushButton_5 = QtWidgets.QPushButton(receive_ros_command)
        self.pushButton_5.setGeometry(QtCore.QRect(320, 140, 121, 41))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.pushButton_5.setFont(font)
        self.pushButton_5.setObjectName("pushButton_5")
        self.pushButton_7 = QtWidgets.QPushButton(receive_ros_command)
        self.pushButton_7.setGeometry(QtCore.QRect(320, 30, 121, 41))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.pushButton_7.setFont(font)
        self.pushButton_7.setObjectName("pushButton_7")

        self.retranslateUi(receive_ros_command)
        self.pushButton_6.clicked.connect(receive_ros_command.sent_ros_command)
        self.pushButton_7.clicked.connect(receive_ros_command.delete_path_point)
        self.pushButton_5.clicked.connect(receive_ros_command.clear_path_point)
        QtCore.QMetaObject.connectSlotsByName(receive_ros_command)

    def retranslateUi(self, receive_ros_command):
        _translate = QtCore.QCoreApplication.translate
        receive_ros_command.setWindowTitle(_translate("receive_ros_command", "接受ros控制命令"))
        self.pushButton_6.setText(_translate("receive_ros_command", "发送控制命令"))
        self.pushButton_5.setText(_translate("receive_ros_command", "清空"))
        self.pushButton_7.setText(_translate("receive_ros_command", "删除"))

