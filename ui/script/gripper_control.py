# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gripper_control.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form_gripper(object):
    def setupUi(self, Form_gripper):
        Form_gripper.setObjectName("Form_gripper")
        Form_gripper.resize(411, 259)
        font = QtGui.QFont()
        font.setPointSize(15)
        Form_gripper.setFont(font)
        self.gridLayoutWidget = QtWidgets.QWidget(Form_gripper)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(20, 100, 371, 141))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setObjectName("gridLayout")
        self.label_2 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1)
        self.pushButton = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton.setCheckable(True)
        self.pushButton.setObjectName("pushButton")
        self.gridLayout.addWidget(self.pushButton, 0, 1, 1, 1)
        self.pushButton_2 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_2.setCheckable(True)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout.addWidget(self.pushButton_2, 1, 1, 1, 1)
        self.label = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.pushButton_3 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_3.setCheckable(True)
        self.pushButton_3.setObjectName("pushButton_3")
        self.gridLayout.addWidget(self.pushButton_3, 0, 2, 1, 1)
        self.pushButton_4 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_4.setCheckable(True)
        self.pushButton_4.setObjectName("pushButton_4")
        self.gridLayout.addWidget(self.pushButton_4, 1, 2, 1, 1)
        self.horizontalLayoutWidget = QtWidgets.QWidget(Form_gripper)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(20, 20, 371, 81))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_3 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout.addWidget(self.label_3)
        self.lineEdit = QtWidgets.QLineEdit(self.horizontalLayoutWidget)
        self.lineEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit.setPlaceholderText("")
        self.lineEdit.setObjectName("lineEdit")
        self.horizontalLayout.addWidget(self.lineEdit)
        self.label_4 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout.addWidget(self.label_4)
        self.pushButton_5 = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.pushButton_5.setObjectName("pushButton_5")
        self.horizontalLayout.addWidget(self.pushButton_5)

        self.retranslateUi(Form_gripper)
        self.pushButton_5.clicked.connect(Form_gripper.gripper_torque_set)
        self.pushButton.clicked['bool'].connect(Form_gripper.gripper_0_open)
        self.pushButton_3.clicked['bool'].connect(Form_gripper.gripper_0_close)
        self.pushButton_2.clicked['bool'].connect(Form_gripper.gripper_6_open)
        self.pushButton_4.clicked['bool'].connect(Form_gripper.gripper_6_close)
        QtCore.QMetaObject.connectSlotsByName(Form_gripper)

    def retranslateUi(self, Form_gripper):
        _translate = QtCore.QCoreApplication.translate
        Form_gripper.setWindowTitle(_translate("Form_gripper", "夹持器控制"))
        self.label_2.setText(_translate("Form_gripper", "夹持器_G6"))
        self.pushButton.setText(_translate("Form_gripper", "开启"))
        self.pushButton_2.setText(_translate("Form_gripper", "开启"))
        self.label.setText(_translate("Form_gripper", "夹持器_G0"))
        self.pushButton_3.setText(_translate("Form_gripper", "关闭"))
        self.pushButton_4.setText(_translate("Form_gripper", "关闭"))
        self.label_3.setText(_translate("Form_gripper", "夹持器\n"
"力矩"))
        self.lineEdit.setToolTip(_translate("Form_gripper", "<html><head/><body><p>整数0~2000</p></body></html>"))
        self.label_4.setText(_translate("Form_gripper", "mN.m"))
        self.pushButton_5.setText(_translate("Form_gripper", "设置"))

