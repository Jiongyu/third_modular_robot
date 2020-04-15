# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'path_point_recorder.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form_recoder_point(object):
    def setupUi(self, Form_recoder_point):
        Form_recoder_point.setObjectName("Form_recoder_point")
        Form_recoder_point.resize(400, 480)
        self.verticalLayoutWidget = QtWidgets.QWidget(Form_recoder_point)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(280, 20, 102, 191))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.pushButton_2 = QtWidgets.QPushButton(self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton_2.setFont(font)
        self.pushButton_2.setObjectName("pushButton_2")
        self.verticalLayout.addWidget(self.pushButton_2)
        self.pushButton = QtWidgets.QPushButton(self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton.setFont(font)
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout.addWidget(self.pushButton)
        self.pushButton_3 = QtWidgets.QPushButton(self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton_3.setFont(font)
        self.pushButton_3.setObjectName("pushButton_3")
        self.verticalLayout.addWidget(self.pushButton_3)
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(Form_recoder_point)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(280, 390, 102, 71))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.pushButton_6 = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton_6.setFont(font)
        self.pushButton_6.setObjectName("pushButton_6")
        self.verticalLayout_3.addWidget(self.pushButton_6)
        self.listWidget = QtWidgets.QListWidget(Form_recoder_point)
        self.listWidget.setGeometry(QtCore.QRect(20, 20, 256, 441))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.listWidget.setFont(font)
        self.listWidget.setObjectName("listWidget")

        self.retranslateUi(Form_recoder_point)
        self.pushButton_2.clicked.connect(Form_recoder_point.insert_point)
        self.pushButton.clicked.connect(Form_recoder_point.delete_point)
        self.pushButton_3.clicked.connect(Form_recoder_point.save_point)
        self.pushButton_6.clicked.connect(Form_recoder_point.clear_point)
        QtCore.QMetaObject.connectSlotsByName(Form_recoder_point)

    def retranslateUi(self, Form_recoder_point):
        _translate = QtCore.QCoreApplication.translate
        Form_recoder_point.setWindowTitle(_translate("Form_recoder_point", "示教记录"))
        self.pushButton_2.setText(_translate("Form_recoder_point", "插入点"))
        self.pushButton.setText(_translate("Form_recoder_point", "删除点"))
        self.pushButton_3.setText(_translate("Form_recoder_point", "保存"))
        self.pushButton_6.setText(_translate("Form_recoder_point", "清空"))

