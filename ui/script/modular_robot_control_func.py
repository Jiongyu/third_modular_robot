#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: modular_robot_control_func.py

@biref:上位机主窗口，包含关节控制，路径离线控制
"""
import rospy
from ui.msg import robot_feedback

from modular_robot_control import Ui_MainWindow_modular_robot
from path_point_recorder_func import Path_point_recorder_func
from gripper_control_func import Gripper_control_func
from zero_point_set_func import Zero_point_set_func
from robot_feedback_fun import Robot_feedback_fun
from receive_ros_command_func import Receive_ros_command_func
from robot_lowlevel_control_muti_thread import Robot_lowlevel_control_Muti_thread
from get_inverse_solution_thread import Get_inverse_solution_thread
from get_positive_solution_thread import Get_positive_solution_thread
from biped_receive_control_command_func import Biped_receive_control_command_func

from path_process import Path_process

from PyQt5.QtWidgets import QMainWindow, QDesktopWidget, QMessageBox, QFileDialog, QWidget
from PyQt5.QtCore import pyqtSignal, QThread
from PyQt5.QtGui import QDoubleValidator,QIntValidator

# from string import atof

class Modular_robot_control_func(QMainWindow,Ui_MainWindow_modular_robot):

    # 窗口传递信号
    sin_sent_data_to_WindowsPathRecoder = pyqtSignal(list)
    sin_sent_data_to_windowsSetZero     = pyqtSignal(list)

    # 关闭窗口信号
    sin_close_windowsPathRecoder    = pyqtSignal()
    sin_close_windowsSetZero        = pyqtSignal()
    sin_close_windowsGripper        = pyqtSignal()
    sin_close_windowsFeedback       = pyqtSignal()
    sin_close_windowsBipedReceive   = pyqtSignal()
    
    # 停止、急停、运行信号
    sin_stop_robot_operation            = pyqtSignal()
    sin_quickStop_robot_operation       = pyqtSignal()
    sin_halt_robot_operation            = pyqtSignal(bool)

    # 关节位置命令信号
    sin_joint_position = pyqtSignal(list)
    # 关节速度命令信号
    sin_joint_velocity = pyqtSignal(list)

    # 夹持器信号
    sin_G0_command = pyqtSignal(int)
    sin_G6_command = pyqtSignal(int)

    # 路径信号
    sin_path_command = pyqtSignal(list)
    # 离线轨迹继续运行信号
    sin_continue_path_command = pyqtSignal()

    # 反馈数据显示信号
    sin_display_feedback_data = pyqtSignal(list)
    # 更新爪子基座
    sin_update_gripper_base = pyqtSignal(bool)


    # 更新 爬壁机器人接收控制命令窗口 机器人状态
    sin_update_biped_robot_state = pyqtSignal(list)

    # 机器人各关节状态
    # position I1, T2, T3, T4, I5
    __pos_joints = [0, 0, 0, 0, 0]



    def  __init__(self,which_robot):
        super(Modular_robot_control_func,self).__init__()
        self.setupUi(self)

        self.__center()
        self.__input_range_limit()
        self.__lineEdit_default_Set()

        self.__string_robot_Enabled = "机器人\n使能成功"
        self.__string_robot_NotEnabled = "机器人\n未使能"
        self.__string_robot_Enable_ing = "机器人\n使能中"

        # 窗口
        self.__window_gripper_control = QWidget()
        self.__window_path_point_record = QWidget()
        self.__window_zero_point_set = QWidget()
        self.__window_robot_state_feedback = QWidget()
        self.__window_receive_ros_robot_command = QWidget()
        self.__window_biped_receive_command = QWidget()
        # 判断窗口是否打开
        self.__window_gripper_control_flag = False
        self.__window_path_point_record_flag = False
        self.__window_zero_point_set_flag = False
        self.__window_robot_state_feedback_flag = False
        self.__window_receive_ros_robot_command_flag = False
        self.__window_biped_receive_command_flag = False

        # 区别攀爬、爬壁
        self.__which_robot = which_robot

        # 判断机器人是否启动
        self.__robot_enabled_flag = False
        # 判断是否为机器人启动中，防止启动过程重复启动
        self.__robot_starting = False
        self.label.setText(self.__string_robot_NotEnabled)

        # 离线路径点集合
        self.__pos_joints_path_array = []
        self.__old_path_array = []

        # 离线路径速度
        self.__max_path_velocity = 5
        self.lineEdit.setText(str(self.__max_path_velocity))

        # 关节速度命令
        self.__joint_velocity = 3
        self.lineEdit_12.setText(str(self.__joint_velocity))
        self.lineEdit_23.setText(str(self.__joint_velocity))

        # 位置模式界面
        self.__joint_velocity_command_posMode = [0,0,0,0,0]
        # 速度模式界面
        self.__joint_velocity_command_VelMode = [0,0,0,0,0]

        # 关节位置命令
        self.__joint_position_command = [0,0,0,0,0]

        # 关节零点 I1, T2, T3, T4, I5
        self.__zero_pos_joints = [0,0,0,0,0]

        # 关节是否正方向(1:正;-1:反)I1, T2, T3, T4, I5
        self.__direction_joints = [1,1,1,1,1]

        # 默认机器人正逆运动学基座为G0
        self.__base_flag = True

        # 机器人反馈数据显示
        self.__robot_state_display_flag = False

        # 判断机器人是否通过ros反馈状态消息  
        self.__ros_feedback_flag = False

        # 机器人笛卡尔增量控制  位置增量 m
        self.__position_incre_positon = 0.001
        # 机器人笛卡尔增量控制  姿态增量 deg
        self.__posture_incre_position = 1
        # 机器人笛卡尔增量控制  位置姿态默认速度(deg/mm)
        # mm
        self.__position_incre_velocity = 1
        # deg
        self.__posture_incre_velocity = 1  

        #　机器人当前末端笛卡尔位置(position:mm, posture:deg)
        self.__actual_robot_tcp_pos = [586.4, 0, 0, 0, 0, 180]

        # 双爪电流状态记录
        self.__last_G0_cur = 0
        self.__last_G6_cur = 0


        # 菜单栏信号槽函数链接
        self.action.triggered.connect(self.__open_gripper_control)
        self.action_2.triggered.connect(self.__open_path_point_record)
        self.action_3.triggered.connect(self.__open_set_zero_point)
        self.action_11.triggered.connect(self.__open_robot_state_feedback)
        self.action_4.triggered.connect(self.__about)
        self.action_5.triggered.connect(self.__quit)
        self.action_8.triggered.connect(self.__auto_gripper_pole)
        self.action_6.triggered.connect(self.__ros_state_feedback_Set)
        self.action_10.triggered.connect(self.__about)
        self.action_12.triggered.connect(self.__open_ros_command)
        self.action_13.triggered.connect(self.__open_biped_receive_command)
        # end

        # ros 反馈初始化
        rospy.init_node("modular_robot_lowlevel_control", anonymous=True)
        self.__publisher = rospy.Publisher("/low_level/joints_point_feedback",robot_feedback,queue_size = 10)
        self.__ros_feedback_msg = robot_feedback()
        # 关节位置
        self.__ros_feedback_msg.feedbackPosData = [0 ,0 ,0 ,0 ,0]
        # 关节速度
        self.__ros_feedback_msg.feedbackVelData = [0 ,0 ,0 ,0 ,0]
        # 关节电流
        self.__ros_feedback_msg.feedbackCurrData = [0 ,0 ,0 ,0 ,0]
        # 双爪夹紧状态
        self.__ros_feedback_msg.isGrasping = [False, False]
        # end

        # 机器人笛卡尔空间增量求逆解服务声明
        self.__getInverseSolution_incre = QThread()
        # 机器人笛卡尔空间选点求逆服务
        self.__getInverseSolu = QThread()
        # 获取机器人当前tcp位置服务
        self.__getActualTcpPosSolu = QThread()

    # 窗口位于屏幕中心
    def __center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    # 启动机器人
    def start_robot(self):

        if (not self.__robot_enabled_flag) and (not self.__robot_starting):
            self.__robot_starting = True
            # 底层控制
            self.__lowLevelControl = Robot_lowlevel_control_Muti_thread(self.__which_robot)                
            self.__lowLevelControl.sin_robot_start_compelete.connect(self.__robot_start_compelete)
            self.__lowLevelControl.sin_robot_start_error.connect(self.start_robot_error)
            self.__lowLevelControl.sin_robot_stop_compelete.connect(self.__robot_stop_compelete)
            self.__lowLevelControl.sin_feedback.connect(self.__update_feeback_joint_position)

            self.sin_quickStop_robot_operation.connect(self.__lowLevelControl.robot_quick_stop)
            self.sin_stop_robot_operation.connect(self.__lowLevelControl.robot_stop)
            self.sin_halt_robot_operation.connect(self.__lowLevelControl.robot_halt)
            self.sin_joint_position.connect(self.__lowLevelControl.set_jointPosition)
            self.sin_joint_velocity.connect(self.__lowLevelControl.set_jointVelocity)
            self.sin_path_command.connect(self.__lowLevelControl.path_command)
            self.sin_continue_path_command.connect(self.__lowLevelControl.continue_path_command)

            self.label.setText(self.__string_robot_Enable_ing)
            if not self.__lowLevelControl.isRunning():
                self.__lowLevelControl.start()

    # 机器人启动错误信号槽函数
    def start_robot_error(self):
        self.__robot_enabled_flag = False
        self.__robot_starting = False
        del self.__lowLevelControl
        self.label.setText(self.__string_robot_NotEnabled)
        QMessageBox.about(self,'错误',"\n\n机器人启动失败,请检查线路连接是否正确.\n\n"   )

    # 机器人停止槽函数
    def stop_robot(self):
        if self.__robot_enabled_flag:
            self.sin_stop_robot_operation.emit()
        pass

    # 机器人急停槽函数
    def quick_stop(self):
        if self.__robot_enabled_flag:
            self.sin_quickStop_robot_operation.emit()
            pass
        pass

    # 机器人暂停槽函数
    def halt(self,data):
        if self.__robot_enabled_flag:
            self.sin_halt_robot_operation.emit(data)
        pass
    
    # 机器人启动成功反馈槽函数
    def __robot_start_compelete(self):
        # print "complete"
        self.__robot_enabled_flag = True
        self.label.setText(self.__string_robot_Enabled)

    # 机器人关闭成功反馈槽函数
    def __robot_stop_compelete(self):
        self.__robot_enabled_flag = False
        self.__robot_starting = False
        del self.__lowLevelControl
        self.label.setText(self.__string_robot_NotEnabled)

    #################### 关节空间位置控制界面 关节控制槽函数 ###################################################
    def joint_i1_command(self):
        if self.__robot_enabled_flag:  
            # print "i1_command"
            self.__joint_position_command[0] =  Modular_robot_control_func.__user_data_to_motor(   \
                                                float(str(self.lineEdit_7.text())), \
                                                self.__zero_pos_joints[0],  \
                                                self.__direction_joints[0]  )
            self.__joint_velocity_command_posMode[0] = self.__joint_velocity
            self.__joint_pos_command(self.__joint_position_command)

    def joint_t2_command(self):
        if self.__robot_enabled_flag:
            self.__joint_position_command[1] =  Modular_robot_control_func.__user_data_to_motor(   \
                                                float(str(self.lineEdit_8.text())), \
                                                self.__zero_pos_joints[1],  \
                                                self.__direction_joints[1]  )
            self.__joint_velocity_command_posMode[1] = self.__joint_velocity
            self.__joint_pos_command(self.__joint_position_command)

    def joint_t3_command(self):
        if self.__robot_enabled_flag:
            self.__joint_position_command[2] =  Modular_robot_control_func.__user_data_to_motor(   \
                                                float(str(self.lineEdit_9.text())), \
                                                self.__zero_pos_joints[2],  \
                                                self.__direction_joints[2]  )
            self.__joint_velocity_command_posMode[2] = self.__joint_velocity
            self.__joint_pos_command(self.__joint_position_command)

    def joint_t4_command(self):
        if self.__robot_enabled_flag:
            self.__joint_position_command[3] =  Modular_robot_control_func.__user_data_to_motor(   \
                                                float(str(self.lineEdit_10.text())), \
                                                self.__zero_pos_joints[3],  \
                                                self.__direction_joints[3]  )
            self.__joint_velocity_command_posMode[3] = self.__joint_velocity
            self.__joint_pos_command(self.__joint_position_command)

    def joint_i5_command(self):
        if self.__robot_enabled_flag:
            self.__joint_position_command[4] =  Modular_robot_control_func.__user_data_to_motor(   \
                                                float(str(self.lineEdit_11.text())), \
                                                self.__zero_pos_joints[4],  \
                                                self.__direction_joints[4]  )
            self.__joint_velocity_command_posMode[4] = self.__joint_velocity
            self.__joint_pos_command(self.__joint_position_command)

    def __joint_pos_command(self,data):
        self.sin_joint_position.emit([data, self.__joint_velocity_command_posMode])

    #################### 关节空间位置控制界面 关节控制槽函数 end ###################################################

    #################### 关节空间速度控制界面 关节控制槽函数 #######################################################
    def joint_i1_vel_command(self,data):
        if not data:
            self.__joint_velocity_command_VelMode[0] = 0
        else:
            self.__joint_velocity_command_VelMode[0] = self.__joint_velocity * self.__direction_joints[0]
        self.sin_joint_velocity.emit(self.__joint_velocity_command_VelMode)

    def joint_t2_vel_command(self,data):
        if not data:
            self.__joint_velocity_command_VelMode[1] = 0
        else:
            self.__joint_velocity_command_VelMode[1] = self.__joint_velocity * self.__direction_joints[1]
        self.sin_joint_velocity.emit(self.__joint_velocity_command_VelMode)
        # print self.__joint_velocity_command_VelMode

    def joint_t3_vel_command(self,data):
        if not data:
            self.__joint_velocity_command_VelMode[2] = 0
        else:
            self.__joint_velocity_command_VelMode[2] = self.__joint_velocity * self.__direction_joints[2]
        self.sin_joint_velocity.emit(self.__joint_velocity_command_VelMode)

    def joint_t4_vel_command(self,data):
        if not data:
            self.__joint_velocity_command_VelMode[3] = 0
        else:
            self.__joint_velocity_command_VelMode[3] = self.__joint_velocity * self.__direction_joints[3]
        self.sin_joint_velocity.emit(self.__joint_velocity_command_VelMode)

    def joint_i5_vel_command(self,data):
        if not data:
            self.__joint_velocity_command_VelMode[4] = 0
        else:
            self.__joint_velocity_command_VelMode[4] = self.__joint_velocity * self.__direction_joints[4]
        self.sin_joint_velocity.emit(self.__joint_velocity_command_VelMode)  
    #################### 关节空间速度控制界面 关节控制槽函数  end ###################################################

    #################### 笛卡尔位置增量控制界面 槽函数 ##############################################################
    # self.pushButton_64.clicked.connect(MainWindow_modular_robot.descartes_incre_pos_set)
    # 笛卡尔空间位置姿态增量设置
    def descartes_incre_pos_set(self):
        self.__position_incre_positon = abs(float(str(self.lineEdit_96.text())))
        self.__posture_incre_position = abs(float(str(self.lineEdit_95.text()))) 
        # print self.__position_incre_positon
        # print self.__posture_incre_position
        pass

    # 笛卡尔增量速度设置
    def descartes_incre_vel_set(self):
        self.__position_incre_velocity = abs(float(str(self.lineEdit_98.text())))
        self.__posture_incre_velocity = abs(float(str(self.lineEdit_97.text())))
        # print self.__position_incre_velocity
        # print self.__posture_incre_velocity 

    # x 增量 +
    def descartes_x_incre_add(self):
        self.__control_robot_by_incre_data(0, self.__position_incre_positon)
        pass

    # x 增量 -
    def descartes_x_incre_sub(self):
        self.__control_robot_by_incre_data(0, - self.__position_incre_positon)
        pass

    # y 增量 +
    def descartes_y_incre_add(self):
        self.__control_robot_by_incre_data(1, self.__position_incre_positon)
        pass

    # y 增量 -
    def descartes_y_incre_sub(self):
        self.__control_robot_by_incre_data(1, - self.__position_incre_positon)
        pass

    # z 增量 +
    def descartes_z_incre_add(self):
        self.__control_robot_by_incre_data(2, self.__position_incre_positon)
        pass
    
    # z 增量 -
    def descartes_z_incre_sub(self):
        self.__control_robot_by_incre_data(2, - self.__position_incre_positon)
        pass
    
    # rx 增量 +
    def descartes_rx_incre_add(self):
        self.__control_robot_by_incre_data(3, self.__posture_incre_position)

        pass
    
    # rx 增量 -
    def descartes_rx_incre_sub(self):
        self.__control_robot_by_incre_data(3, - self.__posture_incre_position)
        pass

    # ry 增量 +
    def descartes_ry_incre_add(self):
        self.__control_robot_by_incre_data(4, self.__posture_incre_position)
        pass
    
    # ry 增量 -
    def descartes_ry_incre_sub(self):
        self.__control_robot_by_incre_data(4, - self.__posture_incre_position)
        pass
    
    # rz 增量 +
    def descartes_rz_incre_add(self):
        self.__control_robot_by_incre_data(5, self.__posture_incre_position)
        pass
    
    # rz 增量 -
    def descartes_rz_incre_sub(self):
        self.__control_robot_by_incre_data(5, - self.__posture_incre_position)
        pass
    
    # 发送笛卡尔增量求解逆解
    def __control_robot_by_incre_data(self, index, increData):
        if not self.__getInverseSolution_incre.isRunning():
            self.__actual_robot_tcp_pos[index] += increData
            # 笛卡尔空间末端速度
            temp_vel_list = [0] * 6

            for i in range(len(temp_vel_list)):
                if(index == i):
                    # X Y Z
                    if(index >= 0 and index <= 2):
                        temp_vel_list[i] = self.__position_incre_velocity
                    # Rx Ry Rz 
                    else:
                        temp_vel_list[i]  = self.__posture_incre_velocity
                else:
                    temp_vel_list[i] = 0
            # print temp_vel_list

            self.__getInverseSolution_incre = Get_inverse_solution_thread(  self.__which_robot, \
                                                                            self.__base_flag, \
                                                                            self.__actual_robot_tcp_pos, \
                                                                            temp_vel_list, \
                                                                            self.__pos_joints)

            self.__getInverseSolution_incre.sin_inverse_solution.connect(self.__receive_and_sent_joint_command)
            self.__getInverseSolution_incre.start()
        pass

    def __receive_and_sent_joint_command(self,data):
        # print data
        self.__sent_joint_command_to_lowlevel(data[0], data[1])
        pass
    
    #################### 笛卡尔位置增量控制界面 槽函数  end #########################################################
       
    # 获取逆解槽函数
    def get_inverse_solution(self):
        
        temp_descartes_postion_command = []
        temp_descartes_postion_command.append(float(str(self.lineEdit_26.text())))  # X
        temp_descartes_postion_command.append(float(str(self.lineEdit_27.text())))  # Y
        temp_descartes_postion_command.append(float(str(self.lineEdit_28.text())))  # Z
        temp_descartes_postion_command.append(float(str(self.lineEdit_29.text())))  # RX
        temp_descartes_postion_command.append(float(str(self.lineEdit_30.text())))  # RY
        temp_descartes_postion_command.append(float(str(self.lineEdit_31.text())))  # RZ

        temp_descartes_velocity_command = []
        temp_descartes_velocity_command.append(float(str(self.lineEdit_34.text())))  # X
        temp_descartes_velocity_command.append(float(str(self.lineEdit_37.text())))  # Y
        temp_descartes_velocity_command.append(float(str(self.lineEdit_32.text())))  # Z
        temp_descartes_velocity_command.append(float(str(self.lineEdit_33.text())))  # RX
        temp_descartes_velocity_command.append(float(str(self.lineEdit_36.text())))  # RY
        temp_descartes_velocity_command.append(float(str(self.lineEdit_35.text())))  # RZ

        # print temp_descartes_postion_command
        # print temp_descartes_velocity_command
        # print self.__pos_joints
        # print self.__which_robot
        # print self.__base_flag

        # 初始化求逆解
        if not self.__getInverseSolu.isRunning():
            self.__getInverseSolu = Get_inverse_solution_thread(self.__which_robot, self.__base_flag, temp_descartes_postion_command, temp_descartes_velocity_command, self.__pos_joints)
            self.__getInverseSolu.sin_inverse_solution.connect(self.__show_the_inverse_solution)
            self.__getInverseSolu.start()

    # 显示运动学逆解结果
    def __show_the_inverse_solution(self,data):
        # joint position command
        self.lineEdit_58.setText(str(data[0][0]))       # I1 deg
        self.lineEdit_61.setText(str(data[0][1]))       # T2
        self.lineEdit_56.setText(str(data[0][2]))       # T3
        self.lineEdit_57.setText(str(data[0][3]))       # T4
        self.lineEdit_60.setText(str(data[0][4]))       # I5
        # joint velocity command 
        self.lineEdit_64.setText(str(data[1][0]))        # I1 deg/s
        self.lineEdit_67.setText(str(data[1][1]))        # T2
        self.lineEdit_62.setText(str(data[1][2]))        # T3
        self.lineEdit_63.setText(str(data[1][3]))        # T4
        self.lineEdit_66.setText(str(data[1][4]))        # I5
        pass

    # 运行运动学逆解结果
    def descartes_command(self):
        # print "descartes_command"
        temp_velocity_command = [   float(str(self.lineEdit_64.text())) * self.__direction_joints[0],   \
                                    float(str(self.lineEdit_67.text())) * self.__direction_joints[1],   \
                                    float(str(self.lineEdit_62.text())) * self.__direction_joints[2],   \
                                    float(str(self.lineEdit_63.text())) * self.__direction_joints[3],   \
                                    float(str(self.lineEdit_66.text())) * self.__direction_joints[4]    ]

        temp_pos_command = [    float(str(self.lineEdit_58.text())),    \
                                float(str(self.lineEdit_61.text())),    \
                                float(str(self.lineEdit_56.text())),    \
                                float(str(self.lineEdit_57.text())),    \
                                float(str(self.lineEdit_60.text()))     ]

        self.__sent_joint_command_to_lowlevel(temp_pos_command, temp_velocity_command)

    # 发送关节控制命令至底层
    def __sent_joint_command_to_lowlevel(self, postion, velocity):
        if self.__robot_enabled_flag:
            for i in range(len(postion)):
                postion[i] = Modular_robot_control_func.__user_data_to_motor(postion[i],self.__zero_pos_joints[i],self.__direction_joints[i])

            self.sin_joint_position.emit([postion, velocity])

    # 关节空间位置控制界面、速度控制界面速度设置
    def joint_velocity_set_posMode(self):
        self.__joint_velocity = abs(float(str(self.lineEdit_12.text())))

    def joint_velocity_set_velMode(self):
        self.__joint_velocity = float(str(self.lineEdit_23.text()))

    # 离线轨迹载入文件槽函数
    def load_point_file(self):

        # file_path = QFileDialog.getOpenFileName(self, "载入文件", "~/")
        # if file_path[0]:
        #     f = open(file_path[0], "r")
        #     data = f.read()
        #     f.close()
        #     self.textEdit.setText(data)

        self.__pos_joints_path_array = []
        temp_data = self.textEdit.toPlainText()
        temp_path_process = Path_process(self.__zero_pos_joints, self.__direction_joints, self.__max_path_velocity)
        try:
            self.__pos_joints_path_array = temp_path_process.get_trajectory(temp_data) 
            QMessageBox.about(self,'通知','\n       路径点载入成功!!        \n')
        except:
            QMessageBox.about(self,'错误','\n       路径点格式错误!!        \n')
            return 
        # print self.__pos_joints_path_array

    # 设置离线轨迹最大速度
    def set_path_max_velovity(self):
        self.__max_path_velocity = int(str(self.lineEdit.text()))
        # print self.__max_path_velocity

    # 运行离线轨迹槽函数
    def operate_point_data(self):

        if self.__robot_enabled_flag:
            if self.__pos_joints_path_array != self.__old_path_array:
                self.__old_path_array = self.__pos_joints_path_array
                self.sin_path_command.emit(self.__pos_joints_path_array)

    # 继续运行离线轨迹
    def continue_operate_point_data(self):
        if self.__robot_enabled_flag:
            self.sin_continue_path_command.emit()
        pass

    # 打开夹持器控制界面
    def __open_gripper_control(self):

        if self.__which_robot == 0:
            if not self.__window_gripper_control_flag:
                self.__window_gripper_control_flag = True
                self.__window_gripper_control = Gripper_control_func()
                temp = self.frameGeometry()
                self.__window_gripper_control.move(temp.right(),temp.top())
                
                self.__window_gripper_control.sin_open_or_close_gripper0.connect(self.__get_G0_torque_from_windowsGripperControl)
                self.__window_gripper_control.sin_open_or_close_gripper6.connect(self.__get_G6_torque_from_windowsGripperControl)
                self.__window_gripper_control.sin_close.connect(self.__open_gripper_control_close_flag)

                self.sin_close_windowsGripper.connect(self.__window_gripper_control.close_windows)
                
                if self.__robot_enabled_flag:
                    self.sin_G0_command.connect(self.__lowLevelControl.G0_command)
                    self.sin_G6_command.connect(self.__lowLevelControl.G6_command)
                    self.__window_gripper_control.sin_open_gripper_feedback.connect(self.__lowLevelControl.open_gripper_feedback)
                    
                    self.__lowLevelControl.sin_gripper_feedback.connect(self.__window_gripper_control.display)
                self.__window_gripper_control.show()
                self.__window_gripper_control.open_gripper_current_feedback()

    def __open_gripper_control_close_flag(self):
        self.__window_gripper_control_flag = False

    # 打开示教点记录界面
    def __open_path_point_record(self):
        if not self.__window_path_point_record_flag:
            self.__window_path_point_record_flag = True
            self.__window_path_point_record = Path_point_recorder_func()
            temp = self.frameGeometry()
            temp_height = self.__window_path_point_record.height()
            self.__window_path_point_record.move(temp.right(), temp.top() + temp_height)
            
            self.__window_path_point_record.sin_request_pos_joints.connect(self.__sent_data_to_windowsPathRecorder)
            self.__window_path_point_record.sin_close.connect(self.__open_path_point_record_close_flag)
            self.sin_sent_data_to_WindowsPathRecoder.connect(self.__window_path_point_record.receive_pos_joints)
            self.sin_close_windowsPathRecoder.connect(self.__window_path_point_record.close_windows)
            self.__window_path_point_record.show()

    def __open_path_point_record_close_flag(self):
        self.__window_path_point_record_flag = False

    # 打开零点设置界面
    def __open_set_zero_point(self):
        if not self.__window_zero_point_set_flag:
            self.__window_zero_point_set_flag = True
            self.__window_zero_point_set = Zero_point_set_func(self.__zero_pos_joints, self.__direction_joints)
            temp = self.frameGeometry()
            temp_width = self.__window_zero_point_set.width()
            self.__window_zero_point_set.move(temp.left() - temp_width, temp.top())
            self.__window_zero_point_set.sin_close.connect(self.__open_set_zero_point_close_flag)
            self.__window_zero_point_set.sin_set_zero_point.connect(self.__get_data_from_windowsSetZeroPoint)
            self.__window_zero_point_set.sin_get_actual_joints_pos.connect(self.__sent_data_to_windowsSetZero)
            self.sin_sent_data_to_windowsSetZero.connect(self.__window_zero_point_set.receive_actual_joint_point)
            self.sin_close_windowsSetZero.connect(self.__window_zero_point_set.close_windows)
            self.__window_zero_point_set.show()
            pass

    def __open_set_zero_point_close_flag(self):
        self.__window_zero_point_set_flag = False
    
    # 打开机器人状态反馈界面
    def __open_robot_state_feedback(self):

        if not self.__window_robot_state_feedback_flag:
            self.__window_robot_state_feedback_flag = True
            self.__window_robot_state_feedback = Robot_feedback_fun(self.__which_robot)
            if self.__robot_enabled_flag:
                self.sin_display_feedback_data.connect(self.__window_robot_state_feedback.display)
                self.sin_update_gripper_base.connect(self.__window_robot_state_feedback.update_gripper)
                self.__window_robot_state_feedback.sin_open_robot_state_feedback.connect(self.__open_robot_state_feedback_signal)
            self.__window_robot_state_feedback.sin_close.connect(self.__open_robot_state_feedback_close_flag)
            self.sin_close_windowsFeedback.connect(self.__window_robot_state_feedback.close_windows)
            temp = self.frameGeometry()
            temp_width = self.__window_robot_state_feedback.width()
            temp_height = self.__window_robot_state_feedback.height()
            self.__window_robot_state_feedback.move(temp.left() - temp_width, temp.top() + temp_height)
            self.__window_robot_state_feedback.show()
            self.__window_robot_state_feedback.open_robot_state_feedback()
    
    def __open_robot_state_feedback_close_flag(self):
        self.__window_robot_state_feedback_flag = False

    # 打开 接收ros command 界面
    def __open_ros_command(self):

        if not self.__window_receive_ros_robot_command_flag:
            self.__window_receive_ros_robot_command_flag = True
            self.__window_receive_ros_robot_command = Receive_ros_command_func()
            self.__window_receive_ros_robot_command.sin_sent_ros_command.connect(self.__get_ros_command_data)
            self.__window_receive_ros_robot_command.sin_close.connect(self.__open_ros_command_close_flag)

            temp = self.frameGeometry()
            self.__window_receive_ros_robot_command.move(temp.right(),temp.top())
            self.__window_receive_ros_robot_command.show()

    def __open_ros_command_close_flag(self):
        self.__window_receive_ros_robot_command_flag = False

    # 获取ros command， 发送至底层
    def __get_ros_command_data(self, data):
        self.__pos_joints_path_array = data
        if self.__old_path_array != self.__pos_joints_path_array:
            self.__old_path_array = self.__pos_joints_path_array
            self.sin_path_command.emit(self.__pos_joints_path_array)
        pass

    # 打开机器人状态反馈界面，开始显示反馈数据
    def __open_robot_state_feedback_signal(self, data):
        self.__robot_state_display_flag = data

    # 自主抓夹
    def __auto_gripper_pole(self):       
        pass
    
    ####################### 爬壁机器人接收控制命令窗口 ###########################
    # 爬壁机器人接收控制命令
    def __open_biped_receive_command(self):
        if not self.__window_biped_receive_command_flag:
            self.__window_biped_receive_command_flag = True
            # 爬壁机器人
            if self.__which_robot == 1:
                self.__window_biped_receive_command = Biped_receive_control_command_func(  self.__which_robot, self.__base_flag, \
                                                                                    self.__pos_joints, self.__direction_joints, \
                                                                                    self.__zero_pos_joints )
                self.__window_biped_receive_command.sin_joint_position.connect(self.__biped_sent_command)
                self.__window_biped_receive_command.sin_close.connect(self.__open_biped_receive_command_close_flag)
                self.__window_biped_receive_command.sin_update_robot_base_flag.connect(self.__update_robot_base_flag)
                self.__window_biped_receive_command.sin_request_update_robot_state.connect(self.__response_update_robot_state)
                self.sin_close_windowsBipedReceive.connect(self.__window_biped_receive_command.close_windows)
                self.sin_update_biped_robot_state.connect(self.__window_biped_receive_command.update_robot_state)
                self.__window_biped_receive_command.show()
                pass
            pass

    def __open_biped_receive_command_close_flag(self):
        self.__window_biped_receive_command_flag = False

    # 更新机器人基坐标base
    def __update_robot_base_flag(self, flag):
        self.__base_flag = flag

    # 爬壁机器人接收控制命令 发送至底层
    def __biped_sent_command(self, data):
        # 爬壁机器人
        if self.__which_robot == 1 and self.__robot_enabled_flag:
            self.sin_joint_position.emit(data[0], data[1])
            pass
        pass
    
    # 更新 爬壁机器人接收控制命令窗口 机器人状态
    def __response_update_robot_state(self):
        self.sin_update_biped_robot_state.emit([    \
            self.__zero_pos_joints,
            self.__direction_joints,
            self.__pos_joints
        ])
        pass
    ####################### 爬壁机器人接收控制命令窗口 end #########################
    
    ####################### 获取夹持器界面控制数据，并发送至底层 ###########################
    def __get_G0_torque_from_windowsGripperControl(self,data):
        if self.__robot_enabled_flag:
            self.sin_G0_command.emit(data)
            if (self.__last_G0_cur == 0) & (data < 0):
                self.__ros_feedback_msg.isGrasping[0] = True
            if (self.__last_G0_cur > 0) & (data == 0):
                self.__ros_feedback_msg.isGrasping[0] = False
            self.__last_G0_cur = data

    def __get_G6_torque_from_windowsGripperControl(self,data):
        if self.__robot_enabled_flag:
            self.sin_G6_command.emit(data)
            if (self.__last_G6_cur == 0) & (data < 0):
                self.__ros_feedback_msg.isGrasping[1] = True
            if (self.__last_G6_cur > 0) & (data == 0):
                self.__ros_feedback_msg.isGrasping[1] = False
            self.__last_G6_cur = data
    ####################### 获取夹持器界面控制数据，并发送至底层 end ########################

    # 界面退出
    def __quit(self):

        self.sin_close_windowsGripper.emit()
        self.sin_close_windowsPathRecoder.emit()
        self.sin_close_windowsSetZero.emit()
        self.sin_close_windowsFeedback.emit()
        self.sin_close_windowsBipedReceive.emit()

        self.close()
        pass
    
    def __about(self):
        QMessageBox.about(self,'关于',"模块化机器人上位机 V1.0.    \
                                     \n\nAuthor:           \
                                      \n\nMail:")
    
    # 更新关节数据 至示教记录界面
    def __sent_data_to_windowsPathRecorder(self):
        self.sin_sent_data_to_WindowsPathRecoder.emit(self.__pos_joints)

    # 更新关节数据 至零点设置界面
    def __sent_data_to_windowsSetZero(self):
        self.sin_sent_data_to_windowsSetZero.emit(self.__pos_joints)
    
    # 获取零点，更新零点
    def __get_data_from_windowsSetZeroPoint(self,data):
        
        # 判断是否为手动修改零点值
        threshold = 0.1
        for i in range(len(self.__pos_joints)):
            if(abs(self.__pos_joints[i] - data[0][i]) > threshold):
                data[0][i] = - data[0][i]
        # end

        for i in range(len(data[0])):
            self.__direction_joints[i] = data[1][i]
            if self.__direction_joints[i] == 1 or self.__zero_pos_joints[i] == 0:
                self.__zero_pos_joints[i] += data[0][i]
            else:
                self.__zero_pos_joints[i] -= data[0][i]      

    # 根据零点、关节方向， 计算关节命令
    @staticmethod
    def __user_data_to_motor(command_pos, zero_pos, direction):
        """
        brief:  关节方向 * 用户值 + 零点   
        """
        return round((direction * command_pos + zero_pos) , 3)

    # 更新机器人关节状态
    def __update_feeback_joint_position(self,data):

        for i in range(len(self.__pos_joints)):
            self.__pos_joints[i] = self.__direction_joints[i] * ( data[0][i] - self.__zero_pos_joints[i] )

        if self.__robot_state_display_flag:
            # 关节位置、速度、电流
            self.sin_display_feedback_data.emit([self.__pos_joints, data[1], data[2]])
        
        if self.__ros_feedback_flag:
            self.__ros_feedback_msg.feedbackPosData = self.__pos_joints
            self.__ros_feedback_msg.feedbackVelData = data[1]
            self.__ros_feedback_msg.feedbackCurrData = data[2]
            self.__ros_feedback_msg.timeHeader.stamp = rospy.Time.now()
            self.__publisher.publish(self.__ros_feedback_msg)

    # 输入框　输入限制
    def __input_range_limit(self):
        
        # 离线轨迹最大速度限制
        pIntValidator = QIntValidator(self)
        pIntValidator.setRange(1,30)
        self.lineEdit.setValidator(pIntValidator)

        # 笛卡尔增量控制模式
        pDoubleDescartesIncre = QDoubleValidator(self)
        pDoubleDescartesIncre.setRange(0,10)
        pDoubleDescartesIncre.setNotation(QDoubleValidator.StandardNotation)
        pDoubleDescartesIncre.setDecimals(2)
        self.lineEdit_95.setValidator(pDoubleDescartesIncre)
        self.lineEdit_96.setValidator(pDoubleDescartesIncre)

        self.lineEdit_98.setValidator(pDoubleDescartesIncre)
        self.lineEdit_97.setValidator(pDoubleDescartesIncre)    
        
        pDoubleValidator_I = QDoubleValidator(self)
        pDoubleValidator_I.setRange(-360, 360)
        pDoubleValidator_I.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_I.setDecimals(2)
        # I1,I5 位置范围
        self.lineEdit_7.setValidator(pDoubleValidator_I)
        self.lineEdit_11.setValidator(pDoubleValidator_I)

        pDoubleValidator_T = QDoubleValidator(self)
        pDoubleValidator_T.setRange(-120, 120)
        pDoubleValidator_T.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_T.setDecimals(2)
        # T2 T3 T4 位置范围 
        self.lineEdit_8.setValidator(pDoubleValidator_T)
        self.lineEdit_9.setValidator(pDoubleValidator_T)
        self.lineEdit_10.setValidator(pDoubleValidator_T)

        pDoubleValidator_V = QDoubleValidator(self)
        pDoubleValidator_V.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_V.setDecimals(2)
        # 关节速度范围
        self.lineEdit_12.setValidator(pDoubleValidator_V)
        pDoubleValidator_V.setRange(-30, 30)
        self.lineEdit_23.setValidator(pDoubleValidator_V)

        # 笛卡尔关节控制输入限制
        pDoubleValidator_descartes_pos_XYZ = QDoubleValidator(self)
        pDoubleValidator_descartes_pos_XYZ.setRange(-1000,1000);
        pDoubleValidator_descartes_pos_XYZ.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_descartes_pos_XYZ.setDecimals(2)
        self.lineEdit_26.setValidator(pDoubleValidator_descartes_pos_XYZ)
        self.lineEdit_27.setValidator(pDoubleValidator_descartes_pos_XYZ)
        self.lineEdit_28.setValidator(pDoubleValidator_descartes_pos_XYZ)

        pDoubleValidator_descartes_pos_RXRYRZ = QDoubleValidator(self)
        pDoubleValidator_descartes_pos_RXRYRZ.setRange(-180,180)
        pDoubleValidator_descartes_pos_RXRYRZ.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_descartes_pos_RXRYRZ.setDecimals(2)
        self.lineEdit_29.setValidator(pDoubleValidator_descartes_pos_RXRYRZ)
        self.lineEdit_30.setValidator(pDoubleValidator_descartes_pos_RXRYRZ)
        self.lineEdit_31.setValidator(pDoubleValidator_descartes_pos_RXRYRZ)

        pDoubleValidator_descartes_vel_XYZ = QDoubleValidator(self)
        pDoubleValidator_descartes_vel_XYZ.setRange(-100,100) 
        pDoubleValidator_descartes_vel_XYZ.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_descartes_vel_XYZ.setDecimals(2)
        self.lineEdit_34.setValidator(pDoubleValidator_descartes_vel_XYZ)
        self.lineEdit_37.setValidator(pDoubleValidator_descartes_vel_XYZ)
        self.lineEdit_32.setValidator(pDoubleValidator_descartes_vel_XYZ)

        pDoubleValidator_descartes_vel_RXRYRZ = QDoubleValidator(self)
        pDoubleValidator_descartes_vel_RXRYRZ.setRange(-30,30)
        pDoubleValidator_descartes_vel_RXRYRZ.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_descartes_vel_RXRYRZ.setDecimals(2)
        self.lineEdit_33.setValidator(pDoubleValidator_descartes_vel_RXRYRZ)
        self.lineEdit_36.setValidator(pDoubleValidator_descartes_vel_RXRYRZ)
        self.lineEdit_35.setValidator(pDoubleValidator_descartes_vel_RXRYRZ)

    # 界面突出处理信号
    def closeEvent(self, event):
        result = QMessageBox.question(self, "模块化机器人上位机", '\n       退出?        \n', QMessageBox.Yes | QMessageBox.No)
        if(result == QMessageBox.Yes):
            self.sin_close_windowsGripper.emit()
            self.sin_close_windowsPathRecoder.emit()
            self.sin_close_windowsSetZero.emit()
            self.sin_close_windowsFeedback.emit()
            self.sin_close_windowsBipedReceive.emit()
            if self.__string_robot_Enabled:
                self.sin_stop_robot_operation.emit()
            event.accept()
        else:
            event.ignore()

    ###############  ｒｏｓ设置  #######################################

    def __ros_state_feedback_Set(self):
        if(self.action_6.isChecked()):
            self.__ros_feedback_flag = True
            # print self.__ros_feedback_flag
        else:
            self.__ros_feedback_flag = False
            # print self.__ros_feedback_flag

    ###############  检测需发送ｒｏｓ消息类型  end #######################################

    def __lineEdit_default_Set(self):
        # 笛卡尔空间控制界面　默认输入设置
        self.lineEdit_26.setText(str(586.4))    # X mm
        self.lineEdit_27.setText(str(0))        # Y
        self.lineEdit_28.setText(str(0))        # Z 
        self.lineEdit_29.setText(str(0))        # RX deg
        self.lineEdit_30.setText(str(0))        # RY
        self.lineEdit_31.setText(str(180))      # RZ

        self.lineEdit_34.setText(str(0))        # X mm/s
        self.lineEdit_37.setText(str(0))        # Y
        self.lineEdit_32.setText(str(0))        # Z 
        self.lineEdit_33.setText(str(0))        # RX deg/s
        self.lineEdit_36.setText(str(0))        # RY
        self.lineEdit_35.setText(str(0))        # RZ

        # 关节空间默认值
        self.lineEdit_7.setText(str(0))
        self.lineEdit_8.setText(str(0))
        self.lineEdit_9.setText(str(0))
        self.lineEdit_10.setText(str(0))
        self.lineEdit_11.setText(str(0))
        
        self.lineEdit_12.setText(str(5))

        self.lineEdit_23.setText(str(5))

    ############## 更新逆运动学基座 ####################################################
    def set_base_0(self):
        self.__base_flag = True
        self.radioButton_2.setChecked(False)
        self.radioButton_4.setChecked(False)
        self.sin_update_gripper_base.emit(self.__base_flag)

        pass

    def set_base_6(self):
        self.__base_flag = False
        self.radioButton.setChecked(False)
        self.radioButton_3.setChecked(False)
        self.sin_update_gripper_base.emit(self.__base_flag)
        pass
    ############## 更新逆运动学基座  end ###############################################

    # 请求获取机器人当前tcp位姿
    def __request_upadte_tcp_position(self):
        if not self.__getActualTcpPosSolu.isRunning():
            self.__getActualTcpPosSolu = Get_positive_solution_thread()
            self.__getActualTcpPosSolu.update_joint_state(self.__which_robot, self.__base_flag, self.__pos_joints)
            self.__getActualTcpPosSolu.sin_positive_solution.connect(self.__upadte_tcp_position)
            self.__getActualTcpPosSolu.start()
            pass

    # 更新机器人当前tcp
    def __upadte_tcp_position(self,data):
        # 笛卡尔位置控制更新更新tcp
        self.lineEdit_26.setText(str(round(data[0], 3)))
        self.lineEdit_27.setText(str(round(data[1], 3)))
        self.lineEdit_28.setText(str(round(data[2], 3)))
        self.lineEdit_29.setText(str(round(data[3], 3)))
        self.lineEdit_30.setText(str(round(data[4], 3)))
        self.lineEdit_31.setText(str(round(data[5], 3)))

        # 笛卡尔增量控制界面更新tcp
        self.__actual_robot_tcp_pos = data
        pass

    # tab 页数改变
    def tab_change(self, data):
        if data == 0:
            # 关节位置控制
            pass
        elif data == 1:
            # 关节速度控制
            pass
        elif data == 2:
            # 离线数据控制
            pass
        elif data == 3:
            # 笛卡尔位置控制
            self.__request_upadte_tcp_position()
            pass
        elif data == 4:
            # 笛卡尔增量控制
            self.__request_upadte_tcp_position()
            pass
        pass