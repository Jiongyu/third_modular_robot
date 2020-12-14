#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: get_fifo_relative_position_thread.py

@biref:　通过命名管道获取相对位置数据

@note: 如果会出现找不到管道路径，可是试试看是不是.vscode的setting.json为c++的，而没有python的配置
"""

#  在read/write fifo 时，需要先刷新缓存区(`http://www.cocoachina.com/cms/wap.php?action=article&id=79893)
# python中，以双下划线"__"开头的函数和变量，是作为该类成员的私有成员，不能进行外部调用(https://www.cnblogs.com/navysummer/p/10438779.html)
from PyQt5.QtCore import QThread,pyqtSignal,QMutex
import os, sys
from rospkg import RosPack
from get_positive_solution_thread import Get_positive_solution_thread
sys.path.append(RosPack().get_path('birl_module_robot') + "/script/")
from inverse_solution_client import Inverse_solution_client
from positive_solution_client import Positive_solution_client
from robot_increTransTool_client import robot_increTransTool_client
import numpy as np
import math

#from modular_robot_control_func import Modular_robot_control_func 

_PATH_NAME_READ_  = "/tmp/file.in"
_PATH_NAME_WRITE_ = "/tmp/file.out"
_PATH_NAME_SEND_ = "/tmp/file.send"

class Get_fifo_relative_position_thread(QThread):

    global relative_pose
    global res_read

    relative_pose = [0,0,0,0,0,0]  # 初始化存储数组
    res_read = 0
    # 接受相对关节位姿数据(单位： mm / degree)
    receive_relative__joint_position_data  = pyqtSignal(list)

    receive_end_tag = pyqtSignal(str)


    # 错误转换反馈
    #wrong_translation_feedback = pyqtSignal()

    def __init__(self):
        super(Get_fifo_relative_position_thread, self).__init__()
        #self.feedback_translation_tag = feedback_tag
        pass
          
    
    def run(self):
        
        self.get_fifo_relative_position()

        pass


    # 通过管道获取相对位姿
    def get_fifo_relative_position(self):

        if os.path.exists(_PATH_NAME_READ_):  # 重新启动管道
           pass
        else:
            os.mkfifo(_PATH_NAME_READ_)

        res_read = os.open(_PATH_NAME_READ_, os.O_RDONLY)  # 
        print "enter fifo_get"
        if res_read < 0:
            print "open res_read_fifo fail"
        else:
            num = 0
            while (num < 6):
                
                cache_data = os.read(res_read, 8)  #缓冲区长度(8位字符)
                relative_pose[num] = float(cache_data)  # 将缓冲区字符串转成 float
                print "received the relative_position in fifo: %f" % relative_pose[num]
                num = num + 1
            
                if len(relative_pose) == 0:
                    print "The fifo now without any data."
                    break
           
            print "end of fifo_get"
            return relative_pose 
                
        os.close(res_read)

        receive_tag = True            
        self.receive_relative__joint_position_data.emit(relative_pose)
        self.receive_end_tag.emit(receive_tag)


class Translate_goal_to_base_thread(QThread):

    relative_pose = [0,0,0,0,0,0]  # 初始化存储数组
    res_read = 0
    # 接受相对关节位姿数据(单位： mm / degree)
    receive_relative__joint_position_data  = pyqtSignal(list)

    goal_to_base_position_ = pyqtSignal(list)

    # 错误转换反馈
    wrong_translation_feedback = pyqtSignal(str)

    # 逆解后机器人的关节位置+速度命令
    goal_to_base_motor_pos_command = pyqtSignal(list, list)

    now_base_flag = pyqtSignal(bool)


    def __init__(self, __which_robot, __base_flag, __pos_joints):
        super(Translate_goal_to_base_thread, self).__init__()
        self._now_robot_pasitive_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._goal_to_base_position_data  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.descartes_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._get_relative_position_data  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # 笛卡尔坐标系的速度命令
        self.descartes_velocity_set_data = [10, 10, 10, 1, 3, 1]
      
        self.__which_robot = __which_robot
        self.__base_flag = __base_flag
        self.__pos_joints = __pos_joints
        self.receive_tag = "False"
        self.receive_base_flag = ""

    def run(self):
        
        self.receive_tag = "False"
        # 接收数据线程
        self.get_fifo_relative_position()
        # 接收数据成功才执行转换与运动
        if self.receive_tag == "True":
            self.get_goal_to_base_position_and_generate_joint_command_fun()
        pass

    # 通过管道获取相对位姿
    def get_fifo_relative_position(self):

        if os.path.exists(_PATH_NAME_READ_):  # 重新启动管道
           pass
        else:
            os.mkfifo(_PATH_NAME_READ_)

        res_read = os.open(_PATH_NAME_READ_, os.O_RDONLY)  # 

        if res_read < 0:
            print "open res_read_fifo fail"
        else:
            num = 0
            while (num < 6):
                cache_data = os.read(res_read, 8)  #缓冲区长度(8位字符)
                self._get_relative_position_data[num] = float(cache_data)  # 将缓冲区字符串转成 float
                print "received the relative_position in fifo: %f" % self._get_relative_position_data[num]
                num = num + 1
            
                if len(self._get_relative_position_data) == 0:
                    print "The fifo now without any data."
                    break
            
            cache_data = os.read(res_read, 2)  #缓冲区长度(1位字符)
            self.receive_base_flag = str(cache_data)  # 将缓冲区字符串转成 float
            print "received the receive_base_flag in fifo: %s" % self.receive_base_flag
            
            if self.receive_base_flag == "G0":
                self.__base_flag = True
                self.__base_flag = True
            elif self.receive_base_flag == "G6":
                self.__base_flag = False           
            else: 
                print "The fifo now without any receive_base_flag."
            # 将base_flag_更新到主界面函数
            self.now_base_flag.emit(self.__base_flag)

            self.receive_tag = "True"
            #self.relative_pose_data_to_rad_fun(self._get_relative_position_data)           
            self.receive_relative__joint_position_data.emit(self._get_relative_position_data)
       
        os.close(res_read)
        
        pass

    # 将笛卡尔位姿转换成4*4的齐次坐标矩阵(输入：1*6的笛卡尔姿态数据； 输出： 4*4的齐次坐标矩阵)
    def get_the_rotate_matrix(self, data):

        return np.array([ [math.cos(data[3])*math.cos(data[4]), math.cos(data[3])*math.sin(data[4])*math.sin(data[5]) - math.sin(data[3])*math.cos(data[5]), math.cos(data[3])*math.sin(data[4])*math.cos(data[5]) + math.sin(data[3])*math.sin(data[5]), data[0] ],   
                        [math.sin(data[3])*math.cos(data[4]), math.cos(data[3])*math.cos(data[5]) + math.sin(data[3])*math.sin(data[4])*math.sin(data[5]), math.sin(data[3])*math.sin(data[4])*math.cos(data[5]) - math.sin(data[5])*math.cos(data[3]), data[1] ],                                                     
                        [math.sin(data[4])*(-1), math.cos(data[4])*math.sin(data[5]), math.cos(data[4])*math.cos(data[5]), data[2]  ],                     
                        [0, 0, 0, 1] ] )
        pass


    # 获得机器人当前末端的位姿
    def get_now_positive_data(self, data):

        self._now_robot_pasitive_position[0] = data[0] 
        self._now_robot_pasitive_position[1] = data[1] 
        self._now_robot_pasitive_position[2] = data[2]
        self._now_robot_pasitive_position[3] = data[3] 
        self._now_robot_pasitive_position[4] = data[4] 
        self._now_robot_pasitive_position[5] = data[5] 

        # 获得目标笛卡尔点到基座标系下的表示(unit : mm; deg)
        self._goal_to_base_position_data  = robot_increTransTool_client(self._now_robot_pasitive_position, self._get_relative_position_data)
        self.goal_to_base_position_.emit(self._goal_to_base_position_data)
        
        # print "print____goal_to_base_position_data: "
        # for j in self._goal_to_base_position_data:
        #     print(j)
        pass

     # 通过管道传送转换标志反馈
    def feedback_translation_tag_fun(self, feedback_translation_tag):

        if os.path.exists(_PATH_NAME_WRITE_):
            os.remove(_PATH_NAME_WRITE_)
            
        os.mkfifo(_PATH_NAME_WRITE_)
        res_write = os.open(_PATH_NAME_WRITE_, os.O_WRONLY)
        
        if res_write < 0:
            print "open res_write_feedback_tag_fifo wrong"
        else:    
            ret = os.write(res_write, str(feedback_translation_tag))
            print "send the feedback_translation_tag to partner..."
            if ret < 0:
                print "send error,please try argin..."
            else:
                print "send sucessful!,tag is: %s" % str(feedback_translation_tag)
           
        os.close(res_write)
        pass

        
    # 获得目标位姿到基座标的表示及生成关节运动命令（函数）    
    def get_goal_to_base_position_and_generate_joint_command_fun(self):   
        
        # 调用机器人正解函数，获得机器人当前的位姿,以及最终得到目标位姿在基座标系下的表示
        self.descartes_position = Positive_solution_client(self.__which_robot, self.__base_flag,  self.__pos_joints)
        print "received the self.__base_flag in fifo: %s" % self.__base_flag
        self.get_now_positive_data(self.descartes_position)
        
        # 调用机器人逆解函数，获得关节运动情况(含逆解正确与否的标志位)  
        [_joint_position_command, _joint_velocity_command, inverse_solution_tag] = Inverse_solution_client(1,  self.__base_flag, self._goal_to_base_position_data, self.descartes_velocity_set_data, self.__pos_joints)
        
        # 发送关机运动和速度指令，显示数据
        self.goal_to_base_motor_pos_command.emit(_joint_position_command, _joint_velocity_command)

        # 在ui界面显示逆解转换成功与否
        self.wrong_translation_feedback.emit(str(inverse_solution_tag))

        # 逆解转换标志位
        if str(inverse_solution_tag) == "False":
            feedback_translation_tag = 1
            

        elif str(inverse_solution_tag) == "True":
            feedback_translation_tag = 0

        else:
            print "before send to fifo, Something wrong"

        # 通过管道传送转换标志反馈
        self.feedback_translation_tag_fun(feedback_translation_tag)
        #####################################
        # 通过管道传送转换标志反馈
        # if os.path.exists(_PATH_NAME_WRITE_):
        #     os.remove(_PATH_NAME_WRITE_)
            
        # os.mkfifo(_PATH_NAME_WRITE_)
        
        # res_write = os.open(_PATH_NAME_WRITE_, os.O_WRONLY)
        # if res_write < 0:
        #     print "open res_write_feedback_tag_fifo wrong"
        # else:    
        #     ret = os.write(res_write, str(feedback_translation_tag))

        #     print "send the feedback_translation_tag to partner..."
        #     if ret < 0:
        #         print "send error,please try argin..."
        #     else:
        #         print "send sucessful!,tag is: %s" % str(feedback_translation_tag)
           
        # os.close(res_write)
        ############################################   
        pass



class Send_now_robot_descarte_goal_position_thread(QThread):


    def __init__(self, __now_robot_descarte_pos):
        super(Send_now_robot_descarte_goal_position_thread, self).__init__()
        self.now_robot_descarte_pos = __now_robot_descarte_pos
        pass

    def run(self):

        self.fifo_send_now_robot_descartes_pos_fun()

        pass

    def fifo_send_now_robot_descartes_pos_fun(self):

        if os.path.exists(_PATH_NAME_SEND_):
            os.remove(_PATH_NAME_SEND_)
            
        os.mkfifo(_PATH_NAME_SEND_)
        
        res_send = os.open(_PATH_NAME_SEND_, os.O_WRONLY)
        if res_send < 0:
            print "open res_send_robot_deacarte_pos_fifo wrong"
        else:
            i = 0

            # 将机器人当前笛卡尔坐标合成字符串流(每个数据由空格隔开)，发送到管道
            ret = os.write(res_send, str(self.now_robot_descarte_pos[0])+ " " + str(self.now_robot_descarte_pos[1]) + " " + str(self.now_robot_descarte_pos[2])+ " "+ str(self.now_robot_descarte_pos[3])+ " " + str(self.now_robot_descarte_pos[4])+ " " + str(self.now_robot_descarte_pos[5]))
         
            # for i in range(6):
                
            #     fate = str(self.now_robot_descarte_pos[i])
                
            #     ret = os.write(res_send, str(self.now_robot_descarte_pos[i])+ " ")
        
            #     print "send the robot_deacarte_pos_fifo to partner%s" % fate
            if ret < 0:
                print "send error,please try argin..."
            else:
                for i in range(6):
                    print "send sucessful!,robot_deacarte_pos_fifo is: %lf" % self.now_robot_descarte_pos[i]
            
        os.close(res_send)
        pass

    

