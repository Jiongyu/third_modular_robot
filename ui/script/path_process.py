#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: path_process.py

@biref: 根据路径点，提取轨迹
"""

import string
from math import radians

class Path_process():
    
    def __init__(self, zero_position, joint_direction, max_path_velocity):

        self.__joint_pos = []
        self.__max_vel = max_path_velocity
        self.__zero_position = zero_position
        self.__joints_direction = joint_direction
        self.__halt_index_list = []

        self.__halt_flag = "HALT"

    def __readfile(self,data):
        """
        @brief: 读取路径点文本文件
        """
        temp_data = data.split('\n')
        # print temp_data
        pos_index = 0
        for i in range(len(temp_data)):
            # 判断是否以P开始
            if temp_data[i][0:1] == 'P':
                temp_data[i] = temp_data[i].replace('=',' ')
                temp_data[i] = temp_data[i].replace(',',' ')
                temp_data[i] = temp_data[i].replace(';','')
                temp_data[i] = temp_data[i].split(' ')
                temp_array = []
                for j in range(1,len(temp_data[i]) - 1):
                    temp_array.append((float(str(temp_data[i][j]))) * self.__joints_direction[j - 1] + self.__zero_position[j - 1] )
                self.__joint_pos.append(temp_array)
                pos_index += 1

            # 获取暂停标志
            elif(temp_data[i] == self.__halt_flag):
                # pos_index += 1
                # self.__halt_index_list.append(pos_index)
                self.__joint_pos.append(temp_data[i]) 
                pass
        # print self.__joint_pos
        # print self.__halt_index_list  

    def __get_joint_position(self):
        """
        @brief: 获取路径点
        """

        temp = []
        temp.append(self.__joint_pos[0])

        deg_interval = self.__max_vel

        i = 0
        flag = 0
        while(i < (len(self.__joint_pos) - 1)):
                # 判断暂停标志:
                if self.__joint_pos[i + 1] == self.__halt_flag:
                    temp.append(self.__joint_pos[i + 1])
                    # print self.__joint_pos[i + 1]
                else:
                    for j in range(len(self.__joint_pos[i])):
                        if(abs(self.__joint_pos[flag][j] - self.__joint_pos[i+1][j]) > deg_interval ):
                            # print self.__joint_pos[i + 1]
                            temp.append(self.__joint_pos[i+1])
                            flag = i + 1
                            break
                i += 1
            
        if(temp[-1] != self.__joint_pos[-1]):
            temp.append(self.__joint_pos[-1])
                
        self.__joint_pos = temp
        # print self.__joint_pos

    def __get_joint_velocity(self):
        """
        @brief : 根据路径点，获取速度信息
        """

        max_pos = 0
        temp_vel = []
        joint_vel = []
        temp_time = []
        halt_number = 0
        i = 0
        while i < len(self.__joint_pos) - 1:
            # 判断当前点是否为暂停标志
            if self.__joint_pos[i] == self.__halt_flag:
                i += 1
                # print "continue"
                halt_number  += 1
                continue

            # 判断下一点是否为暂停标志
            next_point_flag = i + 1
            if self.__joint_pos[next_point_flag] == self.__halt_flag:
                next_point_flag += 1
                # print "next point"

            for j in range(len(self.__joint_pos[i])):
                temp = abs(self.__joint_pos[i][j] - self.__joint_pos[next_point_flag][j])
                if(temp > max_pos):
                    max_pos = temp

            temp_time.append(max_pos / self.__max_vel)

            for j in range(len(self.__joint_pos[i])):
                temp = abs(self.__joint_pos[i][j] - self.__joint_pos[next_point_flag][j])
                temp_vel.append(temp / temp_time[i - halt_number])
            joint_vel.append(temp_vel)

            temp_vel = []
            max_pos = 0
            i += 1

        del self.__joint_pos[0]

        halt_number = 0 
        for i in range(len(self.__joint_pos)):
            # 判断暂停标志
            if self.__joint_pos[i] == self.__halt_flag:
                halt_number += 1
                continue
            for j in range(len(joint_vel[i - halt_number])):
                self.__joint_pos[i].append(joint_vel[i - halt_number][j])
            self.__joint_pos[i].append(temp_time[i - halt_number])


        for i in range(len(self.__joint_pos)):
            # 判断暂停标志
            if self.__joint_pos[i] == self.__halt_flag:
                continue
            for j in range(len(self.__joint_pos[i])):
                self.__joint_pos[i][j] = round(self.__joint_pos[i][j], 3)
            self.__joint_pos[i][j] = round(self.__joint_pos[i][j], 3)

        # print self.__joint_pos      

    def get_trajectory(self,data):
        
        self.__readfile(data)
        self.__get_joint_position()
        self.__get_joint_velocity()
        return self.__joint_pos
          
