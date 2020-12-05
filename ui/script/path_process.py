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
            elif(temp_data[i][0:6] == "HALT"):
                # pos_index += 1
                self.__halt_index_list.append(pos_index)
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

        while(i < (len(self.__joint_pos) - 1)):
                for j in range(len(self.__joint_pos[i])):
                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+1][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+1])
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

        for i in range(len(self.__joint_pos) - 1):

            for j in range(len(self.__joint_pos[i])):
                temp = abs(self.__joint_pos[i][j] - self.__joint_pos[i+1][j])
                if(temp > max_pos):
                    max_pos = temp

            temp_time.append(max_pos / self.__max_vel)

            for j in range(len(self.__joint_pos[i])):
                temp = abs(self.__joint_pos[i][j] - self.__joint_pos[i+1][j])
                temp_vel.append(temp / temp_time[i])
            
            joint_vel.append(temp_vel)

            temp_vel = []
            max_pos = 0

        del self.__joint_pos[0]
        if(len(self.__joint_pos) == len(joint_vel)):
            for i in range(len(self.__joint_pos)):

                for j in range(len(joint_vel[i])):
                    self.__joint_pos[i].append(joint_vel[i][j])
                self.__joint_pos[i].append(temp_time[i])

        for i in range(len(self.__joint_pos)):
            for j in range(len(self.__joint_pos[i])):
                self.__joint_pos[i][j] = round(self.__joint_pos[i][j], 3)
            self.__joint_pos[i][j] = round(self.__joint_pos[i][j], 3)

        # print self.__joint_pos      

    # 插入暂停标志
    def __insert_halt_index(self):
        if(self.__halt_index_list):
            for i in range(len(self.__halt_index_list)):
                self.__halt_index_list[i] -= 1

            # print self.__halt_index_list

            halt_index = 0
            lengthJ = len(self.__joint_pos)
            index = 0
            while (index < lengthJ):
                if(index == self.__halt_index_list[halt_index]):
                    # print index
                    self.__joint_pos.insert(index, "HALT");
                    halt_index += 1

                    for j in range(halt_index, len(self.__halt_index_list)):
                        self.__halt_index_list[j] += 1
                    lengthJ += 1
                    # print self.__halt_index_list
                    if(halt_index == len(self.__halt_index_list)):
                        break 
                
                index += 1
            pass
        # print self.__joint_pos

    def get_trajectory(self,data):
        
        self.__readfile(data)
        self.__get_joint_position()
        self.__get_joint_velocity()
        self.__insert_halt_index()
        return self.__joint_pos
          
