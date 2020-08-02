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
    
    def __init__(self, joint_direction):

        self.__joint_pos = []
        self.__max_vel = 5

        self.__joints_direction = joint_direction

    def __readfile(self,data):
        """
        @brief: 读取路径点文本文件
        """
        temp_data = data.split('\n')
        for i in range(len(temp_data)):
            # 判断是否以P开始
            if temp_data[i][0:1] == 'P': 
                temp_data[i] = temp_data[i].replace('=',' ')
                temp_data[i] = temp_data[i].replace(',',' ')
                temp_data[i] = temp_data[i].replace(';','')
                temp_data[i] = temp_data[i].split(' ')
                temp_array = []
                for j in range(1,len(temp_data[i]) - 1):
                    temp_array.append(float(str(temp_data[i][j])) * self.__joints_direction[j - 1])

                self.__joint_pos.append(temp_array)

        # print self.__joint_pos

    def __get_joint_position(self):
        """
        @brief: 获取路径点
        """

        temp = []
        temp.append(self.__joint_pos[0])

        deg_interval = self.__max_vel

        i = 0
        getNextPoint = False
        while(i < (len(self.__joint_pos))):
            try:
                for j in range(len(self.__joint_pos[i])):

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+1][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+1])
                        i = i+1
                        getNextPoint = True
                        break

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+2][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+2])
                        i = i+2
                        getNextPoint = True
                        break

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+3][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+3])
                        i = i+3
                        getNextPoint = True
                        break

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+4][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+4])
                        i = i+4 
                        getNextPoint = True
                        break

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+5][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+5]) 
                        i = i+5
                        getNextPoint = True
                        break
            except IndexError:
                pass
            if getNextPoint:
                getNextPoint = False
            else:
                i = i + 1

            
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


    def get_trajectory(self,data):
        
        self.__readfile(data)
        self.__get_joint_position()
        self.__get_joint_velocity()
        return self.__joint_pos