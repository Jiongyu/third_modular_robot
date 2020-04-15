#!/usr/bin/env python
# -*- coding: utf-8 -*-
import string
from math import radians
from rospkg import RosPack

class Path_process():
    
    def __init__(self,start_point = [0,0,0,0,0]):

        self.__joint_pos = []
        self.__max_vel = 5

        self.__I1_direction = 1
        self.__T2_direction = 1
        self.__T3_direction = 1
        self.__T4_direction = 1
        self.__I5_direction = 1  

        # add the start point
        temp = []
        for i in range(len(start_point)):
            temp.append(start_point[i])
        self.__joint_pos.append(temp) 

    def __get_joint_direction(self):

        f = open(RosPack().get_path('birl_module_robot') + '/import_file/positive_and_negetive.txt')
        line = f.readline()
        temp = []
        while line:
            if line.startswith('D'):
                temp = line.split(' ')
                temp = temp[1:6]
            line = f.readline()  
        f.close()

        if(string.atof(temp[0])):
            self.__I1_direction = 1
        else:
            self.__I1_direction = -1

        if(string.atof(temp[1])):
            self.__T2_direction = 1
        else:
            self.__T2_direction = -1

        if(string.atof(temp[2])):
            self.__T3_direction = 1
        else:
            self.__T3_direction = -1

        if(string.atof(temp[3])):
            self.__T4_direction = 1
        else:
            self.__T4_direction = -1

        if(string.atof(temp[4])):
            self.__I5_direction = 1
        else:
            self.__I5_direction = -1
        
        # print self.__I1_direction
        # print self.__T2_direction
        # print self.__T3_direction
        # print self.__T4_direction
        # print self.__I5_direction


    def __readfile(self,path):

        f = open(path)     
        line = f.readline()

        while line: 
            if line.startswith('P'):    

                s = line.split("=")

                s = s[1].split(',')     

                s[0] = string.atof(s[0]) * self.__I1_direction 

                s[1] = string.atof(s[1]) * self.__T2_direction

                s[2] = string.atof(s[2]) * self.__T3_direction    

                s[3] = string.atof(s[3]) * self.__T4_direction

                s[4] = string.atof(s[4]) * self.__I5_direction
                
                self.__joint_pos.append(s[0:5])

            line = f.readline()  
        f.close()

        # for i in range(len(self.__joint_pos)):
        #     print self.__joint_pos[i]


    def __get_joint_position(self):

        temp = []
        temp.append(self.__joint_pos[0])

        deg_interval = self.__max_vel * 2
        data_inteval = 5

        i = 0
        while(i != (len(self.__joint_pos))):
            try:
                for j in range(len(self.__joint_pos[i])):

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+1][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+1])
                        i = i+1
                        break

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+2][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+2])
                        i = i+2
                        break

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+3][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+3])
                        i = i+3
                        break

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+4][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+4])
                        i = i+4 
                        break

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+5][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+5]) 
                        i = i+5
                        break
            except IndexError:
                pass
            i+=1
            
        if(temp[-1] != self.__joint_pos[-1]):
            temp.append(self.__joint_pos[-1])
                
        self.__joint_pos = temp

        # print len(self.__joint_pos)
        # for i in range(len(self.__joint_pos)):
        #     print self.__joint_pos[i]

    def __get_joint_velocity(self):

        max_pos = 0
        temp_time = 0
        temp_vel = []
        joint_vel = []


        for i in range(len(self.__joint_pos) - 1):

            for j in range(len(self.__joint_pos[i])):
                temp = abs(self.__joint_pos[i][j] - self.__joint_pos[i+1][j])
                if(temp > max_pos):
                    max_pos = temp

            temp_time = max_pos / self.__max_vel
            # print temp_time

            for j in range(len(self.__joint_pos[i])):
                temp = abs(self.__joint_pos[i][j] - self.__joint_pos[i+1][j])
                temp_vel.append(temp / temp_time)
            
            joint_vel.append(temp_vel)

            temp_vel = []
            max_pos = 0

        # print len(joint_vel)
        # for i in range(len(joint_vel)):
        #     print joint_vel[i]

        del self.__joint_pos[0]
        if(len(self.__joint_pos) == len(joint_vel)):
            for i in range(len(self.__joint_pos)):

                for j in range(len(joint_vel[i])):
                    self.__joint_pos[i].append(joint_vel[i][j])

        # convert from deg to rad
        for i in range(len(self.__joint_pos)):
            for j in range(len(self.__joint_pos[i])):
                self.__joint_pos[i][j] = round(radians(self.__joint_pos[i][j]), 4)

        temp = []
        # convert joint data to one list
        for i in range(len(self.__joint_pos)):
            for j in range(len(self.__joint_pos[i])):
                temp.append(self.__joint_pos[i][j])
        self.__joint_pos = temp
        # print joint_pos

    def get_trajectory(self,data):
        self.__get_joint_direction()
        self.__readfile(RosPack().get_path('birl_module_robot') + '/data/' + data)
        self.__get_joint_position()
        self.__get_joint_velocity()
        return self.__joint_pos


        