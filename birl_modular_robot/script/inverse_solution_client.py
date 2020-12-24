#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: inverse_solution_client.py

@biref: 基于ros服务获取运动学逆解---客户端
        输入输出参考srv/inverse_solution.srv
"""

import rospy
from birl_module_robot.srv import inverse_solution


def Inverse_solution_client( which_robot, which_base, descartes_position_command, descartes_velocity_command, current_joint_position):
    rospy.loginfo("Wait For Server: inverse_solution.")
    rospy.wait_for_service("inverse_solution")
    rospy.loginfo("Client Get New Requset.")
    try:
        client = rospy.ServiceProxy("inverse_solution", inverse_solution)
        
        resp = client.call(which_robot, which_base, descartes_position_command, descartes_velocity_command, current_joint_position)

    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s"%e)
    
    # 关节位置、速度保留二位小数
    joint_position_command = [0,0,0,0,0]
    joint_velocity_command = [0,0,0,0,0]
    for i in range(len(resp.joint_pos_commands)):
        joint_position_command[i] = round(resp.joint_pos_commands[i],2)
    for i in range(len(resp.joint_vel_commands)):
        joint_velocity_command[i] = round(resp.joint_vel_commands[i],2)

    inverse_solution_tag = resp.ifGetSolve
    return [joint_position_command, joint_velocity_command, inverse_solution_tag]

if __name__ == "__main__":
    [temp_1, temp_2, inverse_solution_tag] = Inverse_solution_client(1, True, [550, 0, 100, 0, 0, 180], [0, 0, 10, 0, 0, 0], [0.0, 20.29, -40.59, 20.29, 0.0])
    print temp_1
    print temp_2
