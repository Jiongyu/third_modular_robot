#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: positive_solution_client.py

@biref: 基于ros服务获取运动学正解---客户端
        输入输出参考srv/inverse_solution.srv
"""

import rospy
from birl_module_robot.srv import positive_solution

def Positive_solution_client(which_robot, which_base, current_joint_position):
    rospy.loginfo("Wait For Server: positive_solution.")
    try:
        rospy.wait_for_service("positive_solution", timeout=3)
    except rospy.ROSException:
        rospy.loginfo("positive_solution timeout.")
        return None 

    rospy.loginfo("Client Get New Requset.")
    try:
        client = rospy.ServiceProxy("positive_solution", positive_solution)
        
        resp = client.call(which_robot, which_base, current_joint_position)

    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s"%e)
    
    descartes_position_state = [0,0,0,0,0,0]

    # X Y Z (m) 保留五位小数
    for i in range(0,3):
        descartes_position_state[i] = round(resp.descartes_pos_state[i], 5)

    # RX RY RZ (deg)  保留两位小数
    for i in range(3,6):
        descartes_position_state[i] = round(resp.descartes_pos_state[i], 2)

    return descartes_position_state


if __name__ == "__main__":
    temp = Positive_solution_client(0, True, [0, 20, -50, 30, 0])
    print temp
