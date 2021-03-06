#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: 
@file: find_grasp_point_client.py
@biref: 通过ros服务，根据机器人tcp当前位置姿态（相对于base）、
         杆件两端点（相对于carmera)、机器人夹持器基座，求解夹持
         点位置姿态（相对于base）---客户端
        输入输出参考srv/grasp_point.srv
"""

import rospy
from birl_module_robot.srv import grasp_point
import copy

def Find_grasp_point_client( which_base, current_descartes_position, p1, p2):
    # 检测输入
    if((len(p1) != 3) or (len(p2) != 3) ):
        return [None, False]

    # deg --> rad   
    PI_RAD = 0.0174533
    temp_pos = copy.deepcopy(current_descartes_position)
    temp_pos[3] *= PI_RAD
    temp_pos[4] *= PI_RAD
    temp_pos[5] *= PI_RAD

    rospy.loginfo("Wait For Server: grasp_point.")

    try:
        rospy.wait_for_service("find_grasp_point", timeout=3)
    except rospy.ROSException:
        rospy.loginfo("find_grasp_point timeout.")
        return [None, False]

    rospy.loginfo("Client Get New Requset.")
    
    try:
        client = rospy.ServiceProxy("find_grasp_point", grasp_point)
        
        resp = client.call(temp_pos, p1, p2, which_base)

    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s"%e)
        result = False
    result = True

    return [resp.grasp_point, result]
    
if __name__ == "__main__":
    rospy.init_node("Find_grasp_point_client", log_level=rospy.INFO)
    [temp1, temp2] = Find_grasp_point_client(True, [507.8,0,0,0,0,180], [0, 50, 50], [0, -50, 50])
    print temp1
    print temp2