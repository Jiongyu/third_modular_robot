#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: robot_increTransTool_client.py

@biref: 基于ros服务计算笛卡尔命令---客户端
        输入输出参考srv/robot_increTransTool_server.srv
"""

import rospy
from birl_module_robot.srv import robot_increTransTool


def robot_increTransTool_client(actual_descartes_position, incre_descartes_command):
    # rospy.init_node("test", log_level=rospy.INFO)
    rospy.loginfo("Wait For Server: increTransTool.")
    rospy.wait_for_service("robot_increransTool")
    rospy.loginfo("Client Get New Requset.")
    try:
        client = rospy.ServiceProxy("robot_increransTool", robot_increTransTool)
        resp = client.call(actual_descartes_position, incre_descartes_command)
        
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s" %e)
        return
    # 关节位置、速度保留二位小数
    new_descartes_position = [0,0,0,0,0,0]

    for i in range(len(resp.descartes_command)):
        new_descartes_position[i] = round(resp.descartes_command[i],3)
    
    return new_descartes_position


if __name__ == "__main__":
    actual_descartes_position = [120, 100, 100, 20, 30, 40]
    incre_descartes_command = [5, 10, 15, 5, 5, 5]
    new_descartes_position  = robot_increTransTool_client(actual_descartes_position, incre_descartes_command)
    print new_descartes_position