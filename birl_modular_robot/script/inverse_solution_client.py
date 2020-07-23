#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from birl_module_robot.srv import inverse_solution

def inverse_solution_client( which_gripper, descartes_position_command, descartes_velocity_command, current_joint_position):
    rospy.loginfo("Wait For Server: inverse_solution.")
    rospy.wait_for_service("inverse_solution")
    rospy.loginfo("Client Get New Requset.")
    try:
        client = rospy.ServiceProxy("inverse_solution", inverse_solution)
        
        resp = client.call(which_gripper, descartes_position_command, descartes_velocity_command, current_joint_position)

    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s"%e)
    
    # 关节位置、速度保留二位小数
    joint_position_command = [0,0,0,0,0]
    joint_velocity_command = [0,0,0,0,0]
    for i in range(len(resp.joint_pos_commands)):
        joint_position_command[i] = round(resp.joint_pos_commands[i],2)
    for i in range(len(resp.joint_vel_commands)):
        joint_velocity_command[i] = round(resp.joint_vel_commands[i],2)

    return [joint_position_command, joint_velocity_command]


if __name__ == "__main__":
    [temp_1, temp_2] = inverse_solution_client(True, [0.56, 0, 0, 0, 0, 180], [-0.005, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0])
    print temp_1
    print temp_2
