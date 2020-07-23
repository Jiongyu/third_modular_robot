#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from birl_module_robot.srv import positive_solution

def positive_solution_client( which_gripper, current_joint_position):
    rospy.loginfo("Wait For Server: positive_solution.")
    rospy.wait_for_service("positive_solution")
    rospy.loginfo("Client Get New Requset.")
    try:
        client = rospy.ServiceProxy("positive_solution", positive_solution)
        
        resp = client.call(which_gripper, current_joint_position)

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
    temp = positive_solution_client(True, [0, 0, 0, 0, 0])
    print temp
