#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from rospkg import RosPack
import time
from birl_module_robot.msg import joint_point
from path_process import Path_process

def main():
    start_point = [0,0,0,0,0]
    process = Path_process(start_point)
    trajectory = process.get_trajectory('20200108.txt')
    rospy.init_node("biped5d_sent_joint_command", anonymous=True)
    publisher = rospy.Publisher("/low_level/biped5d_joint_command",joint_point,queue_size = 20)
    joint_command = joint_point()
    joint_command.data.data = trajectory

    while(not rospy.is_shutdown()):
        publisher.publish(joint_command)
        time.sleep(3)
    rospy.spin()

if __name__ == '__main__':
    main()
