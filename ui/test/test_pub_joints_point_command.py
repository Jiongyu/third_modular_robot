#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from ui.msg import robot_command

def main():
    rospy.init_node('test_pub_joints_point_command', anonymous=True)
    pub = rospy.Publisher('/low_level/joints_point_command', robot_command , queue_size=10)
    command_msg = robot_command()

    rate = rospy.Rate(1) 
    index = 0
    
    while index < 10:

        command_msg.CommandPosData = [index] * 5
        command_msg.CommandVelData = [index] * 5
        command_msg.timeHeader.stamp = rospy.Time.now()
        pub.publish(command_msg)

        rate.sleep()
        index += 1


if __name__ == "__main__":
    main()