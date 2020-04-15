#!/usr/bin/env python
# -*- coding: utf-8 -*-
import signal
import ctypes
import inspect
import sys
from math import fabs 

import rospy
from rospkg import RosPack
sys.path.append(RosPack().get_path('canopen_communication') + "/modular/")
from std_msgs.msg import Float64MultiArray
import threading

from modular_T100 import T100
from modular_I100 import I100

class Biped5d_control():

    eds_file = RosPack().get_path('canopen_communication') + "/file/Copley.eds"
    mutex = threading.Lock()
    stop = False
    joint_command = Float64MultiArray()
    index = 0
    command_start = False
    joint_command_length = 0

    def __init__(self):

        rospy.init_node("biped5d_control", anonymous=True, log_level=rospy.DEBUG)
        # rospy.loginfo("Biped5d_control init")

        signal.signal(signal.SIGINT, Biped5d_control.__quit_muti_thread)
        signal.signal(signal.SIGTERM, Biped5d_control.__quit_muti_thread)

        
        self.thread_command = threading.Thread(target = Biped5d_control.task_command,args=(None,))
        self.thread_feedback = threading.Thread(target = Biped5d_control.task_feedback,args=(None,))
        self.thread_get_joint_data = threading.Thread(target = Biped5d_control.get_joint_command,args=(None,))

        self.thread_command.start()
        self.thread_feedback.start()

        signal.pause()

        self.thread_command.join()
        self.thread_feedback.join()
   
    @staticmethod
    def task_command(self):
        
        Biped5d_control.mutex.acquire()
        Biped5d_control.__start_communication()
        Biped5d_control.joint_command.data = []
        Biped5d_control.mutex.release()
        
        rospy.loginfo("Robot start communication complete")

        while(not Biped5d_control.stop):

            while(! Biped5d_control.command_start):
                time.sleep(0.05)
                rospy.loginfo("wait for joint command ")

            Biped5d_control.mutex.acquire()
                Biped5d_control.I1.sent_position(round(Biped5d_control.joint_command.data[index][0],3),round(joint_command.data[index][5],3))
                Biped5d_control.T2.sent_position(round(Biped5d_control.joint_command.data[index][1],3),round(joint_command.data[index][6],3))
                Biped5d_control.T3.sent_position(round(Biped5d_control.joint_command.data[index][2],3),round(joint_command.data[index][7],3))
                Biped5d_control.T4.sent_position(round(Biped5d_control.joint_command.data[index][3],3),round(joint_command.data[index][8],3))
                Biped5d_control.I5.sent_position(round(Biped5d_control.joint_command.data[index][4],3),round(joint_command.data[index][9],3))
                index += 1
                Biped5d_control.command_start = False
            Biped5d_control.mutex.release()

            while(index != Biped5d_control.joint_command_length):

                Biped5d_control.mutex.acquire()
                if( ( Biped5d_control.I1.reached()) and ( Biped5d_control.T2.reached()) and \
                    ( Biped5d_control.T3.reached()) and ( Biped5d_control.T4.reached()) and \
                    ( Biped5d_control.I5.reached())):
                    Biped5d_control.I1.sent_position(round(Biped5d_control.joint_command.data[index][0],3),round(joint_command.data[index][5],3))
                    Biped5d_control.T2.sent_position(round(Biped5d_control.joint_command.data[index][1],3),round(joint_command.data[index][6],3))
                    Biped5d_control.T3.sent_position(round(Biped5d_control.joint_command.data[index][2],3),round(joint_command.data[index][7],3))
                    Biped5d_control.T4.sent_position(round(Biped5d_control.joint_command.data[index][3],3),round(joint_command.data[index][8],3))
                    Biped5d_control.I5.sent_position(round(Biped5d_control.joint_command.data[index][4],3),round(joint_command.data[index][9],3))
                    index += 1
                Biped5d_control.mutex.release() 
                time.sleep(0.001)
            
            index = 0
            
        rospy.loginfo("task command end")

    @staticmethod
    def task_feedback(self):

        rospy.loginfo("task feedback start")
        publisher = rospy.Publisher('/low_level/biped5d_joint_point',Float64MultiArray,queue_size = 10)

        feedback_publish= Float64MultiArray()
        feedback_publish.data = [0] * 5

        while(not Biped5d_control.stop):
            
            Biped5d_control.mutex.acquire()

            feedback = [round(Biped5d_control.I1.get_position(),3),\
                        round(Biped5d_control.T2.get_position(),3),\
                        round(Biped5d_control.T3.get_position(),3),\
                        round(Biped5d_control.T4.get_position(),3),\
                        round(Biped5d_control.I5.get_position(),3) ]

            feedback_publish.data.clear()
            
            for i in range(len(feedback)):
                feedback_publish.data.append(feedback[i])
            publisher.publish(feedback_publish)
            rospy.timer.sleep(0.02) # 20ms


            # ''' test code ''' 
            # test_feedback = [0,0.55,-1.1,0.55,0]
            # for i in range(len(test_feedback)):
            #     feedback_publish.data.append(test_feedback[i])
            # publisher.publish(feedback_publish)

            # # rospy.loginfo("--------------------")
            # # for i in range(len(feedback_publish.data)):
            # #     rospy.loginfo(str(feedback_publish.data[i]))

            # feedback_publish.data = []
            # Biped5d_control.mutex.release()
            # rospy.timer.sleep(0.02) # 20ms
            # ''' end '''

        rospy.loginfo("task feedback end")

    @staticmethod
    def get_joint_command(self):

        rospy.loginfo("task command start")
        sub = rospy.Subscriber("/low_level/biped5d_joint_command",Float64MultiArray,Biped5d_control.positive_value_callbacks)

        while(not Biped5d_control.stop):
            try:
                rospy.wait_for_message('/low_level/biped5d_joint_command', Float64MultiArray, 5)
            except:
                rospy.loginfo("timeout.../low_level/biped5d_joint_command")

    @staticmethod
    def positive_value_callbacks(msg):
        Biped5d_control.joint_command = msg
        Biped5d_control.command_start = True
        Biped5d_control.joint_command_length = len(Biped5d_control.joint_command)

    @staticmethod
    def __start_communication():

        Biped5d_control.I1 = I100(1,Biped5d_control.eds_file)
        Biped5d_control.T2 = T100(2,Biped5d_control.eds_file)
        Biped5d_control.T3 = T100(3,Biped5d_control.eds_file)
        Biped5d_control.T4 = T100(4,Biped5d_control.eds_file)
        Biped5d_control.I5 = I100(5,Biped5d_control.eds_file)

        Biped5d_control.I1.start()
        Biped5d_control.T2.start()
        Biped5d_control.T3.start()
        Biped5d_control.T4.start()
        Biped5d_control.I5.start()

        # set position mode.
        Biped5d_control.I1.set_mode(1)
        Biped5d_control.T2.set_mode(1)
        Biped5d_control.T3.set_mode(1)
        Biped5d_control.T4.set_mode(1)
        Biped5d_control.I5.set_mode(1)
        rospy.loginfo("start canopen communication")

    @staticmethod
    def __stop_robot():
        Biped5d_control.I1.stop()
        Biped5d_control.T2.stop()
        Biped5d_control.T3.stop()
        Biped5d_control.T4.stop()
        Biped5d_control.I5.stop()

    @staticmethod
    def __quit_muti_thread(sig, frame):
        Biped5d_control.stop = True
        Biped5d_control.__stop_robot()
        sys.exit()


if __name__ == '__main__':
    Biped5d_control()