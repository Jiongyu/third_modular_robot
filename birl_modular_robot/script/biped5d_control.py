#!/usr/bin/env python
# -*- coding: utf-8 -*-
import signal
import time

import sys
import rospy
from rospkg import RosPack
sys.path.append(RosPack().get_path('canopen_communication') + "/modular/")
from std_msgs.msg import Float64MultiArray
from birl_module_robot.msg import joint_point

import threading

from modular_T100 import T100
from modular_I100 import I100

class Biped5d_control():

    eds_file = RosPack().get_path('canopen_communication') + "/file/Copley.eds"
    mutex = threading.Lock()
    task_stop = False
    joint_command = joint_point()
    command_start = False

    
    def __init__(self):

        rospy.init_node("biped5d_control", anonymous=True, log_level=rospy.DEBUG)
        rospy.loginfo("Biped5d_control init")

        signal.signal(signal.SIGINT, Biped5d_control.__quit_muti_thread)
        signal.signal(signal.SIGTERM, Biped5d_control.__quit_muti_thread)

        self.thread_command = threading.Thread(target = Biped5d_control.__task_command,args=(None,))
        self.thread_feedback = threading.Thread(target = Biped5d_control.__task_feedback,args=(None,))
        self.thread_get_joint_command = threading.Thread(target = Biped5d_control.__get_joint_command,args=(None,))

        self.thread_get_joint_command.start()
        self.thread_command.start()
        self.thread_feedback.start()


        signal.pause()
        self.thread_get_joint_command.join()
        self.thread_command.join()
        self.thread_feedback.join()
   
    @staticmethod
    def __task_command(self):
        
        rospy.loginfo("task command start")
        
        Biped5d_control.mutex.acquire()
        Biped5d_control.__start_communication()
        Biped5d_control.mutex.release()
        Biped5d_control.joint_command.data.data = []

        while (not Biped5d_control.task_stop):

            while((not Biped5d_control.command_start) and (not Biped5d_control.task_stop)):
                time.sleep(1)
                rospy.loginfo("wait for joint command...")
            
            if(Biped5d_control.task_stop):
                break

            command = []
            index = 0
            command_length = 0

            for i in range(0,len(Biped5d_control.joint_command.data.data),10):
                command.append(Biped5d_control.joint_command.data.data[i:i+10])

            command_length = len(command)

            Biped5d_control.mutex.acquire()

            Biped5d_control.I1.sent_position(command[index][0],command[index][5])
            Biped5d_control.T2.sent_position(command[index][1],command[index][6])
            Biped5d_control.T3.sent_position(command[index][2],command[index][7])
            Biped5d_control.T4.sent_position(command[index][3],command[index][8])
            Biped5d_control.I5.sent_position(command[index][4],command[index][9])
            index += 1
            Biped5d_control.command_start = False
            Biped5d_control.mutex.release()   

            while((index != command_length) and (not Biped5d_control.task_stop)):

                Biped5d_control.mutex.acquire()
                if (Biped5d_control.I1.reached() and Biped5d_control.T2.reached() and
                    Biped5d_control.T3.reached() and Biped5d_control.T4.reached() and 
                    Biped5d_control.I5.reached() ):
                    Biped5d_control.T2.sent_position(command[index][1],command[index][6])
                    Biped5d_control.T3.sent_position(command[index][2],command[index][7])
                    Biped5d_control.T4.sent_position(command[index][3],command[index][8])
                    Biped5d_control.I5.sent_position(command[index][4],command[index][9])
                    index += 1
                # if (Biped5d_control.I1.reached()):
                #     Biped5d_control.I1.sent_position(command[index][0],command[index][5])
                #     index += 1

                Biped5d_control.mutex.release()
                time.sleep(0.001)
        
        rospy.loginfo("task command end")

    @staticmethod
    def __task_feedback(self):

        rospy.loginfo("task feedback start")
        publisher = rospy.Publisher('/low_level/biped5d_joint_point',Float64MultiArray,queue_size = 10)

        feedback_publish= Float64MultiArray()
        feedback_publish.data = [0] * 5

        while(not Biped5d_control.task_stop):
            
            Biped5d_control.mutex.acquire()

            feedback = [round(Biped5d_control.I1.get_position(),3),\
                        round(Biped5d_control.T2.get_position(),3),\
                        round(Biped5d_control.T3.get_position(),3),\
                        round(Biped5d_control.T4.get_position(),3),\
                        round(Biped5d_control.I5.get_position(),3) ]
            # feedback = [round(Biped5d_control.I1.get_position(),3),0,0,0,0 ]
            Biped5d_control.mutex.release()

            feedback_publish.data = []
            
            for i in range(len(feedback)):
                feedback_publish.data.append(feedback[i])

            publisher.publish(feedback_publish)
            rospy.timer.sleep(0.02) # 20ms

        rospy.loginfo("task feedback end")


    @staticmethod
    def __get_joint_command(self):
        rospy.loginfo("task get joint command start")

        sub = rospy.Subscriber("/low_level/biped5d_joint_command",joint_point,Biped5d_control.positive_value_callbacks)

        while(not Biped5d_control.task_stop):
            try:
                rospy.wait_for_message('/low_level/biped5d_joint_command', Float64MultiArray, 5)
            except:
                rospy.loginfo("timeout.../low_level/biped5d_joint_command")
        rospy.loginfo("task get joint command end")
       
    
    @staticmethod
    def positive_value_callbacks(msg):
        if (Biped5d_control.joint_command.data != msg.data):
            Biped5d_control.joint_command = msg
            Biped5d_control.command_start = True
            rospy.loginfo("get command message")
        pass

   
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
        Biped5d_control.I1.opmode_set('PROFILED POSITION')
        Biped5d_control.T2.opmode_set('PROFILED POSITION')
        Biped5d_control.T3.opmode_set('PROFILED POSITION')
        Biped5d_control.T4.opmode_set('PROFILED POSITION')
        Biped5d_control.I5.opmode_set('PROFILED POSITION')
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
        if(Biped5d_control.mutex.locked()):
            Biped5d_control.mutex.release()
        Biped5d_control.task_stop = True
        Biped5d_control.__stop_robot()

if __name__ == '__main__':
    Biped5d_control()