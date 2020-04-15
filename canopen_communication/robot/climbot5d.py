#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*-

import sys
sys.path.append("./src/birl_modular_robot/canopen_communication/modular")
from modular_T100 import T100
from modular_G100 import G100
from modular_I100 import I100
from math import fabs

class Climbot5d(object):
    """
    climb robot with 5 degree of freedom
    """
    def __init__(self,eds_file):
        """
        init a example of climb robot
        :param eds_file: path of the eds file
        """
        self.eds_file = eds_file
        self.I1 = I100(1,self.eds_file)
        self.T2 = T100(2,self.eds_file)
        self.T3 = T100(3,self.eds_file)
        self.T4 = T100(4,self.eds_file)
        self.I5 = I100(5,self.eds_file)
        self.G0 = G100(6,self.eds_file)
        self.G6 = G100(7,self.eds_file)
        print "Climbot5d init is over!"

    def start(self):
        """
        begin the communication between climbot and canopen
        """
        self.G0.start()
        self.I1.start()
        self.T2.start()
        self.T3.start()
        self.T4.start()
        self.I5.start()
        self.G6.start()
        #print "start the communication!"

    def set_mode(self,mode):
        """
        :param mode: motor operation mode(1,3,4)
        1: profiled position
        3: Profiled Velocity
        4: Profiled Torque
        """
        #type_ = {1:"profiled position",3:"Profiled Velocity",4:"Profiled Torque"}
        self.mode = mode
        self.I1.set_mode(self.mode)
        self.T2.set_mode(self.mode)
        self.T3.set_mode(self.mode)
        self.T4.set_mode(self.mode)
        self.I5.set_mode(self.mode)
        #print "set operation mode as {0} !".format(type_[self.mode])
        #del type_

    def sent_joint_position(self,position,velocity = [0.03,0.03,0.03,0.03,0.03]):
        """
        transfer point to control climb robot.
        :param position: control message.
        :param velocity: default velocity.
        """
        if self.mode == 1:
            self.I1.sent_position(position[0], velocity[0])
            self.T2.sent_position(position[1], velocity[1])
            self.T3.sent_position(position[2], velocity[2])
            self.T4.sent_position(position[3], velocity[3])
            self.I5.sent_position(position[4], velocity[4])

    def sent_joint_velocity(self,velocity):
        """
        transfer velocity to control climb robot.
        :param velocity: control message
        """
        if self.mode == 3:
            self.I1.sent_velocity(velocity[0])
            self.T2.sent_velocity(velocity[1])
            self.T3.sent_velocity(velocity[2])
            self.T4.sent_velocity(velocity[3])
            self.I5.sent_velocity(velocity[4])

    def sent_joint_torque(self,torque):
        """
        transfer torque to control climb robot
        :param torque: control message
        """
        if self.mode == 4:
            self.I1.sent_torque(torque[0])
            self.T2.sent_torque(torque[1])
            self.T3.sent_torque(torque[2])
            self.T4.sent_torque(torque[3])
            self.I5.sent_torque(torque[4])

    def sent_G0_torque(self,torque):
        """
        :param torque:
        :return:
        """
        self.G0.sent_torque(torque)

    def sent_G6_torque(self,torque):

        self.G6.sent_torque(torque)

    def get_position(self):
        """
        get the every joint position (rad)
        :return turtle(position joint1 ,.....)
        """
        position = (self.G0.get_position(),self.I1.get_position(),self.T2.get_position(),self.T3.get_position(), \
                self.T4.get_position(),self.I5.get_position(),self.G6.get_position())
        return position

    def get_velocity(self):
        """
        get the every joint velocity (rad/s)
        :return (velocity joint1 ,.....)
        """
        velocity = (self.G0.get_velocity(),self.I1.get_velocity(),self.T2.get_velocity(),self.T3.get_velocity(), \
                    self.T4.get_velocity(),self.I5.get_velocity(),self.G6.get_velocity())
        return velocity

    def get_torque(self):
        """
        get the every joint torque (# rate torque(mN.m) /1000)
        :return (torque joint1 ,.....)
        """
        torque = (self.G0.get_torque(),self.I1.get_torque(),self.T2.get_torque(),self.T3.get_torque(), \
                    self.T4.get_torque(),self.I5.get_torque(),self.G6.get_torque())
        return  torque

    def get_current(self):
        """
        get the every joint current (mA)
        :return (current joint1 ,.....)
        """
        current = (self.G0.get_current(),self.I1.get_current(),self.T2.get_current(),self.T3.get_current(), \
                    self.T4.get_current(),self.I5.get_current(),self.G6.get_current())
        return current

    def stop(self):
        """
        stop the communication between climbot and canopen
        """
        self.G0.stop()
        self.I1.stop()
        self.T2.stop()
        self.T3.stop()
        self.T4.stop()
        self.I5.stop()
        self.G6.stop()
        #print "stop the communication!"