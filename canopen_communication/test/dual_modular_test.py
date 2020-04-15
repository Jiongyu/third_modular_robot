import sys
sys.path.append("./../modular")
import pdb
from math import  fabs

from  modular_I100 import I100
from  modular_T100 import T100

class Dual_modular_test(object):

    def __init__(self,eds_file):
        self.eds_file = eds_file
        self.I1 = I100(1,self.eds_file)
        self.T2 = T100(2,self.eds_file)

    def start(self):
        self.I1.start()
        self.T2.start()

    def set_mode(self,mode):

        self.mode = mode
        type_ = {1:"profiled position",3:"Profiled Velocity",4:"Profiled Torque"}
        self.I1.set_mode(self.mode)
        self.T2.set_mode(self.mode)

    def sent_joint_position(self,position,velocity = [0.02,0.02]):
        if self.mode == 1:
            #self.I1.node.sdo[0x6040].bits[4] = 0
            #self.T2.node.sdo[0x6040].bits[4] = 0
            self.I1.sent_position(position[0], velocity[0])
            self.T2.sent_position(position[1], velocity[1])
            self.I1.node.sdo[0x6040].bits[4] = 1
            self.T2.node.sdo[0x6040].bits[4] = 1
            self.I1.node.sdo[0x6040].bits[4] = 0
            self.T2.node.sdo[0x6040].bits[4] = 0
            self.I1.node.sdo[0x6040].bits[4] = 1
            self.T2.node.sdo[0x6040].bits[4] = 1
            self.I1.node.sdo[0x6040].bits[5] = 1
            self.T2.node.sdo[0x6040].bits[5] = 1
            self.I1.node.sdo[0x6040].bits[6] = 0
            self.T2.node.sdo[0x6040].bits[6] = 0

        ''''
        error = 1e-3
        while 1:
            if ((fabs(self.I1.get_position() - position[0]) < error) and\
              (fabs(self.T2.get_position() - position[1]) < error)):
                self.I1.node.sdo[0x6040].bits[4] = 0
                self.T2.node.sdo[0x6040].bits[4] = 0
                break
            ####feedback#####

            #################

        del error
        '''

    def sent_joint_velocity(self,velocity):
        """
        transfer velocity to control climb robot.
        :param velocity: control message
        """
        if self.mode == 3:
            self.I1.sent_velocity(velocity[0])
            self.T2.sent_velocity(velocity[1])

    def sent_joint_torque(self,torque):
        """
        transfer torque to control climb robot
        :param torque: control message
        """
        if self.mode == 4:
            self.I1.sent_torque(torque[0])
            self.T2.sent_torque(torque[1])

    def get_position(self):
        """
        get the every joint position (rad)
        :return turtle(position joint1 ,.....)
        """
        position = (self.I1.get_position(),self.T2.get_position())
        return position

    def get_velocity(self):
        """
        get the every joint velocity (rad/s)
        :return (velocity joint1 ,.....)
        """
        velocity = (self.I1.get_velocity(),self.T2.get_velocity())
        return velocity

    def get_torque(self):
        """
        get the every joint torque (# rate torque(mN.m) /1000)
        :return (torque joint1 ,.....)
        """
        torque = (self.I1.get_torque(),self.T2.get_torque())
        return  torque

    def get_current(self):
        """
        get the every joint current (mA)
        :return (current joint1 ,.....)
        """
        current = (self.I1.get_current(),self.T2.get_current())
        return current

    def stop(self):
        """
        stop the communication between climbot and canopen
        """
        self.I1.stop()
        self.T2.stop()

        print "stop the communication!"
