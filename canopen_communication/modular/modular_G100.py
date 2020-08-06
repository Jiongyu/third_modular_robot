#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*-
# import sys

import time
from math import radians, degrees
from canopen import BaseNode402
from canopen import network
# import pdb

class G100(object):
    """
    G100 control base on the canopen
    """

    def __init__(self,id,eds_file):
        """
        add node to the can
        :param id: node id
        :param eds_file: the location of the eds file
        :return:
        """
        self.__network = network.Network()
        self.__network.connect(channel='can0', bustype='socketcan')
        self.__network.check()
        self.__network.sync.start(0.015)
        self.__id = id
        self.__eds_file = eds_file
        self.__node = self.__network.add_node(self.__id, self.__eds_file)
        self.__mode = 4

    def start(self):
        """
        start communication
        """
        self.__node.nmt.state = 'RESET'
        self.__node.nmt.wait_for_bootup(10)

        error_log = self.__node.sdo[0x1003]
        for error in error_log.values():
            print("Error {0} was found in the log".format(error.raw))

        self.__node.nmt.state = "OPERATIONAL"
        print('node {1} state 4) = {0}'.format(self.__node.nmt.state, self.__id))


        self.__node.sdo[0x6076].phys = 1000
        self.__motor_rate_torque    = self.__node.sdo[0x6076].phys  #mN.m
        self.__node.sdo[0x6410][0x0c].phys = 91
        self.__torque_constant      = self.__node.sdo[0x6410][0x0c].phys  # 0.001 N.m/A
        self.__node.sdo[0x6410][0x0e].phys = 91
        self.__conti_torque_constant      = self.__node.sdo[0x6410][0x0e].phys  # 0.001 N.m/A
        # print "motor rate torque: {0}".format(self.__motor_rate_torque)
        # print "motor torque constant: {0}".format(self.__torque_constant)
        # print "motor continue torque constant: {0}".format(self.__conti_torque_constant)

        self.__node.sdo[0x6060].raw = self.__mode
        # print self.__node.sdo[0x6060].raw

    def sent_current(self,current):
        """
        In the profile torque mode this function sent some control message to motor.
        :param current: motor current()
        :return:
        """
        if self.__mode == 4:  # Profiled Torque
            self.__node.sdo[0x6071].phys = current * 10

    def get_current(self):
        """
        get the motor actual value
        :return current(mA)
        """
        return self.__node.sdo[0x221c].phys  # mA

    def stop_communication(self):
        """
        stop communication
        """
        self.__node.state = 'READY TO SWITCH ON'
        self.__node.nmt.state = 'PRE-OPERATIONAL'
        print('node {1} state 5) = {0}'.format(self.__node.nmt.state, self.__id))
        self.__network.sync.stop()
        self.__network.disconnect()

    def stop(self):
        self.__node.state = 'SWITCHED ON'
        
    def quick_stop(self):
        self.__node.sdo[0x6040].bits[2] = 0
        self.stop()

