#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*-
# import sys

import time
from math import radians, degrees
from canopen import BaseNode402
from canopen import network
# import pdb

class G100():
    """
    G100 control base on the canopen
    """

    def __init__(self, id, eds_file):
        """
        add node to the can
        :param id: node id
        :param eds_file: the location of the eds file
        :return:
        """
        self.__network = network.Network()
        self.__network.connect(channel='can0', bustype='socketcan')
        self.__network.check()
        self.__network.sync.start(0.15)        # 5ms
        self.__id = id
        self.__eds_file = eds_file
        self.__node = BaseNode402(self.__id, self.__eds_file)
        self.__network.add_node(self.__node)

    def start(self):
        """
        start communication
        """
        print "Motor Start.\n"
        # Reset network
        self.__node.nmt.state = 'RESET'
        self.__node.nmt.state = 'RESET COMMUNICATION'
        self.__node.nmt.wait_for_bootup(10)
        # self.__node.setup_402_state_machine()

        # self.__controlword = 0x80
        # self.__node.controlword = self.__controlword
        # self.__controlword = 0x81
        # self.__node.controlword = self.__controlword
        # self.__node.reset_from_fault()
        # self.__node.setup_402_state_machine()
        self.__node.nmt.state = "OPERATIONAL"

        self.__motor_rate_current = self.__node.sdo[0x6076].phys  #mN.m
        self.__torque_constant = self.__node.sdo[0x6410][0x0c].phys  # 0.001 N.m/A
        # print "motor rate torque: {0}".format(self.__motor_rate_current)
        # print "motor torque constant: {0}".format(self.__torque_constant)

        # self.__controlword = 0x001f
        # self.__node.controlword = self.__controlword
        # self.__controlword = 0x000f
        # self.__node.controlword = self.__controlword
        # self.__node.controlword = ( self.__controlword | (1 << 6) )

        self.__param_config()
        self.__opmode = 4
        self.__node.sdo[0x6060].raw = self.__opmode
        
        while (self.__node.sdo[0x6061].raw != self.__opmode):
            time.sleep(0.1)

        print "Joint {0} Initialization Complete.\n".format(self.__id)

    def stop(self):
        """
        stop communication
        """

        self.serve_off()
        print "Joint {0} Stop.\n".format(self.__id)
        self.__node.nmt.state = 'PRE-OPERATIONAL'
        print('node {1} state 5) = {0}'.format(self.__node.nmt.state, self.__id))
        del self.__network[self.__id]
        self.__network.sync.stop()
        self.__network.disconnect()
        print "Joint {0} Stop success.\n".format(self.__id)

    def get_operation_mode(self):
        return self.__node.op_mode
        pass

    def quick_stop(self):
        self.__node.controlword = (self.__controlword & ~( 1 << 2 ))
        self.__node.nmt.state = 'PRE-OPERATIONAL'
        # del self.__network[self.__id]
        # self.__network.sync.stop()
        # self.__network.disconnect()

    def pause_run(self):
        self.__node.controlword = (self.__controlword | ( 1 << 8))
        pass

    def continue_run(self):
        self.__node.controlword = ( self.__controlword & ~(1 << 8))
        pass

    def __motor_torque_to_user(self,torque):
        return (torque * self.__motor_rate_current / 1000)

    def __user_torque_to_motor(self, torque):
        return (torque * 1000 / self.__motor_rate_current )

    def controlword(self,data):
        self.__controlword = data
        self.__node.controlword = self.__controlword
        pass

    # def opmode_set(self,data):
    #     self.__node.op_mode = data
    #     while( self.__node.op_mode != data ):
    #         self.__node.op_mode = data
    #         time.sleep(0.01)

    def sent_torque(self,data):
        """
            In the profile torque mode this function sent some control message to motor.
            :param torque: motor torque()
            :return:
            """
        self.__node.sdo[0x6071].phys = self.__user_torque_to_motor( data )

    def statusword(self):
        return self.__node.statusword

    def opmode_read(self):
        return  self.__node.op_mode

    def get_torque(self):
        """
        get the motor actual value
        :return torque(rate torque(mN.m) /1000)
        """
        return ( self.__motor_torque_to_user( self.__node.sdo[0x6077].phys ))
        pass

    def serve_on(self):
        while(self.__node.state != 'OPERATION ENABLED'):
            if(self.__node.state == 'NOT READY TO SWITCH ON'):
                self.__node.state = 'SWITCH ON DISABLED'
            elif(self.__node.state == 'SWITCH ON DISABLED'):
                self.__node.state = 'READY TO SWITCH ON'
            elif(self.__node.state == 'READY TO SWITCH ON'):
                self.__node.state = 'SWITCHED ON'
            elif(self.__node.state == 'SWITCHED ON'):
                self.__node.state = 'OPERATION ENABLED'
            else:
                self.__node.reset_from_fault()
            time.sleep(0.01)    # 10ms
            print "Serve On ...\n"

    def serve_off(self):
        print "Serve Off.\n"
        self.__node.state = 'READY TO SWITCH ON'

    def __del__(self):
        pass

    def reached(self):
        return ( self.__node.sdo[0x6041].bits[10] )

    def __param_config(self):
        # pos loop
        self.__node.sdo[0x6083].phys =  500 * 4096 / 10     # acc 10 counts/s^2
        self.__node.sdo[0x6084].phys =  500 * 4096 / 10     # dcc 10 counts/s^2
        self.__node.sdo[0x6086].phys =  0                   # rotate motor

        # motor data
        self.__node.sdo[0x6410][0x0b].phys = 45000     # max velocity 0.1counts/s
        self.__node.sdo[0x6410][0x0d].phys = 1200      # max torque mN.m