#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*-
# import sys

import time
from math import radians, degrees
from canopen import BaseNode402
from canopen import network
# import pdb

class Modular_joint(object):
    """
    Modular_joint control base on the canopen
    """

    def __init__(self, id, eds_file, reduction_ratio):
        """
        add node to the can
        :param id: node id
        :param eds_file: the location of the eds file
        :return:
        """
        self.__network = network.Network()
        self.__network.connect(channel='can0', bustype='socketcan')
        self.__network.check()
        self.__network.sync.start(0.003)        # 5ms
        self.__id = id
        self.__eds_file = eds_file
        self.__node = BaseNode402(self.__id, self.__eds_file)
        self.__network.add_node(self.__node)
        self.__reduction_ratio = reduction_ratio
        self.__velocity_limit = 30

    def start(self):
        """
        start communication
        """
        print "Motor Start."
        # Reset network
        self.__node.nmt.state = 'RESET'
        self.__node.nmt.state = 'RESET COMMUNICATION'
        self.__node.nmt.wait_for_bootup(10)
        self.__node.setup_402_state_machine()

        self.__node.reset_from_fault()

        self.__node.sdo[0x6076].phys = 1000
        self.__motor_rate_torque    = self.__node.sdo[0x6076].phys  #mN.m
        self.__node.sdo[0x6410][0x0c].phys = 91
        self.__torque_constant      = self.__node.sdo[0x6410][0x0c].phys  # 0.001 N.m/A
        # self.__motor_continuous_torque = self.__node.sdo[6410][0x0e].phys # 0.001 N.m/A
        print "motor rate torque: {0}".format(self.__motor_rate_torque)
        print "motor torque constant: {0}".format(self.__torque_constant)
        # print "motor continuous torque: {0}".format(self.__motor_continuous_torque)

        self.__controlword = 0x001f
        self.__node.controlword = self.__controlword
        self.__controlword = 0x000f
        self.__node.controlword = self.__controlword
        self.__node.controlword = ( self.__controlword | (1 << 6) )

        self.__param_config()


        print "Joint {0} Initialization Complete.".format(self.__id)

    def stop_communication(self):
        """
        stop communication
        """

        self.serve_off()
        print "Joint {0} Stop.".format(self.__id)
        self.__node.nmt.state = 'PRE-OPERATIONAL'
        print('node {1} state 5) = {0}'.format(self.__node.nmt.state, self.__id))
        del self.__network[self.__id]
        self.__network.sync.stop()
        self.__network.disconnect()
        print "Joint {0} Stop success.".format(self.__id)

    def get_operation_mode(self):
        return self.__node.op_mode
        pass

    def quick_stop(self):
        self.__node.sdo[0x6040].raw = (self.__node.sdo[0x6040].raw & ~( 1 << 2 ))

    def stop(self):
        self.__node.state = 'SWITCHED ON'

    def pause_run(self):
        self.__controlword = self.__node.sdo[0x6040].raw
        self.__node.controlword = (self.__controlword | ( 1 << 8))
        pass

    def continue_run(self):
        self.__node.controlword = ( self.__controlword & ~(1 << 4) )
        self.__node.controlword = ( self.__controlword | (3  << 4) )
        self.__node.controlword = ( self.__controlword & ~(1 << 4) )
        self.__controlword = self.__node.sdo[0x6040].raw
        self.__node.controlword = ( self.__controlword & ~(1 << 8))
        pass

    def __user_data_to_motor(self, position):
        """
         resolution of encoder * reduction ratio / 360 degree
        :return:
        """
        return  int(position * 4096 * self.__reduction_ratio / 360)
        # return  int(degrees(position) * 4096 * 188 / 360)

    def __motor_data_to_user(self, position):
        """
        actual position [count] -> deg
        :return:
        """
        return round((position * 360 / self.__reduction_ratio / 4096.0),3)

    def __motor_torque_to_user(self,torque):
        return torque * 1000 / float(self.__torque_constant)

    def __user_current_to_motor(self, current):
        return (current / 1000.0) * self.__torque_constant

    def controlword(self,data):
        self.__controlword = data
        self.__node.controlword = self.__controlword
        pass

    def opmode_set(self,data):
        # if self.__node.op_mode == data:
        #     return
        self.__node.op_mode = data
        while( self.__node.op_mode != data ):
            self.__node.op_mode = data
            time.sleep(0.01)

    def set_mode(self, mode):
        """
        :param mode: motor operation mode(1,3,4)
        1: profiled position
        3: Profiled Velocity
        4: Profiled Torque
        """
        if (mode == 1 or mode == 3 or mode == 4):
            try:
                self.node.sdo[0x6060].phys = mode
            except:
                print "set mode error"

    def sent_position(self,pos, vel):
        """
        In the profile position mode this function sent some control message to motor.
        :param position: motor position(rad)
        :param velocity: default motor velocity(rad/s)
        """
        if abs(vel) < self.__velocity_limit:
            self.__node.sdo[0x6081].phys = abs(self.__user_data_to_motor( vel * 10))
            self.__node.sdo[0x607a].phys = self.__user_data_to_motor( pos )
            self.__node.controlword = ( self.__controlword & ~(1 << 4) )
            self.__node.controlword = ( self.__controlword | (3  << 4) )
            self.__node.controlword = ( self.__controlword & ~(1 << 4) )


    def sent_velocity(self,data):
        """
         In the profile velocity mode this function sent some control message to motor.
         :param velocity: motor velocity(rad/s)
         :return:
         """
        if abs(data) < self.__velocity_limit:
            self.__node.sdo[0x60ff].phys = self.__user_data_to_motor( data * 10 )

    def sent_current(self,data):
        """
            In the profile torque mode this function sent some control message to motor.
            :param data: motor current(mA)
            :return:
            """
        self.__node.sdo[0x6071].phys = self.__user_current_to_motor( data )

    # 轨迹模式设置加减速度
    def set_path_mode_acc_dcc(self):
        self.__path_mode_acc_dcc()

    # 正常位置速度模式加减速度设置
    def set_normal_mode_acc_dcc(self):
        self.__normal_mode_acc_dcc()

    def statusword(self):
        return self.__node.statusword

    def opmode_read(self):
        return  self.__node.op_mode

    def get_position(self):
        """
           get the motor actual value
           :return position(rad)
           """
        return (self.__motor_data_to_user( self.__node.sdo[0x6064].phys ))

    def get_velocity(self):
        """
        get the motor actual value
        :return velocity(rad/s)
        """
        return ( self.__motor_data_to_user( self.__node.sdo[0x606c].phys ) / 10)

    def get_current(self):
        """
        get the motor current actual value
        :return current(mA)
        """
        # return ( self.__motor_torque_to_user( self.__node.sdo[0x6077].phys))
        return self.__node.sdo[0x221c].phys  # mA
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
            print "Serve On ..."

    def serve_off(self):
        print "Serve Off."
        self.__node.state = 'SWITCH ON DISABLED'

    def __del__(self):
        pass

    def reached(self):
        return ( self.__node.sdo[0x6041].bits[10] )

    def __param_config(self):
        # pos loop
        self.__normal_mode_acc_dcc()
        self.__node.sdo[0x6086].phys =  0       # rotate motor

        # 急停加减速度
        self.__node.sdo[0x605a].phys = 0x03
        # 暂停加减速度
        self.__node.sdo[0x605d].phys = 0x02
        # motor data
        # self.__node.sdo[0x6410][0x0b].phys = 45000     # max velocity 0.1counts/s
        self.__node.sdo[0x6410][0x0d].phys = 1200      # max torque mN.m

    # 轨迹模式下加减速设置
    def __path_mode_acc_dcc(self):
        # unit 10 counts/sec2
        self.__node.sdo[0x6083].phys =  500
        self.__node.sdo[0x6084].phys =  500
        pass

    # 正常位置，速度模式加减速设置
    def __normal_mode_acc_dcc(self):
        # unit 10 counts/sec2
        self.__node.sdo[0x6083].phys =  5000
        self.__node.sdo[0x6084].phys =  5000
        pass