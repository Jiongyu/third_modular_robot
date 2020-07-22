#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#import can
import canopen

class Canopen_control_init(object):
    """
    motor control by copley with canopen
    """
    def __init__(self):
        self.network = canopen.Network()
        self.network.connect(channel='can0', bustype='socketcan')
        self.network.check()
        self.network.sync.start(0.015)
        # print("canopen init is over!")