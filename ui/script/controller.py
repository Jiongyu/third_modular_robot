#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import radians

class Controller(object):
    jointPosCommand = []

    jointPosCommand = []
    oldJointForceState = []

    dampingVector = []

    def __init__(self, initDamping = 200, gainBp = 13.3, gainBv = 0.7):
        self.__initDamping = initDamping
        self.__gainBp = gainBp
        self.__gainBv = gainBv
        self.__jointNum = 5
        self.__timeInterval = 0.02
        self.__referenceToruqe = [0] * self.__jointNum

        Controller.jointPosCommand = [0] * self.__jointNum
        Controller.dampingVector = [0] * self.__jointNum
        self.__referVelcocity = [radians(0)] * self.__jointNum

        self.__torque_error = 0
        self.__reference_acc = 0

        self.__torque_threshold = 10
        pass

    def getPosComd(self, reference_torque, actualJointForceState, actualVelocity):
        # actualJointForceState: [I1, T2, T3, T4, I5] actual force 

        for i in range(self.__jointNum):

            self.__torque_error = (reference_torque[i] - actualJointForceState[i])
            if(abs(self.__torque_error) > self.__torque_threshold):
                Controller.dampingVector[i] =   self.__initDamping - self.__gainBp * ( self.__torque_error )  \
                                                - self.__gainBv * (self.__torque_error) / self.__timeInterval

                self.__reference_acc = self.__torque_error - (Controller.dampingVector[i]) * (self.__referVelcocity[i] - actualVelocity[i])

                Controller.jointPosCommand[i] = actualVelocity[i] + self.__reference_acc * self.__timeInterval 
            else:
                Controller.jointPosCommand[i] = 0

        return  Controller.jointPosCommand
