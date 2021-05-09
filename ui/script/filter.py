#!/usr/bin/env python
# -*- coding: utf-8 -*-

from copy import deepcopy

class Filter(object):
    
    def __init__(self,number = 20):
        self.__averageData = [0] * number
        self.__number = number
        self.__sumNum = 0

    def averageFilter(self,data):
        self.__sumNum -= self.__averageData[0]
        del(self.__averageData[0])
        self.__sumNum += data
        self.__averageData.append(data)
        return self.__sumNum / float(self.__number)

"""
def main():
    filter = Filter()

    print(filter.averageFilter(1))
    print(filter.averageFilter(1))
    print(filter.averageFilter(1))
    print(filter.averageFilter(1))
    print(filter.averageFilter(1))


if __name__ == "__main__":
    main()
"""