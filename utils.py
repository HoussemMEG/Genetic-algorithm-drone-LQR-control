from math import pi
def deg2rad(list1):
    list1[0] = list1[0] / 180 * pi
    list1[1] = list1[1] / 180 * pi
    list1[2] = list1[2] / 180 * pi
    return list1

from random import gauss
# print(gauss(97, 0.05))

