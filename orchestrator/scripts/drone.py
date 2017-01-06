#!/usr/bin/env python
# coding: utf-8
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

DRONE_RADIUS = 0.16  # m


class Drone(object):
    position = Point()
    orientation = Quaternion()
    size = DRONE_RADIUS

    def __init__(self, x, y, z):
        self.position.x = x
        self.position.y = y
        self.position.z = z
