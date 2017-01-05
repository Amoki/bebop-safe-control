#!/usr/bin/env python
# coding: utf-8
import rospy
import math
from nav_msgs.srv import GetMap


class Map(object):
    data = None
    info = None
    ROOF_CM = 2.5  # cm
    roof = None

    def __init__(self):
        rospy.wait_for_service('static_map')
        try:
            get_map = rospy.ServiceProxy('static_map', GetMap)
            retrieved_map = get_map()
            self.data = retrieved_map.map.data
            self.info = retrieved_map.map.info

            self.roof = self.ROOF_CM / self.info.resolution
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def map_index(self, i, j):
        return i + j * self.info.width

    def map_wxgx(self, i):
        return self.info.origin.position.x + (i - self.info.width / 2) * self.info.resolution

    def map_wygy(self, j):
        return self.info.origin.position.y + (j - self.info.height / 2) * self.info.resolution

    def get_nearest_obstacle(self, position):
        relative_x = position.x / self.info.resolution
        relative_y = position.y / self.info.resolution
        relative_z = position.z / self.info.resolution

        min_dist = float('inf')
        obstacle_x = None
        obstacle_y = None

        for j in range(0, self.info.height):
            for i in range(0, self.info.width):
                color = self.data[self.map_index(i, j)]
                if color == 100:
                    w_x = self.map_wxgx(i)
                    w_y = self.map_wygy(j)
                    dist_sq = pow(i - relative_x, 2) + pow(j - relative_y, 2)
                    if dist_sq < min_dist:
                        min_dist = dist_sq
                        obstacle_x = i
                        obstacle_y = j

        min_dist = math.sqrt(min_dist)
        obstacle_x = obstacle_x * self.info.resolution
        obstacle_y = obstacle_y * self.info.resolution
        obstacle_z = relative_z * self.info.resolution  # the z of the drone itself

        if self.roof - relative_z < min_dist:
            min_dist = self.roof - relative_z
            obstacle_x = position.x  # the x of the drone
            obstacle_y = position.y  # the y of the drone
            obstacle_z = self.ROOF_CM  # the z of the roof

        return (min_dist, obstacle_x, obstacle_y, obstacle_z)
