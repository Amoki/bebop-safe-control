#!/usr/bin/env python
# coding: utf-8
import rospy
import math
from nav_msgs.srv import GetMap
import Image

class Map(object):
    data = None
    info = None

    def __init__(self):
        rospy.wait_for_service('static_map')
        try:
            get_map = rospy.ServiceProxy('static_map', GetMap)
            retrieved_map = get_map()
            self.data = retrieved_map.map.data
            self.info = retrieved_map.map.info
            print(self.info)
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
        min_dist = float('inf')
        obstacle_x = None
        obstacle_y = None

        im = Image.new('RGB', (self.info.width, self.info.height))

        for j in range(0, self.info.height):
            for i in range(0, self.info.width):
                color = self.data[self.map_index(i, j)]
                if j == 114 and i == 40:
                    print('aaaaaaa', color)
                if j == 115 and i == 40:
                    print('bbbbbbbbbbbbbb', color)
                if color == 100:
                    w_x = self.map_wxgx(i)
                    w_y = self.map_wygy(j)
                    dist_sq = pow(w_x - relative_x, 2) + pow(w_y - relative_y, 2)
                    if dist_sq < min_dist:
                        min_dist = dist_sq
                        obstacle_x = w_x
                        obstacle_y = w_y

                if color == 100:
                    color = 255
                if color == -1:
                    color = 128
                im.putpixel((i, j), (color, color, color))
        im.save('test.png')

        min_dist = math.sqrt(min_dist)
        obstacle_x = obstacle_x
        obstacle_y = obstacle_y

        return (min_dist, obstacle_x, obstacle_y)
