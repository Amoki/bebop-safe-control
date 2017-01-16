#!/usr/bin/env python
# coding: utf-8
import rospy
import math
import itertools
import yaml
from os.path import dirname, abspath

from nav_msgs.srv import GetMap
from geometry_msgs.msg import Point
from orchestrator.srv import GetNearestObstacle

import timeit


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

            with open(dirname(dirname(abspath(__file__))) + rospy.get_param('map_path'), 'r') as stream:
                map_details = yaml.load(stream)
                self.anchors = map_details["anchors"]

        except rospy.ServiceException as e:
            print "Service call failed: %s" % e
        except yaml.YAMLError as exc:
            print(exc)

    def map_index(self, i, j):
        return i + j * self.info.width

    def handle_get_nearest_obstacle(self, req):
        start_time = timeit.default_timer()
        position = req.position
        resolution = self.info.resolution
        relative_x = position.x / resolution
        relative_y = position.y / resolution
        relative_z = position.z / resolution

        # optimisations:
        min_dist = float('inf')
        obstacle_x = float()
        obstacle_y = float()
        dist_sq = float()
        data = self.data
        map_index = self.map_index

        # loop over each pixel of the map
        for i, j in itertools.product(xrange(self.info.width), xrange(self.info.height)):
            if data[map_index(i, j)] == 100:
                dist_sq = (i - relative_x) ** 2 + (j - relative_y) ** 2
                if dist_sq < min_dist:
                    min_dist = dist_sq
                    obstacle_x = i
                    obstacle_y = j

        min_dist = math.sqrt(min_dist)
        obstacle_x = obstacle_x * resolution
        obstacle_y = obstacle_y * resolution
        obstacle_z = relative_z * resolution  # the z of the drone itself

        if self.roof - relative_z < min_dist:
            min_dist = self.roof - relative_z
            obstacle_x = position.x  # the x of the drone
            obstacle_y = position.y  # the y of the drone
            obstacle_z = self.ROOF_CM  # the z of the roof

        print timeit.default_timer() - start_time
        return Point(
            x=obstacle_x,
            y=obstacle_y,
            z=obstacle_z
        )


def run():
    rospy.init_node('get_nearest_obstacle')
    map_handler = Map()
    rospy.Service('get_nearest_obstacle', GetNearestObstacle, map_handler.handle_get_nearest_obstacle)
    rospy.spin()


if __name__ == "__main__":
    run()
