#!/usr/bin/env python
# coding: utf-8
import rospy
import math
import itertools
import yaml
from os.path import dirname, abspath

from nav_msgs.srv import GetMap
from geometry_msgs.msg import Point
from orchestrator.srv import GetNearestObstacles

import timeit


class MappedDrone(object):
    def __init__(self, position, resolution):
        self.position = position
        self.relative_position = Point(
            x=position.x / resolution,
            y=position.y / resolution,
            z=position.z / resolution
        )
        self.obstacle_position = Point()
        self.min_distance = float('inf')


class Map(object):
    data = None
    info = None
    ROOF_CM = 2.5  # m
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

    def handle_get_nearest_obstacles(self, req):
        start_time = timeit.default_timer()

        resolution = self.info.resolution

        drones = [
            MappedDrone(req.drone1, resolution),
            MappedDrone(req.drone2, resolution),
            MappedDrone(req.drone3, resolution)
        ]

        # optimisations:
        dist_sq = float()
        data = self.data
        map_index = self.map_index

        # loop over each pixel of the map
        for i, j in itertools.product(xrange(self.info.width), xrange(self.info.height)):
            if data[map_index(i, j)] == 100:
                for drone in drones:
                    # get distance between this point and the drone
                    dist_sq = (i - drone.relative_position.x) ** 2 + (j - drone.relative_position.y) ** 2
                    # if this distance is the shortest
                    if dist_sq < drone.min_distance:
                        drone.min_distance = dist_sq
                        drone.obstacle_position.x = i
                        drone.obstacle_position.y = j

        for drone in drones:
            drone.min_distance = math.sqrt(drone.min_distance)
            drone.obstacle_position.x = drone.obstacle_position.x * resolution
            drone.obstacle_position.y = drone.obstacle_position.y * resolution
            drone.obstacle_position.z = drone.relative_position.z * resolution  # the z of the drone itself

            if self.roof - drone.relative_position.z < drone.min_distance:
                drone.min_distance = self.roof - drone.relative_position.z
                drone.obstacle_position.x = drone.position.x  # the x of the drone
                drone.obstacle_position.y = drone.position.y  # the y of the drone
                drone.obstacle_position.z = self.ROOF_CM  # the z of the roof

        print timeit.default_timer() - start_time

        return [drone.obstacle_position for drone in drones]


def run():
    rospy.init_node('get_nearest_obstacles')
    map_handler = Map()
    rospy.Service('get_nearest_obstacles', GetNearestObstacles, map_handler.handle_get_nearest_obstacles)
    rospy.spin()


if __name__ == "__main__":
    run()
