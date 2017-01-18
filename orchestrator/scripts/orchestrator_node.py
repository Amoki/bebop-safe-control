#!/usr/bin/env python
# coding: utf-8
import rospy
import math
import itertools
import datetime

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from nav_msgs.srv import GetMap
from orchestrator.srv import GetNearestObstacles
from orchestrator.msg import DronePosition

from drone import Drone
from order import Order


class Orchestrator(object):
    def __init__(self):
        # Wait for services first
        rospy.wait_for_service('get_nearest_obstacles')
        self.get_nearest_obstacles = rospy.ServiceProxy('get_nearest_obstacles', GetNearestObstacles)

        rospy.wait_for_service('static_map')
        get_map = rospy.ServiceProxy('static_map', GetMap)
        retrieved_map = get_map()

        self.map_resolution = retrieved_map.map.info.resolution
        self.position_resolution = 0.001
        self.take_off_date = datetime.datetime.now()

        # TODO: use launch config
        self.drones = [Drone(0x683D, 47.0, 40.0, 1.0), Drone(0x683D, 47.0, 40.0, 1.0), Drone(0x683D, 47.0, 40.0, 1.0)]

        # Communication with rviz
        self.pub_collision_point1 = rospy.Publisher('collision_point1', PointStamped, queue_size=10)
        self.pub_collision_point2 = rospy.Publisher('collision_point2', PointStamped, queue_size=10)
        self.pub_collision_point3 = rospy.Publisher('collision_point3', PointStamped, queue_size=10)
        rospy.Subscriber("/clicked_point", PointStamped, self.callback_clicked_point)

        rospy.Subscriber("/position", DronePosition, self.callback_position)
        rospy.Subscriber("leapmotion/order", Twist, self.callback_order)

        print("Orchestrator init\n")

    def will_collide(self):
        '''
        return True if the future position of the drone is to close to a wall
        return False if there is no expected collision
        '''

        # Check collision with wall
        obstacle_positions = self.get_nearest_obstacles(*[d.future_position for d in self.drones])
        positions = [obstacle_positions.drone1, obstacle_positions.drone2, obstacle_positions.drone3]

        will_wall_collide = False
        for drone, obstacle_position in zip(self.drones, positions):
            if self.get_distance(drone.future_position, obstacle_position) <= drone.size:
                will_wall_collide = True

        self.publish_collisions(positions)

        # check collision with one another drone
        will_drone_collide = False
        for a, b in itertools.combinations(self.drones, 2):
            # loop over drone 2 by 2 without repetition
            if self.get_distance(a.position, b.position) <= a.size + b.size:
                will_drone_collide = True
                break

        return will_wall_collide #or will_drone_collide

    def get_distance(self, a, b):
        return math.sqrt(
            (b.x - a.x) ** 2 +
            (b.y - a.y) ** 2 +
            (b.z - a.z) ** 2)

    def publish_collisions(self, positions):
        '''
        debug methode for map visualisation
        '''
        self.pub_collision_point1.publish(PointStamped(
            point=positions[0],
            header=Header(seq=0, frame_id='map', stamp=rospy.Time.now())
        ))
        # self.pub_collision_point2.publish(PointStamped(
        #     point=positions[1],
        #     header=Header(seq=0, frame_id='map', stamp=rospy.Time.now())
        # ))
        # self.pub_collision_point3.publish(PointStamped(
        #     point=positions[2],
        #     header=Header(seq=0, frame_id='map', stamp=rospy.Time.now())
        # ))

    def publish_future_pose(self):
        '''
        debug method for map visualisation
        '''
        for drone in self.drones:
            drone.publish_future_pose()

    def publish_pose(self):
        '''
        debug method for map visualisation
        '''
        for drone in self.drones:
            drone.publish_pose()

    def ajust_position_resolution(self, position):
        return Point(
            x=position.x * self.position_resolution,
            y=position.y * self.position_resolution,
            z=position.z * self.position_resolution,
        )

    def callback_position(self, drone_position):
        '''
        Callback called on new pozyx position
        '''
        position_id = int(drone_position.id.data)
        for drone in self.drones:
            if drone.id == position_id:
                drone.position = self.ajust_position_resolution(drone_position.position.pose.position)
                drone.orientation = drone_position.position.pose.orientation
        self.publish_pose()

    def callback_order(self, twist):
        '''
        Callback called on new leap order is received
        '''

        print("\n=============================================")
        print("Order received, drone is in the air: %s\n" % (self.drones[0].in_the_air))
        order = Order(twist)

        # Check if special is land or take off
        # if x == 1 Take off !
        if order.angular.x == 1:
            print("Special order detected: Take off")
            for drone in self.drones:
                drone.take_off()
                self.take_off_date = datetime.datetime.now()

        # if y == 1 Land !
        elif order.angular.y == 1:
            print("Special order detected: Land")
            for drone in self.drones:
                drone.land()
        else:
            if any([drone.in_the_air is True for drone in self.drones]):
                # if take_off is over
                if (datetime.datetime.now() - self.take_off_date).seconds >= 4:
                    # Look for move order
                    distance_twist = order.transform_to_distance_twist()
                    for drone in self.drones:
                        drone.future_position = Point(
                            x=drone.position.x + distance_twist.linear.x,
                            y=drone.position.y + distance_twist.linear.y,
                            z=drone.position.z + distance_twist.linear.z
                        )

                    if self.will_collide():
                        print('Unable to execute this order\n')
                        # Send stabilization order
                        for drone in self.drones:
                            drone.send_bebop_order(Twist())
                    else:
                        print("Order execution validated no collision point detected \n")
                        bebop_twist = order.transform_to_bebop_twist()
                        print("Sending movement order to bebop\n")
                        for drone in self.drones:
                            drone.send_bebop_order(bebop_twist)

    def callback_clicked_point(self, point_stamped):
        '''
        Debug only
        Callback called on new publish point on rviz position
        '''
        self.drones[0].future_position = Point(
            x=point_stamped.point.x,
            y=point_stamped.point.y,
            z=point_stamped.point.z
        )

        # self.drones[1].future_position = Point(
        #     x=point_stamped.point.x,
        #     y=point_stamped.point.y,
        #     z=point_stamped.point.z
        # )

        # self.drones[2].future_position = Point(
        #     x=point_stamped.point.x + 2,
        #     y=point_stamped.point.y,
        #     z=point_stamped.point.z
        # )

        self.publish_future_pose()
        self.will_collide()


def run():
    rospy.init_node('orchestrator')

    Orchestrator()

    rospy.spin()


if __name__ == '__main__':
    run()
