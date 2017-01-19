#!/usr/bin/env python
# coding: utf-8

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import Header

BEBOP_RADIUS = 0.50  # m


class Drone(object):
    DRONE_INDEX = 0
    position = Point()
    orientation = Quaternion()

    future_position = Point()
    future_orientation = Quaternion()

    size = BEBOP_RADIUS
    in_the_air = False

    def __init__(self, id, x, y, z):
        Drone.DRONE_INDEX += 1
        index = Drone.DRONE_INDEX
        self.pub_position = rospy.Publisher('drone_pose%d' % index, PointStamped, queue_size=10)
        self.pub_take_off = rospy.Publisher('bebop%d/takeoff' % index, Empty, queue_size=10)
        self.pub_land = rospy.Publisher('bebop%d/land' % index, Empty, queue_size=10)
        self.pub_bebop = rospy.Publisher('bebop%d/cmd_vel' % index, Twist, queue_size=10)

        self.position.x = x
        self.position.y = y
        self.position.z = z
        self.id = id

    def __str__(self):
        return '%s' % self.position

    def publish_future_pose(self):
        self.pub_position.publish(PointStamped(
            point=self.future_position,
            header=Header(seq=0, frame_id='map', stamp=rospy.Time.now())
        ))

    def publish_pose(self):
        self.pub_position.publish(PointStamped(
            point=self.position,
            header=Header(seq=0, frame_id='map', stamp=rospy.Time.now())
        ))

    def take_off(self):
        if self.in_the_air is False:
            self.in_the_air = True
            # Transmit take off order
            self.pub_take_off.publish()
        else:
            print("Drone is already in the air")

    def land(self):
        if self.in_the_air is True:
            self.in_the_air = False
            # Transmit land order
            self.pub_land.publish()
        else:
            print("Drone is already landed")

    def send_bebop_order(self, twist):
        self.pub_bebop.publish(twist)
