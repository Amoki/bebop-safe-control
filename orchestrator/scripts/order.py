#!/usr/bin/env python
# coding: utf-8

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist


class Order(object):
    def __init__(self, twist):
        self.linear = twist.linear
        self.angular = twist.angular

    def transform_to_distance_twist(self):
        '''
        Transform the order message (with deltas value in axes) into expected
        tranlations in order to project the futur postion in a map
        '''

        linear = Vector3(
            x=self.linear.x * 5,
            y=self.linear.y * 5,
            z=self.linear.y * 5
        )

        angular = Vector3(
            x=self.angular.x,
            y=self.angular.y,
            z=self.angular.z,
        )

        distance = Twist(linear=linear, angular=angular)

        print("Order distance to travel is [%s;%s;%s]\n" % (
            distance.linear.x,
            distance.linear.y,
            distance.linear.z)
        )

        return distance

    def transform_to_bebop_twist(self):
        '''
        Turn the order message to bebop compatible twist message

        linear.x  (+)      Translate forward
                  (-)      Translate backward
        linear.y  (+)      Translate to left
                  (-)      Translate to right
        linear.z  (+)      Ascend
                  (-)      Descend

        angular.z (+)      Rotate counter clockwise
                  (-)      Rotate clockwise

        angular.x (1)       Take off
        angular.y (1)       Land

        Acceptable range for all fields are [-1;1]
        '''
        speed = 0.2

        if self.linear.y > 0:
            print("Drone order is to go left\n")
            y = speed
        elif self.linear.y < 0:
            print("Drone order is to go right\n")
            y = -speed
        else:
            y = 0

        if self.linear.x > 0:
            print("Drone order is to go forward\n")
            x = speed
        elif self.linear.x < 0:
            print("Drone order is to go backward\n")
            x = -speed
        else:
            x = 0

        if self.linear.z > 0:
            print("Drone order is to go up\n")
            z = speed
        elif self.linear.z < 0:
            print("Drone order is to go down\n")
            z = -speed
        else:
            z = 0

        print("\n")

        linear_vector = Vector3(
            x=x,  # left/right
            y=y,  # forward/rearward
            z=z  # upward/downward
        )

        print("Linear Vector3 initialized")

        angular_vector = Vector3(
            x=0,
            y=0,
            z=0
        )

        print("Angular Vector3 initialized")


        bebop_twist = Twist(linear=linear_vector, angular=angular_vector)

        print("bebop_twist = [%s;%s;%s][%s;%s;%s]\n" % (
            linear_vector.x,
            linear_vector.y,
            linear_vector.z,
            angular_vector.x,
            angular_vector.y,
            angular_vector.z
            ))

        return bebop_twist
