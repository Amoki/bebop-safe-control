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

    def transform_to_bebop_twist(self, twist):
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
        dead_zone = 0.002

        print("Drone raw order is:\n")
        print("Leap_distance_order = [%2.8s;%2.8s;%2.8s][%2.8s;%2.8s;%2.8s]\n" % (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z
            ))

        #print("\nDrone raw order is Linear X(F/R): %2.8s\tLinear Y(L/R): %2.8s\tLinear Z(U/D): %2.8s\n") % (self.linear.x, self.linear.y, self.linear.z)

        if twist.linear.x < -dead_zone:
            print("Drone order is to go left")
            y = twist.linear.x*10
        elif twist.linear.x > dead_zone:
            print("Drone order is to go right")
            y = -twist.linear.x*10
        else:
            print("Drone order (L/R) is in deadzone %2.8s") % (twist.linear.x)
            y = 0

        if twist.linear.y > dead_zone:
            print("Drone order is to go forward")
            x = twist.linear.y*10
        elif twist.linear.y < -dead_zone:
            print("Drone order is to go backward")
            x = -twist.linear.y*10
        else:
            print("Drone order (F/B) is in deadzone %2.8s") % (twist.linear.y)
            x = 0

        if twist.linear.z > dead_zone:
            print("Drone order is to go up")
            z = twist.linear.z*10
        elif twist.linear.z < -dead_zone:
            print("Drone order is to go down")
            z = -twist.linear.z*10
        else:
            print("Drone order (U/D) is in deadzone %2.8s") % (twist.linear.z)
            z = 0

        linear_vector = Vector3(
            x=x,  # left/right
            y=y,  # forward/rearward
            z=z  # upward/downward
        )

        angular_vector = Vector3(
            x=0,
            y=0,
            z=0
        )

        bebop_twist = Twist(linear=linear_vector, angular=angular_vector)

        print("\nbebop_twist = [%2.8s;%2.8s;%2.8s][%2.8s;%2.8s;%2.8s]\n" % (
            linear_vector.y,
            linear_vector.x,
            linear_vector.z,
            angular_vector.x,
            angular_vector.y,
            angular_vector.z
            ))

        return bebop_twist
