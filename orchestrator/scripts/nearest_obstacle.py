#!/usr/bin/env python
# coding: utf-8

import rospy


def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('get_nearest_obstacle', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()
