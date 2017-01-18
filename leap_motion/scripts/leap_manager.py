#!/usr/bin/env python
# coding: utf-8
import argparse
import rospy
import datetime
import leap_interface

from datetime import timedelta
from leap_motion.msg import leap
from leap_motion.msg import leapros
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class Leap_Manager(object):
    def __init__(self):

        self.com_relative_mode = 1
        self.time_data = datetime.datetime.now()
        self.pub = rospy.Publisher('leapmotion/order', Twist, queue_size=1)
        rospy.Subscriber("leapmotion/data", leapros, self.callback_listener)

        '''self.leap_origin = Vector3(
        leap_origin.x=0,
        leap_origin.y=0,
        leap_origin.z=0
        )'''
        self.twist_msg = Twist(
            linear=Vector3(
                x=0,
                y=0,
                z=0
            ),
            angular=Vector3(
                x=0,  # loop
                y=0,  # takeoff
                z=0   # land
            )
        )

        self.old_hand_pos = Vector3(
            x=0,
            y=0,
            z=0
        )

        self.delta_pos = Vector3(
            x=0,
            y=0,
            z=0
        )

    def callback_listener(self, data):

        '''if start position not initialized'''
        #
        '''TO DO'''

        # if command mode set to relative and not to absolute
        if self.com_relative_mode:

            # establish hand position delta since last time
            self.delta_pos.x = data.palmpos.x - self.old_hand_pos.x
            self.delta_pos.y = data.palmpos.y - self.old_hand_pos.y
            self.delta_pos.z = data.palmpos.z - self.old_hand_pos.z

            # update hand position memory
            self.old_hand_pos.x = data.palmpos.x
            self.old_hand_pos.y = data.palmpos.y
            self.old_hand_pos.z = data.palmpos.z

            # calculate time since previous last publish from leap
            delta_time = (datetime.datetime.now() - self.time_data)

            # update time memory
            self.time_data = datetime.datetime.now()

            # create hand speed vector
            linear_vector = Vector3(
                x=(self.delta_pos.x/delta_time.microseconds),  # left/right
                y=(self.delta_pos.y/delta_time.microseconds),  # forward/rearward
                z=(self.delta_pos.z/delta_time.microseconds)  # upward/downward
            )

            angular_vector = Vector3(
                x=0,
                y=0,
                z=0
            )

            print("\n\n[LEAP_MANAGER] roll_value: %s\n\n") % (data.finger_roll)

            if data.finger_roll == 1:
                angular_vector.y = 1
            elif data.finger_roll == -1:
                angular_vector.x = 1

            # create message to be published
            message = Twist(linear=linear_vector, angular=angular_vector)

            # publish message to orchestrator
            self.pub.publish(message)
def listener():
    rospy.init_node('leap_manager', anonymous=True)

    leap_manager = Leap_Manager()

    print("Leap_manager init\n")

    rospy.spin()


if __name__ == '__main__':
    listener()
