#!/usr/bin/env python
__author__ = 'flier'

import argparse

import rospy
import datetime
import leap_interface
from leap_motion.msg import leap
from leap_motion.msg import leapros
from geometry_msg.msg import Twist
from geometry_msg.msg import Vector3

NODENAME = 'hand_vect'

def init():
    time_mem=0

    leap_origin.x=0
    leap_origin.y=0
    leap_origin.z=0

    Twist_msg = Twist()
    Twist_msg.target_pos.x=0
    Twist_msg.target_pos.y=0
    Twist_msg.target_pos.z=0

    Twist_msg.action.loop=0
    Twist_msg.action.takeoff=0
    Twist_msg.action.land=0

    old_hand_pos.x=0
    old_hand_pos.y=0
    old_hand_pos.z=0

    com_relative_mode=1


def listener_callback(data):
    
    pub = rospy.Publisher('leapmotion/data',Twist,queue_size=1)
    rospy.Rate(40)
    rospy.init_node(NODENAME,anonymous=True)
    

    '''if start position not initialized'''
    #
    '''TO DO'''

    #if command mode set to relative and not to absolute
    if com_relative_mode:

        #establish hand position delta since last time
        delta_pos.x= data.palmpos.x - old_hand_pos.x
        delta_pos.y= data.palmpos.y - old_hand_pos.y
        delta_pos.z= data.palmpos.z - old_hand_pos.z

        #update hand position memory
        old_hand_pos.x = data.palmpos.x
        old_hand_pos.y = data.palmpos.y
        old_hand_pos.z = data.palmpos.z

        #calculate time since last publish from leap
        delta_time = datetime.datetime.now() - time_mem

        #update time memory
        time_mem = datetime.datetime.now()

        #create hand speed vector
        linear_vector = Vector3(
            x = (delta_pos.x/delta_time), #left/right
            y = (delta_pos.y/delta_time), #upward/downward
            z = (delta_pos.z/delta_time) #forward/rearward
        )

        angular_vector = Vector3(
            x=0,
            y=0,
            z=0
        )

        #create message to be published
        message = Twist(linear=linear_vector, angular_vector=angular_vector)
        
        #publish message to orchestrator
        pub.publish(message)




def listener():
    init()
    rospy.init_node('leap_sub', anonymous=True)
    rospy.Subscriber("leapmotion/data", leapros, listener_callback)
    rospy.spin()

    if __name__ == '__main__':
        try:
            listener()
        except rospy.ROSInterruptException:
            pass
