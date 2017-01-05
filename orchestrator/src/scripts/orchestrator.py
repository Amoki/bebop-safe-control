#!/usr/bin/env python
# coding: utf-8
import rospy
import time
import sys

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from std_msgs.msg import Empty

from map_handler import Map
from drone import Drone
from order import Order


class Orchestrator(object):
    def __init__(self):
        self.drone = Drone(47, 43, 0)
        self.map = Map()

        self.pub_point = rospy.Publisher('point', PointStamped, queue_size=10)
        self.pub_point2 = rospy.Publisher('point2', PointStamped, queue_size=10)

        self.pub_take_off = rospy.Publisher('bebop/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('bebop/land', Empty, queue_size=10)
        self.pub_bebop = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber("/position", PoseStamped, self.callback_position)
        rospy.Subscriber("/order", Twist, self.callback_order)

    def will_collide(self, pose):
        '''
        return True if the future position of the drone is to close to a wall
        return False if there is no expected collision
        '''
        distance, x, y = self.map.get_nearest_obstacle(pose)
        self.pub_point2.publish(PointStamped(
            point=Point(x=x, y=y, z=0),
            header=Header(seq=0, frame_id='map', stamp=rospy.Time.now())
        ))
        return distance / self.map.info.resolution < self.drone.size

    def publish_poz(self):
        '''
        debug methode for map visualisation
        '''
        self.pub_point.publish(PointStamped(
            point=self.drone.position,
            header=Header(seq=0, frame_id='map', stamp=rospy.Time.now())
        ))

    def callback_position(self, pose_stamped):
        '''
        Callback called on new pozyx position
        '''
        self.drone.position = pose_stamped.pose.position
        self.drone.orientation = pose_stamped.pose.orientation

    def callback_order(self, twist):
        '''
        Callback called on new leap order is received
        '''
        order = Order(twist)
        distance_twist = order.transform_to_distance_twist()
        futur_x = self.drone.position.x + distance_twist.position.x
        futur_y = self.drone.position.y + distance_twist.position.y

        if self.will_collide(Pose(x=futur_x, y=futur_y)):
            print('Unable to execute this order')
        else:
            bebop_twist = order.transform_to_bebop_twist()
            # Checl if special order like land or take off
            self.pub_bebop.publish(bebop_twist)


def run():
    rospy.init_node('orchestrator', anonymous=True)

    orchestrator = Orchestrator()

    while True:
        try:
            orchestrator.will_collide(orchestrator.drone.position)
            orchestrator.publish_poz()
            time.sleep(1)
        except KeyboardInterrupt:
            print "Bye"
            sys.exit()

    rospy.spin()


if __name__ == '__main__':
    run()
