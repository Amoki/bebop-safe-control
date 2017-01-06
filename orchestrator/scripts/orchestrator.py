#!/usr/bin/env python
# coding: utf-8
import rospy

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
        self.drone = Drone(47.0, 40.0, 1.0)
        self.map = Map()

        self.pub_drone_position = rospy.Publisher('drone_pose', PointStamped, queue_size=10)
        self.pub_collision_point = rospy.Publisher('collision_point', PointStamped, queue_size=10)

        self.pub_take_off = rospy.Publisher('bebop/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('bebop/land', Empty, queue_size=10)
        self.pub_bebop = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber("/position", PoseStamped, self.callback_position)
        rospy.Subscriber("/order", Twist, self.callback_order)

        rospy.Subscriber("/clicked_point", PointStamped, self.callback_clicked_point)

    def will_collide(self, pose):
        '''
        return True if the future position of the drone is to close to a wall
        return False if there is no expected collision
        '''
        distance, x, y, z = self.map.get_nearest_obstacle(pose)
        self.pub_collision_point.publish(PointStamped(
            point=Point(x=x, y=y, z=z),
            header=Header(seq=0, frame_id='map', stamp=rospy.Time.now())
        ))
        return distance < self.drone.size

    def publish_pose(self):
        '''
        debug methode for map visualisation
        '''
        self.pub_drone_position.publish(PointStamped(
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

        print("\n=============================================")
        print("Order received, drone is in the air: %s\n" % (self.drone.in_the_air))
        order = Order(twist)

        # Look for special order
        if order.angular.x == 1 or order.angular.y == 1:
            # Check if special is land or take off
            # if x == 1 Take off !
            if order.angular.x:
                print("Special order detected: Take off")
                if self.drone.in_the_air is False:
                    self.drone.in_the_air = True
                    # Transmit take off order
                    self.pub_take_off.publish()

                else:
                    print("Drone is already in the air")

            # if y == 1 Land !
            if order.angular.y:
                print("Special order detected: Land")
                if self.drone.in_the_air is True:
                    self.drone.in_the_air = False
                    # Transmit land order
                    self.pub_land

                else:
                    print("Drone is already landed")

        else:
            # Look for move order
            distance_twist = order.transform_to_distance_twist()
            future_x = self.drone.position.x + distance_twist.linear.x
            future_y = self.drone.position.y + distance_twist.linear.y
            future_z = self.drone.position.z + distance_twist.linear.z

            if self.will_collide(Point(x=future_x, y=future_y, z=future_z)):
                print('Unable to execute this order\n')
                # Send stabilization order
                # TODO
            else:
                print("Order execution validated no collision point detected \n")
                bebop_twist = order.transform_to_bebop_twist()
                print("Sending movement order to bebop\n")
                self.pub_bebop.publish(bebop_twist)

    def callback_clicked_point(self, point_stamped):
        '''
        Debug only
        Callback called on new publish point on rviz position
        '''
        self.drone.position.x = point_stamped.point.x
        self.drone.position.y = point_stamped.point.y
        self.drone.position.z = point_stamped.point.z
        self.publish_pose()
        self.will_collide(self.drone.position)


def run():
    rospy.init_node('orchestrator', anonymous=True)

    orchestrator = Orchestrator()
    print("Orchestrator init\n")

    rospy.spin()


if __name__ == '__main__':
    run()
