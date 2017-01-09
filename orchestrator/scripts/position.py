#!/usr/bin/env python
# coding: utf-8

import rospy
import re
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from std_msgs.msg import String

REGEX_STR = "id: 0x([0-9a-fA-F]+) - pos: (\d+) (\d+) (\d+)"

regex = re.compile(REGEX_STR, re.IGNORECASE)


class Converter(object):
    def __init__(self):
        self.pub_position = rospy.Publisher('/position', PoseStamped, queue_size=10)
        rospy.Subscriber("/pos", String, self.callback)
        self.seq = 0
        self.frame_id = 0

    def callback(self, serial):
        string = serial.data
        if regex.search(string) is not None:
            id = regex.search(string).group(1)
            x = int(regex.search(string).group(2))
            y = int(regex.search(string).group(3))
            z = int(regex.search(string).group(4))
            self.seq += 1
            self.pub_position.publish(
                PoseStamped(
                    pose=Pose(
                        position=Point(x=x, y=y, z=z),
                        orientation=Quaternion()
                    ),
                    header=Header(seq=self.seq, frame_id='map', stamp=rospy.Time.now())
                )
            )


def run():
    rospy.init_node('orchestrator', anonymous=True)

    Converter()

    rospy.spin()


if __name__ == '__main__':
    run()
