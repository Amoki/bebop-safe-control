#!/usr/bin/env python
__author__ = 'flier'

import rospy
from leap_motion.msg import leap
from leap_motion.msg import leapros

# Native datatypes, I've heard this is bad practice, use the geometry messages instead.
# def callback(data):
#    rospy.loginfo(rospy.get_name() + ": Leap Raw Data %5.2s" % data)


# Callback of the ROS subscriber, just print the received data.
def callback_ros(data):
    #rospy.loginfo(rospy.get_name() + ": Leap ROS Data %5.2s" % data)
    #string = """
    #{jdehfvn}
    #""".format(
    #   jdehfvn= data.azerty
    #)
    print("%s" % datetime.datetime.now())

    print("Roll\t sens = %2s") % (data.finger_roll)

    print("Palm\t\t\ndirection[%5.2s,%5.2s,%5.2s]\t\nnormal[%5.2s,%5.2s,%5.2s]\t\npalmpos[%5.2s,%5.2s,%5.2s]\t\nypr[%5.2s,%5.2s,%5.2s]\t\n" % (
        data.direction.x,data.direction.y,data.direction.z,
        data.normal.x,data.normal.y,data.normal.z,
        data.palmpos.x,data.palmpos.y,data.palmpos.z,
        data.ypr.x,data.ypr.y,data.ypr.z))
    print("Thumb\t\t\nmetacarpal[%5.2s,%5.2s,%5.2s]\t\nproximal[%5.2s,%5.2s,%5.2s]\t\nintermediate[%5.2s,%5.2s,%5.2s]\t\ndistal[%5.2s,%5.2s,%5.2s]\t\ntip[%5.2s,%5.2s,%5.2s]\t\n" % (
        data.thumb_metacarpal.x,data.thumb_metacarpal.y,data.thumb_metacarpal.z,
        data.thumb_proximal.x,data.thumb_proximal.y,data.thumb_proximal.z,
        data.thumb_intermediate.x,data.thumb_intermediate.y,data.thumb_intermediate.z,
        data.thumb_distal.x,data.thumb_distal.y,data.thumb_distal.z,
        data.thumb_tip.x,data.thumb_tip.y,data.thumb_tip.z))

    print("Index\t\t\nmetacarpal[%5.2s,%5.2s,%5.2s]\t\nproximal[%5.2s,%5.2s,%5.2s]\t\nintermediate[%5.2s,%5.2s,%5.2s]\t\ndistal[%5.2s,%5.2s,%5.2s]\t\ntip[%5.2s,%5.2s,%5.2s]\t\n" % (
        data.index_metacarpal.x,data.index_metacarpal.y,data.index_metacarpal.z,
        data.index_proximal.x,data.index_proximal.y,data.index_proximal.z,
        data.index_intermediate.x,data.index_intermediate.y,data.index_intermediate.z,
        data.index_distal.x,data.index_distal.y,data.index_distal.z,
        data.index_tip.x,data.index_tip.y,data.index_tip.z))

    print("Middle\t\t\nmetacarpal[%5.2s,%5.2s,%5.2s]\t\nproximal[%5.2s,%5.2s,%5.2s]\t\nintermediate[%5.2s,%5.2s,%5.2s]\t\ndistal[%5.2s,%5.2s,%5.2s]\t\ntip[%5.2s,%5.2s,%5.2s]\t\n" % (
        data.middle_metacarpal.x,data.middle_metacarpal.y,data.middle_metacarpal.z,
        data.middle_proximal.x,data.middle_proximal.y,data.middle_proximal.z,
        data.middle_intermediate.x,data.middle_intermediate.y,data.middle_intermediate.z,
        data.middle_distal.x,data.middle_distal.y,data.middle_distal.z,
        data.middle_tip.x,data.middle_tip.y,data.middle_tip.z))

    print("Ring\t\t\nmetacarpal[%5.2s,%5.2s,%5.2s]\t\nproximal[%5.2s,%5.2s,%5.2s]\t\nintermediate[%5.2s,%5.2s,%5.2s]\t\ndistal[%5.2s,%5.2s,%5.2s]\t\ntip[%5.2s,%5.2s,%5.2s]\t\n" % (
        data.ring_metacarpal.x,data.ring_metacarpal.y,data.ring_metacarpal.z,
        data.ring_proximal.x,data.ring_proximal.y,data.ring_proximal.z,
        data.ring_intermediate.x,data.ring_intermediate.y,data.ring_intermediate.z,
        data.ring_distal.x,data.ring_distal.y,data.ring_distal.z,
        data.ring_tip.x,data.ring_tip.y,data.ring_tip.z))

    print("Pinky\t\t\nmetacarpal[%5.2s,%5.2s,%5.2s]\t\nproximal[%5.2s,%5.2s,%5.2s]\t\nintermediate[%5.2s,%5.2s,%5.2s]\t\ndistal[%5.2s,%5.2s,%5.2s]\t\ntip[%5.2s,%5.2s,%5.2s]\t\n" % (
        data.pinky_metacarpal.x,data.pinky_metacarpal.y,data.pinky_metacarpal.z,
        data.pinky_proximal.x,data.pinky_proximal.y,data.pinky_proximal.z,
        data.pinky_intermediate.x,data.pinky_intermediate.y,data.pinky_intermediate.z,
        data.pinky_distal.x,data.pinky_distal.y,data.pinky_distal.z,
        data.pinky_tip.x,data.pinky_tip.y,data.pinky_tip.z))

    print("\n\n\n\n\n\n\n\n\n\n")

# Yes, a listener aka subscriber ;) obviously. Listens to: leapmotion/data
def listener():
    rospy.init_node('leap_sub', anonymous=True)
    print("leapmotion/data subscriber init\n")
    # rospy.Subscriber("leapmotion/raw", leap, callback)
    rospy.Subscriber("leapmotion/data", leapros, callback_ros)
    rospy.spin()


if __name__ == '__main__':
    listener()
