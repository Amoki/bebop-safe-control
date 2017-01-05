#!/usr/bin/env python
"""
Performs discovery for all tags in range and then configures the positioning
anchor list on all the discovered devices.

This requires all devices to be on the same UWB settings first, so I highly
recommend to run the uwb_configurator node first.
"""

import pypozyx
import rospy

from settings import my_anchors
from settings import com_port_id

# Declare all the anchors, based on the settings.py lists
anchors = [pypozyx.DeviceCoordinates(anchor[0],1, pypozyx.Coordinates(*(anchor[1]))) for anchor in my_anchors]

'''
anchors = [pypozyx.DeviceCoordinates(0x603B, 1, pypozyx.Coordinates(0, 0, 5000)),
           pypozyx.DeviceCoordinates(0x6037, 1, pypozyx.Coordinates(5000, 0, 1000)),
           pypozyx.DeviceCoordinates(0x683D, 1, pypozyx.Coordinates(0, 5000, 1000)),
           pypozyx.DeviceCoordinates(0x606B, 1, pypozyx.Coordinates(5000, 5000, 1000))]
'''

def set_anchor_configuration():
    tag_ids = [None]
    rospy.init_node('uwb_configurator')
    rospy.loginfo("Configuring device list.")

    settings_registers = [0x16, 0x17]  # POS ALG and NUM ANCHORS
    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_serial_ports()[com_port_id].device)
    except:
        rospy.loginfo("Pozyx not connected")
        return
    pozyx.doDiscovery(discovery_type=pypozyx.POZYX_DISCOVERY_TAGS_ONLY)
    device_list_size = pypozyx.SingleRegister()
    pozyx.getDeviceListSize(device_list_size)
    if device_list_size[0] > 0:
        device_list = pypozyx.DeviceList(list_size=device_list_size[0])
        pozyx.getDeviceIds(device_list)
        tag_ids += device_list.data

    for tag in tag_ids:
        for anchor in anchors:
            pozyx.addDevice(anchor, tag)
        if len(anchors) > 4:
            pozyx.setSelectionOfAnchors(pypozyx.POZYX_ANCHOR_SEL_AUTO,
                                        len(anchors), remote_id=tag)
            pozyx.saveRegisters(settings_registers, remote_id=tag)
        pozyx.saveNetwork(remote_id=tag)
        if tag is None:
            rospy.loginfo("Local device configured")
        else:
            rosply.loginfo("Device with ID 0x%0.4x configured." % tag)
    rospy.loginfo("Configuration completed! Shutting down node now...")


if __name__ == '__main__':
    set_anchor_configuration()
