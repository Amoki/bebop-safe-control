#!/usr/bin/env python
'''
For the devices: (0xXXX, (X, Y, Z))
Where 0xXXX is the id of the anchor
X Y and Z the position in mm
'''
my_anchors = [
	(0x603B, (4000, 1000, 1200)),
	(0x6037, (4000, 7000, 1200)),
	(0x683D, (1000, 7000, 1200)),
	(0x606B, (0000, 0000, 1200))
]

my_tags = [0x6F4A,]

com_port_id = 1


'''

from settigns import devices

#Faire une liste
devices_id = [device[0] for device in devices]

#Importer les params
anchors = [pypozyx.x(device[0], pozyx.coord(*device[1])) for device in devices]

'''
