# MISE EN PLACE
We gonna see how to setup the ROS package for Pozyx.
Then we will see how to add the code for the Bebop_project

> Pierre-Emmanuel Cochet 2016-2017

____

## PRE-REQUESTED
You need some things for starting:
- ROS [indigo](http://wiki.ros.org/indigo/Installation)
- [catkin](http://wiki.ros.org/catkin)

## STEP BY STEP
1. Download  examples from the git repo of pozyx_ros
`$ git clone https://github.com/pozyxLabs/pozyx_ros.git`

2. Create a catkin package
```bash
$ mkdir -p ~/catkin_ws_pozyx/src
$ cd ~/catkin_ws_pozyx/src
$ catkin_init_workspace
```

3. Copy-Paste the folder *pozyx_ros/pozyx_ros_examples*, downloaded in step 1, into *~/catkin_ws_pozyx/src*

4. Before performing the `catkin_make` command you need to change a little thing in the code:
In *~/catkin_ws_pozyx/src/pozyx_ros_examples/CMakeLists.txt* at line 108 to look like as followed
```python
catkin_package(
    INCLUDE_DIRS #include
    LIBRARIES pozyx_ros_examples
    CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
    DEPENDS system_lib
)
```
5. Perform the command:
```
$ cd ~/catkin_ws/
$ catkin_make
```
then
```
$ source devel/setup.bash
```

6. Check and configure the COM port
```
$ lsusb
$ sudo apt-get install input-utils
$ sudo lsinput
```
**Every time you reconnect the Arduino make sure that you've done**
`$ sudo chmod 777 /dev/ttyACM0`

Test access to COM port
`$ python -c "from pypozyx import *;list_serial_ports()"`
and
`$ python -c "import pypozyx; import rospy; print(pypozyx.PozyxSerial(pypozyx.get_serial_ports()[1].device))"`
or manually test the COM Port (once pySerial)
`pyserial-3.2.1/examples $ sudo python wxTerminal.py `

7. Launch roscore
`$ roscore`

8. Launch the wanted package...
```
$ rosrun pozyx_ros_examples \[ARGS\]
```
where ARGS can be:
```
\[ARGS\]
anchor_calibration_all_tags.py   positioning_pub.py
anchor_configuration.py          range_info_pub.py
euler_pub.py                     uwb_configurator_all_devices.py
pose_pub.py                      uwb_configurator.py
position_euler_pub.py
```



## ADDITIONAL DEPENDENCIES
- pypozyx library [here](https://github.com/pozyxLabs/Pozyx-Python-library)
before do an `$ sudo chmod +rwx dist-packages/`
then in the */Pozyx-Python-library-master* directory do `$ sudo python setup.py install`
- pozyx arduino library
- pySerial [here](https://pypi.python.org/pypi/pyserial)
then in the *pyserial-3.2.1/* do `$ sudo python setup.py install`
