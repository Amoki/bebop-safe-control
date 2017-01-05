#ROS ON ARDUINO
We see in this file, what you need to do in order to use a ROS node on the Arduino.

> Pierre-Emmanuel Cochet 2016-2017

____

##INSTALLATION OF LIBRARIES
1. Install roserial for ARDUINO
```
$ sudo apt-get install ros-indigo-rosserial-arduino
$ sudo apt-get install ros-indigo-rosserial
```

2. Install sources onto workspace
```
$ cd ~/catkin_ws_pozyx/src
$ git clone https://github.com/ros-drivers/rosserial.git
$ cd ~/catkin_ws_pozyx
$ catkin_make
 ```

3. (Optional) If you want to use the examples from the libray do
```
$ cd ~/Arduino/libraries
$ rm -rf ros_lib
$ rosrun rosserial_arduino make_libraries.py .
```

4.
5.

##CODE TRICKS
