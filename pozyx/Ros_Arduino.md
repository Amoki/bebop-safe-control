# ROS ON ARDUINO
We see in this file, what you need to do in order to use a ROS node on the Arduino.

> Pierre-Emmanuel Cochet 2016-2017

____

## INSTALLATION OF LIBRARIES
1. Install roserial for ARDUINO
```bash
$ sudo apt-get install ros-indigo-rosserial-arduino
$ sudo apt-get install ros-indigo-rosserial
```
2. Install sources onto workspace
```bash
$ cd ~/catkin_ws_pozyx/src
$ git clone https://github.com/ros-drivers/rosserial.git
$ cd ~/catkin_ws_pozyx
$ catkin_make
 ```
3. (Optional) If you want to use the examples from the libray do
```bash
$ cd ~/Arduino/libraries
$ rm -rf ros_lib
$ rosrun rosserial_arduino make_libraries.py .
```
4. You may need also some other libraries. (For example, if during the
  compilation Arduino tells you <string>: no such file or directory)
  download the following lib [STL Source Code](https://www.sgi.com/tech/stl/download.html)
  paste the under your arduino folder: `/arduino-1.0.8/hardware/tools/avr/avr/include/`
5. In any case upload the code on your Arduino, and launch it. You may use the
[code tricks](#code_tricks) detailled bellow.
6. Then launch
`$ rosrun rosserial_python serial_node.py /dev/ttyACM0`
7. If you want to see the messages on the topic type `rostopic echo <chatter>`

## CODE TRICKS
- Arduino code: `ros::NodeHandle nh;` instantiate the node handler,which allows
our program to create publishers and subscribers.
- Arduino code :
```cpp
std_msgs::String str_msg;
ros::Publisher chatter("<chatter>", &str_msg);
```
Instantiate the publishers and subscribers that we will be using. Here we
instantiate a Publisher with a topic name of "<chatter>"
- Arduino:
```cpp
void setup()
{
  nh.initNode();
  nh.advertise(<chatter>);
}
```
Initialise the node handler.
- Arduino:
```cpp
void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
```
-
