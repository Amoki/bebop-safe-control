/*
 * Code for Arduino.
 * You need a pozyx tag plugged on it and a serial connection.
 * It configures the tags and the anchors
 * It recursivly get all the tags locations
 * It send the different values into a ROS topic
 *
 * @author: Pierre-Emmanuel Cochet
 */

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/String.h>

///////////////// PARAMETERS ////////////////
uint8_t   n_tags              = 3;
uint16_t  flyingTags[n_tags]  = {0x0001, 0x0002, 0x0003};
uint16_t  masterTag[1]        = {0x6F4A};

uint8_t   n_anchors           = 4;
uint16_t  anchors[n_anchors]  = { 0x606B, 0x603B, 0x683D, 0x6037};
int32_t   anchors_x[n_anchors]= { 0,      4500,   500,    4450  };
int32_t   anchors_y[n_anchors]= { 0,      0,      3300,   3500  };
int32_t   anchors_z[n_anchors]= { 1500,   1800,   1100,   2000  };

uint8_t   algorithm           = POZYX_POS_ALG_UWB_ONLY;
uint8_t   dimension           = POZYX_3D;

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher p("positions", &str_msg);

char hello[13] = "hello world!";
char hello[13] = "hello world!";

/////////////////////////////////////////////
void setup()
{

  if(Pozyx.begin() == POZYX_FAILURE){
    delay(100);
    abort();
  }

  setAnchorsManual();
  delay(2000);

  nh.initNode();
  nh.advertise(p);
}

/////////////////////////////////////////////
void loop()
{

  for (int i = 0; i < n_tags; i++){
    coordinates_t position;
    int status = Pozyx.doRemotePositioning(flyingTags[i], &position, dimension, height, algorithm);
    if (status == POZYX_SUCCESS){
    // prints out the result
    printCoordinates(position, flyingTags[i]);
    }else{
      // prints out the error code
      printErrorCode("positioning", flyingTags[i]);
    }
  }

  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
