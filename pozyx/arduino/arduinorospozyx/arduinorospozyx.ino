/**
 * Code for Arduino.
 * You need a pozyx tag plugged on it and a serial connection.
 * It configures the tags and the anchors
 * It recursivly get all the tags locations
 * It send the different values into a ROS topic
 *
 * @author: Pierre-Emmanuel Cochet
 * @date :  december 6th 2017
 */

#include <ArduinoHardware.h>
#include <ros.h>
#include <Pozyx_definitions.h>
#include <Pozyx.h>

//#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

///////////////// PARAMETERS ////////////////
/* Pozyx */
uint8_t   n_tags       = 1;
uint16_t  flyingTags[1]= {0x683D};
uint16_t  masterTag[1] = {0x6F4A};

uint8_t   n_anchors   = 3;
uint16_t  anchors[4]  = { 0x606B, 0x603B, 0x6037, 0x683D};  // THIS IS HARD
int32_t   anchors_x[4]= { 2000,   2000,   3600,   0000  };  // CODDED. YOU
int32_t   anchors_y[4]= { 3200,   2000,   3600,   0000  };  // MUST CHANGE (mm)
int32_t   anchors_z[4]= { 0600,   0600,   0000,   0000  };  // FOR YOUR APP

uint8_t   dimension   = POZYX_3D;

/* Ros */
ros::NodeHandle  nh;

//geometry_msgs::PoseStamped geo_pos;
std_msgs::String geo_pos;

ros::Publisher pub_pos("pos",  &geo_pos);

/////////////////////////////////////////////
void setup()
{
  if(Pozyx.begin() == POZYX_FAILURE){
    delay(100);
    abort();
  }
  Pozyx.clearDevices(NULL);
  setAnchorsManual();
  delay(2000);

  nh.initNode();
  nh.advertise(pub_pos);
}

/////////////////////////////////////////////
void loop()
{
  coordinates_t position;

  for (int i = 0; i < n_tags; i++){
    int status = Pozyx.doRemotePositioning(flyingTags[i], &position, dimension, 0, POZYX_POS_ALG_UWB_ONLY);
    if (status == POZYX_SUCCESS){
      // prints out the result
      printCoordinates(flyingTags[i], position);
    }else{
      // prints out the error code
      printErrorCode("pos", flyingTags[i]);
    }
  }

  pub_pos.publish( &geo_pos );
  nh.spinOnce();
  delay(1000);
}


///////////////// FONCTIONS /////////////////
/*Function that set the anchors*/
void setAnchorsManual(){
  
  device_coordinates_t anchor;
  for(int i = 0; i < n_anchors; i++){
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = anchors_z[i];
    Pozyx.addDevice(anchor, masterTag[0]);
 }
 
}

/*Function that print the Error Code*/
void printErrorCode(String operation, uint16_t network_id){
  uint8_t error_code;
  int status = Pozyx.getErrorCode(&error_code, network_id);
  if(status == POZYX_SUCCESS){
   /* Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(" on ID 0x");
    Serial.print(network_id, HEX);
    Serial.print(", error code: 0x");
    Serial.println(error_code, HEX);*/
  }else{
    Pozyx.getErrorCode(&error_code, NULL);
    /*Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", couldn't retrieve remote error code, local error: 0x");
    Serial.println(error_code, HEX);*/
  }
}

/*Function that print the coordinates*/
void printCoordinates(uint16_t tag_id, coordinates_t position){
  //char mik[12];
  //geo_pos.data = mik;
  sprintf(geo_pos.data, "id: 0x%X - pos: %d", tag_id, position.x);
  pub_pos.publish( &geo_pos );
  nh.spinOnce();
  }
