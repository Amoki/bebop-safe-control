/**
 * Code for Arduino.
 * You need a pozyx tag plugged on it and a serial connection.
 * It configures the tags and the anchors
 * It recursivly get all the tags locations
 * It send the different values into a ROS topic
 *
 * @author: Pierre-Emmanuel Cochet
 * @date :  january 6th 2017
 */

#include <ArduinoHardware.h>
#include <ros.h>
#include <Pozyx_definitions.h>
#include <Pozyx.h>

#include <std_msgs/String.h>

///////////////// PARAMETERS ////////////////
#define USB_CON
#define NB_TAGS 1
#define NB_ANCH 4

/* Pozyx */
uint16_t  flyingTags[NB_TAGS]= {0x683D};
uint16_t  masterTag = 0x6F4A;

uint16_t  anchors[NB_ANCH]  = { 0x6001, 0x603A, 0x6037, 0x603B};  // THIS IS HARD
int32_t   anchors_x[NB_ANCH]= { 2900, 0000, 2900, 0000};          // CODDED. YOU
int32_t   anchors_y[NB_ANCH]= { 7900, 0000, 0000, 7900};          // MUST CHANGE (mm)
int32_t   anchors_z[NB_ANCH]= { 0000, 1650, 1050, 1050};          // FOR YOUR APP : TEST

uint8_t   dimension   = POZYX_3D;

/* Ros */
ros::NodeHandle  nh;

std_msgs::String geo_pos;

ros::Publisher pub_pos("pos",  &geo_pos);

/////////////////////////////////////////////
void setup()
{
  nh.initNode();
  nh.advertise(pub_pos);

  if(Pozyx.begin() == POZYX_FAILURE){
    delay(100);
    abort();
  }
  //setMaster();
  setAnchorsManual();
  delay(2000);
  
}

/////////////////////////////////////////////
void loop()
{
  coordinates_t position;
  for (int i = 0; i < NB_TAGS; i++){
    int status = Pozyx.doRemotePositioning(flyingTags[i], &position, dimension, 0,POZYX_POS_ALG_UWB_ONLY );
    
    if (status == POZYX_SUCCESS){
      // prints out the result
      printCoordinates(flyingTags[i], &position);
    }else{
      // prints out the error code
      printErrorCode("pos", flyingTags[i]);
    }
  }
}


///////////////// FONCTIONS /////////////////
/*Function that set the anchors*/
void setAnchorsManual(){
  int i, j = 0;
  int status;
  
  for( j = 0; j < NB_ANCH; j++){
     status &= Pozyx.setOperationMode(POZYX_ANCHOR_MODE,anchors[j]);
     }
  for ( i = 0; i < NB_TAGS; i++){
    status &= Pozyx.clearDevices(flyingTags[i]);
    status &= Pozyx.setOperationMode(POZYX_TAG_MODE,flyingTags[i]);
      for( j = 0; j < NB_ANCH; j++){
        device_coordinates_t anchor;
        anchor.network_id = anchors[j];
        anchor.flag = 1;
        anchor.pos.x = anchors_x[j];
        anchor.pos.y = anchors_y[j];
        anchor.pos.z = anchors_z[j];
 
        status &= Pozyx.addDevice(anchor, flyingTags[i]);
      }
  }
  //if(){}else{}
}

/*Function to set the master*/
void setMaster(){
  Pozyx.clearDevices(masterTag);
  Pozyx.doDiscovery(POZYX_DISCOVERY_ALL_DEVICES, NB_TAGS + NB_ANCH, 20);
}

/*Function that print the Error Code*/
void printErrorCode(String operation, uint16_t network_id){
  uint8_t error_code;
  int status = Pozyx.getErrorCode(&error_code, network_id);
  
  if(status == POZYX_SUCCESS){
    sprintf(geo_pos.data, "ERROR id:%X, error code:%X", network_id, error_code);
    pub_pos.publish( &geo_pos );
    nh.spinOnce();
  }else{
    Pozyx.getErrorCode(&error_code, NULL);
    sprintf(geo_pos.data, "ERROR unknown id, error code:%X", error_code);
    pub_pos.publish( &geo_pos );
    nh.spinOnce();
  }
}

/*Function that print the coordinates*/
void printCoordinates(uint16_t tag_id, coordinates_t* position){
  sprintf(geo_pos.data, "id: 0x%X - pos: %i  %i  %i", tag_id, int(position->x),  int(position->y),  int(position->z));
  pub_pos.publish( &geo_pos );
  nh.spinOnce();
}
