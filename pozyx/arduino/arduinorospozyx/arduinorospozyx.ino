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

/* Pozyx */
uint16_t  flyingTags[NB_TAGS]= {0x683D};
uint16_t  masterTag = 0x6F4A;

uint8_t   n_anchors   = 4;
uint16_t  anchors[4]  = { 0x606B, 0x603B, 0x6037, 0x6F4A};  // THIS IS HARD
int32_t   anchors_x[4]= { 2900, 0000, 2900, 0000};  // CODDED. YOU
int32_t   anchors_y[4]= { 7900, 0000, 0000, 7900};  // MUST CHANGE (mm)
int32_t   anchors_z[4]= { 0000, 1650, 1050, 1050};  // FOR YOUR APP

uint8_t   dimension   = POZYX_3D;

/* Ros */
ros::NodeHandle  nh;

std_msgs::String geo_pos;
std_msgs::String msg_pos;

ros::Publisher pub_pos("pos",  &geo_pos);
//ros::Publisher pub_mdg("pos",  &msg_pos);


/////////////////////////////////////////////
void setup()
{
  nh.initNode();
  nh.advertise(pub_pos); 

  if(Pozyx.begin() == POZYX_FAILURE){
    delay(100);
    abort();
  }
  clearDevices();
  setAnchorsManual();
  setTagsManual();
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
    nh.spinOnce();
  }
}


///////////////// FONCTIONS /////////////////
/*Function that set the anchors*/
void setAnchorsManual(){

  device_coordinates_t anchor;
  for(int i = 0; i < n_anchors; i++){
    anchor.network_id = anchors[i];
    anchor.flag = 1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = anchors_z[i];

    Pozyx.setOperationMode(POZYX_ANCHOR_MODE,anchors[i]);

    for(int i = 0; i < NB_TAGS; i++){
      if (POZYX_SUCCESS == Pozyx.addDevice(anchor, flyingTags[0])){
      // OK
      }
     }
  }

}

/*Function that set the flying tags*/
void setTagsManual(){

  for(int i = 0; i < NB_TAGS; i++){
    if (POZYX_SUCCESS == Pozyx.setOperationMode(POZYX_TAG_MODE,flyingTags[i])){
      // OK
    }
  }

}

/*Function that init all the devices*/
void clearDevices(){
  for(int i = 0; i < n_anchors; i++){
    Pozyx.clearDevices(anchors[i]);
  }
  for(int i = 0; i < NB_TAGS; i++){
    Pozyx.clearDevices(flyingTags[i]);
  }
  
 }
 
/*Function that print the Error Code*/
void printErrorCode(String operation, uint16_t network_id){
  uint8_t error_code;
  int status = Pozyx.getErrorCode(&error_code, network_id);
  
  if(status == POZYX_SUCCESS){
    sprintf(msg_pos.data, "ERROR id:%X, error code:%X", network_id, error_code);
    pub_pos.publish( &msg_pos );

  }else{
    Pozyx.getErrorCode(&error_code, NULL);
    sprintf(msg_pos.data, "ERROR unknown id, error code:%X", error_code);
    pub_pos.publish( &msg_pos );
  }
}

/*Function that print the coordinates*/
void printCoordinates(uint16_t tag_id, coordinates_t* position){
  char mik[50];
  //sprintf(mik, "id: 0x%X - pos: %d ", tag_id, position.x);
  sprintf(geo_pos.data, "id: 0x%X - pos: %i  %i  %i", tag_id, int(position->x),  int(position->y),  int(position->z));

  //geo_pos.data = mik;
  pub_pos.publish( &geo_pos );
}
