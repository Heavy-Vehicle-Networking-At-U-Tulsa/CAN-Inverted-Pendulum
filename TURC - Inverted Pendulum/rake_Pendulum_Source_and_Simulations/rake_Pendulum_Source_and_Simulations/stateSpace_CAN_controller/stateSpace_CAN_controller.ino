//************************************************
//Program Information
//Author: Nathanael Rake
//Hardware: Teensy 3.2

//************************************************
//Includes
#include <FlexCAN.h>

//************************************************
//Defines

  //pins
#define led1 5
#define led2 6
#define ledOnBoard 13

  //maths
#define pi 3.14159

  //CAN
#define CAN0BAUD 1000000
#define controllerAddr 0xAA
#define sensorAddr 0xCC
#define posType 0x01
#define angType 0x02
#define dcType 0x03


//************************************************
//Type Definitions
typedef union {
  float float_t;
  uint8_t int_t[4];
} sensorVal_t;


//************************************************
//Declarations

  //LED
bool ledState = false;

  //State Variable setup
    //Angles ang Angular Velocity
sensorVal_t ang;
sensorVal_t angDeg;
sensorVal_t angVel;
bool newAng = false;
    //Position and Velocity
sensorVal_t pos;
sensorVal_t posIn;
sensorVal_t vel;
bool newPos = false;

  //serial iterrator
int jj = 1;

  //Control calculations
float force = 0;
sensorVal_t dc;

float k1 = -707.10;
float k2 = -575.00;
float k3 = 2493.60;
float k4 = 699.40;

  //CAN
CAN_message_t dcMsg; 

//************************************************
//Timers
elapsedMillis myTimer1;
elapsedMillis velocityTimer;
elapsedMillis refTimer;


//************************************************
//Functions

float forceToDC(float force){
  //converts a force (in Newtons)
  //to an analog signal between -255 and 255
  //for use with analogWrite() (or the motorLeft/motorRight functions)
  float dc;
  if(force > 0){
    //dc = 0.4757*force + 100;
    dc = 2.5*force + 90;
  }
  else {
    //dc = 0.4757*force - 100;
    dc = 2.5*force - 90;
  }
  return(dc);
}


//-------------------
void frameHandler(CAN_message_t &frame, int mailbox){
  int id = frame.id;
  int priority = (id &0xFF000000) >> 24;
  int pgn = (id & 0x00FFFF00) >> 8;
  int type = (id & 0x00FF0000) >> 16;
  int da = (id & 0x0000FF00) >> 8;
  int sa = (id & 0x000000FF);

  if((da==controllerAddr) & (sa==sensorAddr) & (type==posType)){
    newPos = true;
    
    //Position bytes
    pos.int_t[0] = frame.buf[0];
    pos.int_t[1] = frame.buf[1];
    pos.int_t[2] = frame.buf[2];
    pos.int_t[3] = frame.buf[3];

    //velocity bytes
    vel.int_t[0] = frame.buf[4];
    vel.int_t[1] = frame.buf[5];
    vel.int_t[2] = frame.buf[6];
    vel.int_t[3] = frame.buf[7];
  }

  if((da==controllerAddr) & (sa==sensorAddr) & (type==angType)){
    newAng = true;
    
    //Angle bytes
    ang.int_t[0] = frame.buf[0];
    ang.int_t[1] = frame.buf[1];
    ang.int_t[2] = frame.buf[2];
    ang.int_t[3] = frame.buf[3];

    //Angular Velocity bytes
    angVel.int_t[0] = frame.buf[4];
    angVel.int_t[1] = frame.buf[5];
    angVel.int_t[2] = frame.buf[6];
    angVel.int_t[3] = frame.buf[7];
  }

  if(newPos & newAng){
    newPos = false;
    newAng = false;
    
    force = -k1*pos.float_t -k2*vel.float_t -k3*ang.float_t - k4*angVel.float_t;
    dc.float_t = forceToDC(force);

    dcMsg.id = 0x1803ccaa;
    dcMsg.ext = 1;
    dcMsg.len = 4;
    dcMsg.buf[0] = dc.int_t[0];
    dcMsg.buf[1] = dc.int_t[1];
    dcMsg.buf[2] = dc.int_t[2];
    dcMsg.buf[3] = dc.int_t[3];

    Can0.write(dcMsg);

    Serial.println(vel.float_t);
  }
  
}


//************************************************
//CAN Listener

class CANClass : public CANListener 
{
public:
   //void printFrame(CAN_message_t &frame, int mailbox);
   void gotFrame(CAN_message_t &frame, int mailbox); //overrides the parent version so we can actually do something
};

void CANClass::gotFrame(CAN_message_t &frame, int mailbox)
{
    frameHandler(frame,mailbox);
}

CANClass CANClass0;


//************************************************
//Setup
void setup() {
  //Pinmodes
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(ledOnBoard, OUTPUT);

    //CAN0 Setup
  Can0.begin(CAN0BAUD);  
  Can0.attachObj(&CANClass0);
  
  //Filter Setup
  CAN_filter_t allPassFilter;
  allPassFilter.id=0;
  allPassFilter.ext=1;
  allPassFilter.rtr=0;

  //leave the first 4 mailboxes to use the default filter. Just change the higher ones
  for (uint8_t filterNum = 4; filterNum < 16;filterNum++){
    Can0.setFilter(allPassFilter,filterNum); 
  }
  for (uint8_t filterNum = 0; filterNum < 16;filterNum++){
     CANClass0.attachMBHandler(filterNum);
  }

  //Wait for the serial connection to open
  delay(2*1000); 

}


//************************************************
//Loop
void loop() {
   if(myTimer1 > 1*1000){
    myTimer1 = 0;
    ledState = !ledState;

    digitalWrite(ledOnBoard,ledState);
   }

  
}






