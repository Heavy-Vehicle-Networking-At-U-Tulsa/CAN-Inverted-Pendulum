//************************************************
//Program Information
//Author: Nathanael Rake
//Hardware: Teensy 3.2

//************************************************
//Includes
//#define ENCODER_OPTIMIZE_INTERRUPTS //increases encoder interrupt efficiency
                                      //may jeopardize other interrupts
#include <Encoder.h>
#include <FlexCAN.h>

//************************************************
//Defines

  //pins
#define led1 5
#define led2 6
#define ledOnBoard 13

#define ls1 9 //limit switch 1
#define ls2 10 //limit switch 2

#define cha1 16
#define chb1 17
#define cha2 14
#define chb2 15

#define pwm1 23
#define pwm2 22

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

  //Encoder setup
Encoder angEnc(cha1,chb1);
  sensorVal_t ang;
  sensorVal_t angDeg;
  sensorVal_t angVel;
Encoder posEnc(cha2,chb2);
  sensorVal_t pos;
  sensorVal_t posIn;
  sensorVal_t vel;

   //serial iterrator
int jj = 1;

  //velocity calculations
int velocityTimeout = 1;
float previousPos = 0;
float previousAng = 3.14159;
float dx;
float dtheta;
float dt;

  //Motor Driver
sensorVal_t dc;

  //CAN
CAN_message_t posMsg, angMsg;

//************************************************
//Timers
elapsedMillis myTimer1;
elapsedMillis velocityTimer;
elapsedMillis refTimer;


//************************************************
//Functions

void motorLeft(int dc){
  //drives the pendulum carriage to the left
  analogWrite(pwm2,0);
  analogWrite(pwm1,dc);
}

//-------------------
void motorRight(int dc){
  //drives the pendulum carriage right
  analogWrite(pwm1,0);
  analogWrite(pwm2,dc);
}

//-------------------
void findAngZero(){
  Serial.print("Waiting for pendulum to come to rest in downward position...");
  float ang;
  float tol = 10;
  int debouncer = 10;
  float delta = 0;

  int i = 0;
  int j = 0;
  float angPrev = angEnc.read();
  
  while(i < debouncer){
    if(j%2==0){
      Serial.print(".");
    }
    velocityTimer = 0;
    
    while(velocityTimer < 0.5*1000){
      //just wait
    }
    
    ang = angEnc.read();
    delta = abs(ang-angPrev);
    angPrev = ang;
    
    if(delta <= tol){
      i++;
      digitalWrite(led1,HIGH);
    }
    else {
      i = 0;
      digitalWrite(led1,LOW);
    }

    j++; //loop counter
  }

  Serial.println("Done!");
  angEnc.write(4096); //pendulum is 1/2 rotation from 0
  digitalWrite(led1,LOW);
  digitalWrite(led2,HIGH);
}

//-------------------
void findPosZero(){
  //determines the position encoder range and sets the 
  //  center of the platform to be the zero position
  float motSpeed = 200;
  
  Serial.println("Finding the center position");
  motorLeft(motSpeed);
  Serial.print("\tGoing left...");
  myTimer1 = 0;
  while(digitalRead(ls1)!=0){
    //keep going left
    if(myTimer1 > 1000){
      myTimer1 = 0;
      Serial.print(".");
    }
  }
  Serial.println("Done!");
  motorLeft(0);
  posEnc.write(0);
  delay(500);
  
  motorRight(motSpeed);
  Serial.print("\tGoing right...");
  myTimer1 = 0;
  while(digitalRead(ls2)!=0){
    //keep going right
    if(myTimer1 > 1000){
      myTimer1 = 0;
      Serial.print(".");
    }
  }
  Serial.println("Done!");
  motorRight(0);
  int maxPos = posEnc.read();
  posEnc.write(maxPos/2);
  delay(500);

  motorLeft(motSpeed);
  Serial.print("\tGoing toward the center...");
  myTimer1 = 0;
  while(posEnc.read()>0){
    //Keep going to the center
    if(myTimer1 > 1000){
      myTimer1 = 0;
      Serial.print(".");
    }
  }
  Serial.println("Done!");
  motorLeft(0);
}

//-------------------
float getPos(){
  //returns the position in meters
  int counts = posEnc.read();
  float pos = counts*0.0000187094;
  return(pos);
}

//-------------------
float getPosIn(){
  //returns the position in inches
  int counts = posEnc.read();
  float pos = counts*0.000736591;
  return(pos);
}

//-------------------
float getAng(){
  //returns the pendulum angle in radians
  int counts = angEnc.read();
  float ang = counts*0.00076699;
  while(ang > pi){
    ang += -2.0*pi;
  }
  while(ang < -pi){
    ang += 2.0*pi;
  }
  return(-ang);
}

//-------------------
float getAngDeg(){
  //returns the pendulum angle in degrees
  int counts = angEnc.read();
  float ang = counts*0.0439453;
  while(ang > 180){
    ang += -360;
  }
  while(ang < -180){
    ang += 360;
  }
  return(-ang);
}

//-------------------
int engageControl(){
  float lowerBoundAngDeg = -15;
  float upperBoundAngDeg = 15;

  int engage;
  float theAng = getAngDeg();
  if((-15 <= theAng) & (theAng <= 15)){
    engage = 1;
  }
  else {
    engage = 0;
  }
  return(engage);
}


//-------------------
void frameHandler(CAN_message_t &frame, int mailbox){
  int id = frame.id;
  int priority = (id &0xFF000000) >> 24;
  int pgn = (id & 0x00FFFF00) >> 8;
  int type = (id & 0x00FF0000) >> 16;
  int da = (id & 0x0000FF00) >> 8;
  int sa = (id & 0x000000FF);

  Serial.print("DA: ");Serial.print(da);
  Serial.print("\tSA: ");Serial.print(sa);
  Serial.print("\tType: ");Serial.println(type);

  if((da==sensorAddr) & (sa==controllerAddr) & (type==dcType)){
    Serial.println("\t\tMatch Confirmed");
    dc.int_t[0] = frame.buf[0];
    dc.int_t[1] = frame.buf[1];
    dc.int_t[2] = frame.buf[2];
    dc.int_t[3] = frame.buf[3];
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
  
  pinMode(ls1, INPUT);
  pinMode(ls2, INPUT);

  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);

  //Initial Outputs
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);

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
  
  //find position limits
  findAngZero();
  findPosZero();  

}


//************************************************
//Loop
void loop() {

  //velocity derivation
  if(velocityTimer > velocityTimeout){
    dt = velocityTimer/1000.0;
    
    pos.float_t = getPos();
      posIn.float_t = getPosIn();
    ang.float_t = getAng();
      angDeg.float_t = getAngDeg();
    
    dx = getPos()-previousPos;
    dtheta = getAng()-previousAng;
    
    vel.float_t = dx/dt;
    angVel.float_t = dtheta/dt;

    previousPos = pos.float_t;
    previousAng = ang.float_t;

    //Send position data
    posMsg.id = 0x1801aacc;
    posMsg.ext = 1;
    posMsg.len = 8;
      //Position
    posMsg.buf[0] = pos.int_t[0];
    posMsg.buf[1] = pos.int_t[1];
    posMsg.buf[2] = pos.int_t[2];
    posMsg.buf[3] = pos.int_t[3];
      //Velocity
    posMsg.buf[4] = vel.int_t[0];
    posMsg.buf[5] = vel.int_t[1];
    posMsg.buf[6] = vel.int_t[2];
    posMsg.buf[7] = vel.int_t[3];

    Can0.write(posMsg);

        //Send angle data
    angMsg.id = 0x1802aacc;
    angMsg.ext=1;
    angMsg.len = 8;
      //angle
    angMsg.buf[0] = ang.int_t[0];
    angMsg.buf[1] = ang.int_t[1];
    angMsg.buf[2] = ang.int_t[2];
    angMsg.buf[3] = ang.int_t[3];
      //angular velocity
    angMsg.buf[4] = angVel.int_t[0];
    angMsg.buf[5] = angVel.int_t[1];
    angMsg.buf[6] = angVel.int_t[2];
    angMsg.buf[7] = angVel.int_t[3];

    Can0.write(angMsg);

    velocityTimer = 0;  
  }

  //Control loop
  if(engageControl()==1){
    digitalWrite(led1,HIGH);
    if(dc.float_t > 0){
      motorRight(dc.float_t);
    }
    else {
      motorLeft(abs(dc.float_t));
    }
    Serial.print("DC: ");Serial.println(dc.float_t);
  }
  else {
    digitalWrite(led1,LOW);
    motorLeft(0);
  }
}






