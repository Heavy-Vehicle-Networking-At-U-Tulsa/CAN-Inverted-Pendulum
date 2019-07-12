//************************************************
//Program Information
//Author: Nathanael Rake
//Hardware: Teensy 3.2

//************************************************
//Includes
//#define ENCODER_OPTIMIZE_INTERRUPTS //increases encoder interrupt efficiency
                                      //may jeopardize other interrupts
#include <Encoder.h>

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
#define pwm2 22 //22

  //maths
#define pi 3.14159



//************************************************
//Declarations

  //Encoder setup
Encoder angEnc(cha1,chb1);
  float ang;
  float angDeg;
Encoder posEnc(cha2,chb2);
  float pos;
  float posIn;

   //serial iterrator
int jj = 1;

  //velocity calculations
int velocityTimeout = 1;
float previousPos = 0;
float previousAng = 3.14159;
float dx;
float dtheta;
float dt;
float vel;
float angVel;

  //Control calculations
float force = 0;
float dc = 0;

float k1 = -707.10;
float k2 = -575.00;
float k3 = 2493.60;
float k4 = 699.40;

float rScale = -707.1068;
float r = 0;
float rState = 0;

//************************************************
//Timers
elapsedMillis myTimer1;
elapsedMillis velocityTimer;
elapsedMillis refTimer;


//************************************************
//Functions

void motorLeft(int dc){
  //drives the pendulum carriage to the left
  analogWrite(pwm1,dc); // Motor Speed 0-255
  analogWrite(pwm2,0);  // Motor Direction <128
}

//-------------------
void motorRight(int dc){
  //drives the pendulum carriage right
  analogWrite(pwm1,dc); //Motor Speed 0-255
  analogWrite(pwm2,255);//Motor Direction >128
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
  float motSpeed = 100;
  
  Serial.println("Finding the center position");
  motorLeft(motSpeed);
  Serial.print("\tGoing left...");
  myTimer1 = 0;
  Serial.println(digitalRead(ls1));
  while(digitalRead(ls1)!=0){
    //keep going left
    Serial.print(digitalRead(ls1));
    if(myTimer1 > 1000){
      myTimer1 = 0;
      Serial.print(digitalRead(ls1));
    }
    Serial.println();
  }
  Serial.print(digitalRead(ls1));
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
    
    pos = getPos();
      posIn = getPosIn();
    ang = getAng();
      angDeg = getAngDeg();
    
    dx = getPos()-previousPos;
    dtheta = getAng()-previousAng;
    
    vel = dx/dt;
    angVel = dtheta/dt;

    previousPos = pos;
    previousAng = ang;

    //force = -k1*pos -k2*vel -k3*ang - k4*angVel; //No reference
    force = -k1*pos -k2*vel -k3*ang - k4*angVel + rScale*r; //with reference
    dc = forceToDC(force);

    velocityTimer = 0;
    
  }

  if(engageControl()==1){
    digitalWrite(led1,HIGH);
    if(dc > 0){
      motorRight(dc);
    }
    else {
      motorLeft(abs(dc));
    }
    Serial.print("DC: ");Serial.println(dc);
  }
  else {
    digitalWrite(led1,LOW);
    motorLeft(0);
    
    if(Serial.available()){
      float q = Serial.parseFloat();
      switch(jj){
        case 1: k1 = q;
          break;
        case 2: k2 = q;
          break;
        case 3: k3 = q;
          break;
        case 4: k4 = q;
      }

        //For changing gains without reuploading (and waiting through calibration)
//      jj++;
//      if(jj>4){
//        jj=1;
//      }
//      Serial.print("Editing: K");Serial.println(jj);
//      Serial.print("\tK1: ");Serial.println(k1);
//      Serial.print("\tK2: ");Serial.println(k2);
//      Serial.print("\tK3: ");Serial.println(k3);
//      Serial.print("\tK4: ");Serial.println(k4);
    }
  }

  //Serial.print("Ang: ");Serial.println(getAngDeg());
}
