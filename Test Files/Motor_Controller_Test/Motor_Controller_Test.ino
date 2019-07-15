
#define pwm1 23
#define pwm2 22
#define led1 5
#define led2 6
#define ledOnBoard 13

#define ls1 9 //limit switch 1
#define ls2 10 //limit switch 2

#define cha1 16
#define chb1 17
#define cha2 14
#define chb2 15

void setup() {
  delay(2000);
  Serial.println("Starting Motor Controller Test");
  pinMode(ls1, INPUT_PULLUP);
  pinMode(ls2, INPUT_PULLUP);

  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  //Initial Outputs
  digitalWrite(pwm1, 0);
  digitalWrite(pwm2, 0);
}
int dInt;
void loop() {
//  Serial.println("Starting Run Loop");
//  while ((digitalRead(ls1)!=1) && (digitalRead(ls2)!=1)){
    digitalWrite(pwm1, 100); //Value from 0-255
    digitalWrite(pwm2, 0); //0-~127 is counterclockwise >128 is clockwise
//  }
//  digitalWrite(pwm1, 0);
}
