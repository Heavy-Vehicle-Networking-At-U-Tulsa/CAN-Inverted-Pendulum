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
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(ledOnBoard, OUTPUT);
  
  pinMode(ls1, INPUT);
  pinMode(ls2, INPUT);

  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);

  //Initial Outputs
  analogWrite(pwm2, 255);
}
int spdInt = 0;
void loop() {
delay(500);
analogWrite(pwm1, 64);
//spdInt = spdInt + 5;
}
