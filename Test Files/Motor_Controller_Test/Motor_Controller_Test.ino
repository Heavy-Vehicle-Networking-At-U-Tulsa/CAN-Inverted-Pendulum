//This code currently has no safety shutoff 
#define pwm1 23
#define pwm2 22
#define led1 5
#define led2 6
#define ledOnBoard 13

int spdInt = 0;

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

void loop() {
  delay(500);
  analogWrite(pwm1, 64);
  spdInt = spdInt + 5;
}
