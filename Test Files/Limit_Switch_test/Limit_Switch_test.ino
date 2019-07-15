void setup() {
  // put your setup code here, to run once:
#define ls1 9
#define ls2 10
pinMode(ls1, INPUT_PULLUP);
pinMode(ls2, INPUT_PULLUP);
}
int counter = 0;
void loop() {
  // put your main code here, to run repeatedly:
//Serial.println(digitalRead(ls1));
//Serial.println(digitalRead(ls2));
//Serial.println();
//delay(1000);
if (digitalRead(ls1)==1){
  Serial.println("!");
}
}
