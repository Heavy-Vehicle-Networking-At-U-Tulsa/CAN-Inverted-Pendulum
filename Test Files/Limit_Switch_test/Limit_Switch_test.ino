void setup() {
  // put your setup code here, to run once:
#define ls1 9
#define ls2 10
pinMode(ls1, INPUT);
pinMode(ls2, INPUT);
}
int counter = 0;
void loop() {
  // put your main code here, to run repeatedly:
if (digitalRead(ls1)==0){
  counter++;
  Serial.println(counter);
}
}
