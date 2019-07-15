void setup() {
  // put your setup code here, to run once:
#define ls1 9
#define ls2 10
pinMode(ls1, INPUT_PULLUP);
pinMode(ls2, INPUT_PULLUP);
}
void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
  if ((digitalRead(ls1)==1)&&(digitalRead(ls2)==1)){
  Serial.println("Both Limit Switches are Pressed");
  }
  else if ((digitalRead(ls1)==1)&&(digitalRead(ls2)==0)){
    Serial.println("Limit Switch 1 is Pressed");
  }
  else if ((digitalRead(ls1)==0)&&(digitalRead(ls2)==1)){
    Serial.println("Limit Switch 2 is Pressed");
  }
  else{
    Serial.println("Neither Limit Switch is Pressed");
  }
}
