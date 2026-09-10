void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);
  pinMode(6,INPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
}
void loop() {
  // put your main code here, to run repeatedly: press the top-left key on the RC transmitter to switch to RC mode; the boat starts in automatic mode (low level)
if(digitalRead(2) == HIGH){
  Serial.println("remote");
  if (digitalRead(3) == HIGH){
    Serial.println("33333");
    digitalWrite(9,HIGH);
  } else{
    digitalWrite(9,LOW);
  }
  if (digitalRead(4) == HIGH){
    Serial.println("444444");
    digitalWrite(8,HIGH);
  } else{
    digitalWrite(8,LOW);
  }
    if (digitalRead(5) == HIGH){
      Serial.println("5555");
    digitalWrite(10,HIGH);
  } else{
    digitalWrite(10,LOW);
  }
    if (digitalRead(6) == HIGH){
      Serial.println("6666");
    digitalWrite(11,HIGH);
  } else{
    digitalWrite(11,LOW);
  }
}
// press again to switch to automatic mode; the automatic code goes in the else block
// 8 and 9 drive the right propeller; 10 and 11 drive the left propeller
// the right propeller reverses when the black wire (8) is higher than the red wire (9), i.e. 8 HIGH and 9 LOW; 8 LOW and 9 HIGH gives forward. Equal levels stop it
// the left propeller reverses when the black wire (10) is higher than the red wire (11), i.e. 10 HIGH and 11 LOW; 10 LOW and 11 HIGH gives forward. Equal levels stop it
else{
  Serial.println("code control");
  if (Serial.available() > 0) { // check whether data has arrived on the serial port
    String data = Serial.readStringUntil('\n'); // read one line up to the newline
    // decode the data, split the string and extract the left and right power ratios
    String LeftFloatStr = data.substring(0, data.indexOf(','));
    String RightFloatStr = data.substring(data.indexOf(',') + 1);
    // convert the split strings to floats
    LeftFloat = LeftFloatStr.toFloat();
    RightFloat = RightFloatStr.toFloat();
  }
    for(int i=0;i<=5;i++){
      if (LeftFloat >= 0){
        digitalWrite(11,LOW);
        digitalWrite(10,HIGH);
        delayMicroseconds(3333*LeftFloat);
        digitalWrite(10,LOW);
        delayMicroseconds(3333*(1-LeftFloat));
        }
      else{
        digitalWrite(10,LOW);
        digitalWrite(11,HIGH);
        delayMicroseconds(-3333*LeftFloat);
        digitalWrite(11,LOW);
        delayMicroseconds(3333*(1+LeftFloat));
      }
    if (RightFloat >= 0){
        digitalWrite(8,LOW);
        digitalWrite(9,HIGH);
        delayMicroseconds(3333*RightFloat);
        digitalWrite(9,LOW);
        delayMicroseconds(3333*(1-RightFloat));
        }
      else{
        digitalWrite(9,LOW);
        digitalWrite(8,HIGH);
        delayMicroseconds(-3333*RightFloat);
        digitalWrite(8,LOW);
        delayMicroseconds(3333*(1+RightFloat));
      }
    }
}
}
