#include "Arduino.h"


#define beamin A0

void setup() {
  Serial.begin(115200);
  pinMode(beamin,INPUT);
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
}

void loop() {
    int in = analogRead(beamin);
    Serial.println(in);

    if (in == 1023){digitalWrite(13,HIGH);}
    else {
      digitalWrite(13,LOW);
    }

    delay(10);
}
