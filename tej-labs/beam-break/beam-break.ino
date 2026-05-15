#include "Arduino.h"


#define beamin 6

void setup() {
  pinMode(beamin,INPUT);
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
}

void loop() {
    bool in = digitalRead(beamin);

    if (in){
      digitalWrite(13,HIGH);
    } else {
      digitalWrite(13,LOW);
    }

    delay(10);
}
