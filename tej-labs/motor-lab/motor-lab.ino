#include "Arduino.h"


#define in1 3 
#define in2 4


void setup() {
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void loop() {
}
