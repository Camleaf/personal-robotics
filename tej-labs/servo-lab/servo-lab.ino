#include <Arduino.h>
#include <Servo.h> 


#define signalpin 6
Servo s;

void setup() {
  pinMode(signalpin, OUTPUT);
  s.attach(signalpin);
  s.write(0);
}

void loop() {
    s.write(180);
    delay(500);
    s.write(0);
    delay(500);
}
