/*
 * You need the Bluepad32 Arduino library installed/
 * Basically just zeroes every sero
 */
#include <Arduino.h>
#include <ESP32Servo.h>
Servo ground;  // create servo object to control a servo
Servo groundrev;
Servo mid;
// 16 servo objects can be created on the ESP32
 
int pos = 0;    // variable to store the servo position
int groundServo = 18;
int groundServoReversed = 13;
int middleServo = 10;
 
void setup() {
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	ground.setPeriodHertz(50);    // standard 50 hz servo
	ground.attach(groundServo); // attaches the servo on pin 18 to the servo object
                              //
	groundrev.setPeriodHertz(50);    // standard 50 hz servo
	groundrev.attach(groundServoReversed); // attaches the servo on pin 18 to the servo object
  ground.write(0); // Zero the servo
  groundrev.write(180);
}
 
void loop() {
}
