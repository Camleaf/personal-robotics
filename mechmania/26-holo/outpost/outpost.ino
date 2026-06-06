#include <Arduino.h>
#include <HardwareSerial.h>
#include "src/mechanisms.h"
#include "src/state.hpp"
#include <ESP32Servo.h>

#define rx 18 
#define tx 23
#define baud 9600 

HardwareSerial uartConnection(2);
RobotState* rState = new RobotState;


//// INTAKE
//
// Top roller motor
#define ku_in 5

//// SHOOTER
// flywheel motors
#define kfly_sh 0
// servo signal
#define ksvb_sh 32
// feed motor
#define ku_sh 0
// beam break signal
#define kbm_sh 0

Intake* intake = new Intake(ku_in);
Shooter* shooter = new Shooter(kfly_sh,ksvb_sh,ku_sh,kbm_sh);


void setup() {
  Serial.begin(9600);
  ESP32PWM::allocateTimer(0);
  shooter->begin();

  uartConnection.begin(baud, SERIAL_8N1, rx, tx);
  Serial.println("uart init");
}


void handleButtons(){
  
  uint16_t buttons = rState->buttons;
  uint8_t dpad = rState->dpad;

  /////////
  // Shooter angles
  /////////

  if (dpad & 1UL){ // DPAD up
      shooter->setAngle(120); 
  } else if (dpad & (1UL << 1)){ // DPAD down
      shooter->setAngle(180); 
  } else if (dpad & (1UL << 2)){ // DPAD right
      shooter->setAngle(160); 
  } else if (dpad & (1UL << 3)){ // DPAD left
      shooter->setAngle(140); 
  }

  ///////
  // Shooter settings
  ///////

  if (buttons & 1UL){ // X 
    //Enable shooting
    shooter->enabled(true);
  } else if (buttons & (1UL << 1)){ // Circle
    // Stuff off
    shooter->enabled(false);
    shooter->setFeed(0);
    intake->setSpeed(0);
  } else if (buttons & (1UL << 2)){ // Square
    // Shoot
    shooter->shoot();
  } 

  ////////
  // Intake settings
  ////////
  if (buttons & (1UL << 3)){ // Triangle     
  } else if (buttons & (1UL << 4)){ // L1
    //Intake on
    intake->setSpeed(255);
    shooter->setFeed(50);
  } else if (buttons & (1UL << 5)){ // R1
    // Intake off
    intake->setSpeed(0);
    shooter->setFeed(0);
  }

  if (buttons & (1UL << 6)){ // L2

  } else if (buttons & (1UL << 7)){ // R2
  
  }

  
}

int lastTime = 0;
uint32_t buf;

void loop() {
  if (uartConnection.available() >= sizeof(buf)){ // 4 Bytes, or size of sent message 
    uartConnection.read((uint8_t*)&buf, sizeof(buf));
    rState->unload(buf);
    if (millis()-lastTime>250){
      handleButtons();
      Serial.println(rState->dpad);
      lastTime = millis();
    }
  }
}
