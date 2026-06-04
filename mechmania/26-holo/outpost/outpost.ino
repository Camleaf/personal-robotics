#include <Arduino.h>
#include <HardwareSerial.h>
#include "src/mechanisms.h"
#include "src/state.h"
#include <ESP32Servo.h>

#define rx 25 
#define tx 26
#define baud 9600 

HardwareSerial uartConnection(2);
RobotState* rState = new RobotState;


//// INTAKE
//
// Top roller motor
#define ku1_in 0
#define ku2_in 0

//// SHOOTER
// flywheel motors
#define kfly_sh 0
// servo signal
#define ksvb_sh 0
// feed motor
#define ku_sh 0
// beam break signal
#define kbm_sh 0

Intake* intake = new Intake(ku1_in,ku2_in);
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
      shooter->setAngle(80); 
  } else if (dpad & (1UL << 1)){ // DPAD down
      shooter->setAngle(30); 
  } else if (dpad & (1UL << 2)){ // DPAD right
      shooter->setAngle(45); 
  } else if (dpad & (1UL << 3)){ // DPAD left
      shooter->setAngle(60); 
  }

  ///////
  // Shooter settings
  ///////

  if (buttons & 1UL){ // X 
    //Enable shooting
    shooter->enabled(true);
  } else if (buttons & (1UL << 2)){ // Circle
    // Stuff off
    shooter->enabled(false);
    shooter->setFeed(0);
    intake->setSpeed(0);
  } else if (buttons & (1UL << 3)){ // Square
    // Shoot
    shooter->shoot();
  } 

  ////////
  // Intake settings
  ////////
  if (buttons & (1UL << 4)){ // Triangle     
    //Intake reversed
    intake->setSpeed(255,true);
    shooter->setFeed(0);
  } else if (buttons & (1UL << 5)){ // L1
    //Intake on
    intake->setSpeed(255);
    shooter->setFeed(50);
  } else if (buttons & (1UL << 6)){ // R1
    // Intake off
    intake->setSpeed(0);
    shooter->setFeed(0);
  }

  if (buttons & (1UL << 7)){ // L2

  } else if (buttons & (1UL << 8)){ // R2
  
  }

  
}

int lastTime = 0;

void loop() {
  if (uartConnection.available()) {
    rState->unload(uartConnection.read());
    if (millis()-lastTime>250){
      handleButtons();
      lastTime = millis();
    }
  }
}
