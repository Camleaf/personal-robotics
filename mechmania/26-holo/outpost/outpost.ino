#include <Arduino.h>
#include <HardwareSerial.h>
#include "src/mechanisms.h"
#include "src/state.h"
#include <Bluepad32.h>
#include <ESP32Servo.h>

#define rx 16
#define tx 17
#define baud 115200

HardwareSerial uartConnection(2);
StateAssign rStateAssign;
RobotState* rState = &rStateAssign.state;


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

  if (dpad & 1UL){ // DPAD up
  
  } else if (dpad & (1UL << 1)){ // DPAD down

  } else if (dpad & (1UL << 2)){ // DPAD right

  } else if (dpad & (1UL << 3)){ // DPAD left
  
  }


  if (buttons & 1UL){ // X 

  } else if (buttons & (1UL << 2)){ // Circle

  } else if (buttons & (1UL << 3)){ // Square

  } else if (buttons & (1UL << 4)){ // Triangle

  }

  if (buttons & (1UL << 5)){ // L1

  } else if (buttons & (1UL << 6)){ // R1
  
  }

  if (buttons & (1UL << 7)){ // L2

  } else if (buttons & (1UL << 8)){ // R2
  
  }

  
}

int lastTime = 0;

void loop() {
  if (uartConnection.available()) {
    rStateAssign.raw = uartConnection.read();
    rState->buttons;
  }

  if (millis()-lastTime>250){
    handleButtons();
    lastTime = millis();
  }

}
