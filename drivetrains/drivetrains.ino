/*
 * You need the Bluepad32 Arduino library installed
 */
#include <Arduino.h>

#include "ArduinoController.h"
#include "src/drivetrain.h"
#include "src/arm.h"
#include <Bluepad32.h>
#include <cstdlib>

// use gpios 4-18, 21, 38, 39, 40, 41, 42, 47,48 for pwm
//// drivetrain
// Back right
#define kbr1 12
#define kbr2 13
// Back left
#define kbl1 14
#define kbl2 15
// Front right
#define kfr1 25
#define kfr2 26
// Front left
#define kfl1 32
#define kfl2 33

// Misc vars
#define maxSpeed 140
#define turnPower 140

//// Arm
#define kbase1 18
#define kmid1 19
#define kbase2 27
#define kclrot1 4
#define kclaw1 5
const float baseJointLength = 82.2; // mm
const float upperJointLength = 103.8; // mm
const int clawLength = 110; // mm
const int baseHeight = 40; // mm 




Arcade drivetrain(kbr1,kbr2,kbl1,kbl2,kfr1,kfr2,kfl1,kfl2);
Arm arm(kbase1,kbase2,kmid1,kclrot1,kclaw1,baseJointLength,upperJointLength,clawLength,baseHeight);




ControllerPtr contr[BP32_MAX_CONTROLLERS];

void onConnectedController(ControllerPtr cptr) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++){
        if (contr[i] == nullptr) {
            contr[i] = cptr;
            return;
        }
    }
}

void onDisconnectedController(ControllerPtr cptr) {    
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++){
        if (contr[i] == cptr) {
            contr[i] = nullptr;
            return;
        }
    }
}


int movamt = 20;
int baserot = 0;
int midrot = 0;
int lastTime;
void updateArmPosition(ControllerPtr cptr){
  if (millis()-lastTime < 500){
    return;
  }
  uint8_t dpad = cptr->dpad();
  if (dpad & DPAD_UP) {
      Serial.println("Arrow Up Pressed");
      arm.place();
  }
  
  if (dpad & DPAD_DOWN) {
      Serial.println("Arrow Down Pressed");
      arm.pickup();
  }

  if (dpad & DPAD_LEFT) {
      Serial.println("Arrow Left Pressed");
      arm.neutral();
      arm.setClawGrip(true);
  }

  if (dpad & DPAD_RIGHT) {
      Serial.println("Arrow Right Pressed");
      arm.stored();
  }
  if (cptr->l1()){
    arm.setClawWrist(true);
    Serial.println("wrist down");
  } else if (cptr -> r1()){
    arm.setClawWrist(false);
    Serial.println("wrist up");
  }

  if (cptr->b()){
    arm.setClawGrip(false);
    Serial.println("Claw Open");
  } else if (cptr->a()){
    arm.setClawGrip(true);
    Serial.println("Claw Closed");
  }

  lastTime = millis();
}

void processControllers(){
    for (auto cptr: contr) {
        if (!cptr) continue;

        if (cptr->isConnected() && cptr->hasData()){
            if (cptr->isGamepad()){
                
                drivetrain.updateMotor(
                    cptr->axisY(),
                    -cptr->axisRX()
                ); 
                updateArmPosition(cptr);
            }
        }

    }
}

void setup(){
    Serial.begin(115200);
    //lastTime = millis();
    BP32.setup(
            onConnectedController,
            onDisconnectedController
        );
    BP32.forgetBluetoothKeys();
  
    
    drivetrain.setMaxSpeed(maxSpeed);
    drivetrain.setTurnPower(turnPower); 

    arm.begin();
    arm.neutral();  
    arm.setClawWrist(false); 
    
    arm.setClawOCpoint(20,180); // manully put claw servo 0 if you want to switch the claw pieces
    arm.setClawGrip(true);
    
}


void loop(){
    
    if (BP32.update()){
        processControllers();
    }
}
