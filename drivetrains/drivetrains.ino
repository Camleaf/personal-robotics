/*
 * You need the Bluepad32 Arduino library installed
 */
#include <Arduino.h>

#include "ArduinoController.h"
#include "src/drivetrain.h"
#include "src/arm.h"
#include "src/shooter.h"
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
#define maxSpeed 128
#define turnPower 128

//// Arm
#define kbase1 18
#define kmid1 19
#define kbase2 27
#define kclrot1 10
#define kclaw1 11
const float baseJointLength = 82.2; // mm
const float upperJointLength = 103.8; // mm
const int clawLength = 0; // mm
const int baseHeight = 40; // mm 




//Arcade drivetrain(kbr1,kbr2,kbl1,kbl2,kfr1,kfr2,kfl1,kfl2);
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

int armx = 100;
int army = 100;
int movamt = 3;
int baserot = 0;
int midrot = 0;

void updateArmPosition(ControllerPtr cptr){
  /*
  uint8_t dpad = cptr->dpad();
  if (dpad & DPAD_UP) {
        Serial.println("Arrow Up Pressed");
        if (arm.setClawPoint(armx,army+movamt)){
            army+=movamt;
        }
          
    }
    
    if (dpad & DPAD_DOWN) {
        Serial.println("Arrow Down Pressed");
        if (arm.setClawPoint(armx,army-movamt)){
            army-=movamt;
        }
    }

    if (dpad & DPAD_LEFT) {
        Serial.println("Arrow Left Pressed");
        if (arm.setClawPoint(armx-movamt,army)){
            armx-=movamt;
        }
    }

    if (dpad & DPAD_RIGHT) {
        Serial.println("Arrow Right Pressed");
        if (arm.setClawPoint(armx+movamt,army)){
            armx+=movamt;
        }
    }
    delay(20); // Todo make non-blocking so that drivetrain could be run at same time
    */
    int rawLeft = cptr->axisY();
    int rawRight = cptr->axisRY();

    int baseVect,midVect = 0;
    
    if (abs(rawLeft)>50){
      baseVect = map(rawLeft,-512,512,-movamt,movamt);
      Serial.println(rawLeft);
      Serial.println(baseVect);
      if (arm.setBaseRot(baseVect+baserot)){
        baserot = baseVect+baserot;
        Serial.printf("baserot %d\n",baserot);
      }
    }
    if (abs(rawRight)>50){
      midVect = map(rawRight,-512,512,-movamt,movamt);
      Serial.printf("midvect %d\n",midVect);

      int tmp = midVect+midrot;
      if (arm.setMidRot(tmp)){
        midrot = tmp;
        Serial.printf("midrot %d\n",midrot);
      }
    }

    delay(30);
}

void processControllers(){
    for (auto cptr: contr) {
        if (!cptr) continue;

        if (cptr->isConnected() && cptr->hasData()){
            if (cptr->isGamepad()){
                /*
                drivetrain.updateMotor(
                    cptr->axisX(),
                    cptr->axisY()
                    //cptr->axisY()
                );*/ 
                updateArmPosition(cptr);
            }
        }

    }
}

void setup(){
    Serial.begin(115200);
    
    BP32.setup(
            onConnectedController,
            onDisconnectedController
        );
    BP32.forgetBluetoothKeys();
  
    /*
    drivetrain.setMaxSpeed(maxSpeed);
    drivetrain.setTurnPower(turnPower);
    drivetrain.invertMotor(0,true); // invert backright
    drivetrain.invertMotor(1,true); // invert frontright
    */
    arm.begin();
    arm.neutral();  
}


void loop(){
    
    if (BP32.update()){
        processControllers();
    }
}
