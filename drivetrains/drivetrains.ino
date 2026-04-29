/*
 * You need the Bluepad32 Arduino library installed
 */
#include <Arduino.h>

#include "src/drivetrain.h"
#include "src/arm.h"
#include "src/shooter.h"
#include <Bluepad32.h>

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
const int baseHeight = 10; // mm 




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
                
            }
        }

    }
}



int armx = baseJointLength+upperJointLength;
int army = 0;

void setup(){
    Serial.begin(115200);
    /*
    BP32.setup(
            onConnectedController,
            onDisconnectedController
        );
    BP32.forgetBluetoothKeys();
  
    
    drivetrain.setMaxSpeed(maxSpeed);
    drivetrain.setTurnPower(turnPower);
    drivetrain.invertMotor(0,true); // invert backright
    drivetrain.invertMotor(1,true); // invert frontright
    */
    arm.begin();
    arm.setClawPoint(-10,70);
    //arm.zero();  
}



void loop(){
    /*
    if (BP32.update()){
        processControllers();
    }
    */
    /*
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '+') armx += 5;
      if (c == '-') armx -= 5;
      
      army = sqrt(pow(baseJointLength+upperJointLength,2)-pow(armx,2));
      Serial.printf("army %d", army);
      arm.setClawPoint(armx,army);

    }*/
    /*
    for (int i = -180; i <= 180; i+=5){
      army = sqrt(pow(baseJointLength+upperJointLength,2)-pow(i,2));
      arm.setClawPoint(i,army);
      delay(15);
    }
    
    
    for (int i = 180; i >= -180; i-=5){
      army = sqrt(pow(baseJointLength+upperJointLength,2)-pow(i,2));
      arm.setClawPoint(i,army);
      delay(15);
    }*/

}
