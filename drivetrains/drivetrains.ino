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
#define kbase1 32
#define kmid1 33
#define kclrot1 10
#define kclaw1 11
const int baseJointLength = 75; // mm
const int upperJointLength = 95; // mm
const int clawLength = 0; // mm
const int baseHeight = 10; // mm 




//Arcade drivetrain(kbr1,kbr2,kbl1,kbl2,kfr1,kfr2,kfl1,kfl2);
Arm arm(kbase1,kmid1,kclrot1,kclaw1,baseJointLength,upperJointLength,clawLength,baseHeight);




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
    
    arm.setBaseJointRange(0,135);
    arm.setClawOCpoint(0,180);
    arm.setUpperJointRange(0,180);
    arm.setClawPoint(50,50);
  }



void loop(){
    /*
    if (BP32.update()){
        processControllers();
    }
    */
    Serial.println(arm.servos[kbaseidx].read());
    arm.servos[kbaseidx].write(100);
    delay(800);
}
