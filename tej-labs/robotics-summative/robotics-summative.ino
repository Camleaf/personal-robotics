/*
 * You need the Bluepad32 Arduino library installed
 */
#include <Arduino.h>

#include "ArduinoController.h"
#include "src/drivetrain.h"
#include <Bluepad32.h>
#include <cstdlib>

// use gpios 4-18, 21, 38, 39, 40, 41, 42, 47,48 for pwm
//// drivetrain
// Back right
#define kbr1 12
#define kbr2 13
// Back left
#define kbl1 25
#define kbl2 26
// Front right
#define kfr1 22
#define kfr2 23
// Front left
#define kfl1 18
#define kfl2 19

// Misc vars
#define maxSpeed 140
#define turnPower 140




Mecanum drivetrain(kbr1,kbr2,kbl1,kbl2,kfr1,kfr2,kfl1,kfl2);




ControllerPtr contr[BP32_MAX_CONTROLLERS];

void onConnectedController(ControllerPtr cptr) {
    if (!cptr) return;
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++){
        if (contr[i] == nullptr) {
            contr[i] = cptr;
            Serial.println("add successfull");
            return;
        }
    }
}

void onDisconnectedController(ControllerPtr cptr) {   
    if (!cptr) return;
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
                
                drivetrain.updateMotor(
                    cptr->axisX(),
                    cptr->axisRX(),
                    cptr->axisY()
                ); 
            }
        }

    }
}

void setup(){
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
      contr[i] = nullptr;
    }
    Serial.begin(115200);
    //lastTime = millis();
    
    BP32.setup(
            onConnectedController,
            onDisconnectedController
        );
  
    
    drivetrain.setMaxSpeed(maxSpeed);
    drivetrain.setTurnPower(turnPower); 

}


void loop(){
    
    if (BP32.update()){
        processControllers();
    }
}
