/*
 * You need the Bluepad32 Arduino library installed
 */
#include <Arduino.h>

#include "src/drivetrain.h"
#include "src/arm.h"
#include <Bluepad32.h>


// Back right
#define kbr1 0
#define kbr2 0
// Back left
#define kbl1 0
#define kbl2 0
// Front right
#define kfr1 0
#define kfr2 0
// Front left
#define kfl1 0
#define kfl2 0

// Misc vars
#define maxSpeed 128
#define turnPower 128


Mecanum drivetrain(kbr1,kbr2,kbl1,kbl2,kfr1,kfr2,kfl1,kfl2);





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
                drivetrain.updateMotor(
                    cptr->axisX(),
                    cptr->axisRX(),
                    cptr->axisY()
                ); 
            }
        }

    }
}


void SetupBP32(){
    BP32.setup(
            onConnectedController,
            onDisconnectedController
        );
    BP32.forgetBluetoothKeys();
}



void setup(){
    Serial.begin(115200);

    SetupBP32();
    drivetrain.setMaxSpeed(maxSpeed);
    drivetrain.setTurnPower(turnPower);
}



void loop(){
    
    if (BP32.update()){
        processControllers();
    }
}
