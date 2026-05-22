/*
 * You need the Bluepad32 Arduino library installed
 */
#include <Arduino.h>

#include "ArduinoController.h"
#include "src/drivetrain.h"
#include "src/mpu6050.h"
#include <Bluepad32.h>
#include <cstdlib>

//// drivetrain
// Back right
#define kbr1 12
#define kbr2 13
// Back left
#define kbl1 25
#define kbl2 26
// Front right
#define kfr1 22 // switch this because it intersects with sda
#define kfr2 23
// Front left
#define kfl1 18
#define kfl2 19

// Misc vars
#define maxSpeed 140
#define turnPower 140




Drivetrain* drivetrain = new Mecanum(kbr1,kbr2,kbl1,kbl2,kfr1,kfr2,kfl1,kfl2);
OrientationProvider* orientStore = new GyroMPU6050(); // reserve the sda and scl pins. 21 SDA, 22 SCL
// Only making one of these so should be fine to use new


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
                
                drivetrain->updateMotor(
                    cptr->axisX(),
                    cptr->axisRX(),
                    -cptr->axisY(),
                    cptr->axisRY()
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
    
    BP32.setup(
            onConnectedController,
            onDisconnectedController
        );
  
     
    drivetrain->setMaxSpeed(maxSpeed);
    drivetrain->setTurnPower(turnPower); 

    drivetrain->invertMotor(2,true);
    drivetrain->invertMotor(1,true);

    orientStore->generate_tuned_values();
}


void loop(){
    orientStore->fetch_data(esp_timer_get_time());   
    if (BP32.update()){
        processControllers();
    }
}
