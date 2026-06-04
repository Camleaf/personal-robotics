/*
 * You need the Bluepad32 Arduino library installed
 */
#include <Arduino.h>

#include "ArduinoController.h"
#include "src/drivetrain.h"
#include "src/state.hpp"
#include "src/orientationprovider.h"
#include <Bluepad32.h>
#include <HardwareSerial.h>
#include <cstdlib>

//// drivetrain
// Back right
#define kbr1 12
#define kbr2 13
// Back left
#define kbl1 26
#define kbl2 27
// Front right
#define kfr1 18
#define kfr2 19
// Front left
#define kfl1 15
#define kfl2 5

// Misc vars
#define maxSpeed 140
#define turnPower 140

#define rx 32
#define tx 33
#define baud 9600 

HardwareSerial uartConnection(2);
RobotState* rState = new RobotState();

OrientationProvider* orientStore = new GyroMPU6050(); // reserve the sda and scl pins. 21 SDA, 22 SCL
// Only making one of these so should be fine to use new
Drivetrain* drivetrain = new FieldMecanum(kbr1,kbr2,kbl1,kbl2,kfr1,kfr2,kfl1,kfl2,orientStore);


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
            drivetrain->updateMotor(0, 0, 0, 0);
            return;
        }
    }
    
}

uint32_t optionsTimeout;
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
                  
                rState->buttons = cptr->buttons();
                rState->dpad = cptr->dpad();

                if (millis() - optionsTimeout > 1000 && cptr->miscSelect()){
                  orientStore->setYaw(0);
                  optionsTimeout = millis();
                }
            }
        }


        

    }
}

uint32_t msgInterval = 0;
void msgCoproc(){
    if (millis()-msgInterval > 250) {
        int data = rState->getInt();
        uartConnection.write((uint8_t*)&data, sizeof(data));
        msgInterval = millis();
    }
}

void setup(){
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
      contr[i] = nullptr;
    }
    Serial.begin(9600);
    uartConnection.begin(baud,SERIAL_8N1,rx,tx);
    
    orientStore->begin();
    delay(100);
    orientStore->generate_tuned_values();

    BP32.setup(
            onConnectedController,
            onDisconnectedController
        );
  
     
    drivetrain->setMaxSpeed(maxSpeed);
    drivetrain->setTurnPower(turnPower); 

    drivetrain->invertMotor(0,true);
    drivetrain->invertMotor(3,true);
 
    optionsTimeout = millis();
    delay(500);
}


void loop(){
    msgCoproc();
    orientStore->fetch_data(esp_timer_get_time());
    if (BP32.update()){
        processControllers();
    }

    delay(5);

}
