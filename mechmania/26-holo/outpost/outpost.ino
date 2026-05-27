#include <Arduino.h>
#include <HardwareSerial.h>
#include "src/shooter.h"
#include "src/intake.h"
#include "src/state.h"
#include <Bluepad32.h>

#define rx 16
#define tx 17
#define baud 115200

HardwareSerial uartConnection(2);
StateAssign rStateAssign;
RobotState* rState = &rStateAssign.state;

void setup() {
  Serial.begin(9600);
  
  uartConnection.begin(baud, SERIAL_8N1, rx, tx);
  Serial.println("uart init");
}


void loop() {
  if (uartConnection.available()) {
    rStateAssign.raw = uartConnection.read();
    rState->buttons;
  }
}
