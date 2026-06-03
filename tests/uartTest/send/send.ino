
#include <HardwareSerial.h>



#define rx 32
#define tx 33
#define baud 115200

#include <cstdint>
struct  [[gnu::packed]] RobotState{
    uint16_t buttons = 0;
    uint8_t dpad = 0;
    int getInt(){
      return buttons << 16 | dpad;
    }

    void unload(int raw){
      dpad = raw | ((1UL<<8) - 1);
      raw >>= 8;
      buttons = raw | ((1UL<<16)-1);
    }
};





RobotState* rState = new RobotState();

HardwareSerial uartConnection(2);

void setup(){
  Serial.begin(115200);
  uartConnection.begin(baud, SERIAL_8N1, rx, tx);
  Serial.println("uart init");

}
uint32_t msgInterval = 0;
void loop(){
    if (millis()-msgInterval > 250) {
        int data = rState->getInt();
        uartConnection.write((uint8_t*)&data, sizeof(data));
        msgInterval = millis();
        rState->buttons += 1;
        Serial.println(rState->buttons);
    }
}
