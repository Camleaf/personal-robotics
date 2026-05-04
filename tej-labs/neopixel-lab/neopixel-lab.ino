
#include <Adafruit_NeoPixel.h>

#define LED_COUNT 4
#define NEOPIXEL_PIN 6

Adafruit_NeoPixel px(LED_COUNT, NEOPIXEL_PIN, NEO_GRB);

void setup(){
    px.begin();
    px.show();
    for (int i = 0;i < LED_COUNT;i++){
      px.setPixelColor(i,px.Color(255,0,0));
    }
  
    px.setBrightness(100);
    px.show();
}


uint32_t colList[] = {
  px.Color(255,0,0),
  px.Color(0,255,0),
  px.Color(0,0,255)
};

int idx = 0;
void loop(){
     
    delay(300);
    for (int i = 0; i<LED_COUNT;i++){
      px.setPixelColor(i,colList[idx]);
      delay(100);
      px.show();
    }
    px.show();
    idx++;
    idx %=3;
}
