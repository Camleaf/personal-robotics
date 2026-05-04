
#include <Adafruit_NeoPixel.h>

#define LED_COUNT 4
#define NEOPIXEL_PIN 6

Adafruit_NeoPixel px(LED_COUNT, NEOPIXEL_PIN, NEO_KHZ800);

void setup(){
    px.begin();
    px.show();
    px.fill(px.Color(255,0,0),0,NEOPIXEL_PIN);
}



void loop(){

}
