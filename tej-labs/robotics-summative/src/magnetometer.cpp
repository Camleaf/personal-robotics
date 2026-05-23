#include "./orientationprovider.h"
#include <QMC5883LCompass.h>
#include "Wire.h"

Magnetometer::Magnetometer(){
    // Initialize library for magnetometer    
}


void Magnetometer::begin(){
    
    qmc.init();
    qmc.setMode(0x01,0x04,0x10,0xC0);
    qmc.setCalibrationOffsets(-577.00, 499.00, 2442.00);
    qmc.setCalibrationScales(2.06, 2.18, 0.49);
    x = 0;
    y = 0;
}

void Magnetometer::generate_tuned_values(){
    
}

void Magnetometer::fetch_data(uint32_t timestamp){
        
    if (lastTime == 0) {
        lastTime = timestamp;
        return;
    } else if (timestamp-lastTime < 40000) return;
    

    qmc.read();

    this->x = (qmc.getX() * this->alpha_low_pass) + (this->x* (1.0 - this->alpha_low_pass));
    this->y = (qmc.getY() * this->alpha_low_pass) + (this->y* (1.0 - this->alpha_low_pass));
    this->yaw = atan2f((float)y,(float)x) + M_PI;
    Serial.println(this->yaw);
    lastTime = timestamp; 
}

float Magnetometer::get(){
    return this->yaw;// ADJUST IF MAGNETOMETER GIVES IN RADIANS OR DEGREES 
}


float Magnetometer::getRadians(){
    return this->yaw; // ADJUST IF MAGNETOMETER GIVES IN RADIANS OR DEGREES
}
