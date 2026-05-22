#include "./orientationprovider.h"
#include "Wire.h"


Magnetometer::Magnetometer(){
    // Initialize library for magnetometer    
}

void Magnetometer::generate_tuned_values(){
    
}

void Magnetometer::fetch_data(uint32_t timestamp){
    
}

float Magnetometer::get(){
    return this->yaw;// ADJUST IF MAGNETOMETER GIVES IN RADIANS OR DEGREES 
}


float Magnetometer::getRadians(){
    return this->yaw; // ADJUST IF MAGNETOMETER GIVES IN RADIANS OR DEGREES
}
