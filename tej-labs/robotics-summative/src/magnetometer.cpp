#include "./magnetometer.h"
#include "Wire.h"


MagnetometerStore::MagnetometerStore(){
    // Initialize library for magnetometer    
}

void MagnetometerStore::generate_tuned_values(){
    
}

void MagnetometerStore::fetch_data(uint32_t timestamp){
    
}

float MagnetometerStore::get(){
    return this->yaw;
}
