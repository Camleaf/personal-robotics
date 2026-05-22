#include "./orientationprovider.h"
#include "Adafruit_QMC5883P.h"
#include "Wire.h"

Magnetometer::Magnetometer(){
    // Initialize library for magnetometer    
    this->qmc = Adafruit_QMC5883P();
    Serial.println("QMC5883P Test");

    if (!qmc.begin()) {
        Serial.println("Failed to find QMC5883P chip");
        return;
    }

    Serial.println("QMC5883P Found!");

    qmc.setMode(QMC5883P_MODE_NORMAL);

    qmc.setODR(QMC5883P_ODR_50HZ);
    qmc.setOSR(QMC5883P_OSR_4);
    qmc.setDSR(QMC5883P_DSR_2);
    qmc.setRange(QMC5883P_RANGE_8G);
    qmc.setSetResetMode(QMC5883P_SETRESET_ON);

}

void Magnetometer::generate_tuned_values(){
    
}

void Magnetometer::fetch_data(uint32_t timestamp){
    if (qmc.isDataReady()){
    } 
}

float Magnetometer::get(){
    return this->yaw;// ADJUST IF MAGNETOMETER GIVES IN RADIANS OR DEGREES 
}


float Magnetometer::getRadians(){
    return this->yaw; // ADJUST IF MAGNETOMETER GIVES IN RADIANS OR DEGREES
}
