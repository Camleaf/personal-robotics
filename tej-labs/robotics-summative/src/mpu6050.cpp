#include <Wire.h>
#include "./orientationprovider.h"


GyroMPU6050::GyroMPU6050(){
    yaw = 0;
    lastTime = 0;

}

void GyroMPU6050::begin(){
    Wire.begin(21,22);
    delay(500);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B); 
    Wire.write(0x00);
    Wire.endTransmission(true);
    // set sample rate to 19
    Wire.beginTransmission(0x68);
    Wire.write(0x19); 
    Wire.write(0x09); 
    Wire.endTransmission(true);

    // set gyro range to 8g
    Wire.beginTransmission(0x68);
    Wire.write(0x1B); 
    Wire.write(0x08); // 0x08 = 500 deg/s
    Wire.endTransmission(true);
}



double findMin (double num1, double num2){
    return num1 < num2 ? num1 : num2;
}

double findMax(double num1, double num2){
    return num1 > num2 ? num1 : num2;
}

const int tuningCycles = 1000;
void GyroMPU6050::generate_tuned_values(){
    /* Must be flat and still for accurate resting threshold data*/
    Serial.println("generating tuned values for gyro");
    
    float avgGZ = 0.0f;
    int valReads = 0;

    for (int i = 0; i < tuningCycles; i++) {
        Wire.beginTransmission(0x68);
        Wire.write(0x47); //register for GYRO_ZOUT_H
        Wire.endTransmission();         
        Wire.requestFrom(0x68, 2); 
        if (Wire.available() == 2) {
            int16_t rawZ = Wire.read() << 8 | Wire.read();
            float gyroZ = rawZ / 65.5f; 
            
            avgGZ += gyroZ;
            valReads++;
        }
        delay(5);
    }

    if (valReads> 0) {
        this->yawHardBias = avgGZ / valReads;
        Serial.print("Yaw hard bias: ");
        Serial.println(this->yawHardBias);
    } else {
        Serial.println("No response");
    }    

}

void GyroMPU6050::fetch_data(uint32_t timestamp){ // use esp timer to get this to be accurate
    if (lastTime == 0){ lastTime = timestamp; return; }
    
    if (timestamp - lastTime < 20000){ return;}
    
    float delta = (timestamp - lastTime) / 1000000.0f;

    Wire.beginTransmission(0x68);
    Wire.write(0x47); // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 2);
    
    int16_t rawZ = Wire.read() << 8 | Wire.read();
    
    // 500 deg per s range = 65.5 whatever units read
    float gyroZ = (rawZ) / 65.5f; 
    
    yaw += (gyroZ) * delta;
    
    Serial.println(yaw);
    lastTime = timestamp;
}


float GyroMPU6050::get(){
    return this->yaw;
}

float GyroMPU6050::getRadians(){
    return this->yaw * (M_PI/180.f);
}
