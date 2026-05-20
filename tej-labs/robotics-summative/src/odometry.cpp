#include <Adafruit_MPU6050.h>
#include "./drivetrain.h"


Adafruit_MPU6050 mpu;

// Generated
#define OFF_AX 0
#define OFF_AY 0
#define OFF_AZ 0
#define OFF_GX 0
#define OFF_GY 0
#define OFF_GZ 0

void create_odometry(){

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        // should get a neopixel for the final project to display odometry status but an LED works for now

    }
    Serial.println("MPU6050 Found!");
    mpu.begin();
    
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setSampleRateDivisor(9);
    mpu.setFilterBandwidth(MPU6050_BAND_184_HZ); // may need to go lower if too high

    Serial.print("accelerometre: ");
    Serial.println(mpu.getAccelerometerRange()); 
  
    Serial.print("gyro: ");
    Serial.println(mpu.getGyroRange());

    
}



const int tuningCycles = 1000;
void create_tuned_values(){
    /* Must be flat and still for accurate data*/
    double errorAX, errorAY, errorAZ, errorGX, errorGY, errorGZ;

    for (int i = 0;i<tuningCycles;i++){
        
        
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        errorAX += a.acceleration.x;        
        errorAY += a.acceleration.y;        
        errorAZ += a.acceleration.z;        
        errorGX += g.acceleration.x;        
        errorGY += g.acceleration.y;        
        errorGZ += g.acceleration.z;        
        
        delay(15);
    }
            
    Serial.println("Calculated Errors");
    
    Serial.println("Accel: ");
    Serial.printf("X: %f\n", -errorAX/tuningCycles);
    Serial.printf("Y: %f\n", -errorAY/tuningCycles);
    Serial.printf("Z: %f\n", -errorAZ/tuningCycles);

    Serial.println("Gyro: ");
    Serial.printf("X: %f\n", -errorGX/tuningCycles);
    Serial.printf("Y: %f\n", -errorGY/tuningCycles);
    Serial.printf("Z: %f\n", -errorGZ/tuningCycles);

}
