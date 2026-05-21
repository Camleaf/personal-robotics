#include <Adafruit_MPU6050.h>
#include "./drivetrain.h"


Adafruit_MPU6050 mpu;

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



double findMin (double num1, double num2){
    return num1 < num2 ? num1 : num2;
}

double findMax(double num1, double num2){
    return num1 > num2 ? num1 : num2;
}

const int tuningCycles = 1000;
void create_threshold_values(){
    /* Must be flat and still for accurate resting threshold data*/
    double minRestingMagA, maxRestingMagA, thresholdA;
    double minGZ, maxGZ, thresholdGZ;

    double avgMagA, avgGZ;

    for (int i = 0;i<tuningCycles;i++){
        
        
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        float magA = std::sqrt(pow(a.acceleration.x,2) + pow(a.acceleration.y,2)+ pow(a.acceleration.z,2));
        
        maxRestingMagA = findMax(maxRestingMagA,magA);
        minRestingMagA = findMin(minRestingMagA,magA);

        maxGZ = findMax(maxGZ, g.acceleration.heading);
        minGZ = findMin(minGZ, g.acceleration.heading);
        

        avgMagA += magA;
        avgGZ += g.acceleration.heading;

        delay(15);
    }
            
    // Get maximum variance from average
    thresholdA = abs((avgMagA/tuningCycles) - max(abs(minRestingMagA),abs(maxRestingMagA)));
    thresholdGZ = abs((avgGZ/tuningCycles) - max(abs(minGZ),abs(maxGZ)));
    

    Serial.println("Threshold values:");
    Serial.print("Accel magnitude variance: ");
    Serial.println(thresholdA);

    Serial.print("Gyro yaw variance: ");
    Serial.println(thresholdGZ);

    Serial.println("Resting accel magnitude: ");
    Serial.print("Min: ");
    Serial.println(minRestingMagA);
    Serial.print("Max: ");
    Serial.println(maxRestingMagA);

}
