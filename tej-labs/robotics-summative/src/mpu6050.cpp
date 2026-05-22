#include <Wire.h>
#include "./orientationprovider.h"


GyroMPU6050::GyroMPU6050(){
    Wire.begin(21,22); // SDA and SCL
    mpu = Adafruit_MPU6050();

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        // should get a neopixel for the final project to display odometry status but an LED works for now
        return;
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
    yaw = 0;
    lastTime = 0;

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
    float minRestingMagA, maxRestingMagA, thresholdA;
    float minGZ, maxGZ, thresholdGZ;

    float avgMagA, avgGZ;

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

    this->minAccelResting = minRestingMagA;
    this->maxAccelResting = maxAccelResting;
    this->accelMagVariance = thresholdA;
    this->accelHardBias = avgMagA / tuningCycles;
    this->yawVariance = thresholdGZ;
    this->yawHardBias = avgGZ / tuningCycles;

}

void GyroMPU6050::fetch_data(uint32_t timestamp){ // use esp timer to get this to be accurate
    if (lastTime == 0){
        lastTime = timestamp;
        return;
    }
    
    uint32_t delta = (lastTime - timestamp) /1000000.0f;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    yaw += g.acceleration.heading * delta;
    
    lastTime = timestamp;
}


float GyroMPU6050::get(){
    return this->yaw;
}

float GyroMPU6050::getRadians(){
    return this->yaw * (M_PI/180.f);
}
