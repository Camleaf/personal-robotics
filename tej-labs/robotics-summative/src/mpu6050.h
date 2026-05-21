#ifndef CAMLEAF_MPU6050
#define CAMLEAF_MPU6050
#include <Adafruit_MPU6050.h>

using namespace std;


class OrientStore {
    public:
        OrientStore();
        void create_threshold_values();
        
        
        void fetch_data(uint32_t timestamp);
        float get();

    private:
        Adafruit_MPU6050 mpu; 
        static constexpr float bias_filter_pass = 0.5f;
        
        // add hardcoded values for threshold stuff just as default In case I don't feel like recalibrating each boot
        float yawVariance = 0.f;
        float accelMagVariance = 0.f;
        
        float minAccelResting = 0.f;
        float maxAccelResting = 0.f;
        
        // Could dynamically update these
        float accelHardBias = 0.f;
        float yawHardBias = 0.f;
        

        // main collected value
        float yaw = 0;
        uint32_t lastTime = 0; // microseconds
};


#endif
