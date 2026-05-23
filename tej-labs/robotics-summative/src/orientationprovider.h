#ifndef CAMLEAF_ORIENTPROVIDER
#define CAMLEAF_ORIENTPROVIDER


#include <Adafruit_MPU6050.h>
#include <QMC5883LCompass.h>

class OrientationProvider {
    public:
        virtual void begin() = 0;
        virtual float get() = 0;
        virtual float getRadians() = 0;
        virtual void generate_tuned_values() = 0;
        virtual void fetch_data(uint32_t timestamp) = 0; 
};



class Magnetometer: public OrientationProvider{
    public:
        Magnetometer();
        void begin() override;
        void generate_tuned_values() override;
        void fetch_data(uint32_t timestamp) override;
        float get() override; 
        float getRadians() override;

    private:
        QMC5883LCompass qmc;       
        
        const float alpha_low_pass = 0.15f;
        volatile float x;
        volatile float y;

        volatile float yaw = 0;
        uint32_t lastTime = 0;

    
};

class  GyroMPU6050: public OrientationProvider{
    public:
        GyroMPU6050();
        void generate_tuned_values() override;
        void begin() override;
        
        void fetch_data(uint32_t timestamp) override;
        float get() override;
        float getRadians() override;

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
        volatile float yaw = 0;
        uint32_t lastTime = 0; // microseconds
};

#endif
