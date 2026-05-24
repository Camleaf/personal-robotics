#ifndef CAMLEAF_ORIENTPROVIDER
#define CAMLEAF_ORIENTPROVIDER



#include <QMC5883LCompass.h>

class OrientationProvider {
    public:
        virtual void begin() = 0;
        virtual float get() = 0;
        virtual float getRadians() = 0;
        virtual void generate_tuned_values() = 0;
        virtual void fetch_data(uint32_t timestamp) = 0; 
        void setYaw(float yaw);

    protected:
        volatile float yaw = 0;
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
        static constexpr float bias_filter_pass = 0.5f;
        static constexpr int MPUaddr = 0x68; 

        float yawHardBias = 0.f;
         

        // main collected value
        uint32_t lastTime = 0; // microseconds
};

#endif
