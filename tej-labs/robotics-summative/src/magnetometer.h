#ifndef CAMLEAF_MAGNETOMETER 
#define CAMLEAF_MAGNETOMETER

#include "./drivetrain.h"

using namespace std;


class Magnetometer: public OrientationProvider{
    public:
        Magnetometer();
        
        void generate_tuned_values() override;
        void fetch_data(uint32_t timestamp) override;
        float get() override; 

    private:
       
        // hard iron offsets
        const float hardX = 0;
        const float hardY = 0;

        // soft iron offset factors
        const float softX = 0;
        const float softY = 0;


        float yaw = 0;
        uint32_t lastTime = 0;
};


#endif
