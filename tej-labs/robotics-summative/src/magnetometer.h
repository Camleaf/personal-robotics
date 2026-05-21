#ifndef CAMLEAF_MAGNETOMETER 
#define CAMLEAF_MAGNETOMETER

using namespace std;


class MagnetometerStore{
    public:
        MagnetometerStore();
        
        void generate_tuned_values();
        void fetch_data(uint32_t timestamp);
        float get(); 

    private:
       
        // hard iron offsets
        const float hardX = 0;
        const float hardY = 0;

        // soft iron offset factors
        const float softX = 0;
        const float softY = 0;


        const float yaw = 0;
        uint32_t lastTime = 0;
};


#endif
