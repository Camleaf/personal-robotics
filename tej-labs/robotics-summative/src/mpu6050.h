#ifndef CAMLEAF_MPU6050
#define CAMLEAF_MPU6050


using namespace std;


class OrientStore {
    public:
        OrientStore();
        
        void create_threshold_values();

    private:
        static constexpr float bias_filter_pass = 0.5f;

        float yawVariance = 0.f;
        float accelMagVariance = 0.f;
        
        float minAccelResting = 0.f;
        float maxAccelResting = 0.f;
};


#endif
