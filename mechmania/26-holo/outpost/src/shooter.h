#ifndef CAMLEAF_SHOOTER
#define CAMLEAF_SHOOTER

#include <cstdint>
class Shooter{
    public:         
        //flywheel motor out  //servo base  // servo upper // beam break in
        Shooter(uint8_t kfly, uint8_t ksvb, uint8_t ksvu, uint8_t kbm);
        
        void setAngle();
        void enabled(bool en);
        void lockInterrupt(); // auto lock on beam triggering
        void shoot();

    private:
        uint8_t kfly = 0;
        uint8_t ksvb = 0;
        uint8_t ksvu = 0;
        uint8_t kbm = 0;
        volatile bool locked = false;
};

#endif
