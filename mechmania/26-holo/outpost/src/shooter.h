#ifndef CAMLEAF_SHOOTER
#define CAMLEAF_SHOOTER

#include <cstdint>
class Shooter{
    public:         
        //flywheel motor out  //servo base  // motor upper // beam break in
        Shooter(uint8_t kfly, uint8_t ksvb, uint8_t ku, uint8_t kbm);
        
        void setAngle(int angle);
        void enabled(bool en); // flywheels on/off
        
        void feedInterrupt(); // auto lock feed roller on beam triggering
        void shoot(); // run feed roller to push into flywheel.
        void startFeed(); // start the feed roller. Will get auto stopped by lockInterrupt.
    
    private:
        uint8_t kfly = 0;
        uint8_t ksvb = 0;
        uint8_t ku = 0;
        uint8_t kbm = 0;
        volatile bool locked = false;
};

#endif
