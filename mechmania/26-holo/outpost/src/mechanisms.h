#ifndef CAMLEAF_MECANISMS
#define CAMLEAF_MECANISMS

#include <cstdint>
#include <ESP32Servo.h>
#include <driver/mcpwm.h>
#include <sys/types.h>

void setMotor(mcpwm_unit_t unit, mcpwm_timer_t timer, float val);

class Shooter{
    public:         
        //flywheel motor out  //servo base  // motor feed // beam break in
        Shooter(uint8_t kfly, uint8_t kfly2, int8_t ksvb, uint8_t ku, uint8_t kbm);
        
        void begin();
        void setAngle(int angle);
        void enabled(bool en); // flywheels on/off
        
        static void feedInterrupt(void* arg); // auto lock feed roller on beam triggering
        void shoot(); // run feed roller to push into flywheel.
        void setFeed(uint8_t speed=50); // start the feed roller. Will get auto stopped by lockInterrupt.
    
    private:
        uint8_t kfly = 0;
        uint8_t ksvb = 0;
        uint8_t ku = 0;
        uint8_t kbm = 0;
        uint8_t kfly2 = 0;
        volatile bool locked = false;
        volatile bool shooter_running = false;
        Servo srv;
};


class Intake{
    public:
        //  upper roller motor outs
        Intake(uint8_t ku, uint8_t ku2);

        void setSpeed(uint8_t speed, bool reversed = false);
        void off();

    private:
        uint8_t ku = 0;
        uint8_t ku2 = 0;
};


#endif
