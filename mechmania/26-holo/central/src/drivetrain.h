#ifndef CAMLEAF_DRIVETRAINS
#define CAMLEAF_DRIVETRAINS

#include <array>
#include <cstdint>
#include <string>
#include "./orientationprovider.h"

#define DEBUG false 

using namespace std;


class Drivetrain {
    public:
        Drivetrain(uint8_t kbr1, uint8_t kbr2, uint8_t kbl1, uint8_t kbl2, uint8_t kfr1, uint8_t kfr2, uint8_t kfl1, uint8_t kfl2, int deadzone=40);
        void setMaxSpeed(uint8_t maxSpeed); // 0-255
        void setTurnPower(uint8_t turnPower); // 0-255
        void invertMotor(int motor, bool inverted); //0 backright
        /* joyX1 and JoyY are for moving forward, backward, right, and left, joyX2 is for rotation
         */ 
        virtual void updateMotor(int joyX1, int joyX2, int joyY, int joyY2) = 0; // Must be in range (-512,512)
    
    protected:                                        
        // settings                                 
        uint8_t maxSpeed = 255;
        uint8_t turnPower = 255;
        // motor pins
        uint8_t kbr1 = 0;
        uint8_t kbr2 = 0;
        uint8_t kbl1 = 0;
        uint8_t kbl2 = 0;
        uint8_t kfr1 = 0;
        uint8_t kfr2 = 0;
        uint8_t kfl1 = 0;
        uint8_t kfl2 = 0;
        int deadzone = 40;
        array<int,4> invertDir = {1,1,1,1};
};

// robot centric mecanum
class Mecanum: public Drivetrain {
    public:
        Mecanum(uint8_t kbr1, uint8_t kbr2, uint8_t kbl1, uint8_t kbl2, uint8_t kfr1, uint8_t kfr2, uint8_t kfl1, uint8_t kfl2, int deadzone = 40);
        void updateMotor(int joyX1, int joyX2, int joyY, int joyY2) override; // Must be in range (-512,512)
};

// field centric mecanum
class FieldMecanum: public Drivetrain {
    public:
        FieldMecanum(uint8_t kbr1, uint8_t kbr2, uint8_t kbl1, uint8_t kbl2, uint8_t kfr1, uint8_t kfr2, uint8_t kfl1, uint8_t kfl2, OrientationProvider* orientProvider, int deadzone = 40);
        void updateMotor(int joyX1, int joyX2, int joyY, int joyY2) override; // Must be in range (-512,512)
    private:
       OrientationProvider* orientProvider;
       array<array<float,4>,360> LUT;
};




#endif
