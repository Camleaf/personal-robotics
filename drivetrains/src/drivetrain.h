#ifndef CAMLEAF_DRIVETRAINS
#define CAMLEAF_DRIVETRAINS

#include <array>
#include <cstdint>
#include <string>

#define DEBUG true

using namespace std;

class Arcade {
    public:
        Arcade(uint8_t kbr1, uint8_t kbr2, uint8_t kbl1, uint8_t kbl2, uint8_t kfr1, uint8_t kfr2, uint8_t kfl1, uint8_t kfl2, int deadzone=20);
        void setMaxSpeed(uint8_t maxSpeed); // 0-255
        void setTurnPower(uint8_t turnPower); // 0-255

        void invertMotor(int motor, bool inverted); //0 backright
        /* JoyY is for moving forward and back, JoyX is for rotation
         */ 

        void updateMotor(int joyX, int joyY, bool slowmode); // Must be in range (-512,512)
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
        int deadzone = 20;
        array<int,4> invertDir = {1,1,1,1};
};


class Mecanum {
    public:
        Mecanum(uint8_t kbr1, uint8_t kbr2, uint8_t kbl1, uint8_t kbl2, uint8_t kfr1, uint8_t kfr2, uint8_t kfl1, uint8_t kfl2, int deadzone=40);
        void setMaxSpeed(uint8_t maxSpeed); // 0-255
        void setTurnPower(uint8_t turnPower); // 0-255
        /* joyX1 and JoyY are for moving forward, backward, right, and left, joyX2 is for rotation
         */ 
        void updateMotor(int joyX1, int joyX2, int joyY); // Must be in range (-512,512)
        void invertMotor(int motor, bool inverted); //0 backright
                                                    //1 frontright
    private:                                        //2 backleft
        // settings                                  3 frontleft
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



#endif
