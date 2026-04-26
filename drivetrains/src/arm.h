#ifndef CAMLEAF_ARM
#define CAMLEAF_ARM

#include <array>
#include <ESP32Servo.h>

using namespace std;


class Arm {
    public:
        Arm(uint8_t kbase1, uint8_t kmid1, uint8_t krot1, uint8_t kclaw1,
            int baseJointLength, int upperJointLength, int clawLength, // In millimetres
            int baseHeight
        );
        
        void setBaseJointRange(int min, int max);
        void setMidJointRange(int min, int max);

        void openClaw();

        void gripClaw();

        void clawAngle(int angle);

        void setClawPoint(int x, int y); // Where 0,0 is the base of the claw. in millimetres
        
    array<Servo,4> servos;
        
    uint8_t kbase1 = 0;
    uint8_t kmid = 0;
    uint8_t kclrot = 0;
    uint8_t kclaw = 0;

    int baseJointLength = 10;
    int upperJointLength = 10;
    int clawLength = 10;
    int baseHeight = 0;
    array<int,2> baseJointRange = {0,135};
    array<int,2> midJointRange = {20,160};

};


#endif
