#ifndef CAMLEAF_ARM
#define CAMLEAF_ARM

#include <array>
#include <ESP32Servo.h>

using namespace std;

#define ARMDEBUG false 

#define kbaseidx 0
#define kmididx 1
#define kclrotidx 2
#define kclawidx 3
#define kbase2idx 4

class Arm {
    public:
        Arm(uint8_t kbase1, uint8_t kbase2, uint8_t kmid1, uint8_t kclrot1, uint8_t kclaw1,
            int baseJointLength, int upperJointLength, int clawLength, // In millimetres
            int baseHeight
        );
        
        void setBaseJointRange(int min, int max);
        void setUpperJointRange(int min, int max);
        void setClawOCpoint(int low, int high); // Degrees of closed and open points for claw servo

        void setClawGrip(bool closed);
        void setClawWrist(bool down);

        void setClawRot(int angle);

        bool setClawPoint(int x, int y); // Where 0,0 is the base of the claw. and x,y is the desired claw point in mm
        bool setServoRots(int base, int mid);
        bool setBaseRot(int base);
        bool setMidRot(int mid);


        void begin();
        void neutral();
        void zero();
        void place();
        void stored();
        void pickup();
        void score();
        void ringpickup();

    array<Servo,5> servos;
        
    uint8_t kbase = 0;
    uint8_t kmid = 0;
    uint8_t kclrot = 0;
    uint8_t kclaw = 0;
    uint8_t kbase2 = 0;

    double baseJointVector = .5;
    double upperJointVector = .5;
    double totalLength = 10;
    int baseHeight = 0;
    array<int,2> baseJointRange = {0,135};
    array<int,2> midJointRange = {20,160};
    array<int,2> clawOC = {0,180}; // open/close rotations
};


#endif
