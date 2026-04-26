

#include "./arm.h"
#include <cstdint>

#define kbaseidx 0
#define kmididx 1
#define kclrotidx 2
#define kclawidx 3


void setupServo(uint8_t pin, uint8_t id, array<Servo,4> &servos){
    Servo srv;
    srv.attach(pin);
    srv.write(0);
    servos[id] = srv;
};

Arm::Arm(uint8_t kbase1, uint8_t kmid, uint8_t kclrot, uint8_t kclaw){
    
    this->kbase1 = kbase1;
    this->kmid = kmid;
    this->kclrot = kclrot;
    this->kclaw = kclaw;
    
    setupServo(kbase1,kbaseidx,this->servos);
    setupServo(kmid,kmididx,this->servos);
    setupServo(kclrot,kclrotidx,this->servos);
    setupServo(kclaw,kclawidx,this->servos);
    
}



