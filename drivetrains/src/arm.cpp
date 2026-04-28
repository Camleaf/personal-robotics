

#include "./arm.h"
#include <cstdint>
#include <math.h>

bool inBounds(int val, array<int,2> bounds){
    return bounds[0] <= val && val <= bounds[1];
}

void setupServo(uint8_t pin, uint8_t id, array<Servo,4> &servos){
    Servo srv;
    srv.attach(pin);
    srv.write(0);
    servos[id] = srv;
};

Arm::Arm(uint8_t kbase1, uint8_t kmid1, uint8_t kclrot1, uint8_t kclaw1,
            int baseJointLength, int upperJointLength, int clawLength, // In millimetres
            int baseHeight){
    
    this->kbase = kbase1;
    this->kmid = kmid1;
    this->kclrot = kclrot1;
    this->kclaw = kclaw1;
    this->totalLength = baseJointLength+upperJointLength;
    this->baseJointVector = ((double)baseJointLength) / this->totalLength;
    this->upperJointVector = ((double)upperJointLength)/this->totalLength;
    this->baseHeight = baseHeight;

    this->baseJointRange ={0,120}; //defaults
    this->midJointRange={20,160}; //defaults
    this->clawOC={0,180}; //defaults
    
    setupServo(kbase,kbaseidx,this->servos);
    setupServo(kmid,kmididx,this->servos);
    setupServo(kclrot,kclrotidx,this->servos);
    setupServo(kclaw,kclawidx,this->servos);
    
}


void Arm::setBaseJointRange(int min, int max){
    this->baseJointRange={min,max};
}

void Arm::setUpperJointRange(int min, int max){
    this->midJointRange={min,max};
}

void Arm::setClawOCpoint(int low, int high){
    this->clawOC={low,high};
}


void Arm::setClawGrip(bool closed){
    servos[kclawidx].write(clawOC[closed]);
};

void Arm::setClawRot(int angle){
    // No restrictions on movement here so no bounds check
    servos[kclrotidx].write(angle);
};

double invCosLaw(double a, double b, double c){
    return acos((pow(c,2)+pow(b,2)-pow(a,2))/(2*b*c));
}

void Arm::setClawPoint(int x, int y){
    // Basically just inverse kinematics
    
    double xVector = ((double)x)/totalLength;
    double yVector = ((double)y)/totalLength;
    
    if (yVector < -baseHeight){
        return;
    }

    double baseVectorLength = sqrt(pow(xVector,2) + pow(yVector,2));
    double baseVectorRadAngle = atan2(yVector,xVector)+M_PI;
    Serial.println(baseVectorLength);

    if (baseVectorLength > 1.0) {
        Serial.println("Bad points");
        return;
    }

    // Solve for joint1 and joint2 angle with cosine law
    
    double baseJointRadAngle = invCosLaw(upperJointVector,baseVectorLength,baseJointVector) + baseVectorRadAngle; // I may need to solve for cases because of primary trig ratios may cause this to be off in other quadrants
    double upperJointRadAngle = invCosLaw(baseVectorLength,upperJointVector,baseJointVector);

    int baseJoint = constrain(round(baseJointRadAngle*180/M_PI),0,360);
    int upperJoint = constrain(round(upperJointRadAngle*180/M_PI),0,360);
    
    Serial.printf("Basejoint %d\n", baseJoint);
    Serial.printf("uppjoint %d\n\n", upperJoint);
    servos[kbaseidx].write(baseJoint);
    servos[kmididx].write(upperJoint);

    

}
