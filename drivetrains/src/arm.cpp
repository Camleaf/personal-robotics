

#include "./arm.h"
#include <cstdint>
#include <math.h>

bool inBounds(int val, array<int,2> bounds){
    return bounds[0] <= val && val <= bounds[1];
}

void setupServo(uint8_t pin, uint8_t id, array<Servo,5> &servos){
    servos[id].setPeriodHertz(50);  
    servos[id].attach(pin);
};

Arm::Arm(uint8_t kbase1, uint8_t kbase2, uint8_t kmid1, uint8_t kclrot1, uint8_t kclaw1, //kbase2 is reversed
            int baseJointLength, int upperJointLength, int clawLength, // In millimetres
            int baseHeight){
    
    this->kbase = kbase1;
    this->kbase2 = kbase2;
    this->kmid = kmid1;
    this->kclrot = kclrot1;
    this->kclaw = kclaw1;
    this->totalLength = baseJointLength+upperJointLength;
    this->baseJointVector = ((double)baseJointLength) / (baseJointLength+upperJointLength);
    this->upperJointVector = ((double)upperJointLength) / (baseJointLength+upperJointLength);
    
    this->baseHeight = baseHeight;

    this->baseJointRange ={0,120}; //defaults
    this->midJointRange={20,160}; //defaults
    this->clawOC={0,180}; //defaults 
}

void Arm::begin(){
    
    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
    setupServo(kbase,kbaseidx,this->servos);
    setupServo(kmid,kmididx,this->servos);
    //setupServo(kclrot,kclrotidx,this->servos);
    //setupServo(kclaw,kclawidx,this->servos);
    setupServo(kbase2, kbase2idx,this->servos);
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

double cosLaw(double b, double c, double A){
    return pow(b,2) + pow(c,2) + 2*b*c*cos(A);
}

// returns false on fail
bool Arm::setClawPoint(int x, int y){
    // Basically just inverse kinematics

    Serial.println("-----NEW------");
    double xVector = ((double)x)/totalLength;
    double yVector = ((double)y)/totalLength;
    
    if (y < -baseHeight){
        Serial.println("below base limit");
       // return;
    }
    double combVectorLength = sqrt(pow(xVector,2) + pow(yVector,2));
    double combVectorRadAngle = atan2(yVector,xVector); // returns -pi to +pi, -180 to 180
    Serial.printf("CombVectorRadAngle: %f\n",combVectorRadAngle);
    if (combVectorRadAngle > -M_PI/2 && combVectorRadAngle < 0) {Serial.println("Can't put in Q4");return false;}
    if (combVectorRadAngle < 0) combVectorRadAngle += 2*M_PI;
    double combVectorDegAngle = combVectorRadAngle*180/M_PI;
        
    if (combVectorLength > 1.0) {
        Serial.println("Bad points\n\n");
        return false;
    }
//------------------DIVIDE BY ZERO EDGE CASES------------------
/*
    if (abs(combVectorDegAngle) <= 0.5 || abs(abs(combVectorDegAngle)-180)<=0.5){ // divide by 0 edge cases
        Serial.println("Edge case 0 deg");
        if (x < 0 || combVectorDegAngle < 0){
            servos[kbase2idx].write(180);
            servos[kbaseidx].write(0);
            servos[kmididx].write(90);
#if ARMDEBUG
            Serial.printf("Basejoint %d\n", 0);
            Serial.printf("uppjoint %d\n\n", 90);
#endif
        } else {            
            servos[kbase2idx].write(0);
            servos[kbaseidx].write(180);
            servos[kmididx].write(90);
#if ARMDEBUG
             Serial.printf("Basejoint %d\n", 180);
             Serial.printf("uppjoint %d\n\n", 90);
#endif 

        }
        return true;
    } else if (abs(abs(combVectorDegAngle)-90)<=0.5){
        Serial.println("Edge case 90");
        servos[kbase2idx].write(90);
        servos[kbaseidx].write(90);
        servos[kmididx].write(90);
#if ARMDEBUG
        Serial.printf("Basejoint %d\n", 90);
        Serial.printf("uppjoint %d\n\n", 90);
#endif 
        return true;
    }
*/

#if ARMDEBUG
    Serial.println(combVectorLength);
    Serial.println(combVectorDegAngle);
#endif


    double rawRadAngle = invCosLaw(upperJointVector,combVectorLength,baseJointVector); // check this math
    // always two solutions to the inverse kinematics.
    double baseJointRadAngle = rawRadAngle + combVectorRadAngle;
    double invbaseJointRadAngle = combVectorRadAngle - rawRadAngle;
    
    


    if (baseJointRadAngle > M_PI || baseJointRadAngle < 0){
        baseJointRadAngle = NAN;
    }
    if (invbaseJointRadAngle > M_PI || invbaseJointRadAngle < 0){
        invbaseJointRadAngle = NAN;
    }
#if ARMDEBUG
    Serial.printf("RawAngle %f\n", rawRadAngle*180/M_PI);
    Serial.printf("BasejointAngle %f\n", baseJointRadAngle*180/M_PI);
    Serial.printf("invBaseJointAngle %f\n", invbaseJointRadAngle*180/M_PI);
#endif

    double rawUpperJointRad = invCosLaw(combVectorLength,upperJointVector,baseJointVector);
    double upperJointRadAngle = rawUpperJointRad-(M_PI/2);
    double invupperJointRadAngle = (M_PI*3/2)-rawUpperJointRad;

#if ARMDEBUG
    Serial.printf("RawUpperAngle %f\n", rawUpperJointRad*180/M_PI);
    Serial.printf("upperjointAngle %f\n", upperJointRadAngle*180/M_PI);
    Serial.printf("invupperJointAngle %f\n", invupperJointRadAngle*180/M_PI);
#endif
//------------------Calculate Angles---------------------
    int baseJoint;
    int upperJoint;
    if (!isnan(baseJointRadAngle) && upperJointRadAngle>= 0 && upperJointRadAngle <= M_PI){
        baseJoint = constrain(round(baseJointRadAngle*180/M_PI),0,180); 
        upperJoint = constrain(round(upperJointRadAngle*180/M_PI),0,180);

    } else if (!isnan(invbaseJointRadAngle) && invupperJointRadAngle>=0 && invupperJointRadAngle<=M_PI){
        baseJoint = constrain(round(invbaseJointRadAngle*180/M_PI),0,180); 
        upperJoint = constrain(round(invupperJointRadAngle*180/M_PI),0,180);

    } else {
        Serial.println("Can't reach desired position");
        return false;
    } 
    
#if ARMDEBUG
    Serial.printf("Basejoint %d\n", baseJoint);
    Serial.printf("uppjoint %d\n\n", upperJoint);
#endif

    if (baseJoint > 180 || baseJoint < 0) {Serial.println("huge number error");return false;} // should never see this

    servos[kbaseidx].write(180-baseJoint);
    servos[kbase2idx].write(baseJoint);
    servos[kmididx].write(upperJoint);

    return true;

}

void Arm::neutral(){
    servos[kmididx].write(0);
    servos[kbaseidx].write(0);
    servos[kbase2idx].write(180);
}

void Arm::zero(){ // Dont use this when the bot is on the robot
     servos[kmididx].write(90);
     servos[kbaseidx].write(180); // Zero the servo
     servos[kbase2idx].write(0); // So that stuff doesnt get damaged
}

bool Arm::setServoRots(int base, int mid){
    return setBaseRot(base)&&setMidRot(mid);
}

bool Arm::setBaseRot(int base){
    
    if (base > 180 || base < 0) return false;

    servos[kbaseidx].write(constrain(180-base,0,180));
    servos[kbase2idx].write(constrain(base,0,180));
    return true;
}

bool Arm::setMidRot(int mid){
    if (mid < 0 || mid > 180){
        return false;
    }
    servos[kmididx].write(constrain(mid,0,180));
    return true;

}
