

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
    this->baseJointVector = ((double)baseJointLength) / this->totalLength;
    this->upperJointVector = ((double)upperJointLength)/this->totalLength;
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

void Arm::setClawPoint(int x, int y){
    // Basically just inverse kinematics 
    double xVector = ((double)x)/totalLength;
    double yVector = ((double)y)/totalLength;
    
    if (y < -baseHeight){
        return;
    }
    double combVectorLength = sqrt(pow(xVector,2) + pow(yVector,2));
    double combVectorRadAngle = atan2(yVector,xVector); // returns -pi to +pi, -180 to 180
    double combVectorDegAngle = combVectorRadAngle*180/M_PI;
    
#if ARMDEBUG
    Serial.println("-----NEW------");
    Serial.println(combVectorLength);
    Serial.println(combVectorDegAngle);
#endif
    
    if (combVectorLength > 1.0) {
        Serial.println("Bad points\n\n");
        return;
    }
//------------------DIVIDE BY ZERO EDGE CASES------------------
    if (abs(combVectorDegAngle) <= 1 || abs(abs(combVectorDegAngle)-180)<=1){ // divide by 0 edge cases
        Serial.println("Edge case 0 deg");
        if (x < 0 || combVectorDegAngle < 0){
            servos[kbase2idx].write(0);
            servos[kbaseidx].write(180);
            servos[kmididx].write(90);
#if ARMDEBUG
            Serial.printf("Basejoint %d\n", 0);
            Serial.printf("uppjoint %d\n\n", 90);
#endif
        } else {            
            servos[kbase2idx].write(180);
            servos[kbaseidx].write(0);
            servos[kmididx].write(90);
#if ARMDEBUG
             Serial.printf("Basejoint %d\n", 180);
             Serial.printf("uppjoint %d\n\n", 90);
#endif 

        }
        return;
    } else if (abs(abs(combVectorDegAngle)-90)<=1){
        Serial.println("Edge case 90");
        servos[kbase2idx].write(90);
        servos[kbaseidx].write(90);
        servos[kmididx].write(90);
#if ARMDEBUG
        Serial.printf("Basejoint %d\n", 90);
        Serial.printf("uppjoint %d\n\n", 90);
#endif 
        return;
    }

    // Quadrants 3 and 4
    if (combVectorRadAngle < 0){
        if (combVectorRadAngle > -M_PI/2) {Serial.println("Can't put in Q4");return;}
        // Otherwise quadrant 3, and we can put there
        combVectorRadAngle = 270-(-combVectorRadAngle-90);
    }

    double rawRadAngle = invCosLaw(upperJointVector,combVectorLength,baseJointVector); // check this math
    // always two solutions to the inverse kinematics.
    double baseJointRadAngle = rawRadAngle + combVectorRadAngle;
    double invbaseJointRadAngle = combVectorRadAngle - rawRadAngle;

    
    if (combVectorRadAngle<0){ // Physical limitations, arm can't clip though car
        if (baseJointRadAngle > 180 || baseJointRadAngle < 0){
            baseJointRadAngle = NAN;
        }
        if (invbaseJointRadAngle > 180 || baseJointRadAngle < 0){
            invbaseJointRadAngle = NAN;
        }
    }

    double upperJointRadAngle = invCosLaw(combVectorLength,upperJointVector,baseJointVector);
    double invupperJointRadAngle = 180 - upperJointRadAngle;

//------------------Calculate Angles---------------------
    int baseJoint;
    int upperJoint;
    if (!isnan(baseJointRadAngle) && upperJointRadAngle>= 0 && upperJointRadAngle <= 180){
        baseJoint = constrain(round(baseJointRadAngle*180/M_PI),0,180); 
        upperJoint = constrain(round(upperJointRadAngle*180/M_PI+90),0,180);

    } else if (!isnan(invbaseJointRadAngle) && invupperJointRadAngle>=0 && invupperJointRadAngle<=180){
        baseJoint = constrain(round(invbaseJointRadAngle*180/M_PI),0,180); 
        upperJoint = constrain(round(invupperJointRadAngle*180/M_PI+90),0,180);

    } else {
        Serial.println("Can't reach desired position");
        return;
    } 
    
#if ARMDEBUG
    Serial.printf("uppjointnoconst %f\n", upperJointRadAngle*180/M_PI);
    Serial.printf("uppjointradnoconst %f\n", upperJointRadAngle);
    Serial.printf("Basejoint %d\n", baseJoint);
    Serial.printf("uppjoint %d\n\n", upperJoint);
#endif

    if (baseJoint > 180 || baseJoint < 0) {Serial.println("huge number error");return;} // should never see this

    servos[kbaseidx].write(180-baseJoint);
    servos[kbase2idx].write(baseJoint);
    servos[kmididx].write(upperJoint);

    

}

void Arm::zero(){
     servos[kmididx].write(0);
     servos[kbaseidx].write(180); // Zero the servo
     servos[kbase2idx].write(0); // Inversed
}
