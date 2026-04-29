

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
    
    if (yVector < -baseHeight){
        return;
    }
    double combVectorLength = sqrt(pow(xVector,2) + pow(yVector,2));
    double combVectorRadAngle = atan2(yVector,xVector);
    double combVectorDegAngle = combVectorRadAngle*180/M_PI;
#if ARMDEBUG
    Serial.println("-----NEW------");
    Serial.println(combVectorLength);
    Serial.println(combVectorDegAngle);
#endif
    
    if (combVectorLength > 1.0) {
        Serial.println("Bad points\n\n");
        return;
    } else if (abs(combVectorDegAngle) <= 1 || abs(abs(combVectorDegAngle)-180)<=1){
        Serial.println("Edge case 0 deg");
        if (x < 0 || combVectorDegAngle < 0){
            servos[kbaseidx].write(0);
            servos[kbase2idx].write(180);
            servos[kmididx].write(90);
#if ARMDEBUG
            Serial.printf("Basejoint %d\n", 0);
            Serial.printf("uppjoint %d\n\n", 90);
#endif
        } else {            
            servos[kbaseidx].write(180);
            servos[kbase2idx].write(0);
            servos[kmididx].write(90);
#if ARMDEBUG
             Serial.printf("Basejoint %d\n", 180);
             Serial.printf("uppjoint %d\n\n", 90);
#endif 

        }
        return;
    } else if (abs(abs(combVectorDegAngle)-90)<=1){
        Serial.println("Edge case 90");
        servos[kbaseidx].write(90);
        servos[kbase2idx].write(90);
        servos[kmididx].write(90);
#if ARMDEBUG
        Serial.printf("Basejoint %d\n", 90);
        Serial.printf("uppjoint %d\n\n", 90);
#endif 
        return;
    }
    //  -------------
    //  TODO
    // Solve for bsaejoint since that has most restrictions, then use the angle provided to solve for upperjoint angle. Will give better results.
    // ALSO REMEMBER baseVectorLength and baseJointVector refer to TWO DIFFERENT THINGS FOR SOME REASON
    //--------------------
    
    // Solve for joint1 and joint2 angle with cosine law
    
    double baseJointRadAngle = invCosLaw(upperJointVector,combVectorLength,baseJointVector) + combVectorRadAngle; // I may need to solve for cases because of primary trig ratios may cause this to be off in other quadrants
    
    if (isnan(baseJointRadAngle)){
        Serial.println("bad triangle 1");
        return;
    }
    

    // CAST rule
    // never use Q4
    if (combVectorDegAngle > 180) { // Q3
        baseJointRadAngle = 0; // This is very bad we can not be doing this
    } else if (combVectorDegAngle > 90){ // Q2
        baseJointRadAngle = M_PI-baseJointRadAngle;
    } 
    
    int baseJoint = constrain(round(baseJointRadAngle*180/M_PI),0,180); 
    
    double upperJointRadAngle = cosLaw(baseJointVector,combVectorLength,baseJointRadAngle); // Still needs more math, maybe look at atan2

    if (combVectorDegAngle > 180) { // Q3
    } else if (combVectorDegAngle > 90){ // Q2
    } 

    //double upperJointRadAngle = invCosLaw(baseVectorLength,upperJointVector,baseJointVector); // Old way
     
    if (isnan(upperJointRadAngle == NAN)){
        Serial.println("bad triangle 2");
        return;
    }

    int upperJoint = constrain(round(upperJointRadAngle*180/M_PI+90),0,180);
    
    
#if ARMDEBUG
    Serial.printf("uppjointnoconst %f\n", upperJointRadAngle*180/M_PI);
    Serial.printf("uppjointradnoconst %f\n", upperJointRadAngle);
    Serial.printf("Basejoint %d\n", baseJoint);
    Serial.printf("uppjoint %d\n\n", upperJoint);
#endif

    if (baseJoint > 180 || baseJoint < 0) {Serial.println("huge number error");return;} // should never see this

    servos[kbaseidx].write(baseJoint);
    servos[kbase2idx].write(180-baseJoint);
    servos[kmididx].write(upperJoint);

    

}

void Arm::zero(){
     servos[kmididx].write(0);
     servos[kbaseidx].write(0); // Zero the servo
     servos[kbase2idx].write(180); // Inversed
}
