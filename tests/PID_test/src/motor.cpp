// Implementation of the class outline in motor.h
#include "Arduino.h"
#include "motor.h"
#include "esp_attr.h"
#include "driver/ledc.h"
#include "stdint.h"

//////////////////////
// Define ISR Routines
//////////////////////

void IRAM_ATTR ChanA(void* arg){
    RPMController* self = static_cast<RPMController*>(arg);
    if (digitalRead(self->kEncoder2) == HIGH) { // Will need to determine in testing which way counts as forwards
      self->realDir = 1;
    } else {
    self->realDir = 0;
    }
    self->edgeCount += 1;
}

void IRAM_ATTR ChanB(void *arg){
    RPMController* self = static_cast<RPMController*>(arg);
    self->edgeCount += 1;
}

///////////////////
// Initialize Class
///////////////////



RPMController::RPMController(uint8_t In1, uint8_t In2, uint8_t Encoder1, uint8_t Encoder2, int motorMaxRPM, int sampleTime, int effectivePPR, int RPMTolerance, bool debug, uint8_t PWMResolution){
    // Define constants
    kIn1 = In1;
    kIn2 = In2;
    kEncoder1 = Encoder1;
    kEncoder2 = Encoder2;
    kMotorMaxRPM = motorMaxRPM;
    kSampleTime = sampleTime;
    kEffectivePPR = effectivePPR;
    kDebug = debug;
    kRPMTolerance = RPMTolerance;
    // If RPMTolerance is 0, generate it 
    if (RPMTolerance == 0){
        // Default at 255 because that is the steps that the motor driver supports
        kRPMTolerance = kMotorMaxRPM / 255; 
    }

    kPWMResolution = PWMResolution;
    kFineResolution = (1UL << kPWMResolution) - 1;

    pinMode(kIn1, OUTPUT);
    pinMode(kIn2, OUTPUT);
  
    pinMode(kEncoder1, INPUT_PULLUP);
    pinMode(kEncoder2, INPUT_PULLUP);
  
    // Attach ISR interrupts
    attachInterruptArg(digitalPinToInterrupt(kEncoder1), ChanA, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(kEncoder2), ChanB, this, CHANGE);

    
    // Starts forwards
    ledcAttach(kIn1,5000,kPWMResolution);
    ledcWrite(kIn1,0);
}


/**
 * Input a negative RPM for one way rotation, positive for another
 */
void RPMController::setRPM(int rpm){
    idealRPM = constrain(rpm,-kMotorMaxRPM,kMotorMaxRPM);

}

float RPMController::readRPM(){
   return realRPM; 
}

void RPMController::setPIDValues(int kP, int kI, int kD){
    kProportional = kP;
    kIntegral = kI;
    kDerivative = kD;
}

void RPMController::setDebug(bool enabled){
    kDebug = enabled;   
}

void RPMController::tick(){
    if (millis() - lastTime >= kSampleTime) {
        static uint8_t count = 0;
        // Logging for Serial Plotter to monitor PID control
        if (kDebug && count %10==0){
            Serial.print(",realRPM:");
            // Flip to negative based on real direction
            Serial.print(realRPM,2);

            Serial.print(",IdealRPM:");
            Serial.print(idealRPM);
            Serial.printf(",Edge:%d\n",edgeCount);
        }

        long currentCount = edgeCount;
        // 60000 comes from 60 * 1000 (seconds in minute * ms in second)
        realRPM = (currentCount * 60000.0) / (kEffectivePPR * kSampleTime);
        
        if (!realDir) realRPM = -realRPM;

        edgeCount = 0;
        


        // PID operations if outside tolerance
        if (abs(realRPM-idealRPM)>kRPMTolerance){
            withinTolerance = false;
            // Handle PID tuning
            long delta = millis()-lastTime;
            float error = realRPM-idealRPM;
            integral += error * delta;
            float derivative = (error-prevError) / delta; 
            prevError = error;

            float tuned = error*kProportional + derivative * kDerivative + integral * kIntegral;
            outValue += map( // May need to be just = sign. will see which is more stable 
                    (int) tuned,
                    -kMotorMaxRPM,kMotorMaxRPM,-kFineResolution,kFineResolution);
        
            outValue = constrain(outValue,-kFineResolution,kFineResolution);
                        
                        

            if (outValue >= 0){
                if (!prevDir){
                    ledcDetach(kIn2);
                    delay(1);
                    ledcAttach(kIn1,5000,kPWMResolution);
                }

                ledcWrite(kIn1, outValue);
                prevDir = true; 
            } else {
                if (!prevDir){
                    ledcDetach(kIn1);
                    delay(1);
                    ledcAttach(kIn2,5000,kPWMResolution);
                }

                ledcWrite(kIn2, -outValue);
                prevDir = false;
            }
        } else if (!withinTolerance && idealRPM == 0) {
            // Turn off motor if within the tolerance of 0 rpm.
            withinTolerance = true;
            ledcDetach(kIn2);
            ledcDetach(kIn1);
            delay(1);
            ledcAttach(kIn1,5000,kPWMResolution);
            prevDir = true;
        }
        count++;
        lastTime = millis();  
    } 
    
}
