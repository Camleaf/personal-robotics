
// Implementation of the class outline in motor.h
#include "Arduino.h"
#include "motor.h"
#include "esp_attr.h"
#include "driver/ledc.h"
#include "stdint.h"

PositionController::PositionController(uint8_t In1, uint8_t In2, uint8_t Encoder1, uint8_t Encoder2, int motorMaxRPM, int sampleTime, int effectivePPR, int PositionTolerance, bool debug, uint8_t PWMResolution){
    // Define constants
    kIn1 = In1;
    kIn2 = In2;
    kEncoder1 = Encoder1;
    kEncoder2 = Encoder2;
    kMotorMaxRPM = motorMaxRPM;
    kSampleTime = sampleTime;
    kEffectivePPR = effectivePPR;
    kDebug = debug;
    kPositionTolerance = ceil(PositionTolerance / ((float)effectivePPR));
    // If PositionTolerance is 0, generate it 
    if (PositionTolerance == 0){
        kPositionTolerance = 1; // about 8 degrees of tolerance 
    }

    kPWMResolution = PWMResolution;
    kFineResolution = (1UL << kPWMResolution) - 1;

    pinMode(kIn1, OUTPUT);
    pinMode(kIn2, OUTPUT);
  
    pinMode(kEncoder1, INPUT_PULLUP);
    pinMode(kEncoder2, INPUT_PULLUP);
  
    // Attach ISR interrupts
    attachInterruptArg(kEncoder1, *PositionController::Channel, this, CHANGE);
    attachInterruptArg(kEncoder2, *PositionController::Channel, this, CHANGE);

    
    // Starts forwards
    ledcAttach(kIn1,5000,kPWMResolution);
    ledcWrite(kIn1,0);
}


/**
 * Input a negative position for one way rotation, positive for another
 */
void PositionController::setPosition(int position){
     idealPosition = map(position,-360,360,-44,44);
}

float PositionController::readPosition(){
   return realPosition; 
}

void PositionController::setPIDValues(int kP, int kI, int kD){
    kProportional = kP;
    kIntegral = kI;
    kDerivative = kD;
}

void PositionController::setDebug(bool enabled){
    kDebug = enabled;   
}



void IRAM_ATTR PositionController::Channel(void* arg){
    PositionController* self = static_cast<PositionController*>(arg);
    int a = digitalRead(self->kEncoder1);
    int b = digitalRead(self->kEncoder2);
    int encoded = (a << 1) | b;
    int sum = (self->lastEncoded << 2) | encoded;

    if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) {
        self->edgePosition++; // Clockwise
    } else if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) {
        self->edgePosition--; // Counter-clockwise
    }

    self->lastEncoded = encoded;
}



void PositionController::tick(){
     
    if (millis() - lastTime >= kSampleTime) {
        // Logging for Serial Plotter to monitor PID control
        if (kDebug){
            Serial.print(",realPosition:");
            // Flip to negative based on real direction
            Serial.print(realPosition,2);

            Serial.print(",IdealPosition:");
            Serial.print(idealPosition);
            Serial.print("\n");
        }

        realPosition = edgePosition;     
        

        // PID operations if outside tolerance
        if (abs(realPosition-idealPosition)>kPositionTolerance){
            withinTolerance = false;
            // Handle PID tuning
            long delta = millis()-lastTime;
            float error = realPosition-idealPosition;
            integral += error * delta;
            float derivative = (error-prevError) / ((float)delta); 
            prevError = error;

            float tuned = error*kProportional + derivative * kDerivative + integral * kIntegral;
            outValue = map(
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
        } else if (!withinTolerance) {
            // Turn off the motor if within tolerance of position
            withinTolerance = true;
            ledcDetach(kIn2);
            ledcDetach(kIn1);
            delay(1);
            ledcAttach(kIn1,5000,kPWMResolution);
            prevDir = true;
        }

        lastTime = millis();  
    } 
}
