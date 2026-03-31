#ifndef MOTOR
#define MOTOR

#include "stdint.h"
#include <cstdint>

class PositionController {
    public:
        /** 
         * Initializes an instance of PositionController
         *
         * @param In1 the pin that connects to In1 on a compatible motor driver
         * @param In2 the pin that connects to In2 on a compatible motor driver
         * @param Encoder1 the pin that the first quadrature encoder wire connects to
         * @param Encoder2 the pin that the second quadrature encoder wire connects to
         * @param motorMaxRPM the maximum RPM of the motor
         * @param sampleTime the time between updating motor based on readings. Too short times can lead to instability
         * @param effectivePPR the effectivePPR for 4x decoding on the encoder
         * @param PositionTolerance the acceptable difference in degrees between ideal position and real position. If set to 0 the class will determine it automatically
         * @param debug If debug messages should be printed. Compatible with Arduino IDE serial plotter
         */
        PositionController(uint8_t In1, uint8_t In2, uint8_t Encoder1, uint8_t Encoder2, int motorMaxRPM, int sampleTime=50, int effectivePPR=44, int PositionTolerance=0, bool debug=false, uint8_t PWMResolution=8);
        
        void setPosition(int position); 
        float readPosition();

        void setDebug(bool enabled);

        /**
         * @param kP proportional tuned value
         * @param kI integral tuned value
         * @param kD derivative tuned value
         */
        void setPIDValues(int kP, int kI, int kD);
       
        /**
         * Used in mainloop of an Arduino program. Handles moment-to-moment data collection and PID control
         */
        void tick();
        
        

        // Digital input pins which the encoder motors are connected to
        uint8_t kEncoder1;
        uint8_t kEncoder2;
        
        // Pins connected to respective control on motor driver
        uint8_t kIn1;
        uint8_t kIn2;
         
        volatile int lastEncoded = 0;
        volatile int edgePosition = 0; 
    
    private:

        // The rated RPM of the motor
        int kMotorMaxRPM;
        
        // The interval for updating the motor settings. Too fast will result in noise, and too slow will result in inaccurate response times.
        int kSampleTime;
        
        // Effective PPR of quadrature encoders on our motors
        int kEffectivePPR;

        // The acceptable difference in degrees between ideal position and real position.
        int kPositionTolerance;

        // PID constant values
        int kProportional = 0;
        int kIntegral = 0;
        int kDerivative = 0;
        
        // PID relative values
        float integral = 0.0;
        float prevError = 0.0; 

        // Resolution in bits of the outputting PWM signals
        uint8_t kPWMResolution;
        unsigned long long kFineResolution = 0;

        // Debug status
        bool kDebug;
        
        bool withinTolerance = false;
        bool prevDir = true;
        
        int idealPosition;
        int realPosition;
        long lastTime = 0;

        uint32_t outValue = 0;


};

#endif
