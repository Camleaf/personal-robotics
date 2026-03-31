#ifndef MOTOR
#define MOTOR

#include "stdint.h"
#include <cstdint>

class RPMController {
    public:
        /** 
         * Initializes an instance of RPMController
         *
         * @param In1 the pin that connects to In1 on a compatible motor driver
         * @param In2 the pin that connects to In2 on a compatible motor driver
         * @param Encoder1 the pin that the first quadrature encoder wire connects to
         * @param Encoder2 the pin that the second quadrature encoder wire connects to
         * @param motorMaxRPM the maximum RPM of the motor
         * @param sampleTime the time between sampling RPM. Too short times can lead to instability
         * @param effectivePPR the effectivePPR for 4x decoding on the encoder
         * @param RPMTolerance the acceptable difference between ideal RPM and real RPM. If set to 0 the class will determine it automatically
         * @param debug If debug messages should be printed. Compatible with Arduino IDE serial plotter
         */
        RPMController(uint8_t In1, uint8_t In2, uint8_t Encoder1, uint8_t Encoder2, int motorMaxRPM, int sampleTime=50, int effectivePPR=44, int RPMTolerance=0, bool debug=false, uint8_t PWMResolution=8);
        
        void setRPM(int rpm); 
        float readRPM();

        bool readDir();

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
        
        
        volatile long edgeCount = 0;
        volatile bool prevDir = true;
        volatile bool realDir = true;

        // Digital input pins which the encoder motors are connected to
        uint8_t kEncoder1;
        uint8_t kEncoder2;

    private:
        // Pins connected to respective control on motor driver
        uint8_t kIn1;
        uint8_t kIn2;

        // The rated RPM of the motor
        int kMotorMaxRPM;
        
        // The sample interval for RPM. Too fast will result in noise, and too slow will result in inaccurate response times.
        int kSampleTime;
        
        // Effective PPR of quadrature encoders on our motors
        int kEffectivePPR;

        // The acceptable amount of RPM off of the desired RPM. 255 Is the number of steps used in Arduino's AnalogOutput. Decrease if this is too low tolerance. Noisier at higher RPMs.
        int kRPMTolerance;

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
        int idealRPM;
        float realRPM;
        
        long lastTime = 0;

        uint32_t outValue = 0;


};

#endif
