#include <Arduino.h>
#include <cstdint>
#include "./drivetrain.h"
#include "driver/mcpwm.h"

using namespace std;


// Channel defines
// back right
#define kunitbr MCPWM_UNIT_1

#define kunitbl MCPWM_UNIT_1

#define kunitfr MCPWM_UNIT_0

#define kunitfl MCPWM_UNIT_0

#define ktime1 MCPWM_TIMER_0
#define ktime2 MCPWM_TIMER_1

void setMotor(mcpwm_unit_t unit, mcpwm_timer_t timer, double val){
    float duty = (float)abs(val) * 100.0 / 255.0; //MCPWM needs 0 - 100 float
    duty = constrain(duty,0.0,100.0);

    if (val > 0) {
        mcpwm_set_duty(unit, timer, MCPWM_GEN_A, duty);
        mcpwm_set_duty_type(unit, timer, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_signal_low(unit, timer, MCPWM_GEN_B); // Ensure other pin is OFF
    } else if (val < 0) {
        mcpwm_set_signal_low(unit, timer, MCPWM_GEN_A);
        mcpwm_set_duty(unit, timer, MCPWM_GEN_B, duty);
        mcpwm_set_duty_type(unit, timer, MCPWM_GEN_B, MCPWM_DUTY_MODE_0);
    } else {
        mcpwm_set_signal_low(unit, timer, MCPWM_GEN_A);
        mcpwm_set_signal_low(unit, timer, MCPWM_GEN_B);
    }
}

void setupMCPWM(uint8_t kbr1, uint8_t kbr2, uint8_t kbl1, uint8_t kbl2, uint8_t kfr1, uint8_t kfr2, uint8_t kfl1, uint8_t kfl2){
    //Init gpios for mcpwm
    // front left
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, kfl1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, kfl2);
    // front right
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, kfr1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, kfr2);
    //back left
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, kbl1);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, kbl2);
    //back right
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, kbr1);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, kbr2);

    mcpwm_config_t pwm_conf;
    pwm_conf.frequency = 5000;        
    pwm_conf.cmpr_a = 0;          
    pwm_conf.cmpr_b = 0;          
    pwm_conf.counter_mode = MCPWM_UP_COUNTER;
    pwm_conf.duty_mode = MCPWM_DUTY_MODE_0;
    
    // Init mcpwm 
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_conf);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_conf);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_conf);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_conf);
}

Arcade::Arcade(uint8_t kbr1, uint8_t kbr2, uint8_t kbl1, uint8_t kbl2, uint8_t kfr1, uint8_t kfr2, uint8_t kfl1, uint8_t kfl2, int deadzone){
    
    this->maxSpeed = 255;
    this->turnPower = 255;
        
    this->kbr1 = kbr1;
    this->kbr2 = kbr2;
    this->kbl1 = kbl1;
    this->kbl2 = kbl2;
    this->kfr1 = kfr1;
    this->kfr2 = kfr2;
    this->kfl1 = kfl1;
    this->kfl2 = kfl2;
    this->deadzone = deadzone;

    setupMCPWM(kbr1,kbr2,kbl1,kbl2,kfr1,kfr2,kfl1,kfl2);
    
}

void Arcade::setTurnPower(uint8_t turnPower){
    this->turnPower = turnPower;
}
        
void Arcade::setMaxSpeed(uint8_t maxSpeed){
    this->maxSpeed = maxSpeed;
}

void Arcade::updateMotor(int joyX, int joyY){
    
    // Great source for arcade drive https://xiaoxiae.github.io/Robotics-Simplified-Website/drivetrain-control/arcade-drive/

    joyX = map(joyX,-512,512,-turnPower,turnPower);
    joyY = map(joyY,-512,512,-maxSpeed,maxSpeed);

    int turn = constrain(joyX,-maxSpeed,maxSpeed);
    int drive = constrain(joyY,-turnPower,turnPower);

    int maximum = max(abs(drive), abs(turn));
    int total = drive + turn;
    int difference = drive - turn;

    int rightSide;
    int leftSide;

    if (drive >= deadzone){
        if (turn >= deadzone){ // q1
            leftSide = maximum;
            rightSide = difference;
        } else { 
            leftSide = total;
            rightSide = maximum;
        }
    } else if (drive <= -deadzone) {
        if (turn >= deadzone){
            leftSide = total;
            rightSide = -maximum;
        } else { 
            leftSide = -maximum;
            rightSide = difference;
        }
    } else {
        leftSide = 0;
        rightSide = 0;
    }
    
    setMotor(kunitbr,MCPWM_TIMER_1,rightSide); //backright
    setMotor(kunitfr,MCPWM_TIMER_1,rightSide); //frontright
    setMotor(kunitbl,MCPWM_TIMER_0,leftSide); //backleft
    setMotor(kunitfl,MCPWM_TIMER_0,leftSide); //frontleft

}


Mecanum::Mecanum(uint8_t kbr1, uint8_t kbr2, uint8_t kbl1, uint8_t kbl2, uint8_t kfr1, uint8_t kfr2, uint8_t kfl1, uint8_t kfl2, int deadzone){
    
    this->maxSpeed = 255;
    this->turnPower = 255;
        
    this->kbr1 = kbr1;
    this->kbr2 = kbr2;
    this->kbl1 = kbl1;
    this->kbl2 = kbl2;
    this->kfr1 = kfr1;
    this->kfr2 = kfr2;
    this->kfl1 = kfl1;
    this->kfl2 = kfl2;
    this->deadzone = deadzone;
    this->invertDir = {1,1,1,1};
    
    setupMCPWM(kbr1,kbr2,kbl1,kbl2,kfr1,kfr2,kfl1,kfl2);
}



void Mecanum::setTurnPower(uint8_t turnPower){
    this->turnPower = turnPower;
}
        
void Mecanum::setMaxSpeed(uint8_t maxSpeed){
    this->maxSpeed = maxSpeed;
}

void Mecanum::updateMotor(int joyX, int joyX2, int joyY){
    
    // Great source for mecanum drive
    //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

    joyX = map(joyX,-512,512,-maxSpeed, maxSpeed);
    joyX2 = map(joyX2,-512,512,-turnPower,turnPower);
    joyY = map(joyY,-512,512,-maxSpeed,maxSpeed);
    
    int x_drive = constrain(joyX,-maxSpeed,maxSpeed); // horizontal drive
    int y_drive = constrain(joyY,-maxSpeed,maxSpeed); // standard drive
    int turn = constrain(joyX2,-turnPower,turnPower);
    
    if (abs(x_drive) < deadzone) x_drive = 0;
    if (abs(y_drive) < deadzone) y_drive = 0;
    if (abs(turn) < deadzone) turn = 0;
    x_drive *= 1.8;
    
    setMotor(kunitbr,MCPWM_TIMER_1,(y_drive+x_drive-turn)*invertDir[0]); //backright
    setMotor(kunitfr,MCPWM_TIMER_1,(y_drive-x_drive-turn)*invertDir[1]); //frontright
    setMotor(kunitbl,MCPWM_TIMER_0,(y_drive-x_drive+turn)*invertDir[2]); //backleft
    setMotor(kunitfl,MCPWM_TIMER_0,(y_drive+x_drive+turn)*invertDir[3]); //frontleft
    
}

void Mecanum::invertMotor(int motor, bool inverted){
    if (motor < 0 || motor >= 4){
        Serial.println("Tried to invert motor greater than possible");
        return;
    }
    
    if (inverted){
        invertDir[motor] = -1;
        return;
    } 
    invertDir[motor] = 1;
}

