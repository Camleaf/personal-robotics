#include <Arduino.h>
#include <cstdint>
#include "./drivetrain.h"
#include "driver/mcpwm.h"
#include "orientationprovider.h"

using namespace std;


// Channel defines
// back right
#define kunitbr MCPWM_UNIT_1

#define kunitbl MCPWM_UNIT_1

#define kunitfr MCPWM_UNIT_0

#define kunitfl MCPWM_UNIT_0

#define ktime1 MCPWM_TIMER_0
#define ktime2 MCPWM_TIMER_1

void setMotor(mcpwm_unit_t unit, mcpwm_timer_t timer, float val){
    float duty = abs(val) * 100.0f / 255.0f; //MCPWM needs 0 - 100 float
    duty = constrain(duty,0.0f,100.0f);

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



Drivetrain::Drivetrain(uint8_t kbr1, uint8_t kbr2, uint8_t kbl1, uint8_t kbl2, uint8_t kfr1, uint8_t kfr2, uint8_t kfl1, uint8_t kfl2, int deadzone){
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




void Drivetrain::invertMotor(int motor, bool inverted){
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


void Drivetrain::setTurnPower(uint8_t turnPower){
    this->turnPower = turnPower;
}
        
void Drivetrain::setMaxSpeed(uint8_t maxSpeed){
    this->maxSpeed = maxSpeed;
}


/////////////////////////
// Robot-Oriented Mecanum
/////////////////////////

Mecanum::Mecanum(uint8_t kbr1, uint8_t kbr2, uint8_t kbl1, uint8_t kbl2, uint8_t kfr1, uint8_t kfr2, uint8_t kfl1, uint8_t kfl2, int deadzone)
    : Drivetrain(kbr1, kbr2, kbl1, kbl2, kfr1, kfr2, kfl1, kfl2, deadzone){
        
}

void Mecanum::updateMotor(int joyX, int joyX2, int joyY, int joyY2){
    
    // Great source for mecanum drive
    //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
    joyX = map(joyX,-512,512,-maxSpeed, maxSpeed);
    joyX2 = map(joyX2,-512,512,-turnPower,turnPower);
    joyY = map(joyY,-512,512,-maxSpeed,maxSpeed);
    
    float x_drive = constrain(joyX,-maxSpeed,maxSpeed); // horizontal drive
    float y_drive = constrain(joyY,-maxSpeed,maxSpeed); // standard drive
    float turn = constrain(joyX2,-turnPower,turnPower);
    
    if (abs(x_drive) < deadzone) x_drive = 0;
    if (abs(y_drive) < deadzone) y_drive = 0;
    if (abs(turn) < deadzone) turn = 0;
    x_drive *= 1.5f;
    
    setMotor(kunitbr,MCPWM_TIMER_1,(y_drive+x_drive-turn)*invertDir[0]); //backright
    setMotor(kunitfr,MCPWM_TIMER_1,(y_drive-x_drive-turn)*invertDir[1]); //frontright
    setMotor(kunitbl,MCPWM_TIMER_0,(y_drive-x_drive+turn)*invertDir[2]); //backleft
    setMotor(kunitfl,MCPWM_TIMER_0,(y_drive+x_drive+turn)*invertDir[3]); //frontleft
    
}

/////////////////////////
// Field-Oriented Mecanum
/////////////////////////

FieldMecanum::FieldMecanum(uint8_t kbr1, uint8_t kbr2, uint8_t kbl1, uint8_t kbl2, 
        uint8_t kfr1, uint8_t kfr2, uint8_t kfl1, uint8_t kfl2, OrientationProvider* orientProvider, int deadzone)
    : Drivetrain(kbr1, kbr2, kbl1, kbl2, kfr1, kfr2, kfl1, kfl2, deadzone){
    this->orientProvider = orientProvider;
}


void FieldMecanum::updateMotor(int joyX, int joyX2, int joyY, int joyY2){
    
    // Great source for mecanum drive
    //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
    
    if (abs(joyX) < deadzone) joyX = 0;
    if (abs(joyY) < deadzone) joyY = 0;
    if (abs(joyX2) < deadzone) joyX2 = 0;

    joyX = map(joyX,-512,512,-maxSpeed, maxSpeed);
    joyX2 = map(joyX2,-512,512,-turnPower,turnPower);
    joyY = map(joyY,-512,512,-maxSpeed,maxSpeed);

    float botHeading = orientProvider->getRadians();

    int x_drive = constrain(joyX,-maxSpeed,maxSpeed); // horizontal drive
    int y_drive = constrain(joyY,-maxSpeed,maxSpeed); // standard drive
    int turn = constrain(joyX2,-turnPower,turnPower); 
    x_drive *= 1.4f;

    float rot_x_drive = x_drive * cosf(-botHeading) - y_drive * sinf(-botHeading);
    float rot_y_drive = x_drive * sinf(-botHeading) + y_drive * cosf(-botHeading);
    
    setMotor(kunitbr,MCPWM_TIMER_1,(rot_y_drive+rot_x_drive-turn)*invertDir[0]); //backright
    setMotor(kunitfr,MCPWM_TIMER_1,(rot_y_drive-rot_x_drive-turn)*invertDir[1]); //frontright
    setMotor(kunitbl,MCPWM_TIMER_0,(rot_y_drive-rot_x_drive+turn)*invertDir[2]); //backleft
    setMotor(kunitfl,MCPWM_TIMER_0,(rot_y_drive+rot_x_drive+turn)*invertDir[3]); //frontleft
}

