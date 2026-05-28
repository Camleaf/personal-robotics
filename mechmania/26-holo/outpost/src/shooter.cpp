#include "./shooter.h"
#include "driver/mcpwm.h"


void setupMCPWM(uint8_t kfly,uint8_t ku){
    // reserved setup for Shooter

    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, kfly);
    //mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, kb2); // if driving second flywheel independantly
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, ku);

    // Reserve mcpwm unit 1 for shooter

    mcpwm_config_t pwm_conf;
    pwm_conf.frequency = 5000;        
    pwm_conf.cmpr_a = 0;          
    pwm_conf.cmpr_b = 0;          
    pwm_conf.counter_mode = MCPWM_UP_COUNTER;
    pwm_conf.duty_mode = MCPWM_DUTY_MODE_0;
    
    // Init mcpwm 
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_conf);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_conf);
}

Shooter::Shooter(uint8_t kfly, uint8_t ksvb, uint8_t ku, uint8_t kbm){
    this->kfly = kfly;
    this->ksvb = ksvb;
    this->ku = ku;
    this->kbm = kbm;

    setupMCPWM(kfly, ku);
}

void Shooter::enabled(bool en){

}


void Shooter::setAngle(int angle){

}


void Shooter::shoot(){

}


void Shooter::feedInterrupt(){

}
