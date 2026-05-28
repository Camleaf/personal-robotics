#include "./intake.h"
#include "driver/mcpwm.h"
#include <cstdint>


void setupMCPWM(uint8_t kb1,uint8_t kb2, uint8_t ku1, uint8_t ku2){
    // reserved setup for Intake

    // front left
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, kb1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, kb2);
    // front right
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, ku1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, ku2);

    // Reserve mcpwm unit 0 for intake

    mcpwm_config_t pwm_conf;
    pwm_conf.frequency = 5000;        
    pwm_conf.cmpr_a = 0;          
    pwm_conf.cmpr_b = 0;          
    pwm_conf.counter_mode = MCPWM_UP_COUNTER;
    pwm_conf.duty_mode = MCPWM_DUTY_MODE_0;
    
    // Init mcpwm 
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_conf);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_conf);
}



Intake::Intake(uint8_t kb1, uint8_t kb2, uint8_t ku1, uint8_t ku2, uint8_t kbm){
    this->kb1 = kb1;
    this->kb2 = kb2;
    this->ku1 = ku1;
    this->ku2 = ku2;
    this->kbm = kbm;
    
    setupMCPWM(kb1,kb2,ku1,ku2);
}

void Intake::enabled(bool en){
    
}

void Intake::reversed(bool en){
    
} 
void Intake::stageInterrupt(){
    
}
