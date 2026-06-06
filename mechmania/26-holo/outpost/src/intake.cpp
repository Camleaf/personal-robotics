#include "./mechanisms.h"
#include "driver/mcpwm.h"
#include <cstdint>

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

void setupMCPWM_in(uint8_t ku){
    // reserved setup for Intake


    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, ku);

    // Reserve mcpwm unit 0 for intake

    mcpwm_config_t pwm_conf;
    pwm_conf.frequency = 5000;        
    pwm_conf.cmpr_a = 0;          
    pwm_conf.cmpr_b = 0;          
    pwm_conf.counter_mode = MCPWM_UP_COUNTER;
    pwm_conf.duty_mode = MCPWM_DUTY_MODE_0;
    
    // Init mcpwm 
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_conf);
}



Intake::Intake(uint8_t ku){
    this->ku = ku;
    
    setupMCPWM_in(ku);
}

void Intake::setSpeed(uint8_t speed){
     setMotor(MCPWM_UNIT_0,MCPWM_TIMER_0,speed);     
}

void Intake::off(){
    setSpeed(0);
}
