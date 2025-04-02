/**
 * servo.h 
 */


 #ifndef SERVO_H_
 #define SERVO_H_

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <stdarg.h>
 #include <stdint.h>
 #include <math.h>
 
 #include "main.h"
 
 #define SERVO_STEP_DELAY_MS 10 // time for servo to reach desired position, cannot be 0

 typedef struct{
    TIM_HandleTypeDef* timer;         // timer handler for pwm
    uint32_t           pwm_channel;   // timer's pwm channel (check .ioc file)
    
    ADC_HandleTypeDef* vpot_adc;      // ADC handler for servo internal potentiometer
    uint32_t           vpot_channel;  // check .ioc file
    uint32_t           vpot;          // internal potentiometer voltage (12 bit ADC)
    
    ADC_HandleTypeDef* isense_adc;    // ADC handler for servo current sense
    uint32_t           isense_channel;// check .ioc file
    uint32_t           isense;        // current draw (12 bit ADC)
 } Servo;
 
 void run_servo(Servo* servo, uint8_t dutycycle);
 void test_servo(TIM_HandleTypeDef* timer); // for testing only

 #endif /* INC_SERVO_H_ */
