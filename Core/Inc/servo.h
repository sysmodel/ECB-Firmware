/**
 * servo.h 
 */


 #ifndef SERVO_H_
 #define SERVO_H_

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <stdarg.h>
 
 #include "main.h"
 
 #define SERVO_DELAY_MS 10 // time for servo to reach desired position, cannot be 0

 typedef struct{
    TIM_HandleTypeDef* timer;     // timer handler
    uint32_t           channel;   // timer's pwm channel (check .ioc file)
    uint16_t           v_pot;     // internal potentiometer voltage
    uint16_t           i_sense;   // servo's current draw
 } Servo;

 void run_servo(Servo* servo, uint8_t dutycycle);
 void test_servo(TIM_HandleTypeDef* timer); // for testing only

 #endif /* INC_SERVO_H_ */
