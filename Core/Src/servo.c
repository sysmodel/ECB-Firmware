 #include "servo.h"

 void run_servo(Servo* servo, uint8_t dutycycle)
 {
    HAL_TIM_PWM_Start(servo->timer, servo->channel);

    // TODO: add overflow, rounding error check
    uint16_t ccr_val = dutycycle*(__HAL_TIM_GET_AUTORELOAD(servo->timer))/100;
    // printmsg("ccr = %d\r\n",ccr_val);

    __HAL_TIM_SET_COMPARE(servo->timer,servo->channel,ccr_val);
    
    // TODO: maybe change it to timer (hardware) delay instead of HAL?
//     HAL_Delay(SERVO_DELAY_MS);
//
//     HAL_TIM_PWM_Stop(servo->timer, servo->channel); // stop or continuous until stop??
 }

 /**
  * @brief output 5V PWM signals to all 4 servo ports
  *        that goes from 0% to 100% duty cycle
  */
 void test_servo(TIM_HandleTypeDef* timer)
 {
    uint8_t dutycycle = 0;
    uint16_t ccr_val = 0;

    // Start output PWM to channel 1->4 of timer 1
    HAL_TIM_PWM_Start(timer,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(timer,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(timer,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(timer,TIM_CHANNEL_4);

    // Increase duty cycle
    for (dutycycle = 0; dutycycle <= 100; dutycycle++)
    {
    	ccr_val = dutycycle*(__HAL_TIM_GET_AUTORELOAD(timer))/100;
    	__HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_1,ccr_val);
        __HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_2,ccr_val);
        __HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_3,ccr_val);
        __HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_4,ccr_val);
    	HAL_Delay(SERVO_DELAY_MS);
    }

    // Decrease duty cycle
    for (dutycycle = 100; dutycycle > 0; dutycycle--)
    {
    	ccr_val = dutycycle*(__HAL_TIM_GET_AUTORELOAD(timer))/100;
    	__HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_1,ccr_val);
        __HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_2,ccr_val);
        __HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_3,ccr_val);
        __HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_4,ccr_val);
    	HAL_Delay(SERVO_DELAY_MS);
    }

    HAL_TIM_PWM_Stop(timer,TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(timer,TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(timer,TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(timer,TIM_CHANNEL_4);
 }
