 #include "servo.h"
 
 /**
  * @brief read servo's internal potentiometer
  */
 void read_servo_potentiometer(Servo* servo)
 {
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = servo->vpot_channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    // sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // already set in ADC init

    // Configure the ADC to the desired channel
    if (HAL_ADC_ConfigChannel(servo->vpot_adc, &sConfig) != HAL_OK) {
        printmsg("ADC VPOT ERROR\r\n");  // Error
    }

    // Start conversion
    HAL_ADC_Start(servo->vpot_adc);

    // Wait for conversion complete
    if (HAL_ADC_PollForConversion(servo->vpot_adc, HAL_MAX_DELAY) != HAL_OK) {
    	printmsg("ADC VPOT ERROR\r\n");  // Error
    }

    // Read ADC value
    servo->vpot = HAL_ADC_GetValue(servo->vpot_adc);

    // Stop ADC
    HAL_ADC_Stop(servo->vpot_adc);
 }

 /**
  * @brief curve fit to map adc 12bit val to dutycycle
  * TODO: map it to servo current position instead
  */
 static uint8_t map_adc_to_duty(uint32_t adc_value)
 {

	printmsg("vpot = %ld\r\n",adc_value);
	float p1 = 40.6846f, p2 = 493.3433f;
    float duty = (adc_value + p2) / p1;

    if (duty < 0) duty = 0;
    if (duty > 100) duty = 100;

    return (uint8_t)roundf(duty);
 }
 
 /**
 * @brief Interrupt notification when ADC-DMA conv. is finished
 */
 void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
 {

 }

 /**
  * @brief read all servos' current
  */
 static void read_servo_current(Servo* servo)
 {
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = servo->isense_channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    // sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // already set in ADC init

    HAL_ADC_ConfigChannel(servo->isense_adc, &sConfig);  // Set the desired channel

    HAL_ADC_Start_DMA(servo->isense_adc,&(servo->isense), 1);
 }

 /**
  * @brief Output 5V PWM signal to a specific servo
  */
 void run_servo(Servo* servo, uint8_t dutycycle)
 {
    // Start PWM
    HAL_TIM_PWM_Start(servo->timer, servo->pwm_channel);
    
    read_servo_potentiometer(servo);
    uint8_t current_duty = map_adc_to_duty(servo->vpot);

    printmsg("current_duty = %d\r\n",current_duty);

    uint16_t current_ccr = current_duty*(__HAL_TIM_GET_AUTORELOAD(servo->timer))/100;

//    __HAL_TIM_SET_COMPARE(servo->timer, servo->pwm_channel, current_ccr);

    //TODO: Add dutycycle boundary (20% <= duty <= 97%)
//    uint16_t current_ccr = __HAL_TIM_GET_COMPARE(servo->timer, servo->pwm_channel); // doesnt work

//    uint16_t target_ccr = dutycycle*(__HAL_TIM_GET_AUTORELOAD(servo->timer))/100;

    // Start current sense ADC
    read_servo_current(servo);

//    while (current_ccr != target_ccr) {
//    	printmsg("current ccr = %d\r\n, target ccr = %d\r\n", current_ccr, target_ccr);
//        current_ccr += (current_ccr < target_ccr) ? 1 : -1;
//
//        __HAL_TIM_SET_COMPARE(servo->timer, servo->pwm_channel, current_ccr);
//        HAL_Delay(SERVO_STEP_DELAY_MS);
//    }
    while(current_duty != dutycycle)
    {
    	__HAL_TIM_SET_COMPARE(servo->timer, servo->pwm_channel, current_ccr);
    	current_ccr = current_duty*(__HAL_TIM_GET_AUTORELOAD(servo->timer))/100;
    	current_duty += (current_duty < dutycycle) ? 1 : -1;
    	HAL_Delay(SERVO_STEP_DELAY_MS);
    }
    
    HAL_ADC_Stop_DMA(servo->isense_adc);
    
    // Stop PWM here or in a separate function? 
    // PWM need to be ON to hold servo at that position
    // HAL_TIM_PWM_Stop(servo->timer, servo->pwm_channel);
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
    	HAL_Delay(SERVO_STEP_DELAY_MS);
    }

    // Decrease duty cycle
    for (dutycycle = 100; dutycycle > 0; dutycycle--)
    {
    	ccr_val = dutycycle*(__HAL_TIM_GET_AUTORELOAD(timer))/100;
    	__HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_1,ccr_val);
        __HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_2,ccr_val);
        __HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_3,ccr_val);
        __HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_4,ccr_val);
    	HAL_Delay(SERVO_STEP_DELAY_MS);
    }

    HAL_TIM_PWM_Stop(timer,TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(timer,TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(timer,TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(timer,TIM_CHANNEL_4);
 }
