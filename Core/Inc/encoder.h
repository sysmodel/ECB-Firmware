/*
 * encoder.h
 *
 *  Created on: Dec 1, 2024
 *      Author: Quynh Nguyen
 */

 #ifndef ENCODER_H_
 #define ENCODER_H_
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
 #include "main.h"
 
 // Encoder Resolution Setting
 #define AMT23_RES12 14
 #define AMT23_RES14 16
 #define ENCODER_RESOLUTION AMT23_RES12

 // Encoder Error Code
 #define ENCODER_ERR_OK 0x00  // no error
 #define ENCODER_ERR_CHECKBIT 0X90  // checkbit error

 
 // Encoder struct
 typedef struct{
    GPIO_TypeDef* cs_port; // Chip select port
    uint16_t cs_pin;       // Chip select pin (both can be found in main.h)
    uint16_t position;     // Store position data of latest read
 }Encoder;

 
 uint8_t read_encoder(Encoder *encoder);

 #endif /* INC_ENCODER_H_ */
 
