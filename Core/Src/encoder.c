/*
 * encoder.c
 *
 *  Created on: Dec 1, 2024
 *      Author: Quynh Nguyen
 */

 #include "encoder.h"
 
 /*
  * @param input 14bit or 16bit encoder reading
  * @retval 0 for no error
  */
 static uint8_t checkbit_error(uint16_t input)
 {
     uint8_t k1_b = (input >> 13) & 1;
     uint8_t k0_b = (input >> 12) & 1;
 
     uint8_t H3 = (input >> 11) & 1;
     uint8_t H2 = (input >> 10) & 1;
     uint8_t H1 = (input >> 9) & 1;
     uint8_t H0 = (input >> 8) & 1;
 
     uint8_t L7 = (input >> 7) & 1;
     uint8_t L6 = (input >> 6) & 1;
     uint8_t L5 = (input >> 5) & 1;
     uint8_t L4 = (input >> 4) & 1;
     uint8_t L3 = (input >> 3) & 1;
     uint8_t L2 = (input >> 2) & 1;
     uint8_t L1 = (input >> 1) & 1;
     uint8_t L0 = (input >> 0) & 1;
 
     // Calculate K1 (Odd)
     uint8_t K1 = !(H3^H1^L7^L5^L3^L1);
 
     // Calculate K0 (Even)
     uint8_t K0 = !(H2^H0^L6^L4^L2^L0);
 
 #if ENCODER_RESOLUTION == AMT23_RES14
     k1_b = (input >> 15) & 1;
     k0_b = (input >> 14) & 1;
 
     uint8_t H5 = (input >> 13) & 1;
     uint8_t H4 = (input >> 12) & 1;
 
     // Calculate K1 (Odd)
     K1 = !(H5^H3^H1^L7^L5^L3^L1);
 
     // Calculate K0 (Even)
     K0 = !(H4^H2^H0^L6^L4^L2^L0);
 #endif
 
    // printmsg("From AMT23: k1 = 0x%X, k0 = 0x%X\n\r",k1_b,k0_b);
    // printmsg("Calculation: K1 = 0x%X, K0 = 0x%X\n\r",K1,K0);

     return (K1 ^ k1_b) | (K0 ^ k0_b); // 0 for no error
 }
 
 /*
  * @param encoder ptr pointing to an Encoder struct
  * @retval uint8_t error code (0x00) for no error
  */
 uint8_t read_encoder(Encoder *encoder)
 {
     uint8_t error = ENCODER_ERR_OK;
     uint8_t bit_count = 0;
     uint8_t current_bit = 0;
     uint16_t encoder_result = 0;
     uint16_t position = 0;
 
     // Write CS LOW
     HAL_GPIO_WritePin(encoder->cs_port, encoder->cs_pin, GPIO_PIN_RESET);
 
     // Write CLK LOW (as per datasheet)
     HAL_GPIO_WritePin(ENC_CLK_GPIO_Port, ENC_CLK_Pin, GPIO_PIN_RESET); // writing to GPIO takes ~400ns
 
     for(bit_count=0; bit_count < ENCODER_RESOLUTION; bit_count++)
     {
         // write CLK HIGH
         HAL_GPIO_WritePin(ENC_CLK_GPIO_Port, ENC_CLK_Pin, GPIO_PIN_SET);
 
         // Read 1 bit
         current_bit = (HAL_GPIO_ReadPin(ENC_DATA_GPIO_Port, ENC_DATA_Pin) == GPIO_PIN_RESET) ? 0 : 1;
 
         // Left shift result
         encoder_result = (encoder_result << 1) | current_bit;
 
         // write CLK LOW
         if(bit_count != ENCODER_RESOLUTION - 1)
         {
             HAL_GPIO_WritePin(ENC_CLK_GPIO_Port, ENC_CLK_Pin, GPIO_PIN_RESET);
         }

     }
 
     // Write CS HIGH
     HAL_GPIO_WritePin(encoder->cs_port, encoder->cs_pin, GPIO_PIN_SET);

     if(!checkbit_error(encoder_result)) // no checkbit error
     {
 #if ENCODER_RESOLUTION == AMT23_RES12
         position = encoder_result & 0xfff;
 #else
         position = encoder_result & 0x3fff;
 #endif
         encoder->position = position;
     }
     else // error
     {
         error = ENCODER_ERR_CHECKBIT;
     }

     return error;
 }
 
