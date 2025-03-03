/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_LED_D8_Pin GPIO_PIN_2
#define USER_LED_D8_GPIO_Port GPIOE
#define USER_LED_D7_Pin GPIO_PIN_3
#define USER_LED_D7_GPIO_Port GPIOE
#define USER_LED_D6_Pin GPIO_PIN_4
#define USER_LED_D6_GPIO_Port GPIOE
#define USER_LED_D4_Pin GPIO_PIN_5
#define USER_LED_D4_GPIO_Port GPIOE
#define USER_LED_D2_Pin GPIO_PIN_6
#define USER_LED_D2_GPIO_Port GPIOE
#define L_AS_DRV_CURRENT_Pin GPIO_PIN_4
#define L_AS_DRV_CURRENT_GPIO_Port GPIOF
#define R_AS_DRV_CURRENT_Pin GPIO_PIN_5
#define R_AS_DRV_CURRENT_GPIO_Port GPIOF
#define R_UNO_UART_RX_Pin GPIO_PIN_6
#define R_UNO_UART_RX_GPIO_Port GPIOF
#define R_UNO_UART_TX_Pin GPIO_PIN_7
#define R_UNO_UART_TX_GPIO_Port GPIOF
#define L_BBW_ENC_Pin GPIO_PIN_8
#define L_BBW_ENC_GPIO_Port GPIOF
#define R_BBW_ENC_Pin GPIO_PIN_10
#define R_BBW_ENC_GPIO_Port GPIOF
#define L_LC_ENC_Pin GPIO_PIN_1
#define L_LC_ENC_GPIO_Port GPIOA
#define L_ST_SRV_VPOT_Pin GPIO_PIN_2
#define L_ST_SRV_VPOT_GPIO_Port GPIOA
#define L_ST_SRV_ISENSE_Pin GPIO_PIN_3
#define L_ST_SRV_ISENSE_GPIO_Port GPIOA
#define R_ST_SRV_VPOT_Pin GPIO_PIN_4
#define R_ST_SRV_VPOT_GPIO_Port GPIOA
#define ENC_CLK_Pin GPIO_PIN_5
#define ENC_CLK_GPIO_Port GPIOA
#define ENC_DATA_Pin GPIO_PIN_6
#define ENC_DATA_GPIO_Port GPIOA
#define R_ST_SRV_ISENSE_Pin GPIO_PIN_7
#define R_ST_SRV_ISENSE_GPIO_Port GPIOA
#define L_BBW_SRV_VPOT_Pin GPIO_PIN_4
#define L_BBW_SRV_VPOT_GPIO_Port GPIOC
#define L_BBW_SRV_ISENSE_Pin GPIO_PIN_5
#define L_BBW_SRV_ISENSE_GPIO_Port GPIOC
#define R_BBW_SRV_VPOT_Pin GPIO_PIN_0
#define R_BBW_SRV_VPOT_GPIO_Port GPIOB
#define R_BBW_SRV_ISENSE_Pin GPIO_PIN_1
#define R_BBW_SRV_ISENSE_GPIO_Port GPIOB
#define R_LC_ENC_Pin GPIO_PIN_12
#define R_LC_ENC_GPIO_Port GPIOF
#define LCD_SCL_Pin GPIO_PIN_14
#define LCD_SCL_GPIO_Port GPIOF
#define LCD_SDA_Pin GPIO_PIN_15
#define LCD_SDA_GPIO_Port GPIOF
#define L_AS_ENC_Pin GPIO_PIN_1
#define L_AS_ENC_GPIO_Port GPIOG
#define R_AS_ENC_Pin GPIO_PIN_7
#define R_AS_ENC_GPIO_Port GPIOE
#define MCM_TX_Pin GPIO_PIN_10
#define MCM_TX_GPIO_Port GPIOB
#define MCM_RX_Pin GPIO_PIN_11
#define MCM_RX_GPIO_Port GPIOB
#define JOYSTICK_UP_Pin GPIO_PIN_8
#define JOYSTICK_UP_GPIO_Port GPIOD
#define JOYSTICK_DWN_Pin GPIO_PIN_9
#define JOYSTICK_DWN_GPIO_Port GPIOD
#define JOYSTICK_LFT_Pin GPIO_PIN_10
#define JOYSTICK_LFT_GPIO_Port GPIOD
#define JOYSTICK_RHT_Pin GPIO_PIN_11
#define JOYSTICK_RHT_GPIO_Port GPIOD
#define JOYSTICK_MID_Pin GPIO_PIN_12
#define JOYSTICK_MID_GPIO_Port GPIOD
#define R_AS_DRV_SLP_Pin GPIO_PIN_5
#define R_AS_DRV_SLP_GPIO_Port GPIOG
#define R_AS_DRV_FLT_Pin GPIO_PIN_6
#define R_AS_DRV_FLT_GPIO_Port GPIOG
#define L_AS_DRV_SLP_Pin GPIO_PIN_7
#define L_AS_DRV_SLP_GPIO_Port GPIOG
#define L_AS_DRV_FLT_Pin GPIO_PIN_8
#define L_AS_DRV_FLT_GPIO_Port GPIOG
#define L_AS_DRV_PWMA_Pin GPIO_PIN_6
#define L_AS_DRV_PWMA_GPIO_Port GPIOC
#define L_AS_DRV_PWMB_Pin GPIO_PIN_7
#define L_AS_DRV_PWMB_GPIO_Port GPIOC
#define R_AS_DRV_PWMA_Pin GPIO_PIN_8
#define R_AS_DRV_PWMA_GPIO_Port GPIOC
#define R_AS_DRV_PWMB_Pin GPIO_PIN_9
#define R_AS_DRV_PWMB_GPIO_Port GPIOC
#define R_UNO_CAN_RX_Pin GPIO_PIN_0
#define R_UNO_CAN_RX_GPIO_Port GPIOD
#define R_UNO_CAN_TX_Pin GPIO_PIN_1
#define R_UNO_CAN_TX_GPIO_Port GPIOD
#define R_UNO_CAN_TB_Pin GPIO_PIN_2
#define R_UNO_CAN_TB_GPIO_Port GPIOD
#define FTDI_TX_Pin GPIO_PIN_5
#define FTDI_TX_GPIO_Port GPIOD
#define FTDI_RX_Pin GPIO_PIN_6
#define FTDI_RX_GPIO_Port GPIOD
#define PDS_RX_Pin GPIO_PIN_9
#define PDS_RX_GPIO_Port GPIOG
#define FAULT_LED_Pin GPIO_PIN_12
#define FAULT_LED_GPIO_Port GPIOG
#define MANUAL_LED_Pin GPIO_PIN_13
#define MANUAL_LED_GPIO_Port GPIOG
#define PDS_TX_Pin GPIO_PIN_14
#define PDS_TX_GPIO_Port GPIOG
#define L_UNO_CAN_STB_Pin GPIO_PIN_4
#define L_UNO_CAN_STB_GPIO_Port GPIOB
#define L_UNO_CAN_RX_Pin GPIO_PIN_5
#define L_UNO_CAN_RX_GPIO_Port GPIOB
#define L_UNO_CAN_TX_Pin GPIO_PIN_6
#define L_UNO_CAN_TX_GPIO_Port GPIOB
#define AUTO_LED_Pin GPIO_PIN_7
#define AUTO_LED_GPIO_Port GPIOB
#define L_UNO_UART_RX_Pin GPIO_PIN_0
#define L_UNO_UART_RX_GPIO_Port GPIOE
#define L_UNO_UART_TX_Pin GPIO_PIN_1
#define L_UNO_UART_TX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
