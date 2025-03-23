/**
 *******************************************************************************
 * @file    SOLOMotorControllers.h
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the base functions prototypes for the Solo Drivers
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.4.3
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 * ****************************************************************************
 * 2025/03/25: Convert to C code to be compatible with STM32F7 (QN)
 *******************************************************************************
 */

#ifndef SOLO_MOTOR_CONTROLLERS_H
#define SOLO_MOTOR_CONTROLLERS_H

#include <stdbool.h>

/* Error enumeration */
typedef enum {
    NO_ERROR_DETECTED = 0,						/*!< no error detected */
    GENERAL_ERROR = 1,							/*!< general error */
    NO_PROCESSED_COMMAND = 2,					/*!< command is not valid */
    OUT_OF_RANGE_SETTING = 3,					/*!< setting is out of range */
    PACKET_FAILURE_TRIAL_ATTEMPTS_OVERFLOW = 4, /*!< trial attempt overflow */
    RECEIVE_TIMEOUT_ERROR = 5,					/*!< receive time error */
    ABORT_OBJECT = 6,							/*!< abort object */
    ABORT_VALUE = 7,							/*!< abort value */
    MCP2515_TRANSMIT_ARBITRATION_LOST = 8,		/*!< MCP2515 transmit arbitration lost */
    MCP2515_TRANSMIT_ERROR = 9,					/*!< MCP2515 transmit error */
    OBJECT_NOT_INITIALIZE = 10,					/*!< Kvaser object not initialize */
    CAN_EMPTY_BUFFER = 11,						/*!< Kvaser buffer have no data for the defined COBID */
    PDO_PARAMETER_ID_OUT_OF_RANGE = 12,			/*!< PDO configuration id out of range */
    PDO_SYNC_OUT_OF_RANGE = 13,					/*!< PDO configuration sync out of range */
    PDO_MISSING_COB_ID = 14,					/*!< PDO no specific CobId for the specified pdo*/
    PDO_RTR_COMMAND_NOT_ALLOWED = 15,			/*!< PDO RTR specific command not allowed*/
} Error;

/* Command Mode */
typedef enum {
    ANALOGUE = 0,						  /*!< Analogue Mode */
    DIGITAL = 1,						  /*!< Digital Mode */
    ANALOGUE_WITH_DIGITAL_SPEED_GAIN = 2, /*!<Analogue Mode with Speed controller gains taken from the Digital setup*/
    COMMAND_MODE_ERROR = -1				  /*!< error */
} CommandMode;

/* Motor Direction */
typedef enum {
    COUNTERCLOCKWISE = 0, /*!< counter-clockwise direction */
    CLOCKWISE = 1,		  /*!< clockwise direction */
    DIRECTION_ERROR = -1  /*!< error */
} Direction;

/* Feedback Control Mode */
typedef enum {
    SENSORLESS_HSO = 0,				 /*!< sensorless mode */
    ENCODERS = 1,					 /*!< encoders mode */
    HALL_SENSORS = 2,				 /*!< hall sensors mode */
    SENSORLESS_ZSFT = 3,			 /*!< Sensor-less Zero Speed Full Torque feedback Mode (ZSFT) */
    FEEDBACK_CONTROL_MODE_ERROR = -1 /*!< error */
} FeedbackControlMode;

/* Control Mode */
typedef enum {
	SPEED_MODE = 0,			/*!< speed mode */
    TORQUE_MODE = 1,		/*!< torque mode */
    POSITION_MODE = 2,		/*!< position mode */
    CONTROL_MODE_ERROR = -1 /*!< error */
} ControlMode;

/* Motor Type */
typedef enum {
    DC = 0,					 /*!< dc motor */
    BLDC_PMSM = 1,			 /*!< brushless dc motor  */
    ACIM = 2,				 /*!< acim motor */
    BLDC_PMSM_ULTRAFAST = 3, /*!< brushless dc motor fast */
    MOTOR_TYPE_ERROR = -1	 /*!< error */
} MotorType;

/* UART Baudrate */
typedef enum {
    RATE_937500 = 0,		/*!< baud rate 937500 */
    RATE_115200 = 1,		/*!< baud rate 115200 */
    UART_BAUDRATE_ERROR = 2 /*!< error */
} UartBaudrate;

/* CAN Bus Baudrate */
typedef enum {
    RATE_1000 = 0, /*!< Baudrate 1000 kbits/s */
    RATE_500 = 1,  /*!< Baudrate 500 kbits/s */
    RATE_250 = 2,  /*!< Baudrate 250 kbits/s */
    RATE_125 = 3,  /*!< Baudrate 125 kbits/s */
    RATE_100 = 4   /*!< Baudrate 100 kbits/s */
} CanbusBaudrate;

/* Frequency */
typedef enum {
    RATE_8 = 0,	 /*!< Baudrate 8 Mbits/s */
    RATE_16 = 1, /*!< Baudrate 16 Mbits/s */
    RATE_20 = 2, /*!< Baudrate 20 Mbits/s */
} Frequency;

/* Action */
typedef enum {
    STOP = 0,		  /*!< stop */
    START = 1,		  /*!< start */
    ACTION_ERROR = -1 /*!< error */
} Action;

/* Enable/Disable */
typedef enum {
    SOLO_DISABLE = 0,			  /*!< Disable */ // HAL lib also has this macro
    SOLO_ENABLE = 1,				  /*!< Enable */
    SOLO_DISABLE_ENABLE_ERROR = -1 /*!< error */
} DisableEnable;

/* Position Sensor Calibration */
typedef enum {
	STOP_CALIBRATION = 0,						  /*!< stop colibration */
    INCREMENTAL_ENCODER_START_CALIBRATION = 1,	  /*!< incremental encoder start calibration */
    HALL_SENSOR_START_CALIBRATION = 2,			  /*!< hall sensor start calibration */
    POSITION_SENSOR_CALIBRATION_ACTION_ERROR = -1 /*!< error */
} PositionSensorCalibrationAction;

/* Motion Profile */
typedef enum {
    STEP_RAMP_RESPONSE = 0,		   /*!< step ramp profile */
    TIME_BASED_ST_CURVE = 1,	   /*!< time based st curve */
    TIME_OPTIMAL_ST_CURVE = 2,	   /*!< time optimal st curve */
    MOTION_PROFILE_MODE_ERROR = -1 /*!< error */
} MotionProfileMode;

/* Digital I/O State */
typedef enum {
    LOW_IO_STATE = 0,					/*!< GPIO Low State */
    HIGH_IO_STATE = 1,					/*!< GPIO High State */
    DIGITAL_IO_STATE_ERROR = -1 /*!< error */
} DigitalIoState;

/* Channel */
typedef enum {
    CHANNEL0 = 0,	   /*!< Channel 0 */
    CHANNEL1 = 1,	   /*!< Channel 1 */
    CHANNEL2 = 2,	   /*!< Channel 2 */
    CHANNEL3 = 3,	   /*!< Channel 3 */
    CHANNEL_ERROR = -1 /*!< error */
} Channel;

/* Struct representing SOLO Motor Controller */
typedef struct {
    unsigned char deviceAddress;
    int lastError;
} SOLOMotorController;


#endif /* SOLO_MOTOR_CONTROLLERS_H */
