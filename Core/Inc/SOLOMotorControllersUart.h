/**
 *******************************************************************************
 * @file    SOLOMotorControllersUart.h
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the functions prototypes for the Solo Drivers
 *          uart communications.
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

#ifndef SOLO_MOTOR_CONTROLLERS_UART_H
#define SOLO_MOTOR_CONTROLLERS_UART_H

#include "SOLOMotorControllers.h"
#include "SOLOMotorControllersUtils.h"
#include "stm32f7xx_hal.h"
#include "main.h"

// UART_Commands UART Commands
// All uart command hex code
#define INITIATOR 0xFF // 0xFFFF
#define BROADCAST_ADDRESS 0xFF
#define ENDING 0xFE
#define ERROR 0xEE // 0xEEEEEEEE
#define SOLO_CRC 0x00 // CRC already defined in HAL
#define WRITE_DEVICE_ADDRESS 0x01
#define WRITE_COMMAND_MODE 0x02
#define WRITE_CURRENT_LIMIT 0x03
#define WRITE_TORQUE_REFERENCE_IQ 0x04
#define WRITE_SPEED_REFERENCE 0x05
#define WRITE_POWER_REFERENCE 0x06
#define WRITE_MOTOR_PARAMETERS_IDENTIFICATION 0x07
#define WRITE_DRIVE_DISABLE_ENABLE 0x08
#define WRITE_OUTPUT_PWM_FREQUENCY_KHZ 0x09
#define WRITE_SPEED_CONTROLLER_KP 0x0A
#define WRITE_SPEED_CONTROLLER_KI 0x0B
#define WRITE_MOTOR_DIRECTION 0x0C
#define WRITE_MOTOR_RESISTANCE 0x0D
#define WRITE_MOTOR_INDUCTANCE 0x0E
#define WRITE_MOTOR_POLES_COUNTS 0x0F
#define WRITE_INCREMENTAL_ENCODER_LINES 0x10
#define WRITE_SPEED_LIMIT 0x11
#define WRITE_RESET_DEVICE_ADDRESS 0x12
#define WRITE_FEEDBACK_CONTROL_MODE 0x13
#define WRITE_RESET_FACTORY 0x14
#define WRITE_RESET_POSITION 0x1F
#define WRITE_MOTOR_TYPE 0x15
#define WRITE_CONTROL_MODE 0x16
#define WRITE_CURRENT_CONTROLLER_KP 0x17
#define WRITE_CURRENT_CONTROLLER_KI 0x18
#define WRITE_MONITORING_MODE 0x19
#define WRITE_MAGNETIZING_CURRENT_ID_REFERENCE 0x1A
#define WRITE_POSITION_REFERENCE 0x1B
#define WRITE_POSITION_CONTROLLER_KP 0x1C
#define WRITE_POSITION_CONTROLLER_KI 0x1D
#define WRITE_OVERWRITE_ERROR_REGISTER 0x20
#define WRITE_ZSFT_INJECTION_AMPLITUDE 0x21
#define WRITE_ZSFT_POLARITY_AMPLITUDE 0x22
#define WRITE_OBSERVER_GAIN_DC 0x23
#define WRITE_ZSFT_INJECTION_FREQUENCY 0x24
#define WRITE_SENSORLESS_TRANSITION_SPEED 0x25
#define WRITE_UART_BAUDRATE 0x26 // Set UART line baud-rate - 937500 / 115200 [ bits/s]
#define WRITE_SENSOR_CALIBRATION 0x27
#define WRITE_ENCODER_HALL_CCW_OFFSET 0x28
#define WRITE_ENCODER_HALL_CW_OFFSET 0x29
#define WRITE_SPEED_ACCELERATION_VALUE 0x2A
#define WRITE_SPEED_DECELERATION_VALUE 0x2B
#define WRITE_CANBUS_BAUDRATE 0x2C
#define WRITE_ASRDC 0x2D
#define WRITE_MOTION_PROFILE_MODE 0x30
#define WRITE_MOTION_PROFILE_VARIABLE1 0x31
#define WRITE_MOTION_PROFILE_VARIABLE2 0x32
#define WRITE_MOTION_PROFILE_VARIABLE3 0x33
#define WRITE_MOTION_PROFILE_VARIABLE4 0x34
#define WRITE_MOTION_PROFILE_VARIABLE5 0x35
#define WRITE_DIGITAL_OUTPUTS_REGISTER 0x38
#define WRITE_REGENERATION_CURRENT_LIMIT 0x39
#define WRITE_POSITION_SENSOR_DIGITAL_FILTER_LEVEL 0x3A

#define READ_DEVICE_ADDRESS 0x81
#define READ_PHASE_A_VOLTAGE 0x82
#define READ_PHASE_B_VOLTAGE 0x83
#define READ_PHASE_A_CURRENT 0x84
#define READ_PHASE_B_CURRENT 0x85
#define READ_BUS_VOLTAGE 0x86
#define READ_DC_MOTOR_CURRENT_IM 0x87
#define READ_DC_MOTOR_VOLTAGE_VM 0x88
#define READ_SPEED_CONTROLLER_KP 0x89
#define READ_SPEED_CONTROLLER_KI 0x8A
#define READ_OUTPUT_PWM_FREQUENCY_HZ 0x8B
#define READ_CURRENT_LIMIT 0x8C
#define READ_QUADRATURE_CURRENT_IQ_FEEDBACK 0x8D
#define READ_MAGNETIZING_CURRENT_ID_FEEDBACK 0x8E // Magnetizing
#define READ_MOTOR_POLES_COUNTS 0x8F
#define READ_INCREMENTAL_ENCODER_LINES 0x90
#define READ_CURRENT_CONTROLLER_KP 0x91
#define READ_CURRENT_CONTROLLER_KI 0x92
#define READ_BOARD_TEMPERATURE 0x93
#define READ_MOTOR_RESISTANCE 0x94
#define READ_MOTOR_INDUCTANCE 0x95
#define READ_SPEED_FEEDBACK 0x96
#define READ_MOTOR_TYPE 0x97
#define READ_FEEDBACK_CONTROL_MODE 0x99
#define READ_COMMAND_MODE 0x9A
#define READ_CONTROL_MODE 0x9B
#define READ_SPEED_LIMIT 0x9C
#define READ_POSITION_CONTROLLER_KP 0x9D
#define READ_POSITION_CONTROLLER_KI 0x9E
#define READ_POSITION_COUNTS_FEEDBACK 0xA0
#define READ_ERROR_REGISTER 0xA1
#define READ_DEVICE_FIRMWARE_VERSION 0xA2
#define READ_DEVICE_HARDWARE_VERSION 0xA3
#define READ_TORQUE_REFERENCE_IQ 0xA4              // Read Torque /Iq Reference
#define READ_SPEED_REFERENCE 0xA5                  // Read Speed Reference
#define READ_MAGNETIZING_CURRENT_ID_REFERENCE 0xA6 // Read Magnetizing Current / Id Reference
#define READ_POSITION_REFERENCE 0xA7
#define READ_POWER_REFERENCE 0xA8
#define READ_MOTOR_DIRECTION 0xA9
#define READ_ZSFT_INJECTION_AMPLITUDE 0xAA
#define READ_ZSFT_POLARITY_AMPLITUDE 0xAB
#define READ_OBSERVER_GAIN_DC 0xAC
#define READ_ZSFT_INJECTION_FREQUENCY 0xAD
#define READ_SENSORLESS_TRANSITION_SPEED 0xAE
#define READ_3_PHASE_MOTOR_ANGLE 0xB0
#define READ_ENCODER_HALL_CCW_OFFSET 0xB1
#define READ_ENCODER_HALL_CW_OFFSET 0xB2
#define READ_UART_BAUDRATE 0xB3 // 0 / 1 ( 937500 / 115200 [bits/s] )
#define READ_SPEED_ACCELERATION_VALUE 0xB4
#define READ_SPEED_DECELERATION_VALUE 0xB5
#define READ_CANBUS_BAUDRATE 0xB6
#define READ_ASRDC 0xB7
#define READ_ENCODER_INDEX_COUNTS 0xB8
#define READ_MOTION_PROFILE_MODE 0xBB
#define READ_MOTION_PROFILE_VARIABLE1 0xBC
#define READ_MOTION_PROFILE_VARIABLE2 0xBD
#define READ_MOTION_PROFILE_VARIABLE3 0xBE
#define READ_MOTION_PROFILE_VARIABLE4 0xBF
#define READ_MOTION_PROFILE_VARIABLE5 0xC0
#define READ_PT1000_SENSOR_VOLTAGE 0xC3
#define READ_DIGITAL_OUTPUT_REGISTER 0xC4
#define READ_DIGITAL_INPUT_REGISTER 0xC5
#define READ_ANALOGUE_INPUT 0xC6
#define READ_DRIVE_DISABLE_ENABLE 0xC7
#define READ_REGENERATION_CURRENT_LIMIT 0xC8
#define READ_POSITION_SENSOR_DIGITAL_FILTER_LEVEL 0x3A

typedef struct {
    unsigned char addr;
    UART_HandleTypeDef *huart;
    long millisecondsTimeout;
    int packetFailureTrialAttempts;
    int error;
} SOLOMotorControllersUart;

void SOLOMotorControllersUart_Init(SOLOMotorControllersUart* solo_uart, unsigned char deviceAddress, UART_HandleTypeDef *huart, 
                                    long millisecondsTimeout, int packetFailureTrialAttempts);

bool SetDeviceAddress(SOLOMotorControllersUart* solo_uart, unsigned char deviceAddress);
bool SetCommandMode(SOLOMotorControllersUart* solo_uart, CommandMode mode);
bool SetCurrentLimit(SOLOMotorControllersUart* solo_uart, float currentLimit);
bool SetTorqueReferenceIq(SOLOMotorControllersUart* solo_uart, float torqueReferenceIq);
bool SetSpeedReference(SOLOMotorControllersUart* solo_uart, long speedReference);
bool SetPowerReference(SOLOMotorControllersUart* solo_uart, float powerReference);
bool MotorParametersIdentification(SOLOMotorControllersUart* solo_uart, Action identification);
bool SetDriveDisableEnable(SOLOMotorControllersUart* solo_uart, DisableEnable action);
bool SetOutputPwmFrequencyKhz(SOLOMotorControllersUart* solo_uart, long pwmFreq);
bool SetSpeedControllerKp(SOLOMotorControllersUart* solo_uart, float kp);
bool SetSpeedControllerKi(SOLOMotorControllersUart* solo_uart, float ki);
bool SetMotorDirection(SOLOMotorControllersUart* solo_uart, Direction direction);
bool SetMotorResistance(SOLOMotorControllersUart* solo_uart, float resistance);
bool SetMotorInductance(SOLOMotorControllersUart* solo_uart, float inductance);
bool SetMotorPolesCounts(SOLOMotorControllersUart* solo_uart, long poles);
bool SetIncrementalEncoderLines(SOLOMotorControllersUart* solo_uart, long lines);
bool SetSpeedLimit(SOLOMotorControllersUart* solo_uart, long speedLimit);
bool SetFeedbackControlMode(SOLOMotorControllersUart* solo_uart, FeedbackControlMode mode);
bool ResetFactory(SOLOMotorControllersUart* solo_uart);
bool ResetPositionToZero(SOLOMotorControllersUart* solo_uart);
bool SetMotorType(SOLOMotorControllersUart* solo_uart, MotorType motorType);
bool SetControlMode(SOLOMotorControllersUart* solo_uart, ControlMode controlMode);
bool SetCurrentControllerKp(SOLOMotorControllersUart* solo_uart, float kp);
bool SetCurrentControllerKi(SOLOMotorControllersUart* solo_uart, float ki);
bool SetMagnetizingCurrentIdReference(SOLOMotorControllersUart* solo_uart, float id);
bool SetPositionReference(SOLOMotorControllersUart* solo_uart, long position);
bool SetPositionControllerKp(SOLOMotorControllersUart* solo_uart, float kp);
bool SetPositionControllerKi(SOLOMotorControllersUart* solo_uart, float ki);
bool OverwriteErrorRegister(SOLOMotorControllersUart* solo_uart);
bool SetZsftInjectionAmplitude(SOLOMotorControllersUart* solo_uart, float amp);
bool SetZsftPolarityAmplitude(SOLOMotorControllersUart* solo_uart, float amp);
bool SetObserverGainDc(SOLOMotorControllersUart* solo_uart, float gain);
bool SetZsftInjectionFrequency(SOLOMotorControllersUart* solo_uart, long freq);
bool SetSensorlessTransitionSpeed(SOLOMotorControllersUart* solo_uart, long speed);
bool SetUartBaudrate(SOLOMotorControllersUart* solo_uart, UartBaudrate baudrate);
bool SensorCalibration(SOLOMotorControllersUart* solo_uart, PositionSensorCalibrationAction action);
bool SetEncoderHallCcwOffset(SOLOMotorControllersUart* solo_uart, float offset);
bool SetEncoderHallCwOffset(SOLOMotorControllersUart* solo_uart, float offset);
bool SetSpeedAccelerationValue(SOLOMotorControllersUart* solo_uart, float accel);
bool SetSpeedDecelerationValue(SOLOMotorControllersUart* solo_uart, float decel);
bool SetCanbusBaudrate(SOLOMotorControllersUart* solo_uart, CanbusBaudrate baudrate);
bool SetAnalogueSpeedResolutionDivisionCoefficient(SOLOMotorControllersUart* solo_uart, long divCoef);
bool SetMotionProfileMode(SOLOMotorControllersUart* solo_uart, MotionProfileMode mode);
bool SetMotionProfileVariable1(SOLOMotorControllersUart* solo_uart, float val);
bool SetMotionProfileVariable2(SOLOMotorControllersUart* solo_uart, float val);
bool SetMotionProfileVariable3(SOLOMotorControllersUart* solo_uart, float val);
bool SetMotionProfileVariable4(SOLOMotorControllersUart* solo_uart, float val);
bool SetMotionProfileVariable5(SOLOMotorControllersUart* solo_uart, float val);
bool SetRegenerationCurrentLimit(SOLOMotorControllersUart* solo_uart, float current);
bool SetPositionSensorDigitalFilterLevel(SOLOMotorControllersUart* solo_uart, long level);
bool SetDigitalOutputState(SOLOMotorControllersUart* solo_uart, Channel channel, DigitalIoState state);

long GetDeviceAddress(SOLOMotorControllersUart* solo_uart);
float GetPhaseAVoltage(SOLOMotorControllersUart* solo_uart);
float GetPhaseBVoltage(SOLOMotorControllersUart* solo_uart);
float GetPhaseACurrent(SOLOMotorControllersUart* solo_uart);
float GetPhaseBCurrent(SOLOMotorControllersUart* solo_uart);
float GetBusVoltage(SOLOMotorControllersUart* solo_uart);
float GetDcMotorCurrentIm(SOLOMotorControllersUart* solo_uart);
float GetDcMotorVoltageVm(SOLOMotorControllersUart* solo_uart);
float GetSpeedControllerKp(SOLOMotorControllersUart* solo_uart);
float GetSpeedControllerKi(SOLOMotorControllersUart* solo_uart);
long GetOutputPwmFrequencyKhz(SOLOMotorControllersUart* solo_uart);
float GetCurrentLimit(SOLOMotorControllersUart* solo_uart);
float GetQuadratureCurrentIqFeedback(SOLOMotorControllersUart* solo_uart);
float GetMagnetizingCurrentIdFeedback(SOLOMotorControllersUart* solo_uart);
long GetMotorPolesCounts(SOLOMotorControllersUart* solo_uart);
long GetIncrementalEncoderLines(SOLOMotorControllersUart* solo_uart);
float GetCurrentControllerKp(SOLOMotorControllersUart* solo_uart);
float GetCurrentControllerKi(SOLOMotorControllersUart* solo_uart);
float GetBoardTemperature(SOLOMotorControllersUart* solo_uart);
float GetMotorResistance(SOLOMotorControllersUart* solo_uart);
float GetMotorInductance(SOLOMotorControllersUart* solo_uart);
long GetSpeedFeedback(SOLOMotorControllersUart* solo_uart);
MotorType GetMotorType(SOLOMotorControllersUart* solo_uart);
FeedbackControlMode GetFeedbackControlMode(SOLOMotorControllersUart* solo_uart);
CommandMode GetCommandMode(SOLOMotorControllersUart* solo_uart);
ControlMode GetControlMode(SOLOMotorControllersUart* solo_uart);
long GetSpeedLimit(SOLOMotorControllersUart* solo_uart);
float GetPositionControllerKp(SOLOMotorControllersUart* solo_uart);
float GetPositionControllerKi(SOLOMotorControllersUart* solo_uart);
long GetPositionCountsFeedback(SOLOMotorControllersUart* solo_uart);
long GetErrorRegister(SOLOMotorControllersUart* solo_uart);
long GetDeviceFirmwareVersion(SOLOMotorControllersUart* solo_uart);
long GetDeviceHardwareVersion(SOLOMotorControllersUart* solo_uart);
float GetTorqueReferenceIq(SOLOMotorControllersUart* solo_uart);
long GetSpeedReference(SOLOMotorControllersUart* solo_uart);
float GetMagnetizingCurrentIdReference(SOLOMotorControllersUart* solo_uart);
long GetPositionReference(SOLOMotorControllersUart* solo_uart);
float GetPowerReference(SOLOMotorControllersUart* solo_uart);
Direction GetMotorDirection(SOLOMotorControllersUart* solo_uart);
float GetZsftInjectionAmplitude(SOLOMotorControllersUart* solo_uart);
float GetZsftPolarityAmplitude(SOLOMotorControllersUart* solo_uart);
float GetObserverGainDc(SOLOMotorControllersUart* solo_uart);
long GetZsftInjectionFrequency(SOLOMotorControllersUart* solo_uart);
long GetSensorlessTransitionSpeed(SOLOMotorControllersUart* solo_uart);
float Get3PhaseMotorAngle(SOLOMotorControllersUart* solo_uart);
float GetEncoderHallCcwOffset(SOLOMotorControllersUart* solo_uart);
float GetEncoderHallCwOffset(SOLOMotorControllersUart* solo_uart);
UartBaudrate GetUartBaudrate(SOLOMotorControllersUart* solo_uart);
float GetSpeedAccelerationValue(SOLOMotorControllersUart* solo_uart);
float GetSpeedDecelerationValue(SOLOMotorControllersUart* solo_uart);
long GetCanbusBaudrate(SOLOMotorControllersUart* solo_uart);
long GetAnalogueSpeedResolutionDivisionCoefficient(SOLOMotorControllersUart* solo_uart);
long GetEncoderIndexCounts(SOLOMotorControllersUart* solo_uart);
bool CommunicationIsWorking(SOLOMotorControllersUart* solo_uart);
MotionProfileMode GetMotionProfileMode(SOLOMotorControllersUart* solo_uart);
float GetMotionProfileVariable1(SOLOMotorControllersUart* solo_uart);
float GetMotionProfileVariable2(SOLOMotorControllersUart* solo_uart);
float GetMotionProfileVariable3(SOLOMotorControllersUart* solo_uart);
float GetMotionProfileVariable4(SOLOMotorControllersUart* solo_uart);
float GetMotionProfileVariable5(SOLOMotorControllersUart* solo_uart);
long GetDigitalOutputsRegister(SOLOMotorControllersUart* solo_uart);
long GetPT1000SensorVoltage(SOLOMotorControllersUart* solo_uart);
DigitalIoState GetDigitalOutputState(SOLOMotorControllersUart* solo_uart, Channel channel);
float GetRegenerationCurrentLimit(SOLOMotorControllersUart* solo_uart);
long GetPositionSensorDigitalFilterLevel(SOLOMotorControllersUart* solo_uart);
long GetDigitalInputRegister(SOLOMotorControllersUart* solo_uart);
DisableEnable GetDriveDisableEnable(SOLOMotorControllersUart* solo_uart);
int GetDigitalOutput(SOLOMotorControllersUart* solo_uart, int pinNumber);
DigitalIoState GetAnalogueInput(SOLOMotorControllersUart* solo_uart, Channel channel);


#endif // SOLO_MOTOR_CONTROLLERS_UART_H
