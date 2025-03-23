/**
 *******************************************************************************
 * @file    SOLOMotorControllersUart.cpp
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the functions for the Solo Drivers
 *          uart communications.
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.4.3
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 *******************************************************************************
 */

 #include "SOLOMotorControllersUart.h"

 #include <stdint.h>
 #include <stdbool.h>
 #include <stdlib.h>
 #include <string.h>
 

 void SOLOMotorControllersUart_Init(SOLOMotorControllersUart* solo_uart, unsigned char deviceAddress, UART_HandleTypeDef *huart, 
                                    long millisecondsTimeout, int packetFailureTrialAttempts)
 {
    solo_uart->addr = deviceAddress;
    solo_uart->huart = huart;
    solo_uart->millisecondsTimeout = millisecondsTimeout;
    solo_uart->packetFailureTrialAttempts = packetFailureTrialAttempts;
 }
 
 static bool ExeCMD(SOLOMotorControllersUart* solo_uart, unsigned char cmd[]) // private/local
 {
   unsigned char _cmd[] = {INITIATOR, INITIATOR, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], SOLO_CRC, ENDING};
   unsigned char _readPacket[10] = {0};
 
   // DEBUG SENT
   //  printmsg("SENT ");
   //  for (int i=0; i<10; i++) printmsg("0x%.2X ",_cmd[i]);
   //  printmsg("\n");
 
   bool isPACKET_FAILURE_TRIAL_ATTEMPTS_OVERFLOW = true;
   // FailureTrialAttempts block
   for (int attempts = 0; attempts < solo_uart->packetFailureTrialAttempts; attempts++)
   {
 
     // ensure huart buffer to be empty before new comunications
     uint8_t dummy = 0;
     while (HAL_UART_Receive(solo_uart->huart, &dummy, 1, 5) == HAL_OK); // is this necessary?
     
     // clear rx buffer
     memset(_readPacket,0,sizeof(_readPacket)); 
     
     // Send command
     HAL_UART_Transmit(solo_uart->huart, _cmd, sizeof(_cmd), HAL_MAX_DELAY);

     // Wait for transmission to complete (remove if uart timing is not crucial)
     while (__HAL_UART_GET_FLAG(solo_uart->huart, UART_FLAG_TC) == RESET);
     
     if(HAL_UART_Receive(solo_uart->huart, _readPacket, sizeof(_readPacket), solo_uart->millisecondsTimeout) != HAL_OK)
     {
        continue;
     }

     // DEBUG
     //  printmsg("READ ");
     //  for (int r=0; r<10; r++) printmsg("0x%.2X ",_readPacket[r]);
     //  printmsg("\n");
 
     // error, send async error
     if ((_readPacket[3] == 0xA1 && cmd[1] != 0xA1) || _readPacket[3] == 0xFE)
     {
       continue;
     }
 
     // packet response correctly
     if (_readPacket[0] == INITIATOR &&
         _readPacket[1] == INITIATOR &&
         _readPacket[3] != 0 &&
         _readPacket[8] == SOLO_CRC && _readPacket[9] == ENDING)
     {
       isPACKET_FAILURE_TRIAL_ATTEMPTS_OVERFLOW = false;
       break;
     }
   }
 
   if (isPACKET_FAILURE_TRIAL_ATTEMPTS_OVERFLOW)
   {
     cmd[0] = ERROR;
     cmd[1] = ERROR;
     cmd[2] = ERROR;
     cmd[3] = ERROR;
     cmd[4] = ERROR;
     cmd[5] = ERROR;
     solo_uart->error = PACKET_FAILURE_TRIAL_ATTEMPTS_OVERFLOW;
     return false;
   }
 
   if (_readPacket[0] == _cmd[0] && _readPacket[1] == _cmd[1] && (_readPacket[2] == _cmd[2] || _cmd[2] == 0xFF) && _readPacket[3] == _cmd[3] && _readPacket[8] == _cmd[8] && _readPacket[9] == _cmd[9])
   {
     cmd[0] = _readPacket[2];
     cmd[1] = _readPacket[3];
     cmd[2] = _readPacket[4];
     cmd[3] = _readPacket[5];
     cmd[4] = _readPacket[6];
     cmd[5] = _readPacket[7];
     solo_uart->error = NO_ERROR_DETECTED;
     return true;
   }
 
   cmd[0] = ERROR;
   cmd[1] = ERROR;
   cmd[2] = ERROR;
   cmd[3] = ERROR;
   cmd[4] = ERROR;
   cmd[5] = ERROR;
   solo_uart->error = GENERAL_ERROR;
   
   return false;
 }
 
 /**
  * @brief  This command sets the desired device address for a SOLO unit
  *           .The method refers to the Uart Write command: 0x01
  * @param[in] deviceAddress  address want to set for board
  * @retval bool 0 fail / 1 for success
  */
 bool SetDeviceAddress(SOLOMotorControllersUart* solo_uart, unsigned char deviceAddress)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetDeviceAddressInputValidation(deviceAddress,&solo_uart->error))
   {
     return false;
   }
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_DEVICE_ADDRESS, 0x00, 0x00, 0x00, deviceAddress};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the mode of the operation of SOLO
  *         in terms of operating in Analogue mode or Digital
  *           .The method refers to the Uart Write command: 0x02
  * @param[in] mode  enum that specify mode of the operation of SOLO
  * @retval bool 0 fail / 1 for success
  */
 bool SetCommandMode(SOLOMotorControllersUart* solo_uart, CommandMode mode)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_COMMAND_MODE, 0x00, 0x00, 0x00, (unsigned char)mode};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command defines the maximum allowed current into the motor in terms of Amps
  *           .The method refers to the Uart Write command: 0x03
  * @param[in] currentLimit  a float value [Amps]
  * @retval bool 0 fail / 1 for success
  */
 bool SetCurrentLimit(SOLOMotorControllersUart* solo_uart, float currentLimit)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetCurrentLimitInputValidation(currentLimit, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(currentLimit, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_CURRENT_LIMIT, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the amount of desired current that acts in torque generation
  *           .The method refers to the Uart Write command: 0x04
  * @param[in] torqueReferenceIq  a float [Amps]
  * @retval bool 0 fail / 1 for success
  */
 bool SetTorqueReferenceIq(SOLOMotorControllersUart* solo_uart, float torqueReferenceIq)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetTorqueReferenceIqInputValidation(torqueReferenceIq, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(torqueReferenceIq, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_TORQUE_REFERENCE_IQ, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
  *           .The method refers to the Uart Write command: 0x05
  * @param[in] speedReference  a long value [RPM]
  * @retval bool 0 fail / 1 for success
  */
 bool SetSpeedReference(SOLOMotorControllersUart* solo_uart, long speedReference)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetSpeedReferenceInputValidation(speedReference, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertLongToData(speedReference, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_SPEED_REFERENCE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command defines the amount of power percentage during only
  *         Open-loop mode for 3-phase motors
  *           .The method refers to the Uart Write command: 0x06
  * @param[in] powerReference  a float value between 0 to 100
  * @retval bool 0 fail / 1 for success
  */
 bool SetPowerReference(SOLOMotorControllersUart* solo_uart, float powerReference)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetPowerReferenceInputValidation(powerReference, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(powerReference, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_POWER_REFERENCE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
   * @brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
             identifying the electrical parameters of the Motor connected
               .The method refers to the Uart Write command: 0x07
   * @param[in] identification  enum that specify Start or Stop of something in SOLO
   
   * @retval bool 0 fail / 1 for success
   */
 bool MotorParametersIdentification(SOLOMotorControllersUart* solo_uart, Action identification)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_MOTOR_PARAMETERS_IDENTIFICATION, 0x00, 0x00, 0x00, (unsigned char)identification};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command Disables or Enables the Controller resulting in deactivation or activation of the
  *           switching at the output, by disabling the drive, the effect of the Controller on the Motor will be
  *            almost eliminated ( except for body diodes of the Mosfets) allowing freewheeling
  *           .The method refers to the Uart Write command: 0x08
  * @param[in]  action  enum that specify Disable or Enable of something in SOLO
  * @param[out]  solo_uart->error   pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
 bool SetDriveDisableEnable(SOLOMotorControllersUart* solo_uart, DisableEnable action)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, WRITE_DRIVE_DISABLE_ENABLE, 0x00, 0x00, 0x00, (unsigned char)action};
 
   return ExeCMD(solo_uart,cmd);
 }
 
 /**
  * @brief  This command sets the output switching frequency of the whole power unit on the Motor
  *           .The method refers to the Uart Write command: 0x09
  * @param[in] outputPwmFrequencyKhz  switching frequencies [kHz]
  * @retval bool 0 fail / 1 for success
  */
 bool SetOutputPwmFrequencyKhz(SOLOMotorControllersUart* solo_uart, long outputPwmFrequencyKhz)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetOutputPwmFrequencyKhzInputValidation(outputPwmFrequencyKhz, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertLongToData(outputPwmFrequencyKhz, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_OUTPUT_PWM_FREQUENCY_KHZ, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart,cmd);
 }
 
 /**
  * @brief  This command sets the Speed controller Kp Gain, and it will
  *         be functional only in Digital Closed-loop mode
  *           .The method refers to the Uart Write command: 0x0A
  * @param[in] speedControllerKp  a float value between 0 to 300
  * @retval bool 0 fail / 1 for success
  */
 bool SetSpeedControllerKp(SOLOMotorControllersUart* solo_uart, float speedControllerKp)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetSpeedControllerKpInputValidation(speedControllerKp, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(speedControllerKp, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_SPEED_CONTROLLER_KP, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the Speed controller Ki gain, and it will
  *         be functional only in Digital Closed-loop mode
  *           .The method refers to the Uart Write command: 0x0B
  * @param[in] speedControllerKi  a float value between 0 to 300
  * @retval bool 0 fail / 1 for success
  */
 bool SetSpeedControllerKi(SOLOMotorControllersUart* solo_uart, float speedControllerKi)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetSpeedControllerKiInputValidation(speedControllerKi, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(speedControllerKi, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_SPEED_CONTROLLER_KI, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This commands sets the direction of the rotation of the motor
  *         either to ClockWise rotation or to Counter Clockwise Rotation
  *           .The method refers to the Uart Write command: 0x0C
  * @param[in] motorDirection  enum that specify the direction of the rotation of the motor
  * @retval bool 0 fail / 1 for success
  */
 bool SetMotorDirection(SOLOMotorControllersUart* solo_uart, Direction motorDirection)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_MOTOR_DIRECTION, 0x00, 0x00, 0x00, (unsigned char)motorDirection};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the amount of the Phase or Armature resistance
  *         for 3-phase or DC Brushed motors respectively
  *           .The method refers to the Uart Write command: 0x0D
  * @param[in] motorResistance  a float value [Ohm]
  * @retval bool 0 fail / 1 for success
  */
 bool SetMotorResistance(SOLOMotorControllersUart* solo_uart, float motorResistance)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetMotorResistanceInputValidation(motorResistance, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(motorResistance, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_MOTOR_RESISTANCE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the amount of the Phase or Armature Inductance
  *         for 3-phase or DC Brushed motors respectively
  *           .The method refers to the Uart Write command: 0x0E
  * @param[in] motorInductance  a float value [Henry]
  * @retval bool 0 fail / 1 for success
  */
 bool SetMotorInductance(SOLOMotorControllersUart* solo_uart, float motorInductance)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetMotorInductanceInputValidation(motorInductance, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(motorInductance, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_MOTOR_INDUCTANCE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
  *           .The method refers to the Uart Write command: 0x0F
  * @param[in] motorPolesCounts  a long value between 1 to 254
  * @retval bool 0 fail / 1 for success
  */
 bool SetMotorPolesCounts(SOLOMotorControllersUart* solo_uart, long motorPolesCounts)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetMotorPolesCountsInputValidation(motorPolesCounts, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertLongToData(motorPolesCounts, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_MOTOR_POLES_COUNTS, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the pre-quad number of physical lines of an
  *         incremental encoder engraved on its disk
  *           .The method refers to the Uart Write command: 0x10
  * @param[in] incrementalEncoderLines  a long value [pre-quad]
  * @retval bool 0 fail / 1 for success
  */
 bool SetIncrementalEncoderLines(SOLOMotorControllersUart* solo_uart, long incrementalEncoderLines)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetIncrementalEncoderLinesInputValidation(incrementalEncoderLines, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertLongToData(incrementalEncoderLines, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_INCREMENTAL_ENCODER_LINES, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the allowed speed during trajectory following
  *         in closed-loop position controlling mode
  *           .The method refers to the Uart Write command: 0x11
  * @param[in] speedLimit  a long value [RPM]
  * @retval bool 0 fail / 1 for success
  */
 bool SetSpeedLimit(SOLOMotorControllersUart* solo_uart, long speedLimit)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetSpeedLimitInputValidation(speedLimit, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertLongToData(speedLimit, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_SPEED_LIMIT, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the type of the feedback control SOLO has to operate
  *           .The method refers to the Uart Write command: 0x13
  * @param[in] feedbackControlMode  enum that specify the type of the feedback control SOLO
  * @retval bool 0 fail / 1 for success
  */
 bool SetFeedbackControlMode(SOLOMotorControllersUart* solo_uart, FeedbackControlMode feedbackControlMode)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
 
   unsigned char data[4];
   ConvertLongToData((long)feedbackControlMode, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_FEEDBACK_CONTROL_MODE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command resets SOLO to its factory setting to all the default parameters
  *           .The method refers to the Uart Write command: 0x14
  * @retval bool 0 fail / 1 for success
  */
 bool ResetFactory(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_RESET_FACTORY, 0x00, 0x00, 0x00, 0x01};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command resets SOLO position
          .The method refers to the Uart Write command: 0x1F
  * @retval bool 0 fail / 1 for success
  */
 bool ResetPositionToZero(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_RESET_POSITION, 0x00, 0x00, 0x00, 0x01};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the Motor type that is connected to SOLO in Digital Mode
  *           .The method refers to the Uart Write command: 0x15
  * @param[in] motorType  enum that specify the Motor type that is connected to SOLO in Digital Mode
  * @retval bool 0 fail / 1 for success
  */
 bool SetMotorType(SOLOMotorControllersUart* solo_uart, MotorType motorType)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
 
   unsigned char data[4];
   ConvertLongToData((long)motorType, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_MOTOR_TYPE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the Control Mode in terms of Torque,
  *         Speed or Position only in Digital Mode
  *           .The method refers to the Uart Write command: 0x16
  * @param[in] controlMode  enum that specify the Control Mode in terms of Torque,
  *                       Speed or Position only in Digital Mode
  * @retval bool 0 fail / 1 for success
  */
 bool SetControlMode(SOLOMotorControllersUart* solo_uart, ControlMode controlMode)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
 
   unsigned char data[4];
   ConvertLongToData((long)controlMode, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_CONTROL_MODE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the value for Current Controller Kp or proportional gain
  *           .The method refers to the Uart Write command: 0x17
  * @param[in] currentControllerKp  a float value between 0 to 16000
  * @retval bool 0 fail / 1 for success
  */
 bool SetCurrentControllerKp(SOLOMotorControllersUart* solo_uart, float currentControllerKp)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetCurrentControllerKpInputValidation(currentControllerKp, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(currentControllerKp, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_CURRENT_CONTROLLER_KP, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the value for Current Controller Ki or integral gain
  *           .The method refers to the Uart Write command: 0x18
  * @param[in] currentControllerKi  a float value between 0 to 16000
  * @retval bool 0 fail / 1 for success
  */
 bool SetCurrentControllerKi(SOLOMotorControllersUart* solo_uart, float currentControllerKi)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetCurrentControllerKiInputValidation(currentControllerKi, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(currentControllerKi, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_CURRENT_CONTROLLER_KI, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
  *         Weakening current reference to help the motor reaching speeds higher than
  *         nominal values and in case of AC Induction Motors Sets the desired magnetizing
  *         current (Id) required for controlling ACIM motors in FOC in Amps
  *           .The method refers to the Uart Write command: 0x1A
  * @param[in] magnetizingCurrentIdReference  a float value [Amps]
  * @retval bool 0 fail / 1 for success
  */
 bool SetMagnetizingCurrentIdReference(SOLOMotorControllersUart* solo_uart, float magnetizingCurrentIdReference)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetMagnetizingCurrentIdReferenceInputValidation(magnetizingCurrentIdReference, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(magnetizingCurrentIdReference, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_MAGNETIZING_CURRENT_ID_REFERENCE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the desired Position reference in terms of quadrature
  *         pulses while SOLO operates with the Incremental Encoders or in terms of
  *         pulses while while SOLO operates with Hall sensors
  *           .The method refers to the Uart Write command: 0x1B
  * @param[in] positionReference  a long value [Quad-Pulse]  
  * @retval bool 0 fail / 1 for success
  */
 bool SetPositionReference(SOLOMotorControllersUart* solo_uart, long positionReference)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetPositionReferenceInputValidation(positionReference, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertLongToData(positionReference, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_POSITION_REFERENCE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the value for Position Controller Kp or proportional gain
  *           .The method refers to the Uart Write command: 0x1C
  * @param[in] positionControllerKp  a float value between 0 to 16000
  * @retval bool 0 fail / 1 for success
  */
 bool SetPositionControllerKp(SOLOMotorControllersUart* solo_uart, float positionControllerKp)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetPositionControllerKpInputValidation(positionControllerKp, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(positionControllerKp, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_POSITION_CONTROLLER_KP, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the value for Position Controller Ki or integrator gain
  *           .The method refers to the Uart Write command: 0x1D
  * @param[in] positionControllerKi  a float value between 0 to 16000
  * @retval bool 0 fail / 1 for success
  */
 bool SetPositionControllerKi(SOLOMotorControllersUart* solo_uart, float positionControllerKi)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetPositionControllerKiInputValidation(positionControllerKi, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(positionControllerKi, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_POSITION_CONTROLLER_KI, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command overwrites the reported errors in Error Register
  *         reported with command code of "0xA1"
  *           .The method refers to the Uart Write command: 0x20
  * @retval bool 0 fail / 1 for success
  */
 bool OverwriteErrorRegister(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_OVERWRITE_ERROR_REGISTER, 0x00, 0x00, 0x00, 0x00};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  Once in Zero Speed Full Torque algorithm (ZSFT) for controlling the speed of a BLDC or PMSM
  *            in sensorless fashion, this parameter defines the strength of signal injection into the motor, the
  *            user has to make sure this value is not selected too high or too low
  *           .The method refers to the Uart Write command: 0x21
  * @param[in]  zsftInjectionAmplitude  a float value between 0.0 to 0.55
  * @param[out]  solo_uart->error   pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
 bool SetZsftInjectionAmplitude(SOLOMotorControllersUart* solo_uart, float zsftInjectionAmplitude)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetZsftInjectionAmplitudeValidation(zsftInjectionAmplitude, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(zsftInjectionAmplitude, data);
   unsigned char cmd[] = {solo_uart->addr, WRITE_ZSFT_INJECTION_AMPLITUDE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  Once in Zero Speed Full Torque algorithm (ZSFT) for controlling the speed of a BLDC or PMSM
  *             in sensorless fashion, this parameter defines the strength of signal injection into the motor to
  *            identify the polarity of the Motor at the startup
  *           .The method refers to the Uart Write command: 0x22
  * @param[in]  zsftPolarityAmplitude  a float value between 0.0 to 0.55
  * @retval bool 0 fail / 1 for success
  */
 bool SetZsftPolarityAmplitude(SOLOMotorControllersUart* solo_uart, float zsftPolarityAmplitude)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetZsftPolarityAmplitudeValidation(zsftPolarityAmplitude, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(zsftPolarityAmplitude, data);
   unsigned char cmd[] = {solo_uart->addr, WRITE_ZSFT_POLARITY_AMPLITUDE, data[0], data[1], data[2], data[3]};
 
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed of a DC brushed once the motor type
  *         is selected as DC brushed
  *           .The method refers to the Uart Write command: 0x23
  * @param[in] observerGain  a float value between 0.01 to 1000
  * @retval bool 0 fail / 1 for success
  */
 bool SetObserverGainDc(SOLOMotorControllersUart* solo_uart, float observerGain)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetObserverGainDcInputValidation(observerGain, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(observerGain, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_OBSERVER_GAIN_DC, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
   * @brief This command defines the frequency of signal injection into the Motor in
         runtime, by selecting zero the full injection frequency will be applied which allows to reach to
         higher speeds, however for some motors, it’s better to increase this value
  *           .The method refers to the Uart Write command: 0x24
  * @param[in]  zsftInjectionFrequency  a long value between 0 to 10
  * @retval bool 0 fail / 1 for success
  */
 bool SetZsftInjectionFrequency(SOLOMotorControllersUart* solo_uart, long zsftInjectionFrequency)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetZsftInjectionFrequencyInputValidation(zsftInjectionFrequency, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertLongToData(zsftInjectionFrequency, data);
   unsigned char cmd[] = {solo_uart->addr, WRITE_ZSFT_INJECTION_FREQUENCY, data[0], data[1], data[2], data[3]};
 
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  Once in Sensorless speed or torque controlling of a BLDC or PMSM motors, this parameter
  *				defines the speed in which the Low speed algorithm has to switch to high speed algorithm
  *           .The method refers to the Uart Write command: 0x25
  * @param[in]  sensorlessTransitionSpeed  a long value between 1 to 5000
  * @retval bool 0 fail / 1 for success
  */
 bool SetSensorlessTransitionSpeed(SOLOMotorControllersUart* solo_uart, long sensorlessTransitionSpeed)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetSensorlessTransitionSpeedInputValidation(sensorlessTransitionSpeed, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertLongToData(sensorlessTransitionSpeed, data);
   unsigned char cmd[] = {solo_uart->addr, WRITE_SENSORLESS_TRANSITION_SPEED, data[0], data[1], data[2], data[3]};
 
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the baud-rate of the UART line
  *           .The method refers to the Uart Write command: 0x26
  * @param[in] baudrate  enum that specify the baud-rate of the UART line
  * @retval bool 0 fail / 1 for success
  */
 bool SetUartBaudrate(SOLOMotorControllersUart* solo_uart, UartBaudrate baudrate)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
 
   unsigned char data[4];
   ConvertLongToData((long)baudrate, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_UART_BAUDRATE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command starts or stops the process of sensor calibration
  *           .The method refers to the Uart Write command: 0x27
  * @param[in] calibrationAction  enum that specify the process of sensor calibration
  * @retval bool 0 fail / 1 for success
  */
 bool SensorCalibration(SOLOMotorControllersUart* solo_uart, PositionSensorCalibrationAction calibrationAction)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
 
   unsigned char data[4];
   ConvertLongToData((long)calibrationAction, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_SENSOR_CALIBRATION, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.C.W direction
  *           .The method refers to the Uart Write command: 0x28
  * @param[in] encoderHallOffset  a float value between 0.0 to 1.0
  * @retval bool 0 fail / 1 for success
  */
 bool SetEncoderHallCcwOffset(SOLOMotorControllersUart* solo_uart, float encoderHallOffset)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetEncoderHallCcwOffsetInputValidation(encoderHallOffset, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(encoderHallOffset, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_ENCODER_HALL_CCW_OFFSET, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.W direction
  *           .The method refers to the Uart Write command: 0x29
  * @param[in] encoderHallOffset  a float value between 0.0 to 1.0
  * @retval bool 0 fail / 1 for success
  */
 bool SetEncoderHallCwOffset(SOLOMotorControllersUart* solo_uart, float encoderHallOffset)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetEncoderHallCwOffsetInputValidation(encoderHallOffset, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(encoderHallOffset, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_ENCODER_HALL_CW_OFFSET, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command defines the acceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
  *           .The method refers to the Uart Write command: 0x2A
  * @param[in] speedAccelerationValue  a float value [Rev/S^2]
  * @retval bool 0 fail / 1 for success
  */
 bool SetSpeedAccelerationValue(SOLOMotorControllersUart* solo_uart, float speedAccelerationValue)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetSpeedAccelerationValueInputValidation(speedAccelerationValue, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(speedAccelerationValue, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_SPEED_ACCELERATION_VALUE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command defines the deceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
  *           .The method refers to the Uart Write command: 0x2B
  * @param[in] speedDecelerationValue  a float value [Rev/S^2]
  * @retval bool 0 fail / 1 for success
  */
 bool SetSpeedDecelerationValue(SOLOMotorControllersUart* solo_uart, float speedDecelerationValue)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetSpeedDecelerationValueInputValidation(speedDecelerationValue, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(speedDecelerationValue, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_SPEED_DECELERATION_VALUE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command sets the baud rate of CAN bus in CANOpen network
  *           .The method refers to the Uart Write command: 0x2C
  * @param[in] canbusBaudrate  enum that specify the baud rate of CAN bus in CANOpen network
  * @retval bool 0 fail / 1 for success
  */
 bool SetCanbusBaudrate(SOLOMotorControllersUart* solo_uart, CanbusBaudrate canbusBaudrate)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
 
   unsigned char data[4];
   ConvertLongToData((long)canbusBaudrate, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_UART_BAUDRATE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command defines the resolution of the speed at S/T input
  *           while SOLO operates in Analogue mode
  *           .The method refers to the Uart Write command: 0x2D
  * @param[in] divisionCoefficient  a long value
  * @retval bool 0 fail / 1 for success
  */
 bool SetAnalogueSpeedResolutionDivisionCoefficient(SOLOMotorControllersUart* solo_uart, long divisionCoefficient)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetAnalogueSpeedResolutionDivisionCoefficientInputValidation(divisionCoefficient, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertLongToData(divisionCoefficient, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_ASRDC, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command defines the type of the Motion Profile that is
  *           being used in Speed or Position Modes
  *           .The method refers to the Uart Write command: 0x30
  * @param[in] motionProfileMode enum that specify the type of the Motion Profile
  * @retval bool 0 fail / 1 for success
  */
 bool SetMotionProfileMode(SOLOMotorControllersUart* solo_uart, MotionProfileMode motionProfileMode)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
 
   unsigned char data[4];
   ConvertLongToData((long)motionProfileMode, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_MOTION_PROFILE_MODE, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x31
  * @param[in] MotionProfileVariable1 a float value
  * @retval bool 0 fail / 1 for success
  */
 bool SetMotionProfileVariable1(SOLOMotorControllersUart* solo_uart, float MotionProfileVariable1)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetMotionProfileVariable1InputValidation(MotionProfileVariable1, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(MotionProfileVariable1, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_MOTION_PROFILE_VARIABLE1, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x32
  * @param[in] MotionProfileVariable2 a float value
  * @retval bool 0 fail / 1 for success
  */
 bool SetMotionProfileVariable2(SOLOMotorControllersUart* solo_uart, float MotionProfileVariable2)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetMotionProfileVariable2InputValidation(MotionProfileVariable2, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(MotionProfileVariable2, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_MOTION_PROFILE_VARIABLE2, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x33
  * @param[in] MotionProfileVariable3 a float value
  * @retval bool 0 fail / 1 for success
  */
 bool SetMotionProfileVariable3(SOLOMotorControllersUart* solo_uart, float MotionProfileVariable3)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetMotionProfileVariable3InputValidation(MotionProfileVariable3, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(MotionProfileVariable3, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_MOTION_PROFILE_VARIABLE3, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x34
  * @param[in] MotionProfileVariable4 a float value
  * @retval bool 0 fail / 1 for success
  */
 bool SetMotionProfileVariable4(SOLOMotorControllersUart* solo_uart, float MotionProfileVariable4)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetMotionProfileVariable4InputValidation(MotionProfileVariable4, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(MotionProfileVariable4, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_MOTION_PROFILE_VARIABLE4, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x35
  * @param[in] MotionProfileVariable5 a float value
  * @retval bool 0 fail / 1 for success
  */
 bool SetMotionProfileVariable5(SOLOMotorControllersUart* solo_uart, float MotionProfileVariable5)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetMotionProfileVariable5InputValidation(MotionProfileVariable5, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(MotionProfileVariable5, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_MOTION_PROFILE_VARIABLE5, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command Set the Digiatal Ouput pin Status. The method refers to the Uart Write command: 0x38
  * @param[in] channel	@ref Channel
  * @param[in] state   @ref DigitalIoState
  * @retval bool 0 fail / 1 for success
  */
 bool SetDigitalOutputState(SOLOMotorControllersUart* solo_uart, Channel channel, DigitalIoState state)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char data[4];
   long lastOutRegister;
 
   lastOutRegister = GetDigitalOutputsRegister(solo_uart);
   if (solo_uart->error == 0)
     return solo_uart->error;
 
   if (state == 1)
     lastOutRegister = lastOutRegister | (1 << channel);
   else
     lastOutRegister = lastOutRegister & (~(1 << channel));
 
   ConvertLongToData(lastOutRegister, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_DIGITAL_OUTPUTS_REGISTER, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This command defines the maximum allowed regeneration current sent back from the Motor to
  *				the Power Supply during decelerations
  *           .The method refers to the Uart Write command: 0x39
  * @param[in]  current a float value
  * @retval bool 0 fail / 1 for success
  */
 bool SetRegenerationCurrentLimit(SOLOMotorControllersUart* solo_uart, float current)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetRegenerationCurrentLimitValidation(current, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertFloatToData(current, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_REGENERATION_CURRENT_LIMIT, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 
 /**
  * @brief  This value defines the the sampling window of qualification digital filter applied to the output of
  *			the position sensor before being processed by DSP
  *           .The method refers to the Uart Write command: 0x3A
  * @param[in]  level a long value
  * @param[out]  solo_uart->error   pointer to an integer that specify result of function
  * @retval bool 0 fail / 1 for success
  */
 bool SetPositionSensorDigitalFilterLevel(SOLOMotorControllersUart* solo_uart, long level)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!SetPositionSensorDigitalFilterLevelValidation(level, &solo_uart->error))
   {
     return false;
   }
 
   unsigned char data[4];
   ConvertLongToData(level, data);
 
   unsigned char cmd[] = {solo_uart->addr, WRITE_POSITION_SENSOR_DIGITAL_FILTER_LEVEL, data[0], data[1], data[2], data[3]};
   return ExeCMD(solo_uart, cmd);
 }
 //----------Read----------
 
 /**
  * @brief  This command reads the device address connected on the line
  *           .The method refers to the Uart Read command: 0x81
  
  * @retval long device address connected on the line
  */
 long GetDeviceAddress(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {0xFF, READ_DEVICE_ADDRESS, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the phase-A voltage of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors
  *           .The method refers to the Uart Read command: 0x82
  
  * @retval float phase-A voltage of the motor [Volts]
  */
 float GetPhaseAVoltage(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_PHASE_A_VOLTAGE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the phase-B voltage of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors
  *           .The method refers to the Uart Read command: 0x83
  
  * @retval float 0 phase-A voltage of the motor [Volts]
  */
 float GetPhaseBVoltage(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_PHASE_B_VOLTAGE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the phase-A current of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors
  *           .The method refers to the Uart Read command: 0x84
  
  * @retval phase-A current of the motor [Amps]
  */
 float GetPhaseACurrent(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_PHASE_A_CURRENT, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the phase-B current of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors
  *           .The method refers to the Uart Read command: 0x85
  
  * @retval float phase-B current of the motor [Amps]
  */
 float GetPhaseBCurrent(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_PHASE_B_CURRENT, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the input BUS voltage
  *           .The method refers to the Uart Read command: 0x86
  
  * @retval float  BUS voltage [Volts]
  */
 // Battery Voltage
 float GetBusVoltage(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_BUS_VOLTAGE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the current inside the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO
  *           .The method refers to the Uart Read command: 0x87
  
  * @retval float between [Amps]
  */
 float GetDcMotorCurrentIm(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_DC_MOTOR_CURRENT_IM, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the voltage of the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO
  *           .The method refers to the Uart Read command: 0x88
  
  * @retval float [Volts]
  */
 float GetDcMotorVoltageVm(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_DC_MOTOR_VOLTAGE_VM, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of the Speed controller Kp gain,
  *         set for Digital mode operations
  *           .The method refers to the Uart Read command: 0x89
  
  * @retval float between 0 to 16000
  */
 float GetSpeedControllerKp(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_SPEED_CONTROLLER_KP, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of the Speed controller Ki gain,
  *         set for Digital mode operations
  *           .The method refers to the Uart Read command: 0x8A
  
  * @retval float between 0 to 16000
  */
 float GetSpeedControllerKi(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_SPEED_CONTROLLER_KI, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the output switching frequency of SOLO in Hertz
  *           .The method refers to the Uart Read command: 0x8B
  
  * @retval long [Hz]
  */
 long GetOutputPwmFrequencyKhz(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_OUTPUT_PWM_FREQUENCY_HZ, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data); // PWM reading is in Hz
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of the current limit set for SOLO in
  *         closed-loop digital operation mode
  *           .The method refers to the Uart Read command: 0x8C
  
  * @retval float [Amps]
  */
 float GetCurrentLimit(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_CURRENT_LIMIT, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the actual monetary value of “Iq” that is
  *         the current acts in torque generation in FOC mode for 3-phase motors
  *           .The method refers to the Uart Read command: 0x8D
  
  * @retval float [Amps]
  */
 float GetQuadratureCurrentIqFeedback(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_QUADRATURE_CURRENT_IQ_FEEDBACK, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the actual monetary value of Id that is the
  *         direct current acting in FOC
  *           .The method refers to the Uart Read command: 0x8E
  
  * @retval float [Amps]
  */
 float GetMagnetizingCurrentIdFeedback(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_MAGNETIZING_CURRENT_ID_FEEDBACK, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the number of Poles set for 3-phase motors
  *           .The method refers to the Uart Read command: 0x8F
  
  * @retval long between 1 to 254
  */
 long GetMotorPolesCounts(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_MOTOR_POLES_COUNTS, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the number of physical Incremental encoder lines set on SOLO
  *           .The method refers to the Uart Read command: 0x90
  
  * @retval long between 1 to 200000
  */
 long GetIncrementalEncoderLines(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_INCREMENTAL_ENCODER_LINES, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the amount of value set for Current controller
  *         Kp or proportional gain
  *           .The method refers to the Uart Read command: 0x91
  
  * @retval float between 0 to 16000
  */
 float GetCurrentControllerKp(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_CURRENT_CONTROLLER_KP, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the amount of value set for Current controller
  *         Ki or integrator gain
  *           .The method refers to the Uart Read command: 0x92
  
  * @retval float between 0 to 16000
  */
 float GetCurrentControllerKi(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_CURRENT_CONTROLLER_KI, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the momentary temperature of the board in centigrade
  *           .The method refers to the Uart Read command: 0x93
  * @retval float [°C]
  */
 float GetBoardTemperature(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_BOARD_TEMPERATURE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the Phase or Armature resistance of
  *         the 3-phase or DC brushed motor connected to SOLO respectively
  *           .The method refers to the Uart Read command: 0x94
  * @retval float [Ohms]
  */
 float GetMotorResistance(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_MOTOR_RESISTANCE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the Phase or Armature Inductance of
  *         the 3-phase or DC brushed motor connected to SOLO respectively
  *           .The method refers to the Uart Read command: 0x95
  * @retval float [Henry]
  */
 float GetMotorInductance(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_MOTOR_INDUCTANCE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
   * @brief  his command reads the actual speed of the motor measured or estimated by SOLO in
             sensorless or sensor-based modes respectively
               .The method refers to the Uart Read command: 0x96
   * @retval long [RPM]
   */
 long GetSpeedFeedback(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_SPEED_FEEDBACK, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the Motor type selected for Digital or Analogue mode operations
  *           .The method refers to the Uart Read command: 0x97
  * @retval long between 0 to 3
  */
 MotorType GetMotorType(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_MOTOR_TYPE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return (MotorType)ConvertToLong(data);
   }
   return MOTOR_TYPE_ERROR;
 }
 
 /**
  * @brief  This command reads the feedback control mode selected on SOLO both
  *         for Analogue and Digital operations
  *           .The method refers to the Uart Read command: 0x99
  * @retval long between 0 to 2
  */
 FeedbackControlMode GetFeedbackControlMode(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_FEEDBACK_CONTROL_MODE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return (FeedbackControlMode)ConvertToLong(data);
   }
   return FEEDBACK_CONTROL_MODE_ERROR;
 }
 
 /**
  * @brief  This command reads the actual commanding mode that SOLO is operating
  *           .The method refers to the Uart Read command: 0x9A
  * @retval long between 0 or 1
  */
 CommandMode GetCommandMode(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_COMMAND_MODE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return (CommandMode)ConvertToLong(data);
   }
   return COMMAND_MODE_ERROR;
 }
 
 /**
  * @brief  This command reads the Control Mode type in terms of Torque,
  *         Speed or Position in both Digital and Analogue modes
  *           .The method refers to the Uart Read command: 0x9B
  * @retval long between 0 to 2
  */
 ControlMode GetControlMode(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_CONTROL_MODE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return (ControlMode)ConvertToLong(data);
   }
   return CONTROL_MODE_ERROR;
   ;
 }
 
 /**
  * @brief  This command reads the value of the speed limit set on SOLO
  *           .The method refers to the Uart Read command: 0x9C
  * @retval long [RPM]
  */
 long GetSpeedLimit(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_SPEED_LIMIT, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the amount of value set for Position
  *         controller Kp or proportional gain
  *           .The method refers to the Uart Read command: 0x9D
  * @retval float between 0 to 16000
  */
 float GetPositionControllerKp(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_POSITION_CONTROLLER_KP, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the amount of value set for Position
  *         controller Ki or integrator gain
  *           .The method refers to the Uart Read command: 0x9E
  * @retval float between 0 to 16000
  */
 float GetPositionControllerKi(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_POSITION_CONTROLLER_KI, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the number of counted pulses from the
  *         Incremental Encoder or Hall sensors
  *           .The method refers to the Uart Read command: 0xA0
  * @retval long [Quad-Pulses]
  */
 long GetPositionCountsFeedback(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_POSITION_COUNTS_FEEDBACK, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the solo_uart->error register which is a 32 bit register with
  *         each bit corresponding to specific errors
  *           .The method refers to the Uart Read command: 0xA1
  * @retval long
  */
 long GetErrorRegister(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_ERROR_REGISTER, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the Firmware version existing currently on the SOLO unit
  *           .The method refers to the Uart Read command: 0xA2
  * @retval long
  */
 long GetDeviceFirmwareVersion(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_DEVICE_FIRMWARE_VERSION, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the Hardware version of the SOLO unit connected
  *           .The method refers to the Uart Read command: 0xA3
  * @retval long
  */
 long GetDeviceHardwareVersion(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_DEVICE_HARDWARE_VERSION, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the amount of desired Torque reference (Iq or IM)
  *         already set for the Motor to follow in Digital Closed-loop Torque control mode
  *           .The method refers to the Uart Read command: 0xA4
  * @retval float [Amps]
  */
 float GetTorqueReferenceIq(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_TORQUE_REFERENCE_IQ, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the amount of desired Speed reference already set for
  *         the Motor to follow in Digital Closed-loop Speed control mode
  *           .The method refers to the Uart Read command: 0xA5
  * @retval long [RPM]
  */
 long GetSpeedReference(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_SPEED_REFERENCE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the amount of desired Id (direct current) or
  *         Magnetizing current reference already set for the Motor to follow
  *         in Digital Closed-loop Speed control mode for ACIM motors
  *           .The method refers to the Uart Read command: 0xA6
  * @retval float [Amps]
  */
 float GetMagnetizingCurrentIdReference(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_MAGNETIZING_CURRENT_ID_REFERENCE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the desired position reference set for the Motor
  *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses
  *           .The method refers to the Uart Read command: 0xA7
  * @retval long [Quad-Pulses]
  */
 long GetPositionReference(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_POSITION_REFERENCE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the desired Power reference for SOLO to apply in
  *         Digital Open-loop speed control mode for 3-phase motors in terms of percentage
  *           .The method refers to the Uart Read command: 0xA8
  * @retval float [%]
  */
 float GetPowerReference(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_POWER_REFERENCE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This commands reads the desired direction of rotation set for the Motor
  *           .The method refers to the Uart Read command: 0xA9
  * @retval long 0 Counter ClockWise / 1 ClockWise
  */
 Direction GetMotorDirection(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_MOTOR_DIRECTION, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return (Direction)ConvertToLong(data);
   }
   return DIRECTION_ERROR;
 }
 
 /**
  * @brief  This command reads the value of Sensorless Zero Speed Full Torque Injection Amplitude
  *           .The method refers to the Uart Read command: 0xAA
  * @retval float between 0.0 to 0.55
  */
 float GetZsftInjectionAmplitude(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_ZSFT_INJECTION_AMPLITUDE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of Sensorless Zero Speed Full Torque Polarity Amplitude
  *           .The method refers to the Uart Read command: 0xAB
  * @retval float between 0.0 to 0.55
  */
 float GetZsftPolarityAmplitude(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_ZSFT_POLARITY_AMPLITUDE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of Sensorless Observer Gain for DC Motor
  *           .The method refers to the Uart Read command: 0xAC
  * @retval float between 0.01 to 1000
  */
 float GetObserverGainDc(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_OBSERVER_GAIN_DC, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of Sensorless Zero Speed Full Torque Injection Frequency
  *           .The method refers to the Uart Read command: 0xAD
  * @retval float between 0 to 10
  */
 long GetZsftInjectionFrequency(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_ZSFT_INJECTION_FREQUENCY, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of Sensorless Transition Speed
  *           .The method refers to the Uart Read command: 0xAE
  * @retval long between 1 to 5000
  */
 long GetSensorlessTransitionSpeed(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_SENSORLESS_TRANSITION_SPEED, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the measured or estimated per-unit angle of the 3-phase motors
  *           .The method refers to the Uart Read command: 0xB0
  * @retval float [Per Unit]
  */
 float Get3PhaseMotorAngle(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_3_PHASE_MOTOR_ANGLE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction
  *           .The method refers to the Uart Read command: 0xB1
  * @retval float [Per Unit]
  */
 float GetEncoderHallCcwOffset(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_ENCODER_HALL_CCW_OFFSET, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction
  *           .The method refers to the Uart Read command: 0xB2
  * @retval float [Per Unit]
  */
 float GetEncoderHallCwOffset(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_ENCODER_HALL_CW_OFFSET, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line
  *           .The method refers to the Uart Read command: 0xB3
  * @retval long [Bits/s]
  */
 UartBaudrate GetUartBaudrate(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_UART_BAUDRATE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return (UartBaudrate)ConvertToLong(data);
   }
   return UART_BAUDRATE_ERROR;
 }
 
 /**
  * @brief  This command reads the acceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds
  *           .The method refers to the Uart Read command: 0xB4
  * @retval float [Rev/S^2]
  */
 float GetSpeedAccelerationValue(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_SPEED_ACCELERATION_VALUE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the deceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds
  *           .The method refers to the Uart Read command: 0xB5
  * @retval float [Rev/S^2]
  */
 float GetSpeedDecelerationValue(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_SPEED_DECELERATION_VALUE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command Reads the baud rate of CAN bus in CANOpen network
  *           .The method refers to the Uart Read command: 0xB6
  * @retval long [kbits/s]
  */
 long GetCanbusBaudrate(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_CANBUS_BAUDRATE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
  *           .The method refers to the Uart Read command: 0xB7
  * @retval long
  */
 long GetAnalogueSpeedResolutionDivisionCoefficient(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_ASRDC, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This Command reads the number of counted index pulses
  *         seen on the Incremental Encoder’s output
  *           .The method refers to the Uart Read command: 0xB8
  * @retval long [Pulses]
  */
 long GetEncoderIndexCounts(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_ENCODER_INDEX_COUNTS, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command test if the communication is working
  * @retval bool 0 not working / 1 for working
  */
 bool CommunicationIsWorking(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   float temperature = GetBoardTemperature(solo_uart);
   if (solo_uart->error == NO_ERROR_DETECTED)
   {
    printmsg("SOLO temperature: %.2f\r\n", temperature); 
    return true;
   }
   return false;
 }
 
 /**
  * @brief  This command reads the type of the Embedded Motion profile active in the controller
  *           .The method refers to the Uart Read command: 0xBB
  * @retval long
  */
 MotionProfileMode GetMotionProfileMode(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_MOTION_PROFILE_MODE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return (MotionProfileMode)ConvertToLong(data);
   }
   return MOTION_PROFILE_MODE_ERROR;
 }
 
 /**
  * @brief  This command reads the value of the Motion Profile Variable1 set inside the controller
  *           .The method refers to the Uart Read command: 0xBC
  * @retval float
  */
 float GetMotionProfileVariable1(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_MOTION_PROFILE_VARIABLE1, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of the Motion Profile Variable2 set inside the controller
  *           .The method refers to the Uart Read command: 0xBD
  * @retval float
  */
 float GetMotionProfileVariable2(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_MOTION_PROFILE_VARIABLE2, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of the Motion Profile Variable3 set inside the controller
  *           .The method refers to the Uart Read command: 0xBE
  * @retval float
  */
 float GetMotionProfileVariable3(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_MOTION_PROFILE_VARIABLE3, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of the Motion Profile Variable4 set inside the controller
  *           .The method refers to the Uart Read command: 0xBF
  * @retval float
  */
 float GetMotionProfileVariable4(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_MOTION_PROFILE_VARIABLE4, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of the Motion Profile Variable5 set inside the controller
  *           .The method refers to the Uart Read command: 0xC0
  
  * @retval float
  */
 float GetMotionProfileVariable5(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_MOTION_PROFILE_VARIABLE5, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of the Digital Outputs Register as a 32 bits register
  *           .The method refers to the Uart Read command: 0xC4
  * @param[in]  channel  @ref Channel
  * @retval enum @ref DigitalIoState
  */
 DigitalIoState GetDigitalOutputState(SOLOMotorControllersUart* solo_uart, Channel channel)
 {
   long lastOutRegister;
   lastOutRegister = GetDigitalOutputsRegister(solo_uart);
   if (solo_uart->error == NO_ERROR_DETECTED)
     return DIGITAL_IO_STATE_ERROR;
   return (DigitalIoState)((lastOutRegister >> channel) & 0x00000001);
 }
 
 long GetDigitalOutputsRegister(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_DIGITAL_OUTPUT_REGISTER, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the Digiatal Ouput pin Status. The method refers to the Uart Read command: 0xC4
  * @param[out] pinNumber   specify the pin you want to controll. (Ensure your SOLO model support this functions)
  * @retval int
  */
 int GetDigitalOutput(SOLOMotorControllersUart* solo_uart, int pinNumber)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   if (!DigitalInputValidation(pinNumber, &solo_uart->error))
   {
     return -1;
   }
 
   uint8_t informationReceived = (uint8_t)GetDigitalOutputState(solo_uart,(Channel)pinNumber);
   if (solo_uart->error != NO_ERROR_DETECTED)
   {
     return -1;
   }
 
   uint8_t mask = 1 << pinNumber;
   return (informationReceived & mask) != 0;
 }
 
 /**
  * @brief  This command reads the current state of the controller
  *           .The method refers to the Uart Read command: 0xC7
  * @retval enum @ref DisableEnable
  */
 DisableEnable GetDriveDisableEnable(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_DRIVE_DISABLE_ENABLE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return (DisableEnable)ConvertToLong(data);
   }
   return SOLO_DISABLE_ENABLE_ERROR;
 }
 
 /**
  * @brief  This command reads the value of the Regeneration Current Limit
  *           .The method refers to the Uart Read command: 0xC8
  * @retval float
  */
 float GetRegenerationCurrentLimit(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_REGENERATION_CURRENT_LIMIT, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToFloat(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of the Position Sensor Digital Filter Level
  *           .The method refers to the Uart Read command: 0xC9
  * @retval long
  */
 long GetPositionSensorDigitalFilterLevel(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_POSITION_SENSOR_DIGITAL_FILTER_LEVEL, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of the Digital Input Register as a 32 bits register
  *           .The method refers to the Uart Read command: 0xC5
  * @retval long
  */
 long GetDigitalInputRegister(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_DIGITAL_INPUT_REGISTER, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the value of the voltage sensed at the output of PT1000 temperature
  *			sensor amplifier, this command can be used only on devices that come with PT1000 input
  *           .The method refers to the Uart Read command: 0xC3
  * @retval long
  */
 long GetPT1000SensorVoltage(SOLOMotorControllersUart* solo_uart)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_PT1000_SENSOR_VOLTAGE, 0x00, 0x00, 0x00, 0x00};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return ConvertToLong(data);
   }
   return -1;
 }
 
 /**
  * @brief  This command reads the quantized value of an Analogue Input as a number between 0 to 4095
  *				depending on the maximum voltage input possible at the analogue inputs for the controller
  *           .The method refers to the Uart Read command: 0xC6
  * @param[in]  channel  an enum that specify the Channel of Analogue Input
  * @retval enum @ref DigitalIoState
  */
 DigitalIoState GetAnalogueInput(SOLOMotorControllersUart* solo_uart, Channel channel)
 {
   solo_uart->error = NO_PROCESSED_COMMAND;
   unsigned char cmd[] = {solo_uart->addr, READ_ANALOGUE_INPUT, 0x00, 0x00, 0x00, (unsigned char)channel};
 
   if (ExeCMD(solo_uart, cmd))
   {
     unsigned char data[4];
     SplitData(data, cmd);
     return (DigitalIoState)ConvertToLong(data);
   }
   return DIGITAL_IO_STATE_ERROR;
 }
