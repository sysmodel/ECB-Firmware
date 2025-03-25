/**
 *******************************************************************************
 * @file    SOLOMotorControllersUtils.cpp
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the utility common functions
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 *
 * @date    Date: 2024
 * @version 5.4.1
 * *******************************************************************************
 * @attention
 * Copyright: (c) 2021-present, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 * *****************************************************************************
 * @file	SOLOMotorControllersUtils.c
 * 2025/03/25: Convert to C code to be compatible with STM32F7 (QN)
 *******************************************************************************
 */

 #include "SOLOMotorControllersUtils.h"
 
 #include <stdio.h>
 #include <stdint.h>
 #include <string.h>
 #include <math.h>
 
 void ConvertFloatToString(char *buffer, float value, int precision) 
 {
    int int_part = (int)value;
    float fraction = fabsf(value - int_part);

    // Handle sign for negative values
    if (value < 0 && int_part == 0) {
        sprintf(buffer, "-0");
    } else {
        sprintf(buffer, "%d", int_part);
    }

    strcat(buffer, ".");

    for (int i = 0; i < precision; ++i) {
        fraction *= 10.0f;
        int digit = (int)fraction;
        char digit_char[2] = {digit + '0', '\0'};
        strcat(buffer, digit_char);
        fraction -= digit;
    }
 }

 float ConvertToFloat(unsigned char data[])
 {
     float dec = 0;
     dec = (long)data[0] << 24;
     dec += (long)data[1] << 16;
     dec += (long)data[2] << 8;
     dec += (long)data[3];
 
     if (dec <= 0x7FFE0000)
     {
         return (float)dec / 131072.0;
     }
     else
     {
         dec = 0xFFFFFFFF - dec + 1;
         return ((float)dec / 131072.0) * -1;
     }
 }

 void ConvertFloatToData(float f, unsigned char data[])
 {
     long dec = (long)(f * 131072);
     if (dec < 0)
     {
         dec *= -1;
         dec = 0xFFFFFFFF - dec;
     }
     data[0] = dec >> 24;
     dec = dec % 16777216;
     data[1] = dec >> 16;
     dec = dec % 65536;
     data[2] = dec >> 8;
     data[3] = dec % 256;
 }

 long ConvertToLong(unsigned char data[])
 {
     long dec = 0;
     dec = (long)data[0] << 24;
     dec += (long)data[1] << 16;
     dec += (long)data[2] << 8;
     dec += (long)data[3];
 
     if (dec <= 2147483647 /*0x7FFFFFFF*/)
     {
         return dec;
     }
     else
     {
         dec = /*0xFFFFFFFF*/ 4294967295 - dec + 1;
         return dec * -1;
     }
 }

 void ConvertLongToData(long l, unsigned char data[])
 {
     long dec = l;
     if (dec < 0)
     {
         dec *= -1;
         dec = 0xFFFFFFFF - dec + 1;
     }
     data[0] = dec >> 24;
     dec = dec % 16777216;
     data[1] = dec >> 16;
     dec = dec % 65536;
     data[2] = dec >> 8;
     data[3] = dec % 256;
 }

 void SplitData(unsigned char data[], unsigned char cmd[])
 {
     data[0] = cmd[2];
     data[1] = cmd[3];
     data[2] = cmd[4];
     data[3] = cmd[5];
 }
 
 void ExtractData(unsigned char _Data[], unsigned char _ExtractedData[])
 {
 
     _ExtractedData[0] = _Data[7];
     _ExtractedData[1] = _Data[6];
     _ExtractedData[2] = _Data[5];
     _ExtractedData[3] = _Data[4];
 }

 bool SetGuardTimeInputValidation(long guardtime, int *error)
 {
     if (guardtime < 0 || guardtime > 65535)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }

 bool SetLifeTimeFactorInputValidation(long lifeTimeFactor, int *error)
 {
     if (lifeTimeFactor < 0 || lifeTimeFactor > 255)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }

 bool SetProducerHeartbeatTimeInputValidation(long producerHeartbeatTime, int *error)
 {
     if (producerHeartbeatTime < 0 || producerHeartbeatTime > 65535)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetDeviceAddressInputValidation(unsigned char deviceAddress, int *error)
 {
     if (deviceAddress < 0 || deviceAddress > 254)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }

 bool SetCurrentLimitInputValidation(float currentLimit, int *error)
 {
     if (currentLimit < 0 || currentLimit > 200)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }

 bool SetTorqueReferenceIqInputValidation(float torqueReferenceIq, int *error)
 {
     if (torqueReferenceIq < 0 || torqueReferenceIq > 200)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetSpeedReferenceInputValidation(long speedReference, int *error)
 {
     if (speedReference < 0 || speedReference > 200000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetPowerReferenceInputValidation(float powerReference, int *error)
 {
     if (powerReference < 0 || powerReference > 100)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetOutputPwmFrequencyKhzInputValidation(long outputPwmFrequencyKhz, int *error)
 {
     if (outputPwmFrequencyKhz < 8 || outputPwmFrequencyKhz > 80)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetSpeedControllerKpInputValidation(float speedControllerKp, int *error)
 {
     if (speedControllerKp < 0 || speedControllerKp > 300)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetSpeedControllerKiInputValidation(float speedControllerKi, int *error)
 {
     if (speedControllerKi < 0 || speedControllerKi > 300)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetMotorResistanceInputValidation(float motorResistance, int *error)
 {
     if (motorResistance < 0.0 || motorResistance > 100.0)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetMotorInductanceInputValidation(float motorInductance, int *error)
 {
     if (motorInductance < 0.0 || motorInductance > 0.2)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetMotorPolesCountsInputValidation(long motorPolesCounts, int *error)
 {
     if (motorPolesCounts < 1 || motorPolesCounts > 254)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetIncrementalEncoderLinesInputValidation(long incrementalEncoderLines, int *error)
 {
     if (incrementalEncoderLines < 1 || incrementalEncoderLines > 200000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetSpeedLimitInputValidation(long speedLimit, int *error)
 {
     if (speedLimit < 1 || speedLimit > 200000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetCurrentControllerKpInputValidation(float currentControllerKp, int *error)
 {
     if (currentControllerKp < 0 || currentControllerKp > 16000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetCurrentControllerKiInputValidation(float currentControllerKi, int *error)
 {
     if (currentControllerKi < 0 || currentControllerKi > 16000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetMagnetizingCurrentIdReferenceInputValidation(float magnetizingCurrentIdReference, int *error)
 {
     if (magnetizingCurrentIdReference < 0 || magnetizingCurrentIdReference > 200)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetPositionReferenceInputValidation(long positionReference, int *error)
 {
     if (positionReference < -2147483647 || positionReference > 2147483647)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetPositionControllerKpInputValidation(float positionControllerKp, int *error)
 {
     if (positionControllerKp < 0 || positionControllerKp > 16000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetPositionControllerKiInputValidation(float positionControllerKi, int *error)
 {
     if (positionControllerKi < 0 || positionControllerKi > 16000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetZsftInjectionAmplitudeValidation(float amplitude, int *error)
 {
     if (amplitude < 0.0 || amplitude > 0.55)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetZsftPolarityAmplitudeValidation(float amplitude, int *error)
 {
     if (amplitude < 0.0 || amplitude > 0.55)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetObserverGainDcInputValidation(float observerGain, int *error)
 {
     if (observerGain < 0.01 || observerGain > 1000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetZsftInjectionFrequencyInputValidation(long frequency, int *error)
 {
     if (frequency < 0 || frequency > 10)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetSensorlessTransitionSpeedInputValidation(long speed, int *error)
 {
     if (speed < 1 || speed > 5000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetFilterGainBldcPmsmInputValidation(float filterGain, int *error)
 {
     if (filterGain < 0.01 || filterGain > 16000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetFilterGainBldcPmsmUltrafastInputValidation(float filterGain, int *error)
 {
     if (filterGain < 0.01 || filterGain > 16000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetEncoderHallCcwOffsetInputValidation(float encoderHallOffset, int *error)
 {
     if (encoderHallOffset <= 0 || encoderHallOffset >= 1)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetEncoderHallCwOffsetInputValidation(float encoderHallOffset, int *error)
 {
     if (encoderHallOffset <= 0 || encoderHallOffset >= 1)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetSpeedAccelerationValueInputValidation(float speedAccelerationValue, int *error)
 {
     if (speedAccelerationValue < 0 || speedAccelerationValue > 1600)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetSpeedDecelerationValueInputValidation(float speedDecelerationValue, int *error)
 {
     if (speedDecelerationValue < 0 || speedDecelerationValue > 1600)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetAnalogueSpeedResolutionDivisionCoefficientInputValidation(float divisionCoefficient, int *error)
 {
     if (divisionCoefficient < 0.0001 || divisionCoefficient > 10000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetMotionProfileVariable1InputValidation(float MotionProfileVariable1, int *error)
 {
     if (MotionProfileVariable1 < 0 || MotionProfileVariable1 > 16000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetMotionProfileVariable2InputValidation(float MotionProfileVariable2, int *error)
 {
     if (MotionProfileVariable2 < 0 || MotionProfileVariable2 > 16000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetMotionProfileVariable3InputValidation(float MotionProfileVariable3, int *error)
 {
     if (MotionProfileVariable3 < 0 || MotionProfileVariable3 > 16000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetMotionProfileVariable4InputValidation(float MotionProfileVariable4, int *error)
 {
     if (MotionProfileVariable4 < 0 || MotionProfileVariable4 > 16000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetMotionProfileVariable5InputValidation(float MotionProfileVariable5, int *error)
 {
     if (MotionProfileVariable5 < 0 || MotionProfileVariable5 > 16000)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool DigitalInputValidation(int pinNumber, int *error)
 {
     if (pinNumber < 0 || pinNumber > 2)
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
 
 bool SetRegenerationCurrentLimitValidation(float current, int *error)
 {
     if (current < 0 || current > 200) // todo this value must change
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }

 bool SetPositionSensorDigitalFilterLevelValidation(long level, int *error)
 {
     if (level < 0 || level > 255) // todo this value must change
     {
         *error = OUT_OF_RANGE_SETTING;
         return false;
     }
     return true;
 }
