/**
 * @file      Tachometers.h
 * @brief     Measuring the speeds and displacements of two DC motors.
 * @details   Runs on Arduino Due for two DFRobot FIT0458 DC motors. https://wiki.dfrobot.com/Micro_DC_Motor_with_Encoder-SJ02_SKU__FIT0458
 * @author    Stan Baek
 * @affiliation United States Air Force Academy
 * @date      October 7, 2023
 */

/* 
Simplified BSD License (FreeBSD License)
Copyright (c) 2023, Stanley S. Baek, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#ifndef Tachometers_h_
#define Tachometers_h_


/**
 * @brief Initialize Tachometers
 *
 * Initialize the tachometers for the left and right DC motors.
 *
 * @param leftEncoderAPin  Pin for the left motor encoder A.
 * @param leftEncoderBPin  Pin for the left motor encoder B.
 * @param rightEncoderAPin Pin for the right motor encoder A.
 * @param rightEncoderBPin Pin for the right motor encoder B.
 * @return None
 */
void Tacho_Init(char leftEncoderAPin, char leftEncoderBPin, char rightEncoderAPin, char rightEncoderBPin);


/**
 * @brief Initialize Tachometers with the Default Pins
 *
 * Initialize the tachometers for the left and right DC motors using the default pins.
 *
 * @return None
 */
void Tacho_InitDefault(void);


/**
 * @brief Initialize Tachometers for TI-RLSK with the default pins
 *
 * Initialize the tachometers for the left and right DC motors using the default pins.
 *
 * @return None
 */
void Tacho_InitRLSK(void);

  
/**
 * @brief Get Tachometer Periods
 *
 * Retrieve the current periods of the left and right tachometers.
 *
 * @param leftPeriod_us  Pointer to store the period of the left tachometer in microseconds.
 * @param rightPeriod_us Pointer to store the period of the right tachometer in microseconds.
 * @return None
 */
void Tacho_GetPeriods(uint32_t* leftPeriod_us, uint32_t* rightPeriod_us);

/**
 * @brief Get Tachometer Speeds in RPM
 *
 * Retrieve the current speeds of the left and right tachometers in revolutions per minute (RPM).
 *
 * @param leftSpeed_rpm  Pointer to store the speed of the left tachometer in RPM.
 * @param rightSpeed_rpm Pointer to store the speed of the right tachometer in RPM.
 * @return None
 */
void Tacho_GetSpeedsRPM(uint16_t* leftSpeed_rpm, uint16_t* rightSpeed_rpm);
   
/**
 * @brief Get Tachometer Speeds in DPS
 *
 * Retrieve the current speeds of the left and right tachometers in degrees per second (DPS).
 *
 * @param leftSpeed_dps  Pointer to store the speed of the left tachometer in DPS.
 * @param rightSpeed_dps Pointer to store the speed of the right tachometer in DPS.
 * @return None
 */
void Tacho_GetSpeeds(uint16_t* leftSpeed_dps, uint16_t* rightSpeed_dps);

/**
 * @brief Get Tachometer Steps
 *
 * Retrieve the accumulated steps (encoder counts) of the left and right tachometers.
 *
 * @param leftEncoderSteps  Pointer to store the accumulated steps of the left tachometer.
 * @param rightEncoderSteps Pointer to store the accumulated steps of the right tachometer.
 * @return None
 */
void Tacho_GetSteps(int32_t* leftEncoderSteps, int32_t* rightEncoderSteps);

#endif
