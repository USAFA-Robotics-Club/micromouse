/**
 * @file      Tachometers.cpp
 * @brief     Measuring the speeds and displacements of two DC motors.
 * @details   Runs on an Arduino board for the motors on Texas Instruments Robotics System Learning Kit MAX (TI-RSLK MAX) -  https://www.pololu.com/product/3670.
 * @details   Runs on Arduino Due for two DFRobot FIT0458 DC motors. https://wiki.dfrobot.com/Micro_DC_Motor_with_Encoder-SJ02_SKU__FIT0458.
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

#include <Arduino.h>

#define DEFAULT_PERIOD2RPM  62500   // conversion factor: period in us to rev/min
#define DEFAULT_PERIOD2DPS  375000  // conversion factor: period in us to deg/sec
#define RSLK_PERIOD2DPS     1000000 // conversion factor: period in us to deg/sec

// Conversion factor: 1 revolution per minute = 60 degrees per second.
#define RPM2DPS     60  

// Define the minimum tachometer period.
// At 6V, the maximum unloaded motor speed is 160 RPM,
// which corresponds to approximately 400 microseconds.
// For sensor debouncing, we can ignore tachometer 
// rising/falling edges that occur within a half of 400 microseconds.
#define MIN_TACHO_PERIOD 200     

static char LeftEncoderAPin_;
static char LeftEncoderBPin_;
static char RightEncoderAPin_;
static char RightEncoderBPin_;

// Use 'volatile' for variables modified in interrupt service routines
// Time of the last interrupt
volatile static uint32_t LeftLastInterruptTime_us_;  
volatile static uint32_t RightLastInterruptTime_us_;  

// Period of the last interrupt
volatile static uint32_t LeftTimeInterval_us_;  
volatile static uint32_t RightTimeInterval_us_;

volatile static uint16_t LeftSpeed_dps_;  
volatile static uint16_t RightSpeed_dps_;  

volatile static int32_t LeftEncoderSteps_; 
volatile static int32_t RightEncoderSteps_; 
    
#define TACHBUFF_SIZE 10
volatile static uint32_t LeftSpeedBuffer[TACHBUFF_SIZE];
volatile static uint32_t RightSpeedBuffer[TACHBUFF_SIZE];

volatile static int8_t IsTachoTypeRSLK = 0;

void updateLeftEncoder();
void updateRightEncoder();

/**
 * This function initializes the tachometers, sets the appropriate pins as inputs with pull-up resistors, and attaches interrupts for updating encoder counts.
 */
void Tacho_Init(char leftEncoderAPin, char leftEncoderBPin, char rightEncoderAPin, char rightEncoderBPin) {
  
    IsTachoTypeRSLK = 0;

    LeftEncoderAPin_ = leftEncoderAPin;
    LeftEncoderBPin_ = leftEncoderBPin;
    RightEncoderAPin_ = rightEncoderAPin;
    RightEncoderBPin_ = rightEncoderBPin;

    // Set encoder pins as inputs
    pinMode(LeftEncoderAPin_, INPUT_PULLUP);
    pinMode(LeftEncoderBPin_, INPUT_PULLUP);
    pinMode(RightEncoderAPin_, INPUT_PULLUP);
    pinMode(RightEncoderBPin_, INPUT_PULLUP);

    // Another method to enable internal pull-up resistors
    // after setting the pin mode to input.
    // digitalWrite(LeftEncoderAPin_, HIGH);
    // digitalWrite(LeftEncoderBPin_, HIGH);
    // digitalWrite(RightEncoderAPin_, HIGH);
    // digitalWrite(RightEncoderBPin_, HIGH);

    for (int i = 0; i < TACHBUFF_SIZE; i++) {
        LeftSpeedBuffer[i] = 0;    
        RightSpeedBuffer[i] = 0;   
    }

    attachInterrupt(digitalPinToInterrupt(LeftEncoderAPin_), updateLeftEncoder, FALLING);
    attachInterrupt(digitalPinToInterrupt(RightEncoderAPin_), updateRightEncoder, FALLING);

}

/**
 * This function initializes the tachometers with default pins for the left and right motors.
 */
void Tacho_InitDefault(void) {

    char leftEncoderAPin = 2;
    char leftEncoderBPin = 4;
    char rightEncoderAPin = 5;
    char rightEncoderBPin = 6;

    Tacho_Init(leftEncoderAPin, leftEncoderBPin, rightEncoderAPin, rightEncoderBPin);
    
}

/**
 * Initialize Tachometers for TI-RLSK with the default pins
 */
void Tacho_InitRLSK(void) {

    IsTachoTypeRSLK = 1;

    // Left Encoder A connected to P10.5
    // Right Encoder A connected to P10.4
    // Left Encoder B connected to P5.2 
    // Right Encoder B connected to P5.0

    char leftEncoderAPin = 2;   // P10.5 
    char rightEncoderAPin = 5;  // P10.4
    char leftEncoderBPin = 4;   // P5.2
    char rightEncoderBPin = 6;  // P5.0

    Tacho_Init(leftEncoderAPin, leftEncoderBPin, rightEncoderAPin, rightEncoderBPin);

    attachInterrupt(digitalPinToInterrupt(LeftEncoderAPin_), updateLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(RightEncoderAPin_), updateRightEncoder, RISING);

}


/**
 * Retrieve the current periods of the left and right tachometers.
 */
void Tacho_GetPeriods(uint32_t* leftPeriod_us, uint32_t* rightPeriod_us) {
    *leftPeriod_us = LeftTimeInterval_us_;
    *rightPeriod_us = RightTimeInterval_us_;
}

/**
 * Get Tachometer Speeds in RPM
 */
void Tacho_GetSpeedsRPM(uint16_t* leftSpeed_rpm, uint16_t* rightSpeed_rpm) {
   
   *leftSpeed_rpm = LeftSpeed_dps_/RPM2DPS;
   *rightSpeed_rpm = RightSpeed_dps_/RPM2DPS;
}

/**
 * Get Tachometer Speeds in DPS
 */
void Tacho_GetSpeeds(uint16_t* leftSpeed_dps, uint16_t* rightSpeed_dps) {
   *leftSpeed_dps = LeftSpeed_dps_;
   *rightSpeed_dps = RightSpeed_dps_;
}

/**
 * Get Tachometer Steps
 */
void Tacho_GetSteps(int32_t* leftEncoderSteps, int32_t* rightEncoderSteps) {   
   *leftEncoderSteps = LeftEncoderSteps_;
   *rightEncoderSteps = RightEncoderSteps_;
}


/**
 * @brief Calculate Average
 *
 * Calculates the average value of the LeftSpeedBuffer array.
 *
 * @param data  a 16-bit unsigned numbers.
 * @return The average value of the LeftSpeedBuffer array
 *
 */
uint32_t left_average(uint32_t data) {
    
    static uint8_t index = 0;
    static uint32_t sum = 0; 

    sum = sum + data - LeftSpeedBuffer[index];

    LeftSpeedBuffer[index] = data;
    index = (index + 1) % TACHBUFF_SIZE;

    return sum/TACHBUFF_SIZE;
}

/**
 * @brief Calculate Average
 *
 * Calculates the average value of the RightSpeedBuffer array.
 *
 * @param data  a 16-bit unsigned numbers.
 * @return The average value of the RightSpeedBuffer array
 *
 */
uint32_t right_average(uint32_t data) {
    
    static uint8_t index = 0;
    static uint32_t sum = 0; 

    sum = sum + data - RightSpeedBuffer[index];

    RightSpeedBuffer[index] = data;
    index = (index + 1) % TACHBUFF_SIZE;

    return sum/TACHBUFF_SIZE;
}


/**
 * @brief Interrupt Service Routine to Update Left Encoder Position and Speed
 *
 * This ISR updates the left encoder position and speed based on state changes.
 *
 * Note: This ISR is designed for use with the Arduino platform.
 */
void updateLeftEncoder() {

    // Record the time of the interrupt
    uint32_t now = micros();

    // Read the state of the A and B channels
    int bState = digitalRead(LeftEncoderBPin_);
    int aState = digitalRead(LeftEncoderAPin_);

    // time between two consecutive edges
    uint32_t temp = now - LeftLastInterruptTime_us_;
    
    // debouncer
    if (temp < MIN_TACHO_PERIOD) {
        return;
    } else {
        LeftTimeInterval_us_ = temp;
    }

    // Read the state of the A and B channels
    if (IsTachoTypeRSLK) {
        // Update the encoder position based on the state changes
        LeftEncoderSteps_ = (bState)? LeftEncoderSteps_ + 1: LeftEncoderSteps_ - 1;
        LeftSpeed_dps_ = RSLK_PERIOD2DPS/LeftTimeInterval_us_;
    } else {
        // Update the encoder position based on the state changes
        LeftEncoderSteps_ = (aState == bState)? LeftEncoderSteps_ - 1: LeftEncoderSteps_ + 1;
        LeftSpeed_dps_ = left_average(DEFAULT_PERIOD2DPS/LeftTimeInterval_us_);
    }

    LeftLastInterruptTime_us_ = now;

}

/**
 * @brief Interrupt Service Routine to Update Right Encoder Position and Speed
 *
 * This ISR updates the right encoder position and speed based on state changes.
 *
 * Note: This ISR is designed for use with the Arduino platform.
 */
void updateRightEncoder(){

    // Record the time of the interrupt
    uint32_t now = micros();

    // Read the state of the A and B channels
    int aState = digitalRead(RightEncoderAPin_);
    int bState = digitalRead(RightEncoderBPin_);

    // time between two consecutive edges
    uint32_t temp = now - RightLastInterruptTime_us_;
    
    // debouncer
    if (temp < MIN_TACHO_PERIOD) {
        return;
    } else {
        RightTimeInterval_us_ = temp;
    }

    // Read the state of the A and B channels
    if (IsTachoTypeRSLK) {
        // Update the encoder position based on the state changes
        RightEncoderSteps_ = (bState)? RightEncoderSteps_ + 1: RightEncoderSteps_ - 1;
        RightSpeed_dps_ = RSLK_PERIOD2DPS/RightTimeInterval_us_;
    } else {
        // Update the encoder position based on the state changes
        RightEncoderSteps_ = (aState == bState)? RightEncoderSteps_ + 1: RightEncoderSteps_ - 1;
        RightSpeed_dps_ = right_average(DEFAULT_PERIOD2DPS/RightTimeInterval_us_);
    }

    RightLastInterruptTime_us_ = now;
}