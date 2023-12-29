/**
 * @file      Motors.h
 * @brief     Mid-level functions controlling two DC motors.
 * @details   Runs on an Arduino board for Arduino Motor Shield (https://docs.arduino.cc/hardware/motor-shield-rev3).
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
#ifndef Motors_h
#define Motors_h

#include "Arduino.h"

/**
 * How to use:
 * 
 * 1. Declare a motor instance before the setup() function.
 *    It should be a global variable.
 *    Motors motors = Motors();   
 * 
 * 2. Inside the loop, call motor functions.
 *    The function arguments are in permille.
 *    Example usage:
 *    motors.forward(300, 500);   // left = 30%, right = 50%. This will make a left turn.
 *    motors.turn_left(300, 500); // The left motor moves backward, and the right motor moves forward. 
 */
class Motors {
  public:

    /**
     * @brief Constructor for Motors class
     *
     * Initialize pins for the Arduino Motor Shield Rev3.
     * https://store-usa.arduino.cc/products/arduino-motor-shield-rev3?selectedStore=us
     *
     * This constructor uses the following pins:
     * - Left motor direction connected to Pin 12
     * - Left motor PWM connected to Pin 3
     * - Left motor brake connected to Pin 9
     * - Left motor SNS (current sensing) connected to A0
     * - Right motor direction connected to Pin 13
     * - Right motor PWM connected to Pin 11
     * - Right motor enable connected to Pin 8
     * - Right motor SNS (current sensing) connected to A1
     */
    Motors(void);
  
    /**
     * @brief Constructor
     *
     * Initialize pins with the given arguments for the Arduino Motor Shield Rev3.
     */
    Motors(char pwmL_pin, char pwmR_pin, char dirL_pin, char dirR_pin, char brakeL_pin, char brakeR_pin, char snsL_pin, char snsR_pin);

    /**
     * @brief Destructor
     *
     * It does nothing because this class does not manage any dynamic memory (memory allocated with 'new' or 'malloc').
     */
    ~Motors(void);

    /**
     * @brief Drive Forward
     *
     * Move the robot forward by activating the left and right wheels with the specified duty cycles.
     *
     * @param left_duty_permille Duty cycle of the left wheel (0 to 1000).
     * @param right_duty_permille Duty cycle of the right wheel (0 to 1000).
     * @return None
     */
    void forward(uint16_t left_duty_permille, uint16_t right_duty_permille);

    /**
     * @brief Drive Backward
     *
     * Move the robot backward by engaging the left and right wheels in reverse, following the provided duty cycles.
     *
     * @param left_duty_permille Duty cycle of the left wheel (0 to 1000).
     * @param right_duty_permille Duty cycle of the right wheel (0 to 1000).
     * @return None
     */
    void backward(uint16_t left_duty_permille, uint16_t right_duty_permille);


    /**
     * @brief Turn Left
     *
     * Make the robot turn to the left by moving the left wheel backward and the right wheel forward based on the provided duty cycles.
     *
     * @param left_duty_permille Duty cycle of the left wheel (0 to 1000).
     * @param right_duty_permille Duty cycle of the right wheel (0 to 1000).
     * @return None
     */
    void turn_left(uint16_t left_duty_permille, uint16_t right_duty_permille);
    
    /**
     * @brief Turn Right
     *
     * Make the robot turn to the right by moving the left wheel forward and the right wheel backward based on the provided duty cycles.
     *
     * @param left_duty_permille Duty cycle of the left wheel (0 to 1000).
     * @param right_duty_permille Duty cycle of the right wheel (0 to 1000).
     * @return None
     */
    void turn_right(uint16_t left_duty_permille, uint16_t right_duty_permille);
    
    
    /**
     * @brief Coast
     *
     * Bring the robot to a coasting state by setting the PWM speed control to a 0% duty cycle.
     *
     * @param None
     * @return None
     */
    void coast(void);

    /**
     * Brake
     * Bring the robot to a stop by deactivating the motors.
     * 
     * Input: None
     * Output: None
     */
    void brake(void); 

  private:
    
    // internal variables
    char PwmL, PwmR, DirL, DirR, BrakeL, BrakeR, SnsL, SnsR;

};

#endif
