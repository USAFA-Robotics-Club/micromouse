/**
 * @file      Motors.cpp
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

#include "Arduino.h"
#include "Motors.h"


#define MAX_DUTY_CYCLE  1000

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
Motors::Motors(void) {        
    
    // Motors(3, 11, 12, 13, 9, 8, A0, A1); 

    PwmL = 3;
    PwmR = 11;
    DirL = 12;
    DirR = 13;
    BrakeL = 9;
    BrakeR = 8;  
    SnsL = A0;
    SnsR = A1;
        
    pinMode(PwmL, OUTPUT);
    pinMode(PwmR, OUTPUT);
    pinMode(DirL, OUTPUT);
    pinMode(DirR, OUTPUT);
    pinMode(BrakeL, OUTPUT);
    pinMode(BrakeR, OUTPUT);

}


/**
 * @brief Constructor
 *
 * Initialize pins with the given arguments for the Arduino Motor Shield Rev3.
 */
Motors::Motors(char pwmL_pin, char pwmR_pin, char dirL_pin, char dirR_pin, char brakeL_pin, char brakeR_pin, char snsL_pin, char snsR_pin) {
  
    PwmL = pwmL_pin;
    PwmR = pwmR_pin;
    DirL = dirL_pin;
    DirR = dirR_pin;
    BrakeL = brakeL_pin;
    BrakeR = brakeR_pin;  
    SnsL = snsL_pin;
    SnsR = snsR_pin;
        
    pinMode(PwmL, OUTPUT);
    pinMode(PwmR, OUTPUT);
    pinMode(DirL, OUTPUT);
    pinMode(DirR, OUTPUT);
    pinMode(BrakeL, OUTPUT);
    pinMode(BrakeR, OUTPUT);
}

/* trivial destructor */
Motors::~Motors() {
}


/**
 * Drive the robot forward by running left and right wheels forward with the given duty cycles.
*/
void Motors::forward(uint16_t left_duty_permille, uint16_t right_duty_permille) {
  
    if (left_duty_permille > MAX_DUTY_CYCLE || right_duty_permille > MAX_DUTY_CYCLE) {
        return;
    }

    digitalWrite(BrakeL, LOW);  // no brake 
    digitalWrite(BrakeR, LOW);  // no brake 
    digitalWrite(DirL, HIGH);   // Forward
    digitalWrite(DirR, HIGH);   // Forward
    analogWrite(PwmL, 255*left_duty_permille/1000);  
    analogWrite(PwmR, 255*right_duty_permille/1000); 
}

/**
 * Drive the robot backward by running left and right wheels backward with the given duty cycles.
*/
void Motors::backward(uint16_t left_duty_permille, uint16_t right_duty_permille) {

    if (left_duty_permille > MAX_DUTY_CYCLE || right_duty_permille > MAX_DUTY_CYCLE) {
        return;
    }

    digitalWrite(BrakeL, LOW);  // no brake 
    digitalWrite(BrakeR, LOW);  // no brake 
    digitalWrite(DirL, LOW);   // Backward
    digitalWrite(DirR, LOW);   // Backward
    analogWrite(PwmL, 255*left_duty_permille/1000);  
    analogWrite(PwmR, 255*right_duty_permille/1000); 
}

/**
 * Make the robot turn to the left by moving the left wheel backward and the right wheel forward based on the provided duty cycles.
*/
void Motors::turn_left(uint16_t left_duty_permille, uint16_t right_duty_permille) {

    if (left_duty_permille > MAX_DUTY_CYCLE || right_duty_permille > MAX_DUTY_CYCLE) {
        return;
    }

    digitalWrite(BrakeL, LOW);  // no brake 
    digitalWrite(BrakeR, LOW);  // no brake 
    digitalWrite(DirL, LOW);    // Backward
    digitalWrite(DirR, HIGH);   // Forward
    analogWrite(PwmL, 255*left_duty_permille/1000);  
    analogWrite(PwmR, 255*right_duty_permille/1000); 
}

/**
 * Make the robot turn to the right by moving the left wheel forward and the right wheel backward based on the provided duty cycles.
*/
void Motors::turn_right(uint16_t left_duty_permille, uint16_t right_duty_permille) {

    if (left_duty_permille > MAX_DUTY_CYCLE || right_duty_permille > MAX_DUTY_CYCLE) {
        return;
    }

    digitalWrite(BrakeL, LOW);  // no brake 
    digitalWrite(BrakeR, LOW);  // no brake 
    digitalWrite(DirL, HIGH);   // Backward
    digitalWrite(DirR, LOW);   // Backward
    analogWrite(PwmL, 255*left_duty_permille/1000);  
    analogWrite(PwmR, 255*right_duty_permille/1000); 
}

/**
 * Bring the robot to a coasting state by setting the PWM speed control to a 0% duty cycle.
*/
void Motors::coast(void) {
  
    digitalWrite(BrakeL, LOW);  // no brake 
    digitalWrite(BrakeR, LOW);  // no brake 
    analogWrite(PwmL, 0);  
    analogWrite(PwmR, 0); 
}

/**
 * Bring the robot to a stop by deactivating the motors.
*/
void Motors::brake(void) {
  
    digitalWrite(BrakeR, HIGH);  // brake 
    digitalWrite(BrakeL, HIGH);  // brake 
}

