// Motors.c
// Runs on Arduino Motor Shield (https://docs.arduino.cc/hardware/motor-shield-rev3)
// Provide mid-level functions that control two DC motors
// Stan Baek
// October 7, 2023

/* 
Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

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
#include "motors.h"


// Left motor direction connected to Pin 12
// Left motor PWM connected to Pin 3
// Left motor brake connected to Pin 9
// Left motor SNS (current sensing) connected to A0
// Right motor direction connected to Pin 13
// Right motor PWM connected to Pin 11
// Right motor enable connected to Pin 8
// Right motor SNS (current sensing) connected to A1


/**
 * Constructor
 * Initialize pins for the motor shield, which will be
 * used to control the direction of the motors and
 */
Motors::Motors(void) {
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
 * Constructor
 * Initialize pins with the given arguments for the motor shield, 
 * which will be used to control the direction of the motors and
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

/**
 * forward
 * Drive the robot forward by running left and
 * right wheels forward with the given duty cycles.
 * Input: left_duty_permille  duty cycle of left wheel (0 to 1000)
 *        rightDuty_permyriad duty cycle of right wheel (0 to 1000)
 * Output: none
*/
void Motors::forward(unsigned int left_duty_permille, unsigned int right_duty_permille) {
  
    digitalWrite(BrakeL, LOW);  // no brake 
    digitalWrite(BrakeR, LOW);  // no brake 
    digitalWrite(DirL, HIGH);   // Forward
    digitalWrite(DirR, HIGH);   // Forward
    analogWrite(PwmL, 255*left_duty_permille/1000);  
    analogWrite(PwmR, 255*right_duty_permille/1000); 
}

/**
 * backward
 * Drive the robot backward by running left and
 * right wheels backward with the given duty cycles.
 * Input: left_duty_permille  duty cycle of left wheel (0 to 1000)
 *        rightDuty_permyriad duty cycle of right wheel (0 to 1000)
 * Output: none
*/
void Motors::backward(unsigned int left_duty_permille, unsigned int right_duty_permille) {
  
    digitalWrite(BrakeL, LOW);  // no brake 
    digitalWrite(BrakeR, LOW);  // no brake 
    digitalWrite(DirL, LOW);   // Backward
    digitalWrite(DirR, LOW);   // Backward
    analogWrite(PwmL, 255*left_duty_permille/1000);  
    analogWrite(PwmR, 255*right_duty_permille/1000); 
}

/**
 * turn_left
 * Turn the robot to the left by running the left wheel backward
 * and the right wheel forward with the given duty cycles.
 * Input: left_duty_permille  duty cycle of left wheel (0 to 1000)
 *        rightDuty_permyriad duty cycle of right wheel (0 to 1000)
 * Output: none
*/
void Motors::turn_left(unsigned int left_duty_permille, unsigned int right_duty_permille) {
  
    digitalWrite(BrakeL, LOW);  // no brake 
    digitalWrite(BrakeR, LOW);  // no brake 
    digitalWrite(DirL, LOW);    // Backward
    digitalWrite(DirR, HIGH);   // Forward
    analogWrite(PwmL, 255*left_duty_permille/1000);  
    analogWrite(PwmR, 255*right_duty_permille/1000); 
}

/**
 * turn_right
 * Turn the robot to the right by running the left wheel forward
 * and the right wheel backward with the given duty cycles.
 * Input: left_duty_permille  duty cycle of left wheel (0 to 1000)
 *        rightDuty_permyriad duty cycle of right wheel (0 to 1000)
 * Output: none
*/
void Motors::turn_right(unsigned int left_duty_permille, unsigned int right_duty_permille) {
  
    digitalWrite(BrakeL, LOW);  // no brake 
    digitalWrite(BrakeR, LOW);  // no brake 
    digitalWrite(DirL, HIGH);   // Backward
    digitalWrite(DirR, LOW);   // Backward
    analogWrite(PwmL, 255*left_duty_permille/1000);  
    analogWrite(PwmR, 255*right_duty_permille/1000); 
}

/**
 * coast
 * Set the PWM speed control to 0% duty cycle.
 * Input: none
 * Output: none
*/
void Motors::coast(void) {
  
    digitalWrite(BrakeL, LOW);  // no brake 
    digitalWrite(BrakeR, LOW);  // no brake 
    analogWrite(PwmL, 0);  
    analogWrite(PwmR, 0); 
}

/**
 * brake
 * Stop the motors
 * Input: none
 * Output: none
*/
void Motors::brake(void) {
  
    digitalWrite(BrakeL, HIGH);  // no brake 
    digitalWrite(BrakeR, HIGH);  // no brake 
}

/* trivial destructor */
Motors::~Motors() {
}
