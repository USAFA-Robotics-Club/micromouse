// ArduBot.ino
// Stan Baek
// Dec 3, 2023

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


// Include the necessary libraries
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
// #include <Adafruit_PCD8544.h>
#include <Adafruit_SSD1306.h>

#include "Motors.h"
#include "Tachometers.h"
#include "tc_lib.h"

using namespace arduino_due;

#define CALLBACK_PERIOD 2000000 // hundreths of usecs. (1e-8 secs.)  => 20ms.
#define LED_RED     49
#define LED_GREEN   51
#define LED_BLUE    53

// Macro for returning a bounded value:
// If X < MIN, set X to MIN.
// If X > MAX, set X to MAX.
// The value of X is constrained to the range [MIN, MAX], ensuring MIN <= X <= MAX.
// Example:
// x = MINMAX(0, 100, x);  // x will be greater than or equal to 0 and less than 100.
#define MINMAX(Min, Max, X) ((X) < (Min) ? (Min) : ( (X) > (Max)? (Max) : (X) ) )

// IMPORTANT NOTE: If the motors are driven at full speed with excessive voltage, they may be easily damaged. To prevent motor damage, set PWMMAX to less than approximately 60%. Once the motor controller is fully developed, you can gradually increase it to about 90% at 7.2 V or 100% at 6.0 V.
#define PWMMIN 0    // Ensure PWM is no less than than 0.
#define PWMMAX 999  // Ensure PWM is no greater than 9999 (99.9%).

// The controller output will be divided by this number inside the Controller, i.e.,
// (Kp * error) / GAIN_DIVIDER.
// This allows for a wide range of real values for Kp, such as 1.01, 2.43, 3.75, ...
// Otherwise, we might be limited to choosing only integers like 1, 2, 3, ...
// Using a floating-point number for Kp could result in a computational speed
// reduction of more than 1000 times.
#define GAIN_DIVIDER  1000


// PI speed controller
// Experimentally determine a value that provides a stable system.
static int32_t Speed_Kp = 1000; // this is equivalent to 10.0 when divided by GAIN_DIVIDER = 100.
static int32_t Speed_Ki = 100; // this is equivalent to 1.0 when divided by GAIN_DIVIDER = 100.

//uint32_t CurrentSpeed_rpm;
int32_t DesiredSpeed_dps = 300;  // 50 RPM

int32_t LeftSpeedError_dps;
int32_t RightSpeedError_dps;
int32_t AccumLeftSpeedError = 0;
int32_t AccumRightSpeedError = 0;


//===============================================================
//                   Global variables
//===============================================================
// If controller is executed multiple times, run LcdUpdate.
uint16_t NumControllerExecuted = 0;

// Do not run Controller if this semaphore is false.
bool IsControllerEnabled = false;

// If actuator is disabled, motors will not run.
// It will be useful when you examine states and sensor readings.
uint8_t IsActuatorEnabled = 0;

// Variables for Tachometers 
uint32_t LeftPeriod_us;
uint32_t RightPeriod_us;
uint16_t LeftSpeed_dps;
uint16_t RightSpeed_dps;
int32_t LeftEncoderSteps;
int32_t RightEncoderSteps;

// P controller for HW17 Early Bird
// PI controller for Lab17
int16_t LeftDuty_permille;
int16_t RightDuty_permille;

//===============================================================
//                       DC Motors
//===============================================================
Motors motors = Motors();   

//===============================================================
//                 Timer for Periodic Tasks
//===============================================================
// action_tc2 declaration
action_tc2_declaration();

struct ctx {
  ctx() { onoff=false; counter=0; }
  bool onoff;
  uint32_t counter;
};

ctx action_ctx;

//===============================================================
//                             LCD
//===============================================================
// OLED for Arduino Due, https://www.adafruit.com/product/326

#define OLED_DC     48 //11
#define OLED_RESET  50 //13
#define OLED_CS     52 //12
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  &SPI, OLED_DC, OLED_RESET, OLED_CS);

void LCDClear(void) {
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC)) {
        Serial.println(F("SSD1306 allocation failed"));
    while(1);   // Don't proceed, stay here forever
    }

    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 24);     // Start at top-left corner
    display.print(F("* USAFA Robot Club! *"));
    display.display();

    delay(1000);    // Pause for 1 second
}


// This is the action called periodically with action_tc0 object
void LcdUpdate(void) {

    display.clearDisplay();
    display.setCursor(0,0); 

    // Motor Speeds
    display.setCursor(0,0); display.print("Spd(dps): ");    
    display.print(LeftSpeed_dps); display.print(" "); display.println(RightSpeed_dps);
    
    // Speed Errors
    display.setCursor(0,8); display.print("SpE(dps): ");    
    display.print(LeftSpeedError_dps); display.print(" "); display.println(RightSpeedError_dps);

    // Accumulated Speed Errors
    display.setCursor(0,16); display.print("AcE(dps): ");    
    display.print(AccumLeftSpeedError); display.print(" "); display.println(AccumRightSpeedError);

    // Duty Cycles
    display.setCursor(0,24); display.print("Duty: ");    
    display.print(LeftDuty_permille); display.print(" "); display.println(RightDuty_permille);

    // Motor Displacements
    display.setCursor(0,32); display.print("Disp:");
    display.print(LeftEncoderSteps); display.print(" "); display.println(RightEncoderSteps);   
    
    // Tachometer periods
    // display.setCursor(0,40); display.print("P(us):");    
    // display.print(LeftPeriod_us); display.print(" "); display.println(RightPeriod_us);

    display.setCursor(0,48); 
    display.print(F("* USAFA Robot Club! *"));
    display.print(F("*** USAFA ArduBot ***"));

    display.display();
}



//===============================================================
//                  Data Collection
//===============================================================
// #define BUFFER_SIZE  2000 // for 20 second data

// static uint16_t BufferIndex = 0;
// static uint32_t LeftPeriodBuffer[BUFFER_SIZE];
// static uint32_t RightPeriodBuffer[BUFFER_SIZE];
// static uint32_t LeftSpeedBuffer[BUFFER_SIZE];
// static uint32_t RightSpeedBuffer[BUFFER_SIZE];
// static int32_t LeftStepsBuffer[BUFFER_SIZE];
// static int32_t RightStepsBuffer[BUFFER_SIZE];


// this controller is invoked by timer interrupts at 50 Hz
void controller(void* a_ctx) {

    static uint16_t time_20ms = 0;  // 50 Hz

    // Increment the number of controller executed.
    NumControllerExecuted++;

    // Heartbeat
    if (time_20ms  == 25) {  // every 0.5 sec
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_GREEN, LOW);
    } else if (time_20ms == 0) { // every 1 sec
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_GREEN, HIGH);
    }

    Tacho_GetPeriods(&LeftPeriod_us, &RightPeriod_us);
    Tacho_GetSpeeds(&LeftSpeed_dps, &RightSpeed_dps);
    Tacho_GetSteps(&LeftEncoderSteps, &RightEncoderSteps);
    
    // Calculate speed errors
    // error = desired speed - actual (measured) speed
    LeftSpeedError_dps = DesiredSpeed_dps - LeftSpeed_dps;
    RightSpeedError_dps = DesiredSpeed_dps - RightSpeed_dps;

    // accumulated errors for left and right.
    AccumLeftSpeedError += LeftSpeedError_dps;
    AccumRightSpeedError += RightSpeedError_dps;

    // P controller for HW17 Early Bird
    // PI controller for Lab17
    LeftDuty_permille = (Speed_Kp*LeftSpeedError_dps + Speed_Ki*AccumLeftSpeedError)/GAIN_DIVIDER;
    RightDuty_permille = (Speed_Kp*RightSpeedError_dps + Speed_Ki*AccumRightSpeedError)/GAIN_DIVIDER;

    // check min/max duty values
    LeftDuty_permille = MINMAX(PWMMIN, PWMMAX, LeftDuty_permille);
    RightDuty_permille = MINMAX(PWMMIN, PWMMAX, RightDuty_permille);

    motors.forward(LeftDuty_permille, RightDuty_permille);

    // if (BufferIndex < BUFFER_SIZE) {
    //     LeftPeriodBuffer[BufferIndex] = leftPeriod_us;
    //     RightPeriodBuffer[BufferIndex] = rightPeriod_us;
    //     LeftSpeedBuffer[BufferIndex] = leftSpeed_rpm;
    //     RightSpeedBuffer[BufferIndex] = rightSpeed_rpm;
    //     LeftStepsBuffer[BufferIndex] = leftEncoderSteps;
    //     RightStepsBuffer[BufferIndex] = rightEncoderSteps;
    //     BufferIndex++;
    // }

    time_20ms = (time_20ms + 1) % 50;
}

void setup() {

    // Initialize Serial communication
    Serial.begin(115200);
    motors.brake();
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    
    digitalWrite(LED_BUILTIN, HIGH);
        
    LCDClear();

    AccumLeftSpeedError = 0;
    AccumRightSpeedError = 0;

    Tacho_InitDefault();   
    action_tc2.start(CALLBACK_PERIOD, controller, &action_ctx);

    NumControllerExecuted = 0;

}

void loop() {

    // Controller is running at 50 Hz
    // Update LCD at 5 Hz
    if ((NumControllerExecuted % 10) == 0) {  
        NumControllerExecuted = 0;
        LcdUpdate();
    }
    
}

