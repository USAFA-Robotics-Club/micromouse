// Include the necessary libraries
#include <Arduino.h>
#include <motors.h>
#include "tc_lib.h"

using namespace arduino_due;

#define CAPTURE_TIME_WINDOW 2500000 // usecs
#define ANALOG_PIN 7
#define ANALOG_VALUE 127 // values in the interval [0,255] 

// capture_tc0 declaration_with_callback
// IMPORTANT: Take into account that for TC0 (TC0 and channel 0) the TIOA0 is
// PB25, which is pin 2 for Arduino DUE, so  the capture pin in  this example
// is pin 2. For the correspondence between all TIOA inputs for the different 
// TC modules, you should consult uC Atmel ATSAM3X8E datasheet in section "36. 
// Timer Counter (TC)", and the Arduino pin mapping for the DUE.

capture_tc0_declaration_with_callback();

auto& capture_pin2=capture_tc0;

#define PWM_PERIOD_PIN_35 10000000 // hundredths of usecs (1e-8 secs)
#define PSEUDO_PERIOD 1000 // msecs

volatile uint32_t period_counter=0;
void callback(void) {
    period_counter++; 
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  // capture_pin2 initialization
  // initialization of capture objects
  capture_pin2.config(
    (PWM_PERIOD_PIN_35/100)<<1,
        capture_tc0_t::DEFAULT_MAX_OVERRUNS,
        callback // <--- provided callback
                        // the callback prototype is void(*)()
    );  

}

void loop() {
  // put your main code here,to run repeatedly:

  delay(PSEUDO_PERIOD);

  uint32_t status,duty,period,pulses;

  auto now=millis();
  Serial.println("=======================================================================");
  Serial.print(now);
  Serial.print(": [PIN 35 -> PIN 2] "); 
  status=capture_pin2.get_duty_period_and_pulses(duty,period,pulses);
  Serial.print("duty: "); 
  Serial.print(
    static_cast<double>(duty)/
    static_cast<double>(capture_pin2.ticks_per_usec()),
    3
  );
  Serial.print(" usecs. period: ");
  Serial.print(
    static_cast<double>(period)/
    static_cast<double>(capture_pin2.ticks_per_usec()),
    3
  );
  Serial.print(" usecs. ");
  if(capture_pin2.is_overrun(status)) Serial.print("[overrun]");
  if(capture_pin2.is_stopped(status)) Serial.print("[stopped]");  
  Serial.println();   
  Serial.print(now);
  Serial.print(": [PIN 35 -> PIN 2] pulses: "); Serial.print(pulses);
  Serial.print(" period counter: "); Serial.print(period_counter);
  Serial.println();   
  Serial.println("=======================================================================");
}
