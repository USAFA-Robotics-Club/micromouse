/**
 * Motors.cpp
 */

#ifndef Motors_h
#define Motors_h

#include "Arduino.h"

/**
 * How to use
 * 
 * Declare a motor instance before the setup() function.
 * It should be a global variable.
 * Motors motors = Motors();   
 * 
 * Inside loop, call motor functions.
 * the function arguments are in permille.
 * motors.forward(300, 500);  // left = 30%, right = 50%. This will make a left turn
 * motors.turn_left(300, 500); // left motor will move backward and the right motor will move forward. 
 */
class Motors {
  public:

    /**
     * Default Constructor
     * Initialize pins for the Arduino Motor Shield Rev3, 
     * https://store-usa.arduino.cc/products/arduino-motor-shield-rev3?selectedStore=us
     */ 
    Motors(void);
  
    /**
     * Constructor
     * Initialize pins with the given arguments for the motor shield, 
     * which will be used to control the direction of the motors and
     */
    Motors(char pwmL_pin, char pwmR_pin, char dirL_pin, char dirR_pin, char brakeL_pin, char brakeR_pin, char snsL_pin, char snsR_pin);

    /**
     * Destructor
     */
    ~Motors(void);

    /**
     * forward
     * Drive the robot forward by running left and
     * right wheels forward with the given duty cycles.
     * Input: left_duty_permille  duty cycle of left wheel (0 to 1000)
     *        rightDuty_permyriad duty cycle of right wheel (0 to 1000)
     * Output: none
    */
    void forward(uint16_t left_duty_permille, uint16_t right_duty_permille);

    /**
     * backward
     * Drive the robot backward by running left and
     * right wheels backward with the given duty cycles.
     * Input: left_duty_permille  duty cycle of left wheel (0 to 1000)
     *        rightDuty_permyriad duty cycle of right wheel (0 to 1000)
     * Output: none
    */
    void backward(uint16_t left_duty_permille, uint16_t right_duty_permille);

    /**
     * turn_left
     * Turn the robot to the left by running the left wheel backward
     * and the right wheel forward with the given duty cycles.
     * Input: left_duty_permille  duty cycle of left wheel (0 to 1000)
     *        rightDuty_permyriad duty cycle of right wheel (0 to 1000)
     * Output: none
    */
    void turn_left(uint16_t left_duty_permille, uint16_t right_duty_permille);
    
    /**
     * turn_right
     * Turn the robot to the right by running the left wheel forward
     * and the right wheel backward with the given duty cycles.
     * Input: left_duty_permille  duty cycle of left wheel (0 to 1000)
     *        rightDuty_permyriad duty cycle of right wheel (0 to 1000)
     * Output: none
    */
    void turn_right(uint16_t left_duty_permille, uint16_t right_duty_permille);
    
    /**
     * coast
     * Set the PWM speed control to 0% duty cycle.
     * Input: none
     * Output: none
    */
    void coast(void);

    /**
     * brake
     * Stop the motors
     * Input: none
     * Output: none
    */
    void brake(void); 

  private:
    
    char PwmL, PwmR, DirL, DirR, BrakeL, BrakeR, SnsL, SnsR;

};

#endif
