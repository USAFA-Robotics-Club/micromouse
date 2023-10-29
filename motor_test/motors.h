/**
 * Motors.cpp
 */

#ifndef Motors_h
#define Motors_h

#include "Arduino.h"


class Motors {
  public:

    /**
     * Constructor
     * Initialize pins for the motor shield, which will be
     * used to control the direction of the motors and
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
    void forward(unsigned int left_duty_permille, unsigned int right_duty_permille);

    /**
     * backward
     * Drive the robot backward by running left and
     * right wheels backward with the given duty cycles.
     * Input: left_duty_permille  duty cycle of left wheel (0 to 1000)
     *        rightDuty_permyriad duty cycle of right wheel (0 to 1000)
     * Output: none
    */
    void backward(unsigned int left_duty_permille, unsigned int right_duty_permille);

    /**
     * turn_left
     * Turn the robot to the left by running the left wheel backward
     * and the right wheel forward with the given duty cycles.
     * Input: left_duty_permille  duty cycle of left wheel (0 to 1000)
     *        rightDuty_permyriad duty cycle of right wheel (0 to 1000)
     * Output: none
    */
    void turn_left(unsigned int left_duty_permille, unsigned int right_duty_permille);
    
    /**
     * turn_right
     * Turn the robot to the right by running the left wheel forward
     * and the right wheel backward with the given duty cycles.
     * Input: left_duty_permille  duty cycle of left wheel (0 to 1000)
     *        rightDuty_permyriad duty cycle of right wheel (0 to 1000)
     * Output: none
    */
    void turn_right(unsigned int left_duty_permille, unsigned int right_duty_permille);
    
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
