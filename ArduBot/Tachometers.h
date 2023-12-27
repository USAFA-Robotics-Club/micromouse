// Tachometers.c
// Provide mid-level functions that measure the speeds and displancements of
// two DFRobot FIT0458 DC motors.
// Stan Baek
// Dec 3, 2023

#ifndef Tachometers_h_
#define Tachometers_h_


/**
 * Initialize Tachometers
 */
void Tacho_Init(char leftEncoderAPin, char leftEncoderBPin, char rightEncoderAPin, char rightEncoderBPin);


/**
 * Initialize Tachometers with the default pins
 */
void Tacho_InitDefault(void);

/**
 * Initialize Tachometers for the RLSK
 */
void Tacho_InitRLSK(void);

void Tacho_GetPeriods(uint32_t* leftPeriod_us, uint32_t* rightPeriod_us);

void Tacho_GetSpeedsRPM(uint16_t* leftSpeed_rpm, uint16_t* rightSpeed_rpm);
   
void Tacho_GetSpeeds(uint16_t* leftSpeed_dps, uint16_t* rightSpeed_dps);

void Tacho_GetSteps(int32_t* leftEncoderSteps, int32_t* rightEncoderSteps);


#endif
