/**
 ******************************************************************************
 * @file    VL53L4CX_Sat_HelloWorld.ino
 * @author  SRA
 * @version V1.0.0
 * @date    16 March 2022
 * @brief   Arduino test application for the STMicrolectronics VL53L4CX
 *          proximity sensor satellite based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2022 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/*
 * To use this sketch you need to connect the VL53L4CD satellite sensor directly to the Nucleo board with wires in this way:
 * pin 1 (GND) of the VL53L4CD satellite connected to GND of the Nucleo board
 * pin 2 (VDD) of the VL53L4CD satellite connected to 3V3 pin of the Nucleo board
 * pin 3 (SCL) of the VL53L4CD satellite connected to pin D15 (SCL) of the Nucleo board
 * pin 4 (SDA) of the VL53L4CD satellite connected to pin D14 (SDA) of the Nucleo board
 * pin 5 (GPIO1) of the VL53L4CD satellite connected to pin A2 of the Nucleo board
 * pin 6 (XSHUT) of the VL53L4CD satellite connected to pin A1 of the Nucleo board
 */
/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include "DistanceSensors.h"

#define DEV_I2C Wire
#define SerialPort Serial

// address we will assign if dual sensor is present
#define TOF_LEFT_ADDRESS    0x52
#define TOF_RIGHT_ADDRESS   0x62

// set the pins to shutdown
#define SHT_TOF_LEFT    A1
#define SHT_TOF_RIGHT   A3

// Components.
// VL53L4CX TofLeft(&Wire, A1);    // I2C, xhut = A1
// VL53L4CX TofRight(&Wire, A3);   // I2C, xhut = A1


DistanceSensors dist = DistanceSensors();

/* Setup ---------------------------------------------------------------------*/
void setup() {

    // Initialize serial for output.
    Serial.begin(115200);
    Serial.println("Starting...");
    dist.Init();

}

/*
void setup2() {
    // Led.
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialize serial for output.
    SerialPort.begin(115200);
    SerialPort.println("Starting...");

    // Initialize I2C bus.
    DEV_I2C.begin();
    Serial.println(F("I2C has started."));

    VL53L4CX_Error error;

    // Configure VL53L4CX satellite component.
    error = TofLeft.begin();
    if(error) {
        Serial.print(F("Failed to begin the left ToF."));
        while(1);
    } else {
        Serial.println(F("Left ToF sensor has started."));
    }

    // Switch off VL53L4CX satellite component.
    TofLeft.VL53L4CX_Off();
    Serial.println(F("Left ToF sensor has turned off."));

    error = TofRight.begin();
    if(error) {
        Serial.print(F("Failed to begin the right ToF."));
        while(1);
    } else {
        Serial.println(F("Right ToF sensor has started."));        
    }

    TofRight.VL53L4CX_Off();
    
    //Initialize VL53L4CX satellite component.
    error = TofLeft.InitSensor(TOF_LEFT_ADDRESS);
    if(error) {
        Serial.print(F("Failed to set the left ToF address"));
        while(1);
    } 
    Serial.println(F("ToF sensors has initialized."));

    TofLeft.VL53L4CX_Off();

    error = TofRight.InitSensor(TOF_RIGHT_ADDRESS);
    if(error) {
        Serial.print(F("Failed to set the right ToF address"));
        while(1);
    } 

    TofLeft.VL53L4CX_On();
    error = TofLeft.VL53L4CX_StartMeasurement();
    if(error) {
        Serial.print(F("Failed to start measurement"));
        while(1);
    }
    //TofRight.VL53L4CX_StartMeasurement();
    Serial.println(F("ToF sensors has started measurements."));


    error = TofRight.VL53L4CX_StartMeasurement();
    if(error) {
        Serial.print(F("Failed to start measurement"));
        while(1);
    } 
}
*/

void loop() {

    int16_t left, center, right;

    dist.GetDistances(left, center, right);

    Serial.print("L=");
    Serial.print(left);
    Serial.print("mm C=");
    Serial.print(center);
    Serial.print("mm R=");
    Serial.print(right);
    Serial.println("mm");
    delay(100);

}


/*
void loop2() {

    VL53L4CX_MultiRangingData_t MultiRangingData;
    VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t NewDataReady = 0;
    int no_of_object_found = 0, j;
    char report[64];
    int status;

    //Serial.println("Left ToF is waiting for data.");
    do {
        status = TofLeft.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
    } while (!NewDataReady);

    //Serial.println("Left ToF is ready for a new data.");

    //Led on
    digitalWrite(LED_BUILTIN, HIGH);

    if ((!status) && (NewDataReady != 0)) {
        status = TofLeft.VL53L4CX_GetMultiRangingData(pMultiRangingData);
        no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
        //snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
        //SerialPort.print(report);
        for (j = 0; j < no_of_object_found; j++) {
            if (j != 0) {
                SerialPort.print("\r\n                               ");
            }
            //SerialPort.print("status=");
            //SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
            SerialPort.print("L=");
            SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
            SerialPort.print("mm, ");
            //SerialPort.print(", Signal=");
            //SerialPort.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0);
            //SerialPort.print(" Mcps, Ambient=");
            //SerialPort.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0);
            //SerialPort.print(" Mcps");
        }
        // SerialPort.println("");
        if (status == 0) {
            status = TofLeft.VL53L4CX_ClearInterruptAndStartMeasurement();
        }
    }


    do {
        status = TofRight.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
    } while (!NewDataReady);

    //Led on
    digitalWrite(LED_BUILTIN, HIGH);

    if ((!status) && (NewDataReady != 0)) {
        status = TofRight.VL53L4CX_GetMultiRangingData(pMultiRangingData);
        no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
        //snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
        //SerialPort.print(report);
        for (j = 0; j < no_of_object_found; j++) {
            if (j != 0) {
            SerialPort.print("\r\n                               ");
            }
            // SerialPort.print("status=");
            // SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
            SerialPort.print("R=");
            SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
            SerialPort.print("mm");
            //SerialPort.print(", Signal=");
            //SerialPort.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0);
            //SerialPort.print(" Mcps, Ambient=");
            //SerialPort.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0);
            //SerialPort.print(" Mcps");
        }
        // SerialPort.println("");
        if (status == 0) {
            status = TofRight.VL53L4CX_ClearInterruptAndStartMeasurement();
        }
    }

    SerialPort.println("");
    digitalWrite(LED_BUILTIN, LOW);

    delay(100);
}
*/