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

#define DEV_I2C Wire
#define SerialPort Serial

// address we will assign if dual sensor is present
#define TOF_LEFT_ADDRESS    0x52
#define TOF_CENTER_ADDRESS  0x54
#define TOF_RIGHT_ADDRESS   0x56

// set the pins to shutdown
// #define SHT_TOF_LEFT    A1
// #define SHT_TOF_CENTER  A8
// #define SHT_TOF_RIGHT   A5

// Components.
VL53L4CX Left(&Wire, A1);    // I2C, xhut = A1
VL53L4CX Center(&Wire1, A3);   // I2C, xhut = A1
VL53L4CX Right(&Wire, A5);   // I2C, xhut = A1

/* Setup ---------------------------------------------------------------------*/

void setup() {
    // Led.
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialize serial for output.
    Serial.begin(115200);
    Serial.println("Starting...");

    // Initialize I2C bus.
    Wire.begin();
    Left.begin();        
    Left.VL53L4CX_Off();
    Left.InitSensor(0x12);
    Left.VL53L4CX_StartMeasurement();

    Right.begin();        
    Right.VL53L4CX_Off();
    Right.InitSensor(0x62);
    Right.VL53L4CX_StartMeasurement();

    Wire1.begin();
    Center.begin();        
    Center.VL53L4CX_Off();
    Center.InitSensor(0x42);
    Center.VL53L4CX_StartMeasurement();




    //TofLeft.VL53L4CX_Off();

    //Initialize VL53L4CX satellite component.
    // error = TofCenter.InitSensor(TOF_CENTER_ADDRESS);
    // if(error) {
    //     Serial.print(F("Failed to set the center ToF address"));
    //     while(1);
    // } 
    // Serial.println(F("Center ToF sensor has initialized."));


    // TofCenter.VL53L4CX_On();
    // delay(10);
    // error = TofCenter.VL53L4CX_StartMeasurement();
    // if(error) {
    //     Serial.print(F("Failed to start Center measurement"));
    //     //while(1);
    // }
    // Serial.println(F("Center ToF sensors have started measurements."));

    // TofRight.VL53L4CX_On();
    // delay(10);
    // error = TofRight.VL53L4CX_StartMeasurement();
    // if(error) {
    //     Serial.print(F("Failed to start Right measurement"));
    //     while(1);
    // } 
    // Serial.println(F("Right ToF sensors have started measurements."));

}

void loop() {

    VL53L4CX_MultiRangingData_t MultiRangingData;
    VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t NewDataReady = 0;
    int no_of_object_found = 0, j;
    char report[64];
    int status;

    //Serial.println("Left ToF is waiting for data.");
    do {
        status = Left.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
    } while (!NewDataReady);

    //Serial.println("Left ToF is ready for a new data.");

    //Led on
    digitalWrite(LED_BUILTIN, HIGH);

    if ((!status) && (NewDataReady != 0)) {
        status = Left.VL53L4CX_GetMultiRangingData(pMultiRangingData);
        no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
        //snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
        //SerialPort.print(report);
        for (j = 0; j < no_of_object_found; j++) {
            if (j != 0) {
                SerialPort.print("\r\n                               ");
            }
            //SerialPort.print("L status=");
            //SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
            SerialPort.print("L=");
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
            status = Left.VL53L4CX_ClearInterruptAndStartMeasurement();
        }
    }


    do {
        status = Center.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
    } while (!NewDataReady);

    //Led on
    digitalWrite(LED_BUILTIN, HIGH);

    if ((!status) && (NewDataReady != 0)) {
        status = Center.VL53L4CX_GetMultiRangingData(pMultiRangingData);
        no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
        //snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
        //SerialPort.print(report);
        for (j = 0; j < no_of_object_found; j++) {
            if (j != 0) {
            SerialPort.print("\r\n                               ");
            }
            //SerialPort.print("R status=");
            //SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
            SerialPort.print(", C=");
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
            status = Center.VL53L4CX_ClearInterruptAndStartMeasurement();
        }
    }


    do {
        status = Right.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
    } while (!NewDataReady);

    //Led on
    digitalWrite(LED_BUILTIN, HIGH);

    if ((!status) && (NewDataReady != 0)) {
        status = Right.VL53L4CX_GetMultiRangingData(pMultiRangingData);
        no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
        //snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
        //SerialPort.print(report);
        for (j = 0; j < no_of_object_found; j++) {
            if (j != 0) {
            SerialPort.print("\r\n                               ");
            }
            //SerialPort.print("R status=");
            //SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
            SerialPort.print(", R=");
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
            status = Right.VL53L4CX_ClearInterruptAndStartMeasurement();
        }
    }

    SerialPort.println("");
    digitalWrite(LED_BUILTIN, LOW);

    delay(100);
}
