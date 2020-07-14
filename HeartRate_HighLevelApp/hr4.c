/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <sys/time.h>

#include <applibs/gpio.h>
#include <applibs/i2c.h>
#include <applibs/log.h>

#include <hw/avnet_mt3620_sk.h>

#include "algorithm.h"
#include "max30102.h"
#include "hr4.h"
#include "oled.h"
#include "build_options.h"

// Track the current state of the HR4 state machine
enum HR4_State currentState;

// Timer variable to manage changing poll time
extern EventLoopTimer* hr4Timer;

#define IR_BUFFER_LEN 500

uint32_t aun_ir_buffer[IR_BUFFER_LEN];   //IR LED sensor data
int32_t n_ir_buffer_length;              //data length
uint32_t aun_red_buffer[IR_BUFFER_LEN];  //Red LED sensor data
int32_t n_sp02;                  //SPO2 value
int8_t ch_spo2_valid;            //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;            //heart rate value
int8_t  ch_hr_valid;             //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;

float median_hr, median_spo2;
uint8_t sampleCount;

static int  i2cFd = 0;
static int  gpioFd= 0;
int         intOccured;

#define MAX30101_SAD 0x57

#define REPORT_MSG "{" \
             "\"HeartRate\":%.2f, " \
             "\"SP02\":%.2f, " \
             "\"Samples\":%ld "  \
             "}"
#define REPORT_MSG_LEN 55
        
#define DEV_MSG "{\"REVISION\":\"%02X\", \"PART\":\"%02X\"}"
#define DEV_MSG_LEN 35

/// <summary>
/// HR4 detect interrupt routine:  Azure Sphere currently (07/2020) does not support hardware interrupts
/// so we must poll the interrupt signal.  The routine performs that function.
/// </summary>
int intDetect(void)
{

    GPIO_Value_Type val;
    if (!gpioFd) {
        gpioFd = GPIO_OpenAsInput(AVNET_MT3620_SK_GPIO2);
    }
    
    int result = GPIO_GetValue(gpioFd, &val);
    if (result != 0) {
        Log_Debug("ERROR: Cluld not read button GPIO: %s (%d).\n", strerror(errno), errno);
        return -1;
    }
    return (int)val;
}
/// <summary>
/// read_i2c:  Read the MAX30101 device using the passed in device address
/// </summary>
int read_i2c( uint8_t addr, uint16_t count, uint8_t* ptr ) 
{
    int r = I2CMaster_WriteThenRead(i2cFd, MAX30101_SAD, &addr, sizeof(addr), ptr, count);
    if (r == -1)
        printf("ERROR: I2CMaster_WriterThenReadSync: errno=%d \n", errno);
    return r;
}

/// <summary>
/// write_i2c:  Write the MAX30101 device using the passed in device address
/// </summary>
void write_i2c( uint8_t addr, uint16_t count, uint8_t* ptr)
{
    uint8_t buff[2];
    buff[0] = addr;
    buff[1] = *ptr;

    int r = I2CMaster_Write(i2cFd, MAX30101_SAD, buff, 2);
    if( r == -1)
        printf("ERROR: I2CMaster_Writer: errno=%d \n", errno);
}

/// <summary>
/// comp:  Compare routine used by QSort
/// </summary>
int comp (const void * elem1, const void * elem2) 
{
    int32_t f = *((int*)elem1);
    int32_t s = *((int*)elem2);
    if (f > s) return  1;
    if (f < s) return -1;
    return 0;
}

/// <summary>
/// hr4_sys_init:  Initialize the HR4.  Reset the device and start to look for a finger present
/// on the sensor.
/// </summary>
void hr4_sys_init(int fd)
{

    i2cFd = fd;

    // If the gpioFD is valid, then we've already initialized the interface, don't do it again
    if (!gpioFd) {
        maxim_max30102_i2c_setup(read_i2c, write_i2c, intDetect);
    }
    
    maxim_max30102_reset();                //resets the MAX30102
    maxim_max30102_read_reg(0, &uch_dummy); //read and clear status register
    maxim_max30102_init();                 //initializes the MAX30102
 
    // Clear the results variables
    median_hr = 0.0;
    median_spo2 = 0.0;

    // Initialize the HR4 state machine.  If the user pressed the button while a test
    // was taking place, this will re-start the test
    currentState = DETECT_FINGER;

    // Update the OLED status
    oled_state = HR4_STATUS;
    update_oled();
}

/// <summary>
/// HR4 timer event:  Process the HR4 algorithm
/// </summary>
void HR4TimerEventHandler(EventLoopTimer* timer)
{
#define NUM_SAMPLES 30
#define SHOW_RESULTS_COUNT 30
#define VALIDATION_DATA_POINTS 5
#define DELTA_SUM_TARGET 3

    static uint8_t showResultsCount;

    static char str[REPORT_MSG_LEN];
    int      i;
    static int32_t  average_hr = 0;
    static int32_t  average_spo2 = 0;
    static int32_t  nbr_readings = 0;

    static int32_t  hr[NUM_SAMPLES], spo2[NUM_SAMPLES];
  
    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        Log_Debug("HR4: ConsumeEventLoopTimerEvent(timer) failed!\n");
        return;
    }
    
    switch (currentState) {
    
    case IDLE:
        Log_Debug("State: IDLE\n");
        break;
    case HR4_CALIBRATE:

        Log_Debug("State: HR2_CALIBRATE\n");

        sampleCount = 0;
        n_ir_buffer_length = IR_BUFFER_LEN; //buffer length of 500 stores 25 seconds of samples running at 100sps

        //read the first 500 samples, and determine the signal range
        for (i = 0; i < n_ir_buffer_length; i++) {

            while (!max30102_data_available())
             { }; // do nothing
          maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i));  //read from MAX30102 FIFO

        }

        //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

        // Check to see if the data is valid.  If not, then reset the device and re-start the test
        if (!ch_hr_valid || !ch_spo2_valid || (n_heart_rate > 160)) {

            Log_Debug("INVALID data!\n");
            maxim_max30102_reset();                //resets the MAX30102
            maxim_max30102_read_reg(0, &uch_dummy); //read and clear status register
            maxim_max30102_init();                 //initializes the MAX30102

            // Clear the results variables
            median_hr = 0.0;
            median_spo2 = 0.0;

            // Initialize the HR4 state machine.  
            currentState = REPOSITION_FINGER;
            return;

        }
        else {
            Log_Debug("Valid data!\n");
            Log_Debug("HR: %d\n", n_heart_rate);
            Log_Debug("SP02: %d\n", n_sp02);

            // We've collected the calibration data, reset the results variables
            average_hr = nbr_readings = 0;
            average_spo2 = 0;

            // Transition to the data collection state and update the OLED
            currentState = DATA_COLLECTION;
            oled_state = HR4_STATUS;
            update_oled();
        }
        break;

    case REPOSITION_FINGER:

        Log_Debug("State: REPOSITION_FINGER\n");

        // Reinitialize the HR4 state machine.
        currentState = DETECT_FINGER;
        
        // Update the OLED status
        oled_state = HR4_STATUS;
        update_oled();
        
        break;

    // We enter/exit this step multiple times.  If we process this without exiting
    // we starve the rest of the thread and the Azure IoT connection will disconnect
    case DATA_COLLECTION:

        i = 0;

        //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
        for (i = 100; i < 500; i++) {
            aun_red_buffer[i - 100] = aun_red_buffer[i];
            aun_ir_buffer[i - 100] = aun_ir_buffer[i];
        }

        //take 100 sets of samples before calculating the heart rate.
        for (i = 400; i < 500; i++) {
            while (!max30102_data_available()) /* wait here */;
            maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i));
        }

        // Process the data       
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
        
        // Validate that the reading are valid
        if (ch_hr_valid && ch_spo2_valid) {

            Log_Debug("State: DATA_COLLECTION[%d]: ", sampleCount);
            Log_Debug("HR=%ld, SpO2=%ld,\n", n_heart_rate, n_sp02);
            average_hr += n_heart_rate;
            average_spo2 += n_sp02;
            hr[nbr_readings] = n_heart_rate;
            spo2[nbr_readings] = n_sp02;
            nbr_readings++;

            // Make sure we have at least VALIDATION_DATA_POINTS readings before checking
            if (nbr_readings > VALIDATION_DATA_POINTS) {

                int deltaSum = 0;
                int intermediateSum = 0;
                float intermediateAverage = 0.0;

                // Look at the last 5 HR samples.  If the the sum of the deltas between adjacent readings
                // is DELTA_SUM_TARGET or less, then call it stable and report the results

                // Calculate the delta between subsequent readings and tally them up
                for (int count = 0; count < VALIDATION_DATA_POINTS; count++) {
                    deltaSum += abs(hr[(nbr_readings-count-1)] - hr[(nbr_readings-count-2)]);
                    
                    // Keep a running sum to calculate the average if the data is good
                    intermediateSum += hr[(nbr_readings-1)-count];
                }

                // If the difference between the last VALIDATION_DATA_POINTS(5) readings is less than or equal to DELTA_SUM_TARGET (3),
                // then declare a stable reading and stop the test.
                if (deltaSum <= DELTA_SUM_TARGET) {
                    
                    // Calculate the average of the last VALIDATION_DATA_POINTS data points
                    intermediateAverage = (float)intermediateSum / (float)VALIDATION_DATA_POINTS;

                    // We found a stable reading!  Set the results variables and transition
                    // to the next state
                    median_hr = intermediateAverage;
                    median_spo2 = n_sp02;
                    currentState = SEND_RESULTS;

                    Log_Debug("\n\n\nFound stable reading: %0.0f\n\n\n", median_hr);
                }
            }
        }
        
        // If the data is invalid, then transition to the Detect Finger state to 
        // inform the user to reposition the finger and to start the process over
        else {
            Log_Debug("Invalid data!  Reposition finger!!\n");

            maxim_max30102_reset();                //resets the MAX30102
            maxim_max30102_read_reg(0, &uch_dummy); //read and clear status register
            maxim_max30102_init();                 //initializes the MAX30102

            // Clear the results variables
            median_hr = 0.0;
            median_spo2 = 0.0;

            // Initialize the HR4 state machine.  
            currentState = REPOSITION_FINGER;

            // Update the OLED status
            oled_state = HR4_STATUS;
            update_oled();

            // That pass failed!  Bail and give it another go . . .
            return;
        }

        // This last case is where we've read NUM_SAMPLES and . . .
        //   1. The data was valid for all samples
        //   2. The check for stable data never passed
        //  
        //  Transition to the SEND_RESULTS state where the median value of the 
        //  results array will be used to send a result.
        //
        //  The SEND_RESULTS state checks to see if median_hr and median_spo2 are 
        //  still 0.0 and if so, calculates the median values.  

        // Check to see if we've got all the samples, if so, transition to the 
        // next state "Send Results"
        if (++sampleCount > NUM_SAMPLES) {
            currentState = SEND_RESULTS;

        }
        break;

    // Calculate the results and send the telemetry data to Azure 
    case SEND_RESULTS:
        Log_Debug("State: SEND_RESULTS\n");

        // Check to see if we found stable values, or if we're going to use the median value
        if (median_hr == 0.0 && median_spo2 == 0.0) {
            
            // Find the median values from the arrays
            qsort(hr, nbr_readings, sizeof(int32_t), comp);
            median_hr = (nbr_readings % 2) ? hr[nbr_readings / 2] : (hr[(nbr_readings - 1) / 2] + hr[nbr_readings / 2]) / 2.0;

            qsort(spo2, nbr_readings, sizeof(int32_t), comp);
            median_spo2 = (nbr_readings % 2) ? spo2[nbr_readings / 2] : (spo2[(nbr_readings - 1) / 2] + spo2[nbr_readings / 2]) / 2.0;

            Log_Debug("\nAverage Blood Oxygen Level = %ld%%\n", average_spo2 / nbr_readings);
            Log_Debug("        Average Heart Rate = %ld BPM\n", average_hr / nbr_readings);
            Log_Debug("Median Blood Oxygen Level = %.0f%%\n", median_spo2);
            Log_Debug("        Median Heart Rate = %.0f BPM\n", median_hr);

        }
        else {
            Log_Debug("\nStable Blood Oxygen Level = %.0f%%\n", median_spo2);
            Log_Debug("         Stable Heart Rate = %.0f BPM\n", median_hr);
        }

        // Construct a JSON telemetry message.  Report the median values as telemetry.
        sprintf(str, REPORT_MSG, median_hr, median_spo2, nbr_readings-1);
        SendJSONTelemetry(str);

        currentState = DISPLAY_RESULTS;
        showResultsCount = SHOW_RESULTS_COUNT;
        maxim_max30102_shut_down(1);

        // Update the OLED status
        oled_state = HR4_STATUS;
        update_oled();
        break;

    case DETECT_FINGER:
        Log_Debug("State: DETECT_FINGER\n");

        // Keep track of each time we check for a finger detect.  If we just loop
        // until we see a finger we'll starve the thread and the Azure connection 
        // will disconnect
        int loopCnt = 10;
        while (!max30102_finger_detected()) {
            if (--loopCnt == 0) {
                return;
            }
        }

        Log_Debug("Finger detected!\n");
        currentState = HR4_CALIBRATE;

        // Update the OLED status
        oled_state = HR4_STATUS;
        update_oled();
        break;

    // This state is used as a parking lot until we've displayed the results
    // on the OLED for SHOW_RESULTS_COUNT seconds
    case DISPLAY_RESULTS:
        Log_Debug("State: DISPLAY_RESULTS[%d]\n", showResultsCount);

        if (--showResultsCount == 0) {

            // If the auto restart flag is set, transition to the detect state
            if (autoRestartHRTest) {
                
                maxim_max30102_reset();                //resets the MAX30102
                maxim_max30102_read_reg(0, &uch_dummy); //read and clear status register
                maxim_max30102_init();                 //initializes the MAX30102

                // Clear the results variables
                median_hr = 0.0;
                median_spo2 = 0.0;

                // Set the next state to the finger detect state
                currentState = DETECT_FINGER;
            }
            // Otherwise go to the idle state to wait for a new test to start
            else {
                currentState = IDLE;
            }

            oled_state = HR4_STATUS;
            update_oled();

        }
        break;
    }
}


