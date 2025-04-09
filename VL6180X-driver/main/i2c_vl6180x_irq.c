/**
 * @file i2c_vl6180x_irq.c
 * @brief Implementation of interrupt handling and configuration for VL6180X sensor.
 *
 * This file contains the implementation of interrupt handling and configuration
 * for the VL6180X proximity and ambient light sensor. It includes functions to
 * initialize the device, configure interrupts, and handle interrupt events.
 * 
 * @authors Julian Sanchez
 *          Nelson Parra
 *          Angel Graciano
 * 
 * 
 * @date 09-04-2025
 * @version 1.0
 * 
 * @copyright Copyright (c) RoboCup SISTEMIC 2025 
 * 
 * MIT LICENSE
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/


#include "vl6180x_api.h"

VL6180xDev_t theVL6180xDev;        // Global unique device in use

/**
 * @brief Interrupt handler triggered on the rising edge of VL6180X GPIO1.
 *
 * This function clears the interrupt at the CPU hardware level and sets an event
 * for further processing. No I/O operations are performed in the interrupt handler.
 */
void Sample_InterrupHandler(void) {
    MyDev_ClearInterruptCpu(theVL6180xDev); // Clear interrupt at CPU hardware level
    MyDev_SetEvent(theVL6180xDev);          // Set event for further processing
    // Avoid performing I/O operations in the interrupt handler
}

/**
 * @brief Main interrupt handling and configuration function for VL6180X.
 *
 * This function initializes the VL6180X device, configures its interrupts, and
 * handles interrupt events in different modes (low threshold, high threshold,
 * out-of-window, and new sample ready). It also demonstrates dynamic switching
 * between interrupt modes.
 */
void Sample_Interrupt(void) {
    VL6180x_RangeData_t RangeData;

    MyDev_Init(theVL6180xDev); // Initialize device-specific structure
    MyDev_SetChipEnable(theVL6180xDev); // Enable VL6180X device chip
    MyDev_uSleep(2000); // Sleep for 2ms to allow the device to boot

    VL6180x_InitData(theVL6180xDev);
    VL6180x_FilterSetState(theVL6180xDev, 0); // Disable filtering in continuous mode
    VL6180x_Prepare(theVL6180xDev); // Default VL6180X initialization
    VL6180x_UpscaleSetScaling(theVL6180xDev, 2); // Set scaling to 2 for 0-400mm range
    VL6180x_RangeSetInterMeasPeriod(theVL6180xDev, 100); // Set inter-measurement period

    VL6180x_SetupGPIO1(theVL6180xDev, GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT, INTR_POL_HIGH); // Configure GPIO1
    VL6180x_RangeConfigInterrupt(theVL6180xDev, CONFIG_GPIO_INTERRUPT_LEVEL_LOW); // Configure low threshold interrupt
    VL6180x_RangeSetThresholds(theVL6180xDev, 100, 0, 0); // Set thresholds
    MyDev_EnableInterrupt(theVL6180xDev); // Enable and arm interrupt
    VL6180x_ClearAllInterrupt(theVL6180xDev);
    VL6180x_RangeStartContinuousMode(theVL6180xDev);

    // Handle low threshold interrupt events
    do {
        MyDev_WaitForEvent(theVL6180xDev); // Wait for interrupt trigger
        MyDev_ShowIntr('l');
        MyDev_RestartInterrupt(theVL6180xDev); // Re-arm interrupt
        VL6180x_ClearAllInterrupt(theVL6180xDev);
    } while (!MyDev_UserSayStop(theVL6180xDev));

    // Dynamically switch to high threshold interrupt mode
    VL6180x_SetGroupParamHold(theVL6180xDev, 1);
    VL6180x_RangeConfigInterrupt(theVL6180xDev, CONFIG_GPIO_INTERRUPT_LEVEL_HIGH);
    VL6180x_RangeSetThresholds(theVL6180xDev, 0, 200, 0);
    VL6180x_SetGroupParamHold(theVL6180xDev, 0);
    VL6180x_ClearAllInterrupt(theVL6180xDev);

    // Handle high threshold interrupt events
    do {
        MyDev_WaitForEvent(theVL6180xDev);
        MyDev_ShowIntr('h');
        MyDev_RestartInterrupt(theVL6180xDev);
        VL6180x_RangeClearInterrupt(theVL6180xDev);
    } while (!MyDev_UserSayStop(theVL6180xDev));

    // Dynamically switch to out-of-window threshold interrupt mode
    VL6180x_SetGroupParamHold(theVL6180xDev, 1);
    VL6180x_RangeConfigInterrupt(theVL6180xDev, CONFIG_GPIO_INTERRUPT_OUT_OF_WINDOW);
    VL6180x_RangeSetThresholds(theVL6180xDev, 150, 300, 0);
    VL6180x_SetGroupParamHold(theVL6180xDev, 0);
    VL6180x_ClearAllInterrupt(theVL6180xDev);

    // Handle out-of-window interrupt events
    do {
        MyDev_WaitForEvent(theVL6180xDev);
        MyDev_ShowIntr('o');
        MyDev_RestartInterrupt(theVL6180xDev);
        VL6180x_RangeClearInterrupt(theVL6180xDev);
    } while (!MyDev_UserSayStop(theVL6180xDev));

    // Switch to new sample ready interrupt mode
    VL6180x_SetGroupParamHold(theVL6180xDev, 1);
    VL6180x_RangeConfigInterrupt(theVL6180xDev, CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY);
    VL6180x_SetGroupParamHold(theVL6180xDev, 0);
    VL6180x_RangeClearInterrupt(theVL6180xDev);

    // Handle new sample ready interrupt events
    do {
        MyDev_WaitForEvent(theVL6180xDev);
        VL6180x_RangeGetMeasurement(theVL6180xDev, &RangeData);
        if (RangeData.errorStatus == 0) {
            MyDev_ShowRange(theVL6180xDev, RangeData.range_mm, 0);
        } else {
            MyDev_ShowErr(theVL6180xDev, RangeData.errorStatus);
        }
        MyDev_RestartInterrupt(theVL6180xDev);
        VL6180x_RangeClearInterrupt(theVL6180xDev);
    } while (!MyDev_UserSayStop(theVL6180xDev));

    // Stop continuous mode
    VL6180x_RangeSetSystemMode(theVL6180xDev, MODE_START_STOP | MODE_CONTINUOUS);
}
#