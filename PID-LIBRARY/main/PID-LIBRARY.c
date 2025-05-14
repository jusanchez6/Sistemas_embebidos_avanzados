/**
 * @file PID-LIBRARY.c
 * 
 * @brief Example PID controller implementation for ESP32
 * 
 * This file contains the example implementation of a PID controller for the ESP32 platform.
 * 
 * @authors Julian Sanchez
 *          Angel Graciano
 *          Nelson Parra
 * 
 * 
 * @date 30-04-2025
 * 
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


#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>


#include "pid.h"



#define TAG "PID"


#define PID_KP 1.0f         // Proportional gain
#define PID_KI 0.1f         // Integral gain
#define PID_KD 0.01f        // Derivative gain

#define PID_SAMPLE_TIME 0.1f    // Sample time in seconds (same as the delay in the loop or the alarm timer)


// Function to simulate a sensor reading
float get_sensor_value(void);

/**
 * @brief strcture to hold PID controller parameters
 */
PIDController pid;


void app_main(void)
{
    ESP_LOGI(TAG, "PDI Controller Example");

    // Initialize the PID controller
    esp_err_t ret = PID_Init(&pid, PID_KP, PID_KI, PID_KD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PID controller");
        return;
    }

    // Set the desired setpoint
    PID_SetSetpoint(&pid, 50.0f);

    while (1) {
        // Simulate a measured value (for example, from a sensor)

        float measured_value = get_sensor_value();     // Replace with actual sensor reading

        // Compute the PID control output
        esp_err_t ret = PID_Compute(&pid, measured_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to compute PID control");
            return;
        }

        // Use the control output (for example, to control a motor)
        float control_output = pid.control;
        ESP_LOGI(TAG, "Control Output: %f", control_output);

        // Wait for the next sample time
        vTaskDelay(pdMS_TO_TICKS(PID_SAMPLE_TIME * 1000));
    }


}
