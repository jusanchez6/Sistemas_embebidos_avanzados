/**
 * @file pid.h
 * 
 * @brief Simple pid implementation header file
 * 
 * This file contains the definitions and function prototypes for a simple PID
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

#ifndef PID_H

#define PID_H

#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>

typedef struct {
    
    float Kp;   // Proportional gain
    float Ki;   // Integral gain
    float Kd;   // Derivative gain

    float setpoint; // Desired value
    float integral; // Integral term
    
    float previous_error; // Previous error for derivative calculation

    float control;         // Output value
} PIDController;


/**
 * @brief Initializes the PID controller with given gains.
 * 
 * @param pid Pointer to the PIDController structure.
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 * 
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t PID_Init(PIDController *pid, float Kp, float Ki, float Kd);

/**
 * @brief Sets the desired setpoint for the PID controller.
 * 
 * @param pid Pointer to the PIDController structure.
 * @param setpoint Desired setpoint value.
 */
void PID_SetSetpoint(PIDController *pid, float setpoint);

/**
 * @brief Computes the PID output based on the measured value.
 * 
 * @param pid Pointer to the PIDController structure.
 * @param measured_value Current measured value.
 * 
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t PID_Compute(PIDController *pid, float measured_value);



#endif // PID_H