/**
 * @file pid.h
 * 
 * @brief Simple pid implementation file
 * 
 * This file contains the implementation of a simple PID controller.
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
#include "pid.h"

esp_err_t PID_Init(PIDController *pid, float Kp, float Ki, float Kd) {
    
    // Initialize PID gains
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    
    // Initialize other parameters
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    
    // Initialize previous error
    pid->previous_error = 0.0f;
    
    // Initialize control output
    pid->control = 0.0f;

    return ESP_OK;
}

void PID_SetSetpoint(PIDController *pid, float setpoint) {
    pid->setpoint = setpoint;

}

esp_err_t PID_Compute(PIDController *pid, float measured_value, float dt) {
    // Calculate error
    float error = pid->setpoint - measured_value;
    
    // Proportional term
    float Pout = pid->Kp * error;
    
    // Integral term
    pid->integral += error * dt;
    float Iout = pid->Ki * pid->integral;
    
    // Derivative term
    float derivative = (error - pid->previous_error) / dt;
    float Dout = pid->Kd * derivative;

    
    // Save current error for next iteration
    pid->previous_error = error;

    pid->control = Pout + Iout + Dout;
    
    return ESP_OK;
}

