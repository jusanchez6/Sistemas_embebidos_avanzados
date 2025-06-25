/**
 * @file main.c
 * 
 * @brief main file for obtaining data from sensors and control the 3-wheeled robot
 * 
 * Main file for wheel modeling purposes, this code is used to get the data from the sensors AS5600, VL531Lx and TM151 
 * 
 * @authors Julian Sanchez
 *          Angel Graciano
 *          Nelson Parra
 * 
 * 
 * @date 20-06-2025
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

#include "control_main.h"

void app_main(void)
{

    bldc_pwm_motor_t pwmR, pwmL, pwmB;   ///< BLDC motor object right, left and back


    ///<------- Initialize the BLDC motors PWMs ----------
    bldc_init(&pwmR, PWM_GPIO_R, PWM_REV_GPIO_R, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwmR); ///< Enable the BLDC motor
    bldc_set_duty(&pwmR, 0); ///< Set the duty cycle to 0%

    bldc_init(&pwmL, PWM_GPIO_L, PWM_REV_GPIO_L, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwmL); ///< Enable the BLDC motor
    bldc_set_duty(&pwmL, 0); ///< Set the duty cycle to 0%

    bldc_init(&pwmB, PWM_GPIO_B, PWM_REV_GPIO_B, PWM_FREQ, 1, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwmB); ///< Enable the BLDC motor
    bldc_set_duty(&pwmB, 0); ///< Set the duty cycle to 0%
    ///<--------------------------------------------------

    vTaskDelay(pdMS_TO_TICKS(2000)); ///< Delay for 2 second to allow the motors to initialize

    int32_t duty = 0; ///< Duty cycle variable
    bool reverse = false; ///< Reverse variable

    for (;;)
    {

        duty += 5; ///< Increase the duty cycle
        if (duty > 50) ///< If the duty cycle is greater than 100%
        {
            duty = 0; ///< Set the duty cycle to 0%
            reverse = !reverse; ///< Reverse the direction
            vTaskDelay(pdMS_TO_TICKS(1000)); ///< Delay for 1 second
        }
        if (reverse) { ///< If the direction is reversed
            ESP_LOGI("PWM", "Motor running in reverse! With duty: %li", duty); ///< Log message
            bldc_set_duty(&pwmR, -duty); ///< Set the duty cycle to -duty
            bldc_set_duty(&pwmL, -duty); ///< Set the duty cycle to -duty
            bldc_set_duty(&pwmB, -duty); ///< Set the duty cycle to -duty
        } else {
            ESP_LOGI("PWM", "Motor running in forward! With duty: %li", duty); ///< Log message
            bldc_set_duty(&pwmR, duty); ///< Set the duty cycle to duty
            bldc_set_duty(&pwmL, duty); ///< Set the duty cycle to duty
            bldc_set_duty(&pwmB, duty); ///< Set the duty cycle to duty
        }

        vTaskDelay(pdMS_TO_TICKS(500)); ///< Delay for 1 second
    }

    
    

}