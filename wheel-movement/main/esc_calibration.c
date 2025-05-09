/**
 * @file main.c
 * @author Striker 1
 * @brief Main file for obtaining the angle from the AS5600 sensor
 * @details For wheel modeling purposes, this code is used to obtain the angle from the AS5600 sensor. The angle is obtained through the ADC and the GPIO pin.
 * @version 0.1
 * @date 23/04/2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <stdio.h>

#include "bldc_pwm.h"

#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PWM_GPIO 3 ///< GPIO number for PWM signal
#define PWM_REV_GPIO 8 ///< GPIO number for PWM reverse signal
#define PWM_FREQ 50 ///< PWM frequency in Hz
#define PWM_RESOLUTION 100000 ///< PWM resolution in bits

bldc_pwm_motor_t pwm, pwm2;

void app_main(void)
{
    
    // bldc_init(&pwm, PWM_GPIO, 10, PWM_FREQ, 0, PWM_RESOLUTION); ///< Initialize the BLDC motor
    // bldc_init(&pwm2, PWM_REV_GPIO, 11, PWM_FREQ, 0, PWM_RESOLUTION); ///< Initialize the BLDC motor
    // bldc_enable(&pwm); ///< Enable the BLDC motor
    // bldc_enable(&pwm2); ///< Enable the BLDC motor

    // ESP_LOGI("PWM", "Starting calibration!"); ///< Log message
    // bldc_set_duty(&pwm, 100); ///< Set the duty cycle to 0%
    // bldc_set_duty(&pwm2, 100); ///< Set the duty cycle to 0%

    // vTaskDelay(3000 / portTICK_PERIOD_MS); ///< Wait for 10 seconds
    // ESP_LOGI("PWM", "PWM to 0!"); ///< Log message

    // bldc_set_duty(&pwm, 10); ///< Set the duty cycle to 0%
    // bldc_set_duty(&pwm2, 10); ///< Set the duty cycle to 0%

    // vTaskDelay(2000 / portTICK_PERIOD_MS); ///< Wait for 10 seconds

    // bldc_set_duty(&pwm, 30); ///< Set the duty cycle to 0%
    // bldc_set_duty(&pwm2, 30); ///< Set the duty cycle to 0%


    // while (1)
    // {
    //     ESP_LOGI("PWM", "ESC Calibrated!"); ///< Log message
    //     vTaskDelay(1000 / portTICK_PERIOD_MS); ///< Wait for 10 seconds
    // }

    bldc_init(&pwm, PWM_GPIO, 10, PWM_FREQ, 0, PWM_RESOLUTION); ///< Initialize the BLDC motor
    bldc_enable(&pwm); ///< Enable the BLDC motor

    bldc_init(&pwm2, 8, 11, PWM_FREQ, 0, PWM_RESOLUTION); ///< Initialize the BLDC motor
    bldc_enable(&pwm2); ///< Enable the BLDC motor

    ESP_LOGI("PWM", "Starting calibration!"); ///< Log message
    bldc_set_duty(&pwm, 10); ///< Set the duty cycle to 0%
    bldc_set_duty(&pwm2, 10); ///< Set the duty cycle to 0%

    vTaskDelay(8000 / portTICK_PERIOD_MS); ///< Wait for 10 seconds

    bldc_set_duty(&pwm, 30); ///< Set the duty cycle to 0%
    bldc_set_duty(&pwm2, 30); ///< Set the duty cycle to 0%


    while (1)
    {
        ESP_LOGI("PWM", "ESC running!"); ///< Log message
        vTaskDelay(1000 / portTICK_PERIOD_MS); ///< Wait for 10 seconds
    }
    
}