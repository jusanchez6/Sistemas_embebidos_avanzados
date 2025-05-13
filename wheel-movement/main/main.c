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

#include "as5600_lib.h"
//#include "VL53L1X.h"
#include "bldc_pwm.h"

#include <assert.h>
#include "esp_partition.h"
#include "spi_flash_mmap.h"

#include "freertos/task.h"

///<---------- Main mode: ------------------------
#define MAIN_MODE 0 ///< 0 = No Calibration, 1 = ESC Calibration
///<---------------------------------------------

///<-------------- AS5600 configuration ---------------
#define AS5600_I2C_MASTER_SCL_GPIO 5  ///< gpio number for I2C master clock
#define AS5600_I2C_MASTER_SDA_GPIO 4  ///< gpio number for I2C master data 
#define AS5600_OUT_GPIO 7             ///< gpio number for OUT signal
#define AS5600_I2C_MASTER_NUM 0       ///< I2C port number for master dev
#define AS5600_MODE 1                 ///< Calibration = 0, Angle through ADC = 1
///<--------------------------------------------------

///<------------- TM151 configuration ----------------
///<--------------------------------------------------

///<------------- VL53L1X configuration --------------
#define VL53L1X_I2C_PORT 0
#define VL53L1X_SDA_GPIO 21
#define VL53L1X_SCL_GPIO 22
///<--------------------------------------------------

///<-------------- BLDC configuration -----------------
#define PWM_GPIO 3 ///< GPIO number for PWM signal
#define PWM_REV_GPIO 8 ///< GPIO number for PWM reverse signal
#define PWM_FREQ 50 ///< PWM frequency in Hz
#define PWM_RESOLUTION 100000 ///< PWM resolution in bits
#define MAX_PWM_CAL 120 ///< Maximum PWM value
#define MIN_PWM_CAL 35 ///< Minimum PWM value
#define MAX_PWM_RE 119 ///< Maximum PWM value (moves fully)
#define MIN_PWM_RE 38 ///< Minimum PWM value (does not move)
///<--------------------------------------------------

#if MAIN_MODE == 0
bldc_pwm_motor_t pwm, pwm2;

AS5600_t gAs5600;
//vl53l1x_t gVl53l1x;

float angle; ///< Angle read from the AS5600 sensor

void app_main(void)
{
    ///<------------- Initialize the AS5600 sensor -------
    AS5600_Init(&gAs5600, AS5600_I2C_MASTER_NUM, AS5600_I2C_MASTER_SCL_GPIO, AS5600_I2C_MASTER_SDA_GPIO, AS5600_OUT_GPIO);

    // Set some configurations to the AS5600
    AS5600_config_t conf = {
        .PM = AS5600_POWER_MODE_NOM, ///< Normal mode
        .HYST = AS5600_HYSTERESIS_OFF, ///< Hysteresis off
        .OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR, ///< Analog output 10%-90%
        .PWMF = AS5600_PWM_FREQUENCY_115HZ, ///< PWM frequency 115Hz
        .SF = AS5600_SLOW_FILTER_16X, ///< Slow filter 16x
        .FTH = AS5600_FF_THRESHOLD_SLOW_FILTER_ONLY, ///< Slow filter only
        .WD = AS5600_WATCHDOG_ON, ///< Watchdog on
    };
    AS5600_SetConf(&gAs5600, conf);
    
    // Read the configuration
    uint16_t conf_reg;
    AS5600_ReadReg(&gAs5600, AS5600_REG_CONF_H, &conf_reg);
    printf("Configuration register read: 0x%04X\n", conf_reg);
    printf("Configuration register written: 0x%04X\n", conf.WORD);

    AS5600_ReadReg(&gAs5600, AS5600_REG_STATUS, &conf_reg);

    printf("Status register read: 0x%02X\n", conf_reg);

    AS5600_SetStartPosition(&gAs5600, 0); ///< Set the start position to 0
    AS5600_SetStopPosition(&gAs5600, 4095); ///< Set the stop position to 4095

    AS5600_InitADC(&gAs5600); ///< Initialize the ADC driver
    ///<--------------------------------------------------

    ///<-------------- Initialize the VL53L1X sensor -----
    /*if(!VL53L1X_init(&gVl53l1x, VL53L1X_I2C_PORT, VL53L1X_SCL_GPIO, VL53L1X_SDA_GPIO, 0)){
        ESP_LOGE(TAG_VL53L1X, "Could not initialize VL53L1X sensor...");
        return;
    }
    VL53L1X_setDistanceMode(&gVl53l1x, Short); 
    VL53L1X_setMeasurementTimingBudget(&gVl53l1x, 20000);
    VL53L1X_startContinuous(&gVl53l1x, 30);
    ///<--------------------------------------------------
    */


    ///<----------- Initialize the BLDC motor PWM --------
    // ESP_LOGI("PWM", "Starting test..."); ///< Log message
    bldc_init(&pwm, PWM_GPIO, PWM_REV_GPIO, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwm); ///< Enable the BLDC motor
    bldc_set_duty(&pwm, 0); ///< Set the duty cycle to 0%
    ///<--------------------------------------------------

    vTaskDelay(5000 / portTICK_PERIOD_MS); ///< Wait for 15 seconds    

    int16_t duty = 0; ///< Duty cycle variable
    bool reverse = false; ///< Reverse variable
    float last_angle = 0;
    for (uint16_t i = 0; i < 500; i++)
    {
        // ESP_LOGI("PWM", "ESC running!"); ///< Log message

        ///<-------------- Get angle through ADC -------------
        angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC
        // ESP_LOGI("Encoder_ADC", "Angle: %f", angle); ///< Log message
        // vTaskDelay(10 / portTICK_PERIOD_MS); ///< Wait for 10 miliseconds
        ///<--------------------------------------------------

        if(i){
            float diff = angle - last_angle;
            if(diff < 0) diff += 360;

            ESP_LOGI("Encoder_PWM", "Angle: %f, RPM: %f", angle, diff* (float)(100 / 6)); ///< Log message
            last_angle = angle;
        }

        ///<-------------- Get distance through VL53L1X ------
        /* ESP_LOGI(TAG_VL53L1X, "Distance %d mm", VL53L1X_readDistance(&gVl53l1x, 1)); */
        ///<--------------------------------------------------

        // if (abs(duty) > 50) ///< If the duty cycle is greater than 100%
        // {
        //     if(reverse) duty = -1; ///< Set the duty cycle to 0%
        //     else duty = 1; ///< Set the duty cycle to 0%
        //     reverse = !reverse; ///< Reverse the direction
        //     bldc_set_duty(&pwm, duty); ///< Set the duty cycle to 0%
        //     vTaskDelay(1000 / portTICK_PERIOD_MS); ///< Wait for 5 seconds
        // }
        // if (reverse) { ///< If the direction is reversed
        //     duty -= 2; ///< Increase the reverse duty cycle
        //     // ESP_LOGI("PWM", "Motor running in reverse! PWM: %hd.", duty); ///< Log message
        //     bldc_set_duty(&pwm, duty); ///< Set the duty cycle to 0%
        // } else {
        //     duty += 2; ///< Increase the forward duty cycle
        //     // ESP_LOGI("PWM", "Motor running in forward! PWM: %hd.", duty); ///< Log message
        //     bldc_set_duty(&pwm, duty); ///< Set the duty cycle to the current PWM value
        // }

        bldc_set_duty(&pwm, 80); ///< Set the duty cycle to the current PWM value

        vTaskDelay(10 / portTICK_PERIOD_MS); ///< Wait for 2 seconds
    }

    bldc_set_duty(&pwm, 0); ///< Set the duty cycle to the current PWM value

}
#endif

#if MAIN_MODE == 1

bldc_pwm_motor_t pwm, pwm2;

void app_main(void)
{
    
    bldc_init(&pwm, PWM_GPIO, 10, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_init(&pwm2, PWM_REV_GPIO, 11, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwm); ///< Enable the BLDC motor
    bldc_enable(&pwm2); ///< Enable the BLDC motor

    ESP_LOGI("PWM", "Starting calibration!"); ///< Log message
    bldc_set_duty_old(&pwm, MAX_PWM_CAL); ///< Set the duty cycle to 0%
    bldc_set_duty_old(&pwm2, MAX_PWM_CAL); ///< Set the duty cycle to 0%

    vTaskDelay(7000 / portTICK_PERIOD_MS); ///< Wait for 10 seconds
    
    ESP_LOGI("PWM", "PWM to 0!"); ///< Log message
    bldc_set_duty_old(&pwm, MIN_PWM_CAL); ///< Set the duty cycle to 0%
    bldc_set_duty_old(&pwm2, MIN_PWM_CAL); ///< Set the duty cycle to 0%

    vTaskDelay(2000 / portTICK_PERIOD_MS); ///< Wait for 10 seconds

    ESP_LOGI("PWM", "ESC Calibrated!"); ///< Log message

    vTaskDelay(5000 / portTICK_PERIOD_MS); ///< Wait for 10 seconds

    ESP_LOGI("PWM", "Starting test..."); ///< Log message

    int32_t duty = MIN_PWM_CAL; ///< Duty cycle variable
    bool reverse = false; ///< Reverse variable
    while (1)
    {
        ESP_LOGI("PWM", "ESC running!"); ///< Log message

        duty += 1; ///< Increase the duty cycle
        if (duty > MAX_PWM_CAL) ///< If the duty cycle is greater than 100%
        {
            duty = MIN_PWM_CAL; ///< Set the duty cycle to 0%
            reverse = !reverse; ///< Reverse the direction
        }
        if (reverse) { ///< If the direction is reversed
            ESP_LOGI("PWM", "Motor running in reverse! With duty: %li", duty); ///< Log message
            bldc_set_duty_old(&pwm2, 90); ///< Set the duty cycle to the current PWM value
            bldc_set_duty_old(&pwm, duty); ///< Set the duty cycle to 0%
        } else {
            ESP_LOGI("PWM", "Motor running in forward! With duty: %li", duty); ///< Log message
            bldc_set_duty_old(&pwm2, 10); ///< Set the duty cycle to 0%
            bldc_set_duty_old(&pwm, duty); ///< Set the duty cycle to the current PWM value
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS); ///< Wait for 2 seconds
    }
    
}
#endif