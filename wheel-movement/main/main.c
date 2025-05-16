/**
 * @file main.c
 * 
 * @brief main file for obtaining data from sensors and controlle the wheels
 * 
 * Main file for wheel modeling purposes, this code is used to get the data from the sensors AS5600, VL531Lx and TM151 
 * 
 * @authors Julian Sanchez
 *          Angel Graciano
 *          Nelson Parra
 * 
 * 
 * @date 14-05-2025
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


// Include standar libraries 
#include <stdio.h>

// Include personalized sensors libraries
#include "as5600_lib.h"
#include "VL53L1X.h"
#include "EasyObjectDictionary.h"
#include "EasyProfile.h"
#include "EasyRetrieve.h"
#include "bldc_pwm.h"


// Include ESP IDF libraries
#include <assert.h>
#include "esp_partition.h"
#include "spi_flash_mmap.h"
#include "freertos/task.h"




///<---------- Main mode: ------------------------
#define MAIN_MODE 0 ///< 0 = No Calibration, 1 = ESC Calibration
///<---------------------------------------------

///<-------------- AS5600 configuration ---------------
#define AS5600_I2C_MASTER_SCL_GPIO 5    ///< gpio number for I2C master clock
#define AS5600_I2C_MASTER_SDA_GPIO 4    ///< gpio number for I2C master data 
#define AS5600_OUT_GPIO 7               ///< gpio number for OUT signal
#define AS5600_I2C_MASTER_NUM 0         ///< I2C port number for master dev
#define AS5600_MODE 1                   ///< Calibration = 0, Angle through ADC = 1
///<--------------------------------------------------

///<------------- TM151 configuration ----------------
#define UART_TX 17                          ///< Gpio pin for UART TX
#define UART_RX 18                          ///< GPIO pin for UART RX
static const char* TAG_TM151 = "TM151";     ///< Tag for TM151

///<--------------------------------------------------

///<------------- VL53L1X configuration --------------
#define VL53L1X_I2C_PORT 0      ///< I2C port number for master dev
#define VL53L1X_SDA_GPIO 16     ///< gpio number for I2C master data 
#define VL53L1X_SCL_GPIO 15     ///< gpio number for I2C mastes clock
///<--------------------------------------------------

///<-------------- BLDC configuration -----------------
#define PWM_GPIO 3                  ///< GPIO number for PWM signal
#define PWM_REV_GPIO 8              ///< GPIO number for PWM reverse signal
#define PWM_FREQ 50                 ///< PWM frequency in Hz
#define PWM_RESOLUTION 100000       ///< PWM resolution in bits
#define MAX_PWM_CAL 120             ///< Maximum PWM value
#define MIN_PWM_CAL 35              ///< Minimum PWM value
#define MAX_PWM_RE 119              ///< Maximum PWM value (moves fully)
#define MIN_PWM_RE 38               ///< Minimum PWM value (does not move)
///<--------------------------------------------------

#if MAIN_MODE == 0

// configuration of motors
bldc_pwm_motor_t pwm, pwm2;

// object to control the magnetic rotary encoder
AS5600_t gAs5600;

// Object to control the 
vl53l1x_t gVl53l1x;

// object to control the TM151
uart_t myUART;                  ///< UART object


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
    if(!VL53L1X_init(&gVl53l1x, VL53L1X_I2C_PORT, VL53L1X_SCL_GPIO, VL53L1X_SDA_GPIO, 0)){
        ESP_LOGE(TAG_VL53L1X, "Could not initialize VL53L1X sensor...");
        return;
    }
    VL53L1X_setDistanceMode(&gVl53l1x, Short); 
    VL53L1X_setMeasurementTimingBudget(&gVl53l1x, 20000);
    VL53L1X_startContinuous(&gVl53l1x, 30);
    ///<--------------------------------------------------
    

    ///<-------------- Initialize the TM151 sensor ------
    if (!uart_init_with_defaults(&myUART, 115200, 1024, UART_TX, UART_RX) ) {
        ESP_LOGI(TAG_TM151, "Could not initialize  TM151 sensor... ");
        return;
    }
    ESP_LOGI(TAG_TM151, "TM151 initialized! ");

    // Initialize the TM151 interface
    
    EasyProfile_C_Interface_Init();

    ///<--------------------------------------------------

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
        ///<--------------------------------------------------

        if(i){
            float diff = angle - last_angle;
            if(diff < 0) diff += 360;

            ESP_LOGI("Encoder_PWM", "Angle: %f, RPM: %f", angle, diff* (float)(100 / 6)); ///< Log message
            last_angle = angle;
        }

        ///<-------------- Get distance through VL53L1X ------
        ESP_LOGI(TAG_VL53L1X, "Distance %d mm", VL53L1X_readDistance(&gVl53l1x, 1)); 
        ///<--------------------------------------------------


        ///<-------------- Get data from TM151 --------------
        uint16_t toId = EP_ID_BROADCAST_;
        char* txData;
        int txSize;

        if (EP_SUCC_ == EasyProfile_C_Interface_TX_Request(toId, EP_CMD_RPY_, &txData, &txSize)) {
            uart_write(&myUART, (uint8_t*)txData, (size_t)txSize);
        } 

        SerialPort_DataGet(&myUART);

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