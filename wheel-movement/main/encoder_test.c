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

// Include personalized driver libraries
#include "sensor_fusion.h"

// Include ESP IDF libraries
#include <assert.h>
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"

#define SAMPLE_TIME 10 ///< Sample time in ms

///<---------- Main mode: ------------------------
#define MAIN_MODE 0 ///< 0 = No Calibration, 1 = ESC Calibration
///<---------------------------------------------

///<-------------- AS5600 configuration ---------------
#define AS5600_I2C_MASTER_SCL_GPIO 4    ///< gpio number for I2C master clock
#define AS5600_I2C_MASTER_SDA_GPIO 5    ///< gpio number for I2C master data 
#define AS5600_OUT_GPIO 6               ///< gpio number for OUT signal
#define AS5600_I2C_MASTER_NUM 0         ///< I2C port number for master dev
#define AS5600_MODE 1                   ///< Calibration = 0, Angle through ADC = 1
///<--------------------------------------------------

#if MAIN_MODE == 0

AS5600_t gAs5600;     ///< AS5600 object for angle sensor
float angle;          ///< Angle read from the AS5600 sensor


encoder_data_t encoder_data;

/**
 * @brief Task to control the wheel
 * 
 * @param pvParameters 
 */
void control_task( void * pvParameters );

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
    AS5600_InitADC(&gAs5600); ///< Initialize the ADC driver
    
    // Read the configuration
    uint16_t conf_reg;
    AS5600_ReadReg(&gAs5600, AS5600_REG_CONF_H, &conf_reg);
    printf("Configuration register read: 0x%04X\n", conf_reg);
    printf("Configuration register written: 0x%04X\n", conf.WORD);

    AS5600_ReadReg(&gAs5600, AS5600_REG_STATUS, &conf_reg);

    printf("Status register read: 0x%02X\n", conf_reg);

    AS5600_SetStartPosition(&gAs5600, 0x0000); ///< Set the start position to 0
    AS5600_SetStopPosition(&gAs5600, 0x0FFF); ///< Set the stop position to 4095

    
    vTaskDelay(5000 / portTICK_PERIOD_MS); ///< Wait for 500 ms
    ///<--------------------------------------------------

    ///<-------------- Create the task ---------------
    TaskHandle_t xControlTaskHandle = NULL; ///< Task handles
    xTaskCreate(control_task, "control_task", 4096, NULL, 10, &xControlTaskHandle); ///< Create the task to control the wheel
    configASSERT(xControlTaskHandle); ///< Check if the task was created successfully
    if (xControlTaskHandle == NULL) {
        ESP_LOGE("CTRL_TASK", "Failed to create task...");
        return;
    }
    ///<--------------------------------------------------

}

// Task to control the wheel
void control_task( void * pvParameters ){

    while (1)
    {
        ///<-------------- Get angle through ADC -------------
        angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC
        ESP_LOGI(TAG_ADC, "Angle: %0.2f\n", angle); ///< Log message
        estimate_velocity_encoder(&encoder_data, angle, SAMPLE_TIME / 1000.0f); ///< Estimate the velocity using encoder data
        // printf("Angle: %0.2f degrees, with time interval %.2f seconds\n", angle, SAMPLE_TIME / 1000.0f); ///< Log message
        // ///<--------------------------------------------------        

        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 10 ms
    }
}

#endif