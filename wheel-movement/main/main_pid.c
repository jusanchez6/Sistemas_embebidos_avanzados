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
#include <math.h>

// Include personalized sensors libraries
#include "as5600_lib.h"
#include "VL53L1X.h"
#include "EasyRetrieve.h"

// Include personalized driver libraries
#include "bldc_pwm.h"
#include "pid_ext.h"

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
#define AS5600_I2C_MASTER_SCL_GPIO 5    ///< gpio number for I2C master clock
#define AS5600_I2C_MASTER_SDA_GPIO 4    ///< gpio number for I2C master data 
#define AS5600_OUT_GPIO 7               ///< gpio number for OUT signal
#define AS5600_I2C_MASTER_NUM 0         ///< I2C port number for master dev
#define AS5600_MODE 1                   ///< Calibration = 0, Angle through ADC = 1
///<--------------------------------------------------

///<------------- TM151 configuration ----------------
#define TM151_UART_TX 17                          ///< Gpio pin for UART TX
#define TM151_UART_RX 18                          ///< GPIO pin for UART RX
#define TM151_UART_BAUDRATE 921600                ///< Baudrate for UART communication
#define TM151_BUFFER_SIZE 1024                       ///< Buffer size for UART communication

///<--------------------------------------------------

///<------------- VL53L1X configuration --------------
#define VL53L1X_I2C_PORT 1      ///< I2C port number for master dev
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

#define EULER 2.71828
#define PI 3.14159
#define SAMPLE_RATE 100 // Hz (10ms per sample)

static volatile pid_block_handle_t pid; // PID control block handle
pid_parameter_t pid_param = {
    .kp = 25.0f,
    .ki = 0.5f,
    .kd = 50.0f,
    .max_output = 100.0f,
    .min_output = -100.0f,
    .set_point = 2.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};


bldc_pwm_motor_t pwm; ///< BLDC motor object
AS5600_t gAs5600;     ///< AS5600 object for angle sensor
vl53l1x_t gVl53l1x;   ///< VL53L1X object for distance sensor
uart_t myUART;        ///< UART object for TM151 IMU
float angle;          ///< Angle read from the AS5600 sensor

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
    AS5600_SetStopPosition(&gAs5600, 2000); ///< Set the stop position to 4095

    AS5600_InitADC(&gAs5600); ///< Initialize the ADC driver
    vTaskDelay(500 / portTICK_PERIOD_MS); ///< Wait for 500 ms
    ///<--------------------------------------------------


    ///<-------------- Initialize the VL53L1X sensor -----
    if(!VL53L1X_init(&gVl53l1x, VL53L1X_I2C_PORT, VL53L1X_SCL_GPIO, VL53L1X_SDA_GPIO, 0)){
        ESP_LOGE(TAG_VL53L1X, "Could not initialize VL53L1X sensor...");
        return;
    }
    VL53L1X_setDistanceMode(&gVl53l1x, Short); 
    VL53L1X_setMeasurementTimingBudget(&gVl53l1x, 20000);
    VL53L1X_startContinuous(&gVl53l1x, SAMPLE_TIME);
    vTaskDelay(500 / portTICK_PERIOD_MS); ///< Wait for 500 ms
    ///<--------------------------------------------------
    

    ///<-------------- Initialize the TM151 sensor ------
    tm151_init(&myUART, TM151_UART_BAUDRATE, TM151_BUFFER_SIZE, TM151_UART_TX, TM151_UART_RX); ///< Initialize the TM151 sensor

    float acceleration[3];
    ///<--------------------------------------------------

    ///<----------- Initialize the BLDC motor PWM --------
    ESP_LOGI("PWM", "Starting test..."); ///< Log message
    bldc_init(&pwm, PWM_GPIO, PWM_REV_GPIO, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwm); ///< Enable the BLDC motor
    bldc_set_duty(&pwm, 0); ///< Set the duty cycle to 0%
    ///<--------------------------------------------------

    vTaskDelay(4000 / portTICK_PERIOD_MS); ///< Wait for 4 seconds

    pid_config_t pid_config = {
        .init_param = pid_param
    };

    pid_new_control_block(&pid_config, &pid);


    int last_count = 0;
    uint32_t timestamp = 1000000; // 1 second
    float velocity_lidar = 0.0f, last_velocity = 0.0f;
    float beta = exp(-2 * PI * 1 / SAMPLE_RATE);  // 10Hz cutoff frequency
    float output = 0.0f;
    uint16_t distance, last_distance = 0; ///< Distance variables

    while (1) {
        distance = VL53L1X_readDistance(&gVl53l1x, 0); ///< Get the distance from the VL53L1X sensor

        if(distance <= 20 && distance != 0) { ///< If the distance is less than 20 cm
            ESP_LOGI(TAG_VL53L1X, "Distance %d mm. PWM: %hd", distance, 0);
            bldc_set_duty(&pwm, 0); ///< Set the duty cycle to 0%
        } else {

            // ///<-------------- Get distance through VL53L1X ------
            if(abs(distance - last_distance) > 10){
                velocity_lidar = 100.0 * (float)(last_distance - distance)  / (float)(SAMPLE_TIME); ///< Calculate the velocity cm/s
            }
            // velocity_lidar = 100.0 * (float)(distance - last_distance)  / (float)(SAMPLE_TIME); ///< Calculate the velocity cm/s
            last_distance = distance; ///< Update the last distance
            ///<--------------------------------------------------

            // ESP_LOGI(TAG_VL53L1X, "Distance %d mm. Velocity: %.2f. PWM: %hd", distance, velocity_lidar, pid.control); ///< Log message

            // Low-pass filter
            velocity_lidar = beta * last_velocity + (1 - beta) * velocity_lidar;
            last_velocity = velocity_lidar;

            // Update PID Controller
            pid_compute(pid, velocity_lidar, &output);
            bldc_set_duty(&pwm, output);

            printf("I,%" PRIu32 ",%hd,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp, distance, velocity_lidar, pid_param.set_point, output, 0.0, 0.0);

            timestamp += 1000000 / SAMPLE_RATE;  // 100Hz
        }

        

        vTaskDelay(200 / portTICK_PERIOD_MS);  // 100Hz sampling
    }
}

#endif