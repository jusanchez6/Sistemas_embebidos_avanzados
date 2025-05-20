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
#include "EasyRetrieve.h"

// Include personalized driver libraries
#include "bldc_pwm.h"
#include "pid.h"
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
#define AS5600_I2C_MASTER_SCL_GPIO 5    ///< gpio number for I2C master clock
#define AS5600_I2C_MASTER_SDA_GPIO 4    ///< gpio number for I2C master data 
#define AS5600_OUT_GPIO 7               ///< gpio number for OUT signal
#define AS5600_I2C_MASTER_NUM 0         ///< I2C port number for master dev
#define AS5600_MODE 1                   ///< Calibration = 0, Angle through ADC = 1
///<--------------------------------------------------

///<------------- TM151 configuration ----------------
#define TM151_UART_TX 17                          ///< Gpio pin for UART TX
#define TM151_UART_RX 18                          ///< GPIO pin for UART RX
#define TM151_UART_BAUDRATE 115600                ///< Baudrate for UART communication
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

typedef struct {
    float AS5600_angle;
    uint16_t VL53L1X_distance;
    float TM151_acceleration[3];
} alarm_event_t;

typedef struct
{
    QueueHandle_t queue; ///< Queue to send the event data

    AS5600_t *gAs5600; ///< AS5600 object for angle sensor
    vl53l1x_t *gVl53l1x; ///< VL53L1X object for distance sensor
    uart_t *myUART; ///< UART object for TM151 IMU
} alarm_params_t;


bldc_pwm_motor_t pwm; ///< BLDC motor object
AS5600_t gAs5600;     ///< AS5600 object for angle sensor
vl53l1x_t gVl53l1x;   ///< VL53L1X object for distance sensor
uart_t myUART;        ///< UART object for TM151 IMU
// PIDController pid;    ///< PID controller object
float angle;          ///< Angle read from the AS5600 sensor

/*
 * @brief Task to control the wheel
 */
// void vTaskControl( void * pvParameters );

/*
 * @brief Callback function for the timer to retrieve the data from the sensors
 */
// static bool IRAM_ATTR ret_sensor_data_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data);
// static bool IRAM_ATTR ret_sensor_data_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
// {
//     BaseType_t high_task_awoken = pdFALSE;
//     alarm_params_t usr_dt = *(alarm_params_t *)user_data; ///< Get the user data from the callback
//     QueueHandle_t queue = (QueueHandle_t)(usr_dt.queue); ///< Get the queue from the user data
//     AS5600_t *gAs5600 = (AS5600_t *)(usr_dt.gAs5600);
//     vl53l1x_t *gVl53l1x = (vl53l1x_t *)(usr_dt.gVl53l1x);
//     uart_t *myUART = (uart_t *)(usr_dt.myUART);

//     ESP_LOGI("ALRM", "Getting sensors info\n");
//     float angle = AS5600_ADC_GetAngle(gAs5600); ///< Get the angle from the ADC
//     uint16_t distance = VL53L1X_readDistance(gVl53l1x, 0); ///< Get the distance from the VL53L1X sensor

//     alarm_event_t evt = {

//         ///<-------------- Get angle through ADC -------------
//         .AS5600_angle = angle, ///< Get the angle from the ADC
//         ///<--------------------------------------------------

//         ///<-------------- Get distance through VL53L1X ------
//         .VL53L1X_distance = distance
//         ///<--------------------------------------------------

//         ///<-------------- Get data from TM151 --------------
//         // SerialPort_DataReceived_RawAcc(&myUART, acceleration);
//         ///<--------------------------------------------------

//     };

//     xQueueSendFromISR(queue, &evt, &high_task_awoken);

//     return high_task_awoken == pdTRUE;
// }

void app_main(void)
{
    ///<----------- Initialize the BLDC motor PWM --------
    // ESP_LOGI("PWM", "Starting test..."); ///< Log message
    // bldc_init(&pwm, PWM_GPIO, PWM_REV_GPIO, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    // bldc_enable(&pwm); ///< Enable the BLDC motor
    // bldc_set_duty(&pwm, 0); ///< Set the duty cycle to 0%
    ///<--------------------------------------------------

    vTaskDelay(4000 / portTICK_PERIOD_MS); ///< Wait for 4 seconds

    ///<------------- Initialize the AS5600 sensor -------
    // AS5600_Init(&gAs5600, AS5600_I2C_MASTER_NUM, AS5600_I2C_MASTER_SCL_GPIO, AS5600_I2C_MASTER_SDA_GPIO, AS5600_OUT_GPIO);

    // // Set some configurations to the AS5600
    // AS5600_config_t conf = {
    //     .PM = AS5600_POWER_MODE_NOM, ///< Normal mode
    //     .HYST = AS5600_HYSTERESIS_OFF, ///< Hysteresis off
    //     .OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR, ///< Analog output 10%-90%
    //     .PWMF = AS5600_PWM_FREQUENCY_115HZ, ///< PWM frequency 115Hz
    //     .SF = AS5600_SLOW_FILTER_16X, ///< Slow filter 16x
    //     .FTH = AS5600_FF_THRESHOLD_SLOW_FILTER_ONLY, ///< Slow filter only
    //     .WD = AS5600_WATCHDOG_ON, ///< Watchdog on
    // };
    // AS5600_SetConf(&gAs5600, conf);
    
    // // Read the configuration
    // uint16_t conf_reg;
    // AS5600_ReadReg(&gAs5600, AS5600_REG_CONF_H, &conf_reg);
    // printf("Configuration register read: 0x%04X\n", conf_reg);
    // printf("Configuration register written: 0x%04X\n", conf.WORD);

    // AS5600_ReadReg(&gAs5600, AS5600_REG_STATUS, &conf_reg);

    // printf("Status register read: 0x%02X\n", conf_reg);

    // AS5600_SetStartPosition(&gAs5600, 0); ///< Set the start position to 0
    // AS5600_SetStopPosition(&gAs5600, 2000); ///< Set the stop position to 4095

    // AS5600_InitADC(&gAs5600); ///< Initialize the ADC driver
    // vTaskDelay(500 / portTICK_PERIOD_MS); ///< Wait for 500 ms
    ///<--------------------------------------------------


    ///<-------------- Initialize the VL53L1X sensor -----
    // if(!VL53L1X_init(&gVl53l1x, VL53L1X_I2C_PORT, VL53L1X_SCL_GPIO, VL53L1X_SDA_GPIO, 0)){
    //     ESP_LOGE(TAG_VL53L1X, "Could not initialize VL53L1X sensor...");
    //     return;
    // }
    // VL53L1X_setDistanceMode(&gVl53l1x, Short); 
    // VL53L1X_setMeasurementTimingBudget(&gVl53l1x, 20000);
    // VL53L1X_startContinuous(&gVl53l1x, SAMPLE_TIME);
    // vTaskDelay(500 / portTICK_PERIOD_MS); ///< Wait for 500 ms
    ///<--------------------------------------------------
    

    ///<-------------- Initialize the TM151 sensor ------
    tm151_init(&myUART, TM151_UART_BAUDRATE, TM151_BUFFER_SIZE, TM151_UART_TX, TM151_UART_RX); ///< Initialize the TM151 sensor

    float acceleration[3];
    imu_data_t imu_data;
    imu_data.velocity = 0.0f; ///< Initialize the velocity to 0
    imu_data.prev_acc = 0.0f; ///< Initialize the previous acceleration to 0
    ///<--------------------------------------------------

    ///<-------------- Initialize the PID controller ------
    // if (PID_Init(&pid, 1.66, 0.6, /*1.8*/0.05) != ESP_OK)///< Initialize the PID controller
    // {
    //     ESP_LOGE(TAG_PID, "Could not initialize PID controller...");
    //     return;
    // }
    // PID_SetSetpoint(&pid, 2); ///< Set the setpoint to 0
    ///<---------------------------------------------------

    ///<-------------- Initialize the timer ---------------
    // // Config the timer
    // alarm_event_t evt; ///< Event data for the timer


    // gptimer_handle_t gptimer = NULL;
    // gptimer_config_t config = {
    //     .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    //     .direction = GPTIMER_COUNT_UP,
    //     .resolution_hz = 1000000,   // 1 MHz - 1 microsegundo por tick
    // };

    // ESP_ERROR_CHECK(gptimer_new_timer(&config, &gptimer));

    // // Config the callback
    // gptimer_event_callbacks_t cbs = {
    //     .on_alarm = ret_sensor_data_alarm,
    // };

    // // Create a queue to send the event data
    // QueueHandle_t queue = xQueueCreate(25, sizeof(alarm_event_t));
    // if (queue == NULL) {
    //     ESP_LOGE("QUEUE_ALRM", "Failed to create queue...");
    //     return;
    // }
    // ESP_LOGI("QUEUE_ALRM", "Queue created successfully!");

    // // Set the user data to the queue
    // alarm_params_t user_data = {
    //     .queue = queue,
    //     .gAs5600 = &gAs5600,
    //     .gVl53l1x = &gVl53l1x,
    //     .myUART = &myUART,
    // };

    // ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, &user_data));
    // ESP_ERROR_CHECK(gptimer_enable(gptimer));

    // // Config alarm with SAMPLE_TIME milliseconds period and auto-reload
    // gptimer_alarm_config_t alarm_config = {
    //     .alarm_count = 1000000, // Shoot every SAMPLE_TIME milliseconds
    //     .reload_count = 0,                 // Reload count
    //     .flags.auto_reload_on_alarm = true,
    // };
    // ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    // // Initialize the timer
    // ESP_ERROR_CHECK(gptimer_start(gptimer));
    // ESP_LOGI("CTRL_ALRM", "Timer inited successfully!");
    ///<--------------------------------------------------

    ///<-------------- Create the task ---------------
    // TaskHandle_t xTaskControlHandle = NULL;
    // xTaskCreate(vTaskControl, "vTaskControl", 2048, queue, 10, &xTaskControlHandle); ///< Create the task to control the wheel
    // configASSERT(xTaskControlHandle); ///< Check if the task was created successfully
    // if (xTaskControlHandle == NULL) {
    //     ESP_LOGE("CTRL_TASK", "Failed to create task...");
    //     return;
    // }
    ///<--------------------------------------------------

    while (1)
    {
        // if (xQueueReceive(queue, &evt, pdMS_TO_TICKS(2000))) {
        //     ESP_LOGI("TAG", "Angle: %f", evt.AS5600_angle);
        // }
        // ESP_LOGI("PWM", "ESC running!"); ///< Log message
        // distance = VL53L1X_readDistance(&gVl53l1x, 0); ///< Get the distance from the VL53L1X sensor

        // if(distance <= 20 && distance != 0) { ///< If the distance is less than 20 cm
        //     ESP_LOGI(TAG_VL53L1X, "Distance %d mm. PWM: %hd", distance, 0);
        //     bldc_set_duty(&pwm, 0); ///< Set the duty cycle to 0%
        // } else {

            ///<-------------- Get angle through ADC -------------
            // angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC
            // ESP_LOGI(TAG_ADC, "Angle: %0.2f\n", angle); ///< Log message
            // ///<--------------------------------------------------

            // ///<-------------- Get distance through VL53L1X ------
            // if(abs(distance - last_distance) > 10){
            //     velocity_lidar = 100.0 * (float)(last_distance - distance)  / (float)(SAMPLE_TIME); ///< Calculate the velocity cm/s
            // }
            // else{
            //     velocity_lidar = 0;
            // }
            // // velocity_lidar = 100.0 * (float)(distance - last_distance)  / (float)(SAMPLE_TIME); ///< Calculate the velocity cm/s
            // last_distance = distance; ///< Update the last distance
            ///<--------------------------------------------------

            ///<-------------- Get data from TM151 --------------
            SerialPort_DataReceived_RawAcc(&myUART, acceleration);
            // printf("Acceleration: %0.4f %0.4f %0.4f\n", acceleration[0], acceleration[1], acceleration[2]); ///< Log message
            float acc_2dp = (int)(acceleration[0] * 10);
            estimate_velocity_imu(&imu_data, (float)acc_2dp / 10.0, SAMPLE_TIME / 1000.0f); ///< Estimate the velocity using IMU data
            printf("Velocity: %0.4f cm/s, with time interval %.2f seconds\n", imu_data.velocity, SAMPLE_TIME / 1000.0f); ///< Log message
            ///<--------------------------------------------------

            ///<-------------- PID Control ---------------
            // PID_Compute(&pid, velocity_lidar, SAMPLE_TIME / 1000.0f); ///< Compute the PID control
            // bldc_set_duty(&pwm, pid.control); ///< Set the duty cycle to target
            ///<------------------------------------------

            // ESP_LOGI(TAG_VL53L1X, "Distance %d mm. Velocity: %.2f. PWM: %hd", distance, velocity_lidar, pid.control); ///< Log message
        // }

        vTaskDelay(10 / portTICK_PERIOD_MS); ///< Wait for 2 seconds
    }
}

#endif

// Task to control the wheel
// void vTaskControl( void * pvParameters ){
//     QueueHandle_t queue = (QueueHandle_t) pvParameters;
//     alarm_event_t evt; ///< Event data for the timer

//     while(1){
//         if (xQueueReceive(queue, &evt, pdMS_TO_TICKS(500))) {
//             ///<----------------- Act to angle -------------------
//             ESP_LOGI(TAG_ADC, "Angle: %0.2f\n", evt.AS5600_angle); ///< Log message
//             ///<--------------------------------------------------

//             ///<------------------ Act to distance ---------------
//             ESP_LOGI(TAG_VL53L1X, "Distance %d mm", evt.VL53L1X_distance);
//             ///<--------------------------------------------------

//             ///<-------------- Act to acceleration ---------------
//             // ESP_LOGI(TAG_TM151, "Acceleration: %0.4f %0.4f %0.4f", evt.acceleration[0], evt.acceleration[1], evt.acceleration[2]); ///< Log message
//             ///<--------------------------------------------------

//             ///<-------------- PID Control ---------------
//             // PID_Compute(&pid, /*Velocity estimation*/, SAMPLE_TIME / 1000.0f); ///< Compute the PID control
//             // bldc_set_duty(&pwm, pid.control); ///< Set the duty cycle to target
//             ///<------------------------------------------
//         }
//     }
// }

// Alarm callback function for wheel control
// static bool IRAM_ATTR ret_sensor_data_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
// {
//     BaseType_t high_task_awoken = pdFALSE;
//     QueueHandle_t queue = (QueueHandle_t)user_data;

//     ESP_LOGI("ALRM", "Getting sensors info\n");
//     float angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC
//     uint16_t distance = VL53L1X_readDistance(&gVl53l1x, 0); ///< Get the distance from the VL53L1X sensor

//     alarm_event_t evt = {

//         ///<-------------- Get angle through ADC -------------
//         .AS5600_angle = angle, ///< Get the angle from the ADC
//         ///<--------------------------------------------------

//         ///<-------------- Get distance through VL53L1X ------
//         .VL53L1X_distance = distance
//         ///<--------------------------------------------------

//         ///<-------------- Get data from TM151 --------------
//         // SerialPort_DataReceived_RawAcc(&myUART, acceleration);
//         ///<--------------------------------------------------

//     };

//     xQueueSendFromISR(queue, &evt, &high_task_awoken);

//     return high_task_awoken == pdTRUE;
// }

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