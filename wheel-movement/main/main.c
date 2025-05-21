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
#include "pid_ext.h"
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
#define PWM_GPIO 20                  ///< GPIO number for PWM signal
#define PWM_REV_GPIO 21              ///< GPIO number for PWM reverse signal
#define PWM_FREQ 50                 ///< PWM frequency in Hz
#define PWM_RESOLUTION 100000       ///< PWM resolution in bits
#define MAX_PWM_CAL 120             ///< Maximum PWM value
#define MIN_PWM_CAL 35              ///< Minimum PWM value
#define MAX_PWM_RE 119              ///< Maximum PWM value (moves fully)
#define MIN_PWM_RE 38               ///< Minimum PWM value (does not move)
///<--------------------------------------------------

///<-------------- PID configuration -----------------
#define PID_KP 1.66f
#define PID_KI 0.6f
#define PID_KD 0.05f /*1.8f*/
#define EULER 2.71828
#define PI 3.14159
#define SAMPLE_RATE 100 // Hz (10ms per sample)
///<--------------------------------------------------

#if MAIN_MODE == 0

bldc_pwm_motor_t pwm; ///< BLDC motor object
AS5600_t gAs5600;     ///< AS5600 object for angle sensor
vl53l1x_t gVl53l1x;   ///< VL53L1X object for distance sensor
uart_t myUART;        ///< UART object for TM151 IMU
float angle;          ///< Angle read from the AS5600 sensor
uint16_t distance;     ///< Distance read from the VL53L1X sensor

static volatile pid_block_handle_t pid; // PID control block handle
pid_parameter_t pid_param = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .max_output = 100.0f,
    .min_output = -100.0f,
    .set_point = 5.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

/*
 * @brief Task to control the wheel
 */
// void vTaskControl( void * pvParameters );

void uart_task(void* arg) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    uart_flush(UART_NUM_0);  // Flush UART buffer

    char data[128];
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)data, sizeof(data) - 1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            data[len] = '\0';
            sscanf(data, "%f %f %f %f", &pid_param.kp, &pid_param.ki, &pid_param.kd, &pid_param.set_point);
            pid_update_parameters(pid, &pid_param);
            printf("Updated PID values: kp=%.2f, ki=%.2f, kd=%.2f, setpoint=%.2f\n", pid_param.kp, pid_param.ki, pid_param.kd, pid_param.set_point);
        }
    }
}

void app_main(void)
{
    ///<----------- Initialize the BLDC motor PWM --------
    ESP_LOGI("PWM", "Starting test..."); ///< Log message
    bldc_init(&pwm, PWM_GPIO, PWM_REV_GPIO, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwm); ///< Enable the BLDC motor
    bldc_set_duty(&pwm, 0); ///< Set the duty cycle to 0%
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

    // AS5600_SetStartPosition(&gAs5600, 0x00); ///< Set the start position to 0
    // AS5600_SetStopPosition(&gAs5600, 0xFF); ///< Set the stop position to 4095

    // AS5600_InitADC(&gAs5600); ///< Initialize the ADC driver
    // vTaskDelay(4000 / portTICK_PERIOD_MS); ///< Wait for 500 ms
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
    imu_data_t imu_data;
    imu_data.velocity = 0.0f; ///< Initialize the velocity to 0
    imu_data.prev_acc = 0.0f; ///< Initialize the previous acceleration to 0

    // ///<-------------- Initialize the AS5600 sensor ------
    // encoder_data_t encoder_data;
    // encoder_data.angle = 0.0f;
    // encoder_data.radio = 3.0; ///< Initialize the radio 3.0 cm 
    // ///<--------------------------------------------------

    ///<-------------- Initialize the PID controller ------
    // if (PID_Init(&pid, 1.66, 0.6, /*1.8*/0.05) != ESP_OK)///< Initialize the PID controller
    // {
    //     ESP_LOGE(TAG_PID, "Could not initialize PID controller...");
    //     return;
    // }
    // PID_SetSetpoint(&pid, 2); ///< Set the setpoint to 0
    pid_config_t pid_config = {
        .init_param = pid_param
    };

    pid_new_control_block(&pid_config, &pid);
    ///<---------------------------------------------------

    ///<-------------- Create the task ---------------
    // TaskHandle_t xTaskControlHandle = NULL;
    // xTaskCreate(vTaskControl, "vTaskControl", 2048, queue, 10, &xTaskControlHandle); ///< Create the task to control the wheel
    // configASSERT(xTaskControlHandle); ///< Check if the task was created successfully
    // if (xTaskControlHandle == NULL) {
    //     ESP_LOGE("CTRL_TASK", "Failed to create task...");
    //     return;
    // }
    ///<--------------------------------------------------

    uint32_t timestamp = 1000000; // 1 second
    float est_velocity = 0.0f, last_est_velocity = 0.0f;
    float beta = exp(-2 * PI * 1 / SAMPLE_RATE);  // 10Hz cutoff frequency
    float output = 0.0f;

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);  // UART task

    while (1)
    {
        ///<-------------- Get angle through ADC -------------
        // angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC
        // ESP_LOGI(TAG_ADC, "Angle: %0.2f\n", angle); ///< Log message
        // ///<--------------------------------------------------

        // ///<-------------- Get distance through VL53L1X ------
        distance = VL53L1X_readDistance(&gVl53l1x, 0); ///< Get the distance from the VL53L1X sensor
        ESP_LOGI(TAG_VL53L1X, "Distance %d mm", distance); ///< Log message
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

        // Low-pass filter
        est_velocity = imu_data.velocity; ///< Update the estimated velocity

        est_velocity = beta * last_est_velocity + (1 - beta) * est_velocity;
        last_est_velocity = est_velocity;

        // Update PID Controller
        pid_compute(pid, est_velocity, &output);
        if(distance > 100){
            bldc_set_duty(&pwm, output);
        } else {
            bldc_set_duty(&pwm, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS); ///< Wait for 1 second
            bldc_set_duty(&pwm, -20);
            vTaskDelay(1500 / portTICK_PERIOD_MS); ///< Wait for 1 second
        }

        // printf("I,%" PRIu32 ",%hd,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp, distance, est_velocity, pid_param.set_point, output, 0.0, 0.0);
        printf("I,%" PRIu32 ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp, est_velocity, pid_param.set_point, 0.0, output, 0.0, 0.0);

        timestamp += 1000000 / SAMPLE_RATE;  // 100Hz
        ///<------------------------------------------

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