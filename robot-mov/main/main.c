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

uint32_t temp_ctr = 0; ///< Temporary counter to stop move_one_time

void app_main(void)
{
    AS5600_t gAs5600R, gAs5600L, gAs5600B;  ///< AS5600 object for angle sensor right, left and back
    vl53l1x_t gVl53l1x;                     ///< VL53L1X object for distance sensor
    uart_t myUART;                          ///< UART object for TM151 IMU

    extern encoder_data_t right_encoder_data, left_encoder_data, back_encoder_data; ///< Encoder data for right, left and back wheels
    extern imu_data_t imu_data;
    extern lidar_data_t lidar_data;

    bldc_pwm_motor_t pwmR, pwmL, pwmB;   ///< BLDC motor object right, left and back
    pid_block_handle_t pidR, pidL, pidB; ///< PID control block handle

    extern pid_parameter_t pid_paramR, pid_paramL, pid_paramB; ///< PID parameters for right, left and back wheels

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


    ///<------- Initialize the BLDC motors PWMs ----------
    bldc_init(&pwmR, PWM_GPIO_R, PWM_REV_GPIO_R, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwmR); ///< Enable the BLDC motor
    bldc_set_duty(&pwmR, 0); ///< Set the duty cycle to 0%

    bldc_init(&pwmL, PWM_GPIO_L, PWM_REV_GPIO_L, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwmL); ///< Enable the BLDC motor
    bldc_set_duty(&pwmL, 0); ///< Set the duty cycle to 0%

    bldc_init(&pwmB, PWM_GPIO_B, PWM_REV_GPIO_B, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwmB); ///< Enable the BLDC motor
    bldc_set_duty(&pwmB, 0); ///< Set the duty cycle to 0%
    ///<--------------------------------------------------

    ///<------------- Initialize the AS5600 sensor -------
    AS5600_Init(&gAs5600R, AS5600_I2C_MASTER_NUM, AS5600_I2C_MASTER_SCL_GPIO, AS5600_I2C_MASTER_SDA_GPIO, AS5600_OUT_GPIO); ///< Initialize the AS5600 sensor for right wheel
    AS5600_Init(&gAs5600L, AS5600_I2C_MASTER_NUM, AS5600_I2C_MASTER_SCL_GPIO, AS5600_I2C_MASTER_SDA_GPIO, AS5600_OUT_GPIO + 1); ///< Initialize the AS5600 sensor for left wheel
    AS5600_Init(&gAs5600B, AS5600_I2C_MASTER_NUM, AS5600_I2C_MASTER_SCL_GPIO, AS5600_I2C_MASTER_SDA_GPIO, AS5600_OUT_GPIO + 2); ///< Initialize the AS5600 sensor for back wheel

    AS5600_InitADC(&gAs5600R); ///< Initialize the ADC driver for the right AS5600 sensor
    AS5600_InitADC(&gAs5600L); ///< Initialize the ADC driver for the left AS5600 sensor
    AS5600_InitADC(&gAs5600B); ///< Initialize the ADC driver for the back AS5600 sensor
    ///<--------------------------------------------------
    
    ///<-------------- Initialize the TM151 sensor ------
    tm151_init(&myUART, TM151_UART_BAUDRATE, TM151_BUFFER_SIZE, TM151_UART_TX, TM151_UART_RX); ///< Initialize the TM151 sensor
    ///<--------------------------------------------------

    ///<------------- Initialize the PID controllers ------
    pid_config_t pid_config = {
        .init_param = pid_paramR
    };
    pid_new_control_block(&pid_config, &pidR);

    pid_config.init_param = pid_paramL;
    pid_new_control_block(&pid_config, &pidL);

    pid_config.init_param = pid_paramB;
    pid_new_control_block(&pid_config, &pidB);
    ///<---------------------------------------------------

    control_params_t right_control_params = {
        .gStruct = &gAs5600R,
        .sensor_data = &right_encoder_data,
        .pid_block = pidR,
        .pwm_motor = &pwmR
    };

    control_params_t left_control_params = {
        .gStruct = &gAs5600L,
        .sensor_data = &left_encoder_data,
        .pid_block = pidL,
        .pwm_motor = &pwmL
    };

    control_params_t back_control_params = {
        .gStruct = &gAs5600B,
        .sensor_data = &back_encoder_data,
        .pid_block = pidB,
        .pwm_motor = &pwmB
    };

    ///<-------------- Create the task ---------------

    TaskHandle_t xRightEncoderTaskHandle, xLeftEncoderTaskHandle, xBackEncoderTaskHandle; ///< Task handles for encoders
    /*, xIMUTaskHandle = NULL, xLidarTaskHandle = NULL*/ ///< Task handles
    xTaskCreate(vTaskEncoder, "right_encoder_task", 2048, &right_control_params, 9, &xRightEncoderTaskHandle); ///< Create the task to read from encoder
    xTaskCreate(vTaskEncoder, "left_encoder_task", 2048, &left_control_params, 9, &xLeftEncoderTaskHandle); ///< Create the task to read from encoder
    xTaskCreate(vTaskEncoder, "back_encoder_task", 2048, &back_control_params, 9, &xBackEncoderTaskHandle); ///< Create the task to read from encoder
    configASSERT(xRightEncoderTaskHandle); ///< Check if the task was created successfully
    if (xRightEncoderTaskHandle == NULL) {
        ESP_LOGE("ENCODER_TASK", "Failed to create task...");
        return;
    }
    configASSERT(xLeftEncoderTaskHandle); ///< Check if the task was created successfully
    if (xLeftEncoderTaskHandle == NULL) {
        ESP_LOGE("ENCODER_TASK", "Failed to create task...");
        return;
    }
    configASSERT(xBackEncoderTaskHandle); ///< Check if the task was created successfully
    if (xBackEncoderTaskHandle == NULL) {
        ESP_LOGE("ENCODER_TASK", "Failed to create task...");
        return;
    }

    // xTaskCreate(vTaskIMU, "imu_task", 2048, NULL, 9, &xIMUTaskHandle); ///< Create the task to read from IMU
    // configASSERT(xIMUTaskHandle); ///< Check if the task was created successfully
    // if (xIMUTaskHandle == NULL) {
    //     ESP_LOGE("IMU_TASK", "Failed to create task...");
    //     return;
    // }

    // xTaskCreate(vTaskLidar, "lidar_task", 2048, NULL, 9, &xLidarTaskHandle); ///< Create the task to read from Lidar
    // configASSERT(xLidarTaskHandle); ///< Check if the task was created successfully
    // if (xLidarTaskHandle == NULL) {
    //     ESP_LOGE("LIDAR_TASK", "Failed to create task...");
    //     return;
    // }

    TaskHandle_t xRightControlTaskHandle, xLeftControlTaskHandle, xBackControlTaskHandle; ///< Task handles for control tasks
    xTaskCreate(vTaskControl, "rwh_control_task", 4096, &right_control_params, 9, &xRightControlTaskHandle); ///< Create the task to control the wheel
    xTaskCreate(vTaskControl, "lwh_control_task", 4096, &left_control_params,  9, &xLeftControlTaskHandle); ///< Create the task to control the wheel
    xTaskCreate(vTaskControl, "bwh_control_task", 4096, &back_control_params,  9, &xBackControlTaskHandle); ///< Create the task to control the wheel
    configASSERT(xRightControlTaskHandle); ///< Check if the task was created successfully
    if (xRightControlTaskHandle == NULL) {
        ESP_LOGE("CTRL_TASK", "Failed to create task...");
        return;
    }
    configASSERT(xLeftControlTaskHandle); ///< Check if the task was created successfully
    if (xLeftControlTaskHandle == NULL) {
        ESP_LOGE("CTRL_TASK", "Failed to create task...");
        return;
    }
    configASSERT(xBackControlTaskHandle); ///< Check if the task was created successfully
    if (xBackControlTaskHandle == NULL) {
        ESP_LOGE("CTRL_TASK", "Failed to create task...");
        return;
    }
    ///<--------------------------------------------------

}