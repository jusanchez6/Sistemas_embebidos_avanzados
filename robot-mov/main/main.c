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

#define SAMPLE_TIME 2 ///< Sample time in ms
#define WHEEL_RADIO 3.0f ///< Radio of the wheel in cm

///<-------------- AS5600 configuration ---------------
#define AS5600_I2C_MASTER_SCL_GPIO 5    ///< gpio number for I2C master clock
#define AS5600_I2C_MASTER_SDA_GPIO 4    ///< gpio number for I2C master data 
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
#define VL53L1X_SDA_GPIO 41     ///< gpio number for I2C master data 
#define VL53L1X_SCL_GPIO 42     ///< gpio number for I2C mastes clock
///<--------------------------------------------------

///<-------------- BLDC configuration -----------------
#define PWM_GPIO_R 20               ///< GPIO number for right PWM signal
#define PWM_REV_GPIO_R 21           ///< GPIO number for right PWM reverse signal

#define PWM_GPIO_L 22               ///< GPIO number for left PWM signal
#define PWM_REV_GPIO_L 23           ///< GPIO number for left PWM reverse signal

#define PWM_GPIO_B 24               ///< GPIO number for back PWM signal
#define PWM_REV_GPIO_B 25           ///< GPIO number for back PWM reverse signal

#define PWM_FREQ 50                 ///< PWM frequency in Hz
#define PWM_RESOLUTION 100000       ///< PWM resolution in bits
#define MAX_PWM_CAL 120             ///< Maximum PWM value
#define MIN_PWM_CAL 35              ///< Minimum PWM value
#define MAX_PWM_RE 119              ///< Maximum PWM value (moves fully)
#define MIN_PWM_RE 38               ///< Minimum PWM value (does not move)
///<--------------------------------------------------

///<-------------- PID configuration -----------------
#define PID_KP .01f
#define PID_KI .01f
#define PID_KD .001f
#define EULER 2.71828
#define PI 3.14159
///<--------------------------------------------------

bldc_pwm_motor_t pwmR, pwmL, pwmB;      ///< BLDC motor object right, left and back
AS5600_t gAs5600R, gAs5600L, gAs5600B;  ///< AS5600 object for angle sensor right, left and back
vl53l1x_t gVl53l1x;                     ///< VL53L1X object for distance sensor
uart_t myUART;                          ///< UART object for TM151 IMU
uint16_t distance, goal_distance;       ///< Distance read from the VL53L1X sensor

bool reached_goal = true, move_one_time = false; ///< Flag to indicate if the goal distance is reached
int16_t duty;

pid_block_handle_t pidR, pidL, pidB; // PID control block handle

uint32_t temp_ctr = 0; ///< Temporary counter to stop move_one_time

imu_data_t imu_data = {
    .velocity = 0.0f,         ///< Velocity in cm/s
    .prev_acc = 0.0f,         ///< Previous acceleration values
    .window = {}          ///< Window for sampling
};         ///< IMU data structure

encoder_data_t right_encoder_data = {
    .velocity = 0.0f,         ///< Velocity in cm/s
    .last_vel = 0.0f,         ///< Last velocity in cm/s
    .angle_prev = 0.0f,       ///< Previous angle in degrees
    .radio = WHEEL_RADIO,     ///< Radio for the wheel
    .distance = 0.0f,          ///< Distance in cm
    .time_interval = SAMPLE_TIME / 1000.0f ///< Time interval in seconds
}; ///< Encoder data structure

encoder_data_t left_encoder_data = {
    .velocity = 0.0f,         ///< Velocity in cm/s
    .last_vel = 0.0f,         ///< Last velocity in cm/s
    .angle_prev = 0.0f,       ///< Previous angle in degrees
    .radio = WHEEL_RADIO,     ///< Radio for the wheel
    .distance = 0.0f,          ///< Distance in cm
    .time_interval = SAMPLE_TIME / 1000.0f ///< Time interval in seconds
}; ///< Encoder data structure

encoder_data_t back_encoder_data = {
    .velocity = 0.0f,         ///< Velocity in cm/s
    .last_vel = 0.0f,         ///< Last velocity in cm/s
    .angle_prev = 0.0f,       ///< Previous angle in degrees
    .radio = WHEEL_RADIO,     ///< Radio for the wheel
    .distance = 0.0f,          ///< Distance in cm
    .time_interval = SAMPLE_TIME / 1000.0f ///< Time interval in seconds
}; ///< Encoder data structure

lidar_data_t lidar_data = {
    .velocity = 0.0f,         ///< Velocity in cm/s
    .prev_distance = 0,       ///< Previous distance in cm
    .start_distance = 0        ///< Start distance in cm
};     ///< Lidar data structure

typedef struct {
    void * gStruct; ///< Velocity estimation from encoder in cm/s
    void * sensor_data;     ///< Velocity estimation from IMU in cm/s
    pid_block_handle_t pid_block;   ///< Velocity estimation from Lidar in cm/s
    bldc_pwm_motor_t * pwm_motor; ///< BLDC motor object
} control_params_t;

pid_parameter_t pid_paramR = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

pid_parameter_t pid_paramL = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

pid_parameter_t pid_paramB = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

/**
 * @brief Task to read from encoder
 */
void vTaskEncoder(void * pvParameters);

/**
 * @brief Task to read from IMU
 */
void vTaskIMU(void * pvParameters);

/**
 * @brief Task to read from Lidar
 */
void vTaskLidar(void * pvParameters);

/**
 * @brief Task to control the wheel
 * 
 * @param pvParameters 
 */
void vTaskControl( void * pvParameters );

/**
 * @brief UART task to read data from console
 * 
 * @param pvParameters 
 */
void vTaskUART(void * pvParameters);

void app_main(void)
{

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
    xTaskCreate(vTaskControl, "control_task", 4096, &right_control_params, 9, &xRightControlTaskHandle); ///< Create the task to control the wheel
    xTaskCreate(vTaskControl, "left_control_task", 4096, &left_control_params, 9, &xLeftControlTaskHandle); ///< Create the task to control the wheel
    xTaskCreate(vTaskControl, "back_control_task", 4096, &back_control_params, 9, &xBackControlTaskHandle); ///< Create the task to control the wheel
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

// Task to read from encoder
void vTaskEncoder(void * pvParameters) {

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure

    ///<-------------- Get angle through ADC -------------
    while (1) {
        encoder_data->angle = AS5600_ADC_GetAngle(params->gStruct); ///< Get the angle from the ADC
        estimate_velocity_encoder(encoder_data); ///< Estimate the velocity using encoder data
        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 2 ms
    }
    ///<--------------------------------------------------
}

// Task to read from IMU
void vTaskIMU(void * pvParameters) {
    float acceleration[3];
    
    while (1) {
        // Read acceleration data from TM151 IMU
        SerialPort_DataReceived_RawAcc(&myUART, acceleration);
        
        // Estimate the velocity using IMU data
        estimate_velocity_imu(&imu_data, acceleration[0], SAMPLE_TIME / 1000.0f); ///< Estimate the velocity using IMU data
        
        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 2 ms
    }
}

// Task to read from Lidar
void vTaskLidar(void * pvParameters) {
    while (1) {
        // Read distance data from VL53L1X sensor
        distance = VL53L1X_readDistance(&gVl53l1x, 0); ///< Get the distance from the VL53L1X sensor
        estimate_velocity_lidar(&lidar_data, distance, SAMPLE_TIME / 1000.0f); ///< Estimate the velocity using lidar data
        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 2 ms
    }
}

// Task to control the wheel
void vTaskControl( void * pvParameters ){

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure

    uint32_t timestamp = 1000000; // 1 second
    float est_velocity = 0.0f, last_est_velocity = 0.0f;
    // float beta = exp(-2 * PI * 1 / 100);  // 10Hz cutoff frequency
    float output = 0.0f;

    while (1)
    {
        ///<-------------- PID Control ---------------
        // Low-pass filter
        est_velocity = encoder_data->velocity;

        // est_velocity = beta * last_est_velocity + (1 - beta) * est_velocity; ///< Apply low-pass filter to the estimated velocity when there is more than one sensor
        last_est_velocity = est_velocity; ///< Update the last estimated velocity

        // Update PID Controller
        pid_compute(params->pid_block, est_velocity, &output);
        bldc_set_duty(params->pwm_motor, output); ///< Set the duty cycle to the output of the PID controller
        
        if(timestamp % 100000 == 0) { ///< Print every 100ms to debug with IMU software
            // printf("I,%" PRIu32 ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp, est_velocity, pid_param.set_point, 0.0, output, 0.0, 0.0);
        }

        timestamp += 2000;  // 2ms in us
        ///<------------------------------------------
        
        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 2 ms
    }
}