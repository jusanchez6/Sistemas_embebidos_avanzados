/**
 * @file main.c
 * 
 * @brief main file for obtaining data from sensors and control the wheel
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
 * @version 4.2
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

///<---------- Main mode: ------------------------
#define MAIN_MODE 0 ///< 0 = No Calibration, 1 = ESC Calibration, 2 AS5600 PROBE
///<---------------------------------------------

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
#define PID_KP .01f
#define PID_KI .01f
#define PID_KD .001f
#define EULER 2.71828
#define PI 3.14159
///<--------------------------------------------------

#if MAIN_MODE == 0

bldc_pwm_motor_t pwm; ///< BLDC motor object
AS5600_t gAs5600;     ///< AS5600 object for angle sensor
vl53l1x_t gVl53l1x;   ///< VL53L1X object for distance sensor
uart_t myUART;        ///< UART object for TM151 IMU
float angle;          ///< Angle read from the AS5600 sensor
uint16_t distance, goal_distance;     ///< Distance read from the VL53L1X sensor
bool reached_goal = true, move_one_time = false; ///< Flag to indicate if the goal distance is reached
int16_t duty;

static volatile pid_block_handle_t pid; // PID control block handle

uint32_t temp_ctr = 0; ///< Temporary counter to stop move_one_time

typedef struct {
    float encoder_velocity; ///< Velocity estimation from encoder in cm/s
    float imu_velocity;     ///< Velocity estimation from IMU in cm/s
    float lidar_velocity;   ///< Velocity estimation from Lidar in cm/s

    float encoder_distance; ///< Distance estimation from encoder in cm
} control_params_t;

pid_parameter_t pid_param = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

imu_data_t imu_data;
encoder_data_t encoder_data;
lidar_data_t lidar_data;

control_params_t control_params = {
    .encoder_velocity = 0.0f,
    .imu_velocity = 0.0f,
    .lidar_velocity = 0.0f,
    .encoder_distance = 0.0f
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


    ///<----------- Initialize the BLDC motor PWM --------
    ESP_LOGI("PWM", "Starting test..."); ///< Log message
    bldc_init(&pwm, PWM_GPIO, PWM_REV_GPIO, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwm); ///< Enable the BLDC motor
    bldc_set_duty(&pwm, 0); ///< Set the duty cycle to 0%
    ///<--------------------------------------------------

    vTaskDelay(4000 / portTICK_PERIOD_MS); ///< Wait for 4 seconds

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
        .WD = AS5600_WATCHDOG_OFF, ///< Watchdog on
    };
    AS5600_SetConf(&gAs5600, conf);
    
    // Read the configuration
    uint16_t conf_reg;
    AS5600_ReadReg(&gAs5600, AS5600_REG_CONF_H, &conf_reg);
    if (conf_reg != conf.WORD) {
        ESP_LOGI(TAG_AS5600, "AS5600 configuration failed");
    }
    else {
        ESP_LOGI(TAG_AS5600, "AS5600 configuration successful");
    }


    AS5600_SetStartPosition(&gAs5600, 0x0000); ///< Set the start position to 0
    AS5600_SetStopPosition(&gAs5600, 0x0FFF); ///< Set the stop position to 4095

    AS5600_InitADC(&gAs5600); ///< Initialize the ADC driver
    vTaskDelay(500 / portTICK_PERIOD_MS); ///< Wait for 500 ms
    ///<--------------------------------------------------
    
    ///<-------------- Initialize the TM151 sensor ------
    tm151_init(&myUART, TM151_UART_BAUDRATE, TM151_BUFFER_SIZE, TM151_UART_TX, TM151_UART_RX); ///< Initialize the TM151 sensor
    ///<--------------------------------------------------

    ///<-------------- Initialize the PID controller ------
    pid_config_t pid_config = {
        .init_param = pid_param
    };

    pid_new_control_block(&pid_config, &pid);
    ///<---------------------------------------------------

    sf_init(&imu_data, &encoder_data, &lidar_data); ///< Initialize the sensor fusion module

    ///<-------------- Create the task ---------------

    TaskHandle_t xEncoderTaskHandle = NULL, xIMUTaskHandle = NULL, xLidarTaskHandle = NULL; ///< Task handles
    xTaskCreate(vTaskEncoder, "encoder_task", 2048, NULL, 9, &xEncoderTaskHandle); ///< Create the task to read from encoder
    configASSERT(xEncoderTaskHandle); ///< Check if the task was created successfully
    if (xEncoderTaskHandle == NULL) {
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

    TaskHandle_t xControlTaskHandle = NULL, xUartTaskHandle = NULL; ///< Task handles
    xTaskCreate(vTaskControl, "control_task", 4096, NULL, 9, &xControlTaskHandle); ///< Create the task to control the wheel
    configASSERT(xControlTaskHandle); ///< Check if the task was created successfully
    if (xControlTaskHandle == NULL) {
        ESP_LOGE("CTRL_TASK", "Failed to create task...");
        return;
    }

    xTaskCreate(vTaskUART, "uart_task", 4096, NULL, 10, &xUartTaskHandle); ///< Create the task to read data from console
    configASSERT(xUartTaskHandle); ///< Check if the task was created successfully
    if (xUartTaskHandle == NULL) {
        ESP_LOGE("UART_TASK", "Failed to create task...");
        return;
    }
    ///<--------------------------------------------------

}

// Task to read from encoder
void vTaskEncoder(void * pvParameters) {
    ///<-------------- Get angle through ADC -------------
    while (1) {
        angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC
        estimate_velocity_encoder(&encoder_data, angle, SAMPLE_TIME / 1000.0f); ///< Estimate the velocity using encoder data
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

    uint32_t timestamp = 1000000; // 1 second
    float est_velocity = 0.0f, last_est_velocity = 0.0f;
    float beta = exp(-2 * PI * 1 / 100);  // 10Hz cutoff frequency
    float output = 0.0f;

    while (1)
    {
        ///<-------------- PID Control ---------------
        // Low-pass filter
        est_velocity = encoder_data.velocity;//estimate_velocity(imu_data.velocity, lidar_data.velocity, encoder_data.velocity); ///< Update the estimated velocity

        // est_velocity = beta * last_est_velocity + (1 - beta) * est_velocity; ///< Apply low-pass filter to the estimated velocity when there is more than one sensor
        last_est_velocity = est_velocity; ///< Update the last estimated velocity

        // Update PID Controller
        pid_compute(pid, est_velocity, &output);
        bldc_set_duty(&pwm, output); ///< Set the duty cycle to the output of the PID controller
        
        if(timestamp % 100000 == 0) { ///< Print every 100ms to debug with IMU software
            // printf("I,%" PRIu32 ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp, est_velocity, pid_param.set_point, 0.0, output, 0.0, 0.0);
        }

        timestamp += 2000;  // 2ms in us
        ///<------------------------------------------

        ///<-------------- Logic to process the data ------
        if (move_one_time) {
            if (temp_ctr < 2000) {
                temp_ctr += SAMPLE_TIME; ///< Increment the temporary counter
                bldc_set_duty(&pwm, duty);
                // printf("Updated duty cycle: %hd\n", duty);
            } else {
                temp_ctr = 0; ///< Reset the temporary counter
                bldc_set_duty(&pwm, 0);
                encoder_data.estimate = 0; ///< Reset the flag
                move_one_time = false; ///< Reset the flag
            }
        }

        float est_dist = encoder_data.distance /*+ fabsf(distance - lidar_data.start_distance)) * 0.5f*/;
        // printf("DIST Encoder: %.2f cm\t Lidar: %hu cm\tEstimated: %.2f cm\n",
        //         encoder_data.distance, fabsf(distance - lidar_data.start_distance), est_dist); ///< Log message

        if ((est_dist > goal_distance && !reached_goal) /*|| distance < 70 || distance > 2000*/) {
            // bldc_set_duty(&pwm, 0); ///< Stop the motor
            pid_param.set_point = 0.0f; ///< Set the setpoint to 0
            pid_update_parameters(pid, &pid_param);

            reached_goal = true; ///< Set the flag to true

            printf("Goal distance reached: %.2f mm\n", encoder_data.distance); ///< Log message
        }
        ///<--------------------------------------------
        
        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 2 ms
    }
}

// Task to read data from console
void vTaskUART(void* pvParameters) {
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
            

            switch (data[0])
            {
            case 'P': ///< Change the PID parameters
                sscanf(data, "P %f %f %f %f", &pid_param.kp, &pid_param.ki, &pid_param.kd, &pid_param.set_point);
                pid_update_parameters(pid, &pid_param);
                printf("Updated PID values: kp=%.2f, ki=%.2f, kd=%.2f, setpoint=%.2f\n", pid_param.kp, pid_param.ki, pid_param.kd, pid_param.set_point);
                break;
            
            case 'X': ///< Change the duty cycle
                sscanf(data, "X %hd", &duty);
                move_one_time = true; ///< Set the flag to move a bit
                encoder_data.distance = 0; ///< Set the distance to 0
                encoder_data.last_vel = 0; ///< Reset the last velocity
                encoder_data.estimate = 1; ///< Set the flag to estimate the distance
                break;

            case 'S': ///< Change the setpoint
                sscanf(data, "S %f", &pid_param.set_point);
                pid_update_parameters(pid, &pid_param);
                printf("Updated setpoint: %.2f\n", pid_param.set_point);
                break;

            case 'D': ///< Move to the right (derecha)
                encoder_data.estimate = 1; ///< Set the flag to estimate the distance
                sscanf(data, "D%hu_%f", &goal_distance, &pid_param.set_point);
                pid_update_parameters(pid, &pid_param);
                lidar_data.start_distance = distance; ///< Set the start distance
                lidar_data.prev_distance = distance; ///< Set the previous distance
                encoder_data.distance = 0; ///< Set the distance to 0
                reached_goal = false; ///< Reset the flag
                encoder_data.distance = 0; ///< Set the distance to 0
                encoder_data.last_vel = 0; ///< Reset the last velocity
                printf("Moving to the right with goal distance: %hucm and velocity: %.2fcm/s\n", goal_distance, pid_param.set_point);
                break;
            
            case 'I': ///< Move to the left (izquierda)
                encoder_data.estimate = 1; ///< Set the flag to estimate the distance
                sscanf(data, "I%hu_%f", &goal_distance, &pid_param.set_point);
                pid_param.set_point = -pid_param.set_point;
                pid_update_parameters(pid, &pid_param);
                lidar_data.start_distance = distance; ///< Set the start distance
                lidar_data.prev_distance = distance; ///< Set the previous distance
                encoder_data.distance = 0; ///< Set the distance to 0
                reached_goal = false; ///< Reset the flag
                encoder_data.distance = 0; ///< Set the distance to 0
                encoder_data.last_vel = 0; ///< Reset the last velocity
                printf("Moving to the left with goal distance: %hucm and velocity: %.2fcm/s\n", goal_distance, -pid_param.set_point);
                break;
            
            default:
                break;
            }
        }
    }
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


#if MAIN_MODE == 3

const char *TAG = "AS5600";
AS5600_t gAs5600;     ///< AS5600 object for angle sensor



void app_main(void)
{

    ///< ---------------------- AS5600 -------------------
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
    

    ESP_LOGI(TAG, "Configuration: 0x%04X", conf.WORD); ///< Read the configuration register
    
    // Read the configuration
    uint16_t conf_reg;
    AS5600_ReadReg(&gAs5600, AS5600_REG_CONF_H, &conf_reg);

    
    if (conf_reg != conf.WORD)
    {
        ESP_LOGE(TAG, "Configuration error: 0x%04X", conf_reg); ///< Configuration error
    }
    else
    {
        ESP_LOGI(TAG, "Configuration OK"); ///< Configuration OK
    }
    
    vTaskDelay(10000 / portTICK_PERIOD_MS); ///< Delay 1 second
    // calibration:

    AS5600_SetStartPosition(&gAs5600, 0); ///< Set the start position to 0 degrees
    AS5600_SetStopPosition(&gAs5600, 4095); ///< Set the end position to 360 degrees
    AS5600_InitADC(&gAs5600); ///< Initialize the ADC driver

    vTaskDelay(10000 / portTICK_PERIOD_MS); ///< Delay 1 second
    
    while (true)
    {
        ESP_LOGI(TAG, "AS5600 angle: %0.2f", AS5600_ADC_GetAngle(&gAs5600)); ///< Get the angle from the ADC
        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); ///< Delay 1 second
    }
    
}



#endif