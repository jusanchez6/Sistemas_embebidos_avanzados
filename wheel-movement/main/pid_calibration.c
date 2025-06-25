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
#include "freertos/semphr.h"

#define SAMPLE_TIME 10 ///< Sample time in ms

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
#define PID_KP .01f//.01f//1.66f
#define PID_KI .01f//0.03f//0.6f
#define PID_KD .001f//0.1f//0.05f /*1.8f*/
#define EULER 2.71828
#define PI 3.14159
#define SAMPLE_RATE 100 // Hz (10ms per sample)
///<--------------------------------------------------

bldc_pwm_motor_t pwm; ///< BLDC motor object
AS5600_t gAs5600;     ///< AS5600 object for angle sensor
float angle;          ///< Angle read from the AS5600 sensor
int16_t duty;

static volatile pid_block_handle_t pid; // PID control block handle

pid_parameter_t pid_param = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .max_output = 60.0f,
    .min_output = -60.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};


imu_data_t imu_data;
encoder_data_t encoder_data;
lidar_data_t lidar_data;

/**
 * @brief Task to control the wheel
 * 
 * @param pvParameters 
 */
void control_task( void * pvParameters );

/**
 * @brief UART task to read data from console
 * 
 * @param pvParameters 
 */
void uart_task(void * pvParameters);

#define UART_RX_BUFFER_SIZE 256
#define UART_QUEUE_SIZE     10

static QueueHandle_t uart_queue;
static char uart_buffer[UART_RX_BUFFER_SIZE];
static volatile int uart_buffer_index = 0;
TaskHandle_t xHandleParserTask;
SemaphoreHandle_t xPidMutex = NULL; // Mutex for PID control


void uart_init_task() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, UART_RX_BUFFER_SIZE * 2, 0, UART_QUEUE_SIZE, &uart_queue, 0);
    uart_flush(UART_NUM_0);
    uart_enable_rx_intr(UART_NUM_0);
}

void vTaskUartHandler(void *arg) {
    uart_event_t event;
    char d;

    while (1) {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                for (int i = 0; i < event.size; ++i) {
                    if (uart_read_bytes(UART_NUM_0, (uint8_t *)&d, 1, pdMS_TO_TICKS(10)) > 0) {
                        if (d == '\n' || d == '\r') {
                            uart_buffer[uart_buffer_index] = '\0';
                            uart_buffer_index = 0;
                            // Notify parser task
                            xTaskNotifyGive(xHandleParserTask);
                        } else if (uart_buffer_index < UART_RX_BUFFER_SIZE - 1) {
                            uart_buffer[uart_buffer_index++] = d;
                        }
                    }
                }
            }
        }
    }
}

void vTaskUartParser(void *arg) {
    float kp, ki, kd, setpoint;
    char parsed[128];

    xHandleParserTask = xTaskGetCurrentTaskHandle();

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        char *start = strstr(uart_buffer, "\"default\":\"");
        if (start) {
            start += strlen("\"default\":\"");
            char *end = strchr(start, '"');
            if (end && (end - start) < sizeof(parsed)) {
                strncpy(parsed, start, end - start);
                parsed[end - start] = '\0';

                if (sscanf(parsed, "%f %f %f %f", &kp, &ki, &kd, &setpoint) == 4) {
                    if (xSemaphoreTake(xPidMutex, portMAX_DELAY) == pdTRUE) {
                        pid_param.kp = kp;
                        pid_param.ki = ki;
                        pid_param.kd = kd;
                        pid_param.set_point = setpoint;

                        pid_update_parameters(pid, &pid_param);
                        xSemaphoreGive(xPidMutex);

                        printf("Updated PID: kp=%.2f, ki=%.2f, kd=%.2f, setpoint=%.2f\n", kp, ki, kd, setpoint);
                    } else {
                        printf("PID mutex busy.\n");
                    }
                } else {
                    printf("Invalid format inside 'default'.\n");
                }
            } else {
                printf("Value too long or malformed.\n");
            }
        } else {
            printf("Expected format: {\"default\":\"kp ki kd setpoint\"}\n");
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

    AS5600_SetStartPosition(&gAs5600, 0x0000); ///< Set the start position to 0
    AS5600_SetStopPosition(&gAs5600, 0x0FFF); ///< Set the stop position to 4095

    AS5600_InitADC(&gAs5600); ///< Initialize the ADC driver
    vTaskDelay(500 / portTICK_PERIOD_MS); ///< Wait for 500 ms
    ///<--------------------------------------------------

    ///<-------------- Initialize the PID controller ------
    pid_config_t pid_config = {
        .init_param = pid_param
    };

    pid_new_control_block(&pid_config, &pid);
    ///<---------------------------------------------------

    sf_init(&imu_data, &encoder_data, &lidar_data); ///< Initialize the sensor fusion module

    uart_init_task(); ///< Initialize the UART task for reading data from console

    xPidMutex = xSemaphoreCreateMutex();

    ///<-------------- Create the task ---------------
    TaskHandle_t xControlTaskHandle = NULL, xUartTaskHandle = NULL; ///< Task handles
    xTaskCreate(control_task, "control_task", 4096, NULL, 10, &xControlTaskHandle); ///< Create the task to control the wheel
    configASSERT(xControlTaskHandle); ///< Check if the task was created successfully
    if (xControlTaskHandle == NULL) {
        ESP_LOGE("CTRL_TASK", "Failed to create task...");
        return;
    }

    // xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, &xUartTaskHandle); ///< Create the task to read data from console
    // configASSERT(xUartTaskHandle); ///< Check if the task was created successfully
    // if (xUartTaskHandle == NULL) {
    //     ESP_LOGE("UART_TASK", "Failed to create task...");
    //     return;
    // }

    // Start the UART tunning the pid parameters task
    xTaskCreate(vTaskUartHandler, "uart_handler", 4096, NULL, 10, NULL);
    xTaskCreate(vTaskUartParser, "uart_parser", 4096, NULL, 10, &xHandleParserTask);

    ///<--------------------------------------------------

}

// Task to control the wheel
void control_task( void * pvParameters ){

    float acceleration[3];

    uint32_t timestamp = 1000000; // 1 second
    float est_velocity = 0.0f, last_est_velocity = 0.0f;
    float beta = exp(-2 * PI * 1 / SAMPLE_RATE);  // 10Hz cutoff frequency
    float output = 0.0f;

    while (1)
    {
        ///<-------------- Get angle through ADC -------------
        angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC
        estimate_velocity_encoder(&encoder_data, angle, 2 / 1000.0f); ///< Estimate the velocity using encoder data
        // ///<--------------------------------------------------

        ///<-------------- PID Control ---------------
        // Low-pass filter
        est_velocity = encoder_data.velocity;//estimate_velocity(imu_data.velocity, lidar_data.velocity, encoder_data.velocity); ///< Update the estimated velocity

        // est_velocity = beta * last_est_velocity + (1 - beta) * est_velocity;
        // last_est_velocity = est_velocity;

        // Update PID Controller
        pid_compute(pid, est_velocity, &output);
        bldc_set_duty(&pwm, output); ///< Set the duty cycle to the output of the PID controller
        
        // // printf("I,%" PRIu32 ",%hd,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp, distance, est_velocity, pid_param.set_point, output, 0.0, 0.0);
        if(timestamp%100000 == 0) { ///< Print every 5 iterations
            printf("I,%" PRIu32 ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp, est_velocity, pid_param.set_point, 0.0, output, 0.0, 0.0);
        }

        timestamp += 2000;  // 100Hz
        ///<------------------------------------------
        

        vTaskDelay(2 / portTICK_PERIOD_MS); ///< Wait for 10 ms
    }
}

// Task to read data from console
void uart_task(void* pvParameters) {
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

            case 'S': ///< Change the setpoint
                sscanf(data, "S %f", &pid_param.set_point);
                pid_update_parameters(pid, &pid_param);
                printf("Updated setpoint: %.2f\n", pid_param.set_point);
                break;
            
            default:
                break;
            }
        }
    }
}