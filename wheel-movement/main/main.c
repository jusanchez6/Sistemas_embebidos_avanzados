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
#include "bldc_pwm.h"

#include <assert.h>
#include "esp_partition.h"
#include "spi_flash_mmap.h"

#include "freertos/task.h"

/* ---------------------------------------------------------------
 * Main mode: 0 = No Calibration, 1 = ESC Calibration
 * ---------------------------------------------------------------
 */
#define MAIN_MODE 0

/* ---------------------------------------------------------------
 * AS5600 configuration
 * ---------------------------------------------------------------
 */
#define I2C_MASTER_SCL_GPIO 4       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_GPIO 5       /*!< gpio number for I2C master data  */
#define AS5600_OUT_GPIO 6           /*!< gpio number for OUT signal */
#define I2C_MASTER_NUM 1            /*!< I2C port number for master dev */
#define AS5600_MODE 1         /*!< Calibration = 0, Angle through ADC = 1 */

/* ---------------------------------------------------------------
 * BLDC configuration
 * ---------------------------------------------------------------
 */
#define PWM_GPIO 3 ///< GPIO number for PWM signal
#define PWM_REV_GPIO 8 ///< GPIO number for PWM reverse signal
#define PWM_FREQ 50 ///< PWM frequency in Hz
#define PWM_RESOLUTION 100000 ///< PWM resolution in bits
#define MAX_PWM_CAL 120 ///< Maximum PWM value
#define MIN_PWM_CAL 35 ///< Minimum PWM value
#define MAX_PWM_RE 119 ///< Maximum PWM value (moves fully)
#define MIN_PWM_RE 38 ///< Minimum PWM value (does not move)

#if MAIN_MODE == 0
bldc_pwm_motor_t pwm, pwm2;

// AS5600_t gAs5600;

// float angle; ///< Angle read from the AS5600 sensor
// uint32_t pwm_list = {0, 200, 500, 700, 850, 1000}; ///< List of PWM values to be used in the simulation
// uint8_t pwm_index = 0; ///< Index of the PWM value to be used in the simulation

void app_main(void)
{

    ESP_LOGI("PWM", "Starting test..."); ///< Log message

    bldc_init(&pwm, PWM_GPIO, PWM_REV_GPIO, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor

    bldc_enable(&pwm); ///< Enable the BLDC motor

    bldc_set_duty(&pwm, 0); ///< Set the duty cycle to 0%

    vTaskDelay(5000 / portTICK_PERIOD_MS); ///< Wait for 15 seconds    

    int16_t duty = 0; ///< Duty cycle variable
    bool reverse = false; ///< Reverse variable
    while (1)
    {
        ESP_LOGI("PWM", "ESC running!"); ///< Log message

        if (abs(duty) > 50) ///< If the duty cycle is greater than 100%
        {
            if(reverse) duty = -1; ///< Set the duty cycle to 0%
            else duty = 1; ///< Set the duty cycle to 0%
            reverse = !reverse; ///< Reverse the direction
            bldc_set_duty(&pwm, duty); ///< Set the duty cycle to 0%
            vTaskDelay(2000 / portTICK_PERIOD_MS); ///< Wait for 5 seconds
        }
        if (reverse) { ///< If the direction is reversed
            duty -= 5; ///< Increase the reverse duty cycle
            ESP_LOGI("PWM", "Motor running in reverse! PWM: %hd.", duty); ///< Log message
            bldc_set_duty(&pwm, duty); ///< Set the duty cycle to 0%
        } else {
            duty += 5; ///< Increase the forward duty cycle
            ESP_LOGI("PWM", "Motor running in forward! PWM: %hd.", duty); ///< Log message
            bldc_set_duty(&pwm, duty); ///< Set the duty cycle to the current PWM value
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); ///< Wait for 2 seconds
    }


    // bldc_init(&pwm, PWM_GPIO, PWM_REV_GPIO, PWM_FREQ, 0, PWM_RESOLUTION); ///< Initialize the BLDC motor
    // ESP_LOGI("BLDC", "BLDC initialaized!");
    // bldc_enable(&pwm); ///< Enable the BLDC motor
    // bldc_set_duty_old(&pwm, 1000); ///< Set the duty cycle to 100%

    // vTaskDelay(10000/portTICK_PERIOD_MS); ///< Wait 5 second

    // bldc_set_duty_old(&pwm, 0); ///< Set the duty cycle to 0%

    // vTaskDelay(10000/portTICK_PERIOD_MS); ///< Wait 5 second

    // ESP_LOGI("BLDC", "ESC calibrated!"); ///< Print to the console

//     // // Find the partition map in the partition table
//     // const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
//     // size_t i = 0;
//     // assert(partition != NULL);

//     // static char store_data[] = "";
//     // static char read_data[sizeof(store_data)];

//     // // Erase entire partition
//     // memset(read_data, 0xFF, sizeof(read_data));
//     // ESP_ERROR_CHECK(esp_partition_erase_range(partition, 0, partition->size));
    
       

//     // for (uint16_t current_timestamp = 0; current_timestamp < SIM_SECS * 1000; current_timestamp += TS)
//     // {
//     //     if(current_timestamp % 1000 == 0) ///< Every second
//     //     {
//     //         bldc_set_duty_old(&pwm, pwm_list[current_timestamp / 1000]); ///< Set the duty cycle to the current PWM value
//     //     }
//     while(1){

//         ///< ------------- Get angle through ADC -------------
//         angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC
//         ///<--------------------------------------------------

//         // Write the data, starting from the beginning of the partition
//         // ESP_ERROR_CHECK(esp_partition_write(partition, 0+i, store_data, sizeof(store_data)));
//         // ESP_LOGI("ESP_PARTITION", "Written data: %s", store_data);
//         // i += sizeof(float);

//         ESP_LOGI("RET_DATA", "Angle: %.2f, PWM: %i", angle, 100); ///< Print the angle to the console

//         vTaskDelay(TS/portTICK_PERIOD_MS); ///< Wait 1 second
//     }

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