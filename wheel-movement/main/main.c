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

#include "freertos/task.h"

#include "as5600_lib.h"

#define I2C_MASTER_SCL_GPIO 4       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_GPIO 5       /*!< gpio number for I2C master data  */
#define AS5600_OUT_GPIO 6           /*!< gpio number for OUT signal */
#define I2C_MASTER_NUM 1            /*!< I2C port number for master dev */
#define AS5600_MODE 1         /*!< Calibration = 0, Angle through ADC = 1 */

AS5600_t gAs5600;

float angle; ///< Raw angle readed from the AS5600 sensor

void app_main(void)
{

    ///< ---------------------- AS5600 -------------------
    AS5600_Init(&gAs5600, I2C_MASTER_NUM, I2C_MASTER_SCL_GPIO, I2C_MASTER_SDA_GPIO, AS5600_OUT_GPIO);

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
    printf("Configuration register readed: 0x%04X\n", conf_reg);
    printf("Configuration register written: 0x%04X\n", conf.WORD);

    ///< ------------- Get angle through ADC -------------

    while (1)
    {
        AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC
        vTaskDelay(1000/portTICK_PERIOD_MS); ///< Wait 1 second
    }
    
    

}