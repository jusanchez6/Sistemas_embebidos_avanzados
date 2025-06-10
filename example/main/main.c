#include <stdio.h>

#include "as5600_lib.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


AS5600_t gAs5600;

#define SAMPLE_TIME 1000 

///<-------------- AS5600 configuration ---------------
#define AS5600_I2C_MASTER_SCL_GPIO 5    ///< gpio number for I2C master clock
#define AS5600_I2C_MASTER_SDA_GPIO 4    ///< gpio number for I2C master data 
#define AS5600_OUT_GPIO 7               ///< gpio number for OUT signal
#define AS5600_I2C_MASTER_NUM 0         ///< I2C port number for master dev
#define AS5600_MODE 1                   ///< Calibration = 0, Angle through ADC = 1
///<--------------------------------------------------

float angle; ///< Angle variable

uint16_t angle_i2c; ///< Angle variable for I2C

const char *TAG = "AS5600";

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


    // AS5600_ReadReg(&gAs5600, AS5600_REG_STATUS, &conf_reg); ///< Read the MANG register
    // ESP_LOGI(TAG, "STATUS: 0x%04X", conf_reg); ///< Read the MANG register

    // calibration:

    AS5600_SetStartPosition(&gAs5600, 0); ///< Set the start position to 0 degrees
    AS5600_SetStopPosition(&gAs5600, 4094); ///< Set the end position to 360 degrees

    AS5600_InitADC(&gAs5600); ///< Initialize the ADC driver

    vTaskDelay(5000 / portTICK_PERIOD_MS); ///< Delay 1 second
    
    while (true)
    {
        angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC

        AS5600_GetAngle(&gAs5600, &angle_i2c);

        ESP_LOGI(TAG, "AS5600 ADC angle: %0.2f", angle); ///< Get the angle from the ADC
        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); ///< Delay 1 second
    }
    
}
