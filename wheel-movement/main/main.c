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

#include <assert.h>
#include "esp_partition.h"
#include "spi_flash_mmap.h"

#define I2C_MASTER_SCL_GPIO 4       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_GPIO 5       /*!< gpio number for I2C master data  */
#define AS5600_OUT_GPIO 6           /*!< gpio number for OUT signal */
#define I2C_MASTER_NUM 1            /*!< I2C port number for master dev */
#define AS5600_MODE 1         /*!< Calibration = 0, Angle through ADC = 1 */

AS5600_t gAs5600;

float angle; ///< Angle read from the AS5600 sensor

void app_main(void)
{

    // Find the partition map in the partition table
    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    size_t i = 0;
    assert(partition != NULL);

    static char store_data[] = "";
    static char read_data[sizeof(store_data)];

    // Erase entire partition
    memset(read_data, 0xFF, sizeof(read_data));
    ESP_ERROR_CHECK(esp_partition_erase_range(partition, 0, partition->size));

    

    ///< ------------- Get angle through ADC -------------

    while (1)
    {
        angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC

        // Write the data, starting from the beginning of the partition
        ESP_ERROR_CHECK(esp_partition_write(partition, 0+i, store_data, sizeof(store_data)));
        ESP_LOGI("ESP_PARTITION", "Written data: %s", store_data);
        i += sizeof(float);

        vTaskDelay(1000/portTICK_PERIOD_MS); ///< Wait 1 second
    }

}