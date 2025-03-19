#include <stdio.h>
#include "esp_log.h"



class AS5600 {
public:
    AS5600(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl) {
        // Constructor
    }

    esp_err_t begin() {
        // Initialize the AS5600 driver
        return ESP_OK;
    }

private:
    i2c_port_t i2c_num;
    gpio_num_t sda;
    gpio_num_t scl;
};







extern "C" {
    void app_main(void);
}


void app_main(void)
{
    AS5600 as5600(I2C_NUM_0, GPIO_NUM_17, GPIO_NUM_18);
    esp_err_t ret = as5600.begin();
    if (ret != ESP_OK) {
        ESP_LOGE("app_main", "Error initializing AS5600 driver");
        return;
    }

    while (1) {
        uint16_t angle = as5600.readRawAngle();
        ESP_LOGI("app_main", "Angle: %d", angle);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}
