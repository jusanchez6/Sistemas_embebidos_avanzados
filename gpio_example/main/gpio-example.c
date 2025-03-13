#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

gpio_config_t io_conf = {
    .pin_bit_mask = 1ULL << GPIO_NUM_18,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};


void app_main(void)
{
    gpio_config(&io_conf);
    while (1) {
        gpio_set_level(GPIO_NUM_18, 1);
        vTaskDelay(10);
        gpio_set_level(GPIO_NUM_18, 0);
        vTaskDelay(10);
    }

}
