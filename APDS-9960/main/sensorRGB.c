#include <stdio.h>


#include "apds9960.h"

#define I2C_MASTER_SCL_IO 17
#define I2C_MASTER_SDA_IO 18
#define I2C_MASTER_NUM I2C_NUM_0


void app_main(void)
{
    APDS9960_t apds9960 = {};
    APDS9960_init(&apds9960, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);    

    // Set the mode to enable ALS and Proximity detection
    APDS9960_set_mode(&apds9960, APDS9960_AEN_ENABLE | APDS9960_PEN_ENABLE | APDS9960_PIEN_ENABLE);

    APDS9960_set_ambient_light_interrupt_threshold(&apds9960, 10, APDS9960_PERS_4);

    uint16_t r, g, b;

    while (1) {
        APDS9960_get_RGB(&apds9960, &r, &g, &b);
        printf("R: %d, G: %d, B: %d\n", r, g, b);

        // PROXIMITY
        uint8_t proximity;
        APDS9960_read_proximity(&apds9960, &proximity);
        printf("Proximity: %d\n", proximity);


        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }


}
