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
    APDS9960_set_mode(&apds9960, APDS9960_AEN_ENABLE);
    APDS9960_set_gain(&apds9960, APDS9960_AGAIN_64X);


    uint16_t r, g, b;

    while (1) {
        APDS9960_get_RGB(&apds9960, &r, &g, &b, true);
        printf("R: %d, G: %d, B: %d\n", r, g, b);

        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }


}
