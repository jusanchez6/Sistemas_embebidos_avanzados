#include <stdio.h>


#include "apds9960.h"

#define I2C_MASTER_SCL_IO 17
#define I2C_MASTER_SDA_IO 18
#define I2C_MASTER_NUM I2C_NUM_0


void app_main(void)
{
    APDS9960_t apds9960 = {};
    APDS9960_init(&apds9960, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);    


}
