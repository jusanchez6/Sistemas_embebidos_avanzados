/**
 * @file sensorRGB.c
 * 
 * @brief Sensor RGB simple usage example
 * 
 * This file contains a simple example of how to use the APDS9960 sensor to read RGB values.
 * 
 * @authors Julian Sanchez
 *          Angel Graciano
 *          Nelson Parra
 * 
 * 
 * @date 08-04-2025
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


    

    while (1) {
        uint16_t r, g, b;
        APDS9960_get_RGB(&apds9960, &r, &g, &b, true);
        printf("R: %d, G: %d, B: %d\n", r, g, b);

        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }

    APDS9960_disable(&apds9960); // Disable the sensor


}
