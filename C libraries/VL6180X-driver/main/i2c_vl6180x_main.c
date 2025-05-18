/**
 * @file i2c_vl6180x_main.c
 * @brief Main implementation file for interfacing with the VL6180X sensor using I2C.
 * 
 * This file contains the main logic for initializing and communicating with the 
 * VL6180X proximity and ambient light sensor over the I2C bus. It is part of the 
 * project for the Advanced Embedded Systems course.
 * 
 * @authors Julian Sanchez
 *          Nelson Parra
 *          Angel Graciano 
 * @date 09-04-2025
 * @version 1.0
 * 
 * @note Ensure proper I2C configuration and connection to the VL6180X sensor 
 *       before running this program.
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

#include "vl6180x_api.h"

#define I2C_SCL 15
#define I2C_SDA 16

void app_main(void)
{
    i2c_t vl6180x_i2c;

    i2c_init(&vl6180x_i2c, I2C_NUM_1, I2C_SCL, I2C_SDA, 400000, 0x29);

    VL6180xDev_t myDev = vl6180x_i2c;
    VL6180x_RangeData_t Range;

    vTaskDelay(10/portTICK_PERIOD_MS);

    int status = 1, init_status = VL6180x_InitData(myDev);
    ESP_LOGI("VL6180x", "init_status %d", init_status);
    if(init_status == 0 || init_status == CALIBRATION_WARNING ){
        status = VL6180x_Prepare(myDev);
        ESP_LOGI("VL6180x", "Prepare status %d", status);
        if( !status )
            status=init_status; // if prepare is successfull return potential init warning
    }

    while (1) {
        VL6180x_RangePollMeasurement(myDev, &Range);
        if (Range.errorStatus == 0 )
            ESP_LOGI("VL6180x", "Range %ld mm", Range.range_mm); // your code display range in mm
        else
            ESP_LOGE("VL6180x", "Range error %lu", Range.errorStatus);
        vTaskDelay(1000/portTICK_PERIOD_MS); // your code sleep at least 1 msec
    }
}
