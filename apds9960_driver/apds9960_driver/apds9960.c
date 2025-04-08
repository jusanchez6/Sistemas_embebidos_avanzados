/**
 * @file apds9960.c
 * 
 * @brief APDS9960 driver implementation
 * 
 * This file contains the implementation of the APDS9960 driver functions. The driver is used to interface with the APDS9960 sensor over I2C.
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


#include "include/apds9960.h"



void APDS9960_init(APDS9960_t *apds9960, i2c_port_t i2c_port, uint8_t sda, uint8_t scl) {

    if (i2c_init(&apds9960->i2c_handle, i2c_port, scl, sda, 400000, APDS9960_I2C_ADDR)) {
        ESP_LOGI("APDS9960", "I2C initialized");
    } else {
        ESP_LOGE("APDS9960", "I2C initialization failed");
    }

    // check the id register
    uint8_t id = 0;
    i2c_read_reg(&apds9960->i2c_handle, APDS9960_REG_ID, &id, 1);

    if (id == APDS9960_ID) {
        ESP_LOGI("APDS9960", "ID register matches");
    } else {
        ESP_LOGE("APDS9960", "ID register does not match");
    }

    // set the default configuration for the sensor (just enable ALS)
    apds9960->conf.WORD = APDS9960_AEN_ENABLE;
    i2c_write_reg(&apds9960->i2c_handle, APDS9960_REG_ENABLE, &apds9960->conf.WORD, 1);

    ESP_LOGE("APDS9960", "Configuration set");

}

void APDS9960_set_mode(APDS9960_t *apds9960, apds9960_mode_t mode) {
    apds9960->conf.WORD = mode;
    i2c_write_reg(&apds9960->i2c_handle, APDS9960_REG_ENABLE, &apds9960->conf.WORD, 1);

}

void APDS9960_get_RGB(APDS9960_t *apds9960, uint16_t *r, uint16_t *g, uint16_t *b, bool scale) {
    uint8_t data[6] = {0};

    // Leer los 6 bytes desde el registro RDATAL (0x96)
    i2c_read_reg(&apds9960->i2c_handle, APDS9960_REG_RDATAL, data, 6);

    *r = (data[1] << 8) | data[0];  // Rojo
    *g = (data[3] << 8) | data[2];  // Verde
    *b = (data[5] << 8) | data[4];  // Azul


    if (scale) {
        // Scale the values to 0-255
        *r = map_func(*r, 0, 1025, 0, 255);
        *g = map_func(*g, 0, 1025, 0, 255);
        *b = map_func(*b, 0, 1025, 0, 255);
    }


    
}

void APDS9960_set_gain(APDS9960_t *apds9960, apds9960_gain_t gain) {
    
    uint8_t data = gain;

    printf("Setting gain to %d\n", data);
    
    // Set the gain register

    i2c_write_reg(&apds9960->i2c_handle, APDS9960_REG_CONTROL, &data, 1);
}

void APDS9960_disable(APDS9960_t *apds9960) {
    apds9960->conf.WORD = APDS9960_MODE_OFF;
    i2c_write_reg(&apds9960->i2c_handle, APDS9960_REG_ENABLE, &apds9960->conf.WORD, 1);
    ESP_LOGI("APDS9960", "Sensor disabled");
}