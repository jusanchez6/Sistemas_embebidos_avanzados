/**
 * @file apds9960.h
 * 
 * @brief APDS9960 driver header file
 * 
 * This file contains the definitions and function prototypes for the APDS9960 driver. The driver is used to interface with the APDS9960 sensor, which is a digital RGB color sensor with proximity detection capabilities.
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

#ifndef APDS9960_H
#define APDS9960_H


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


#include "apds9960_defs.h"
#include "platform_i2c_esp32s3.h"



#define APDS9960_I2C_ADDR 0x39          ///< I2C address of the APDS9960
#define I2C_MASTER_FREQ_HZ 400000       ///< I2C master clock frequency 


typedef struct 
{
    apds9960_config_t conf;         ///< Configuration register union
    apds9960_regs_t regs;           ///< Registers

    // Peripheral handles
    i2c_t i2c_handle;                  ///< I2C master driver

}APDS9960_t;


/**
 * @brief Initialize the APDS9960 sensor
 * 
 * @param apds9960 Pointer to the APDS9960 object
 * @param i2c_num I2C port number
 * @param sda GPIO number for SDA
 * @param scl GPIO number for SCL
 */
void APDS9960_init(APDS9960_t *apds9960, i2c_port_t i2c_num, uint8_t sda, uint8_t scl);

/**
 * @brief Set the mode of the APDS9960 sensor
 * 
 * @param apds9960 Pointer to the APDS9960 object
 * @param mode Mode to set
 */
void APDS9960_set_mode(APDS9960_t *apds9960, apds9960_mode_t mode);


/**
 * @brief Get the RGB values from the sensor
 * 
 * @param apds9960 Pointer to the APDS9960 object
 * @param r Pointer to store the red value
 * @param g Pointer to store the green value
 * @param b Pointer to store the blue value
 */
void APDS9960_get_RGB(APDS9960_t *apds9960, uint16_t *r, uint16_t *g, uint16_t *b, bool scale);


/**
 * @brief Set the gain of the sensor
 * 
 * @param apds9960 Pointer to the APDS9960 object
 * @param gain Gain value to set
 */
void APDS9960_set_gain(APDS9960_t *apds9960, apds9960_gain_t gain);


/**
 * @brief disable the sensor
 * 
 * @param apds9960 Pointer to the APDS9960 object
 */
void APDS9960_disable(APDS9960_t *apds9960);



#endif // APDS9960_H