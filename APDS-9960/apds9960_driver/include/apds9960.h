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
void APDS9960_get_RGB(APDS9960_t *apds9960, uint16_t *r, uint16_t *g, uint16_t *b);



#endif // APDS9960_H