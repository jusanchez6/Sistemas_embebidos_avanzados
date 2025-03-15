/**
 * @file as5600_driver.hpp
 * 
 * @brief AS5600 Driver
 * 
 * This is a driver for the AS5600 magnetic rotary encoder. It uses the ESP-IDF I2C driver.
 * The AS5600 is a magnetic rotary encoder that outputs an analog voltage proportional to the angle of a magnet.
 * 
 * The driver provides functions to read the raw angle value, get the angle in degrees, and check if a magnet is present.
 * 
 * @author Julian Sanchez 
 * 
 * @version 1.01
 * 
 * @date 14-03-2025
 * 
 * 
 */
#ifndef AS5600_DRIVER_HPP
#define AS5600_DRIVER_HPP

#include <stdint.h>
#include "driver/i2c.h"


#define AS5600_I2C_ADDR 0x36

class AS5600 {
private:
    i2c_config_t conf;
    i2c_port_t i2c_port;
    gpio_num_t sda_pin, scl_pin;

    bool writeRegister(uint8_t reg, uint8_t data);
    bool readRegister(uint8_t reg, uint8_t *data, size_t size);


public:

    AS5600(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin);    
    esp_err_t begin();
    uint16_t readRawAngle();
    float getAngleDegrees();
    esp_err_t isMagnetPresent();
};


/**
 * @addtogroup AS5600_registers
 * @{
 */

/**
 * @brief AS5600 Registers
 * 
 * These are the registers of the AS5600 magnetic rotary encoder.
 * 
 */
enum AS5600Registers {
    RAW_ANGLE = 0x0C,
    ANGLE_MAGNET_PRESENT = 0x0B,
    ANGLE_MAGNET_NOT_PRESENT = 0x0A
};

/**
 * @}
 */

#endif  // AS5600_DRIVER_H