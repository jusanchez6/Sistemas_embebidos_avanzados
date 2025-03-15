/**
 * @file as5600_driver.c
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
 */


#include <cstdint>
#include "esp_log.h"
#include "as5600_driver.hpp"

#define TAG "AS5600"
#define I2C_FREQ_HZ 400000      // 400kHz


/**
 * @brief Constructor for the AS5600 class
 * 
 * @param i2c_port I2C port to use
 * @param sda_pin GPIO pin for SDA
 * @param scl_pin GPIO pin for SCL
 */
AS5600::AS5600(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin) {
    this->i2c_port = i2c_port;
    this->sda_pin = sda_pin;
    this->scl_pin = scl_pin;
}


/**
 * @brief Initialize the AS5600 driver
 * 
 * @return esp_err_t 
 */
esp_err_t AS5600::begin() {
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.scl_io_num = scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;

    esp_err_t ret = i2c_param_config(i2c_port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configuring I2C parameters");
        return ret;
    }

    ret = i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error installing I2C driver");
        return ret;
    }

    return ESP_OK;  
}


uint16_t AS5600::readRawAngle() {
    uint8_t data[2] = {0};
    if (readRegister(RAW_ANGLE, data) != ESP_OK) {
        ESP_LOGE(TAG, "Error reading raw angle");
        return 0;
    }
    return (data[0] << 8) | data[1];
}


float AS5600::getAngleDegrees() {
    uint16_t rawAngle = readRawAngle();
    return (rawAngle * 360.0f) / 4096.0f; 
}

esp_err_t AS5600::isMagnetPresent() {
    uint8_t data = 0;
    if (readRegister(ANGLE_MAGNET_PRESENT, &data) != ESP_OK) {
        ESP_LOGE(TAG, "Error reading magnet present register");
        return ESP_FAIL;
    }
    return data;
}

bool AS5600::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    return i2c_master_write_to_device(i2c_port, AS5600_I2C_ADDR, data, 2, 1000 / portTICK_PERIOD_MS) == ESP_OK;
}

bool AS5600::readRegister(uint8_t reg, uint8_t* data, size_t len) {
    return i2c_master_write_read_device(i2c_port, AS5600_I2C_ADDR, &reg, 1, data, len, 1000 / portTICK_PERIOD_MS) == ESP_OK;
}
