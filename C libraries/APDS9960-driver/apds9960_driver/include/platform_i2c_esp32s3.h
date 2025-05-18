/**
 * \file        platform_esp32s3.h
 * \brief
 * \details     I2C and ADC HAL for ESP32-S3
 * 
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/12/2024
 * \copyright   Unlicensed
 */

 #ifndef __HAL_ESP32S3__
 #define __HAL_ESP32S3__


 
 #include <stdint.h>
 #include <stdbool.h>
 #include <stdlib.h>
 #include <string.h>
 #include "esp_log.h"
 #include "esp_check.h"
 #include "freertos/FreeRTOS.h"
 
 // -------------------------------------------------------------
 // ---------------------- I2C MASTER ---------------------------
 // -------------------------------------------------------------
 
 #include "driver/i2c_master.h"

 #define I2C_TIMEOUT_MS 100
 
 static const char* TAG_I2C = "i2c";
 
 /**
  * @brief I2C master driver structure
  * 
  * 
  */
 typedef struct 
 {
     uint8_t addr; // I2C device address
     uint32_t clk_speed_hz;
 
     i2c_port_t i2c_num; // I2C port number
     uint8_t gpio_scl;
     uint8_t gpio_sda;
 
     // I2C configuration structure for ESP32 - S3
     i2c_master_dev_handle_t dev_handle;
     i2c_master_bus_handle_t bus_handle;
 } i2c_t;
 
 /**
  * @brief Initialize the I2C master driver
  * 
  * @param i2c          Pointer to the I2C master object 
  * @param i2c_num      I2C port number 
  * @param gpio_scl     scl pin 
  * @param gpio_sda     sda pin
  * @param clk_speed_hz clock speed in Hz
  * @param addr         I2C device address
  *  
  * @return true if the I2C is initialized correctly
  * @return false if the I2C initialization failed
  */
 bool i2c_init(i2c_t *i2c, i2c_port_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda, uint32_t clk_speed_hz, uint16_t addr);
 
 /**
  * @brief Deinitialize the I2C master driver
  * 
  * @param i2c        Pointer to the I2C master object
  */
 void i2c_deinit(i2c_t *i2c);
 
 /**
  * @brief Given a register address, read the data from the register.
  * 
  * @param i2c  Pointer to the I2C master object 
  * @param reg  Register address to read
  * @param data Pointer to buffer where data will be stored
  * @param len  Number of bytes to read 
  */
 void i2c_read_reg(i2c_t *i2c, uint8_t reg, uint8_t *data, size_t len);
 
 /**
  * @brief Given a register address, write the data to the register.
  * 
  * @param i2c  Pointer to the I2C master object 
  * @param reg  Register address to write 
  * @param data Pointer to data buffer 
  * @param len  Number of bytes to write 
  */
 void i2c_write_reg(i2c_t *i2c, uint8_t reg, uint8_t *data, size_t len);
 
 /**
  * @brief Write data to the I2C bus
  * 
  * @param i2c  Pointer to the I2C master object
  * @param data Pointer to data buffer to write
  * @param len  Number of bytes to write
  */
 void i2c_write(i2c_t *i2c, uint8_t *data, size_t len);
 
 
 // -------------------------------------------------------------
// ---------------------- MAP ---------------------------
// -------------------------------------------------------------


    /**
    * @brief Map a value from one range to another
    * 
    * @param x        Value to map
    * @param in_min   Minimum value of the input range
    * @param in_max   Maximum value of the input range
    * @param out_min  Minimum value of the output range
    * @param out_max  Maximum value of the output range
    * 
    * @return Mapped value
    */
 uint16_t map_func(long x, long in_min, long in_max, long out_min, long out_max);

 
 #endif // __HAL_ESP32__