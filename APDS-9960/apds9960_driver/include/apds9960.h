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



void APDS9960_init(APDS9960_t *apds9960, i2c_port_t i2c_num, uint8_t sda, uint8_t scl);

void APDS9960_set_mode(APDS9960_t *apds9960, apds9960_mode_t mode);

void APDS9960_set_ambient_light_interrupt_threshold(APDS9960_t *apds9960, uint16_t low, uint16_t high);





#endif // APDS9960_H