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
    apds9960->conf.PON = mode;
    i2c_write_reg(&apds9960->i2c_handle, APDS9960_REG_ENABLE, &apds9960->conf.WORD, 1);

    ESP_LOGI("APDS9960", "Mode set: %d", mode);
}


