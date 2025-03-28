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

void APDS9960_get_RGB(APDS9960_t *apds9960, uint16_t *r, uint16_t *g, uint16_t *b) {
    uint8_t data[6] = {0};

    // Leer los 6 bytes desde el registro RDATAL (0x96)
    i2c_read_reg(&apds9960->i2c_handle, APDS9960_REG_RDATAL, data, 6);

    *r = (data[1] << 8) | data[0];  // Rojo
    *g = (data[3] << 8) | data[2];  // Verde
    *b = (data[5] << 8) | data[4];  // Azul
}

void APDS9960_set_ambient_light_interrupt_threshold(APDS9960_t *apds9960, uint16_t high, apds9960_pers_t pers) {
    
    uint8_t high_byte = (high >> 8) & 0xFF;
    uint8_t low_byte = high & 0xFF;

    i2c_write_reg(&apds9960->i2c_handle, APDS9960_REG_AILTL, &low_byte, 1);
    i2c_write_reg(&apds9960->i2c_handle, APDS9960_REG_AILTH, &high_byte, 1);

    // Set the persistence
    uint8_t pers_byte = pers;
    i2c_write_reg(&apds9960->i2c_handle, APDS9960_REG_PERS, &pers_byte, 1);

}


bool APDS9960_read_proximity(APDS9960_t *apds9960, uint8_t *proximity) {
    uint8_t data = 0;
    i2c_read_reg(&apds9960->i2c_handle, APDS9960_REG_PDATA, &data, 1);

    *proximity = data;
    return true;
}