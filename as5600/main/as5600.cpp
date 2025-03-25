#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 18
#define I2C_MASTER_SDA_IO 17
#define I2C_MASTER_NUM I2C_NUM_0
#define I2c_SLAVE_ADDR 0x36



class AS5600
{
public:

    /**
     * @brief Construct a new AS5600 object
     * 
     * @param i2c_num I2C port number
     * @param sda GPIO pin for SDA
     * @param scl GPIO pin for SCL
     */
    AS5600(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl)
        : i2c_num(i2c_num), sda(sda), scl(scl) {}

    /**
     * @brief Initialize the AS5600
     *  
     * @return esp_err_t 
     */
    esp_err_t begin()
        {
        // Initialize the I2C AS5600 driver

        i2c_config_t conf = {};

        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = this->sda;
        conf.scl_io_num = this->scl;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = 100000;
        conf.clk_flags = 0;

        ESP_ERROR_CHECK(i2c_param_config(i2c_num, &conf));
        vTaskDelay(pdMS_TO_TICKS(100));

        ESP_ERROR_CHECK(i2c_driver_install(i2c_num, conf.mode, 0, 0, 0));
        vTaskDelay(pdMS_TO_TICKS(100));

        return ESP_OK;
    }

    /**
     * @brief Read the angle from the AS5600
     * 
     * @param angle Pointer to the angle variable
     * @param raw If true, read the raw angle, otherwise read the processed angle
     * 
     * @return esp_err_t 
     */
    esp_err_t read_angle(int16_t *angle, bool raw = false)
    {

        uint8_t rx_data[2];
        uint8_t reg = raw ? 0x0C : 0x0E;        // 0x0C para angulo sin procesar, 0x0E para angulo procesado

        esp_err_t err = i2c_master_write_read_device(i2c_num, I2c_SLAVE_ADDR, &reg, 1, rx_data, 2, pdMS_TO_TICKS(1000));

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading angle: %d", err);
            return err;
        }

        *angle = ((rx_data[0]& 0x0F)<< 8 | rx_data[1]);
        return ESP_OK;
    }

    /**
     * @brief Detect the magnet
     * 
     * @return true if the magnet is detected, false otherwise
     * 
     * @note The magnet is detected when the bit 4 of the status register is 1
     * 
     */
    bool magnet_detect()
    {

        uint8_t rx_data[1];
        uint8_t reg = 0x0B;
        esp_err_t err = i2c_master_write_read_device(i2c_num, I2c_SLAVE_ADDR, &reg, 1, rx_data, 1, pdMS_TO_TICKS(1000));
        
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error reading the register 0x0B");
            return false;
        }

        return (rx_data[0] & 0x20) == 0x20;

    }

    get_encoder_steps () {
        int16_t angle;

        if (read_angle(&angle) != ESP_OK) {
            ESP_LOGE(TAG, "Error reading angle");
            return total_steps;
        }

        int delta = angle -last_angle;


        // Ajuste para cruce de 0° - 360°
        if (delta > 2048)  // Rotación en sentido antihorario
            delta -= 4096;
        else if (delta < -2048)  // Rotación en sentido horario
            delta += 4096;

        total_steps += delta;
        last_angle = angle;

        return total_steps
    }

private:
    
    i2c_port_t i2c_num;     //!< I2C port number
    gpio_num_t sda;         //!< GPIO pin for SDA
    gpio_num_t scl;         //!< GPIO pin for SCL
    int16_t last_angle;     //!< Last read of angle
    int total_steps         //!< Counter of the total steps

    static constexpr const char *TAG = "AS5600";

};

extern "C"
{
    void app_main(void);
}

void app_main(void)
{

    
}
