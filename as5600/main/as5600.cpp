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

static const char *TAG = "AS5600";

void print_binary(uint8_t value) {
    printf("0b");
    for (int i = 7; i >= 0; i--) {  // Recorre los 8 bits de mayor a menor
        printf("%d", (value >> i) & 1);
    }
    printf("\n");
}


void scan_i2c_devices(i2c_port_t i2c_num) {
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("Device found at address 0x%02X\n", addr);
        } else {
            printf("No device found at address 0x%02X\n", addr);
        }
    }
}

class AS5600
{
public:
    AS5600(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl)
        : i2c_num(i2c_num), sda(sda), scl(scl) {}

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

        ESP_ERROR_CHECK(i2c_driver_install(i2c_num, conf.mode, 0, 0, ESP_INTR_FLAG_LEVEL1));
        vTaskDelay(pdMS_TO_TICKS(100));

        return ESP_OK;
    }

    esp_err_t read_angle(uint8_t *angle)
    {
        // Read the angle from the AS5600

        uint8_t rx_data[1];
        uint8_t reg = 0x0E;

        esp_err_t err = i2c_master_write_read_device(i2c_num, I2c_SLAVE_ADDR, &reg, 1, rx_data, 1, pdMS_TO_TICKS(1000));

        printf("error lectura del angulo: 0x%04x\n", err);
        *angle = rx_data[0];


        return ESP_OK;
    }

    bool magnet_detect()
    {
        // Detect the magnet

        uint8_t rx_data[1];
        uint8_t reg = 0x0B;

        esp_err_t err = i2c_master_write_read_device(i2c_num, I2c_SLAVE_ADDR, &reg, 1, rx_data, 1, pdMS_TO_TICKS(1000));

        printf("rx_data[0]: 0x%02X\n", rx_data[0]);

        printf("error deteccion del iman: 0x%04x\n", err);
        print_binary(rx_data[0]);
        //buscar en el bit 4 si es 1 detecto un magnet  
        return (rx_data[0] & 0x20) == 0x20;

    }

private:
    i2c_port_t i2c_num;
    gpio_num_t sda;
    gpio_num_t scl;
};

extern "C"
{
    void app_main(void);
}

void app_main(void)
{

    // detectar dispositivos I2C

    // Setup
    AS5600 sensor(I2C_NUM_0, (gpio_num_t)I2C_MASTER_SDA_IO, (gpio_num_t)I2C_MASTER_SCL_IO);
    printf("AS5600\n");


    ESP_ERROR_CHECK(sensor.begin());

    printf("AS5600 inicializado\n");
    uint8_t angle;

    // esperar un momento
    vTaskDelay(pdMS_TO_TICKS(1000));

    scan_i2c_devices(I2C_NUM_0);

    while (true)
    {
        sensor.read_angle(&angle);

        ESP_LOGI(TAG, "Angle: %d", angle);
        
        if (sensor.magnet_detect())
        {
            ESP_LOGI(TAG, "Magnet detected");
        }
        else
        {
            ESP_LOGI(TAG, "Magnet not detected");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
