/**
 * @file platform_esp32s3.c
 * 
 * @brief Implementation of the HAL
 * 
 * This file contains the implementation of the functions that bellow to a HAL speciallized in the use of UART, I2C, ADS and GPIO interfaces
 *  
 * 
 * @authors Julian Sanchez
 *          Angel Graciano
 *          Nelson Parra
 * 
 * 
 * @date 14-05-2025
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


#include "platform_esp32s3.h"


// -------------------------------------------------------------
// ---------------------- UART ---------------------------------
// -------------------------------------------------------------
/**
 * @brief Find UART number for the ESP32-S3 microcontroller
 * 
 * @param gpio_tx 
 * @param gpio_rx 
 * @param gpio_rts 
 * @param gpio_cts 
 * @return uart_port_t 
 */
static uart_port_t find_uart_num(uint8_t gpio_tx, uint8_t gpio_rx, uint8_t gpio_rts, uint8_t gpio_cts)
{
    // UART number for the ESP32-S3 microcontroller
    if (gpio_tx == 43 && gpio_rx == 44)
    {
        return UART_NUM_0;
    }
    else if (gpio_tx == 17 && gpio_rx == 18)
    {
        return UART_NUM_1;
    }

    return UART_NUM_2;
}

int uart_init(uart_t *uart_config, uint32_t baud_rate, uint16_t buffer_size, int8_t gpio_tx, int8_t gpio_rx, int8_t gpio_rts, int8_t gpio_cts)
{
    // Check if the UART gpio pins are valid
    if (gpio_tx == UART_PIN_NO_USE || gpio_rx == UART_PIN_NO_USE)
    {
        ESP_LOGE("UART_INIT", "Invalid UART TX or RX pin");
        return -1;
    }
    

    
    // Find UART number for the ESP32-S3 microcontroller
    uart_port_t uart_num = find_uart_num(gpio_tx, gpio_rx, gpio_rts, gpio_cts);

    // Configure UART parameters
    uart_config->baud_rate = baud_rate;
    uart_config->buffer_size = buffer_size;
    uart_config->uart_num = uart_num;
    uart_config->gpio_tx = gpio_tx;
    uart_config->gpio_rx = gpio_rx;
    uart_config->gpio_rts = gpio_rts;
    uart_config->gpio_cts = gpio_cts;

    // Configure UART structure
    uart_config->uart_config = (uart_config_t){
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = (gpio_rts != UART_PIN_NO_USE && gpio_cts != UART_PIN_NO_USE) 
                                    ? UART_HW_FLOWCTRL_CTS_RTS 
                                    : UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    


    int log = 0;
    // Configure UART parameters and check if UART was initialized successfully
    if (uart_param_config(uart_num, &(uart_config->uart_config)) != ESP_OK)
    {
        ESP_LOGE("UART_INIT", "Failed to configure UART parameters");
        log = -1;
    }

    // Configure UART pins
    if (uart_set_pin(uart_num, gpio_tx, gpio_rx, gpio_rts, gpio_cts) != ESP_OK)
    {
        ESP_LOGE("UART_INIT", "Failed to configure UART pins");
        log = -1;
    }

    // Install UART driver
    if (uart_driver_install(uart_num, buffer_size * 2, 0, 0, NULL, 0) != ESP_OK)
    {
        ESP_LOGE("UART_INIT", "Failed to install UART driver");
        log = -1;
    }

    // Clear UART buffer
    if (uart_flush(uart_num) != ESP_OK)
    {
        ESP_LOGE("UART_INIT", "Failed to flush UART buffer");
        log = -1;
    }

    // ESP_LOGI("UART_INIT", "UART initialized successfully");
    return log;
}

int uart_write(uart_t *uart_config, const uint8_t *data, size_t length)
{
    if (uart_config == NULL || data == NULL || length == 0) {
        ESP_LOGE("UART_WRITE", "Invalid parameters.");
        return -1; // Error
    }

    // Attempt to write the data to the UART
    int bytes_written = uart_write_bytes(uart_config->uart_num, (const char *)data, length);

    // Check for errors
    if (bytes_written < 0) {
        ESP_LOGE("UART_WRITE", "Failed to write data to UART.");
        return -1; // Error
    }

    // ESP_LOGI("UART_WRITE", "Successfully wrote %d bytes to UART.", bytes_written);
    return bytes_written;
}

int uart_read(uart_t *uart_config, uint8_t *buffer, size_t length, int timeout_ms)
{
    if (uart_config == NULL || buffer == NULL || length == 0) {
        ESP_LOGE("UART_READ", "Invalid parameters.");
        return -1; // Error
    }
    
    // Attempt to read data from the UART
    int bytes_read = uart_read_bytes(uart_config->uart_num, buffer, length, timeout_ms / portTICK_PERIOD_MS);

    // Check for errors
    if (bytes_read < 0) {
        ESP_LOGE("UART_READ", "Failed to read data from UART.");
        return -1; // Error
    }

    // ESP_LOGDI"UART_READ", "Successfully read %d bytes from UART.", bytes_read);
    return bytes_read;
}

int uart_clear(uart_t *uart_config)
{
    if (uart_config == NULL) {
        ESP_LOGE("UART_FLUSH", "Invalid parameters.");
        return -1; // Error
    }

    // Attempt to flush the UART buffer
    if (uart_flush(uart_config->uart_num) != ESP_OK) {
        ESP_LOGE("UART_FLUSH", "Failed to flush UART buffer.");
        return -1; // Error
    }
    return 0;
}


// -------------------------------------------------------------
// ---------------------- I2C MASTER ---------------------------
// -------------------------------------------------------------


bool i2c_init(i2c_t *i2c, i2c_port_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda, uint32_t clk_speed_hz, uint16_t addr)
{
    esp_err_t ret = ESP_OK;

    i2c->addr = addr;
    i2c->clk_speed_hz = clk_speed_hz;
    i2c->i2c_num = i2c_num;
    i2c->gpio_scl = gpio_scl;
    i2c->gpio_sda = gpio_sda;

    // ------------- I2C master configuration ------------- //
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_num,
        .scl_io_num = gpio_scl,
        .sda_io_num = gpio_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_GOTO_ON_ERROR(i2c_new_master_bus(&i2c_mst_config, &bus_handle), err, TAG_I2C, "i2c io to channel failed");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = clk_speed_hz,
    };

    static i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    i2c->dev_handle = dev_handle;

    return true;
err:
    return ret == ESP_OK;
}

void i2c_deinit(i2c_t *i2c)
{
    i2c_del_master_bus(i2c->bus_handle);
}

void i2c_read_reg(i2c_t *i2c, uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t write_buffer[] = {reg};
    i2c_master_transmit_receive(i2c->dev_handle, write_buffer, 1, (uint8_t *)data, len, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void i2c_read_reg16bits(i2c_t *i2c, uint16_t reg, uint8_t *data, size_t len) 
{
    uint8_t writebuff[] = {reg >> 8, reg};
    i2c_master_transmit_receive(i2c->dev_handle, writebuff, 2, (uint8_t *)data, len, I2C_TIMEOUT_MS/ portTICK_PERIOD_MS);

}


void i2c_write_reg(i2c_t *i2c, uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t write_buffer[len + 1];
    write_buffer[0] = reg;
    memcpy(&write_buffer[1], data, len);
    i2c_master_transmit(i2c->dev_handle, write_buffer, len + 1, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void i2c_write_reg16bits(i2c_t *i2c, uint16_t reg, uint8_t *data, size_t len)
{
    uint8_t writebuff [len+2];
    writebuff[0] = reg >> 8;
    writebuff[1] = reg;
    memcpy(&writebuff[2], data, len);
    i2c_master_transmit(i2c->dev_handle, writebuff, len+2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void i2c_write(i2c_t *i2c, uint8_t *data, size_t len)
{
    i2c_master_transmit(i2c->dev_handle, data, len, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}



// -------------------------------------------------------------
// ---------------------- ADC ---------------------------------
// -------------------------------------------------------------

bool adc_init(adc_t *adc, uint8_t gpio_out)
{
    esp_err_t ret = ESP_OK;

    adc->gpio_out = gpio_out;

    // From GPIO to ADC channel
    adc->unit = ADC_CONF_UNIT;
    ESP_GOTO_ON_ERROR(adc_oneshot_io_to_channel(adc->gpio_out, &adc->unit, &adc->chan), err, TAG_ADC, "adc io to channel failed");
    ESP_LOGI(TAG_ADC, "ADC channel: %d", adc->chan);

    // ------------- ADC pin OUT configuration ------------- //
    // The DIG ADC2 controller of ESP32-S3 doesn’t work properly (pag. 1444).
    // So we need to use the ADC1 controller (GPIO-1 to GPIO-10, pag. 318).

    // ADC Init
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_CONF_UNIT,
    };
    ESP_GOTO_ON_ERROR(adc_oneshot_new_unit(&init_config1, &adc_handle), err, TAG_ADC, "adc init failed");

    // ADC Config
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BIT_WIDTH,
    };
    ESP_GOTO_ON_ERROR(adc_oneshot_config_channel(adc_handle, adc->chan, &config), err, TAG_ADC, "adc config failed");

    // ADC calibration 
    adc_cali_handle_t cali_handle = NULL;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_CONF_UNIT,
        .chan = adc->chan,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BIT_WIDTH,
    };
    adc->is_calibrated = false;
    ESP_GOTO_ON_ERROR(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle), err, TAG_ADC, "adc calibration failed");
    
    adc->is_calibrated = true;
    adc->adc_cali_handle = cali_handle;
    adc->adc_handle = adc_handle;

    return true;

err:
    return false;
}

bool adc_deinit(adc_t *adc)
{
    esp_err_t ret = ESP_OK;
    ret = adc_oneshot_del_unit(adc->adc_handle);
    ESP_GOTO_ON_ERROR(ret, err, TAG_ADC, "adc oneshot del unit failed");
    ESP_GOTO_ON_ERROR(adc_cali_delete_scheme_curve_fitting(adc->adc_cali_handle), err, TAG_ADC, "adc cali delete scheme curve fitting failed");

    return true;
err:
    return false;
}

bool adc_create_unit(adc_oneshot_unit_handle_t *handle, uint8_t unit_id)
{    
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = unit_id
    };
    return adc_oneshot_new_unit(&init_config, handle) == ESP_OK;
}

bool adc_config_channel(adc_t *adc, uint8_t gpio, uint8_t unit_id)
{    
    esp_err_t ret = ESP_OK;
    adc_oneshot_unit_handle_t adc_handle = adc->adc_handle;
    adc->gpio_out = gpio;
    adc->unit = unit_id;

    ESP_GOTO_ON_ERROR(adc_oneshot_io_to_channel(adc->gpio_out, &adc->unit, &adc->chan), err, TAG_ADC, "adc io to channel failed");
    ESP_LOGI(TAG_ADC, "ADC channel: %d", adc->chan);

    // ADC Config
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BIT_WIDTH,
    };
    ESP_GOTO_ON_ERROR(adc_oneshot_config_channel(adc_handle, adc->chan, &config), err, TAG_ADC, "adc config failed");

    // ADC calibration 
    adc_cali_handle_t cali_handle = NULL;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit_id,
        .chan = adc->chan,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BIT_WIDTH,
    };
    adc->is_calibrated = false;
    ESP_GOTO_ON_ERROR(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle), err, TAG_ADC, "adc calibration failed");
    
    adc->is_calibrated = true;
    adc->adc_cali_handle = cali_handle;

    return true;

err:
    return false;
}

void adc_read_raw(adc_t *adc, int *raw)
{
    ESP_ERROR_CHECK(adc_oneshot_read(adc->adc_handle, adc->chan, raw));
}

void adc_read_mvolt(adc_t *adc, uint16_t *mvolt)
{
    int cali_result = 0;
    ESP_ERROR_CHECK(adc_oneshot_get_calibrated_result(adc->adc_handle, adc->adc_cali_handle, adc->chan, &cali_result));
    *mvolt = cali_result;
}



// -------------------------------------------------------------
// ---------------------- GPIO ---------------------------------
// -------------------------------------------------------------

bool gpio_init_basic(gpio_t *gpio, uint8_t gpio_num, uint8_t mode, bool pulldown_en, bool pullup_en)
{
    esp_err_t ret = ESP_OK;

    ///< Set GPIO configuration
    gpio->gpio_num = gpio_num;
    gpio->mode = mode;
    gpio->pulldown_en = pulldown_en;
    gpio->pullup_en = pullup_en;
    gpio->intr_type = GPIO_INTR_DISABLE;

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = mode,
        .pin_bit_mask = (1ULL << gpio_num),
        .pull_down_en = pulldown_en,
        .pull_up_en = pullup_en,
    };
    ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG_GPIO, "gpio config failed");
    ESP_LOGI(TAG_GPIO, "GPIO %d configured", gpio_num);

    return true;
err:
    return false;
}

bool gpio_deinit(gpio_t *gpio)
{
    esp_err_t ret = ESP_OK;
    ret = gpio_reset_pin(gpio->gpio_num);
    ESP_GOTO_ON_ERROR(ret, err, TAG_GPIO, "gpio reset pin failed");
    ESP_LOGI(TAG_GPIO, "GPIO %d deinitialized", gpio->gpio_num);

    return true;
err:
    return false;
}

void gpio_set_high(gpio_t *gpio)
{
    gpio_set_level(gpio->gpio_num, 1);
}

void gpio_set_low(gpio_t *gpio)
{
    gpio_set_level(gpio->gpio_num, 0);
}


// -------------------------------------------------------------
// ---------------------- WIFI ---------------------------------
// -------------------------------------------------------------

esp_err_t wifi_init_station(void) {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&cfg);

    if (ret != ESP_OK) {
        ESP_LOGE("WIFI_INIT", "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    // Set WiFi configuration
    if (esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) != ESP_OK) {
        ESP_LOGE("WIFI_INIT", "Failed to set WiFi configuration.");
        return ESP_FAIL;
    }

    // Start WiFi
    
    if (esp_wifi_start() != ESP_OK) {
        ESP_LOGE("WIFI_INIT", "Failed to start WiFi.");
        return ESP_FAIL;
    }

    ESP_LOGI("WIFI_INIT", "WiFi station initialized successfully");

    // Connect to WiFi
    if (esp_wifi_connect() != ESP_OK) {
        ESP_LOGE("WIFI_INIT", "Failed to connect to WiFi.");
        return ESP_FAIL;
    }

    return ESP_OK;

}

void get_ip_address(void) {
    
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == NULL) {
        ESP_LOGE("WIFI_INIT", "Could not find netif for WIFI_STA_DEF");
        return;
    }

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(netif, &ip_info);
    ESP_LOGI("WIFI_INIT", "\n \nIP: " IPSTR, IP2STR(&ip_info.ip));

}



