#include "wifi_lib.h"

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

