
#include <errno.h>
#include <string.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_err.h"

#include "driver/gpio.h"

#define casa 1

#if casa
    #define WIFI_SSID "iPhone de Julián "
    #define WIFI_PASS "sanchez06"
#else
    #define WIFI_SSID "Howlers - UdeA"
    #define WIFI_PASS "9876543210"

#endif

#define WIFI_MAXIMUM_RETRY 5
#define PORT 3333

#define PIN_LED 48

#define CHANGE_PID_FLAG 0x01
#define CHANGE_DUTY_FLAG 0x02
#define CHANGE_SET_POINT_FLAG 0x04
#define MOVE_RIGHT_FLAG 0x08
#define MOVE_LEFT_FLAG 0x10

static const char *TAG = "wifi station";

void dev_wifi_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_err_t ret = esp_wifi_start();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start Wi-Fi: %s", esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(TAG, "Wi-Fi started successfully");
        esp_wifi_connect();
    }
}

void get_ip_address(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == NULL)
    {
        ESP_LOGE(TAG, "Failed to get netif handle");
        return;
    }

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(netif, &ip_info);
    ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&ip_info.ip));
}

void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)};

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP server listening on port %d", PORT);

    while (1)
    {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len < 0)
        {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            continue;
        }

        rx_buffer[len] = 0;
        ESP_LOGI(TAG, "Received: %s", rx_buffer);

        if (strncmp(rx_buffer, "L ", 2) == 0)
        {
            char direction[16];
            float degrees, velocity, distance;
            sscanf(rx_buffer + 2, "%s %f %f %f", direction, &degrees, &velocity, &distance);
            // lógica de movimiento lineal
        }
        else if (strncmp(rx_buffer, "C ", 2) == 0)
        {
            char direction[16];
            float degrees, velocity, radius;
            sscanf(rx_buffer + 2, "%s %f %f %f", direction, &degrees, &velocity, &radius);
            // lógica de movimiento circular
        }
        else if (strncmp(rx_buffer, "R ", 2) == 0)
        {
            char direction[16];
            float degrees, velocity;
            sscanf(rx_buffer + 2, "%s %f %f", direction, &degrees, &velocity);
            // lógica de rotación sobre sí mismo
        }
    }

    close(sock);

}


void print_ip_task(void *pvParameters) {
    while (1) {
        esp_netif_ip_info_t ip;
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
            ESP_LOGI("IP_MON", "Mi IP: " IPSTR, IP2STR(&ip.ip));
        } else {
            ESP_LOGW("IP_MON", "No tengo IP asignada");
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    dev_wifi_init();

    get_ip_address();

    xTaskCreate(&udp_server_task, "udp_server_task", 4096, NULL, 5, NULL);
    xTaskCreate(print_ip_task, "print_ip_task", 4096, NULL, 5, NULL);

}
