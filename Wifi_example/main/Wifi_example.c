
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
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "driver/gpio.h"

#define WIFI_SSID               "Margarita 2"
#define WIFI_PASS               "m43420813s"
#define WIFI_MAXIMUM_RETRY      5
#define PORT                    3333


#define PIN_LED                 48

static const char *TAG = "wifi station";


/**
 * @brief Initialize WiFi in station mode
 * 
 * This function initializes the WiFi in station mode, sets the SSID and password,
 * and starts the WiFi connection process.
 * 
 * It uses the ESP-IDF WiFi API to configure the WiFi settings and connect to the specified network. 
 * It also handles the initialization of the NVS flash storage and the event loop for WiFi events.
 * 
 * @note This function should be called in the `app_main` function to start the WiFi connection process.
 * @note Make sure to replace `WIFI_SSID` and `WIFI_PASS` with your actual WiFi credentials.
 */

void wifi_init_sta(void) {
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
    esp_wifi_start();

    ESP_LOGI(TAG, "Conecting to %s...", WIFI_SSID);
    esp_wifi_connect();

}


/**
 * @brief Get the IP address of the connected WiFi network
 * 
 * This function retrieves the IP address assigned to the WiFi station interface
 * and logs it to the console.
 * 
 * It uses the ESP-IDF API to get the IP information from the network interface
 * and prints it in a human-readable format.
 */
void get_ip_address(void) {
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == NULL) {
        ESP_LOGE(TAG, "Could not find netif for WIFI_STA_DEF");
        return;
    }

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(netif, &ip_info);
    ESP_LOGI(TAG, "\n \nIP: " IPSTR, IP2STR(&ip_info.ip));
}



/**
 * @brief Configure the LED GPIO pin
 * 
 * This function sets up the GPIO pin for the LED as an output and initializes it to a low state (LED off).
 * 
 * It uses the ESP-IDF GPIO API to configure the pin direction and initial level.
 */
void configure_led(void) {
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, 0); 
}



/**
 * @brief Task to handle incoming TCP connections
 * 
 * This function creates a TCP server that listens for incoming connections on a specified port.
 * When a client connects, it receives commands to turn an LED on or off and responds accordingly.
 * 
 * It uses the ESP-IDF socket API to create and manage the TCP server and client connections.
 */
void tcp_server_task(void *pvParameters) {
    char rx_buffer[128];
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    listen(listen_sock, 1);
    ESP_LOGI(TAG, "TCP server listening at: %d", PORT);

    while (1) {
        struct sockaddr_in client_addr;
        uint addr_len = sizeof(client_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        ESP_LOGI(TAG, "Client connected");

        while (1) {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len <= 0) break;

            rx_buffer[len] = 0;
            ESP_LOGI(TAG, "Recived: %s", rx_buffer);

            // Procesar el comando recibido
            
            if (strcmp(rx_buffer, "LED_ON") == 0) {
                gpio_set_level(PIN_LED, 1);
                send(sock, "LED is ON\n", 11, 0);


            } else if (strcmp(rx_buffer, "LED_OFF") == 0) {
                gpio_set_level(PIN_LED, 0);
                send(sock, "LED is OFF\n", 12, 0);

            } else {
                ESP_LOGI(TAG, "Command not recognized: %s", rx_buffer);
                send(sock, "ERROR: Command not recognized\n", 30, 0);
            }

        }

        close(sock);
        ESP_LOGI(TAG, "Client disconnected");
    }
}



void app_main(void) {
   nvs_flash_init();
    wifi_init_sta();
    configure_led();

    vTaskDelay(3000 / portTICK_PERIOD_MS); // Espera un segundo para que la conexión se establezca
    get_ip_address();

    xTaskCreate(tcp_server_task, "tcp_server_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "TCP server task created");

}