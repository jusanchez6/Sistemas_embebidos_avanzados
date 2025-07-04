
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

#ifndef _WIFI_LIB_H_
#define _WIFI_LIB_H_

#define WIFI_SSID       "er nombre er güifi"        
#define WIFI_PASSWORD   "a contrasseña er güifi"
#define WIFI_MAX_RETRY  5
#define WIFI_PORT       8080


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
 * 
 * 
 * @return esp_err_t Returns `ESP_OK` on success, or an error code on failure.
 */

esp_err_t wifi_init_station(void);

/**
 * @brief Get the IP address of the connected WiFi network
 * 
 * This function retrieves the IP address assigned to the WiFi station interface
 * and logs it to the console.
 * 
 * It uses the ESP-IDF API to get the IP information from the network interface
 * and prints it in a human-readable format.
 */
void get_ip_address(void);

#endif // _WIFI_LIB_H_

