
#include <string.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#ifndef _WIFI_LIB_H_
#define _WIFI_LIB_H_

#define AP_SSID       "er nombre er güifi"        
#define AP_PASSWORD   "a contrasseña er güifi"
#define MAX_STA_CONN  5
#define WIFI_PORT       8080


/**
 * @brief Initialize WiFi in soft Access point mode
 * 
 * This function initializes the WiFi in soft access point mode, sets the SSID and password,
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

esp_err_t dev_wifi_init(void);

/**
 * @brief Start the HTTP server
 * 
 * This function starts the HTTP server and registers the URI handlers for the server.
 * 
 * It uses the ESP-IDF HTTP server API to create and start the server, and registers the URI handlers for the server.
 * 
 * @note This function should be called after initializing the WiFi connection.
 * 
 * @return httpd_handle_t Returns a handle to the HTTP server, or NULL on failure.
 */
httpd_handle_t start_server (void);

/**
 * @brief Handler for line movement requests
 * 
 * This function handles HTTP GET requests for line movement commands.
 * It extracts parameters from the query string, such as direction, degrees, velocity, and distance,
 * and processes the command accordingly.
 * 
 * @param req Pointer to the HTTP request structure
 * 
 * @return esp_err_t Returns `ESP_OK` on success, or an error code on failure.
 */
esp_err_t line_movement_handler(httpd_req_t *req);


/**
 * @brief Handler for circular movement requests
 * 
 * This function handles HTTP GET requests for circular movement commands.
 * It extracts parameters from the query string, such as direction, degrees, velocity, and distance,
 * and processes the command accordingly.
 * 
 * @param req Pointer to the HTTP request structure
 * @return esp_err_t Returns `ESP_OK` on success, or an error code on failure.
 */
esp_err_t circular_movement_handler(httpd_req_t *req);



/**
 * @brief Handler for self rotation movement requests
 * 
 * This function handles HTTP GET requests for self rotation movement commands.
 * It extracts parameters from the query string, such as direction, degrees, velocity, and distance,
 * and processes the command accordingly.
 * 
 * @param req Pointer to the HTTP request structure
 * @return esp_err_t Returns `ESP_OK` on success, or an error code on failure.
 */
esp_err_t selfRotation_movement_handler(httpd_req_t *req);


#endif // _WIFI_LIB_H_

