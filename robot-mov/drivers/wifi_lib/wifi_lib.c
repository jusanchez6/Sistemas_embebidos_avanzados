#include "wifi_lib.h"

esp_err_t esp_err_t dev_wifi_init(void) {
    
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_LOGE("WIFI_INIT", "NVS Flash initialization: %s", esp_err_to_name(ret));
    
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = AP_PASSWORD,
            .max_connection = MAX_STA_CONN,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK
        },
    };

    IF (strlen(WIFI_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN; // Open network if no password is set
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
    ret = esp_wifi_start();

    if (ret != ESP_OK) {
        ESP_LOGE("WIFI_INIT", "WiFi start failed: %s", esp_err_to_name(ret));
        return ret;
    } else {
        ESP_LOGI("WIFI_INIT", "WiFi AP mode initialized successfully");
        return ret;
    }
}

httpd_handle_t start_server (void) {

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_line_movement = {
            .uri = "/lineMovement",
            .method = HTTP_GET,
            .handler = line_movement_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_line_movement);

        httpd_uri_t uri_circular_movement = {
            .uri = "/circularMovement",
            .method = HTTP_GET,
            .handler = circular_movement_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_circular_movement);

        httpd_uri_t uri_self_rotation = {
            .uri = "/selfRotation",
            .method = HTTP_GET,
            .handler = selfRotation_movement_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_self_rotation);        

        ESP_LOGI(TAG, "Servidor web iniciado en modo AP");
    }

    return server;

}

esp_err_t line_movement_handler(httpd_req_t *req) {
    char param[128];
    ESP_LOGI(TAG, ">> Entró al handler /lineMovement");

    // Configurar encabezados CORS
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    if (httpd_req_get_url_query_str(req, param, sizeof(param)) == ESP_OK) {
        ESP_LOGI(TAG, "Query string: %s", param);

        char direction[16], degrees[8], velocity[8], distance[8];

        if (httpd_query_key_value(param, "direction", direction, sizeof(direction)) == ESP_OK &&
            httpd_query_key_value(param, "degrees", degrees, sizeof(degrees)) == ESP_OK &&
            httpd_query_key_value(param, "velocity", velocity, sizeof(velocity)) == ESP_OK &&
            httpd_query_key_value(param, "distance", distance, sizeof(distance)) == ESP_OK) {

            ESP_LOGI(TAG, ">> Params OK");
            ESP_LOGI(TAG, "Direction: %s", direction);
            ESP_LOGI(TAG, "Degrees: %s", degrees);
            ESP_LOGI(TAG, "Velocity: %s", velocity);
            ESP_LOGI(TAG, "Distance: %s", distance);

            httpd_resp_sendstr(req, "Command received");
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "Error al obtener parámetros");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing params");
            return ESP_FAIL;
        }

    } else {
        ESP_LOGE(TAG, "No se pudo obtener la query string");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No query string");
        return ESP_FAIL;
    }
}

esp_err_t circular_movement_handler(httpd_req_t *req) {
    char param[128];
    ESP_LOGI(TAG, ">> Entró al handler /circularMovement");

    // Configurar encabezados CORS
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    if (httpd_req_get_url_query_str(req, param, sizeof(param)) == ESP_OK) {
        ESP_LOGI(TAG, "Query string: %s", param);

        char direction[16], degrees[8], velocity[8], radius[8];

        if (httpd_query_key_value(param, "direction", direction, sizeof(direction)) == ESP_OK &&
            httpd_query_key_value(param, "degrees", degrees, sizeof(degrees)) == ESP_OK &&
            httpd_query_key_value(param, "velocity", velocity, sizeof(velocity)) == ESP_OK &&
            httpd_query_key_value(param, "distance", radius, sizeof(radius)) == ESP_OK) {

            ESP_LOGI(TAG, ">> Params OK");
            ESP_LOGI(TAG, "Direction: %s", direction);
            ESP_LOGI(TAG, "Degrees: %s", degrees);
            ESP_LOGI(TAG, "Velocity: %s", velocity);
            ESP_LOGI(TAG, "Radius: %s", radius);

            httpd_resp_sendstr(req, "Command received");
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "Error obtaining parameters");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing params");
            return ESP_FAIL;
        }

    } else {
        ESP_LOGE(TAG, "Couldn't get the query string");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No query string");
        return ESP_FAIL;
    }
}

esp_err_t selfRotation_movement_handler(httpd_req_t *req) {
    char param[128];
    ESP_LOGI(TAG, ">> Entró al handler /selfRotation");

    // Configurar encabezados CORS
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    if (httpd_req_get_url_query_str(req, param, sizeof(param)) == ESP_OK) {
        ESP_LOGI(TAG, "Query string: %s", param);

        char direction[16], degrees[8], velocity[8];

        if (httpd_query_key_value(param, "direction", direction, sizeof(direction)) == ESP_OK &&
            httpd_query_key_value(param, "degrees", degrees, sizeof(degrees)) == ESP_OK &&
            httpd_query_key_value(param, "velocity", velocity, sizeof(velocity)) == ESP_OK ) {

            ESP_LOGI(TAG, ">> Params OK");
            ESP_LOGI(TAG, "Direction: %s", direction);
            ESP_LOGI(TAG, "Degrees: %s", degrees);
            ESP_LOGI(TAG, "Velocity: %s", velocity);

            httpd_resp_sendstr(req, "Command received");
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "Error obtaining parameters");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing params");
            return ESP_FAIL;
        }

    } else {
        ESP_LOGE(TAG, "Couldn't get the query string");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No query string");
        return ESP_FAIL;
    }
}


