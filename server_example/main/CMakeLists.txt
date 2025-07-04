idf_component_register(SRCS "server_example.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_http_server esp_event esp_wifi nvs_flash driver)